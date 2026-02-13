// Copyright 2026 RipLab
// Licensed under TODO

#include "climber_base/climber_hardware_interface.hpp"

#include <chrono>
#include <functional>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace climber_base
{

// ═══════════════════════════════════════════════════════════════════
//  Lifecycle: on_init
// ═══════════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn ClimberHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Store the parent executor for later use
  parent_executor_ = params.executor;

  // Read optional hardware parameter for comms timeout
  if (info_.hardware_parameters.count("comms_timeout")) {
    comms_timeout_sec_ = std::stod(info_.hardware_parameters.at("comms_timeout"));
  }

  // Validate expected joint count: 4 wheel + 4 actuator = 8
  if (info_.joints.size() != NUM_ARMS * 2) {
    RCLCPP_FATAL(rclcpp::get_logger("ClimberHardwareInterface"),
      "Expected %zu joints, got %zu", NUM_ARMS * 2, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize arm data
  for (auto & arm : arms_) {
    arm = ArmData{};
  }

  RCLCPP_INFO(rclcpp::get_logger("ClimberHardwareInterface"),
    "Initialized with %zu joints, comms_timeout=%.2fs",
    info_.joints.size(), comms_timeout_sec_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════
//  Lifecycle: on_configure — create ROS node and topics
// ═══════════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn ClimberHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Create an internal ROS 2 node for topic I/O
  node_ = rclcpp::Node::make_shared("climber_hardware_interface");

  // Add our node to the parent executor (provided by controller_manager)
  if (auto executor = parent_executor_.lock()) {
    executor->add_node(node_);
  } else {
    RCLCPP_WARN(node_->get_logger(),
      "Parent executor not available — MCU topics won't be spun automatically");
  }

  setup_topics();

  RCLCPP_INFO(node_->get_logger(), "Configured — topics created");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════
//  Lifecycle: on_activate — send initial zero commands
// ═══════════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn ClimberHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Initialise timestamps
  auto now = node_->get_clock()->now();
  for (size_t i = 0; i < NUM_ARMS; ++i) {
    last_state_rx_time_[i] = now;
    arms_[i].wheel_velocity_cmd = 0.0;
    arms_[i].actuator_position_cmd = 0.0;
  }

  RCLCPP_INFO(node_->get_logger(), "Activated — hardware interface running");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════
//  Lifecycle: on_deactivate — stop all motors
// ═══════════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn ClimberHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Send zero commands to all MCUs
  for (size_t i = 0; i < NUM_ARMS; ++i) {
    auto cmd = climber_msgs::msg::ArmCommand();
    cmd.actuator_setpoint = 0.0f;
    cmd.wheel_velocity = 0.0f;
    cmd.mode = climber_msgs::msg::ArmCommand::RELEASE;
    arm_cmd_pubs_[i]->publish(cmd);
  }

  RCLCPP_INFO(node_->get_logger(), "Deactivated — motors stopped");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════
//  Lifecycle: on_cleanup — tear down node and executor
// ═══════════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn ClimberHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (auto executor = parent_executor_.lock()) {
    executor->remove_node(node_);
  }
  node_.reset();

  RCLCPP_INFO(rclcpp::get_logger("ClimberHardwareInterface"), "Cleaned up");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════
//  export_state_interfaces
// ═══════════════════════════════════════════════════════════════════
std::vector<hardware_interface::StateInterface>
ClimberHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < NUM_ARMS; ++i) {
    const std::string prefix(kArmPrefixes[i]);

    // Wheel joint: position + velocity state
    state_interfaces.emplace_back(
      prefix + "_wheel_joint",
      hardware_interface::HW_IF_POSITION,
      &arms_[i].wheel_position);
    state_interfaces.emplace_back(
      prefix + "_wheel_joint",
      hardware_interface::HW_IF_VELOCITY,
      &arms_[i].wheel_velocity);

    // Actuator joint: position + velocity state
    state_interfaces.emplace_back(
      prefix + "_actuator_joint",
      hardware_interface::HW_IF_POSITION,
      &arms_[i].actuator_position);
    state_interfaces.emplace_back(
      prefix + "_actuator_joint",
      hardware_interface::HW_IF_VELOCITY,
      &arms_[i].actuator_velocity);
  }

  return state_interfaces;
}

// ═══════════════════════════════════════════════════════════════════
//  export_command_interfaces
// ═══════════════════════════════════════════════════════════════════
std::vector<hardware_interface::CommandInterface>
ClimberHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < NUM_ARMS; ++i) {
    const std::string prefix(kArmPrefixes[i]);

    // Wheel joint: velocity command
    command_interfaces.emplace_back(
      prefix + "_wheel_joint",
      hardware_interface::HW_IF_VELOCITY,
      &arms_[i].wheel_velocity_cmd);

    // Actuator joint: position command
    command_interfaces.emplace_back(
      prefix + "_actuator_joint",
      hardware_interface::HW_IF_POSITION,
      &arms_[i].actuator_position_cmd);
  }

  return command_interfaces;
}

// ═══════════════════════════════════════════════════════════════════
//  read — pull latest sensor data from micro-ROS topic callbacks
// ═══════════════════════════════════════════════════════════════════
hardware_interface::return_type ClimberHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Data is updated by the subscriber callbacks in the background executor.
  // The state variables are already pointed to by the exported state interfaces,
  // so controller_manager reads them directly.
  //
  // Check for communication timeouts.
  auto now = node_->get_clock()->now();
  for (size_t i = 0; i < NUM_ARMS; ++i) {
    double dt = (now - last_state_rx_time_[i]).seconds();
    if (dt > comms_timeout_sec_) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "No state from MCU '%s' for %.1fs (timeout=%.1fs)",
        kArmPrefixes[i], dt, comms_timeout_sec_);
    }
  }

  return hardware_interface::return_type::OK;
}

// ═══════════════════════════════════════════════════════════════════
//  write — push commands to MCUs via micro-ROS topics
// ═══════════════════════════════════════════════════════════════════
hardware_interface::return_type ClimberHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < NUM_ARMS; ++i) {
    auto cmd = climber_msgs::msg::ArmCommand();
    cmd.actuator_setpoint = static_cast<float>(arms_[i].actuator_position_cmd);
    cmd.wheel_velocity = static_cast<float>(arms_[i].wheel_velocity_cmd);
    cmd.mode = climber_msgs::msg::ArmCommand::NORMAL;
    arm_cmd_pubs_[i]->publish(cmd);
  }

  return hardware_interface::return_type::OK;
}

// ═══════════════════════════════════════════════════════════════════
//  setup_topics — create publishers and subscribers for each arm
// ═══════════════════════════════════════════════════════════════════
void ClimberHardwareInterface::setup_topics()
{
  for (size_t i = 0; i < NUM_ARMS; ++i) {
    const std::string prefix(kArmPrefixes[i]);

    // Publisher: /mcu_{ne,nw,sw,se}/arm_cmd
    arm_cmd_pubs_[i] = node_->create_publisher<climber_msgs::msg::ArmCommand>(
      "/mcu_" + prefix + "/arm_cmd", 10);

    // Subscriber: /mcu_{ne,nw,sw,se}/arm_state
    arm_state_subs_[i] = node_->create_subscription<climber_msgs::msg::ArmState>(
      "/mcu_" + prefix + "/arm_state", 10,
      [this, i](const climber_msgs::msg::ArmState::SharedPtr msg) {
        this->arm_state_callback(i, msg);
      });

    RCLCPP_INFO(node_->get_logger(),
      "Topics created for arm '%s'", prefix.c_str());
  }
}

// ═══════════════════════════════════════════════════════════════════
//  arm_state_callback — receive state from one MCU
// ═══════════════════════════════════════════════════════════════════
void ClimberHardwareInterface::arm_state_callback(
  size_t arm_idx,
  const climber_msgs::msg::ArmState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  arms_[arm_idx].wheel_position = static_cast<double>(msg->wheel_position);
  arms_[arm_idx].wheel_velocity = static_cast<double>(msg->wheel_velocity);
  arms_[arm_idx].actuator_position = static_cast<double>(msg->actuator_position);
  arms_[arm_idx].actuator_velocity = static_cast<double>(msg->actuator_velocity);
  arms_[arm_idx].contact_state = msg->contact_state;
  arms_[arm_idx].tof_distances = msg->tof_distances;

  last_state_rx_time_[arm_idx] = node_->get_clock()->now();
}

}  // namespace climber_base

// ═══════════════════════════════════════════════════════════════════
//  Register the plugin with pluginlib
// ═══════════════════════════════════════════════════════════════════
PLUGINLIB_EXPORT_CLASS(
  climber_base::ClimberHardwareInterface,
  hardware_interface::SystemInterface)
