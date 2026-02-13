// Copyright 2026 RipLab
// Licensed under TODO

#ifndef CLIMBER_BASE__CLIMBER_HARDWARE_INTERFACE_HPP_
#define CLIMBER_BASE__CLIMBER_HARDWARE_INTERFACE_HPP_

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "climber_msgs/msg/arm_state.hpp"
#include "climber_msgs/msg/arm_command.hpp"

namespace climber_base
{

/// Number of arm assemblies (NE, NW, SW, SE)
static constexpr size_t NUM_ARMS = 4;

/// Per-arm data exchanged between ROS topics and the ros2_control loop.
struct ArmData
{
  // --- State (read from MCU via micro-ROS) ---
  double wheel_position{0.0};
  double wheel_velocity{0.0};
  double actuator_position{0.0};
  double actuator_velocity{0.0};
  uint8_t contact_state{0};
  std::vector<float> tof_distances;

  // --- Command (written by ros2_control, sent to MCU) ---
  double wheel_velocity_cmd{0.0};
  double actuator_position_cmd{0.0};
};

class ClimberHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ClimberHardwareInterface)

  // ── Lifecycle transitions ────────────────────────────────────────
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  // ── Interface export ─────────────────────────────────────────────
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // ── Read / Write cycle ──────────────────────────────────────────
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Arm prefixes in joint-order matching the URDF
  static constexpr std::array<const char *, NUM_ARMS> kArmPrefixes = {
    "ne", "nw", "sw", "se"
  };

  // Per-arm runtime data
  std::array<ArmData, NUM_ARMS> arms_;

  // Mutex protects arms_ (written by subscriber threads, read by control loop)
  std::mutex state_mutex_;

  // ROS 2 node for topic I/O (runs inside the hardware interface)
  rclcpp::Node::SharedPtr node_;
  rclcpp::Executor::WeakPtr parent_executor_;

  // Per-arm subscribers & publishers
  std::array<rclcpp::Subscription<climber_msgs::msg::ArmState>::SharedPtr, NUM_ARMS> arm_state_subs_;
  std::array<rclcpp::Publisher<climber_msgs::msg::ArmCommand>::SharedPtr, NUM_ARMS> arm_cmd_pubs_;

  // Communication health tracking
  std::array<rclcpp::Time, NUM_ARMS> last_state_rx_time_;
  double comms_timeout_sec_{0.5};

  // Helper: create subscriptions and publishers for all arms
  void setup_topics();

  // Helper: callback for arm state messages  
  void arm_state_callback(size_t arm_idx,
    const climber_msgs::msg::ArmState::SharedPtr msg);
};

}  // namespace climber_base

#endif  // CLIMBER_BASE__CLIMBER_HARDWARE_INTERFACE_HPP_
