/*
 * Climber MCU Firmware — Skeleton
 *
 * Single firmware for all 4 arm MCUs. ARM_ID selects which arm this
 * instance controls (set via platformio.ini build flag or DIP switch).
 *
 *   - Fast local PID loop for actuator position control using ToF feedback
 *   - Wheel velocity passthrough to motor driver
 *   - Local safety override: emergency grip on comms loss or contact fault
 *   - Publishes ArmState at ~50 Hz to ROS 2 via micro-ROS
 *   - Subscribes to ArmCommand from ROS 2 via micro-ROS
 *
 * Hardware assumptions (adapt to your board):
 *   - Wheel motor: DC motor with encoder, driven via H-bridge PWM
 *   - Linear actuator: DC motor or stepper with limit switches
 *   - ToF sensors: VL53L0X or VL53L1X array on I2C multiplexer
 *   - Encoder: quadrature, read via hardware timer interrupt
 */

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include "RoboClaw.h"

//odrive support
#include "ODriveCAN.h"

// CAN bus baudrate. Make sure this matches for every device on the bus
#define CAN_BAUDRATE 1000000

// ODrive node_id for odrv0
#define ODRV0_NODE_ID 0

#define IS_ESP32_TWAI // ESP32 boards with built-in TWAI (CAN) interface. Directly uses the ESP-IDF TWAI driver.

#include "driver/twai.h"
#include "ODriveESP32TWAI.hpp"

// Pins used to connect to CAN bus transceiver
//d8 = 17
//d9 =18
#define ESP32_TWAI_TX_PIN 18 // these are actually d8  d8 is tx, d9 is rx for north
#define ESP32_TWAI_RX_PIN 17 //d9

ESP32TWAIIntf can_intf;

bool setupCan() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)ESP32_TWAI_TX_PIN,
        (gpio_num_t)ESP32_TWAI_RX_PIN,
        TWAI_MODE_NORMAL
    );

    twai_timing_config_t t_config;
    switch (CAN_BAUDRATE) {
        case 1000000: t_config = TWAI_TIMING_CONFIG_1MBITS(); break;
        case 800000:  t_config = TWAI_TIMING_CONFIG_800KBITS(); break;
        case 500000:  t_config = TWAI_TIMING_CONFIG_500KBITS(); break;
        case 250000:  t_config = TWAI_TIMING_CONFIG_250KBITS(); break;
        case 125000:  t_config = TWAI_TIMING_CONFIG_125KBITS(); break;
        case 100000:  t_config = TWAI_TIMING_CONFIG_100KBITS(); break;
        case 50000:   t_config = TWAI_TIMING_CONFIG_50KBITS(); break;
        case 25000:   t_config = TWAI_TIMING_CONFIG_25KBITS(); break;
        default:      t_config = TWAI_TIMING_CONFIG_250KBITS(); break;
    }

    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        return false;
    }

    if (twai_start() != ESP_OK) {
        twai_driver_uninstall();
        return false;
    }

    return true;
}

// Instantiate ODrive objects
ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID); // Standard CAN message ID
ODriveCAN* odrives[] = {&odrv0}; // Make sure all ODriveCAN instances are accounted for here

struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

// Keep some application-specific user data for every ODrive.
ODriveUserData odrv0_user_data;

// Called every time a Heartbeat message arrives from the ODrive
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_heartbeat = msg;
  odrv_user_data->received_heartbeat = true;
}

// Called every time a feedback message arrives from the ODrive
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_feedback = msg;
  odrv_user_data->received_feedback = true;
}

// Called for every message that arrives on the CAN bus
void onCanMessage(const CanMsg& msg) {
  for (auto odrive: odrives) {
    onReceive(msg, *odrive);
  }
}
// end odrive support


#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// micro-ROS message types (generated from climber_msgs)
#include <climber_msgs/msg/arm_state.h>
#include <climber_msgs/msg/arm_command.h>

// ═══════════════════════════════════════════════════════════════════
//  Configuration
// ═══════════════════════════════════════════════════════════════════

#ifndef ARM_ID
#define ARM_ID 0  // 0=N, 1=W, 2=S, 3=E
#endif

static const char* ARM_NAMES[] = {"n", "w", "s", "e"};
static const char* ARM_NAME = ARM_NAMES[ARM_ID];

// Timing
#define CONTROL_LOOP_HZ   50
#define STATE_PUBLISH_HZ  50
#define TOF_READ_HZ       50
#define COMMS_TIMEOUT_MS  200

// TCA9548A I2C multiplexer
#define TCAADDR 0x70

// RoboClaw actuator controller
#define ROBOCLAW_ADDR 0x80
#define ROBO_BAUDRATE 38400
#define ACTUATOR_PPR 103.8f
#define ACTUATOR_M_PER_REV 0.008f
#define ROBO_MAX_SPEED 10000
#define ROBO_MIN_SPEED 50

// Actuator limits (metres, matching URDF)
#define ACTUATOR_MIN     -0.01f
#define ACTUATOR_MAX      0.15f
#define EMERGENCY_GRIP   -0.008f  // clamp position on fault

// PID gains for actuator position control (tune these!)
#define KP_ACTUATOR  7.17f
#define KI_ACTUATOR  0.09f
#define KD_ACTUATOR  0.00f

/*
output from controller, reusing gains on all roboclaws
PID gains loaded:
  KP: 55.15
  KI: 9.39
  KD: 269.99
  KiMax: 0
  DeadZone: 1
Scaled gains (x0.1):
  KP: 7.17
  KI: 0.09
  KD: 0.00
*/
//helper
static constexpr float PI_F = 3.14159265358979323846f;
static constexpr float TWO_PI_F = 2.0f * PI_F;

// ═══════════════════════════════════════════════════════════════════
//  Pin definitions
// ═══════════════════════════════════════════════════════════════════


// Linear actuator motor
// #define PIN_ACT_PWM       D6
// #define PIN_ACT_DIR       D11
#define PIN_ACT_LIMIT_IN  D5  // limit switch — fully retracted
#define PIN_ACT_LIMIT_OUT D3   // limit switch — fully extended

// RoboClaw UART
#define PIN_ROBO_RX       D7 // maybe swap //on N arm, use D7 as RX and D6 as TX
#define PIN_ROBO_TX       D6 // on N arm D6 goes to S2, same for W
//for east, rx=d7, tx=d6
//for north, rx=d7, tx=d6
//for south, rx=d7, tx = d6
//

// ToF sensor array (I2C)
#define PIN_SDA           A4 //used for imu and the muxer
#define PIN_SCL           A5
#define PIN_TOF_XSHUT_0   A0   // XSHUT for multiplexing individual sensors
#define PIN_TOF_XSHUT_1   A1
#define PIN_TOF_XSHUT_2   A2

#define NUM_TOF_SENSORS   1

// ═══════════════════════════════════════════════════════════════════
//  Global state
// ═══════════════════════════════════════════════════════════════════

// Firmware state machine
enum FirmwareState : uint8_t {
    STATE_INIT = 0,
    STATE_IDLE,
    STATE_NORMAL,
    STATE_EMERGENCY_GRIP,
    STATE_FAULT
};

static volatile FirmwareState fw_state = STATE_INIT;

// Sensor readings
static volatile float tof_distances[NUM_TOF_SENSORS] = {0};
static volatile float actuator_position = 0.0f;   // metres
static volatile float actuator_velocity = 0.0f;    // m/s
static volatile float wheel_position = 0.0f;       // rad (accumulated)
static volatile float wheel_velocity = 0.0f;       // rad/s

static Adafruit_VL53L0X tof_sensors[NUM_TOF_SENSORS];
static bool tof_sensor_ok[NUM_TOF_SENSORS] = {false};

HardwareSerial RoboSerial(1);
RoboClaw roboclaw(&RoboSerial, 50); // Changed timeout from 10000 to 50ms

// RoboClaw PID gains loaded from controller
static float rc_kp = 0.0f;
static float rc_ki = 0.0f;
static float rc_kd = 0.0f;
static uint32_t rc_ki_max = 0;
static uint32_t rc_deadzone = 1;
static uint32_t rc_pos_min = 0;
static uint32_t rc_pos_max = 0;

// Movement controller state
static float actuator_pid_integral = 0.0f;
static int32_t actuator_last_error = 0;
static uint32_t actuator_last_time_ms = 0;
static int32_t actuator_target_ticks = 0;
static int32_t actuator_home_ticks = 0;
static bool actuator_pid_active = false;
static bool actuator_ready = false;
static int32_t current_actuator_ticks = 0; // Cache the ticks per loop

// Commands from ROS 2
static volatile float cmd_actuator_setpoint = 0.0f;
static volatile float cmd_wheel_velocity = 0.0f;
static volatile uint8_t cmd_mode = 0;  // ArmCommand mode

// Timing
static unsigned long last_cmd_rx_ms = 0;
static unsigned long last_state_pub_ms = 0;
static unsigned long last_tof_read_ms = 0;
static unsigned long last_control_us = 0;

// ═══════════════════════════════════════════════════════════════════
//  micro-ROS entities
// ═══════════════════════════════════════════════════════════════════

static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;

static rcl_publisher_t state_pub;
static rcl_subscription_t cmd_sub;

static climber_msgs__msg__ArmState state_msg;
static climber_msgs__msg__ArmCommand cmd_msg;

// ═══════════════════════════════════════════════════════════════════
//  Forward declarations
// ═══════════════════════════════════════════════════════════════════

void setup_micro_ros();
void setup_hardware();
void setup_tof_sensors();
void tca_select(uint8_t channel);
void read_tof_sensors();
void read_wheel_encoder();
void read_actuator_position_sensor();
void run_actuator_pid(float dt);
void initialize_actuator_motion();
void move_actuator_relative_ticks(int32_t delta_ticks);
void update_actuator_pid();
int32_t meters_to_ticks(float metres);
float ticks_to_meters(int32_t ticks);
void set_wheel_motor(float velocity_cmd);
void set_actuator_motor(float pwm);  // -1.0 to 1.0
void publish_state();
void cmd_callback(const void* msg_in);
void check_safety();

// ═══════════════════════════════════════════════════════════════════
//  Encoder ISR (wheel)
// ═══════════════════════════════════════════════════════════════════

static volatile long encoder_count = 0;
static const float ENCODER_TICKS_PER_REV = 1440.0f;  // Adjust to your encoder
static const float WHEEL_RADIUS = 0.06f;              // metres

static inline void blink_setup_led(uint8_t pulses = 1, uint16_t on_ms = 80, uint16_t off_ms = 80) {
    pinMode(LED_BUILTIN, OUTPUT);
    for (uint8_t i = 0; i < pulses; ++i) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(on_ms);
        digitalWrite(LED_BUILTIN, LOW);
        if (i + 1 < pulses) {
            delay(off_ms);
        }
    }
}


// ═══════════════════════════════════════════════════════════════════
//  Setup
// ═══════════════════════════════════════════════════════════════════

void setup() {
    // Serial.begin(115200);
    // delay(7000);
    
    blink_setup_led();
    // blink_setup_led();
    initialize_actuator_motion(); // TEMPORARILY DISABLED: Blocks for 5s without roboclaw
    // blink_setup_led();
    setup_micro_ros();
    setup_hardware();
    // blink_setup_led(2);
      // Register callbacks for the heartbeat and encoder feedback messages
      fw_state = STATE_IDLE;
      last_control_us = micros();
      
      // Topic names: /mcu_{ne,nw,sw,se}/arm_state and /mcu_{ne,nw,sw,se}/arm_cmd
      // (created in setup_micro_ros)
      
    odrv0.onFeedback(onFeedback, &odrv0_user_data);
    odrv0.onStatus(onHeartbeat, &odrv0_user_data);
    if (!setupCan()) {
     // Serial.println("CAN failed to initialize: reset required");
        // blink_setup_led(3, 500, 150);
        // while (true); // spin indefinitely
  }

//   // Serial.println("Waiting for ODrive...");
//   while (!odrv0_user_data.received_heartbeat) {
//     blink_setup_led(1, 100, 100);
//     pumpEvents(can_intf);
// }
  while (odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrv0.clearErrors();
    delay(1);
    blink_setup_led();
    odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    // Serial.print("fail");
    // Pump events for 150ms. This delay is needed for two reasons;
    // 1. If there is an error condition, such as missing DC power, the ODrive might
    //    briefly attempt to enter CLOSED_LOOP_CONTROL state, so we can't rely
    //    on the first heartbeat response, so we want to receive at least two
    //    heartbeats (100 ms default interval).
    // 2. If the bus is congested, the setState command won't get through
    //    immediately but can be delayed.
    for (int i = 0; i < 15; ++i) {
      delay(10);
      pumpEvents(can_intf);
    }
    break;
  }
}

// ═══════════════════════════════════════════════════════════════════
//  Main loop
// ═══════════════════════════════════════════════════════════════════

void loop() {
    unsigned long now_us = micros();
    unsigned long now_ms = millis();
    pumpEvents(can_intf); // This is required on some platforms to handle incoming feedback CAN messages

    // ── Fast control loop (1 kHz) ────────────────────────────────
    float dt = (now_us - last_control_us) / 1e6f;
    if (dt >= (1.0f / CONTROL_LOOP_HZ)) {
        last_control_us = now_us;

        // Read sensors
        read_wheel_encoder();
        read_actuator_position_sensor();

        // Run control based on state
        switch (fw_state) {
            case STATE_NORMAL:
                set_wheel_motor(cmd_wheel_velocity);
                run_actuator_pid(dt); // todo christian's code might change this, update when we get it
                break;

            case STATE_EMERGENCY_GRIP:
                set_wheel_motor(0.0f);  // stop wheels
                cmd_actuator_setpoint = EMERGENCY_GRIP;
                run_actuator_pid(dt); // todo above
                break;

            case STATE_FAULT:
                set_wheel_motor(0.0f);
                set_actuator_motor(0.0f);  // hold position (no PID)
                break;

            case STATE_IDLE:
            default:
                set_wheel_motor(0.0f);
                set_actuator_motor(0.0f);
                break;
        }

        // Safety checks
        //check_safety();
    }

    // ── ToF sensor read (100 Hz) ─────────────────────────────────
    if ((now_ms - last_tof_read_ms) >= (1000 / TOF_READ_HZ)) {
        last_tof_read_ms = now_ms;
        read_tof_sensors();
    }

    // ── Publish state (50 Hz) ────────────────────────────────────
    if ((now_ms - last_state_pub_ms) >= (1000 / STATE_PUBLISH_HZ)) {
        last_state_pub_ms = now_ms;
        publish_state();
    }

    // ── Spin micro-ROS executor (process incoming messages) ──────
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
}

// ═══════════════════════════════════════════════════════════════════
//  micro-ROS setup
// ═══════════════════════════════════════════════════════════════════

void setup_micro_ros() {
    set_microros_serial_transports(Serial);

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);

    // Node name: mcu_n, mcu_w, mcu_s, mcu_e
    char node_name[16];
    snprintf(node_name, sizeof(node_name), "mcu_%s", ARM_NAME);
    rclc_node_init_default(&node, node_name, "", &support);

    // Publisher: /mcu_{arm}/arm_state
    char state_topic[32];
    snprintf(state_topic, sizeof(state_topic), "/mcu_%s/arm_state", ARM_NAME);
    rclc_publisher_init_default(
        &state_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(climber_msgs, msg, ArmState),
        state_topic);

    // Subscriber: /mcu_{arm}/arm_cmd
    char cmd_topic[32];
    snprintf(cmd_topic, sizeof(cmd_topic), "/mcu_%s/arm_cmd", ARM_NAME);
    rclc_subscription_init_default(
        &cmd_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(climber_msgs, msg, ArmCommand),
        cmd_topic);

    // Executor with 1 subscription
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &cmd_callback, ON_NEW_DATA);

    // Allocate ToF distance array in state message
    state_msg.tof_distances.capacity = NUM_TOF_SENSORS;
    state_msg.tof_distances.size = NUM_TOF_SENSORS;
    state_msg.tof_distances.data = (float*)malloc(NUM_TOF_SENSORS * sizeof(float));
}

// ═══════════════════════════════════════════════════════════════════
//  Hardware setup
// ═══════════════════════════════════════════════════════════════════

void setup_hardware() {
    // Wheel motor

    // Actuator motor
    // pinMode(PIN_ACT_PWM, OUTPUT);
    // pinMode(PIN_ACT_DIR, OUTPUT);
    pinMode(PIN_ACT_LIMIT_IN, INPUT_PULLUP);
    pinMode(PIN_ACT_LIMIT_OUT, INPUT_PULLUP);

    // // Boot ToF sensors by setting XSHUT pins HIGH
    // pinMode(PIN_TOF_XSHUT_0, OUTPUT);
    // pinMode(PIN_TOF_XSHUT_1, OUTPUT);
    // pinMode(PIN_TOF_XSHUT_2, OUTPUT);
    // digitalWrite(PIN_TOF_XSHUT_0, HIGH);
    // digitalWrite(PIN_TOF_XSHUT_1, HIGH);
    // digitalWrite(PIN_TOF_XSHUT_2, HIGH);
    delay(10); // Wait for sensors to boot up

    // I2C for ToF sensors
    Wire.begin(PIN_SDA, PIN_SCL);
    setup_tof_sensors();
}

void tca_select(uint8_t channel) {
    if (channel > 7) {
        return;
    }
    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
}

void setup_tof_sensors() {
    for (uint8_t i = 0; i < NUM_TOF_SENSORS; i++) {
        // tca_select(i == 0 ? 0 : i == 1 ? 1 : i == 2 ? 2 : i == 3 ? 6 : 7);
        tca_select(2);
        tof_sensor_ok[i] = tof_sensors[i].begin(0x29, false, &Wire);
    }
}

void initialize_actuator_motion() {
    RoboSerial.begin(ROBO_BAUDRATE, SERIAL_8N1, PIN_ROBO_RX, PIN_ROBO_TX);
    // roboclaw.ReadMainBatteryVoltage(ROBOCLAW_ADDR, &valid);
    bool ok = false;
    // bool ok = roboclaw.ReadM1PositionPID(
    //     ROBOCLAW_ADDR,
    //     rc_kp,
    //     rc_ki,   
    //     rc_kd,
    //     rc_ki_max,
    //     rc_deadzone,
    //     rc_pos_min,
    //     rc_pos_max);



    if (!ok) {
        // Fallback to compile-time gains if controller gains are unavailable.
        rc_kp = KP_ACTUATOR;
        rc_ki = KI_ACTUATOR;
        rc_kd = KD_ACTUATOR;
        rc_ki_max = 5000;
        rc_deadzone = 1;
    } else {
        if (rc_deadzone == 0) {
            rc_deadzone = 1;
        }
        rc_kp = KP_ACTUATOR; // Scale down for position control
        rc_ki = KI_ACTUATOR;
        rc_kd = KD_ACTUATOR;
    }

    // Home to the retract limit, then set zero from first switch release.
    unsigned long start_ms = millis();
    // Serial.println("Homing: Moving DOWN...");
    roboclaw.clear(); // Ensure serial buffer is clean
    roboclaw.BackwardM1(ROBOCLAW_ADDR, 40);
     
    while (digitalRead(PIN_ACT_LIMIT_IN) != LOW && (millis() - start_ms) < 5000) {
        delay(2);
        blink_setup_led();
    }
    roboclaw.clear(); // Ensure serial buffer is clean
    
    // delay(500); // Pause briefly before writing stop
    roboclaw.BackwardM1(ROBOCLAW_ADDR, 0);
    delay(500); // Pause briefly before moving opposite direction
roboclaw.clear(); // Ensure serial buffer is clean
    
    // if (digitalRead(PIN_ACT_LIMIT_IN) == LOW) {    
        roboclaw.ResetEncoders(ROBOCLAW_ADDR);
        start_ms = millis();
        roboclaw.ForwardM1(ROBOCLAW_ADDR, 80); // Full power to break away from switch!
        while (digitalRead(PIN_ACT_LIMIT_IN) == LOW && (millis() - start_ms) < 3000) {
            delay(2);
            // roboclaw.clear(); // Ensure serial buffer is clean
        }
        roboclaw.ForwardM1(ROBOCLAW_ADDR, 0);

        uint8_t status;
        bool valid;
        int32_t enc = roboclaw.ReadEncM1(ROBOCLAW_ADDR, &status, &valid);
        actuator_home_ticks = valid ? enc : 0;
        actuator_ready = true;
    
}


// ═══════════════════════════════════════════════════════════════════
//  Sensor reading stubs (implement for your hardware)
// ═══════════════════════════════════════════════════════════════════

void read_tof_sensors() {
    VL53L0X_RangingMeasurementData_t measure;
    for (uint8_t i = 0; i < NUM_TOF_SENSORS; i++) {
        if (!tof_sensor_ok[i]) {
            tof_distances[i] = -10.0f - (float)i; // Init failed (-10 to -14)
            continue;
        }

        // tca_select(i == 0 ? 0 : i == 1 ? 1 : i == 2 ? 2 : i == 3 ? 6 : 7);

        tca_select(2);
        tof_sensors[i].rangingTest(&measure, false);

        if (measure.RangeStatus != 4) {
            tof_distances[i] = (float)measure.RangeMilliMeter / 1000.0f;
        } else {
            tof_distances[i] = -20.0f - (float)i; // Out of range/error (-20 to -24)
        }
    }
}

void read_wheel_encoder() {
    static long prev_count = 0;
    static unsigned long prev_time_us = 0;
    unsigned long now = micros();

    long count = encoder_count;
    float dt = (now - prev_time_us) / 1e6f;
    if (odrv0_user_data.received_feedback) {
    Get_Encoder_Estimates_msg_t feedback = odrv0_user_data.last_feedback;
    odrv0_user_data.received_feedback = false;
    // Update wheel position and velocity from ODrive feedback
    wheel_position = feedback.Pos_Estimate * TWO_PI_F; // Convert revolutions to radians
    wheel_velocity = feedback.Vel_Estimate * TWO_PI_F; // Convert rev/s to rad/s; 
    }
    // if (dt > 0.0f) {
    //     float delta_rad = ((float)(count - prev_count) / ENCODER_TICKS_PER_REV) * 2.0f * PI;
    //     // wheel_velocity = delta_rad / dt;
    //     // wheel_position += delta_rad;
    // }

    prev_count = count;
    prev_time_us = now;
}

void read_actuator_position_sensor() {
    if (!actuator_ready) {
        actuator_position = 0.0f;
        actuator_velocity = 0.0f;
        return;
    }

    static int32_t prev_ticks = 0;
    static unsigned long prev_us = 0;

    uint8_t status;
    bool valid;
    int32_t enc = roboclaw.ReadEncM1(ROBOCLAW_ADDR, &status, &valid);
    if (!valid) {
        return;
    }
    
    current_actuator_ticks = enc; // Save for PID use

    int32_t rel_ticks = enc - actuator_home_ticks;
    actuator_position = ticks_to_meters(rel_ticks);

    unsigned long now_us = micros();
    if (prev_us != 0) {
        float dt = (now_us - prev_us) / 1e6f;
        if (dt > 0.0f) {
            actuator_velocity = ticks_to_meters(rel_ticks - prev_ticks) / dt;
        }
    }
    prev_ticks = rel_ticks;
    prev_us = now_us;
}

// ═══════════════════════════════════════════════════════════════════
//  Actuator PID controller
// ═══════════════════════════════════════════════════════════════════



//todo: may not be needed due to christians code, update when we get it.
void run_actuator_pid(float dt) {
    (void)dt;
    if (!actuator_ready) {
        set_actuator_motor(0.0f);
        return;
    }

    // Use cached ticks instead of re-reading
    int32_t enc = current_actuator_ticks;

    int32_t desired_ticks = actuator_home_ticks + meters_to_ticks(cmd_actuator_setpoint);
    if (!actuator_pid_active || desired_ticks != actuator_target_ticks) {
        move_actuator_relative_ticks(desired_ticks - enc);
    }

    update_actuator_pid();
}

int32_t meters_to_ticks(float metres) {
    return (int32_t)lroundf((ACTUATOR_PPR / ACTUATOR_M_PER_REV) * metres);
}

float ticks_to_meters(int32_t ticks) {
    return ((float)ticks * ACTUATOR_M_PER_REV) / ACTUATOR_PPR;
}

void move_actuator_relative_ticks(int32_t delta_ticks) {
    int32_t current = current_actuator_ticks; // Use cached ticks

    actuator_target_ticks = current + delta_ticks;
    actuator_pid_integral = 0.0f;
    actuator_last_error = 0;
    actuator_last_time_ms = millis();
    actuator_pid_active = true;
}

void update_actuator_pid() {
    if (!actuator_pid_active) {
        return;
    }

    int32_t current = current_actuator_ticks; // Use cached ticks

    uint32_t now = millis();
    float dt = (now - actuator_last_time_ms) / 1000.0f;
    if (dt < 0.001f) {
        return;
    }
    actuator_last_time_ms = now;

    int32_t error = actuator_target_ticks - current;
    actuator_pid_integral += error * dt;
    actuator_pid_integral = constrain(actuator_pid_integral, -(float)rc_ki_max, (float)rc_ki_max);

    float derivative = (float)(error - actuator_last_error) / dt;
    actuator_last_error = error;

    float output = (rc_kp * error) + (rc_ki * actuator_pid_integral) + (rc_kd * derivative);

    int32_t speed = 0;
    // Introduce a static flag to keep track of if we are currently holding position
    static bool is_holding = false;
    
    // Increased pseudo-deadzone so it doesn't endlessly hunt for 1 encoder tick
    if (abs(error) <= (int32_t)rc_deadzone + 10) {
        speed = 0; // Stop motor when very close to setpoint
        is_holding = true;
        actuator_pid_integral = 0.0f; // Clear integral so it doesn't accumulate
    } else {
        // If we were holding but error gets too large, start moving again
        is_holding = false;
        speed = (int32_t)constrain(output, -(float)ROBO_MAX_SPEED, (float)ROBO_MAX_SPEED);
        // Only enforce MIN_SPEED if we are not supposed to be holding
        if (speed > 0 && speed < ROBO_MIN_SPEED) speed = ROBO_MIN_SPEED;
        if (speed < 0 && speed > -ROBO_MIN_SPEED) speed = -ROBO_MIN_SPEED;
    }

    // Apply limit switches to prevent movement in the direction of the triggered switch
    // Assuming positive speed moves OUT and negative speed moves IN
    if (digitalRead(PIN_ACT_LIMIT_IN) == LOW && speed < 0) {
        // blink_setup_led(3, 50, 50); // Rapid triple blink on limit switch hit
        speed = 0;
        actuator_pid_integral = 0.0f; // Prevent integral windup against limit
    }
    if (digitalRead(PIN_ACT_LIMIT_OUT) == LOW && speed > 0) {
        // blink_setup_led(2, 50, 50); // Rapid triple blink on limit switch hit
        speed = 0;
        actuator_pid_integral = 0.0f; // Prevent integral windup against limit
    }

    roboclaw.SpeedM1(ROBOCLAW_ADDR, speed);
}

// ═══════════════════════════════════════════════════════════════════
//  Motor drivers (implement for your H-bridge)
// ═══════════════════════════════════════════════════════════════════

void set_wheel_motor(float velocity_cmd) {
odrv0.setVelocity(velocity_cmd); // Set velocity for axis 0 

}

void set_actuator_motor(float output) {
    int32_t speed = (int32_t)constrain((int32_t)(output * (float)ROBO_MAX_SPEED),
        -ROBO_MAX_SPEED, ROBO_MAX_SPEED);
    roboclaw.SpeedM1(ROBOCLAW_ADDR, speed);
}

// ═══════════════════════════════════════════════════════════════════
//  Command callback (from ROS 2)
// ═══════════════════════════════════════════════════════════════════

void cmd_callback(const void* msg_in) {
    const climber_msgs__msg__ArmCommand* cmd =
        (const climber_msgs__msg__ArmCommand*)msg_in;

    last_cmd_rx_ms = millis();

    switch (cmd->mode) {
        case 0:  // NORMAL
            cmd_actuator_setpoint = constrain(cmd->actuator_setpoint, ACTUATOR_MIN, ACTUATOR_MAX);
            cmd_wheel_velocity = cmd->wheel_velocity;
            if (fw_state == STATE_IDLE || fw_state == STATE_EMERGENCY_GRIP) {
                fw_state = STATE_NORMAL;
            }
            break;

        case 1:  // EMERGENCY_GRIP
            fw_state = STATE_EMERGENCY_GRIP;
            break;

        case 2:  // RELEASE
            cmd_actuator_setpoint = ACTUATOR_MAX;
            cmd_wheel_velocity = 0.0f;
            fw_state = STATE_NORMAL;
            break;

        case 3:  // CLEAR_FAULT
            if (fw_state == STATE_FAULT) {
                actuator_pid_integral = 0.0f;
                actuator_last_error = 0;
                actuator_pid_active = false;
                fw_state = STATE_IDLE;
            }
            break;
    }
}

// ═══════════════════════════════════════════════════════════════════
//  Safety checks
// ═══════════════════════════════════════════════════════════════════

void check_safety() {
    unsigned long now = millis();

    // Comms timeout: no command received → emergency grip
    if (fw_state == STATE_NORMAL &&
        (now - last_cmd_rx_ms) > COMMS_TIMEOUT_MS) {
        fw_state = STATE_EMERGENCY_GRIP;
    }

    // ToF contact loss detection
    // If minimum ToF distance suddenly jumps (losing contact), emergency grip
    if (fw_state == STATE_NORMAL) {
        float min_tof = tof_distances[0];
        for (int i = 1; i < NUM_TOF_SENSORS; i++) {
            if (tof_distances[i] < min_tof) min_tof = tof_distances[i];
        }
        // If all sensors read > 200mm, we've likely lost the surface
        // (Threshold increased to 0.20f because actuator max release is 0.15m)
        if (min_tof > 0.20f) {
            // Uncomment below to re-enable ToF safety retraction
            // fw_state = STATE_EMERGENCY_GRIP; 
        }
    }
}

// ═══════════════════════════════════════════════════════════════════
//  Publish state to ROS 2
// ═══════════════════════════════════════════════════════════════════

void publish_state() {
    // Fill message
    // state_msg.header.stamp = ... // micro-ROS handles this if agent syncs time
    state_msg.actuator_position = actuator_position;
    state_msg.actuator_velocity = actuator_velocity;
    state_msg.wheel_position = wheel_position;
    state_msg.wheel_velocity = wheel_velocity;

    for (int i = 0; i < NUM_TOF_SENSORS; i++) {
        state_msg.tof_distances.data[i] = tof_distances[i];
    }
    state_msg.tof_distances.size = NUM_TOF_SENSORS;

    // Map firmware state to contact_state enum
    switch (fw_state) {
        case STATE_NORMAL:
            state_msg.contact_state = 1;  // GRIPPING
            break;
        case STATE_IDLE:
            state_msg.contact_state = 2;  // RELEASED
            break;
        case STATE_EMERGENCY_GRIP:
            state_msg.contact_state = 1;  // GRIPPING (emergency)
            break;
        case STATE_FAULT:
            state_msg.contact_state = 3;  // FAULT
            break;
        default:
            state_msg.contact_state = 0;  // UNKNOWN
            break;
    }

    rcl_publish(&state_pub, &state_msg, NULL);
}
