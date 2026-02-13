/*
 * Climber MCU Firmware — Skeleton
 *
 * Single firmware for all 4 arm MCUs. ARM_ID selects which arm this
 * instance controls (set via platformio.ini build flag or DIP switch).
 *
 * Architecture (Option C — Hybrid):
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
#include <micro_ros_arduino.h>

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
#define ARM_ID 0  // 0=NE, 1=NW, 2=SW, 3=SE
#endif

static const char* ARM_NAMES[] = {"ne", "nw", "sw", "se"};
static const char* ARM_NAME = ARM_NAMES[ARM_ID];

// Timing
#define CONTROL_LOOP_HZ   1000
#define STATE_PUBLISH_HZ  50
#define TOF_READ_HZ       100
#define COMMS_TIMEOUT_MS  200

// Actuator limits (metres, matching URDF)
#define ACTUATOR_MIN     -0.01f
#define ACTUATOR_MAX      0.04f
#define EMERGENCY_GRIP   -0.008f  // clamp position on fault

// PID gains for actuator position control (tune these!)
#define KP_ACTUATOR  500.0f
#define KI_ACTUATOR  10.0f
#define KD_ACTUATOR  5.0f

// ═══════════════════════════════════════════════════════════════════
//  Pin definitions (adapt to your hardware)
// ═══════════════════════════════════════════════════════════════════

// Wheel motor (H-bridge)
#define PIN_WHEEL_PWM     25
#define PIN_WHEEL_DIR     26
#define PIN_WHEEL_ENC_A   34
#define PIN_WHEEL_ENC_B   35

// Linear actuator motor
#define PIN_ACT_PWM       27
#define PIN_ACT_DIR       14
#define PIN_ACT_LIMIT_IN  32   // limit switch — fully retracted
#define PIN_ACT_LIMIT_OUT 33   // limit switch — fully extended

// ToF sensor array (I2C)
#define PIN_SDA           21
#define PIN_SCL           22
#define PIN_TOF_XSHUT_0   4   // XSHUT for multiplexing individual sensors
#define PIN_TOF_XSHUT_1   16
#define PIN_TOF_XSHUT_2   17

#define NUM_TOF_SENSORS   3

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

// Commands from ROS 2
static volatile float cmd_actuator_setpoint = 0.0f;
static volatile float cmd_wheel_velocity = 0.0f;
static volatile uint8_t cmd_mode = 0;  // ArmCommand mode

// Timing
static unsigned long last_cmd_rx_ms = 0;
static unsigned long last_state_pub_ms = 0;
static unsigned long last_tof_read_ms = 0;
static unsigned long last_control_us = 0;

// PID state for actuator
static float pid_integral = 0.0f;
static float pid_prev_error = 0.0f;

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
void read_tof_sensors();
void read_wheel_encoder();
void read_actuator_position_sensor();
void run_actuator_pid(float dt);
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

void IRAM_ATTR encoder_isr() {
    if (digitalRead(PIN_WHEEL_ENC_B)) {
        encoder_count++;
    } else {
        encoder_count--;
    }
}

// ═══════════════════════════════════════════════════════════════════
//  Setup
// ═══════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(921600);

    setup_hardware();
    setup_micro_ros();

    fw_state = STATE_IDLE;
    last_control_us = micros();

    // Topic names: /mcu_{ne,nw,sw,se}/arm_state and /mcu_{ne,nw,sw,se}/arm_cmd
    // (created in setup_micro_ros)
}

// ═══════════════════════════════════════════════════════════════════
//  Main loop
// ═══════════════════════════════════════════════════════════════════

void loop() {
    unsigned long now_us = micros();
    unsigned long now_ms = millis();

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
                run_actuator_pid(dt);
                break;

            case STATE_EMERGENCY_GRIP:
                set_wheel_motor(0.0f);  // stop wheels
                cmd_actuator_setpoint = EMERGENCY_GRIP;
                run_actuator_pid(dt);
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
        check_safety();
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
    set_microros_transports();

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);

    // Node name: mcu_ne, mcu_nw, mcu_sw, mcu_se
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
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg,
        &cmd_callback, ON_NEW_DATA);

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
    pinMode(PIN_WHEEL_PWM, OUTPUT);
    pinMode(PIN_WHEEL_DIR, OUTPUT);
    pinMode(PIN_WHEEL_ENC_A, INPUT_PULLUP);
    pinMode(PIN_WHEEL_ENC_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_WHEEL_ENC_A), encoder_isr, RISING);

    // Actuator motor
    pinMode(PIN_ACT_PWM, OUTPUT);
    pinMode(PIN_ACT_DIR, OUTPUT);
    pinMode(PIN_ACT_LIMIT_IN, INPUT_PULLUP);
    pinMode(PIN_ACT_LIMIT_OUT, INPUT_PULLUP);

    // I2C for ToF sensors
    // Wire.begin(PIN_SDA, PIN_SCL);
    // TODO: Initialize VL53L0X/VL53L1X sensors with XSHUT multiplexing
}

// ═══════════════════════════════════════════════════════════════════
//  Sensor reading stubs (implement for your hardware)
// ═══════════════════════════════════════════════════════════════════

void read_tof_sensors() {
    // TODO: Read VL53L0X/VL53L1X array
    // Convert mm readings to metres and store in tof_distances[]
    for (int i = 0; i < NUM_TOF_SENSORS; i++) {
        tof_distances[i] = 0.010f;  // placeholder 10mm
    }
}

void read_wheel_encoder() {
    static long prev_count = 0;
    static unsigned long prev_time_us = 0;
    unsigned long now = micros();

    long count = encoder_count;
    float dt = (now - prev_time_us) / 1e6f;
    if (dt > 0.0f) {
        float delta_rad = ((float)(count - prev_count) / ENCODER_TICKS_PER_REV) * 2.0f * PI;
        wheel_velocity = delta_rad / dt;
        wheel_position += delta_rad;
    }
    prev_count = count;
    prev_time_us = now;
}

void read_actuator_position_sensor() {
    // TODO: Read actuator position from a linear potentiometer, string pot,
    //       or second encoder. For now, use a simple integrator placeholder.
    // actuator_position = analogRead(PIN_ACT_POS) * scale + offset;
}

// ═══════════════════════════════════════════════════════════════════
//  Actuator PID controller
// ═══════════════════════════════════════════════════════════════════

void run_actuator_pid(float dt) {
    float error = cmd_actuator_setpoint - actuator_position;

    pid_integral += error * dt;
    pid_integral = constrain(pid_integral, -0.1f, 0.1f);  // anti-windup

    float derivative = (dt > 0.0f) ? (error - pid_prev_error) / dt : 0.0f;
    pid_prev_error = error;

    float output = KP_ACTUATOR * error + KI_ACTUATOR * pid_integral + KD_ACTUATOR * derivative;
    output = constrain(output, -1.0f, 1.0f);

    // Respect limit switches
    if (output < 0 && digitalRead(PIN_ACT_LIMIT_IN) == LOW) {
        output = 0.0f;  // can't go further in
    }
    if (output > 0 && digitalRead(PIN_ACT_LIMIT_OUT) == LOW) {
        output = 0.0f;  // can't go further out
    }

    set_actuator_motor(output);
}

// ═══════════════════════════════════════════════════════════════════
//  Motor drivers (implement for your H-bridge)
// ═══════════════════════════════════════════════════════════════════

void set_wheel_motor(float velocity_cmd) {
    // TODO: Implement wheel velocity PID or open-loop PWM
    // For now, simple proportional mapping:
    int pwm = constrain((int)(velocity_cmd * 25.5f), -255, 255);
    digitalWrite(PIN_WHEEL_DIR, pwm >= 0 ? HIGH : LOW);
    analogWrite(PIN_WHEEL_PWM, abs(pwm));
}

void set_actuator_motor(float output) {
    // output: -1.0 (retract/grip) to +1.0 (extend/release)
    int pwm = constrain((int)(output * 255.0f), -255, 255);
    digitalWrite(PIN_ACT_DIR, pwm >= 0 ? HIGH : LOW);
    analogWrite(PIN_ACT_PWM, abs(pwm));
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
                pid_integral = 0.0f;
                pid_prev_error = 0.0f;
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
        // If all sensors read > 50mm, we've likely lost the surface
        if (min_tof > 0.05f) {
            fw_state = STATE_EMERGENCY_GRIP;
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
