#include <cstdio>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>
#include "bxi_can_node.hpp"
#include "stark-sdk.h"
#include <unistd.h>
using namespace std::chrono_literals;

static void print_device_info(DeviceHandler *handle, uint8_t slave_id)
{
    CDeviceInfo *info = stark_get_device_info(handle, slave_id);
    if (!info) {
        printf("[WARN] Failed to get device info\n");
        return;
    }

    printf("Device Info:\n");
    printf("  Hardware Type: %d\n", info->hardware_type);
    printf("  Serial Number: %s\n",
           info->serial_number ? info->serial_number : "");
    printf("  Firmware Version: %s\n",
           info->firmware_version ? info->firmware_version : "");
    free_device_info(info);
}

//=============================================================================
// Demo Functions
//=============================================================================

/**
 * Demo 1: Basic position control (Revo1 & Revo2)
 */
void demo_basic_position(DeviceHandler *handle, uint8_t slave_id)
{
    printf("\n=== Demo 1: Basic Position Control ===\n");

    useconds_t delay = 1000 * 1000; // 1000ms

    // Fist gesture
    printf("[Demo] Performing fist gesture...\n");
    uint16_t positions_fist[] = { 500, 500, 1000, 1000, 1000, 1000 };
    stark_set_finger_positions(handle, slave_id, positions_fist, 6);
    usleep(delay);

    // Open hand
    printf("[Demo] Performing open hand...\n");
    uint16_t positions_open[] = { 0, 0, 0, 0, 0, 0 };
    stark_set_finger_positions(handle, slave_id, positions_open, 6);
    usleep(delay);

    // Single finger control
    printf("[Demo] Moving single finger (middle)...\n");
    stark_set_finger_position(handle, slave_id, STARK_FINGER_ID_MIDDLE, 800);
    usleep(delay);
    stark_set_finger_position(handle, slave_id, STARK_FINGER_ID_MIDDLE, 0);
    usleep(delay);

    // Get motor status
    printf("[Demo] Reading motor status...\n");
    CMotorStatusData *status = stark_get_motor_status(handle, slave_id);
    if (status != NULL) {
        printf("  Positions: %hu, %hu, %hu, %hu, %hu, %hu\n",
               status->positions[0], status->positions[1], status->positions[2],
               status->positions[3], status->positions[4],
               status->positions[5]);
        printf("  Speeds: %hd, %hd, %hd, %hd, %hd, %hd\n", status->speeds[0],
               status->speeds[1], status->speeds[2], status->speeds[3],
               status->speeds[4], status->speeds[5]);
        printf("  Currents: %hd, %hd, %hd, %hd, %hd, %hd\n",
               status->currents[0], status->currents[1], status->currents[2],
               status->currents[3], status->currents[4], status->currents[5]);
        printf("  States: %hhu, %hhu, %hhu, %hhu, %hhu, %hhu\n",
               status->states[0], status->states[1], status->states[2],
               status->states[3], status->states[4], status->states[5]);
        free_motor_status_data(status);
    }
}

/**
 * Demo 2: Speed and Current control (Revo2 primarily, partial Revo1)
 */
void demo_speed_current(DeviceHandler *handle, uint8_t slave_id,
                        bool uses_revo2_api)
{
    printf("\n=== Demo 2: Speed & Current Control ===\n");

    useconds_t delay = 1000 * 1000; // 1000ms

    // Speed control - single finger
    printf("[Demo] Speed control - single finger (middle)...\n");
    stark_set_finger_speed(handle, slave_id, STARK_FINGER_ID_MIDDLE,
                           500); // Close
    usleep(delay);
    stark_set_finger_speed(handle, slave_id, STARK_FINGER_ID_MIDDLE,
                           -500); // Open
    usleep(delay);
    stark_set_finger_speed(handle, slave_id, STARK_FINGER_ID_MIDDLE, 0); // Stop

    // Speed control - multiple fingers
    printf("[Demo] Speed control - all fingers...\n");
    int16_t speeds_close[] = { 100, 100, 500, 500, 500, 500 };
    stark_set_finger_speeds(handle, slave_id, speeds_close, 6);
    usleep(delay);
    int16_t speeds_open[] = { -100, -100, -500, -500, -500, -500 };
    stark_set_finger_speeds(handle, slave_id, speeds_open, 6);
    usleep(delay);
    int16_t speeds_stop[] = { 0, 0, 0, 0, 0, 0 };
    stark_set_finger_speeds(handle, slave_id, speeds_stop, 6);

    // Current control - single finger
    printf("[Demo] Current control - single finger...\n");
    stark_set_finger_current(handle, slave_id, STARK_FINGER_ID_MIDDLE,
                             300); // Close
    usleep(delay);
    stark_set_finger_current(handle, slave_id, STARK_FINGER_ID_MIDDLE,
                             -300); // Open
    usleep(delay);

    // Current control - multiple fingers
    printf("[Demo] Current control - all fingers...\n");
    int16_t currents_close[] = { 200, 200, 300, 300, 300, 300 };
    stark_set_finger_currents(handle, slave_id, currents_close, 6);
    usleep(delay);
    int16_t currents_open[] = { -200, -200, -300, -300, -300, -300 };
    stark_set_finger_currents(handle, slave_id, currents_open, 6);
    usleep(delay);

    // PWM control (Revo2 only)
    if (uses_revo2_api) {
        printf("[Demo] PWM control (Revo2 only)...\n");
        stark_set_finger_pwm(handle, slave_id, STARK_FINGER_ID_MIDDLE, 700);
        usleep(delay);
        stark_set_finger_pwm(handle, slave_id, STARK_FINGER_ID_MIDDLE, -700);
        usleep(delay);

        int16_t pwms[] = { 100, 100, 700, 700, 700, 700 };
        stark_set_finger_pwms(handle, slave_id, pwms, 6);
        usleep(delay);
    }

    // Reset to open
    uint16_t positions_open[] = { 0, 0, 0, 0, 0, 0 };
    stark_set_finger_positions(handle, slave_id, positions_open, 6);
    usleep(delay);
}

/**
 * Demo 3: Advanced control (Revo2 only)
 * - Position + time
 * - Position + speed
 * - Finger parameter configuration
 * - Unit mode
 */
void demo_advanced_revo2(DeviceHandler *handle, uint8_t slave_id)
{
    printf("\n=== Demo 3: Advanced Control (Revo2 Only) ===\n");

    useconds_t delay = 1500 * 1000; // 1500ms

    // Unit mode
    printf("[Demo] Setting unit mode to Normalized...\n");
    stark_set_finger_unit_mode(handle, slave_id, FINGER_UNIT_MODE_NORMALIZED);
    FingerUnitMode mode = stark_get_finger_unit_mode(handle, slave_id);
    printf("  Current mode: %s\n",
           mode == FINGER_UNIT_MODE_NORMALIZED ? "Normalized" : "Physical");

    // Position + time (single finger)
    printf("[Demo] Position + time control (single finger)...\n");
    stark_set_finger_position_with_millis(handle, slave_id,
                                          STARK_FINGER_ID_MIDDLE, 1000, 1000);
    usleep(delay);

    stark_set_finger_position_with_millis(handle, slave_id,
                                          STARK_FINGER_ID_MIDDLE, 0, 1000);
    usleep(delay);

    // Position + speed (single finger)
    printf("[Demo] Position + speed control (single finger)...\n");
    stark_set_finger_position_with_speed(handle, slave_id,
                                         STARK_FINGER_ID_MIDDLE, 1000, 50);
    usleep(delay);

    stark_set_finger_position_with_speed(handle, slave_id,
                                         STARK_FINGER_ID_MIDDLE, 0, 50);
    usleep(delay);

    // Position + duration (multiple fingers)
    // Supported protocols: Modbus, CAN 2.0, CAN FD
    printf("[Demo] Position + duration control (all fingers)...\n");
    uint16_t positions[] = { 300, 300, 500, 500, 500, 500 };
    uint16_t durations[] = { 500, 500, 500, 500, 500, 500 };
    stark_set_finger_positions_and_durations(handle, slave_id, positions,
                                             durations, 6);
    usleep(delay);

    // Position + speed (multiple fingers)
    // Supported protocols: Modbus, CAN FD only (NOT CAN 2.0)
    // Note: CAN 2.0 only supports position+duration due to frame size
    // limitations
    printf("[Demo] Position + speed control (all fingers)...\n");
    uint16_t positions2[] = { 100, 100, 800, 800, 800, 800 };
    uint16_t speeds[] = { 300, 300, 300, 300, 300, 300 };
    stark_set_finger_positions_and_speeds(handle, slave_id, positions2, speeds,
                                          6);
    usleep(delay);

    // Read finger parameters
    printf("[Demo] Reading finger parameters...\n");
    StarkFingerId finger = STARK_FINGER_ID_MIDDLE;
    uint16_t max_pos = stark_get_finger_max_position(handle, slave_id, finger);
    uint16_t min_pos = stark_get_finger_min_position(handle, slave_id, finger);
    uint16_t max_speed = stark_get_finger_max_speed(handle, slave_id, finger);
    uint16_t max_current =
        stark_get_finger_max_current(handle, slave_id, finger);
    uint16_t prot_current =
        stark_get_finger_protected_current(handle, slave_id, finger);
    printf("  Middle finger params:\n");
    printf("    Max position: %hu\n", max_pos);
    printf("    Min position: %hu\n", min_pos);
    printf("    Max speed: %hu\n", max_speed);
    printf("    Max current: %hu\n", max_current);
    printf("    Protected current: %hu\n", prot_current);

    // Reset to open
    uint16_t positions_open[] = { 0, 0, 0, 0, 0, 0 };
    stark_set_finger_positions(handle, slave_id, positions_open, 6);
    usleep(delay);
}

/**
 * Demo 4: Action sequences
 */
void demo_action_sequences(DeviceHandler *handle, uint8_t slave_id)
{
    printf("\n=== Demo 4: Action Sequences ===\n");

    useconds_t delay = 1500 * 1000; // 1500ms

    printf("[Demo] Running built-in gestures...\n");

    printf("  Open hand...\n");
    stark_run_action_sequence(handle, slave_id,
                              ACTION_SEQUENCE_ID_DEFAULT_GESTURE_OPEN);
    usleep(delay);

    printf("  Fist...\n");
    stark_run_action_sequence(handle, slave_id,
                              ACTION_SEQUENCE_ID_DEFAULT_GESTURE_FIST);
    usleep(delay);

    printf("  Two-finger pinch...\n");
    stark_run_action_sequence(handle, slave_id,
                              ACTION_SEQUENCE_ID_DEFAULT_GESTURE_PINCH_TWO);
    usleep(delay);

    printf("  Three-finger pinch...\n");
    stark_run_action_sequence(handle, slave_id,
                              ACTION_SEQUENCE_ID_DEFAULT_GESTURE_PINCH_THREE);
    usleep(delay);

    printf("  Side pinch...\n");
    stark_run_action_sequence(handle, slave_id,
                              ACTION_SEQUENCE_ID_DEFAULT_GESTURE_PINCH_SIDE);
    usleep(delay);

    printf("  Point...\n");
    stark_run_action_sequence(handle, slave_id,
                              ACTION_SEQUENCE_ID_DEFAULT_GESTURE_POINT);
    usleep(delay);

    printf("  Open hand (reset)...\n");
    stark_run_action_sequence(handle, slave_id,
                              ACTION_SEQUENCE_ID_DEFAULT_GESTURE_OPEN);
    usleep(delay);
}

/**
 * Demo 5: Device info and configuration
 * Reads various device configuration parameters
 */
void demo_device_info(DeviceHandler *handle, uint8_t slave_id,
                      bool uses_revo2_api)
{
    printf("\n=== Demo 5: Device Info & Configuration ===\n");

    // Communication settings
    printf("\n[Config] Communication:\n");
    uint32_t rs485_baud = stark_get_rs485_baudrate(handle, slave_id);
    printf("  RS485 Baudrate: %u\n", rs485_baud);

    if (uses_revo2_api) {
        uint32_t canfd_baud = stark_get_canfd_baudrate(handle, slave_id);
        printf("  CANFD Baudrate: %u\n", canfd_baud);
    }

    // System settings
    printf("\n[Config] System:\n");
    bool turbo = stark_get_turbo_mode_enabled(handle, slave_id);
    printf("  Turbo mode: %s\n", turbo ? "enabled" : "disabled");

    bool auto_cal = stark_get_auto_calibration(handle, slave_id);
    printf("  Auto calibration: %s\n", auto_cal ? "enabled" : "disabled");

    if (uses_revo2_api) {
        FingerUnitMode unit_mode = stark_get_finger_unit_mode(handle, slave_id);
        printf("  Unit mode: %s\n", unit_mode == FINGER_UNIT_MODE_NORMALIZED ?
                                        "Normalized" :
                                        "Physical");

        // Motor parameters (read from middle finger as example)
        printf("\n[Config] Motor Parameters (Middle finger):\n");
        StarkFingerId finger = STARK_FINGER_ID_MIDDLE;

        uint16_t max_pos =
            stark_get_finger_max_position(handle, slave_id, finger);
        uint16_t min_pos =
            stark_get_finger_min_position(handle, slave_id, finger);
        printf("  Position range: %hu - %hu\n", min_pos, max_pos);

        uint16_t max_speed =
            stark_get_finger_max_speed(handle, slave_id, finger);
        printf("  Max speed: %hu\n", max_speed);

        uint16_t max_current =
            stark_get_finger_max_current(handle, slave_id, finger);
        uint16_t prot_current =
            stark_get_finger_protected_current(handle, slave_id, finger);
        printf("  Max current: %hu, Protected: %hu\n", max_current,
               prot_current);
    }
}
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    BxiDeviceContext left_ctx_;
    BxiDeviceContext right_ctx_;
    left_ctx_.hw_type_override = STARK_HARDWARE_TYPE_REVO2_BASIC;
    right_ctx_.hw_type_override = STARK_HARDWARE_TYPE_REVO2_BASIC;
    left_ctx_.master_id = 1;
    right_ctx_.master_id = 1;
    if (!init_bxipci_device(&left_ctx_, 5, 126, true)) {
        rclcpp::shutdown();
        return 1;
    }
    if (!init_bxipci_device(&right_ctx_, 6, 127, true)) {
        rclcpp::shutdown();
        return 1;
    }
    sleep(1); //等待Bridge启动
    //测试左手
    print_device_info(left_ctx_.handle,left_ctx_.slave_id);
    demo_basic_position(left_ctx_.handle, left_ctx_.slave_id);
    bool uses_revo2_api =
        !stark_uses_revo1_motor_api(left_ctx_.hw_type_override);
    demo_speed_current(left_ctx_.handle, left_ctx_.slave_id, uses_revo2_api);
    demo_advanced_revo2(left_ctx_.handle, left_ctx_.slave_id);
    demo_action_sequences(left_ctx_.handle, left_ctx_.slave_id);
    demo_device_info(left_ctx_.handle, left_ctx_.slave_id, uses_revo2_api);
    //测试右手
    print_device_info(right_ctx_.handle,right_ctx_.slave_id);
    demo_basic_position(right_ctx_.handle, right_ctx_.slave_id);
    uses_revo2_api =
        !stark_uses_revo1_motor_api(right_ctx_.hw_type_override);
    demo_speed_current(right_ctx_.handle, right_ctx_.slave_id, uses_revo2_api);
    demo_advanced_revo2(right_ctx_.handle, right_ctx_.slave_id);
    demo_action_sequences(right_ctx_.handle, right_ctx_.slave_id);
    demo_device_info(right_ctx_.handle, right_ctx_.slave_id, uses_revo2_api);
    rclcpp::shutdown();
    return 0;
}
