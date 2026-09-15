#ifndef __DYNAMIXEL_STM32_MX28_P1_H
#define __DYNAMIXEL_STM32_MX28_P1_H

#include "main.h"
#include <stdint.h>

/*
 * DYNAMIXEL MX-28T / MX-28R, legacy Control Table, Protocol 1.0.
 *
 * IMPORTANT:
 * - This driver is for the old MX-28 Protocol 1.0 control table
 *   (Goal Position at address 30, Torque Enable at 24, etc.).
 * - MX-28 with MX(2.0) firmware uses a different control table.
 * - The UART helper below assumes a TTL single-wire half-duplex interface
 *   (MX-28T/AT). MX-28R/AR needs an external RS-485 transceiver and
 *   direction control instead of STM32 single-wire half-duplex mode.
 *
 * Public functions return uint8_t: 1 = success, 0 = failure.
 */

#define DXL_DEFAULT_TIMEOUT_MS          100
#define DXL_BROADCAST_ID                0xFE

/* Protocol 1.0 instructions */
#define DXL_INST_PING                   0x01
#define DXL_INST_READ                   0x02
#define DXL_INST_WRITE                  0x03
#define DXL_INST_REG_WRITE              0x04
#define DXL_INST_ACTION                 0x05
#define DXL_INST_FACTORY_RESET          0x06
#define DXL_INST_SYNC_WRITE             0x83

/* MX-28 Protocol 1.0 Control Table: EEPROM area */
#define DXL_ADDR_MODEL_NUMBER           0
#define DXL_ADDR_FIRMWARE_VERSION       2
#define DXL_ADDR_ID                     3
#define DXL_ADDR_BAUD_RATE              4
#define DXL_ADDR_RETURN_DELAY_TIME      5
#define DXL_ADDR_CW_ANGLE_LIMIT         6
#define DXL_ADDR_CCW_ANGLE_LIMIT        8
#define DXL_ADDR_TEMP_LIMIT             11
#define DXL_ADDR_MIN_VOLTAGE_LIMIT      12
#define DXL_ADDR_MAX_VOLTAGE_LIMIT      13
#define DXL_ADDR_MAX_TORQUE             14
#define DXL_ADDR_STATUS_RETURN_LEVEL    16
#define DXL_ADDR_ALARM_LED              17
#define DXL_ADDR_SHUTDOWN               18
#define DXL_ADDR_MULTI_TURN_OFFSET      20
#define DXL_ADDR_RESOLUTION_DIVIDER     22

/* MX-28 Protocol 1.0 Control Table: RAM area */
#define DXL_ADDR_TORQUE_ENABLE          24
#define DXL_ADDR_LED                    25
#define DXL_ADDR_D_GAIN                 26
#define DXL_ADDR_I_GAIN                 27
#define DXL_ADDR_P_GAIN                 28
#define DXL_ADDR_GOAL_POSITION          30
#define DXL_ADDR_MOVING_SPEED           32
#define DXL_ADDR_TORQUE_LIMIT           34
#define DXL_ADDR_PRESENT_POSITION       36
#define DXL_ADDR_PRESENT_SPEED          38
#define DXL_ADDR_PRESENT_LOAD           40
#define DXL_ADDR_PRESENT_VOLTAGE        42
#define DXL_ADDR_PRESENT_TEMP           43
#define DXL_ADDR_REGISTERED             44
#define DXL_ADDR_MOVING                 46
#define DXL_ADDR_LOCK                   47
#define DXL_ADDR_PUNCH                  48
#define DXL_ADDR_REALTIME_TICK          50
#define DXL_ADDR_GOAL_ACCELERATION      73

/* MX-28 limits / units */
#define DXL_MX28_MAX_POSITION           4095
#define DXL_MX28_MAX_SPEED              1023
#define DXL_MX28_MAX_TORQUE             1023
#define DXL_MX28_MAX_PID_GAIN           254
#define DXL_MX28_MAX_GOAL_ACCELERATION  254
#define DXL_MX28_MULTI_TURN_MIN_POSITION (-28672)
#define DXL_MX28_MULTI_TURN_MAX_POSITION  28672
#define DXL_MX28_MULTI_TURN_OFFSET_MIN   (-24576)
#define DXL_MX28_MULTI_TURN_OFFSET_MAX    24576

#define DXL_MX28_POSITION_DEG_PER_UNIT  (360.0f / 4096.0f)
#define DXL_MX28_SPEED_RPM_PER_UNIT     0.114f
#define DXL_MX28_ACCEL_DEG_S2_PER_UNIT  8.583f

#define DXL_CW_DIRECTION                0
#define DXL_CCW_DIRECTION               1

/* Common baud-rate register values */
#define DXL_MX28_BAUD_2000000           0
#define DXL_MX28_BAUD_1000000           1
#define DXL_MX28_BAUD_500000            3
#define DXL_MX28_BAUD_400000            4
#define DXL_MX28_BAUD_250000            7
#define DXL_MX28_BAUD_200000            9
#define DXL_MX28_BAUD_115200            16
#define DXL_MX28_BAUD_57600             34
#define DXL_MX28_BAUD_19200             103
#define DXL_MX28_BAUD_9600              207
#define DXL_MX28_BAUD_2250000           250
#define DXL_MX28_BAUD_2500000           251
#define DXL_MX28_BAUD_3000000           252

/* Low-level functions */
HAL_StatusTypeDef dynamixel_uart_send(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t length);
uint8_t dynamixel_send_packet_v1(UART_HandleTypeDef *huart, uint8_t id, uint8_t instruction, const uint8_t *params, uint8_t params_len);
uint8_t dynamixel_read_status_packet_v1(UART_HandleTypeDef *huart, uint8_t *packet, uint8_t packet_max_len, uint8_t *packet_len);
uint8_t dynamixel_checksum_v1(const uint8_t *packet, uint8_t packet_len);

/* Common Protocol 1.0 operations */
uint8_t dynamixel_ping(UART_HandleTypeDef *huart, uint8_t id);
uint8_t dynamixel_write(UART_HandleTypeDef *huart, uint8_t id, uint8_t address, const uint8_t *data, uint8_t data_len);
uint8_t dynamixel_read(UART_HandleTypeDef *huart, uint8_t id, uint8_t address, uint8_t data_len, uint8_t *out_data, uint8_t *out_len);
uint8_t dynamixel_action(UART_HandleTypeDef *huart);
uint8_t dynamixel_factory_reset(UART_HandleTypeDef *huart, uint8_t id);

/* MX-28 write helpers */
uint8_t dynamixel_set_led(UART_HandleTypeDef *huart, uint8_t id, uint8_t enable);
uint8_t dynamixel_set_torque_enable(UART_HandleTypeDef *huart, uint8_t id, uint8_t enable);
uint8_t dynamixel_set_angle_limits(UART_HandleTypeDef *huart, uint8_t id, uint16_t cw_limit, uint16_t ccw_limit);
uint8_t dynamixel_set_joint_mode(UART_HandleTypeDef *huart, uint8_t id);
uint8_t dynamixel_set_wheel_mode(UART_HandleTypeDef *huart, uint8_t id);
uint8_t dynamixel_set_multiturn_mode(UART_HandleTypeDef *huart, uint8_t id);
uint8_t dynamixel_set_goal_position(UART_HandleTypeDef *huart, uint8_t id, int32_t position);
uint8_t dynamixel_set_multiturn_goal_position(UART_HandleTypeDef *huart, uint8_t id, int32_t position);
uint8_t dynamixel_set_moving_speed(UART_HandleTypeDef *huart, uint8_t id, int32_t speed);
uint8_t dynamixel_set_wheel_speed(UART_HandleTypeDef *huart, uint8_t id, int32_t speed);
uint8_t dynamixel_set_position_and_velocity(UART_HandleTypeDef *huart, uint8_t id, int32_t position, int32_t velocity);
uint8_t dynamixel_set_multiturn_position_and_velocity(UART_HandleTypeDef *huart, uint8_t id, int32_t position, int32_t velocity);
uint8_t dynamixel_set_max_torque(UART_HandleTypeDef *huart, uint8_t id, uint16_t max_torque);
uint8_t dynamixel_set_torque_limit(UART_HandleTypeDef *huart, uint8_t id, uint16_t torque_limit);
uint8_t dynamixel_set_id(UART_HandleTypeDef *huart, uint8_t id, uint8_t new_id);
uint8_t dynamixel_set_baudrate(UART_HandleTypeDef *huart, uint8_t id, uint8_t baudrate_value);
uint8_t dynamixel_set_return_delay_time(UART_HandleTypeDef *huart, uint8_t id, uint8_t delay_time);
uint8_t dynamixel_set_multi_turn_offset(UART_HandleTypeDef *huart, uint8_t id, int32_t offset);
uint8_t dynamixel_set_resolution_divider(UART_HandleTypeDef *huart, uint8_t id, uint8_t divider);
uint8_t dynamixel_set_d_gain(UART_HandleTypeDef *huart, uint8_t id, uint8_t gain);
uint8_t dynamixel_set_i_gain(UART_HandleTypeDef *huart, uint8_t id, uint8_t gain);
uint8_t dynamixel_set_p_gain(UART_HandleTypeDef *huart, uint8_t id, uint8_t gain);
uint8_t dynamixel_set_pid_gains(UART_HandleTypeDef *huart, uint8_t id, uint8_t p_gain, uint8_t i_gain, uint8_t d_gain);
uint8_t dynamixel_set_punch(UART_HandleTypeDef *huart, uint8_t id, uint16_t punch);
uint8_t dynamixel_set_goal_acceleration(UART_HandleTypeDef *huart, uint8_t id, uint8_t acceleration);

/* MX-28 read helpers */
uint8_t dynamixel_read_present_position(UART_HandleTypeDef *huart, uint8_t id, int32_t *position);
uint8_t dynamixel_read_present_moving_velocity(UART_HandleTypeDef *huart, uint8_t id, int32_t *velocity);
uint8_t dynamixel_read_present_load(UART_HandleTypeDef *huart, uint8_t id, int32_t *load);
uint8_t dynamixel_read_present_voltage(UART_HandleTypeDef *huart, uint8_t id, int32_t *voltage_x10);
uint8_t dynamixel_read_present_temperature(UART_HandleTypeDef *huart, uint8_t id, int32_t *temperature_c);
uint8_t dynamixel_read_moving(UART_HandleTypeDef *huart, uint8_t id, uint8_t *moving);
uint8_t dynamixel_read_realtime_tick(UART_HandleTypeDef *huart, uint8_t id, uint16_t *tick_ms);
uint8_t dynamixel_read_id(UART_HandleTypeDef *huart, uint8_t id, uint8_t *read_id);
uint8_t dynamixel_read_baudrate(UART_HandleTypeDef *huart, uint8_t id, uint8_t *baudrate_value);

/* Utility */
uint8_t dynamixel_baudrate_to_value(uint32_t baudrate);
uint32_t dynamixel_value_to_baudrate(uint8_t value);
uint16_t dynamixel_pos_deg_to_value(float pos_deg);
float dynamixel_pos_value_to_deg(uint16_t position);
uint16_t dynamixel_vel_deg_s_to_value(float speed_deg_s);
uint16_t dynamixel_vel_pct_to_wheel_value(float speed_pct);
uint8_t dynamixel_accel_deg_s2_to_value(float acceleration_deg_s2);
int32_t dynamixel_mx28_decode_signed_speed(uint16_t raw);
int32_t dynamixel_mx28_decode_signed_load(uint16_t raw);

#endif /* __DYNAMIXEL_STM32_MX28_P1_H */
