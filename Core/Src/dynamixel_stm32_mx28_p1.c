#include "dynamixel_stm32_mx28_p1.h"
#include <string.h>

/*
 * Set to 1 if you want WRITE commands to wait for a Status Packet.
 * For first bring-up, 0 is safer because it avoids blocking on every WRITE
 * if Status Return Level is configured to not respond to writes.
 */
#define DXL_WAIT_STATUS_AFTER_WRITE  0

static uint16_t clamp_u16(uint16_t value, uint16_t max_value)
{
    return (value > max_value) ? max_value : value;
}

static int32_t clamp_i32(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static void make_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)((value >> 0) & 0xFF);
    data[1] = (uint8_t)((value >> 8) & 0xFF);
}

static uint16_t read_u16_le(const uint8_t *data)
{
    return ((uint16_t)data[0] << 0) |
           ((uint16_t)data[1] << 8);
}

HAL_StatusTypeDef dynamixel_uart_send(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t length)
{
    HAL_StatusTypeDef ret;

    HAL_HalfDuplex_EnableTransmitter(huart);

    ret = HAL_UART_Transmit(huart, (uint8_t *)data, length, DXL_DEFAULT_TIMEOUT_MS);
    if (ret != HAL_OK)
    {
        HAL_HalfDuplex_EnableReceiver(huart);
        return ret;
    }

    while (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == RESET)
    {
    }

    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);

    HAL_HalfDuplex_EnableReceiver(huart);
    return HAL_OK;
}

uint8_t dynamixel_checksum_v1(const uint8_t *packet, uint8_t packet_len)
{
    /*
     * Protocol 1.0 packet:
     * FF FF ID LENGTH INSTRUCTION PARAMS... CHECKSUM
     * CHECKSUM = ~(ID + LENGTH + INSTRUCTION + PARAMS...)
     */
    uint16_t sum = 0;

    if (packet_len < 6)
        return 0;

    for (uint8_t i = 2; i < packet_len - 1; i++)
    {
        sum += packet[i];
    }

    return (uint8_t)(~sum & 0xFF);
}

uint8_t dynamixel_send_packet_v1(
    UART_HandleTypeDef *huart,
    uint8_t id,
    uint8_t instruction,
    const uint8_t *params,
    uint8_t params_len
)
{
    /*
     * FF FF ID LENGTH INSTRUCTION PARAMS... CHECKSUM
     * LENGTH = params_len + 2, where 2 = instruction + checksum
     */
    uint8_t packet_len = (uint8_t)(6 + params_len);

    if (packet_len > 128)
        return 0;

    uint8_t packet[128];

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = id;
    packet[3] = (uint8_t)(params_len + 2);
    packet[4] = instruction;

    for (uint8_t i = 0; i < params_len; i++)
    {
        packet[5 + i] = params[i];
    }

    packet[packet_len - 1] = dynamixel_checksum_v1(packet, packet_len);

    return (dynamixel_uart_send(huart, packet, packet_len) == HAL_OK);
}

uint8_t dynamixel_read_status_packet_v1(
    UART_HandleTypeDef *huart,
    uint8_t *packet,
    uint8_t packet_max_len,
    uint8_t *packet_len
)
{
    uint8_t header[4];

    /*
     * Protocol 1.0 Status Packet:
     * FF FF ID LENGTH ERROR PARAMS... CHECKSUM
     * Total length = 4 + LENGTH
     */
    if (HAL_UART_Receive(huart, header, 4, DXL_DEFAULT_TIMEOUT_MS) != HAL_OK)
        return 0;

    if (header[0] != 0xFF || header[1] != 0xFF)
        return 0;

    uint8_t length_field = header[3];
    uint8_t total_len = (uint8_t)(4 + length_field);

    if (total_len > packet_max_len)
        return 0;

    memcpy(packet, header, 4);

    if (HAL_UART_Receive(huart, &packet[4], length_field, DXL_DEFAULT_TIMEOUT_MS) != HAL_OK)
        return 0;

    uint8_t expected = dynamixel_checksum_v1(packet, total_len);
    if (packet[total_len - 1] != expected)
        return 0;

    if (packet_len != NULL)
        *packet_len = total_len;

    return 1;
}

uint8_t dynamixel_ping(UART_HandleTypeDef *huart, uint8_t id)
{
    uint8_t packet[16];
    uint8_t packet_len = 0;

    if (!dynamixel_send_packet_v1(huart, id, DXL_INST_PING, NULL, 0))
        return 0;

    if (!dynamixel_read_status_packet_v1(huart, packet, sizeof(packet), &packet_len))
        return 0;

    if (packet[2] != id)
        return 0;

    /* packet[4] = ERROR */
    if (packet[4] != 0x00)
        return 0;

    return 1;
}

uint8_t dynamixel_write(
    UART_HandleTypeDef *huart,
    uint8_t id,
    uint8_t address,
    const uint8_t *data,
    uint8_t data_len
)
{
    /* WRITE params: START_ADDRESS DATA... */
    uint8_t params_len = (uint8_t)(1 + data_len);

    if (params_len > 64)
        return 0;

    uint8_t params[64];
    params[0] = address;

    for (uint8_t i = 0; i < data_len; i++)
    {
        params[1 + i] = data[i];
    }

    if (!dynamixel_send_packet_v1(huart, id, DXL_INST_WRITE, params, params_len))
        return 0;

#if DXL_WAIT_STATUS_AFTER_WRITE
    if (id != DXL_BROADCAST_ID)
    {
        uint8_t status[16];
        uint8_t status_len = 0;

        if (!dynamixel_read_status_packet_v1(huart, status, sizeof(status), &status_len))
            return 0;

        if (status[2] != id || status[4] != 0x00)
            return 0;
    }
#endif

    return 1;
}

uint8_t dynamixel_read(
    UART_HandleTypeDef *huart,
    uint8_t id,
    uint8_t address,
    uint8_t data_len,
    uint8_t *out_data,
    uint8_t *out_len
)
{
    uint8_t params[2];
    params[0] = address;
    params[1] = data_len;

    if (!dynamixel_send_packet_v1(huart, id, DXL_INST_READ, params, 2))
        return 0;

    uint8_t packet[64];
    uint8_t packet_len = 0;

    if (!dynamixel_read_status_packet_v1(huart, packet, sizeof(packet), &packet_len))
        return 0;

    if (packet[2] != id || packet[4] != 0x00)
        return 0;

    /* LENGTH = error + params + checksum, so params_len = LENGTH - 2 */
    uint8_t params_returned = (uint8_t)(packet[3] - 2);
    if (params_returned > data_len)
        params_returned = data_len;

    for (uint8_t i = 0; i < params_returned; i++)
    {
        out_data[i] = packet[5 + i];
    }

    if (out_len != NULL)
        *out_len = params_returned;

    return 1;
}

uint8_t dynamixel_action(UART_HandleTypeDef *huart)
{
    return dynamixel_send_packet_v1(huart, DXL_BROADCAST_ID, DXL_INST_ACTION, NULL, 0);
}

uint8_t dynamixel_factory_reset(UART_HandleTypeDef *huart, uint8_t id)
{
    return dynamixel_send_packet_v1(huart, id, DXL_INST_FACTORY_RESET, NULL, 0);
}

uint8_t dynamixel_set_led(UART_HandleTypeDef *huart, uint8_t id, uint8_t enable)
{
    uint8_t data = enable ? 1U : 0U;
    return dynamixel_write(huart, id, DXL_ADDR_LED, &data, 1);
}

uint8_t dynamixel_set_torque_enable(UART_HandleTypeDef *huart, uint8_t id, uint8_t enable)
{
    uint8_t data = enable ? 1U : 0U;
    return dynamixel_write(huart, id, DXL_ADDR_TORQUE_ENABLE, &data, 1);
}

uint8_t dynamixel_set_angle_limits(UART_HandleTypeDef *huart, uint8_t id, uint16_t cw_limit, uint16_t ccw_limit)
{
    uint8_t data[4];

    cw_limit = clamp_u16(cw_limit, DXL_MX28_MAX_POSITION);
    ccw_limit = clamp_u16(ccw_limit, DXL_MX28_MAX_POSITION);

    make_u16_le(&data[0], cw_limit);
    make_u16_le(&data[2], ccw_limit);

    return dynamixel_write(huart, id, DXL_ADDR_CW_ANGLE_LIMIT, data, 4);
}

uint8_t dynamixel_set_joint_mode(UART_HandleTypeDef *huart, uint8_t id)
{
    /* 0 / 4095 gives the full single-turn joint range. */
    return dynamixel_set_angle_limits(huart, id, 0, DXL_MX28_MAX_POSITION);
}

uint8_t dynamixel_set_wheel_mode(UART_HandleTypeDef *huart, uint8_t id)
{
    /* MX-28 Protocol 1.0: CW = 0 and CCW = 0 -> Wheel Mode. */
    return dynamixel_set_angle_limits(huart, id, 0, 0);
}

uint8_t dynamixel_set_multiturn_mode(UART_HandleTypeDef *huart, uint8_t id)
{
    /* MX-28 Protocol 1.0: CW = 4095 and CCW = 4095 -> Multi-turn Mode. */
    return dynamixel_set_angle_limits(huart, id,
                                      DXL_MX28_MAX_POSITION,
                                      DXL_MX28_MAX_POSITION);
}

uint8_t dynamixel_set_goal_position(UART_HandleTypeDef *huart, uint8_t id, int32_t position)
{
    uint8_t data[2];
    uint16_t pos = (uint16_t)clamp_i32(position, 0, DXL_MX28_MAX_POSITION);

    make_u16_le(data, pos);
    return dynamixel_write(huart, id, DXL_ADDR_GOAL_POSITION, data, 2);
}

uint8_t dynamixel_set_multiturn_goal_position(UART_HandleTypeDef *huart, uint8_t id, int32_t position)
{
    uint8_t data[2];

    position = clamp_i32(position,
                         DXL_MX28_MULTI_TURN_MIN_POSITION,
                         DXL_MX28_MULTI_TURN_MAX_POSITION);

    /* Multi-turn negative positions are stored as 16-bit two's complement. */
    make_u16_le(data, (uint16_t)(int16_t)position);
    return dynamixel_write(huart, id, DXL_ADDR_GOAL_POSITION, data, 2);
}

uint8_t dynamixel_set_moving_speed(UART_HandleTypeDef *huart, uint8_t id, int32_t speed)
{
    uint8_t data[2];

    /*
     * Joint / Multi-turn mode:
     *   0..1023, approximately 0.114 rpm per unit.
     *   0 means no speed limiting (maximum available speed).
     */
    uint16_t encoded = (uint16_t)clamp_i32(speed, 0, DXL_MX28_MAX_SPEED);

    make_u16_le(data, encoded);
    return dynamixel_write(huart, id, DXL_ADDR_MOVING_SPEED, data, 2);
}

uint8_t dynamixel_set_wheel_speed(UART_HandleTypeDef *huart, uint8_t id, int32_t speed)
{
    uint8_t data[2];
    uint16_t encoded;

    /*
     * Wheel Mode:
     *   bits 0..9 = magnitude, bit 10 = direction.
     *   positive -> CCW, negative -> CW.
     *   0x000 = CCW stop, 0x400 = CW stop.
     */
    if (speed < 0)
    {
        int32_t magnitude_i32 = -speed;
        if (speed == INT32_MIN)
            magnitude_i32 = DXL_MX28_MAX_SPEED;

        uint16_t magnitude = (uint16_t)clamp_i32(magnitude_i32, 0, DXL_MX28_MAX_SPEED);
        encoded = (uint16_t)(magnitude | 0x0400U);
    }
    else
    {
        encoded = (uint16_t)clamp_i32(speed, 0, DXL_MX28_MAX_SPEED);
    }

    make_u16_le(data, encoded);
    return dynamixel_write(huart, id, DXL_ADDR_MOVING_SPEED, data, 2);
}

uint8_t dynamixel_set_position_and_velocity(UART_HandleTypeDef *huart, uint8_t id, int32_t position, int32_t velocity)
{
    uint8_t data[4];

    uint16_t pos = (uint16_t)clamp_i32(position, 0, DXL_MX28_MAX_POSITION);
    uint16_t vel = (uint16_t)clamp_i32(velocity, 0, DXL_MX28_MAX_SPEED);

    /* 30..31 = Goal Position, 32..33 = Moving Speed. */
    make_u16_le(&data[0], pos);
    make_u16_le(&data[2], vel);

    return dynamixel_write(huart, id, DXL_ADDR_GOAL_POSITION, data, 4);
}

uint8_t dynamixel_set_multiturn_position_and_velocity(UART_HandleTypeDef *huart, uint8_t id, int32_t position, int32_t velocity)
{
    uint8_t data[4];

    position = clamp_i32(position,
                         DXL_MX28_MULTI_TURN_MIN_POSITION,
                         DXL_MX28_MULTI_TURN_MAX_POSITION);

    uint16_t pos = (uint16_t)(int16_t)position;
    uint16_t vel = (uint16_t)clamp_i32(velocity, 0, DXL_MX28_MAX_SPEED);

    make_u16_le(&data[0], pos);
    make_u16_le(&data[2], vel);

    return dynamixel_write(huart, id, DXL_ADDR_GOAL_POSITION, data, 4);
}

uint8_t dynamixel_set_max_torque(UART_HandleTypeDef *huart, uint8_t id, uint16_t max_torque)
{
    uint8_t data[2];
    max_torque = clamp_u16(max_torque, DXL_MX28_MAX_TORQUE);
    make_u16_le(data, max_torque);
    return dynamixel_write(huart, id, DXL_ADDR_MAX_TORQUE, data, 2);
}

uint8_t dynamixel_set_torque_limit(UART_HandleTypeDef *huart, uint8_t id, uint16_t torque_limit)
{
    uint8_t data[2];
    torque_limit = clamp_u16(torque_limit, DXL_MX28_MAX_TORQUE);
    make_u16_le(data, torque_limit);
    return dynamixel_write(huart, id, DXL_ADDR_TORQUE_LIMIT, data, 2);
}

uint8_t dynamixel_set_id(UART_HandleTypeDef *huart, uint8_t id, uint8_t new_id)
{
    if (new_id >= DXL_BROADCAST_ID)
        return 0;

    return dynamixel_write(huart, id, DXL_ADDR_ID, &new_id, 1);
}

uint8_t dynamixel_set_baudrate(UART_HandleTypeDef *huart, uint8_t id, uint8_t baudrate_value)
{
    return dynamixel_write(huart, id, DXL_ADDR_BAUD_RATE, &baudrate_value, 1);
}

uint8_t dynamixel_set_return_delay_time(UART_HandleTypeDef *huart, uint8_t id, uint8_t delay_time)
{
    /* Unit = 2 us, valid register range = 0..254. */
    if (delay_time > 254U)
        delay_time = 254U;

    return dynamixel_write(huart, id, DXL_ADDR_RETURN_DELAY_TIME, &delay_time, 1);
}

uint8_t dynamixel_set_multi_turn_offset(UART_HandleTypeDef *huart, uint8_t id, int32_t offset)
{
    uint8_t data[2];

    offset = clamp_i32(offset,
                       DXL_MX28_MULTI_TURN_OFFSET_MIN,
                       DXL_MX28_MULTI_TURN_OFFSET_MAX);

    make_u16_le(data, (uint16_t)(int16_t)offset);
    return dynamixel_write(huart, id, DXL_ADDR_MULTI_TURN_OFFSET, data, 2);
}

uint8_t dynamixel_set_resolution_divider(UART_HandleTypeDef *huart, uint8_t id, uint8_t divider)
{
    if (divider < 1U)
        divider = 1U;
    if (divider > 4U)
        divider = 4U;

    return dynamixel_write(huart, id, DXL_ADDR_RESOLUTION_DIVIDER, &divider, 1);
}

uint8_t dynamixel_set_d_gain(UART_HandleTypeDef *huart, uint8_t id, uint8_t gain)
{
    if (gain > DXL_MX28_MAX_PID_GAIN)
        gain = DXL_MX28_MAX_PID_GAIN;

    return dynamixel_write(huart, id, DXL_ADDR_D_GAIN, &gain, 1);
}

uint8_t dynamixel_set_i_gain(UART_HandleTypeDef *huart, uint8_t id, uint8_t gain)
{
    if (gain > DXL_MX28_MAX_PID_GAIN)
        gain = DXL_MX28_MAX_PID_GAIN;

    return dynamixel_write(huart, id, DXL_ADDR_I_GAIN, &gain, 1);
}

uint8_t dynamixel_set_p_gain(UART_HandleTypeDef *huart, uint8_t id, uint8_t gain)
{
    if (gain > DXL_MX28_MAX_PID_GAIN)
        gain = DXL_MX28_MAX_PID_GAIN;

    return dynamixel_write(huart, id, DXL_ADDR_P_GAIN, &gain, 1);
}

uint8_t dynamixel_set_pid_gains(UART_HandleTypeDef *huart, uint8_t id, uint8_t p_gain, uint8_t i_gain, uint8_t d_gain)
{
    /* Registers are adjacent in D, I, P order: 26, 27, 28. */
    uint8_t data[3];

    if (p_gain > DXL_MX28_MAX_PID_GAIN) p_gain = DXL_MX28_MAX_PID_GAIN;
    if (i_gain > DXL_MX28_MAX_PID_GAIN) i_gain = DXL_MX28_MAX_PID_GAIN;
    if (d_gain > DXL_MX28_MAX_PID_GAIN) d_gain = DXL_MX28_MAX_PID_GAIN;

    data[0] = d_gain;
    data[1] = i_gain;
    data[2] = p_gain;

    return dynamixel_write(huart, id, DXL_ADDR_D_GAIN, data, 3);
}

uint8_t dynamixel_set_punch(UART_HandleTypeDef *huart, uint8_t id, uint16_t punch)
{
    uint8_t data[2];
    punch = clamp_u16(punch, 1023U);
    make_u16_le(data, punch);
    return dynamixel_write(huart, id, DXL_ADDR_PUNCH, data, 2);
}

uint8_t dynamixel_set_goal_acceleration(UART_HandleTypeDef *huart, uint8_t id, uint8_t acceleration)
{
    if (acceleration > DXL_MX28_MAX_GOAL_ACCELERATION)
        acceleration = DXL_MX28_MAX_GOAL_ACCELERATION;

    return dynamixel_write(huart, id, DXL_ADDR_GOAL_ACCELERATION, &acceleration, 1);
}

uint8_t dynamixel_read_present_position(UART_HandleTypeDef *huart, uint8_t id, int32_t *position)
{
    uint8_t data[2];
    uint8_t len = 0;

    if (position == NULL)
        return 0;

    if (!dynamixel_read(huart, id, DXL_ADDR_PRESENT_POSITION, 2, data, &len))
        return 0;

    if (len != 2)
        return 0;

    /*
     * In Joint Mode values are 0..4095.
     * In Multi-turn Mode negative positions are 16-bit two's complement.
     * int16_t decoding handles both cases correctly.
     */
    *position = (int32_t)(int16_t)read_u16_le(data);
    return 1;
}

uint8_t dynamixel_read_present_moving_velocity(UART_HandleTypeDef *huart, uint8_t id, int32_t *velocity)
{
    uint8_t data[2];
    uint8_t len = 0;

    if (velocity == NULL)
        return 0;

    if (!dynamixel_read(huart, id, DXL_ADDR_PRESENT_SPEED, 2, data, &len))
        return 0;

    if (len != 2)
        return 0;

    *velocity = dynamixel_mx28_decode_signed_speed(read_u16_le(data));
    return 1;
}

uint8_t dynamixel_read_present_load(UART_HandleTypeDef *huart, uint8_t id, int32_t *load)
{
    uint8_t data[2];
    uint8_t len = 0;

    if (load == NULL)
        return 0;

    if (!dynamixel_read(huart, id, DXL_ADDR_PRESENT_LOAD, 2, data, &len))
        return 0;

    if (len != 2)
        return 0;

    *load = dynamixel_mx28_decode_signed_load(read_u16_le(data));
    return 1;
}

uint8_t dynamixel_read_present_voltage(UART_HandleTypeDef *huart, uint8_t id, int32_t *voltage_x10)
{
    uint8_t data[1];
    uint8_t len = 0;

    if (voltage_x10 == NULL)
        return 0;

    if (!dynamixel_read(huart, id, DXL_ADDR_PRESENT_VOLTAGE, 1, data, &len))
        return 0;

    if (len != 1)
        return 0;

    /* Unit = 0.1 V. Example: 127 -> 12.7 V. */
    *voltage_x10 = data[0];
    return 1;
}

uint8_t dynamixel_read_present_temperature(UART_HandleTypeDef *huart, uint8_t id, int32_t *temperature_c)
{
    uint8_t data[1];
    uint8_t len = 0;

    if (temperature_c == NULL)
        return 0;

    if (!dynamixel_read(huart, id, DXL_ADDR_PRESENT_TEMP, 1, data, &len))
        return 0;

    if (len != 1)
        return 0;

    *temperature_c = data[0];
    return 1;
}

uint8_t dynamixel_read_moving(UART_HandleTypeDef *huart, uint8_t id, uint8_t *moving)
{
    uint8_t data[1];
    uint8_t len = 0;

    if (moving == NULL)
        return 0;

    if (!dynamixel_read(huart, id, DXL_ADDR_MOVING, 1, data, &len))
        return 0;

    if (len != 1)
        return 0;

    *moving = data[0] & 0x01U;
    return 1;
}

uint8_t dynamixel_read_realtime_tick(UART_HandleTypeDef *huart, uint8_t id, uint16_t *tick_ms)
{
    uint8_t data[2];
    uint8_t len = 0;

    if (tick_ms == NULL)
        return 0;

    if (!dynamixel_read(huart, id, DXL_ADDR_REALTIME_TICK, 2, data, &len))
        return 0;

    if (len != 2)
        return 0;

    *tick_ms = read_u16_le(data);
    return 1;
}

uint8_t dynamixel_read_id(UART_HandleTypeDef *huart, uint8_t id, uint8_t *read_id)
{
    uint8_t data[1];
    uint8_t len = 0;

    if (read_id == NULL)
        return 0;

    if (!dynamixel_read(huart, id, DXL_ADDR_ID, 1, data, &len))
        return 0;

    if (len != 1)
        return 0;

    *read_id = data[0];
    return 1;
}

uint8_t dynamixel_read_baudrate(UART_HandleTypeDef *huart, uint8_t id, uint8_t *baudrate_value)
{
    uint8_t data[1];
    uint8_t len = 0;

    if (baudrate_value == NULL)
        return 0;

    if (!dynamixel_read(huart, id, DXL_ADDR_BAUD_RATE, 1, data, &len))
        return 0;

    if (len != 1)
        return 0;

    *baudrate_value = data[0];
    return 1;
}

uint8_t dynamixel_baudrate_to_value(uint32_t baudrate)
{
    /* MX-28 has special encodings above 2 Mbps. */
    if (baudrate == 2250000U) return DXL_MX28_BAUD_2250000;
    if (baudrate == 2500000U) return DXL_MX28_BAUD_2500000;
    if (baudrate == 3000000U) return DXL_MX28_BAUD_3000000;

    if (baudrate == 0U)
        return 0;

    uint32_t divider = 2000000U / baudrate;
    if (divider == 0U)
        return 0;

    uint32_t value = divider - 1U;
    if (value > 249U)
        value = 249U;

    return (uint8_t)value;
}

uint32_t dynamixel_value_to_baudrate(uint8_t value)
{
    if (value == DXL_MX28_BAUD_2250000) return 2250000U;
    if (value == DXL_MX28_BAUD_2500000) return 2500000U;
    if (value == DXL_MX28_BAUD_3000000) return 3000000U;

    return (uint32_t)(2000000U / ((uint32_t)value + 1U));
}

uint16_t dynamixel_pos_deg_to_value(float pos_deg)
{
    if (pos_deg < 0.0f)
        pos_deg = 0.0f;

    /* 4096 encoder counts/revolution; 4095 corresponds to about 359.91 deg. */
    if (pos_deg >= 360.0f)
        return DXL_MX28_MAX_POSITION;

    uint32_t value = (uint32_t)(pos_deg / DXL_MX28_POSITION_DEG_PER_UNIT + 0.5f);
    if (value > DXL_MX28_MAX_POSITION)
        value = DXL_MX28_MAX_POSITION;

    return (uint16_t)value;
}

float dynamixel_pos_value_to_deg(uint16_t position)
{
    position = clamp_u16(position, DXL_MX28_MAX_POSITION);
    return (float)position * DXL_MX28_POSITION_DEG_PER_UNIT;
}

uint16_t dynamixel_vel_deg_s_to_value(float speed_deg_s)
{
    if (speed_deg_s <= 0.0f)
        return 0;

    /* 0.114 rpm/unit * 6 deg/s per rpm = 0.684 deg/s per unit. */
    uint32_t value = (uint32_t)(speed_deg_s / (DXL_MX28_SPEED_RPM_PER_UNIT * 6.0f) + 0.5f);
    if (value > DXL_MX28_MAX_SPEED)
        value = DXL_MX28_MAX_SPEED;

    return (uint16_t)value;
}

uint16_t dynamixel_vel_pct_to_wheel_value(float speed_pct)
{
    if (speed_pct < 0.0f)
        speed_pct = 0.0f;

    if (speed_pct > 100.0f)
        speed_pct = 100.0f;

    return (uint16_t)(speed_pct * (float)DXL_MX28_MAX_SPEED / 100.0f + 0.5f);
}

uint8_t dynamixel_accel_deg_s2_to_value(float acceleration_deg_s2)
{
    if (acceleration_deg_s2 <= 0.0f)
        return 0;  /* 0 means maximum acceleration / acceleration control disabled. */

    uint32_t value = (uint32_t)(acceleration_deg_s2 / DXL_MX28_ACCEL_DEG_S2_PER_UNIT + 0.5f);
    if (value > DXL_MX28_MAX_GOAL_ACCELERATION)
        value = DXL_MX28_MAX_GOAL_ACCELERATION;

    return (uint8_t)value;
}

int32_t dynamixel_mx28_decode_signed_speed(uint16_t raw)
{
    int32_t magnitude = (int32_t)(raw & 0x03FFU);

    /* Bit 10 is direction: 0 = CCW, 1 = CW. */
    if (raw & 0x0400U)
        return -magnitude;

    return magnitude;
}

int32_t dynamixel_mx28_decode_signed_load(uint16_t raw)
{
    int32_t magnitude = (int32_t)(raw & 0x03FFU);

    /* Bit 10 is direction: 0 = CCW, 1 = CW. */
    if (raw & 0x0400U)
        return -magnitude;

    return magnitude;
}
