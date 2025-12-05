/**
 * @file rs485_motor.c
 * @brief RS-485 Motor Driver Implementation
 */

#include "rs485_motor.h"
#include "stm32h7xx_hal.h"
#include <string.h>

#define DBG_TAG "rs485"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* ============== Internal Macros ============== */

#define RS485_DIR_RX    0
#define RS485_DIR_TX    1

/* ============== Protocol Handlers ============== */

static rs485_encode_func_t custom_encode[RS485_MAX_BUSES] = {NULL};
static rs485_decode_func_t custom_decode[RS485_MAX_BUSES] = {NULL};

/* ============== CRC-16 Table (Modbus) ============== */

static const uint16_t crc16_table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

/* ============== Internal Functions ============== */

static inline void rs485_set_direction(rs485_bus_t *bus, uint8_t dir)
{
    if (bus->config.de_port != NULL) {
        HAL_GPIO_WritePin(bus->config.de_port, bus->config.de_pin,
                         dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

static uint8_t rs485_find_bus_for_motor(rs485_motor_system_t *sys, uint8_t motor_id)
{
    for (uint8_t b = 0; b < RS485_MAX_BUSES; b++) {
        if (!sys->bus[b].initialized) continue;
        for (uint8_t m = 0; m < sys->bus[b].config.motor_count; m++) {
            if (sys->bus[b].config.motor_ids[m] == motor_id) {
                return b;
            }
        }
    }
    return 0xFF;  /* Not found */
}

static uint16_t rs485_encode_generic(const motor_command_t *cmd, uint8_t *buffer)
{
    /* Generic protocol format:
     * [HEADER:2][ID:1][CMD:1][LEN:1][DATA:N][CRC:2]
     */
    uint16_t idx = 0;

    buffer[idx++] = 0xFF;
    buffer[idx++] = 0xFF;
    buffer[idx++] = cmd->id;
    buffer[idx++] = cmd->mode;
    buffer[idx++] = 8;  /* Data length */

    /* Position (4 bytes) */
    buffer[idx++] = (cmd->target_position >> 0) & 0xFF;
    buffer[idx++] = (cmd->target_position >> 8) & 0xFF;
    buffer[idx++] = (cmd->target_position >> 16) & 0xFF;
    buffer[idx++] = (cmd->target_position >> 24) & 0xFF;

    /* Velocity (2 bytes) */
    buffer[idx++] = (cmd->target_velocity >> 0) & 0xFF;
    buffer[idx++] = (cmd->target_velocity >> 8) & 0xFF;

    /* Torque (2 bytes) */
    buffer[idx++] = (cmd->target_torque >> 0) & 0xFF;
    buffer[idx++] = (cmd->target_torque >> 8) & 0xFF;

    /* CRC */
    uint16_t crc = rs485_crc16(buffer, idx);
    buffer[idx++] = crc & 0xFF;
    buffer[idx++] = (crc >> 8) & 0xFF;

    return idx;
}

static rt_err_t rs485_decode_generic(const uint8_t *buffer, uint16_t len,
                                     motor_feedback_t *feedback)
{
    if (len < 15) return -RT_EINVAL;

    /* Verify header */
    if (buffer[0] != 0xFF || buffer[1] != 0xFF) {
        return -RT_EINVAL;
    }

    /* Verify CRC */
    uint16_t crc_calc = rs485_crc16(buffer, len - 2);
    uint16_t crc_recv = buffer[len-2] | (buffer[len-1] << 8);
    if (crc_calc != crc_recv) {
        return -RT_ERROR;
    }

    feedback->id = buffer[2];
    feedback->state = buffer[3];

    /* Position */
    feedback->position = buffer[5] | (buffer[6] << 8) |
                        (buffer[7] << 16) | (buffer[8] << 24);

    /* Velocity */
    feedback->velocity = buffer[9] | (buffer[10] << 8);

    /* Current */
    feedback->current = buffer[11] | (buffer[12] << 8);

    /* Temperature */
    feedback->temperature = buffer[13];

    /* Voltage */
    feedback->voltage = buffer[14];

    feedback->timestamp = rt_tick_get();

    return RT_EOK;
}

/* ============== Public API Implementation ============== */

uint16_t rs485_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc = (crc >> 8) ^ crc16_table[(crc ^ *data++) & 0xFF];
    }
    return crc;
}

rt_err_t rs485_motor_init(rs485_motor_system_t *sys)
{
    RT_ASSERT(sys != RT_NULL);

    memset(sys, 0, sizeof(rs485_motor_system_t));

    for (uint8_t i = 0; i < RS485_MAX_BUSES; i++) {
        rs485_bus_t *bus = &sys->bus[i];

        rt_err_t ret = rt_mutex_init(&bus->lock, "rs485", RT_IPC_FLAG_PRIO);
        if (ret != RT_EOK) {
            LOG_E("Failed to create mutex for bus %d", i);
            return ret;
        }

        ret = rt_sem_init(&bus->rx_sem, "rs485rx", 0, RT_IPC_FLAG_FIFO);
        if (ret != RT_EOK) {
            LOG_E("Failed to create semaphore for bus %d", i);
            return ret;
        }
    }

    sys->initialized = true;
    LOG_I("RS-485 motor system initialized");

    return RT_EOK;
}

rt_err_t rs485_bus_configure(rs485_motor_system_t *sys, uint8_t bus_id,
                             const rs485_bus_config_t *config)
{
    RT_ASSERT(sys != RT_NULL);
    RT_ASSERT(config != RT_NULL);
    RT_ASSERT(bus_id < RS485_MAX_BUSES);

    rs485_bus_t *bus = &sys->bus[bus_id];

    rt_mutex_take(&bus->lock, RT_WAITING_FOREVER);

    memcpy(&bus->config, config, sizeof(rs485_bus_config_t));

    /* Configure baudrate if different */
    if (config->baudrate != RS485_DEFAULT_BAUDRATE) {
        bus->config.huart->Init.BaudRate = config->baudrate;
        if (HAL_UART_Init(bus->config.huart) != HAL_OK) {
            rt_mutex_release(&bus->lock);
            return -RT_EIO;
        }
    }

    /* Set DE pin to RX mode */
    rs485_set_direction(bus, RS485_DIR_RX);

    /* Enable UART RX interrupt */
    __HAL_UART_ENABLE_IT(bus->config.huart, UART_IT_RXNE);

    /* Initialize feedback structures */
    for (uint8_t i = 0; i < config->motor_count; i++) {
        bus->feedback[i].id = config->motor_ids[i];
        bus->feedback[i].state = MOTOR_STATE_OFFLINE;
    }

    bus->initialized = true;
    sys->total_motors += config->motor_count;

    LOG_I("RS-485 bus %d configured: %d motors, %lu baud",
          bus_id, config->motor_count, config->baudrate);

    rt_mutex_release(&bus->lock);

    return RT_EOK;
}

rt_err_t rs485_bus_send_raw(rs485_motor_system_t *sys, uint8_t bus_id,
                            const uint8_t *data, uint16_t len)
{
    RT_ASSERT(sys != RT_NULL);
    RT_ASSERT(bus_id < RS485_MAX_BUSES);

    rs485_bus_t *bus = &sys->bus[bus_id];

    if (!bus->initialized) return -RT_ENOSYS;

    rt_mutex_take(&bus->lock, RT_WAITING_FOREVER);

    /* Switch to TX mode */
    rs485_set_direction(bus, RS485_DIR_TX);

    /* Small delay for direction switch */
    for (volatile int i = 0; i < 100; i++);

    /* Send data */
    HAL_StatusTypeDef status = HAL_UART_Transmit(bus->config.huart,
                                                  (uint8_t*)data, len,
                                                  RS485_TIMEOUT_MS);

    /* Wait for transmission complete */
    while (__HAL_UART_GET_FLAG(bus->config.huart, UART_FLAG_TC) == RESET);

    /* Switch back to RX mode */
    rs485_set_direction(bus, RS485_DIR_RX);

    if (status == HAL_OK) {
        bus->tx_count++;
    } else {
        bus->error_count++;
    }

    rt_mutex_release(&bus->lock);

    return (status == HAL_OK) ? RT_EOK : -RT_EIO;
}

uint16_t rs485_bus_receive_raw(rs485_motor_system_t *sys, uint8_t bus_id,
                               uint8_t *buffer, uint16_t max_len,
                               uint32_t timeout_ms)
{
    RT_ASSERT(sys != RT_NULL);
    RT_ASSERT(bus_id < RS485_MAX_BUSES);

    rs485_bus_t *bus = &sys->bus[bus_id];

    if (!bus->initialized) return 0;

    /* Wait for data with timeout */
    rt_err_t ret = rt_sem_take(&bus->rx_sem, rt_tick_from_millisecond(timeout_ms));
    if (ret != RT_EOK) {
        return 0;
    }

    rt_mutex_take(&bus->lock, RT_WAITING_FOREVER);

    uint16_t len = (bus->rx_index < max_len) ? bus->rx_index : max_len;
    memcpy(buffer, bus->rx_buffer, len);
    bus->rx_index = 0;

    rt_mutex_release(&bus->lock);

    return len;
}

rt_err_t rs485_motor_send_command(rs485_motor_system_t *sys, const motor_command_t *cmd)
{
    RT_ASSERT(sys != RT_NULL);
    RT_ASSERT(cmd != RT_NULL);

    uint8_t bus_id = rs485_find_bus_for_motor(sys, cmd->id);
    if (bus_id == 0xFF) {
        LOG_W("Motor %d not found on any bus", cmd->id);
        return -RT_EINVAL;
    }

    rs485_bus_t *bus = &sys->bus[bus_id];
    uint16_t len;

    /* Encode command */
    if (custom_encode[bus_id] != NULL) {
        len = custom_encode[bus_id](cmd, bus->tx_buffer);
    } else {
        len = rs485_encode_generic(cmd, bus->tx_buffer);
    }

    /* Send */
    return rs485_bus_send_raw(sys, bus_id, bus->tx_buffer, len);
}

rt_err_t rs485_motor_send_commands(rs485_motor_system_t *sys,
                                    const motor_command_t *cmds, uint8_t count)
{
    rt_err_t ret = RT_EOK;

    for (uint8_t i = 0; i < count; i++) {
        rt_err_t r = rs485_motor_send_command(sys, &cmds[i]);
        if (r != RT_EOK) ret = r;
    }

    return ret;
}

rt_err_t rs485_motor_request_feedback(rs485_motor_system_t *sys, uint8_t motor_id)
{
    motor_command_t cmd = {0};
    cmd.id = motor_id;
    cmd.mode = 0x00;  /* Read request */

    rt_err_t ret = rs485_motor_send_command(sys, &cmd);
    if (ret != RT_EOK) return ret;

    /* Wait for response */
    uint8_t bus_id = rs485_find_bus_for_motor(sys, motor_id);
    rs485_bus_t *bus = &sys->bus[bus_id];

    uint8_t rx_buf[64];
    uint16_t rx_len = rs485_bus_receive_raw(sys, bus_id, rx_buf, sizeof(rx_buf),
                                            RS485_TIMEOUT_MS);

    if (rx_len > 0) {
        motor_feedback_t feedback;
        rt_err_t decode_ret;

        if (custom_decode[bus_id] != NULL) {
            decode_ret = custom_decode[bus_id](rx_buf, rx_len, &feedback);
        } else {
            decode_ret = rs485_decode_generic(rx_buf, rx_len, &feedback);
        }

        if (decode_ret == RT_EOK) {
            /* Update cached feedback */
            for (uint8_t i = 0; i < bus->config.motor_count; i++) {
                if (bus->feedback[i].id == feedback.id) {
                    memcpy(&bus->feedback[i], &feedback, sizeof(motor_feedback_t));
                    bus->rx_count++;
                    break;
                }
            }
        }
    }

    return RT_EOK;
}

rt_err_t rs485_motor_request_all_feedback(rs485_motor_system_t *sys)
{
    for (uint8_t b = 0; b < RS485_MAX_BUSES; b++) {
        if (!sys->bus[b].initialized) continue;

        for (uint8_t m = 0; m < sys->bus[b].config.motor_count; m++) {
            rs485_motor_request_feedback(sys, sys->bus[b].config.motor_ids[m]);
        }
    }

    return RT_EOK;
}

rt_err_t rs485_motor_get_feedback(rs485_motor_system_t *sys, uint8_t motor_id,
                                   motor_feedback_t *feedback)
{
    RT_ASSERT(sys != RT_NULL);
    RT_ASSERT(feedback != RT_NULL);

    uint8_t bus_id = rs485_find_bus_for_motor(sys, motor_id);
    if (bus_id == 0xFF) return -RT_EINVAL;

    rs485_bus_t *bus = &sys->bus[bus_id];

    rt_mutex_take(&bus->lock, RT_WAITING_FOREVER);

    for (uint8_t i = 0; i < bus->config.motor_count; i++) {
        if (bus->feedback[i].id == motor_id) {
            memcpy(feedback, &bus->feedback[i], sizeof(motor_feedback_t));
            rt_mutex_release(&bus->lock);
            return RT_EOK;
        }
    }

    rt_mutex_release(&bus->lock);
    return -RT_EINVAL;
}

rt_err_t rs485_motor_get_all_feedback(rs485_motor_system_t *sys,
                                       motor_feedback_t *feedback)
{
    RT_ASSERT(sys != RT_NULL);
    RT_ASSERT(feedback != RT_NULL);

    uint8_t idx = 0;

    for (uint8_t b = 0; b < RS485_MAX_BUSES; b++) {
        if (!sys->bus[b].initialized) continue;

        rs485_bus_t *bus = &sys->bus[b];
        rt_mutex_take(&bus->lock, RT_WAITING_FOREVER);

        for (uint8_t m = 0; m < bus->config.motor_count; m++) {
            memcpy(&feedback[idx++], &bus->feedback[m], sizeof(motor_feedback_t));
        }

        rt_mutex_release(&bus->lock);
    }

    return RT_EOK;
}

rt_err_t rs485_motor_set_position(rs485_motor_system_t *sys, uint8_t motor_id,
                                   int32_t position)
{
    motor_command_t cmd = {0};
    cmd.id = motor_id;
    cmd.mode = MOTOR_MODE_POSITION;
    cmd.target_position = position;

    return rs485_motor_send_command(sys, &cmd);
}

rt_err_t rs485_motor_set_velocity(rs485_motor_system_t *sys, uint8_t motor_id,
                                   int16_t velocity)
{
    motor_command_t cmd = {0};
    cmd.id = motor_id;
    cmd.mode = MOTOR_MODE_VELOCITY;
    cmd.target_velocity = velocity;

    return rs485_motor_send_command(sys, &cmd);
}

rt_err_t rs485_motor_set_torque(rs485_motor_system_t *sys, uint8_t motor_id,
                                 int16_t torque)
{
    motor_command_t cmd = {0};
    cmd.id = motor_id;
    cmd.mode = MOTOR_MODE_TORQUE;
    cmd.target_torque = torque;

    return rs485_motor_send_command(sys, &cmd);
}

rt_err_t rs485_motor_enable(rs485_motor_system_t *sys, uint8_t motor_id)
{
    motor_command_t cmd = {0};
    cmd.id = motor_id;
    cmd.mode = MOTOR_MODE_POSITION;
    cmd.flags = 0x01;  /* Enable flag */

    return rs485_motor_send_command(sys, &cmd);
}

rt_err_t rs485_motor_disable(rs485_motor_system_t *sys, uint8_t motor_id)
{
    motor_command_t cmd = {0};
    cmd.id = motor_id;
    cmd.mode = MOTOR_MODE_DISABLE;

    return rs485_motor_send_command(sys, &cmd);
}

rt_err_t rs485_motor_disable_all(rs485_motor_system_t *sys)
{
    for (uint8_t b = 0; b < RS485_MAX_BUSES; b++) {
        if (!sys->bus[b].initialized) continue;

        for (uint8_t m = 0; m < sys->bus[b].config.motor_count; m++) {
            rs485_motor_disable(sys, sys->bus[b].config.motor_ids[m]);
        }
    }

    return RT_EOK;
}

rt_err_t rs485_motor_ping(rs485_motor_system_t *sys, uint8_t motor_id)
{
    /* Send ping command */
    uint8_t ping_cmd[] = {0xFF, 0xFF, motor_id, 0x01, 0x00};
    uint16_t crc = rs485_crc16(ping_cmd, 5);
    ping_cmd[3] = crc & 0xFF;
    ping_cmd[4] = (crc >> 8) & 0xFF;

    uint8_t bus_id = rs485_find_bus_for_motor(sys, motor_id);
    if (bus_id == 0xFF) {
        /* Try all buses */
        for (uint8_t b = 0; b < RS485_MAX_BUSES; b++) {
            if (sys->bus[b].initialized) {
                rs485_bus_send_raw(sys, b, ping_cmd, sizeof(ping_cmd));
                uint8_t rx[16];
                if (rs485_bus_receive_raw(sys, b, rx, sizeof(rx), RS485_TIMEOUT_MS) > 0) {
                    return RT_EOK;
                }
            }
        }
        return -RT_ETIMEOUT;
    }

    rs485_bus_send_raw(sys, bus_id, ping_cmd, sizeof(ping_cmd));
    uint8_t rx[16];
    if (rs485_bus_receive_raw(sys, bus_id, rx, sizeof(rx), RS485_TIMEOUT_MS) > 0) {
        return RT_EOK;
    }

    return -RT_ETIMEOUT;
}

uint8_t rs485_motor_scan(rs485_motor_system_t *sys, uint8_t bus_id,
                         uint8_t *found_ids, uint8_t max_count)
{
    uint8_t count = 0;

    for (uint8_t id = 1; id <= 32 && count < max_count; id++) {
        uint8_t ping_cmd[] = {0xFF, 0xFF, id, 0x01, 0x00, 0x00, 0x00};
        uint16_t crc = rs485_crc16(ping_cmd, 5);
        ping_cmd[5] = crc & 0xFF;
        ping_cmd[6] = (crc >> 8) & 0xFF;

        rs485_bus_send_raw(sys, bus_id, ping_cmd, 7);

        uint8_t rx[16];
        if (rs485_bus_receive_raw(sys, bus_id, rx, sizeof(rx), 5) > 0) {
            found_ids[count++] = id;
            LOG_I("Found motor ID %d on bus %d", id, bus_id);
        }
    }

    return count;
}

void rs485_bus_get_stats(rs485_motor_system_t *sys, uint8_t bus_id,
                         uint32_t *tx_count, uint32_t *rx_count,
                         uint32_t *error_count)
{
    if (bus_id >= RS485_MAX_BUSES) return;

    rs485_bus_t *bus = &sys->bus[bus_id];

    if (tx_count) *tx_count = bus->tx_count;
    if (rx_count) *rx_count = bus->rx_count;
    if (error_count) *error_count = bus->error_count;
}

rt_err_t rs485_set_protocol_handlers(rs485_motor_system_t *sys, uint8_t bus_id,
                                     rs485_encode_func_t encode_func,
                                     rs485_decode_func_t decode_func)
{
    if (bus_id >= RS485_MAX_BUSES) return -RT_EINVAL;

    custom_encode[bus_id] = encode_func;
    custom_decode[bus_id] = decode_func;

    return RT_EOK;
}

void rs485_rx_callback(rs485_motor_system_t *sys, uint8_t bus_id, uint8_t data)
{
    if (bus_id >= RS485_MAX_BUSES) return;

    rs485_bus_t *bus = &sys->bus[bus_id];

    if (bus->rx_index < RS485_RX_BUFFER_SIZE) {
        bus->rx_buffer[bus->rx_index++] = data;
    }

    /* Signal that data is available */
    rt_sem_release(&bus->rx_sem);
}
