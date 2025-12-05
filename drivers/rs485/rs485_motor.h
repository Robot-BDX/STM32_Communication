/**
 * @file rs485_motor.h
 * @brief RS-485 Motor Driver for STM32H723 DM-MC02
 * @author BDX_SIm Project
 * @version 1.0
 *
 * Generic RS-485 motor communication driver supporting multiple motor protocols.
 * Supports 2 RS-485 buses (USART2, USART3) for up to 11 motors total.
 */

#ifndef __RS485_MOTOR_H__
#define __RS485_MOTOR_H__

#include <rtthread.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============== Configuration ============== */

#define RS485_MAX_MOTORS_PER_BUS    8
#define RS485_MAX_BUSES             2
#define RS485_MAX_MOTORS_TOTAL      (RS485_MAX_MOTORS_PER_BUS * RS485_MAX_BUSES)

#define RS485_RX_BUFFER_SIZE        256
#define RS485_TX_BUFFER_SIZE        64

#define RS485_DEFAULT_BAUDRATE      115200
#define RS485_TIMEOUT_MS            10

/* ============== Motor Protocol Types ============== */

typedef enum {
    MOTOR_PROTOCOL_GENERIC = 0,     /* Generic RS-485 protocol */
    MOTOR_PROTOCOL_FEETECH,         /* Feetech/SCS servo protocol */
    MOTOR_PROTOCOL_DYNAMIXEL,       /* Dynamixel protocol */
    MOTOR_PROTOCOL_MODBUS_RTU,      /* Modbus RTU */
    MOTOR_PROTOCOL_CUSTOM           /* Custom protocol */
} motor_protocol_t;

/* ============== Motor State ============== */

typedef enum {
    MOTOR_STATE_OFFLINE = 0,
    MOTOR_STATE_IDLE,
    MOTOR_STATE_RUNNING,
    MOTOR_STATE_ERROR,
    MOTOR_STATE_CALIBRATING
} motor_state_t;

/* ============== Data Structures ============== */

/**
 * @brief Motor feedback data
 */
typedef struct {
    uint8_t id;                     /* Motor ID (1-11) */
    motor_state_t state;            /* Current state */
    int32_t position;               /* Current position (encoder ticks or degrees*100) */
    int16_t velocity;               /* Current velocity (RPM or deg/s) */
    int16_t current;                /* Current draw (mA) */
    int16_t torque;                 /* Torque feedback */
    int8_t temperature;             /* Temperature (°C) */
    uint8_t voltage;                /* Voltage (0.1V units) */
    uint8_t error_code;             /* Error code if any */
    uint32_t timestamp;             /* RT-Thread tick when updated */
} motor_feedback_t;

/**
 * @brief Motor command
 */
typedef struct {
    uint8_t id;                     /* Motor ID */
    int32_t target_position;        /* Target position */
    int16_t target_velocity;        /* Target velocity limit */
    int16_t target_torque;          /* Target torque/current limit */
    uint8_t mode;                   /* Control mode */
    uint8_t flags;                  /* Command flags */
} motor_command_t;

/**
 * @brief Control modes
 */
typedef enum {
    MOTOR_MODE_POSITION = 0,        /* Position control */
    MOTOR_MODE_VELOCITY,            /* Velocity control */
    MOTOR_MODE_TORQUE,              /* Torque/current control */
    MOTOR_MODE_PWM,                 /* Direct PWM */
    MOTOR_MODE_DISABLE              /* Motor disabled */
} motor_control_mode_t;

/**
 * @brief RS-485 Bus configuration
 */
typedef struct {
    UART_HandleTypeDef *huart;      /* UART handle */
    GPIO_TypeDef *de_port;          /* DE (Driver Enable) GPIO port */
    uint16_t de_pin;                /* DE pin */
    uint32_t baudrate;              /* Baudrate */
    motor_protocol_t protocol;       /* Communication protocol */
    uint8_t motor_count;            /* Number of motors on this bus */
    uint8_t motor_ids[RS485_MAX_MOTORS_PER_BUS];  /* Motor IDs on this bus */
} rs485_bus_config_t;

/**
 * @brief RS-485 Bus handle
 */
typedef struct {
    rs485_bus_config_t config;
    uint8_t rx_buffer[RS485_RX_BUFFER_SIZE];
    uint8_t tx_buffer[RS485_TX_BUFFER_SIZE];
    uint16_t rx_index;
    motor_feedback_t feedback[RS485_MAX_MOTORS_PER_BUS];
    struct rt_mutex lock;
    struct rt_semaphore rx_sem;
    bool initialized;
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t error_count;
} rs485_bus_t;

/**
 * @brief Complete motor system handle
 */
typedef struct {
    rs485_bus_t bus[RS485_MAX_BUSES];
    motor_feedback_t all_feedback[RS485_MAX_MOTORS_TOTAL];
    uint8_t total_motors;
    bool initialized;
    struct rt_timer poll_timer;
} rs485_motor_system_t;

/* ============== API Functions ============== */

/**
 * @brief Initialize the RS-485 motor system
 * @param sys System handle
 * @return RT_EOK on success
 */
rt_err_t rs485_motor_init(rs485_motor_system_t *sys);

/**
 * @brief Configure a RS-485 bus
 * @param sys System handle
 * @param bus_id Bus ID (0 or 1)
 * @param config Bus configuration
 * @return RT_EOK on success
 */
rt_err_t rs485_bus_configure(rs485_motor_system_t *sys, uint8_t bus_id,
                             const rs485_bus_config_t *config);

/**
 * @brief Send command to a single motor
 * @param sys System handle
 * @param cmd Motor command
 * @return RT_EOK on success
 */
rt_err_t rs485_motor_send_command(rs485_motor_system_t *sys, const motor_command_t *cmd);

/**
 * @brief Send commands to multiple motors (synchronized)
 * @param sys System handle
 * @param cmds Array of commands
 * @param count Number of commands
 * @return RT_EOK on success
 */
rt_err_t rs485_motor_send_commands(rs485_motor_system_t *sys,
                                    const motor_command_t *cmds, uint8_t count);

/**
 * @brief Request feedback from a motor
 * @param sys System handle
 * @param motor_id Motor ID
 * @return RT_EOK on success
 */
rt_err_t rs485_motor_request_feedback(rs485_motor_system_t *sys, uint8_t motor_id);

/**
 * @brief Request feedback from all motors
 * @param sys System handle
 * @return RT_EOK on success
 */
rt_err_t rs485_motor_request_all_feedback(rs485_motor_system_t *sys);

/**
 * @brief Get cached feedback for a motor
 * @param sys System handle
 * @param motor_id Motor ID
 * @param feedback Output feedback structure
 * @return RT_EOK on success
 */
rt_err_t rs485_motor_get_feedback(rs485_motor_system_t *sys, uint8_t motor_id,
                                   motor_feedback_t *feedback);

/**
 * @brief Get feedback for all motors
 * @param sys System handle
 * @param feedback Array of feedback structures (size = total_motors)
 * @return RT_EOK on success
 */
rt_err_t rs485_motor_get_all_feedback(rs485_motor_system_t *sys,
                                       motor_feedback_t *feedback);

/**
 * @brief Set motor position
 * @param sys System handle
 * @param motor_id Motor ID
 * @param position Target position
 * @return RT_EOK on success
 */
rt_err_t rs485_motor_set_position(rs485_motor_system_t *sys, uint8_t motor_id,
                                   int32_t position);

/**
 * @brief Set motor velocity
 * @param sys System handle
 * @param motor_id Motor ID
 * @param velocity Target velocity
 * @return RT_EOK on success
 */
rt_err_t rs485_motor_set_velocity(rs485_motor_system_t *sys, uint8_t motor_id,
                                   int16_t velocity);

/**
 * @brief Set motor torque/current
 * @param sys System handle
 * @param motor_id Motor ID
 * @param torque Target torque/current
 * @return RT_EOK on success
 */
rt_err_t rs485_motor_set_torque(rs485_motor_system_t *sys, uint8_t motor_id,
                                 int16_t torque);

/**
 * @brief Enable motor
 * @param sys System handle
 * @param motor_id Motor ID
 * @return RT_EOK on success
 */
rt_err_t rs485_motor_enable(rs485_motor_system_t *sys, uint8_t motor_id);

/**
 * @brief Disable motor
 * @param sys System handle
 * @param motor_id Motor ID
 * @return RT_EOK on success
 */
rt_err_t rs485_motor_disable(rs485_motor_system_t *sys, uint8_t motor_id);

/**
 * @brief Disable all motors (emergency stop)
 * @param sys System handle
 * @return RT_EOK on success
 */
rt_err_t rs485_motor_disable_all(rs485_motor_system_t *sys);

/**
 * @brief Ping a motor
 * @param sys System handle
 * @param motor_id Motor ID
 * @return RT_EOK if motor responds
 */
rt_err_t rs485_motor_ping(rs485_motor_system_t *sys, uint8_t motor_id);

/**
 * @brief Scan bus for motors
 * @param sys System handle
 * @param bus_id Bus ID
 * @param found_ids Array to store found motor IDs
 * @param max_count Maximum number to find
 * @return Number of motors found
 */
uint8_t rs485_motor_scan(rs485_motor_system_t *sys, uint8_t bus_id,
                         uint8_t *found_ids, uint8_t max_count);

/**
 * @brief Get bus statistics
 * @param sys System handle
 * @param bus_id Bus ID
 * @param tx_count Output TX packet count
 * @param rx_count Output RX packet count
 * @param error_count Output error count
 */
void rs485_bus_get_stats(rs485_motor_system_t *sys, uint8_t bus_id,
                         uint32_t *tx_count, uint32_t *rx_count,
                         uint32_t *error_count);

/* ============== Protocol-Specific Functions ============== */

/**
 * @brief Set custom protocol handlers
 * @param sys System handle
 * @param bus_id Bus ID
 * @param encode_func Custom encode function
 * @param decode_func Custom decode function
 */
typedef uint16_t (*rs485_encode_func_t)(const motor_command_t *cmd, uint8_t *buffer);
typedef rt_err_t (*rs485_decode_func_t)(const uint8_t *buffer, uint16_t len,
                                        motor_feedback_t *feedback);

rt_err_t rs485_set_protocol_handlers(rs485_motor_system_t *sys, uint8_t bus_id,
                                     rs485_encode_func_t encode_func,
                                     rs485_decode_func_t decode_func);

/* ============== Low-Level Functions ============== */

/**
 * @brief Send raw data on bus
 * @param sys System handle
 * @param bus_id Bus ID
 * @param data Data to send
 * @param len Data length
 * @return RT_EOK on success
 */
rt_err_t rs485_bus_send_raw(rs485_motor_system_t *sys, uint8_t bus_id,
                            const uint8_t *data, uint16_t len);

/**
 * @brief Receive raw data from bus
 * @param sys System handle
 * @param bus_id Bus ID
 * @param buffer Receive buffer
 * @param max_len Maximum length
 * @param timeout_ms Timeout in milliseconds
 * @return Number of bytes received
 */
uint16_t rs485_bus_receive_raw(rs485_motor_system_t *sys, uint8_t bus_id,
                               uint8_t *buffer, uint16_t max_len,
                               uint32_t timeout_ms);

/**
 * @brief Calculate CRC-16 (Modbus)
 * @param data Input data
 * @param len Data length
 * @return CRC-16 value
 */
uint16_t rs485_crc16(const uint8_t *data, uint16_t len);

/**
 * @brief UART RX callback (call from interrupt)
 * @param sys System handle
 * @param bus_id Bus ID
 * @param data Received byte
 */
void rs485_rx_callback(rs485_motor_system_t *sys, uint8_t bus_id, uint8_t data);

#ifdef __cplusplus
}
#endif

#endif /* __RS485_MOTOR_H__ */
