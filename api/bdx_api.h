/**
 * @file bdx_api.h
 * @brief BDX_SIm Real-Time Robot Control API
 * @author BDX_SIm Project
 * @version 2.0
 *
 * High-performance API for bipedal robot control optimized for:
 * - 600 Hz control loop (1.667 ms period)
 * - < 200 µs communication latency via UART @ 2Mbps
 * - 14 joint position setpoints from policy
 * - IMU + joint feedback for observations
 *
 * Architecture:
 * - Timer-driven 600 Hz interrupt for deterministic timing
 * - DMA-based UART communication (no CPU blocking)
 * - Double-buffered state for lock-free access
 * - Direct RS-485 motor control at 600 Hz
 */

#ifndef __BDX_API_H__
#define __BDX_API_H__

#include <rtthread.h>
#include <stdint.h>
#include <stdbool.h>

#include "bmi088.h"
#include "rs485_motor.h"
#include "bdx_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============== Configuration ============== */

#define BDX_CONTROL_FREQ_HZ         600         /* Main control loop frequency */
#define BDX_CONTROL_PERIOD_US       1667        /* = 1,000,000 / 600 */

#define BDX_NUM_JOINTS              14          /* Total actuated joints */
#define BDX_NUM_MOTORS_BUS1         6           /* Motors on RS-485 bus 1 */
#define BDX_NUM_MOTORS_BUS2         8           /* Motors on RS-485 bus 2 */

/* Thread configuration */
#define BDX_CONTROL_THREAD_STACK    4096
#define BDX_CONTROL_THREAD_PRIO     3           /* Very high priority */

/* DMA buffer alignment for STM32H7 (must be in D2 SRAM) */
#define BDX_DMA_BUFFER_SIZE         256
#define BDX_DMA_ALIGN               __attribute__((aligned(32)))

/* Timeout for Jetson communication */
#define BDX_COMM_TIMEOUT_MS         50          /* 50ms = 30 missed frames */

/* ============== Joint Mapping (BDX_SIm Standard) ============== */

typedef enum {
    /* Left leg */
    JOINT_LEFT_HIP_YAW = 0,
    JOINT_LEFT_HIP_ROLL = 1,
    JOINT_LEFT_HIP_PITCH = 2,
    JOINT_LEFT_KNEE = 3,
    JOINT_LEFT_ANKLE = 4,

    /* Neck and head */
    JOINT_NECK_PITCH = 5,
    JOINT_HEAD_PITCH = 6,
    JOINT_HEAD_YAW = 7,
    JOINT_HEAD_ROLL = 8,

    /* Right leg */
    JOINT_RIGHT_HIP_YAW = 9,
    JOINT_RIGHT_HIP_ROLL = 10,
    JOINT_RIGHT_HIP_PITCH = 11,
    JOINT_RIGHT_KNEE = 12,
    JOINT_RIGHT_ANKLE = 13
} bdx_joint_id_t;

/* ============== System States ============== */

typedef enum {
    BDX_SYS_UNINIT = 0,             /* Not initialized */
    BDX_SYS_IDLE,                   /* Initialized, waiting for Jetson */
    BDX_SYS_ACTIVE,                 /* Running, motors enabled */
    BDX_SYS_ESTOP,                  /* Emergency stop */
    BDX_SYS_ERROR                   /* Error state */
} bdx_sys_state_t;

/* ============== Data Structures ============== */

/**
 * @brief Robot state (double-buffered for lock-free access)
 */
typedef struct {
    /* IMU data */
    float imu_accel[3];             /* Linear acceleration (m/s²) */
    float imu_gyro[3];              /* Angular velocity (rad/s) */
    float imu_temperature;          /* Temperature (°C) */

    /* Joint feedback */
    float joint_positions[BDX_NUM_JOINTS];      /* Current positions (rad) */
    float joint_velocities[BDX_NUM_JOINTS];     /* Current velocities (rad/s) */
    float joint_currents[BDX_NUM_JOINTS];       /* Motor currents (mA) */

    /* Foot contacts (from force sensors or current threshold) */
    bool contact_left;
    bool contact_right;

    /* Timing */
    uint32_t timestamp_us;          /* Microsecond timestamp */
    uint32_t sequence;              /* Frame sequence number */
} bdx_robot_state_t;

/**
 * @brief Joint setpoints from policy
 */
typedef struct {
    float positions[BDX_NUM_JOINTS];    /* Target positions (rad) */
    uint8_t flags;                       /* Control flags */
    uint16_t sequence;                   /* Command sequence */
    uint32_t timestamp_us;               /* When received */
} bdx_setpoints_t;

/**
 * @brief Motor mapping configuration
 */
typedef struct {
    uint8_t bus_id;                 /* RS-485 bus (0 or 1) */
    uint8_t motor_id;               /* Motor ID on bus */
    float gear_ratio;               /* Gear ratio for position conversion */
    float offset;                   /* Zero position offset (rad) */
    bool inverted;                  /* Invert direction */
} bdx_motor_map_t;

/**
 * @brief System configuration
 */
typedef struct {
    /* UART for Jetson communication */
    UART_HandleTypeDef *huart_jetson;

    /* IMU */
    SPI_HandleTypeDef *hspi_imu;
    GPIO_TypeDef *imu_acc_cs_port;
    uint16_t imu_acc_cs_pin;
    GPIO_TypeDef *imu_gyro_cs_port;
    uint16_t imu_gyro_cs_pin;

    /* RS-485 buses */
    UART_HandleTypeDef *huart_rs485_1;
    GPIO_TypeDef *rs485_1_de_port;
    uint16_t rs485_1_de_pin;

    UART_HandleTypeDef *huart_rs485_2;
    GPIO_TypeDef *rs485_2_de_port;
    uint16_t rs485_2_de_pin;

    /* Motor mapping */
    bdx_motor_map_t motor_map[BDX_NUM_JOINTS];

    /* Control timer (for 600 Hz interrupt) */
    TIM_HandleTypeDef *htim_control;

    /* Contact detection thresholds */
    float contact_current_threshold;    /* mA threshold for contact detection */
} bdx_config_t;

/**
 * @brief System statistics
 */
typedef struct {
    uint32_t control_loops;         /* Total control loop iterations */
    uint32_t comm_rx_frames;        /* Frames received from Jetson */
    uint32_t comm_tx_frames;        /* Frames sent to Jetson */
    uint32_t comm_errors;           /* Communication errors */
    uint32_t motor_errors;          /* Motor communication errors */
    uint32_t imu_errors;            /* IMU read errors */

    uint32_t max_loop_time_us;      /* Maximum loop execution time */
    uint32_t avg_loop_time_us;      /* Average loop time */
    uint32_t max_latency_us;        /* Maximum command latency */

    uint32_t missed_frames;         /* Missed Jetson frames */
    uint32_t last_jetson_time_us;   /* Last Jetson communication */
} bdx_stats_t;

/**
 * @brief Main system handle
 */
typedef struct {
    /* Configuration */
    bdx_config_t config;

    /* Hardware drivers */
    bmi088_device_t imu;
    rs485_motor_system_t motors;
    bdx_protocol_ctx_t protocol;

    /* DMA buffers (must be in non-cached SRAM for H7) */
    uint8_t dma_tx_buffer[BDX_DMA_BUFFER_SIZE] BDX_DMA_ALIGN;
    uint8_t dma_rx_buffer[BDX_DMA_BUFFER_SIZE] BDX_DMA_ALIGN;

    /* Double-buffered state (lock-free) */
    bdx_robot_state_t state[2];
    volatile uint8_t state_write_idx;
    volatile uint8_t state_read_idx;

    /* Setpoints from Jetson */
    bdx_setpoints_t setpoints;
    volatile bool new_setpoints;

    /* System state */
    bdx_sys_state_t sys_state;
    bdx_error_code_t last_error;

    /* Timing */
    uint32_t loop_start_time_us;
    uint32_t last_comm_time_us;

    /* Statistics */
    bdx_stats_t stats;

    /* Control thread */
    struct rt_thread control_thread;
    uint8_t control_stack[BDX_CONTROL_THREAD_STACK];
    volatile bool running;

    bool initialized;
} bdx_system_t;

/* ============== API Functions ============== */

/**
 * @brief Get default configuration
 * @param config Configuration to fill with defaults
 */
void bdx_get_default_config(bdx_config_t *config);

/**
 * @brief Initialize the BDX system
 * @param sys System handle
 * @param config System configuration
 * @return RT_EOK on success
 */
rt_err_t bdx_init(bdx_system_t *sys, const bdx_config_t *config);

/**
 * @brief Start the 600 Hz control loop
 * @param sys System handle
 * @return RT_EOK on success
 */
rt_err_t bdx_start(bdx_system_t *sys);

/**
 * @brief Stop the control loop
 * @param sys System handle
 * @return RT_EOK on success
 */
rt_err_t bdx_stop(bdx_system_t *sys);

/**
 * @brief Emergency stop - immediately disable all motors
 * @param sys System handle
 */
void bdx_estop(bdx_system_t *sys);

/**
 * @brief Clear emergency stop and return to idle
 * @param sys System handle
 * @return RT_EOK on success
 */
rt_err_t bdx_clear_estop(bdx_system_t *sys);

/**
 * @brief Get current robot state (lock-free)
 * @param sys System handle
 * @param state Output state structure
 */
void bdx_get_state(bdx_system_t *sys, bdx_robot_state_t *state);

/**
 * @brief Get system statistics
 * @param sys System handle
 * @param stats Output statistics
 */
void bdx_get_stats(bdx_system_t *sys, bdx_stats_t *stats);

/**
 * @brief Get current system state
 * @param sys System handle
 * @return Current state
 */
bdx_sys_state_t bdx_get_sys_state(bdx_system_t *sys);

/**
 * @brief Check if Jetson communication is active
 * @param sys System handle
 * @return true if communication is active (< timeout)
 */
bool bdx_is_comm_active(bdx_system_t *sys);

/* ============== Callbacks (implement in user code) ============== */

/**
 * @brief Timer interrupt callback (call from TIM IRQ)
 * @param sys System handle
 *
 * This triggers the 600 Hz control loop.
 */
void bdx_timer_callback(bdx_system_t *sys);

/**
 * @brief UART RX complete callback (call from UART IRQ)
 * @param sys System handle
 */
void bdx_uart_rx_callback(bdx_system_t *sys);

/**
 * @brief UART TX complete callback (call from UART IRQ)
 * @param sys System handle
 */
void bdx_uart_tx_callback(bdx_system_t *sys);

/* ============== Internal Functions (for advanced use) ============== */

/**
 * @brief Single control loop iteration (called from timer ISR)
 * @param sys System handle
 */
void bdx_control_loop(bdx_system_t *sys);

/**
 * @brief Update IMU readings
 * @param sys System handle
 */
void bdx_update_imu(bdx_system_t *sys);

/**
 * @brief Update motor feedback
 * @param sys System handle
 */
void bdx_update_motors(bdx_system_t *sys);

/**
 * @brief Send motor commands
 * @param sys System handle
 */
void bdx_send_motor_commands(bdx_system_t *sys);

/**
 * @brief Send state to Jetson
 * @param sys System handle
 */
void bdx_send_state_to_jetson(bdx_system_t *sys);

/* ============== MSH Commands ============== */

#ifdef RT_USING_FINSH
void bdx_register_msh_commands(bdx_system_t *sys);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __BDX_API_H__ */
