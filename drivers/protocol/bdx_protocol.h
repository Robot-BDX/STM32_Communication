/**
 * @file bdx_protocol.h
 * @brief BDX_SIm Communication Protocol - Optimized for 600Hz Real-Time Control
 * @author BDX_SIm Project
 * @version 2.0
 *
 * Ultra-low latency binary protocol for UART communication at 2Mbps.
 * Designed for real-time robot control with < 200µs latency.
 *
 * Protocol Features:
 * - Fixed-size frames for deterministic timing
 * - CRC-8 for fast error detection
 * - DMA-friendly aligned structures
 * - 600 Hz bidirectional communication
 *
 * Data Flow:
 * - Jetson → STM32: Joint setpoints (14 floats) @ 600 Hz
 * - STM32 → Jetson: IMU + Joint feedback @ 600 Hz
 */

#ifndef __BDX_PROTOCOL_H__
#define __BDX_PROTOCOL_H__

#include <rtthread.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============== Protocol Configuration ============== */

#define BDX_UART_BAUDRATE           2000000     /* 2 Mbps */
#define BDX_CONTROL_FREQ_HZ         600         /* Control loop frequency */
#define BDX_POLICY_FREQ_HZ          50          /* Policy update frequency */

#define BDX_NUM_JOINTS              14          /* Number of actuated joints */
#define BDX_IMU_DATA_SIZE           6           /* ax, ay, az, gx, gy, gz */

/* Frame markers */
#define BDX_FRAME_START             0xAA
#define BDX_FRAME_CMD               0x01        /* Command frame (Jetson → STM32) */
#define BDX_FRAME_STATE             0x02        /* State frame (STM32 → Jetson) */
#define BDX_FRAME_CONFIG            0x03        /* Configuration frame */
#define BDX_FRAME_ACK               0x04        /* Acknowledgment */

/* ============== Joint Order (BDX_SIm Standard) ============== */
/*
 * Index  Joint Name           Actuator Type
 * -----  -------------------  -------------
 *   0    left_hip_yaw         Unitree Go1
 *   1    left_hip_roll        Unitree A1
 *   2    left_hip_pitch       Unitree A1
 *   3    left_knee            Unitree A1
 *   4    left_ankle           Unitree Go1
 *   5    neck_pitch           Unitree Go1
 *   6    head_pitch           Dynamixel XH540
 *   7    head_yaw             Dynamixel XH540
 *   8    head_roll            Dynamixel XH540
 *   9    right_hip_yaw        Unitree Go1
 *  10    right_hip_roll       Unitree A1
 *  11    right_hip_pitch      Unitree A1
 *  12    right_knee           Unitree A1
 *  13    right_ankle          Unitree Go1
 */

/* ============== Frame Structures (Packed, DMA-aligned) ============== */

#pragma pack(push, 1)

/**
 * @brief Command frame: Jetson → STM32 (60 bytes)
 *
 * Sent at 600 Hz from Jetson with joint position setpoints.
 * At 2Mbps: 60 bytes × 8 bits / 2,000,000 = 240 µs transmission time
 */
typedef struct {
    uint8_t start;                              /* 0xAA */
    uint8_t type;                               /* BDX_FRAME_CMD (0x01) */
    uint16_t sequence;                          /* Sequence number (wraps at 65535) */
    float joint_setpoints[BDX_NUM_JOINTS];      /* 14 joint positions (rad) */
    uint8_t flags;                              /* Bit 0: enable, Bit 1: estop */
    uint8_t crc;                                /* CRC-8 */
} bdx_cmd_frame_t;                              /* Total: 60 bytes */

/**
 * @brief State frame: STM32 → Jetson (136 bytes)
 *
 * Sent at 600 Hz with complete robot state.
 * At 2Mbps: 136 bytes × 8 bits / 2,000,000 = 544 µs transmission time
 */
typedef struct {
    uint8_t start;                              /* 0xAA */
    uint8_t type;                               /* BDX_FRAME_STATE (0x02) */
    uint16_t sequence;                          /* Sequence number */
    uint32_t timestamp_us;                      /* Microsecond timestamp */

    /* IMU Data (24 bytes) */
    float imu_accel[3];                         /* Linear acceleration (m/s²) */
    float imu_gyro[3];                          /* Angular velocity (rad/s) */

    /* Joint Feedback (112 bytes) */
    float joint_positions[BDX_NUM_JOINTS];      /* Current positions (rad) */
    float joint_velocities[BDX_NUM_JOINTS];     /* Current velocities (rad/s) */

    /* Status (2 bytes) */
    uint8_t status;                             /* Bit 0-1: state, Bit 2: imu_ok, Bit 3-4: contacts */
    uint8_t error_code;                         /* Error code if any */

    uint8_t crc;                                /* CRC-8 */
} bdx_state_frame_t;                            /* Total: 136 bytes */

/**
 * @brief Configuration frame (bidirectional)
 */
typedef struct {
    uint8_t start;                              /* 0xAA */
    uint8_t type;                               /* BDX_FRAME_CONFIG (0x03) */
    uint16_t sequence;
    uint8_t config_type;                        /* 0: query, 1: set frequency, 2: set gains */
    uint8_t data[16];                           /* Configuration data */
    uint8_t crc;
} bdx_config_frame_t;                           /* Total: 22 bytes */

/**
 * @brief Minimal acknowledgment frame
 */
typedef struct {
    uint8_t start;                              /* 0xAA */
    uint8_t type;                               /* BDX_FRAME_ACK (0x04) */
    uint16_t sequence;                          /* Sequence being acknowledged */
    uint8_t status;                             /* 0: OK, >0: error code */
    uint8_t crc;
} bdx_ack_frame_t;                              /* Total: 6 bytes */

#pragma pack(pop)

/* Frame sizes for DMA configuration */
#define BDX_CMD_FRAME_SIZE          sizeof(bdx_cmd_frame_t)     /* 60 bytes */
#define BDX_STATE_FRAME_SIZE        sizeof(bdx_state_frame_t)   /* 136 bytes */
#define BDX_CONFIG_FRAME_SIZE       sizeof(bdx_config_frame_t)  /* 22 bytes */
#define BDX_ACK_FRAME_SIZE          sizeof(bdx_ack_frame_t)     /* 6 bytes */

/* ============== Status Flags ============== */

/* Command flags */
#define BDX_FLAG_ENABLE             (1 << 0)    /* Motors enabled */
#define BDX_FLAG_ESTOP              (1 << 1)    /* Emergency stop */
#define BDX_FLAG_CALIBRATE          (1 << 2)    /* Start calibration */

/* State status bits */
#define BDX_STATUS_STATE_MASK       0x03        /* Bits 0-1: system state */
#define BDX_STATUS_IMU_OK           (1 << 2)    /* IMU operational */
#define BDX_STATUS_CONTACT_LEFT     (1 << 3)    /* Left foot contact */
#define BDX_STATUS_CONTACT_RIGHT    (1 << 4)    /* Right foot contact */
#define BDX_STATUS_MOTORS_OK        (1 << 5)    /* All motors responding */

/* System states (2 bits) */
typedef enum {
    BDX_STATE_IDLE = 0,             /* Motors disabled */
    BDX_STATE_ACTIVE = 1,           /* Motors enabled, running */
    BDX_STATE_ESTOP = 2,            /* Emergency stop */
    BDX_STATE_ERROR = 3             /* Error state */
} bdx_system_state_t;

/* Error codes */
typedef enum {
    BDX_ERR_NONE = 0,
    BDX_ERR_CRC = 1,
    BDX_ERR_TIMEOUT = 2,
    BDX_ERR_INVALID_FRAME = 3,
    BDX_ERR_IMU_FAULT = 4,
    BDX_ERR_MOTOR_FAULT = 5,
    BDX_ERR_COMM_FAULT = 6,
    BDX_ERR_ESTOP = 7
} bdx_error_code_t;

/* ============== Protocol Context ============== */

/**
 * @brief Protocol statistics
 */
typedef struct {
    uint32_t tx_frames;
    uint32_t rx_frames;
    uint32_t rx_errors;
    uint32_t crc_errors;
    uint32_t timeout_errors;
    uint32_t sequence_errors;
    uint16_t last_rx_seq;
    uint16_t last_tx_seq;
    uint32_t max_latency_us;
    uint32_t avg_latency_us;
} bdx_protocol_stats_t;

/**
 * @brief Protocol context
 */
typedef struct {
    /* UART handle */
    UART_HandleTypeDef *huart;

    /* DMA buffers (must be in non-cached memory for H7) */
    uint8_t *tx_buffer;
    uint8_t *rx_buffer;

    /* Sequence numbers */
    uint16_t tx_seq;
    uint16_t rx_seq;

    /* State */
    bdx_state_frame_t current_state;
    bdx_cmd_frame_t last_cmd;
    bool cmd_received;

    /* Timing */
    uint32_t last_rx_time_us;
    uint32_t last_tx_time_us;

    /* Statistics */
    bdx_protocol_stats_t stats;

    /* Callbacks */
    void (*on_cmd_received)(const bdx_cmd_frame_t *cmd);
    void (*on_error)(bdx_error_code_t error);

    /* Synchronization */
    struct rt_semaphore rx_sem;
    struct rt_mutex tx_lock;

    bool initialized;
} bdx_protocol_ctx_t;

/* ============== API Functions ============== */

/**
 * @brief Calculate CRC-8 (polynomial 0x07)
 * @param data Input data
 * @param len Data length
 * @return CRC-8 value
 */
uint8_t bdx_crc8(const uint8_t *data, uint16_t len);

/**
 * @brief Initialize protocol context
 * @param ctx Protocol context
 * @param huart UART handle (configured for 2Mbps)
 * @param tx_buffer TX DMA buffer (must be in SRAM, aligned)
 * @param rx_buffer RX DMA buffer (must be in SRAM, aligned)
 * @return RT_EOK on success
 */
rt_err_t bdx_protocol_init(bdx_protocol_ctx_t *ctx,
                           UART_HandleTypeDef *huart,
                           uint8_t *tx_buffer,
                           uint8_t *rx_buffer);

/**
 * @brief Start DMA reception (call once after init)
 * @param ctx Protocol context
 * @return RT_EOK on success
 */
rt_err_t bdx_protocol_start_rx(bdx_protocol_ctx_t *ctx);

/**
 * @brief Send state frame (called from 600Hz loop)
 * @param ctx Protocol context
 * @param state State data to send
 * @return RT_EOK on success
 */
rt_err_t bdx_protocol_send_state(bdx_protocol_ctx_t *ctx,
                                  const bdx_state_frame_t *state);

/**
 * @brief Send acknowledgment
 * @param ctx Protocol context
 * @param seq Sequence to acknowledge
 * @param status Status code
 * @return RT_EOK on success
 */
rt_err_t bdx_protocol_send_ack(bdx_protocol_ctx_t *ctx,
                                uint16_t seq, uint8_t status);

/**
 * @brief Check if new command available
 * @param ctx Protocol context
 * @return true if command available
 */
bool bdx_protocol_cmd_available(bdx_protocol_ctx_t *ctx);

/**
 * @brief Get last received command
 * @param ctx Protocol context
 * @param cmd Output command structure
 * @return RT_EOK on success
 */
rt_err_t bdx_protocol_get_cmd(bdx_protocol_ctx_t *ctx, bdx_cmd_frame_t *cmd);

/**
 * @brief Process received data (call from DMA complete callback)
 * @param ctx Protocol context
 * @param data Received data
 * @param len Data length
 */
void bdx_protocol_process_rx(bdx_protocol_ctx_t *ctx,
                              const uint8_t *data, uint16_t len);

/**
 * @brief UART RX complete callback (call from HAL callback)
 * @param ctx Protocol context
 */
void bdx_protocol_rx_complete_callback(bdx_protocol_ctx_t *ctx);

/**
 * @brief UART TX complete callback (call from HAL callback)
 * @param ctx Protocol context
 */
void bdx_protocol_tx_complete_callback(bdx_protocol_ctx_t *ctx);

/**
 * @brief Get protocol statistics
 * @param ctx Protocol context
 * @param stats Output statistics
 */
void bdx_protocol_get_stats(bdx_protocol_ctx_t *ctx, bdx_protocol_stats_t *stats);

/**
 * @brief Reset protocol state
 * @param ctx Protocol context
 */
void bdx_protocol_reset(bdx_protocol_ctx_t *ctx);

/**
 * @brief Get current timestamp in microseconds
 * @return Timestamp
 */
uint32_t bdx_get_timestamp_us(void);

#ifdef __cplusplus
}
#endif

#endif /* __BDX_PROTOCOL_H__ */
