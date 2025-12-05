/**
 * @file bdx_api.c
 * @brief BDX_SIm Real-Time Robot Control API Implementation
 * @version 2.0
 *
 * 600 Hz control loop implementation optimized for bipedal robot control.
 * Features:
 * - Timer-driven 600 Hz interrupt for deterministic timing
 * - DMA-based UART communication with Jetson (< 200 µs latency)
 * - Double-buffered state for lock-free access
 * - Direct RS-485 motor control
 */

#include "bdx_api.h"
#include "stm32h7xx_hal.h"
#include <string.h>

#define DBG_TAG "bdx_api"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* ============== External References ============== */

extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart1;   /* Jetson communication */
extern UART_HandleTypeDef huart2;   /* RS-485 bus 1 */
extern UART_HandleTypeDef huart3;   /* RS-485 bus 2 */
extern TIM_HandleTypeDef htim6;     /* 600 Hz timer */

/* ============== Global Instance ============== */

static bdx_system_t *g_bdx_sys = RT_NULL;

/* ============== Motor Mapping (BDX_SIm Standard) ============== */

/**
 * Default motor mapping for BDX_SIm robot:
 * - Bus 0: Left leg (5 motors) + Neck (1 motor) = 6 motors
 * - Bus 1: Head (3 motors) + Right leg (5 motors) = 8 motors
 */
static const bdx_motor_map_t default_motor_map[BDX_NUM_JOINTS] = {
    /* Left leg - Bus 0 */
    [JOINT_LEFT_HIP_YAW]   = { .bus_id = 0, .motor_id = 1, .gear_ratio = 1.0f, .offset = 0.0f, .inverted = false },
    [JOINT_LEFT_HIP_ROLL]  = { .bus_id = 0, .motor_id = 2, .gear_ratio = 1.0f, .offset = 0.0f, .inverted = false },
    [JOINT_LEFT_HIP_PITCH] = { .bus_id = 0, .motor_id = 3, .gear_ratio = 1.0f, .offset = 0.0f, .inverted = false },
    [JOINT_LEFT_KNEE]      = { .bus_id = 0, .motor_id = 4, .gear_ratio = 1.0f, .offset = 0.0f, .inverted = false },
    [JOINT_LEFT_ANKLE]     = { .bus_id = 0, .motor_id = 5, .gear_ratio = 1.0f, .offset = 0.0f, .inverted = false },

    /* Neck - Bus 0 */
    [JOINT_NECK_PITCH]     = { .bus_id = 0, .motor_id = 6, .gear_ratio = 1.0f, .offset = 0.0f, .inverted = false },

    /* Head - Bus 1 */
    [JOINT_HEAD_PITCH]     = { .bus_id = 1, .motor_id = 1, .gear_ratio = 1.0f, .offset = 0.0f, .inverted = false },
    [JOINT_HEAD_YAW]       = { .bus_id = 1, .motor_id = 2, .gear_ratio = 1.0f, .offset = 0.0f, .inverted = false },
    [JOINT_HEAD_ROLL]      = { .bus_id = 1, .motor_id = 3, .gear_ratio = 1.0f, .offset = 0.0f, .inverted = false },

    /* Right leg - Bus 1 */
    [JOINT_RIGHT_HIP_YAW]   = { .bus_id = 1, .motor_id = 4, .gear_ratio = 1.0f, .offset = 0.0f, .inverted = false },
    [JOINT_RIGHT_HIP_ROLL]  = { .bus_id = 1, .motor_id = 5, .gear_ratio = 1.0f, .offset = 0.0f, .inverted = false },
    [JOINT_RIGHT_HIP_PITCH] = { .bus_id = 1, .motor_id = 6, .gear_ratio = 1.0f, .offset = 0.0f, .inverted = false },
    [JOINT_RIGHT_KNEE]      = { .bus_id = 1, .motor_id = 7, .gear_ratio = 1.0f, .offset = 0.0f, .inverted = false },
    [JOINT_RIGHT_ANKLE]     = { .bus_id = 1, .motor_id = 8, .gear_ratio = 1.0f, .offset = 0.0f, .inverted = false },
};

/* ============== Internal Helper Functions ============== */

/**
 * @brief Convert joint position from radians to motor units
 */
static inline int32_t joint_to_motor_pos(float rad, const bdx_motor_map_t *map)
{
    float pos = (rad - map->offset) * map->gear_ratio;
    if (map->inverted) pos = -pos;
    /* Convert to motor encoder units (assume 16-bit encoder) */
    return (int32_t)(pos * 65536.0f / (2.0f * 3.14159265f));
}

/**
 * @brief Convert motor position to joint radians
 */
static inline float motor_to_joint_pos(int32_t motor_pos, const bdx_motor_map_t *map)
{
    float rad = motor_pos * (2.0f * 3.14159265f) / 65536.0f;
    if (map->inverted) rad = -rad;
    return rad / map->gear_ratio + map->offset;
}

/**
 * @brief Convert motor velocity to joint rad/s
 */
static inline float motor_to_joint_vel(int16_t motor_vel, const bdx_motor_map_t *map)
{
    float vel = motor_vel * (2.0f * 3.14159265f) / 65536.0f;  /* rad/s */
    if (map->inverted) vel = -vel;
    return vel / map->gear_ratio;
}

/**
 * @brief Swap state buffer indices (for double buffering)
 */
static inline void swap_state_buffers(bdx_system_t *sys)
{
    sys->state_read_idx = sys->state_write_idx;
    sys->state_write_idx = 1 - sys->state_write_idx;
}

/* ============== Protocol Callbacks ============== */

/**
 * @brief Called when a command frame is received from Jetson
 */
static void on_cmd_received(const bdx_cmd_frame_t *cmd)
{
    if (g_bdx_sys == RT_NULL) return;

    bdx_system_t *sys = g_bdx_sys;

    /* Copy setpoints atomically */
    memcpy(sys->setpoints.positions, cmd->joint_setpoints,
           sizeof(float) * BDX_NUM_JOINTS);
    sys->setpoints.flags = cmd->flags;
    sys->setpoints.sequence = cmd->sequence;
    sys->setpoints.timestamp_us = bdx_get_timestamp_us();
    sys->new_setpoints = true;

    /* Update communication timestamp */
    sys->last_comm_time_us = sys->setpoints.timestamp_us;

    /* Handle control flags */
    if (cmd->flags & BDX_FLAG_ESTOP) {
        bdx_estop(sys);
    } else if (cmd->flags & BDX_FLAG_ENABLE) {
        if (sys->sys_state == BDX_SYS_IDLE) {
            sys->sys_state = BDX_SYS_ACTIVE;
        }
    }
}

/**
 * @brief Called on protocol error
 */
static void on_protocol_error(bdx_error_code_t error)
{
    if (g_bdx_sys == RT_NULL) return;

    g_bdx_sys->last_error = error;
    g_bdx_sys->stats.comm_errors++;

    LOG_W("Protocol error: %d", error);
}

/* ============== 600 Hz Control Loop ============== */

/**
 * @brief Main control loop iteration - called from timer ISR
 *
 * This function executes in ~500-800 µs and handles:
 * 1. Read IMU data
 * 2. Read motor feedback
 * 3. Apply setpoints to motors
 * 4. Send state to Jetson
 */
void bdx_control_loop(bdx_system_t *sys)
{
    uint32_t loop_start = bdx_get_timestamp_us();

    /* Get write buffer */
    bdx_robot_state_t *state = &sys->state[sys->state_write_idx];

    /* ===== 1. Read IMU ===== */
    bdx_update_imu(sys);

    /* Copy IMU data to state */
    bmi088_data_t imu_data;
    if (bmi088_read_all(&sys->imu, &imu_data) == RT_EOK) {
        state->imu_accel[0] = imu_data.accel.x;
        state->imu_accel[1] = imu_data.accel.y;
        state->imu_accel[2] = imu_data.accel.z;
        state->imu_gyro[0] = imu_data.gyro.x;
        state->imu_gyro[1] = imu_data.gyro.y;
        state->imu_gyro[2] = imu_data.gyro.z;
        state->imu_temperature = imu_data.temperature;
    } else {
        sys->stats.imu_errors++;
    }

    /* ===== 2. Read Motor Feedback ===== */
    bdx_update_motors(sys);

    /* Copy motor feedback to state */
    motor_feedback_t feedback[BDX_NUM_JOINTS];
    uint8_t motor_count;
    rs485_motor_get_all_feedback(&sys->motors, feedback, &motor_count);

    for (int i = 0; i < BDX_NUM_JOINTS; i++) {
        const bdx_motor_map_t *map = &sys->config.motor_map[i];

        /* Find matching motor feedback */
        for (int j = 0; j < motor_count; j++) {
            if (feedback[j].id == map->motor_id) {
                state->joint_positions[i] = motor_to_joint_pos(feedback[j].position, map);
                state->joint_velocities[i] = motor_to_joint_vel(feedback[j].velocity, map);
                state->joint_currents[i] = (float)feedback[j].current;
                break;
            }
        }
    }

    /* Detect foot contacts based on ankle motor current */
    state->contact_left = (state->joint_currents[JOINT_LEFT_ANKLE] >
                           sys->config.contact_current_threshold);
    state->contact_right = (state->joint_currents[JOINT_RIGHT_ANKLE] >
                            sys->config.contact_current_threshold);

    /* ===== 3. Apply Motor Commands ===== */
    if (sys->sys_state == BDX_SYS_ACTIVE && sys->new_setpoints) {
        bdx_send_motor_commands(sys);
        sys->new_setpoints = false;
    }

    /* ===== 4. Update State Metadata ===== */
    state->timestamp_us = bdx_get_timestamp_us();
    state->sequence = sys->stats.control_loops++;

    /* Swap buffers */
    swap_state_buffers(sys);

    /* ===== 5. Send State to Jetson ===== */
    bdx_send_state_to_jetson(sys);

    /* ===== 6. Update Statistics ===== */
    uint32_t loop_time = bdx_get_timestamp_us() - loop_start;

    if (loop_time > sys->stats.max_loop_time_us) {
        sys->stats.max_loop_time_us = loop_time;
    }
    /* Simple moving average */
    sys->stats.avg_loop_time_us = (sys->stats.avg_loop_time_us * 7 + loop_time) / 8;

    /* Check for Jetson timeout */
    uint32_t comm_age = bdx_get_timestamp_us() - sys->last_comm_time_us;
    if (comm_age > BDX_COMM_TIMEOUT_MS * 1000 && sys->sys_state == BDX_SYS_ACTIVE) {
        sys->stats.missed_frames++;
        if (sys->stats.missed_frames > 30) {  /* 50ms without communication */
            LOG_W("Jetson communication timeout");
            /* Could trigger safety action here */
        }
    } else {
        sys->stats.missed_frames = 0;
    }
}

/**
 * @brief Update IMU readings
 */
void bdx_update_imu(bdx_system_t *sys)
{
    /* IMU read is handled directly in control loop for minimal latency */
    (void)sys;
}

/**
 * @brief Update motor feedback from RS-485 buses
 */
void bdx_update_motors(bdx_system_t *sys)
{
    /* Request feedback from all motors on both buses */
    rs485_motor_request_all_feedback(&sys->motors);
}

/**
 * @brief Send motor commands based on current setpoints
 */
void bdx_send_motor_commands(bdx_system_t *sys)
{
    motor_command_t cmds[BDX_NUM_JOINTS];

    for (int i = 0; i < BDX_NUM_JOINTS; i++) {
        const bdx_motor_map_t *map = &sys->config.motor_map[i];

        cmds[i].id = map->motor_id;
        cmds[i].mode = MOTOR_MODE_POSITION;
        cmds[i].target_position = joint_to_motor_pos(sys->setpoints.positions[i], map);
        cmds[i].target_velocity = 0;
        cmds[i].target_torque = 0;
    }

    /* Send commands to bus 0 (motors 0-5) */
    for (int i = 0; i < BDX_NUM_MOTORS_BUS1; i++) {
        rs485_motor_send_command(&sys->motors, &cmds[i]);
    }

    /* Send commands to bus 1 (motors 6-13) */
    for (int i = BDX_NUM_MOTORS_BUS1; i < BDX_NUM_JOINTS; i++) {
        rs485_motor_send_command(&sys->motors, &cmds[i]);
    }
}

/**
 * @brief Send current state to Jetson via UART
 */
void bdx_send_state_to_jetson(bdx_system_t *sys)
{
    bdx_robot_state_t *state = &sys->state[sys->state_read_idx];

    /* Build state frame */
    bdx_state_frame_t frame;
    memset(&frame, 0, sizeof(frame));

    frame.start = BDX_FRAME_START;
    frame.type = BDX_FRAME_STATE;
    frame.sequence = (uint16_t)state->sequence;
    frame.timestamp_us = state->timestamp_us;

    /* IMU data */
    memcpy(frame.imu_accel, state->imu_accel, sizeof(float) * 3);
    memcpy(frame.imu_gyro, state->imu_gyro, sizeof(float) * 3);

    /* Joint feedback */
    memcpy(frame.joint_positions, state->joint_positions, sizeof(float) * BDX_NUM_JOINTS);
    memcpy(frame.joint_velocities, state->joint_velocities, sizeof(float) * BDX_NUM_JOINTS);

    /* Status byte */
    frame.status = sys->sys_state & BDX_STATUS_STATE_MASK;
    if (sys->imu.initialized) {
        frame.status |= BDX_STATUS_IMU_OK;
    }
    if (state->contact_left) {
        frame.status |= BDX_STATUS_CONTACT_LEFT;
    }
    if (state->contact_right) {
        frame.status |= BDX_STATUS_CONTACT_RIGHT;
    }
    frame.status |= BDX_STATUS_MOTORS_OK;  /* TODO: check motor status */

    frame.error_code = sys->last_error;

    /* Send via protocol layer */
    bdx_protocol_send_state(&sys->protocol, &frame);
    sys->stats.comm_tx_frames++;
}

/* ============== Timer Callback ============== */

/**
 * @brief Timer interrupt callback - triggers 600 Hz control loop
 *
 * Call this from HAL_TIM_PeriodElapsedCallback for the control timer.
 */
void bdx_timer_callback(bdx_system_t *sys)
{
    if (sys == RT_NULL || !sys->running) return;

    sys->loop_start_time_us = bdx_get_timestamp_us();

    /* Run the control loop */
    bdx_control_loop(sys);
}

/**
 * @brief UART RX complete callback
 *
 * Call this from HAL_UART_RxCpltCallback for the Jetson UART.
 */
void bdx_uart_rx_callback(bdx_system_t *sys)
{
    if (sys == RT_NULL || !sys->initialized) return;

    bdx_protocol_rx_complete_callback(&sys->protocol);
    sys->stats.comm_rx_frames++;
}

/**
 * @brief UART TX complete callback
 *
 * Call this from HAL_UART_TxCpltCallback for the Jetson UART.
 */
void bdx_uart_tx_callback(bdx_system_t *sys)
{
    if (sys == RT_NULL || !sys->initialized) return;

    bdx_protocol_tx_complete_callback(&sys->protocol);
}

/* ============== Public API Implementation ============== */

void bdx_get_default_config(bdx_config_t *config)
{
    RT_ASSERT(config != RT_NULL);

    memset(config, 0, sizeof(bdx_config_t));

    /* Jetson UART */
    config->huart_jetson = &huart1;

    /* IMU (SPI2 with CS pins) */
    config->hspi_imu = &hspi2;
    config->imu_acc_cs_port = GPIOC;
    config->imu_acc_cs_pin = GPIO_PIN_0;
    config->imu_gyro_cs_port = GPIOC;
    config->imu_gyro_cs_pin = GPIO_PIN_3;

    /* RS-485 bus 1 (USART2) */
    config->huart_rs485_1 = &huart2;
    config->rs485_1_de_port = GPIOD;
    config->rs485_1_de_pin = GPIO_PIN_4;

    /* RS-485 bus 2 (USART3) */
    config->huart_rs485_2 = &huart3;
    config->rs485_2_de_port = GPIOB;
    config->rs485_2_de_pin = GPIO_PIN_14;

    /* Motor mapping (default BDX_SIm configuration) */
    memcpy(config->motor_map, default_motor_map, sizeof(default_motor_map));

    /* Control timer */
    config->htim_control = &htim6;

    /* Contact detection threshold */
    config->contact_current_threshold = 500.0f;  /* 500 mA */
}

rt_err_t bdx_init(bdx_system_t *sys, const bdx_config_t *config)
{
    RT_ASSERT(sys != RT_NULL);
    RT_ASSERT(config != RT_NULL);

    memset(sys, 0, sizeof(bdx_system_t));
    memcpy(&sys->config, config, sizeof(bdx_config_t));

    rt_err_t ret;

    /* ===== Initialize Protocol ===== */
    ret = bdx_protocol_init(&sys->protocol,
                            config->huart_jetson,
                            sys->dma_tx_buffer,
                            sys->dma_rx_buffer);
    if (ret != RT_EOK) {
        LOG_E("Protocol init failed");
        return ret;
    }

    /* Set protocol callbacks */
    sys->protocol.on_cmd_received = on_cmd_received;
    sys->protocol.on_error = on_protocol_error;

    /* ===== Initialize IMU ===== */
    ret = bmi088_init(&sys->imu,
                      config->hspi_imu,
                      config->imu_acc_cs_port,
                      config->imu_acc_cs_pin,
                      config->imu_gyro_cs_port,
                      config->imu_gyro_cs_pin);
    if (ret != RT_EOK) {
        LOG_W("IMU init failed, continuing without IMU");
    } else {
        /* Configure IMU for 800 Hz ODR */
        bmi088_config_t imu_config;
        bmi088_get_default_config(&imu_config);
        imu_config.acc_odr = BMI088_ACC_ODR_800;
        imu_config.gyro_odr = BMI088_GYRO_ODR_1000_BW_116;
        ret = bmi088_configure(&sys->imu, &imu_config);
        if (ret != RT_EOK) {
            LOG_W("IMU config failed");
        }
    }

    /* ===== Initialize Motor System ===== */
    ret = rs485_motor_init(&sys->motors);
    if (ret != RT_EOK) {
        LOG_E("Motor system init failed");
        return ret;
    }

    /* Configure RS-485 bus 1 */
    rs485_bus_config_t bus1_cfg = {
        .huart = config->huart_rs485_1,
        .de_port = config->rs485_1_de_port,
        .de_pin = config->rs485_1_de_pin,
        .baudrate = 1000000,  /* 1 Mbps for Unitree motors */
        .protocol = MOTOR_PROTOCOL_UNITREE,
        .motor_count = BDX_NUM_MOTORS_BUS1,
    };
    for (int i = 0; i < BDX_NUM_MOTORS_BUS1; i++) {
        bus1_cfg.motor_ids[i] = config->motor_map[i].motor_id;
    }
    ret = rs485_bus_configure(&sys->motors, 0, &bus1_cfg);
    if (ret != RT_EOK) {
        LOG_W("RS-485 bus 1 config failed");
    }

    /* Configure RS-485 bus 2 */
    rs485_bus_config_t bus2_cfg = {
        .huart = config->huart_rs485_2,
        .de_port = config->rs485_2_de_port,
        .de_pin = config->rs485_2_de_pin,
        .baudrate = 1000000,
        .protocol = MOTOR_PROTOCOL_UNITREE,
        .motor_count = BDX_NUM_MOTORS_BUS2,
    };
    for (int i = 0; i < BDX_NUM_MOTORS_BUS2; i++) {
        bus2_cfg.motor_ids[i] = config->motor_map[BDX_NUM_MOTORS_BUS1 + i].motor_id;
    }
    ret = rs485_bus_configure(&sys->motors, 1, &bus2_cfg);
    if (ret != RT_EOK) {
        LOG_W("RS-485 bus 2 config failed");
    }

    /* Set global instance for callbacks */
    g_bdx_sys = sys;

    sys->sys_state = BDX_SYS_IDLE;
    sys->initialized = true;

    LOG_I("BDX system initialized");
    LOG_I("  Control freq: %d Hz", BDX_CONTROL_FREQ_HZ);
    LOG_I("  Joints: %d (bus1: %d, bus2: %d)",
          BDX_NUM_JOINTS, BDX_NUM_MOTORS_BUS1, BDX_NUM_MOTORS_BUS2);
    LOG_I("  IMU: %s", sys->imu.initialized ? "OK" : "FAIL");

    return RT_EOK;
}

rt_err_t bdx_start(bdx_system_t *sys)
{
    RT_ASSERT(sys != RT_NULL);

    if (!sys->initialized) {
        LOG_E("System not initialized");
        return -RT_ENOSYS;
    }

    if (sys->running) {
        return RT_EOK;
    }

    /* Start UART DMA reception */
    rt_err_t ret = bdx_protocol_start_rx(&sys->protocol);
    if (ret != RT_EOK) {
        LOG_E("Failed to start UART RX");
        return ret;
    }

    /* Start 600 Hz control timer */
    HAL_TIM_Base_Start_IT(sys->config.htim_control);

    sys->running = true;
    sys->sys_state = BDX_SYS_IDLE;
    sys->last_comm_time_us = bdx_get_timestamp_us();

    LOG_I("BDX system started - 600 Hz control loop running");

    return RT_EOK;
}

rt_err_t bdx_stop(bdx_system_t *sys)
{
    RT_ASSERT(sys != RT_NULL);

    if (!sys->running) {
        return RT_EOK;
    }

    /* Stop timer first */
    HAL_TIM_Base_Stop_IT(sys->config.htim_control);

    /* Disable all motors */
    rs485_motor_disable_all(&sys->motors);

    sys->running = false;
    sys->sys_state = BDX_SYS_IDLE;

    LOG_I("BDX system stopped");

    return RT_EOK;
}

void bdx_estop(bdx_system_t *sys)
{
    RT_ASSERT(sys != RT_NULL);

    /* Immediately disable all motors */
    rs485_motor_disable_all(&sys->motors);

    sys->sys_state = BDX_SYS_ESTOP;
    sys->last_error = BDX_ERR_ESTOP;

    LOG_W("EMERGENCY STOP ACTIVATED");
}

rt_err_t bdx_clear_estop(bdx_system_t *sys)
{
    RT_ASSERT(sys != RT_NULL);

    if (sys->sys_state != BDX_SYS_ESTOP) {
        return RT_EOK;
    }

    sys->sys_state = BDX_SYS_IDLE;
    sys->last_error = BDX_ERR_NONE;

    LOG_I("ESTOP cleared");

    return RT_EOK;
}

void bdx_get_state(bdx_system_t *sys, bdx_robot_state_t *state)
{
    RT_ASSERT(sys != RT_NULL);
    RT_ASSERT(state != RT_NULL);

    /* Read from the current read buffer (lock-free) */
    memcpy(state, &sys->state[sys->state_read_idx], sizeof(bdx_robot_state_t));
}

void bdx_get_stats(bdx_system_t *sys, bdx_stats_t *stats)
{
    RT_ASSERT(sys != RT_NULL);
    RT_ASSERT(stats != RT_NULL);

    memcpy(stats, &sys->stats, sizeof(bdx_stats_t));
}

bdx_sys_state_t bdx_get_sys_state(bdx_system_t *sys)
{
    RT_ASSERT(sys != RT_NULL);
    return sys->sys_state;
}

bool bdx_is_comm_active(bdx_system_t *sys)
{
    RT_ASSERT(sys != RT_NULL);

    uint32_t age = bdx_get_timestamp_us() - sys->last_comm_time_us;
    return (age < BDX_COMM_TIMEOUT_MS * 1000);
}

/* ============== MSH Commands ============== */

#ifdef RT_USING_FINSH
#include <finsh.h>

static void bdx_status_cmd(int argc, char **argv)
{
    if (g_bdx_sys == RT_NULL) {
        rt_kprintf("BDX system not initialized\n");
        return;
    }

    bdx_system_t *sys = g_bdx_sys;
    bdx_robot_state_t state;
    bdx_get_state(sys, &state);

    const char *state_names[] = {"UNINIT", "IDLE", "ACTIVE", "ESTOP", "ERROR"};

    rt_kprintf("\n=== BDX System Status ===\n");
    rt_kprintf("State: %s\n", state_names[sys->sys_state]);
    rt_kprintf("Running: %s\n", sys->running ? "YES" : "NO");
    rt_kprintf("Control loops: %lu\n", sys->stats.control_loops);
    rt_kprintf("Loop time: avg=%lu us, max=%lu us\n",
               sys->stats.avg_loop_time_us, sys->stats.max_loop_time_us);
    rt_kprintf("\n");

    rt_kprintf("=== Communication ===\n");
    rt_kprintf("RX frames: %lu\n", sys->stats.comm_rx_frames);
    rt_kprintf("TX frames: %lu\n", sys->stats.comm_tx_frames);
    rt_kprintf("Errors: %lu\n", sys->stats.comm_errors);
    rt_kprintf("Active: %s\n", bdx_is_comm_active(sys) ? "YES" : "NO");
    rt_kprintf("\n");

    rt_kprintf("=== IMU ===\n");
    rt_kprintf("Accel: [%.2f, %.2f, %.2f] m/s²\n",
               state.imu_accel[0], state.imu_accel[1], state.imu_accel[2]);
    rt_kprintf("Gyro:  [%.2f, %.2f, %.2f] rad/s\n",
               state.imu_gyro[0], state.imu_gyro[1], state.imu_gyro[2]);
    rt_kprintf("Errors: %lu\n", sys->stats.imu_errors);
    rt_kprintf("\n");

    rt_kprintf("=== Contacts ===\n");
    rt_kprintf("Left: %s  Right: %s\n",
               state.contact_left ? "YES" : "NO",
               state.contact_right ? "YES" : "NO");
}
MSH_CMD_EXPORT_ALIAS(bdx_status_cmd, bdx_status, Show BDX system status);

static void bdx_joints_cmd(int argc, char **argv)
{
    if (g_bdx_sys == RT_NULL) {
        rt_kprintf("BDX system not initialized\n");
        return;
    }

    bdx_robot_state_t state;
    bdx_get_state(g_bdx_sys, &state);

    const char *joint_names[] = {
        "L_Hip_Yaw", "L_Hip_Roll", "L_Hip_Pitch", "L_Knee", "L_Ankle",
        "Neck_Pitch", "Head_Pitch", "Head_Yaw", "Head_Roll",
        "R_Hip_Yaw", "R_Hip_Roll", "R_Hip_Pitch", "R_Knee", "R_Ankle"
    };

    rt_kprintf("\n=== Joint Status ===\n");
    rt_kprintf("%-12s  %8s  %8s  %8s\n", "Joint", "Pos(rad)", "Vel(r/s)", "Cur(mA)");
    rt_kprintf("%-12s  %8s  %8s  %8s\n", "-----", "--------", "--------", "-------");

    for (int i = 0; i < BDX_NUM_JOINTS; i++) {
        rt_kprintf("%-12s  %8.3f  %8.3f  %8.1f\n",
                   joint_names[i],
                   state.joint_positions[i],
                   state.joint_velocities[i],
                   state.joint_currents[i]);
    }
}
MSH_CMD_EXPORT_ALIAS(bdx_joints_cmd, bdx_joints, Show joint status);

static void bdx_estop_cmd(int argc, char **argv)
{
    if (g_bdx_sys == RT_NULL) {
        rt_kprintf("BDX system not initialized\n");
        return;
    }
    bdx_estop(g_bdx_sys);
    rt_kprintf("ESTOP activated\n");
}
MSH_CMD_EXPORT_ALIAS(bdx_estop_cmd, bdx_estop, Emergency stop);

static void bdx_clear_cmd(int argc, char **argv)
{
    if (g_bdx_sys == RT_NULL) {
        rt_kprintf("BDX system not initialized\n");
        return;
    }
    bdx_clear_estop(g_bdx_sys);
    rt_kprintf("ESTOP cleared\n");
}
MSH_CMD_EXPORT_ALIAS(bdx_clear_cmd, bdx_clear, Clear emergency stop);

void bdx_register_msh_commands(bdx_system_t *sys)
{
    /* Commands are auto-registered by MSH_CMD_EXPORT */
    (void)sys;
}

#endif /* RT_USING_FINSH */
