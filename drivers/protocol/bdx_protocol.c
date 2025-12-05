/**
 * @file bdx_protocol.c
 * @brief BDX_SIm Communication Protocol Implementation - 600Hz Real-Time
 */

#include "bdx_protocol.h"
#include "stm32h7xx_hal.h"
#include <string.h>

#define DBG_TAG "bdx_proto"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* ============== CRC-8 Table (Polynomial 0x07) ============== */

static const uint8_t crc8_table[256] = {
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
    0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
    0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
    0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
    0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
    0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
    0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
    0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
    0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
    0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
    0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
    0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
    0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
    0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
    0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
    0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
    0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};

/* ============== Timestamp using DWT cycle counter ============== */

static uint32_t cpu_freq_mhz = 550;  /* Default for STM32H723 */

static void init_dwt_counter(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* Get actual CPU frequency */
    cpu_freq_mhz = HAL_RCC_GetHCLKFreq() / 1000000;
}

uint32_t bdx_get_timestamp_us(void)
{
    return DWT->CYCCNT / cpu_freq_mhz;
}

/* ============== CRC-8 Implementation ============== */

uint8_t bdx_crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00;
    while (len--) {
        crc = crc8_table[crc ^ *data++];
    }
    return crc;
}

/* ============== Protocol Implementation ============== */

rt_err_t bdx_protocol_init(bdx_protocol_ctx_t *ctx,
                           UART_HandleTypeDef *huart,
                           uint8_t *tx_buffer,
                           uint8_t *rx_buffer)
{
    RT_ASSERT(ctx != RT_NULL);
    RT_ASSERT(huart != RT_NULL);
    RT_ASSERT(tx_buffer != RT_NULL);
    RT_ASSERT(rx_buffer != RT_NULL);

    memset(ctx, 0, sizeof(bdx_protocol_ctx_t));

    ctx->huart = huart;
    ctx->tx_buffer = tx_buffer;
    ctx->rx_buffer = rx_buffer;

    /* Initialize synchronization primitives */
    rt_err_t ret = rt_sem_init(&ctx->rx_sem, "bdx_rx", 0, RT_IPC_FLAG_FIFO);
    if (ret != RT_EOK) {
        LOG_E("Failed to create RX semaphore");
        return ret;
    }

    ret = rt_mutex_init(&ctx->tx_lock, "bdx_tx", RT_IPC_FLAG_PRIO);
    if (ret != RT_EOK) {
        LOG_E("Failed to create TX mutex");
        return ret;
    }

    /* Initialize DWT for microsecond timestamps */
    init_dwt_counter();

    ctx->initialized = true;
    LOG_I("BDX protocol initialized (UART @ %lu bps)", huart->Init.BaudRate);

    return RT_EOK;
}

rt_err_t bdx_protocol_start_rx(bdx_protocol_ctx_t *ctx)
{
    RT_ASSERT(ctx != RT_NULL);
    RT_ASSERT(ctx->initialized);

    /* Start DMA reception for command frame */
    HAL_StatusTypeDef status = HAL_UART_Receive_DMA(ctx->huart,
                                                     ctx->rx_buffer,
                                                     BDX_CMD_FRAME_SIZE);

    if (status != HAL_OK) {
        LOG_E("Failed to start UART DMA RX");
        return -RT_EIO;
    }

    LOG_D("DMA RX started, waiting for %d bytes", BDX_CMD_FRAME_SIZE);
    return RT_EOK;
}

rt_err_t bdx_protocol_send_state(bdx_protocol_ctx_t *ctx,
                                  const bdx_state_frame_t *state)
{
    RT_ASSERT(ctx != RT_NULL);
    RT_ASSERT(state != RT_NULL);

    rt_mutex_take(&ctx->tx_lock, RT_WAITING_FOREVER);

    /* Copy state to TX buffer */
    bdx_state_frame_t *tx_frame = (bdx_state_frame_t *)ctx->tx_buffer;
    memcpy(tx_frame, state, sizeof(bdx_state_frame_t));

    /* Fill header */
    tx_frame->start = BDX_FRAME_START;
    tx_frame->type = BDX_FRAME_STATE;
    tx_frame->sequence = ctx->tx_seq++;
    tx_frame->timestamp_us = bdx_get_timestamp_us();

    /* Calculate CRC (exclude CRC byte itself) */
    tx_frame->crc = bdx_crc8(ctx->tx_buffer, BDX_STATE_FRAME_SIZE - 1);

    /* Send via DMA */
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(ctx->huart,
                                                      ctx->tx_buffer,
                                                      BDX_STATE_FRAME_SIZE);

    ctx->last_tx_time_us = bdx_get_timestamp_us();
    ctx->stats.tx_frames++;

    rt_mutex_release(&ctx->tx_lock);

    return (status == HAL_OK) ? RT_EOK : -RT_EIO;
}

rt_err_t bdx_protocol_send_ack(bdx_protocol_ctx_t *ctx,
                                uint16_t seq, uint8_t status_code)
{
    RT_ASSERT(ctx != RT_NULL);

    rt_mutex_take(&ctx->tx_lock, RT_WAITING_FOREVER);

    bdx_ack_frame_t *ack = (bdx_ack_frame_t *)ctx->tx_buffer;
    ack->start = BDX_FRAME_START;
    ack->type = BDX_FRAME_ACK;
    ack->sequence = seq;
    ack->status = status_code;
    ack->crc = bdx_crc8(ctx->tx_buffer, BDX_ACK_FRAME_SIZE - 1);

    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(ctx->huart,
                                                      ctx->tx_buffer,
                                                      BDX_ACK_FRAME_SIZE);

    rt_mutex_release(&ctx->tx_lock);

    return (status == HAL_OK) ? RT_EOK : -RT_EIO;
}

bool bdx_protocol_cmd_available(bdx_protocol_ctx_t *ctx)
{
    return ctx->cmd_received;
}

rt_err_t bdx_protocol_get_cmd(bdx_protocol_ctx_t *ctx, bdx_cmd_frame_t *cmd)
{
    RT_ASSERT(ctx != RT_NULL);
    RT_ASSERT(cmd != RT_NULL);

    if (!ctx->cmd_received) {
        return -RT_EEMPTY;
    }

    memcpy(cmd, &ctx->last_cmd, sizeof(bdx_cmd_frame_t));
    ctx->cmd_received = false;

    return RT_EOK;
}

void bdx_protocol_process_rx(bdx_protocol_ctx_t *ctx,
                              const uint8_t *data, uint16_t len)
{
    uint32_t rx_time = bdx_get_timestamp_us();

    /* Validate minimum size */
    if (len < 6) {
        ctx->stats.rx_errors++;
        return;
    }

    /* Check start marker */
    if (data[0] != BDX_FRAME_START) {
        ctx->stats.rx_errors++;
        return;
    }

    uint8_t frame_type = data[1];

    /* Process based on frame type */
    switch (frame_type) {
        case BDX_FRAME_CMD: {
            if (len < BDX_CMD_FRAME_SIZE) {
                ctx->stats.rx_errors++;
                return;
            }

            /* Verify CRC */
            uint8_t crc_calc = bdx_crc8(data, BDX_CMD_FRAME_SIZE - 1);
            uint8_t crc_recv = data[BDX_CMD_FRAME_SIZE - 1];

            if (crc_calc != crc_recv) {
                ctx->stats.crc_errors++;
                ctx->stats.rx_errors++;
                if (ctx->on_error) {
                    ctx->on_error(BDX_ERR_CRC);
                }
                return;
            }

            /* Copy command */
            memcpy(&ctx->last_cmd, data, sizeof(bdx_cmd_frame_t));

            /* Check sequence */
            uint16_t expected_seq = ctx->rx_seq + 1;
            if (ctx->last_cmd.sequence != expected_seq && ctx->stats.rx_frames > 0) {
                ctx->stats.sequence_errors++;
            }
            ctx->rx_seq = ctx->last_cmd.sequence;

            /* Update timing stats */
            if (ctx->last_rx_time_us > 0) {
                uint32_t latency = rx_time - ctx->last_rx_time_us;
                if (latency > ctx->stats.max_latency_us) {
                    ctx->stats.max_latency_us = latency;
                }
                /* Simple moving average */
                ctx->stats.avg_latency_us = (ctx->stats.avg_latency_us * 7 + latency) / 8;
            }
            ctx->last_rx_time_us = rx_time;

            ctx->cmd_received = true;
            ctx->stats.rx_frames++;
            ctx->stats.last_rx_seq = ctx->rx_seq;

            /* Callback */
            if (ctx->on_cmd_received) {
                ctx->on_cmd_received(&ctx->last_cmd);
            }

            /* Release semaphore */
            rt_sem_release(&ctx->rx_sem);
            break;
        }

        case BDX_FRAME_CONFIG: {
            /* Handle configuration frames */
            if (len < BDX_CONFIG_FRAME_SIZE) {
                ctx->stats.rx_errors++;
                return;
            }
            /* TODO: Process configuration */
            break;
        }

        default:
            ctx->stats.rx_errors++;
            break;
    }
}

void bdx_protocol_rx_complete_callback(bdx_protocol_ctx_t *ctx)
{
    /* Process received data */
    bdx_protocol_process_rx(ctx, ctx->rx_buffer, BDX_CMD_FRAME_SIZE);

    /* Restart DMA reception immediately */
    HAL_UART_Receive_DMA(ctx->huart, ctx->rx_buffer, BDX_CMD_FRAME_SIZE);
}

void bdx_protocol_tx_complete_callback(bdx_protocol_ctx_t *ctx)
{
    /* TX complete - could signal if needed */
    (void)ctx;
}

void bdx_protocol_get_stats(bdx_protocol_ctx_t *ctx, bdx_protocol_stats_t *stats)
{
    if (ctx && stats) {
        memcpy(stats, &ctx->stats, sizeof(bdx_protocol_stats_t));
    }
}

void bdx_protocol_reset(bdx_protocol_ctx_t *ctx)
{
    if (ctx) {
        ctx->tx_seq = 0;
        ctx->rx_seq = 0;
        ctx->cmd_received = false;
        memset(&ctx->stats, 0, sizeof(bdx_protocol_stats_t));
        ctx->last_rx_time_us = 0;
        ctx->last_tx_time_us = 0;
    }
}
