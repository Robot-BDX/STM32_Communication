/**
 * @file bmi088.c
 * @brief BMI088 IMU Driver Implementation
 */

#include "bmi088.h"
#include "stm32h7xx_hal.h"
#include <string.h>

#define DBG_TAG "bmi088"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* ============== Internal Macros ============== */

#define BMI088_SPI_TIMEOUT      100
#define BMI088_RESET_DELAY_MS   50
#define BMI088_INIT_DELAY_MS    10

/* Gravity constant */
#define GRAVITY_EARTH           9.80665f
#define DEG_TO_RAD              0.017453292519943295f

/* ============== Internal Functions ============== */

static inline void bmi088_acc_cs_low(bmi088_device_t *dev)
{
    HAL_GPIO_WritePin(dev->acc_cs_port, dev->acc_cs_pin, GPIO_PIN_RESET);
}

static inline void bmi088_acc_cs_high(bmi088_device_t *dev)
{
    HAL_GPIO_WritePin(dev->acc_cs_port, dev->acc_cs_pin, GPIO_PIN_SET);
}

static inline void bmi088_gyro_cs_low(bmi088_device_t *dev)
{
    HAL_GPIO_WritePin(dev->gyro_cs_port, dev->gyro_cs_pin, GPIO_PIN_RESET);
}

static inline void bmi088_gyro_cs_high(bmi088_device_t *dev)
{
    HAL_GPIO_WritePin(dev->gyro_cs_port, dev->gyro_cs_pin, GPIO_PIN_SET);
}

static rt_err_t bmi088_acc_read_reg(bmi088_device_t *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    uint8_t tx_buf[2];
    uint8_t dummy;
    HAL_StatusTypeDef status;

    tx_buf[0] = reg | 0x80;  /* Read bit */

    bmi088_acc_cs_low(dev);

    /* Send register address */
    status = HAL_SPI_Transmit(dev->hspi, tx_buf, 1, BMI088_SPI_TIMEOUT);
    if (status != HAL_OK) {
        bmi088_acc_cs_high(dev);
        return -RT_EIO;
    }

    /* BMI088 accelerometer requires dummy byte after address */
    status = HAL_SPI_Receive(dev->hspi, &dummy, 1, BMI088_SPI_TIMEOUT);
    if (status != HAL_OK) {
        bmi088_acc_cs_high(dev);
        return -RT_EIO;
    }

    /* Read data */
    status = HAL_SPI_Receive(dev->hspi, data, len, BMI088_SPI_TIMEOUT);

    bmi088_acc_cs_high(dev);

    return (status == HAL_OK) ? RT_EOK : -RT_EIO;
}

static rt_err_t bmi088_acc_write_reg(bmi088_device_t *dev, uint8_t reg, uint8_t data)
{
    uint8_t tx_buf[2];
    HAL_StatusTypeDef status;

    tx_buf[0] = reg & 0x7F;  /* Write bit (MSB = 0) */
    tx_buf[1] = data;

    bmi088_acc_cs_low(dev);
    status = HAL_SPI_Transmit(dev->hspi, tx_buf, 2, BMI088_SPI_TIMEOUT);
    bmi088_acc_cs_high(dev);

    return (status == HAL_OK) ? RT_EOK : -RT_EIO;
}

static rt_err_t bmi088_gyro_read_reg(bmi088_device_t *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    uint8_t tx_buf[1];
    HAL_StatusTypeDef status;

    tx_buf[0] = reg | 0x80;  /* Read bit */

    bmi088_gyro_cs_low(dev);

    /* Send register address */
    status = HAL_SPI_Transmit(dev->hspi, tx_buf, 1, BMI088_SPI_TIMEOUT);
    if (status != HAL_OK) {
        bmi088_gyro_cs_high(dev);
        return -RT_EIO;
    }

    /* Read data - gyroscope doesn't need dummy byte */
    status = HAL_SPI_Receive(dev->hspi, data, len, BMI088_SPI_TIMEOUT);

    bmi088_gyro_cs_high(dev);

    return (status == HAL_OK) ? RT_EOK : -RT_EIO;
}

static rt_err_t bmi088_gyro_write_reg(bmi088_device_t *dev, uint8_t reg, uint8_t data)
{
    uint8_t tx_buf[2];
    HAL_StatusTypeDef status;

    tx_buf[0] = reg & 0x7F;
    tx_buf[1] = data;

    bmi088_gyro_cs_low(dev);
    status = HAL_SPI_Transmit(dev->hspi, tx_buf, 2, BMI088_SPI_TIMEOUT);
    bmi088_gyro_cs_high(dev);

    return (status == HAL_OK) ? RT_EOK : -RT_EIO;
}

static void bmi088_update_acc_sensitivity(bmi088_device_t *dev)
{
    /* Sensitivity in LSB/g */
    switch (dev->config.acc_range) {
        case BMI088_ACC_RANGE_3G:
            dev->acc_sensitivity = GRAVITY_EARTH / 10920.0f;
            break;
        case BMI088_ACC_RANGE_6G:
            dev->acc_sensitivity = GRAVITY_EARTH / 5460.0f;
            break;
        case BMI088_ACC_RANGE_12G:
            dev->acc_sensitivity = GRAVITY_EARTH / 2730.0f;
            break;
        case BMI088_ACC_RANGE_24G:
            dev->acc_sensitivity = GRAVITY_EARTH / 1365.0f;
            break;
        default:
            dev->acc_sensitivity = GRAVITY_EARTH / 5460.0f;
            break;
    }
}

static void bmi088_update_gyro_sensitivity(bmi088_device_t *dev)
{
    /* Sensitivity in LSB/(deg/s) */
    switch (dev->config.gyro_range) {
        case BMI088_GYRO_RANGE_2000DPS:
            dev->gyro_sensitivity = DEG_TO_RAD / 16.384f;
            break;
        case BMI088_GYRO_RANGE_1000DPS:
            dev->gyro_sensitivity = DEG_TO_RAD / 32.768f;
            break;
        case BMI088_GYRO_RANGE_500DPS:
            dev->gyro_sensitivity = DEG_TO_RAD / 65.536f;
            break;
        case BMI088_GYRO_RANGE_250DPS:
            dev->gyro_sensitivity = DEG_TO_RAD / 131.072f;
            break;
        case BMI088_GYRO_RANGE_125DPS:
            dev->gyro_sensitivity = DEG_TO_RAD / 262.144f;
            break;
        default:
            dev->gyro_sensitivity = DEG_TO_RAD / 16.384f;
            break;
    }
}

/* ============== Public API Implementation ============== */

void bmi088_get_default_config(bmi088_config_t *config)
{
    config->acc_range = BMI088_ACC_RANGE_6G;
    config->gyro_range = BMI088_GYRO_RANGE_2000DPS;
    config->acc_odr = BMI088_ACC_ODR_800HZ;
    config->gyro_odr = BMI088_GYRO_ODR_1000HZ_116HZ;
}

rt_err_t bmi088_init(bmi088_device_t *dev,
                     SPI_HandleTypeDef *hspi,
                     GPIO_TypeDef *acc_cs_port, uint16_t acc_cs_pin,
                     GPIO_TypeDef *gyro_cs_port, uint16_t gyro_cs_pin)
{
    rt_err_t ret;
    uint8_t chip_id;

    RT_ASSERT(dev != RT_NULL);
    RT_ASSERT(hspi != RT_NULL);

    /* Initialize device structure */
    memset(dev, 0, sizeof(bmi088_device_t));
    dev->hspi = hspi;
    dev->acc_cs_port = acc_cs_port;
    dev->acc_cs_pin = acc_cs_pin;
    dev->gyro_cs_port = gyro_cs_port;
    dev->gyro_cs_pin = gyro_cs_pin;

    /* Initialize mutex */
    ret = rt_mutex_init(&dev->lock, "bmi088", RT_IPC_FLAG_PRIO);
    if (ret != RT_EOK) {
        LOG_E("Failed to create mutex");
        return ret;
    }

    /* Set CS pins high (deselect) */
    bmi088_acc_cs_high(dev);
    bmi088_gyro_cs_high(dev);

    rt_thread_mdelay(BMI088_INIT_DELAY_MS);

    /* Reset accelerometer */
    ret = bmi088_acc_write_reg(dev, BMI088_ACC_SOFTRESET, 0xB6);
    if (ret != RT_EOK) {
        LOG_E("Failed to reset accelerometer");
        return ret;
    }
    rt_thread_mdelay(BMI088_RESET_DELAY_MS);

    /* Reset gyroscope */
    ret = bmi088_gyro_write_reg(dev, BMI088_GYRO_SOFTRESET, 0xB6);
    if (ret != RT_EOK) {
        LOG_E("Failed to reset gyroscope");
        return ret;
    }
    rt_thread_mdelay(BMI088_RESET_DELAY_MS);

    /* Read and verify accelerometer chip ID */
    ret = bmi088_acc_read_reg(dev, BMI088_ACC_CHIP_ID, &chip_id, 1);
    if (ret != RT_EOK || chip_id != BMI088_ACC_CHIP_ID_VALUE) {
        LOG_E("Accelerometer chip ID mismatch: 0x%02X (expected 0x%02X)",
              chip_id, BMI088_ACC_CHIP_ID_VALUE);
        return -RT_ENOSYS;
    }
    LOG_I("Accelerometer detected: chip ID 0x%02X", chip_id);

    /* Read and verify gyroscope chip ID */
    ret = bmi088_gyro_read_reg(dev, BMI088_GYRO_CHIP_ID, &chip_id, 1);
    if (ret != RT_EOK || chip_id != BMI088_GYRO_CHIP_ID_VALUE) {
        LOG_E("Gyroscope chip ID mismatch: 0x%02X (expected 0x%02X)",
              chip_id, BMI088_GYRO_CHIP_ID_VALUE);
        return -RT_ENOSYS;
    }
    LOG_I("Gyroscope detected: chip ID 0x%02X", chip_id);

    /* Power on accelerometer */
    ret = bmi088_acc_write_reg(dev, BMI088_ACC_PWR_CTRL, 0x04);
    if (ret != RT_EOK) return ret;
    rt_thread_mdelay(5);

    ret = bmi088_acc_write_reg(dev, BMI088_ACC_PWR_CONF, 0x00);
    if (ret != RT_EOK) return ret;
    rt_thread_mdelay(5);

    /* Apply default configuration */
    bmi088_config_t default_config;
    bmi088_get_default_config(&default_config);
    ret = bmi088_configure(dev, &default_config);
    if (ret != RT_EOK) return ret;

    dev->initialized = true;
    LOG_I("BMI088 initialized successfully");

    return RT_EOK;
}

rt_err_t bmi088_configure(bmi088_device_t *dev, const bmi088_config_t *config)
{
    rt_err_t ret;

    RT_ASSERT(dev != RT_NULL);
    RT_ASSERT(config != RT_NULL);

    rt_mutex_take(&dev->lock, RT_WAITING_FOREVER);

    /* Configure accelerometer range */
    ret = bmi088_acc_write_reg(dev, BMI088_ACC_RANGE, config->acc_range);
    if (ret != RT_EOK) goto exit;

    /* Configure accelerometer ODR and bandwidth */
    ret = bmi088_acc_write_reg(dev, BMI088_ACC_CONF, (0x80 | config->acc_odr));
    if (ret != RT_EOK) goto exit;

    /* Configure gyroscope range */
    ret = bmi088_gyro_write_reg(dev, BMI088_GYRO_RANGE, config->gyro_range);
    if (ret != RT_EOK) goto exit;

    /* Configure gyroscope ODR */
    ret = bmi088_gyro_write_reg(dev, BMI088_GYRO_BANDWIDTH, config->gyro_odr);
    if (ret != RT_EOK) goto exit;

    /* Set gyroscope to normal mode */
    ret = bmi088_gyro_write_reg(dev, BMI088_GYRO_LPM1, 0x00);
    if (ret != RT_EOK) goto exit;

    /* Save configuration */
    memcpy(&dev->config, config, sizeof(bmi088_config_t));

    /* Update sensitivity factors */
    bmi088_update_acc_sensitivity(dev);
    bmi088_update_gyro_sensitivity(dev);

    LOG_D("BMI088 configured: acc_range=%d, gyro_range=%d",
          config->acc_range, config->gyro_range);

exit:
    rt_mutex_release(&dev->lock);
    return ret;
}

rt_err_t bmi088_read_accel(bmi088_device_t *dev, bmi088_scaled_data_t *accel)
{
    rt_err_t ret;
    uint8_t raw[6];
    bmi088_raw_data_t raw_data;

    RT_ASSERT(dev != RT_NULL);
    RT_ASSERT(accel != RT_NULL);

    rt_mutex_take(&dev->lock, RT_WAITING_FOREVER);

    ret = bmi088_acc_read_reg(dev, BMI088_ACC_X_LSB, raw, 6);
    if (ret == RT_EOK) {
        raw_data.x = (int16_t)((raw[1] << 8) | raw[0]);
        raw_data.y = (int16_t)((raw[3] << 8) | raw[2]);
        raw_data.z = (int16_t)((raw[5] << 8) | raw[4]);

        accel->x = raw_data.x * dev->acc_sensitivity;
        accel->y = raw_data.y * dev->acc_sensitivity;
        accel->z = raw_data.z * dev->acc_sensitivity;
    }

    rt_mutex_release(&dev->lock);
    return ret;
}

rt_err_t bmi088_read_gyro(bmi088_device_t *dev, bmi088_scaled_data_t *gyro)
{
    rt_err_t ret;
    uint8_t raw[6];
    bmi088_raw_data_t raw_data;

    RT_ASSERT(dev != RT_NULL);
    RT_ASSERT(gyro != RT_NULL);

    rt_mutex_take(&dev->lock, RT_WAITING_FOREVER);

    ret = bmi088_gyro_read_reg(dev, BMI088_GYRO_X_LSB, raw, 6);
    if (ret == RT_EOK) {
        raw_data.x = (int16_t)((raw[1] << 8) | raw[0]);
        raw_data.y = (int16_t)((raw[3] << 8) | raw[2]);
        raw_data.z = (int16_t)((raw[5] << 8) | raw[4]);

        gyro->x = raw_data.x * dev->gyro_sensitivity;
        gyro->y = raw_data.y * dev->gyro_sensitivity;
        gyro->z = raw_data.z * dev->gyro_sensitivity;
    }

    rt_mutex_release(&dev->lock);
    return ret;
}

rt_err_t bmi088_read_temperature(bmi088_device_t *dev, float *temperature)
{
    rt_err_t ret;
    uint8_t raw[2];
    int16_t temp_raw;

    RT_ASSERT(dev != RT_NULL);
    RT_ASSERT(temperature != RT_NULL);

    rt_mutex_take(&dev->lock, RT_WAITING_FOREVER);

    ret = bmi088_acc_read_reg(dev, BMI088_ACC_TEMP_MSB, raw, 2);
    if (ret == RT_EOK) {
        temp_raw = (int16_t)((raw[0] << 3) | (raw[1] >> 5));
        if (temp_raw > 1023) {
            temp_raw -= 2048;
        }
        *temperature = temp_raw * 0.125f + 23.0f;
    }

    rt_mutex_release(&dev->lock);
    return ret;
}

rt_err_t bmi088_read_all(bmi088_device_t *dev, bmi088_data_t *data)
{
    rt_err_t ret;

    RT_ASSERT(dev != RT_NULL);
    RT_ASSERT(data != RT_NULL);

    data->timestamp = rt_tick_get();

    ret = bmi088_read_accel(dev, &data->accel);
    if (ret != RT_EOK) return ret;

    ret = bmi088_read_gyro(dev, &data->gyro);
    if (ret != RT_EOK) return ret;

    ret = bmi088_read_temperature(dev, &data->temperature);

    return ret;
}

bool bmi088_is_connected(bmi088_device_t *dev)
{
    uint8_t acc_id, gyro_id;
    rt_err_t ret;

    ret = bmi088_acc_read_reg(dev, BMI088_ACC_CHIP_ID, &acc_id, 1);
    if (ret != RT_EOK || acc_id != BMI088_ACC_CHIP_ID_VALUE) {
        return false;
    }

    ret = bmi088_gyro_read_reg(dev, BMI088_GYRO_CHIP_ID, &gyro_id, 1);
    if (ret != RT_EOK || gyro_id != BMI088_GYRO_CHIP_ID_VALUE) {
        return false;
    }

    return true;
}

rt_err_t bmi088_reset(bmi088_device_t *dev)
{
    rt_err_t ret;

    RT_ASSERT(dev != RT_NULL);

    rt_mutex_take(&dev->lock, RT_WAITING_FOREVER);

    ret = bmi088_acc_write_reg(dev, BMI088_ACC_SOFTRESET, 0xB6);
    rt_thread_mdelay(BMI088_RESET_DELAY_MS);

    ret |= bmi088_gyro_write_reg(dev, BMI088_GYRO_SOFTRESET, 0xB6);
    rt_thread_mdelay(BMI088_RESET_DELAY_MS);

    rt_mutex_release(&dev->lock);

    return ret;
}

rt_err_t bmi088_self_test(bmi088_device_t *dev)
{
    /* Simplified self-test: verify chip IDs */
    if (bmi088_is_connected(dev)) {
        LOG_I("BMI088 self-test passed");
        return RT_EOK;
    }
    LOG_E("BMI088 self-test failed");
    return -RT_ERROR;
}
