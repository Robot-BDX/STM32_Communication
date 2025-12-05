/**
 * @file bmi088.h
 * @brief BMI088 IMU Driver for STM32H723 DM-MC02
 * @author BDX_SIm Project
 * @version 1.0
 *
 * Driver for Bosch BMI088 6-axis IMU (Accelerometer + Gyroscope)
 * Connected via SPI2 on DM-MC02 board
 */

#ifndef __BMI088_H__
#define __BMI088_H__

#include <rtthread.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============== BMI088 Register Definitions ============== */

/* Accelerometer Registers */
#define BMI088_ACC_CHIP_ID          0x00
#define BMI088_ACC_ERR_REG          0x02
#define BMI088_ACC_STATUS           0x03
#define BMI088_ACC_X_LSB            0x12
#define BMI088_ACC_X_MSB            0x13
#define BMI088_ACC_Y_LSB            0x14
#define BMI088_ACC_Y_MSB            0x15
#define BMI088_ACC_Z_LSB            0x16
#define BMI088_ACC_Z_MSB            0x17
#define BMI088_ACC_SENSORTIME_0     0x18
#define BMI088_ACC_INT_STAT_1       0x1D
#define BMI088_ACC_TEMP_MSB         0x22
#define BMI088_ACC_TEMP_LSB         0x23
#define BMI088_ACC_CONF             0x40
#define BMI088_ACC_RANGE            0x41
#define BMI088_ACC_INT1_IO_CONF     0x53
#define BMI088_ACC_INT2_IO_CONF     0x54
#define BMI088_ACC_INT1_INT2_MAP    0x58
#define BMI088_ACC_SELF_TEST        0x6D
#define BMI088_ACC_PWR_CONF         0x7C
#define BMI088_ACC_PWR_CTRL         0x7D
#define BMI088_ACC_SOFTRESET        0x7E

/* Gyroscope Registers */
#define BMI088_GYRO_CHIP_ID         0x00
#define BMI088_GYRO_X_LSB           0x02
#define BMI088_GYRO_X_MSB           0x03
#define BMI088_GYRO_Y_LSB           0x04
#define BMI088_GYRO_Y_MSB           0x05
#define BMI088_GYRO_Z_LSB           0x06
#define BMI088_GYRO_Z_MSB           0x07
#define BMI088_GYRO_INT_STAT_1      0x0A
#define BMI088_GYRO_RANGE           0x0F
#define BMI088_GYRO_BANDWIDTH       0x10
#define BMI088_GYRO_LPM1            0x11
#define BMI088_GYRO_SOFTRESET       0x14
#define BMI088_GYRO_INT_CTRL        0x15
#define BMI088_GYRO_INT3_INT4_IO    0x16
#define BMI088_GYRO_INT3_INT4_MAP   0x18
#define BMI088_GYRO_SELF_TEST       0x3C

/* Chip IDs */
#define BMI088_ACC_CHIP_ID_VALUE    0x1E
#define BMI088_GYRO_CHIP_ID_VALUE   0x0F

/* Accelerometer Range */
typedef enum {
    BMI088_ACC_RANGE_3G  = 0x00,  /* +/- 3g  */
    BMI088_ACC_RANGE_6G  = 0x01,  /* +/- 6g  */
    BMI088_ACC_RANGE_12G = 0x02,  /* +/- 12g */
    BMI088_ACC_RANGE_24G = 0x03   /* +/- 24g */
} bmi088_acc_range_t;

/* Gyroscope Range */
typedef enum {
    BMI088_GYRO_RANGE_2000DPS = 0x00,  /* +/- 2000 deg/s */
    BMI088_GYRO_RANGE_1000DPS = 0x01,  /* +/- 1000 deg/s */
    BMI088_GYRO_RANGE_500DPS  = 0x02,  /* +/- 500 deg/s  */
    BMI088_GYRO_RANGE_250DPS  = 0x03,  /* +/- 250 deg/s  */
    BMI088_GYRO_RANGE_125DPS  = 0x04   /* +/- 125 deg/s  */
} bmi088_gyro_range_t;

/* Accelerometer ODR (Output Data Rate) */
typedef enum {
    BMI088_ACC_ODR_12_5HZ = 0x05,
    BMI088_ACC_ODR_25HZ   = 0x06,
    BMI088_ACC_ODR_50HZ   = 0x07,
    BMI088_ACC_ODR_100HZ  = 0x08,
    BMI088_ACC_ODR_200HZ  = 0x09,
    BMI088_ACC_ODR_400HZ  = 0x0A,
    BMI088_ACC_ODR_800HZ  = 0x0B,
    BMI088_ACC_ODR_1600HZ = 0x0C
} bmi088_acc_odr_t;

/* Gyroscope ODR */
typedef enum {
    BMI088_GYRO_ODR_2000HZ_532HZ = 0x00,
    BMI088_GYRO_ODR_2000HZ_230HZ = 0x01,
    BMI088_GYRO_ODR_1000HZ_116HZ = 0x02,
    BMI088_GYRO_ODR_400HZ_47HZ   = 0x03,
    BMI088_GYRO_ODR_200HZ_23HZ   = 0x04,
    BMI088_GYRO_ODR_100HZ_12HZ   = 0x05,
    BMI088_GYRO_ODR_200HZ_64HZ   = 0x06,
    BMI088_GYRO_ODR_100HZ_32HZ   = 0x07
} bmi088_gyro_odr_t;

/* ============== Data Structures ============== */

/**
 * @brief Raw IMU data (16-bit signed)
 */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} bmi088_raw_data_t;

/**
 * @brief Scaled IMU data (float, physical units)
 */
typedef struct {
    float x;
    float y;
    float z;
} bmi088_scaled_data_t;

/**
 * @brief Complete IMU reading
 */
typedef struct {
    bmi088_scaled_data_t accel;      /* Acceleration in m/s² */
    bmi088_scaled_data_t gyro;       /* Angular velocity in rad/s */
    float temperature;                /* Temperature in °C */
    uint32_t timestamp;               /* RT-Thread tick timestamp */
} bmi088_data_t;

/**
 * @brief BMI088 Configuration
 */
typedef struct {
    bmi088_acc_range_t acc_range;
    bmi088_gyro_range_t gyro_range;
    bmi088_acc_odr_t acc_odr;
    bmi088_gyro_odr_t gyro_odr;
} bmi088_config_t;

/**
 * @brief BMI088 Device Handle
 */
typedef struct {
    SPI_HandleTypeDef *hspi;         /* SPI handle */
    GPIO_TypeDef *acc_cs_port;       /* Accelerometer CS GPIO port */
    uint16_t acc_cs_pin;             /* Accelerometer CS pin */
    GPIO_TypeDef *gyro_cs_port;      /* Gyroscope CS GPIO port */
    uint16_t gyro_cs_pin;            /* Gyroscope CS pin */
    bmi088_config_t config;          /* Current configuration */
    float acc_sensitivity;           /* Accelerometer sensitivity factor */
    float gyro_sensitivity;          /* Gyroscope sensitivity factor */
    bool initialized;                /* Initialization status */
    struct rt_mutex lock;            /* Thread safety mutex */
} bmi088_device_t;

/* ============== API Functions ============== */

/**
 * @brief Initialize BMI088 device
 * @param dev Device handle
 * @param hspi SPI handle (SPI2)
 * @param acc_cs_port Accelerometer CS GPIO port
 * @param acc_cs_pin Accelerometer CS pin
 * @param gyro_cs_port Gyroscope CS GPIO port
 * @param gyro_cs_pin Gyroscope CS pin
 * @return RT_EOK on success, error code otherwise
 */
rt_err_t bmi088_init(bmi088_device_t *dev,
                     SPI_HandleTypeDef *hspi,
                     GPIO_TypeDef *acc_cs_port, uint16_t acc_cs_pin,
                     GPIO_TypeDef *gyro_cs_port, uint16_t gyro_cs_pin);

/**
 * @brief Configure BMI088 parameters
 * @param dev Device handle
 * @param config Configuration structure
 * @return RT_EOK on success
 */
rt_err_t bmi088_configure(bmi088_device_t *dev, const bmi088_config_t *config);

/**
 * @brief Read all IMU data (accelerometer + gyroscope + temperature)
 * @param dev Device handle
 * @param data Output data structure
 * @return RT_EOK on success
 */
rt_err_t bmi088_read_all(bmi088_device_t *dev, bmi088_data_t *data);

/**
 * @brief Read accelerometer only
 * @param dev Device handle
 * @param accel Output accelerometer data
 * @return RT_EOK on success
 */
rt_err_t bmi088_read_accel(bmi088_device_t *dev, bmi088_scaled_data_t *accel);

/**
 * @brief Read gyroscope only
 * @param dev Device handle
 * @param gyro Output gyroscope data
 * @return RT_EOK on success
 */
rt_err_t bmi088_read_gyro(bmi088_device_t *dev, bmi088_scaled_data_t *gyro);

/**
 * @brief Read temperature
 * @param dev Device handle
 * @param temperature Output temperature in °C
 * @return RT_EOK on success
 */
rt_err_t bmi088_read_temperature(bmi088_device_t *dev, float *temperature);

/**
 * @brief Perform self-test
 * @param dev Device handle
 * @return RT_EOK if self-test passed
 */
rt_err_t bmi088_self_test(bmi088_device_t *dev);

/**
 * @brief Software reset
 * @param dev Device handle
 * @return RT_EOK on success
 */
rt_err_t bmi088_reset(bmi088_device_t *dev);

/**
 * @brief Check if BMI088 is connected and responsive
 * @param dev Device handle
 * @return true if device responds correctly
 */
bool bmi088_is_connected(bmi088_device_t *dev);

/**
 * @brief Get default configuration
 * @param config Configuration to fill
 */
void bmi088_get_default_config(bmi088_config_t *config);

#ifdef __cplusplus
}
#endif

#endif /* __BMI088_H__ */
