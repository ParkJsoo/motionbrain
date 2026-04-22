#include "mpu6050.h"
#include <math.h>

#define ACCEL_SCALE 16384.0f  // ±2g
#define GYRO_SCALE  131.0f    // ±250°/s

HAL_StatusTypeDef MPU6050_Check(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, uint8_t *who_am_i)
{
    uint8_t who = 0;
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(hi2c, dev_addr, MPU6050_REG_WHO_AM_I,
                                             I2C_MEMADD_SIZE_8BIT, &who, 1, 100);

    if (who_am_i != NULL) {
        *who_am_i = who;
    }

    if (ret != HAL_OK || who != 0x68) {
        return HAL_ERROR;
    }

    return ret;
}

HAL_StatusTypeDef MPU6050_InitAt(I2C_HandleTypeDef *hi2c, uint16_t dev_addr)
{
    uint8_t val = 0x00;  // wake up, use internal 8MHz oscillator

    if (MPU6050_Check(hi2c, dev_addr, NULL) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Write(hi2c, dev_addr, MPU6050_REG_PWR,
                             I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

HAL_StatusTypeDef MPU6050_ReadSampleAt(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, MPU6050_Sample *sample)
{
    uint8_t buf[14];
    HAL_StatusTypeDef ret;

    if (sample == NULL) {
        return HAL_ERROR;
    }

    ret = HAL_I2C_Mem_Read(hi2c, dev_addr, MPU6050_REG_ACCEL,
                           I2C_MEMADD_SIZE_8BIT, buf, 14, 100);
    if (ret != HAL_OK) {
        return ret;
    }

    int16_t ax = (int16_t)(buf[0]  << 8 | buf[1]);
    int16_t ay = (int16_t)(buf[2]  << 8 | buf[3]);
    int16_t az = (int16_t)(buf[4]  << 8 | buf[5]);
    int16_t gx = (int16_t)(buf[8]  << 8 | buf[9]);
    int16_t gy = (int16_t)(buf[10] << 8 | buf[11]);
    int16_t gz = (int16_t)(buf[12] << 8 | buf[13]);

    sample->ax_g = ax / ACCEL_SCALE;
    sample->ay_g = ay / ACCEL_SCALE;
    sample->az_g = az / ACCEL_SCALE;
    sample->gx_dps = gx / GYRO_SCALE;
    sample->gy_dps = gy / GYRO_SCALE;
    sample->gz_dps = gz / GYRO_SCALE;
    sample->roll_acc = atan2f(sample->ay_g, sample->az_g) * 180.0f / M_PI;
    sample->pitch_acc = atan2f(-sample->ax_g,
                               sqrtf(sample->ay_g * sample->ay_g +
                                     sample->az_g * sample->az_g)) * 180.0f / M_PI;
    sample->accelMag = sqrtf(sample->ax_g * sample->ax_g +
                             sample->ay_g * sample->ay_g +
                             sample->az_g * sample->az_g);

    return ret;
}

HAL_StatusTypeDef MPU6050_ReadAt(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, MPU6050_Data *data)
{
    MPU6050_Sample sample = {0};
    HAL_StatusTypeDef ret;

    if (data == NULL) {
        return HAL_ERROR;
    }

    ret = MPU6050_ReadSampleAt(hi2c, dev_addr, &sample);
    if (ret != HAL_OK) {
        return ret;
    }

    data->roll = sample.roll_acc;
    data->pitch = sample.pitch_acc;
    data->accelMag = sample.accelMag;

    return ret;
}

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    return MPU6050_InitAt(hi2c, MPU6050_ADDR);
}

HAL_StatusTypeDef MPU6050_Read(I2C_HandleTypeDef *hi2c, MPU6050_Data *data)
{
    return MPU6050_ReadAt(hi2c, MPU6050_ADDR, data);
}
