#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32f4xx_hal.h"

#define MPU6050_ADDR        (0x68 << 1)
#define MPU6050_REG_PWR     0x6B
#define MPU6050_REG_ACCEL   0x3B
#define MPU6050_REG_GYRO    0x43
#define MPU6050_REG_WHO_AM_I 0x75

typedef struct {
    float roll;
    float pitch;
    float accelMag;
} MPU6050_Data;

typedef struct {
    float ax_g;
    float ay_g;
    float az_g;
    float gx_dps;
    float gy_dps;
    float gz_dps;
    float roll_acc;
    float pitch_acc;
    float accelMag;
} MPU6050_Sample;

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MPU6050_Read(I2C_HandleTypeDef *hi2c, MPU6050_Data *data);
HAL_StatusTypeDef MPU6050_Check(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, uint8_t *who_am_i);
HAL_StatusTypeDef MPU6050_InitAt(I2C_HandleTypeDef *hi2c, uint16_t dev_addr);
HAL_StatusTypeDef MPU6050_ReadAt(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, MPU6050_Data *data);
HAL_StatusTypeDef MPU6050_ReadSampleAt(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, MPU6050_Sample *sample);

#endif /* INC_MPU6050_H_ */
