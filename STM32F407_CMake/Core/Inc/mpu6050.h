#ifndef MPU6050_H
#define MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

typedef struct {
    /* raw data read from sensor */
    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;

    float ax, ay, az;      /* acceleration, unit: g */
    float gx, gy, gz;      /* angular velocity, degrees/s */
    float temp;            /* temperature, degrees C */

    float roll, pitch;     /* tilt angles, degrees (complementary filter) */
} MPU6050_sensor;

/* Initialize the sensor. Returns HAL_OK if successful. */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);

/* Read 14 bytes (accel + temp + gyro) in one go, calculate roll/pitch.
 * Returns HAL_OK if successful. On error, the data in *d remains unchanged. */
HAL_StatusTypeDef MPU6050_ReadAll(I2C_HandleTypeDef *hi2c, MPU6050_sensor *d);

/* Calibrate the gyroscope. Call this after Init. */
HAL_StatusTypeDef MPU6050_CalibrateGyro(I2C_HandleTypeDef *hi2c);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */