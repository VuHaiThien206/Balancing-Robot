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

    /* Data after: offset -> median -> EMA -> deadband (gyro) */
    float ax, ay, az;      /* acceleration, unit: g */
    float gx, gy, gz;      /* angular velocity, degrees/s */
    float temp;            /* temperature, degrees C */

    float roll, pitch;     /* tilt angles, degrees (Kalman filter + output deadband) */
    float roll_acc, pitch_acc; /* angles from accelerometer only (for debugging) */
    float bias_x, bias_y;      /* gyro bias estimated online by Kalman (deg/s) */
} MPU6050_sensor;

/* Initialize the sensor. Returns HAL_OK if successful. */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);

/* Read 14 bytes (accel + temp + gyro) in one go, filter, calculate roll/pitch.
 * Returns HAL_OK if successful. On error, the data in *d remains unchanged. */
HAL_StatusTypeDef MPU6050_ReadAll(I2C_HandleTypeDef *hi2c, MPU6050_sensor *d);

/* Calibrate the gyroscope. Keep the sensor STILL. Call after Init (wait ~1-2 min
 * warm-up for best result). Returns HAL_ERROR if the sensor moved during calibration. */
HAL_StatusTypeDef MPU6050_CalibrateGyro(I2C_HandleTypeDef *hi2c);

/* Calibrate accel offset assuming the board lies FLAT (Z axis up = +1g).
 * Only use if the board is really level; otherwise use MPU6050_SetAccelOffset. */
HAL_StatusTypeDef MPU6050_CalibrateAccelLevel(I2C_HandleTypeDef *hi2c);

/* Set accel offset manually (unit: g), e.g. from a 6-position calibration. */
void MPU6050_SetAccelOffset(float ox, float oy, float oz);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */