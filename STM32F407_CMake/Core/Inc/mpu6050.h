#ifndef MPU6050_H
#define MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* Du lieu sau khi doc va quy doi */
typedef struct {
    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;

    float ax, ay, az;      /* gia toc, don vi g        */
    float gx, gy, gz;      /* toc do goc, do/s         */
    float temp;            /* nhiet do, do C           */

    float roll, pitch;     /* goc nghieng, do (complementary filter) */
} MPU6050_t;

/* Khoi tao cam bien. Tra ve HAL_OK neu thanh cong. */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);

/* Doc 14 byte (accel + temp + gyro) trong mot lan, tinh roll/pitch.
 * Tra ve HAL_OK neu thanh cong. Khi loi, du lieu cu trong *d khong bi sua. */
HAL_StatusTypeDef MPU6050_ReadAll(I2C_HandleTypeDef *hi2c, MPU6050_t *d);

/* Do offset gyro luc dung yen (khoang 1 giay), goi sau Init. */
HAL_StatusTypeDef MPU6050_CalibrateGyro(I2C_HandleTypeDef *hi2c);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */