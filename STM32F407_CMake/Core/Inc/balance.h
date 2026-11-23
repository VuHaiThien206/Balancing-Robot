#ifndef BALANCE_H
#define BALANCE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "mpu6050.h"

#define BAL_LOOP_MS    5U      // 200hz // og5U

void    Balance_Init(void);

/* Call every BAL_LOOP_MS. imu_ok = 1 if MPU6050_ReadAll() succeeded this cycle. */
void    Balance_Update(const MPU6050_sensor *imu, uint8_t imu_ok);

void    Balance_SetGains(float kp, float ki, float kd);
void    Balance_SetSetpoint(float deg);     /* balance angle (in the sign-corrected frame) */
void    Balance_SetTurn(int16_t turn);      /* differential steering, 0 = straight */

uint8_t Balance_IsArmed(void);
float   Balance_GetOutput(void);            /* last PID output (-1000..1000) */

#ifdef __cplusplus
}
#endif

#endif /* BALANCE_H */