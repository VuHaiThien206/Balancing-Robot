#ifndef MOTOR_H
#define MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* Speed range for Motor_Set(): -MOTOR_SPEED_MAX .. +MOTOR_SPEED_MAX
 * (positive = forward, negative = reverse). */
#define MOTOR_SPEED_MAX     1000

/* Start the PWM channels (CH1 = left/ENA, CH2 = right/ENB) of the given timer
 * and put both motors in coast. The timer must already be initialised by
 * CubeMX (MX_TIM3_Init). */
void Motor_Init(TIM_HandleTypeDef *htim);

/* Set speed of both motors. Values are clamped to +-MOTOR_SPEED_MAX.
 * 0 = coast (free-wheel). Non-zero values are mapped above the motor
 * dead zone (MOTOR_DEADZONE in motor.c). */
void Motor_Set(int16_t left, int16_t right);

/* Active brake: both inputs high, enable at 100%. */
void Motor_Brake(void);

/* Free-wheel: enable = 0, inputs low. */
void Motor_Coast(void);

/* Invert the direction of a motor if its wires are swapped. */
void Motor_SetInvert(uint8_t left_inverted, uint8_t right_inverted);

/* Non-blocking demo: both wheels same direction, speed ramps up slowly to the
 * peak, holds it for 30 s, brakes briefly, then reverses direction and repeats
 * forever (forward, reverse, forward, ...). Call this repeatedly from the main
 * loop (every ~10 ms is enough). */
void Motor_DemoUpdate(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H */