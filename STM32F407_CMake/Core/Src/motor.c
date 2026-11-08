#include "motor.h"
#include "main.h"
#include <stdlib.h>

/* ---------------------------------------------------------------------
 *  Pin map (use these User Labels in CubeMX, or the fallbacks below)
 *    Left  motor : ENA = TIM3_CH1 (PA6), IN1 = PB0,  IN2 = PB1
 *    Right motor : ENB = TIM3_CH2 (PA7), IN3 = PB12, IN4 = PB13
 * --------------------------------------------------------------------- */
#ifndef MOT_L_IN1_Pin
#define MOT_L_IN1_GPIO_Port GPIOB
#define MOT_L_IN1_Pin       GPIO_PIN_0
#endif
#ifndef MOT_L_IN2_Pin
#define MOT_L_IN2_GPIO_Port GPIOB
#define MOT_L_IN2_Pin       GPIO_PIN_1
#endif
#ifndef MOT_R_IN3_Pin
#define MOT_R_IN3_GPIO_Port GPIOB
#define MOT_R_IN3_Pin       GPIO_PIN_12
#endif
#ifndef MOT_R_IN4_Pin
#define MOT_R_IN4_GPIO_Port GPIOB
#define MOT_R_IN4_Pin       GPIO_PIN_13
#endif

/* ---- Tuning ---- */
/* Minimum PWM used to overcome static friction. Calibrate per motor by
 * increasing the duty until the wheel starts moving reliably. */
#define MOTOR_DEADZONE      700

/* Ignore tiny PID commands caused by sensor noise. Above this threshold,
 * apply the friction-compensated PWM mapping. */
/* ---- Demo tuning ---- */
#define DEMO_PEAK_SPEED     MOTOR_SPEED_MAX /* full speed (1000) */
#define DEMO_RAMP_MS        0       /* 0 = jump straight to full speed (no ramp) */
#define DEMO_HOLD_MS        30000   /* time held at peak speed */
#define DEMO_STOP_MS        1500    /* brake pause before reversing (protects driver/motor) */

/* Per-wheel strength trim in per mille (1000 = 100 %). If the robot keeps turning
 * to one side, lower the scale of the STRONGER wheel (e.g. 900 = 90 %). */
#define MOTOR_LEFT_SCALE    1000
#define MOTOR_RIGHT_SCALE   1000

typedef struct {
    GPIO_TypeDef *port_a;  uint16_t pin_a;   /* IN1 / IN3 */
    GPIO_TypeDef *port_b;  uint16_t pin_b;   /* IN2 / IN4 */
    uint32_t      channel;                   /* PWM channel (ENA / ENB) */
    uint8_t       inverted;
    uint16_t      scale;                     /* per-wheel trim, per mille */
} motor_t;

static motor_t mot_left  = { MOT_L_IN1_GPIO_Port, MOT_L_IN1_Pin,
                             MOT_L_IN2_GPIO_Port, MOT_L_IN2_Pin,
                             TIM_CHANNEL_1, 0, MOTOR_LEFT_SCALE };
static motor_t mot_right = { MOT_R_IN3_GPIO_Port, MOT_R_IN3_Pin,
                             MOT_R_IN4_GPIO_Port, MOT_R_IN4_Pin,
                             TIM_CHANNEL_2, 0, MOTOR_RIGHT_SCALE };

static TIM_HandleTypeDef *tim = NULL;

/* duty: 0..MOTOR_SPEED_MAX, scaled to the timer's real ARR so the code works
 * with any timer clock / PWM frequency set in CubeMX. */
static void set_duty(const motor_t *m, uint32_t duty)
{
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(tim) + 1U;
    uint32_t ccr = (duty * period) / MOTOR_SPEED_MAX;
    if (ccr > period) ccr = period;            /* CCR > ARR = 100 % */
    __HAL_TIM_SET_COMPARE(tim, m->channel, ccr);
}

static void coast_one(const motor_t *m)
{
    set_duty(m, 0);
    HAL_GPIO_WritePin(m->port_a, m->pin_a, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(m->port_b, m->pin_b, GPIO_PIN_RESET);
}

static void brake_one(const motor_t *m)
{
    HAL_GPIO_WritePin(m->port_a, m->pin_a, GPIO_PIN_SET);
    HAL_GPIO_WritePin(m->port_b, m->pin_b, GPIO_PIN_SET);
    set_duty(m, MOTOR_SPEED_MAX);
}

static void drive_one(const motor_t *m, int16_t u)
{
    if (u > MOTOR_SPEED_MAX)  u = MOTOR_SPEED_MAX;
    if (u < -MOTOR_SPEED_MAX) u = -MOTOR_SPEED_MAX;

    if (u == 0) {
        coast_one(m);
        return;
    }

    uint8_t forward = (u > 0);
    if (m->inverted) forward = !forward;

    uint32_t mag = (uint32_t)abs(u);
    if (mag <= MOTOR_COMMAND_DEADBAND) {
        coast_one(m);
        return;
    }

    /* Compensate for static friction instead of slowly ramping through a PWM
     * range where the motor only hums but does not turn. */
    mag = MOTOR_DEADZONE
        + ((mag - MOTOR_COMMAND_DEADBAND) * (MOTOR_SPEED_MAX - MOTOR_DEADZONE))
          / (MOTOR_SPEED_MAX - MOTOR_COMMAND_DEADBAND);

    mag = (mag * m->scale) / 1000U;                 /* per-wheel trim */
    if (mag > MOTOR_SPEED_MAX) mag = MOTOR_SPEED_MAX;

    HAL_GPIO_WritePin(m->port_a, m->pin_a, forward ? GPIO_PIN_SET   : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(m->port_b, m->pin_b, forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
    set_duty(m, mag);
}

void Motor_Init(TIM_HandleTypeDef *htim)
{
    tim = htim;
    HAL_TIM_PWM_Start(tim, mot_left.channel);
    HAL_TIM_PWM_Start(tim, mot_right.channel);
    Motor_Coast();
}

void Motor_Set(int16_t left, int16_t right)
{
    if (tim == NULL) return;
    drive_one(&mot_left,  left);
    drive_one(&mot_right, right);
}

void Motor_Brake(void)
{
    if (tim == NULL) return;
    brake_one(&mot_left);
    brake_one(&mot_right);
}

void Motor_Coast(void)
{
    if (tim == NULL) return;
    coast_one(&mot_left);
    coast_one(&mot_right);
}

void Motor_SetInvert(uint8_t left_inverted, uint8_t right_inverted)
{
    mot_left.inverted  = left_inverted;
    mot_right.inverted = right_inverted;
}

/* ---------------------------------------------------------------------
 *  Demo: ramp up -> stop -> ramp up -> stop ... (non-blocking)
 * --------------------------------------------------------------------- */
void Motor_DemoUpdate(void)
{
    enum { PH_RAMP, PH_HOLD, PH_STOP };

    static uint8_t  started = 0;
    static uint8_t  phase = PH_RAMP;
    static int8_t   dir = 1;           /* +1 forward, -1 reverse */
    static uint32_t t0 = 0;

    uint32_t now = HAL_GetTick();
    if (!started) {
        started = 1;
        t0 = now;
    }

    uint32_t el = now - t0;

    switch (phase) {
    case PH_RAMP: {                    /* slow -> fast */
        if (el >= DEMO_RAMP_MS) {
            phase = PH_HOLD;
            t0 = now;
            Motor_Set(dir * DEMO_PEAK_SPEED, dir * DEMO_PEAK_SPEED);
            break;
        }
        int16_t sp = (int16_t)(((int32_t)DEMO_PEAK_SPEED * (int32_t)el) / DEMO_RAMP_MS);
        Motor_Set(dir * sp, dir * sp); /* both wheels, same direction */
        break;
    }
    case PH_HOLD:                      /* keep peak speed for 30 s */
        if (el >= DEMO_HOLD_MS) {
            Motor_Brake();
            phase = PH_STOP;
            t0 = now;
        }
        break;

    case PH_STOP:                      /* short brake, then reverse direction */
        if (el >= DEMO_STOP_MS) {
            dir = -dir;
            phase = PH_RAMP;
            t0 = now;
        }
        break;
    }
}