#include "balance.h"
#include "motor.h"
#include "pid.h"
#include <math.h>

/* =====================================================================
 *  CONFIG  (adjust after looking at your own logs)
 * ===================================================================== */

/* Which tilt axis is the balance axis? 0 = pitch (gyro Y), 1 = roll (gyro X) */
#define BAL_USE_ROLL        0

/* Make "angle > 0" mean "robot leans FORWARD" (the direction the wheels must
 * drive to get back under the body). Flip to -1 if the sign is wrong. */
#define BAL_ANGLE_SIGN      (+1.0f)

#define BAL_DEADBAND_DEG   5.0f   /* Suppress chatter within +-0.15 degrees */
#define BAL_DEADZONE_DEG    1.0f  /* Suppress chatter within +-0.15 degrees */

/* Balance angle in the sign-corrected frame (degrees). This is the angle where
 * the robot stands by itself (centre of mass above the axle). Tune by hand. */
#define BAL_SETPOINT_DEG    2.1f

/* Safety */
#define BAL_FALL_CUTOFF     0       /* 1 = stop motors when fallen; 0 = wheels keep running even if fallen */
#define BAL_FALL_ANGLE      35.0f   /* only used when BAL_FALL_CUTOFF = 1 */
#define BAL_ARM_BAND        3.0f    /* must be within +-3 deg of setpoint ... */
#define BAL_ARM_MS          500U    /* ... for 0.5 s to (re)start balancing */
#define BAL_IMU_FAIL_MAX    20      /* consecutive IMU read failures (20 x 5 ms = 100 ms) -> motors off.
                                       Raised from 3 so a single I2C glitch from motor noise does not stop the wheels. */

/* Starting gains. Output unit = Motor_Set units (+-1000), angle in degrees.
 * Tune in this order: Kp up until it starts oscillating, Kd to damp it,
 * then a small Ki for steady drift.
 * Kp 300 -> 200 and Kd 6 -> 5: the log still shows bursts of +-1000 chatter
 * (gyro +-200 deg/s) caused by too much high-frequency gain for the loop delay.
 * Raise Kp again step by step only while the robot stays calm. */
#define BAL_KP              200.0f // multiplier for debugging
#define BAL_KI              20.0f // og 1.0
#define BAL_KD              1.0f // og 5.0
#define BAL_I_LIMIT         300.0f
#define BAL_OUT_LIMIT       ((float)MOTOR_SPEED_MAX)

/* Anti-chatter */
#define BAL_SLEW_PER_CYCLE  400.0f  /* max output change per 5 ms cycle (was 200: it added too much lag) */
#define BAL_RATE_LPF        0.6f    /* low-pass factor for the gyro used by the D term (1.0 = no filter) */

/* Yaw-rate damping: counters unwanted rotation of the robot (unequal wheels).
 * turn = BAL_YAW_SIGN * BAL_YAW_KP * gz.  Set BAL_YAW_KP = 0 to disable.
 * If the rotation gets WORSE, flip BAL_YAW_SIGN to -1. */
#define BAL_YAW_KP          4.0f    /* output units per deg/s */
#define BAL_YAW_SIGN        (+1.0f)
#define BAL_YAW_DEADBAND    0.25f   /* deg/s: ignore gyro noise, correct small chassis drift */
#define BAL_YAW_LIMIT       250.0f  /* max correction (output units) */
#define BAL_YAW_LPF         0.3f

/* ===================================================================== */

static PID_Controller pid;
static float    setpoint = BAL_SETPOINT_DEG;
static int16_t  turn_cmd = 0;
static uint8_t  armed = 0;
static uint32_t arm_t0 = 0;
static uint8_t  imu_fail = 0;
static float    last_out = 0.0f;
static float    u_prev = 0.0f;          /* previous output, for the slew limit */
static float    rate_f = 0.0f;          /* filtered gyro rate for the D term */
static uint8_t  rate_init = 0;
static float    yaw_f = 0.0f;           /* filtered yaw rate */
static uint8_t  yaw_init = 0;

void Balance_Init(void)
{
    PID_Init(&pid, BAL_KP, BAL_KI, BAL_KD, BAL_I_LIMIT, BAL_OUT_LIMIT);
    armed = 0;
    arm_t0 = 0;
    imu_fail = 0;
    last_out = 0.0f;
    Motor_Coast();
}

void Balance_SetGains(float kp, float ki, float kd)
{
    pid.kp = kp;
    pid.ki = ki;
    pid.kd = kd;
}

void Balance_SetSetpoint(float deg) { setpoint = deg; }
void Balance_SetTurn(int16_t turn)  { turn_cmd = turn; }
uint8_t Balance_IsArmed(void)       { return armed; }
float Balance_GetOutput(void)       { return last_out; }

static void disarm(void)
{
    armed = 0;
    arm_t0 = 0;
    last_out = 0.0f;
    PID_Reset(&pid);
    Motor_Coast();
}

void Balance_Update(const MPU6050_sensor *imu, uint8_t imu_ok)
{
    /* ---- IMU watchdog ---- */
    if (!imu_ok) {
        if (++imu_fail >= BAL_IMU_FAIL_MAX) disarm();
        return;                         /* keep last motor command for 1-2 cycles */
    }
    imu_fail = 0;

    /* ---- measurement ---- */
    #if BAL_USE_ROLL
        float angle = BAL_ANGLE_SIGN * imu->roll;
        float rate  = BAL_ANGLE_SIGN * imu->gx_fast;
    #else
        float angle = BAL_ANGLE_SIGN * imu->pitch;
        float rate  = BAL_ANGLE_SIGN * imu->gy_fast;
    #endif
    
    float err = angle - setpoint;       /* > 0: leaning forward -> drive forward */
    /* --- Deadband Buffer --- */
    float gain_scale = fabsf(err) / BAL_DEADBAND_DEG;
    if (gain_scale > 1.0f) gain_scale = 1.0f;
    if (fabsf(err) < BAL_DEADZONE_DEG) gain_scale = 0.0f;  /* suppress chatter within +-0.15 degrees */
    uint32_t now = HAL_GetTick();

    /* ---- disarmed: wait until the robot is held upright ---- */
    if (!armed) {
        Motor_Coast();
        u_prev = 0.0f;
        rate_init = 0;
        yaw_init = 0;
        if (fabsf(err) < BAL_ARM_BAND) {
            if (arm_t0 == 0) arm_t0 = now;
            if ((now - arm_t0) >= BAL_ARM_MS) {
                PID_Reset(&pid);
                armed = 1;
            }
        } else {
            arm_t0 = 0;
        }
        return;
    }

    /* ---- fallen over (optional cut-off, disabled by default) ---- */
    #if BAL_FALL_CUTOFF
        if (fabsf(err) > BAL_FALL_ANGLE) {
            disarm();
            return;
        }
    #endif

    /* ---- PID ---- */
    float dt = BAL_LOOP_MS / 1000.0f;

    /* low-pass the gyro rate used by the D term (chassis vibration makes it very noisy) */
    if (!rate_init) { rate_f = rate; rate_init = 1; }
    else            rate_f += BAL_RATE_LPF * (rate - rate_f);

    float u = PID_Update(&pid, err, rate_f, dt);
    u *= gain_scale;

    /* slew limit: the output may change by at most BAL_SLEW_PER_CYCLE per cycle,
     * so the wheels cannot flip from full forward to full reverse in one step */
    float du = u - u_prev;
    if (du >  BAL_SLEW_PER_CYCLE) du =  BAL_SLEW_PER_CYCLE;
    if (du < -BAL_SLEW_PER_CYCLE) du = -BAL_SLEW_PER_CYCLE;
    u = u_prev + du;
    u_prev = u;
    last_out = u;

    /* yaw-rate damping: counter any unwanted rotation of the robot */
    if (!yaw_init) { yaw_f = imu->gz_fast; yaw_init = 1; }
    else           yaw_f += BAL_YAW_LPF * (imu->gz_fast - yaw_f);

    float yaw_in = (fabsf(yaw_f) < BAL_YAW_DEADBAND) ? 0.0f : yaw_f;
    float yaw_turn = BAL_YAW_SIGN * BAL_YAW_KP * yaw_in;
    if (yaw_turn >  BAL_YAW_LIMIT) yaw_turn =  BAL_YAW_LIMIT;
    if (yaw_turn < -BAL_YAW_LIMIT) yaw_turn = -BAL_YAW_LIMIT;

    int32_t base = (int32_t)u;
    int32_t turn = (int32_t)turn_cmd + (int32_t)yaw_turn;
    int32_t base_mag = (base < 0) ? -base : base;
    int32_t max_turn = base_mag - MOTOR_MIN_ACTIVE_COMMAND;
    if (max_turn < 0) max_turn = 0;
    if (turn >  max_turn) turn =  max_turn;
    if (turn < -max_turn) turn = -max_turn;

    /* Keep differential steering from cancelling the balance command on one
     * wheel near upright; both wheels must remain above the motor deadband. */
    int32_t l = base + turn;
    int32_t r = base - turn;
    if (l >  MOTOR_SPEED_MAX) l =  MOTOR_SPEED_MAX;
    if (l < -MOTOR_SPEED_MAX) l = -MOTOR_SPEED_MAX;
    if (r >  MOTOR_SPEED_MAX) r =  MOTOR_SPEED_MAX;
    if (r < -MOTOR_SPEED_MAX) r = -MOTOR_SPEED_MAX;

    Motor_Set((int16_t)l, (int16_t)r);
}