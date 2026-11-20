#include "mpu6050.h"
#include <math.h>
#include <string.h>

/* ---- Config ---- */
#define MPU6050_ADDR        (0x68 << 1)   /* AD0 = GND. If AD0 = 3.3V: (0x69 << 1) */
#define MPU6050_TIMEOUT_MS  10U     /* was 100: a failed read must not stall the 200 Hz loop for 100 ms */

/* ---- Registers ---- */
#define REG_SMPLRT_DIV      0x19
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_XOUT_H    0x3B
#define REG_GYRO_XOUT_H     0x43
#define REG_PWR_MGMT_1      0x6B
#define REG_WHO_AM_I        0x75

/* ---- Conversion factors (accel +-2g, gyro +-250 deg/s) ---- */
#define ACCEL_LSB_PER_G     16384.0f
#define GYRO_LSB_PER_DPS    131.0f
#define RAD_TO_DEG_F        57.2957795f

/* =====================================================================
 *  FILTER TUNING  (values derived from the log: still sensor, 31 C)
 *  Measured noise (1 sigma): accel ~0.002-0.003 g, gyro x ~0.15 dps,
 *  gyro y ~0.06 dps, gyro z ~0.05 dps, accel angle ~0.1 deg.
 *  Residual gyro bias after old calibration: x ~ -0.2, y ~ -0.6 dps.
 * ===================================================================== */

/* 1) DLPF inside the MPU: CONFIG = 4 -> accel 21 Hz / gyro 20 Hz (gyro rate 1 kHz)
 *    (was 3 = 44/42 Hz). Output rate stays 200 Hz (1 kHz / (1+4)). */
#define MPU_DLPF_CFG        0x02        /* 94/98 Hz: lowest delay that is still usable (was 0x03 = 44/42 Hz) */
#define MPU_SMPLRT_DIV      0x04
#define SAMPLE_PERIOD_MS    5U              /* 200 Hz, used by calibration loops */

/* 3) Median filter window (odd). 5 samples kills single-sample spikes
 *    (e.g. the sample with gx=-3.29, gy=-1.63 in the log). */
#define MED_N               3           /* 3 = only 1 sample of delay (was 5) */

/* 4) EMA: y += alpha * (x - y). Smaller alpha = smoother but more lag.
 *    The gyro is already filtered by DLPF + median, so only a light EMA. */
#define EMA_ALPHA_ACC       0.25f
#define EMA_ALPHA_GYRO      0.80f       /* less lag on the gyro (D term) */

/* 5) Deadband (about 2-3 sigma of the noise), unit deg/s */
#define DEADBAND_GX         0.40f
#define DEADBAND_GY         0.20f
#define DEADBAND_GZ         0.15f
#define ANGLE_DEADBAND      0.03f           /* output angle holds if change < 0.03 deg */

/* 6) Kalman (angle + gyro bias), one per axis */
#define USE_KALMAN          1               /* 0 = fall back to complementary filter */
#define KF_Q_ANGLE          0.001f          /* process noise of angle */
#define KF_Q_BIAS           0.003f          /* process noise of gyro bias */
#define KF_R_MEASURE        0.03f           /* variance of accel angle (deg^2), measured ~0.015 */
#define COMP_ALPHA          0.98f           /* used only when USE_KALMAN = 0 */

/* Calibration limits */
#define CAL_SAMPLES         400             /* 400 x 5 ms = 2 s */
#define CAL_MAX_STD_DPS     8.0f            /* gyro std above this = sensor is moving */
#define CAL_MAX_STD_G       0.2f           /* accel std above this = sensor is moving */

/* ---- 2) Offsets ---- */
static float gyro_off_x = 0.0f, gyro_off_y = 0.0f, gyro_off_z = 0.0f;   /* deg/s */
static float acc_off_x  = 0.0f, acc_off_y  = 0.0f, acc_off_z  = 0.0f;   /* g */

/* ---- Median filter ---- */
typedef struct {
    float   buf[MED_N];
    uint8_t idx, cnt;
} median_t;

static float median_push(median_t *m, float v)
{
    float tmp[MED_N];
    uint8_t n, i, j;

    m->buf[m->idx] = v;
    m->idx = (uint8_t)((m->idx + 1) % MED_N);
    if (m->cnt < MED_N) m->cnt++;

    n = m->cnt;
    for (i = 0; i < n; i++) tmp[i] = m->buf[i];
    for (i = 1; i < n; i++) {                 /* insertion sort */
        float key = tmp[i];
        j = i;
        while (j > 0 && tmp[j - 1] > key) { tmp[j] = tmp[j - 1]; j--; }
        tmp[j] = key;
    }
    return tmp[n / 2];
}

/* ---- Kalman (angle + bias) ---- */
typedef struct {
    float angle, bias;
    float P[2][2];
} kalman_t;

static void kalman_init(kalman_t *k, float angle)
{
    k->angle = angle;
    k->bias  = 0.0f;
    k->P[0][0] = 1.0f; k->P[0][1] = 0.0f;
    k->P[1][0] = 0.0f; k->P[1][1] = 1.0f;
}

static float kalman_update(kalman_t *k, float meas_angle, float gyro_rate, float dt)
{
    /* Predict */
    float rate = gyro_rate - k->bias;
    k->angle += dt * rate;

    k->P[0][0] += dt * (dt * k->P[1][1] - k->P[0][1] - k->P[1][0] + KF_Q_ANGLE);
    k->P[0][1] -= dt * k->P[1][1];
    k->P[1][0] -= dt * k->P[1][1];
    k->P[1][1] += KF_Q_BIAS * dt;

    /* Update */
    float S  = k->P[0][0] + KF_R_MEASURE;
    float K0 = k->P[0][0] / S;
    float K1 = k->P[1][0] / S;
    float y  = meas_angle - k->angle;

    k->angle += K0 * y;
    k->bias  += K1 * y;

    float P00 = k->P[0][0], P01 = k->P[0][1];
    k->P[0][0] -= K0 * P00;
    k->P[0][1] -= K0 * P01;
    k->P[1][0] -= K1 * P00;
    k->P[1][1] -= K1 * P01;

    return k->angle;
}

/* ---- Filter state ---- */
static median_t  med[6];                 /* ax ay az gx gy gz */
static float     ema_y[6];
static kalman_t  kf_roll, kf_pitch;
static uint8_t   filter_ready = 0;
static uint32_t  last_tick    = 0;
static float     out_roll = 0.0f, out_pitch = 0.0f;
static float     roll_f = 0.0f, pitch_f = 0.0f;       /* complementary fallback */

static void reset_filters(void)
{
    memset(med, 0, sizeof(med));
    memset(ema_y, 0, sizeof(ema_y));
    filter_ready = 0;
}

/* ---- Online gyro bias tracking (zero-velocity update) ----
 * If the sensor stays still for STILL_WIN samples (gyro + |accel| almost constant),
 * the mean gyro output is pure bias -> move the offset toward it. */
#define STILL_WIN           100
#define STILL_RANGE_DPS     0.6f      /* max (max-min) of each gyro axis in the window */
#define STILL_RANGE_G       0.02f     /* max (max-min) of |accel| in the window */
#define STILL_MAX_BIAS_DPS  10.0f     /* never "correct" more than this */
#define STILL_GAIN          0.5f

static struct { float mn[4], mx[4], sum[3]; uint16_t n; } still;

static void still_track(void)
{
    float amag = sqrtf(ema_y[0]*ema_y[0] + ema_y[1]*ema_y[1] + ema_y[2]*ema_y[2]);
    float v[4] = { ema_y[3], ema_y[4], ema_y[5], amag };

    if (still.n == 0) {
        for (int k = 0; k < 4; k++) { still.mn[k] = still.mx[k] = v[k]; }
        still.sum[0] = still.sum[1] = still.sum[2] = 0.0f;
    }
    for (int k = 0; k < 4; k++) {
        if (v[k] < still.mn[k]) still.mn[k] = v[k];
        if (v[k] > still.mx[k]) still.mx[k] = v[k];
        float th = (k < 3) ? STILL_RANGE_DPS : STILL_RANGE_G;
        if (still.mx[k] - still.mn[k] > th) { still.n = 0; return; }   /* moving */
    }
    for (int k = 0; k < 3; k++) still.sum[k] += v[k];

    if (++still.n < STILL_WIN) return;
    still.n = 0;

    float c[3];
    for (int k = 0; k < 3; k++) {
        c[k] = STILL_GAIN * still.sum[k] / STILL_WIN;
        if (fabsf(c[k]) > STILL_MAX_BIAS_DPS) return;
    }
    gyro_off_x += c[0];  gyro_off_y += c[1];  gyro_off_z += c[2];
    for (int k = 0; k < 3; k++) ema_y[3 + k] -= c[k];     /* keep output continuous */
    kf_roll.bias  -= c[0];                                 /* Kalman bias is relative to offset */
    kf_pitch.bias -= c[1];
}

static inline float deadband(float v, float th)
{
    return (fabsf(v) < th) ? 0.0f : v;
}

static HAL_StatusTypeDef write_reg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                             &val, 1, MPU6050_TIMEOUT_MS);
}

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t id = 0;

    if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, REG_WHO_AM_I, I2C_MEMADD_SIZE_8BIT,
                         &id, 1, MPU6050_TIMEOUT_MS) != HAL_OK)
        return HAL_ERROR;

    /* 0x68: MPU6050 ready. 0x70/0x72: some clone modules or MPU6500 */
    if (id != 0x68 && id != 0x70 && id != 0x72)
        return HAL_ERROR;

    if (write_reg(hi2c, REG_PWR_MGMT_1, 0x80) != HAL_OK)   /* reset device */
        return HAL_ERROR;
    HAL_Delay(100);

    if (write_reg(hi2c, REG_PWR_MGMT_1, 0x01) != HAL_OK)   /* wake up, clock = PLL gyro X */
        return HAL_ERROR;
    HAL_Delay(10);

    if (write_reg(hi2c, REG_CONFIG, MPU_DLPF_CFG) != HAL_OK)     /* DLPF */
        return HAL_ERROR;
    if (write_reg(hi2c, REG_SMPLRT_DIV, MPU_SMPLRT_DIV) != HAL_OK) /* 1kHz / (1+4) = 200Hz */
        return HAL_ERROR;
    if (write_reg(hi2c, REG_ACCEL_CONFIG, 0x00) != HAL_OK) /* +-2g */
        return HAL_ERROR;
    if (write_reg(hi2c, REG_GYRO_CONFIG, 0x00) != HAL_OK)  /* +-250 dps */
        return HAL_ERROR;

    reset_filters();
    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadAll(I2C_HandleTypeDef *hi2c, MPU6050_sensor *d)
{
    uint8_t buf[14];

    if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, REG_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT,
                         buf, sizeof(buf), MPU6050_TIMEOUT_MS) != HAL_OK)
        return HAL_ERROR;

    d->ax_raw = (int16_t)((buf[0]  << 8) | buf[1]);
    d->ay_raw = (int16_t)((buf[2]  << 8) | buf[3]);
    d->az_raw = (int16_t)((buf[4]  << 8) | buf[5]);
    int16_t t = (int16_t)((buf[6]  << 8) | buf[7]);
    d->gx_raw = (int16_t)((buf[8]  << 8) | buf[9]);
    d->gy_raw = (int16_t)((buf[10] << 8) | buf[11]);
    d->gz_raw = (int16_t)((buf[12] << 8) | buf[13]);

    d->temp = t / 340.0f + 36.53f;

    /* low-latency gyro (offset removed only) */
    d->gx_fast = d->gx_raw / GYRO_LSB_PER_DPS - gyro_off_x;
    d->gy_fast = d->gy_raw / GYRO_LSB_PER_DPS - gyro_off_y;
    d->gz_fast = d->gz_raw / GYRO_LSB_PER_DPS - gyro_off_z;

    /* ---- Step 1: convert + offset ---- */
    float x[6];
    x[0] = d->ax_raw / ACCEL_LSB_PER_G  - acc_off_x;
    x[1] = d->ay_raw / ACCEL_LSB_PER_G  - acc_off_y;
    x[2] = d->az_raw / ACCEL_LSB_PER_G  - acc_off_z;
    x[3] = d->gx_raw / GYRO_LSB_PER_DPS - gyro_off_x;
    x[4] = d->gy_raw / GYRO_LSB_PER_DPS - gyro_off_y;
    x[5] = d->gz_raw / GYRO_LSB_PER_DPS - gyro_off_z;

    /* ---- Step 2: median (spike rejection) -> Step 3: EMA ---- */
    for (int i = 0; i < 6; i++) {
        float m = median_push(&med[i], x[i]);
        float a = (i < 3) ? EMA_ALPHA_ACC : EMA_ALPHA_GYRO;
        if (!filter_ready) ema_y[i] = m;
        else               ema_y[i] += a * (m - ema_y[i]);
    }

    if (filter_ready) still_track();

    d->ax = ema_y[0];
    d->ay = ema_y[1];
    d->az = ema_y[2];

    /* ---- Step 4: deadband on gyro ---- */
    d->gx = deadband(ema_y[3], DEADBAND_GX);
    d->gy = deadband(ema_y[4], DEADBAND_GY);
    d->gz = deadband(ema_y[5], DEADBAND_GZ);

    /* ---- Roll/Pitch from accelerometer ---- */
    float acc_roll  = atan2f(d->ay, d->az) * RAD_TO_DEG_F;
    float acc_pitch = atan2f(-d->ax, sqrtf(d->ay * d->ay + d->az * d->az)) * RAD_TO_DEG_F;
    d->roll_acc  = acc_roll;
    d->pitch_acc = acc_pitch;

    /* ---- Step 5: Kalman fusion ---- */
    uint32_t now = HAL_GetTick();
    if (!filter_ready) {
        kalman_init(&kf_roll,  acc_roll);
        kalman_init(&kf_pitch, acc_pitch);
        roll_f = acc_roll;
        pitch_f = acc_pitch;
        out_roll = acc_roll;
        out_pitch = acc_pitch;
        filter_ready = 1;
    } else {
        float dt = (now - last_tick) / 1000.0f;
        if (dt <= 0.0f || dt > 0.5f) dt = 0.0f;   /* ignore invalid dt */

#if USE_KALMAN
        /* handle +-180 deg wrap of roll */
        if ((acc_roll < -90.0f && kf_roll.angle > 90.0f) ||
            (acc_roll >  90.0f && kf_roll.angle < -90.0f)) {
            kf_roll.angle = acc_roll;
        }
        roll_f  = kalman_update(&kf_roll,  acc_roll,  d->gx, dt);
        pitch_f = kalman_update(&kf_pitch, acc_pitch, d->gy, dt);
#else
        roll_f  = COMP_ALPHA * (roll_f  + d->gx * dt) + (1.0f - COMP_ALPHA) * acc_roll;
        pitch_f = COMP_ALPHA * (pitch_f + d->gy * dt) + (1.0f - COMP_ALPHA) * acc_pitch;
#endif
        /* output deadband: hold the angle if the change is tiny */
        if (fabsf(roll_f  - out_roll)  > ANGLE_DEADBAND) out_roll  = roll_f;
        if (fabsf(pitch_f - out_pitch) > ANGLE_DEADBAND) out_pitch = pitch_f;
    }
    last_tick = now;

    d->roll  = out_roll;
    d->pitch = out_pitch;
    d->bias_x = kf_roll.bias;
    d->bias_y = kf_pitch.bias;
    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_CalibrateGyro(I2C_HandleTypeDef *hi2c)
{
    float s[3] = {0}, ss[3] = {0};
    uint8_t buf[6];
    int ok = 0;

    /* NOTE: offsets are NOT cleared here: this loop reads raw registers, and on
     * failure (sensor moving / I2C error) the previous offsets must stay valid. */

    for (int i = 0; i < CAL_SAMPLES; i++) {
        if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, REG_GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT,
                             buf, sizeof(buf), MPU6050_TIMEOUT_MS) == HAL_OK) {
            float v[3];
            v[0] = (int16_t)((buf[0] << 8) | buf[1]) / GYRO_LSB_PER_DPS;
            v[1] = (int16_t)((buf[2] << 8) | buf[3]) / GYRO_LSB_PER_DPS;
            v[2] = (int16_t)((buf[4] << 8) | buf[5]) / GYRO_LSB_PER_DPS;
            for (int k = 0; k < 3; k++) { s[k] += v[k]; ss[k] += v[k] * v[k]; }
            ok++;
        }
        HAL_Delay(SAMPLE_PERIOD_MS);      /* match the 200 Hz output rate (no duplicate samples) */
    }

    if (ok < CAL_SAMPLES / 2)
        return HAL_ERROR;

    float mean[3];
    for (int k = 0; k < 3; k++) {
        mean[k] = s[k] / ok;
        float var = ss[k] / ok - mean[k] * mean[k];
        if (var < 0.0f) var = 0.0f;
        if (sqrtf(var) > CAL_MAX_STD_DPS)   /* sensor was moving */
            return HAL_ERROR;
    }

    gyro_off_x = mean[0];
    gyro_off_y = mean[1];
    gyro_off_z = mean[2];
    reset_filters();
    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_CalibrateAccelLevel(I2C_HandleTypeDef *hi2c)
{
    float s[3] = {0}, ss[3] = {0};
    uint8_t buf[6];
    int ok = 0;

    for (int i = 0; i < CAL_SAMPLES; i++) {
        if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, REG_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT,
                             buf, sizeof(buf), MPU6050_TIMEOUT_MS) == HAL_OK) {
            float v[3];
            v[0] = (int16_t)((buf[0] << 8) | buf[1]) / ACCEL_LSB_PER_G;
            v[1] = (int16_t)((buf[2] << 8) | buf[3]) / ACCEL_LSB_PER_G;
            v[2] = (int16_t)((buf[4] << 8) | buf[5]) / ACCEL_LSB_PER_G;
            for (int k = 0; k < 3; k++) { s[k] += v[k]; ss[k] += v[k] * v[k]; }
            ok++;
        }
        HAL_Delay(SAMPLE_PERIOD_MS);
    }

    if (ok < CAL_SAMPLES / 2)
        return HAL_ERROR;

    float mean[3];
    for (int k = 0; k < 3; k++) {
        mean[k] = s[k] / ok;
        float var = ss[k] / ok - mean[k] * mean[k];
        if (var < 0.0f) var = 0.0f;
        if (sqrtf(var) > CAL_MAX_STD_G)
            return HAL_ERROR;
    }

    acc_off_x = mean[0];            /* expected 0 g */
    acc_off_y = mean[1];            /* expected 0 g */
    acc_off_z = mean[2] - 1.0f;     /* expected +1 g */
    reset_filters();
    return HAL_OK;
}

void MPU6050_SetAccelOffset(float ox, float oy, float oz)
{
    acc_off_x = ox;
    acc_off_y = oy;
    acc_off_z = oz;
    reset_filters();
}