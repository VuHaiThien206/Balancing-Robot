#include "mpu6050.h"
#include <math.h>

/* ---- Cau hinh ---- */
#define MPU6050_ADDR        (0x68 << 1)   /* AD0 = GND. Neu AD0 = 3.3V: (0x69 << 1) */
#define MPU6050_TIMEOUT_MS  100U

/* ---- Thanh ghi ---- */
#define REG_SMPLRT_DIV      0x19
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_XOUT_H    0x3B
#define REG_PWR_MGMT_1      0x6B
#define REG_WHO_AM_I        0x75

/* ---- He so quy doi (accel +-2g, gyro +-250 do/s) ---- */
#define ACCEL_LSB_PER_G     16384.0f
#define GYRO_LSB_PER_DPS    131.0f

#define RAD_TO_DEG_F        57.2957795f
#define COMP_ALPHA          0.98f         /* ti trong cua gyro trong bo loc */

/* Offset gyro do duoc bang MPU6050_CalibrateGyro (don vi do/s) */
static float gyro_off_x = 0.0f, gyro_off_y = 0.0f, gyro_off_z = 0.0f;

/* Trang thai bo loc */
static uint8_t  filter_ready = 0;
static uint32_t last_tick    = 0;
static float    roll_f = 0.0f, pitch_f = 0.0f;

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

    /* 0x68: MPU6050 that. 0x70/0x72: mot so module clone hoac MPU6500 */
    if (id != 0x68 && id != 0x70 && id != 0x72)
        return HAL_ERROR;

    if (write_reg(hi2c, REG_PWR_MGMT_1, 0x80) != HAL_OK)   /* reset thiet bi */
        return HAL_ERROR;
    HAL_Delay(100);

    if (write_reg(hi2c, REG_PWR_MGMT_1, 0x01) != HAL_OK)   /* wake up, clock = PLL gyro X */
        return HAL_ERROR;
    HAL_Delay(10);

    if (write_reg(hi2c, REG_CONFIG, 0x03) != HAL_OK)       /* DLPF ~42Hz, gyro 1kHz */
        return HAL_ERROR;
    if (write_reg(hi2c, REG_SMPLRT_DIV, 0x04) != HAL_OK)   /* 1kHz / (1+4) = 200Hz */
        return HAL_ERROR;
    if (write_reg(hi2c, REG_ACCEL_CONFIG, 0x00) != HAL_OK) /* +-2g */
        return HAL_ERROR;
    if (write_reg(hi2c, REG_GYRO_CONFIG, 0x00) != HAL_OK)  /* +-250 do/s */
        return HAL_ERROR;

    filter_ready = 0;
    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadAll(I2C_HandleTypeDef *hi2c, MPU6050_t *d)
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

    d->ax = d->ax_raw / ACCEL_LSB_PER_G;
    d->ay = d->ay_raw / ACCEL_LSB_PER_G;
    d->az = d->az_raw / ACCEL_LSB_PER_G;

    d->gx = d->gx_raw / GYRO_LSB_PER_DPS - gyro_off_x;
    d->gy = d->gy_raw / GYRO_LSB_PER_DPS - gyro_off_y;
    d->gz = d->gz_raw / GYRO_LSB_PER_DPS - gyro_off_z;

    d->temp = t / 340.0f + 36.53f;

    /* ---- Goc roll/pitch tu accel ---- */
    float acc_roll  = atan2f(d->ay, d->az) * RAD_TO_DEG_F;
    float acc_pitch = atan2f(-d->ax, sqrtf(d->ay * d->ay + d->az * d->az)) * RAD_TO_DEG_F;

    uint32_t now = HAL_GetTick();
    if (!filter_ready) {
        roll_f = acc_roll;
        pitch_f = acc_pitch;
        filter_ready = 1;
    } else {
        float dt = (now - last_tick) / 1000.0f;
        if (dt <= 0.0f || dt > 0.5f) dt = 0.0f;   /* bo qua dt bat thuong */
        roll_f  = COMP_ALPHA * (roll_f  + d->gx * dt) + (1.0f - COMP_ALPHA) * acc_roll;
        pitch_f = COMP_ALPHA * (pitch_f + d->gy * dt) + (1.0f - COMP_ALPHA) * acc_pitch;
    }
    last_tick = now;

    d->roll  = roll_f;
    d->pitch = pitch_f;
    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_CalibrateGyro(I2C_HandleTypeDef *hi2c)
{
    const int N = 500;
    float sx = 0, sy = 0, sz = 0;
    uint8_t buf[6];
    int ok = 0;

    gyro_off_x = gyro_off_y = gyro_off_z = 0.0f;

    for (int i = 0; i < N; i++) {
        if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, 0x43, I2C_MEMADD_SIZE_8BIT,
                             buf, sizeof(buf), MPU6050_TIMEOUT_MS) == HAL_OK) {
            sx += (int16_t)((buf[0] << 8) | buf[1]) / GYRO_LSB_PER_DPS;
            sy += (int16_t)((buf[2] << 8) | buf[3]) / GYRO_LSB_PER_DPS;
            sz += (int16_t)((buf[4] << 8) | buf[5]) / GYRO_LSB_PER_DPS;
            ok++;
        }
        HAL_Delay(2);
    }

    if (ok < N / 2)
        return HAL_ERROR;

    gyro_off_x = sx / ok;
    gyro_off_y = sy / ok;
    gyro_off_z = sz / ok;
    filter_ready = 0;
    return HAL_OK;
}