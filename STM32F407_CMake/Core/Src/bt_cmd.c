#include "bt_cmd.h"
#include "balance.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

/* Getters added to balance.c (same prototypes should also be in balance.h) */
float Balance_GetSetpoint(void);
void  Balance_GetGains(float *kp, float *ki, float *kd);

/* =====================================================================
 *  CONFIG
 * ===================================================================== */
#define BT_RX_BUF_SIZE      128
#define BT_LINE_MAX         32

#define BT_LEAN_DEG         2.0f    /* setpoint offset while F / B is held (deg). Flip sign if F drives backwards */
#define BT_TURN_CMD         150     /* turn command while L / R is held (output units). Flip sign if L/R are swapped */
#define BT_CMD_TIMEOUT_MS   400     /* no move command for this long -> stop (web resends every 100 ms) */

/* Limits for values received over Bluetooth */
#define BT_KP_MAX           100.0f
#define BT_KI_MAX           50.0f
#define BT_KD_MAX           20.0f
#define BT_SP_MAX           20.0f   /* base setpoint limit +-deg */

/* ===================================================================== */

static UART_HandleTypeDef *bt = NULL;

static uint8_t  rx_buf[BT_RX_BUF_SIZE];
static uint16_t rd_pos = 0;

static char    line[BT_LINE_MAX];
static uint8_t line_len = 0;
static uint8_t line_overflow = 0;

static float   base_sp, kp, ki, kd;
static float   lean_deg = 0.0f;     /* current offset added to base_sp */
static int16_t turn_cmd = 0;
static uint8_t moving = 0;
static uint32_t last_move_tick = 0;
static float   target_sp;           /* what is actually sent to Balance (for telemetry) */

static char tx_buf[48];

/* ---------------------------------------------------------------------
 *  Helpers
 * --------------------------------------------------------------------- */
static void start_rx(void)
{
    if (HAL_UART_Receive_DMA(bt, rx_buf, BT_RX_BUF_SIZE) == HAL_OK) {
        rd_pos = 0;
        line_len = 0;
        line_overflow = 0;
    }
}

static void apply_motion(void)
{
    target_sp = base_sp + lean_deg;
    Balance_SetSetpoint(target_sp);
    Balance_SetTurn(turn_cmd);
}

static void do_move(char c)
{
    switch (c) {
    case 'F': lean_deg =  BT_LEAN_DEG; turn_cmd = 0;            break;
    case 'B': lean_deg = -BT_LEAN_DEG; turn_cmd = 0;            break;
    case 'L': lean_deg = 0.0f;         turn_cmd = -BT_TURN_CMD; break;
    case 'R': lean_deg = 0.0f;         turn_cmd =  BT_TURN_CMD; break;
    default:  lean_deg = 0.0f;         turn_cmd = 0;            break;   /* 'S' */
    }
    moving = (c != 'S');
    last_move_tick = HAL_GetTick();
    apply_motion();
}

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static void exec_line(char *s)
{
    /* single-letter move commands */
    if (s[1] == '\0') {
        if (s[0] == 'F' || s[0] == 'B' || s[0] == 'L' || s[0] == 'R' || s[0] == 'S')
            do_move(s[0]);
        return;
    }

    /* KEY=value */
    char *eq = strchr(s, '=');
    if (eq == NULL) return;
    *eq = '\0';
    char *end;
    float v = strtof(eq + 1, &end);
    if (end == eq + 1) return;                    /* no number */

    if      (!strcmp(s, "KP")) { kp = clampf(v, 0.0f, BT_KP_MAX); Balance_SetGains(kp, ki, kd); }
    else if (!strcmp(s, "KI")) { ki = clampf(v, 0.0f, BT_KI_MAX); Balance_SetGains(kp, ki, kd); }
    else if (!strcmp(s, "KD")) { kd = clampf(v, 0.0f, BT_KD_MAX); Balance_SetGains(kp, ki, kd); }
    else if (!strcmp(s, "SP")) { base_sp = clampf(v, -BT_SP_MAX, BT_SP_MAX); apply_motion(); }
}

static void feed_byte(uint8_t c)
{
    if (c == '\n' || c == '\r') {
        if (line_len > 0 && !line_overflow) {
            line[line_len] = '\0';
            exec_line(line);
        }
        line_len = 0;
        line_overflow = 0;
    } else if (line_len < BT_LINE_MAX - 1) {
        line[line_len++] = (char)toupper(c);
    } else {
        line_overflow = 1;                        /* too long: drop until next newline */
    }
}

/* ---------------------------------------------------------------------
 *  Public API
 * --------------------------------------------------------------------- */
void BtCmd_Init(UART_HandleTypeDef *huart)
{
    bt = huart;
    base_sp = Balance_GetSetpoint();            /* start from whatever balance.c is using */
    Balance_GetGains(&kp, &ki, &kd);
    lean_deg = 0.0f;
    turn_cmd = 0;
    moving = 0;
    target_sp = base_sp;
    start_rx();
}

void BtCmd_Process(void)
{
    if (bt == NULL) return;

    /* bytes written so far by DMA (circular buffer) */
    uint16_t wr = (uint16_t)(BT_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(bt->hdmarx));
    if (wr >= BT_RX_BUF_SIZE) wr = 0;

    while (rd_pos != wr) {
        feed_byte(rx_buf[rd_pos]);
        if (++rd_pos >= BT_RX_BUF_SIZE) rd_pos = 0;
    }

    /* failsafe: lost link / button released without 'S' -> stand still */
    if (moving && (HAL_GetTick() - last_move_tick) > BT_CMD_TIMEOUT_MS)
        do_move('S');
}

static void fmt_tenths(char *b, size_t n, float v)
{
    int x = (int)lroundf(v * 10.0f);
    snprintf(b, n, "%s%d.%d", x < 0 ? "-" : "", abs(x) / 10, abs(x) % 10);
}

void BtCmd_SendTelemetry(float pitch_deg, float pid_out)
{
    if (bt == NULL || bt->gState != HAL_UART_STATE_READY) return;   /* still sending: skip */

    char p[12], t[12];
    fmt_tenths(p, sizeof p, pitch_deg);
    fmt_tenths(t, sizeof t, target_sp);
    int len = snprintf(tx_buf, sizeof tx_buf, "P:%s,O:%d,T:%s\n",
                       p, (int)lroundf(pid_out), t);
    if (len > 0) HAL_UART_Transmit_IT(bt, (uint8_t *)tx_buf, (uint16_t)len);
}

/* Overrun / framing error (e.g. motor noise): HAL stops the DMA, so restart it. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (bt != NULL && huart == bt) start_rx();
}