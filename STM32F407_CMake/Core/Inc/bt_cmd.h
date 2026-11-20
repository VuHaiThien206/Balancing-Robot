#ifndef BT_CMD_H
#define BT_CMD_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Start receiving commands from the HM-10 (USART1 RX DMA, circular).
 * Initial setpoint and gains are read from balance.c, so nothing is hard-coded here;
 * the web page can change them at run time with SP= / KP= / KI= / KD=. */
void BtCmd_Init(UART_HandleTypeDef *huart);

/* Call every main-loop iteration: parses received bytes + command-timeout failsafe. */
void BtCmd_Process(void);

/* Non-blocking: sends "P:<pitch>,O:<pid out>,T:<target>\n" (skipped if UART is busy). */
void BtCmd_SendTelemetry(float pitch_deg, float pid_out);

#ifdef __cplusplus
}
#endif

#endif /* BT_CMD_H */