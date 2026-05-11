/**
 * Binary state telemetry — DMA USART3 TX at 200 Hz, non-blocking.
 * Packets: 42 bytes (sync + 10×float32 + checksum) at 921600 baud = 0.36ms TX time.
 * DMA runs in background; 5ms ISR only needs ~5us to fire.
 */
#include "rl_send.h"
#include "usart.h"

/* Variables from control.c */
extern float theta_L, theta_R, theta_1, theta_2;
extern float theta_L_dot, theta_R_dot, theta_dot_1, theta_dot_2;
extern float u_L, u_R;

static uint8_t tx_buf[RL_PACKET_SIZE];
static volatile uint8_t tx_busy = 0;

void RL_Send_Init(void) {
    tx_busy = 0;
}

void RL_Send_Data(void) {
    if (tx_busy)
        return; /* previous DMA still in flight — skip this 5ms tick */

    float *floats = (float *)(tx_buf + 1);
    tx_buf[0] = RL_SYNC;

    floats[0] = theta_L;
    floats[1] = theta_R;
    floats[2] = theta_1;
    floats[3] = theta_2;
    floats[4] = theta_L_dot;
    floats[5] = theta_R_dot;
    floats[6] = theta_dot_1;
    floats[7] = theta_dot_2;
    floats[8] = u_L;
    floats[9] = u_R;

    /* XOR checksum over bytes 0..40 */
    uint8_t ck = 0;
    for (int i = 0; i < RL_PACKET_SIZE - 1; i++)
        ck ^= tx_buf[i];
    tx_buf[RL_PACKET_SIZE - 1] = ck;

    tx_busy = 1;
    HAL_UART_Transmit_DMA(&huart3, tx_buf, RL_PACKET_SIZE);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        tx_busy = 0;
    }
}
