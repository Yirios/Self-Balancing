/**
 * Binary state telemetry — polling USART1 TX (USB) at 200 Hz.
 * 42 bytes @ 921600 = 0.36ms, well within 5ms ISR budget.
 * No DMA, no IRQ — simple and reliable.
 */
#include "rl_send.h"
#include "usart.h"

extern float theta_L, theta_R, theta_1, theta_2;
extern float theta_L_dot, theta_R_dot, theta_dot_1, theta_dot_2;
extern float u_L, u_R;

static uint8_t tx_buf[RL_PACKET_SIZE];

void RL_Send_Init(void) {}

void RL_Send_Data(void) {
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

    uint8_t ck = 0;
    for (int i = 0; i < RL_PACKET_SIZE - 1; i++)
        ck ^= tx_buf[i];
    tx_buf[RL_PACKET_SIZE - 1] = ck;

    /* polling send — 0.36ms at 921600, no DMA dependencies */
    for (int i = 0; i < RL_PACKET_SIZE; i++) {
        while (!(USART1->SR & USART_SR_TXE));
        USART1->DR = tx_buf[i];
    }
}
