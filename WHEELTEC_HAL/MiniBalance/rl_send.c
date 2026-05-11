/**
 * Binary state telemetry — polling USART1 TX at 100 Hz.
 * 12×float32 + sync + checksum = 50 bytes. @460800 = 1.09ms per packet.
 */
#include "rl_send.h"
#include "usart.h"

extern float theta_L, theta_R, theta_1, theta_2;
extern float theta_L_dot, theta_R_dot, theta_dot_1, theta_dot_2;
extern float u_L, u_R;
extern float Target_theta_L, Target_theta_R;

static uint8_t tx_buf[RL_PACKET_SIZE];

void RL_Send_Init(void) {}

void RL_Send_Data(void) {
    float *f = (float *)(tx_buf + 1);
    tx_buf[0] = RL_SYNC;

    f[0]  = theta_L;
    f[1]  = theta_R;
    f[2]  = theta_1;
    f[3]  = theta_2;
    f[4]  = theta_L_dot;
    f[5]  = theta_R_dot;
    f[6]  = theta_dot_1;
    f[7]  = theta_dot_2;
    f[8]  = u_L;
    f[9]  = u_R;
    f[10] = Target_theta_L;
    f[11] = Target_theta_R;

    uint8_t ck = 0;
    for (int i = 0; i < RL_PACKET_SIZE - 1; i++)
        ck ^= tx_buf[i];
    tx_buf[RL_PACKET_SIZE - 1] = ck;

    for (int i = 0; i < RL_PACKET_SIZE; i++) {
        while (!(USART1->SR & USART_SR_TXE));
        USART1->DR = tx_buf[i];
    }
}
