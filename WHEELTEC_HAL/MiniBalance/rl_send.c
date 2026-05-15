/**
 * Binary state telemetry + external control receive — USART1, 100 Hz.
 *
 * TX (STM32->PC): 16×float32 + sync + checksum = 66 bytes. @460800 = 1.43ms/pkt.
 * RX (PC->STM32): [0xCC] [u_L float32 LE] [u_R float32 LE] = 10 bytes.
 */
#include "rl_send.h"
#include "usart.h"

extern float theta_L, theta_R, theta_1, theta_2;
extern float theta_L_dot, theta_R_dot, theta_dot_1, theta_dot_2;
extern float u_L, u_R;
extern float Target_theta_L, Target_theta_R;
extern float TargetVal_L, TargetVal_R;
extern int PWM_L, PWM_R;

float ext_u_L = 0, ext_u_R = 0;
u8 ext_u_fresh = 0;

static uint8_t tx_buf[RL_PACKET_SIZE];
static uint8_t rx_buf[EXT_CTRL_SIZE];
static u8 rx_idx = 0;
static u16 ext_timeout = 0;

void RL_Send_Init(void) {
    ext_u_L = 0; ext_u_R = 0; ext_u_fresh = 0;
    rx_idx = 0; ext_timeout = 0;
}

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
    f[12] = TargetVal_L;
    f[13] = TargetVal_R;
    f[14] = (float)PWM_L;
    f[15] = (float)PWM_R;

    uint8_t ck = 0;
    for (int i = 0; i < RL_PACKET_SIZE - 1; i++)
        ck ^= tx_buf[i];
    tx_buf[RL_PACKET_SIZE - 1] = ck;

    for (int i = 0; i < RL_PACKET_SIZE; i++) {
        while (!(USART1->SR & USART_SR_TXE));
        USART1->DR = tx_buf[i];
    }
}

/* Poll USART1 RX for PC-u commands.  Call once per 10ms control cycle. */
void RL_Recv_ExtCtrl(void) {
    while (USART1->SR & USART_SR_RXNE) {
        uint8_t b = USART1->DR;
        if (rx_idx == 0 && b == EXT_CTRL_SYNC) {
            rx_buf[0] = b; rx_idx = 1;
        } else if (rx_idx > 0 && rx_idx < EXT_CTRL_SIZE) {
            rx_buf[rx_idx++] = b;
            if (rx_idx == EXT_CTRL_SIZE) {
                float *f = (float *)(rx_buf + 1);
                ext_u_L = f[0]; ext_u_R = f[1];
                ext_u_fresh = 1; ext_timeout = 0; rx_idx = 0;
                rx_idx = 0;
            }
        } else {
            rx_idx = 0;
        }
    }
    // Timeout: fallback to internal LQR after EXT_TIMEOUT_MAX cycles
    if (ext_timeout < EXT_TIMEOUT_MAX)
        ext_timeout++;
    else
        ext_u_fresh = 0;
}
