/**
 * Binary state telemetry + external control — USART1, 100 Hz.
 *
 * STM32 -> PC: [0xDD] [16×float32 LE] [XOR] = 66 bytes
 * PC -> STM32: [0xCC] [u_L float32 LE] [u_R float32 LE] = 10 bytes
 */

#ifndef __RL_SEND_H
#define __RL_SEND_H

#include "sys.h"

#define RL_PACKET_SIZE  66
#define RL_SYNC         0xDD

#define EXT_CTRL_SYNC   0xCC
#define EXT_CTRL_SIZE   9
#define EXT_TIMEOUT_MAX 50    // 500ms timeout -> fallback to LQR

extern float ext_u_L, ext_u_R;
extern u8 ext_u_fresh;

void RL_Send_Init(void);
void RL_Send_Data(void);
void RL_Recv_ExtCtrl(void);   // poll USART1 RX, parse PC u commands

#endif
