/**
 * Binary state telemetry over USART3 DMA TX — uniform 200 Hz.
 * Protocol: [0xDD] [10×float32 LE] [XOR checksum] = 1 + 40 + 1 = 42 bytes
 * Floats: theta_L,theta_R,theta_1,theta_2,thetadot_L,thetadot_R,thetadot_1,thetadot_2,u_L,u_R
 */

#ifndef __RL_SEND_H
#define __RL_SEND_H

#include "sys.h"

#define RL_PACKET_SIZE  42
#define RL_SYNC         0xDD

void RL_Send_Init(void);
void RL_Send_Data(void);

#endif
