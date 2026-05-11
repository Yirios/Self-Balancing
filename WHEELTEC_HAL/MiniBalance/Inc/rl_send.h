/**
 * Binary state telemetry — USART1 polling, 100 Hz.
 * Protocol: [0xDD] [15×float32 LE] [XOR checksum] = 1 + 60 + 1 = 62 bytes
 * Floats: theta_L,theta_R,theta_1,theta_2,thetadot_L/R/dot_1/dot_2,u_L,u_R,
 *         Target_theta_L,Target_theta_R,Target_theta_dot_L,Target_theta_dot_R,Target_theta_1
 */

#ifndef __RL_SEND_H
#define __RL_SEND_H

#include "sys.h"

#define RL_PACKET_SIZE  62
#define RL_SYNC         0xDD

void RL_Send_Init(void);
void RL_Send_Data(void);

#endif
