/**
 * Binary state telemetry — USART1 polling, 100 Hz.
 * Protocol: [0xDD] [16×float32 LE] [XOR checksum] = 1 + 64 + 1 = 66 bytes
 * Floats: theta_L,theta_R,theta_1,theta_2,thetadot_L/R/dot_1/dot_2,u_L,u_R,
 *         Target_theta_L,Target_theta_R, TargetVal_L,TargetVal_R, PWM_L,PWM_R
 */

#ifndef __RL_SEND_H
#define __RL_SEND_H

#include "sys.h"

#define RL_PACKET_SIZE  66
#define RL_SYNC         0xDD

void RL_Send_Init(void);
void RL_Send_Data(void);

#endif
