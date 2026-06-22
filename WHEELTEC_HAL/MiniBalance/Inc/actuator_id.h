#ifndef __ACTUATOR_ID_H
#define __ACTUATOR_ID_H

#include "actuator_id_config.h"
#include "sys.h"

#if ACTUATOR_ID_MODE

typedef enum {
    ACT_ID_STATE_IDLE = 0,
    ACT_ID_STATE_ARMED = 1,
    ACT_ID_STATE_SPINUP = 2,
    ACT_ID_STATE_DRIVE_HOLD = 3,
    ACT_ID_STATE_COAST = 4,
    ACT_ID_STATE_BRAKE = 5,
    ACT_ID_STATE_COMPLETE = 6,
    ACT_ID_STATE_E_STOP = 7
} ActuatorIdState;

typedef enum {
    ACT_ID_BRIDGE_DRIVE = 0,
    ACT_ID_BRIDGE_COAST = 1,
    ACT_ID_BRIDGE_BRAKE = 2,
    ACT_ID_BRIDGE_OFF = 3
} ActuatorIdBridgeMode;

void ActuatorId_Init(void);
void ActuatorId_Process(void);
void ActuatorId_SysTick1ms(void);

#endif

#endif
