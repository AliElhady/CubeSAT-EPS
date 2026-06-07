/* ============================================================
 *  eps_fsm.h
 * ============================================================ */

#ifndef EPS_FSM_H
#define EPS_FSM_H

#include "main.h"

typedef enum
{
    STATE_BOOT = 0,
    STATE_POWER_CHECK,
    STATE_CHARGING,
    STATE_NORMAL,
    STATE_LOW_POWER,
    STATE_CRITICAL,
    STATE_HIBERNATE,
    STATE_ERROR,
    STATE_COUNT
} EPS_State;

void FSM_Update(float vbat, float vsol);

EPS_State FSM_GetState(void);
int FSM_GetSOC(void);
float FSM_GetBattVoltage(void);
float FSM_GetSolarVoltage(void);
const char *FSM_GetStateName(void);

uint8_t FSM_IsCharging(void);
uint8_t FSM_IsFullCharge(void);
uint8_t FSM_IsError(void);

void FSM_SetError(void);
void FSM_SetReset(void);

#endif
