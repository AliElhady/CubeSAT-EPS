#ifndef EPS_TELEMETRY_H
#define EPS_TELEMETRY_H
#include "main.h"
#include "eps_fsm.h"
#include "eps_flash.h"

void Telem_Init(void);
void Telem_TickSecond(void);
void Telem_Update(void);
void Telem_SendNow(void);

#endif
