#ifndef EPS_WATCHDOG_H
#define EPS_WATCHDOG_H

#include "main.h"

typedef enum
{
    WDG_REASON_COLD = 0,
    WDG_REASON_IWDG,
    WDG_REASON_PIN,
    WDG_REASON_BROWNOUT,
    WDG_REASON_SOFTWARE,
    WDG_REASON_UNKNOWN
} WatchdogResetReason;

void WDG_Init(void);
void WDG_Refresh(void);
WatchdogResetReason WDG_GetResetReason(void);
WatchdogResetReason WDG_GetLastResetReason(void);

#endif
