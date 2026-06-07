#ifndef EPS_GPIO_H
#define EPS_GPIO_H

#include "main.h"
#define CHARGE_EN_PORT GPIOA
#define CHARGE_EN_PIN GPIO_PIN_0

void GPIO_Loads_Init(void);
void EnableAllLoads(void);
void EnableCriticalLoads(void);
void DisableNonEssential(void);
void DisableAllLoads(void);
void ChargeEnable(void);
void ChargeDisable(void);

uint8_t GPIO_GetLoadCh1State(void);
uint8_t GPIO_GetLoadCh2State(void);
uint8_t GPIO_GetChargeState(void);

#endif

