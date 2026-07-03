/* ============================================================
 *  eps_gpio.c — CubeSat EPS load control
 *
 *  Step 4: Implement load profiles for each FSM state.
 * ============================================================ */

#include "eps_gpio.h"
#include <string.h>

#define LOAD_ON GPIO_PIN_RESET
#define LOAD_OFF GPIO_PIN_SET

#define CHARGE_ON GPIO_PIN_SET
#define CHARGE_OFF GPIO_PIN_RESET

static uint8_t s_ch1_on = 0;
static uint8_t s_ch2_on = 0;
static uint8_t s_charge_on = 0;

static void set_ch1(uint8_t on) { (void)on; }

static void set_ch2(uint8_t on) { (void)on; }

static void set_charge(uint8_t on)
{
    HAL_GPIO_WritePin(CHARGE_EN_PORT, CHARGE_EN_PIN,
                      on ? CHARGE_ON : CHARGE_OFF);
    s_charge_on = on;
}

void GPIO_Loads_Init(void)
{
    set_ch1(0);
    set_ch2(0);
    set_charge(0);
}

void EnableAllLoads(void)
{
    set_charge(1);
    set_ch1(1);
    set_ch2(1);
}

void EnableCriticalLoads(void)
{
    set_charge(1);
    set_ch1(1);
    set_ch2(0);
}

void DisableNonEssential(void)
{
    set_charge(1);
    set_ch1(0);
    set_ch2(0);
}

void DisableAllLoads(void)
{
    set_ch1(0);
    set_ch2(0);
    set_charge(0);
}

void ChargeEnable(void) { set_charge(1); }
void ChargeDisable(void) { set_charge(0); }

uint8_t GPIO_GetLoadCh1State(void) { return s_ch1_on; }
uint8_t GPIO_GetLoadCh2State(void) { return s_ch2_on; }
uint8_t GPIO_GetChargeState(void) { return s_charge_on; }
