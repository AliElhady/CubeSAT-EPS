/* ============================================================
 *  CubeSat EPS — Step 4: State Machine + GPIO Control
 *
 *  File: eps_fsm.c
 *
 *  Enter_X() — now calls the actual GPIOs via eps_gpio.h
 *  Run_X()   — transition logic based on the state diagram (unchanged)
 *
 *  Dependencies:
 *    eps_soc.h  — VoltageToSOC
 *    eps_gpio.h — load control and CHARGE_EN
 *
 *  Readiness criterion:
 *    loads switch correctly upon transition to each state
 *    — verify with a multimeter at the test points
 * ============================================================ */

#include "eps_fsm.h"
#include "eps_soc.h"
#include "eps_gpio.h"
#include "eps_flash.h"
#include "eps_telemetry.h"

#define VBAT_FULL       8.20f
#define VBAT_NORMAL     7.30f
#define VBAT_LOWPOWER   6.90f
#define VBAT_HIBERNATE  6.20f
#define VBAT_DEAD       6.00f

static EPS_State s_current = STATE_BOOT;
static EPS_State s_prev    = STATE_BOOT;

static float s_vbat = 0.0f;
static float s_vsol = 0.0f;
static int s_soc = 0;
static uint8_t s_charging = 0;
static uint8_t s_full_charge = 0;
static uint8_t s_error = 0;
static uint8_t s_reset = 0;

static const char *STATE_NAMES[] = {
    "BOOT",
    "PWR_CHECK",
    "CHARGING",
    "NORMAL",
    "LOW_POWER",
    "CRITICAL",
    "HIBERNATE",
    "ERROR"
};

static void Enter_Boot(void)
{
    RestoreStateOnStartup();
}

static void Enter_PowerCheck(void)
{
}

static void Enter_Charging(void)
{
    EnableCriticalLoads();
    Telem_SendNow();
}

static void Enter_Normal(void)
{
    /* Battery > 50% — turn everything on                          */
    EnableAllLoads();
    Telem_SendNow();
}

static void Enter_LowPower(void)
{
    /* Battery 20–50% — turn off non-critical loads (channel 2) */
    EnableCriticalLoads();
    Telem_SendNow();
}

static void Enter_Critical(void)
{
    /* Battery < 20% — everything is turned off, only charging is active */
    DisableNonEssential();
    Telem_SendNow();
}

static void Enter_Hibernate(void)
{
    DisableAllLoads();

    EPS_Flash_Save((uint8_t)STATE_HIBERNATE,
                   (uint8_t)s_soc,
                   s_vbat,
                   s_vsol);

    HAL_SuspendTick();
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

    HAL_ResumeTick();
    SystemClock_Config();
}

static void Enter_Error(void)
{
    DisableAllLoads();
    Telem_SendNow();
}

static EPS_State Run_Boot(void)
{
    return STATE_POWER_CHECK;
}

static EPS_State Run_PowerCheck(void)
{
    if (s_error) return STATE_ERROR;
    if (s_vbat <= VBAT_DEAD) return STATE_HIBERNATE;
    if (s_charging && !s_full_charge) return STATE_CHARGING;
    if (s_vbat >= VBAT_NORMAL) return STATE_NORMAL;
    if (s_vbat >= VBAT_LOWPOWER) return STATE_LOW_POWER;
    return STATE_CRITICAL;
}

static EPS_State Run_Charging(void)
{
    if (s_error) return STATE_ERROR;
    if (s_full_charge) return STATE_POWER_CHECK;
    return STATE_CHARGING;
}

static EPS_State Run_Normal(void)
{
    if (s_error) return STATE_ERROR;
    if (s_vbat < VBAT_NORMAL) return STATE_LOW_POWER;
    return STATE_NORMAL;
}

static EPS_State Run_LowPower(void)
{
    if (s_error) return STATE_ERROR;
    if (s_vbat >= VBAT_NORMAL) return STATE_NORMAL;
    if (s_charging) return STATE_POWER_CHECK;
    if (s_vbat < VBAT_LOWPOWER) return STATE_CRITICAL;
    return STATE_LOW_POWER;
}

static EPS_State Run_Critical(void)
{
    if (s_error) return STATE_ERROR;
    if (s_vbat <= VBAT_HIBERNATE) return STATE_HIBERNATE;
    if (s_vbat >= VBAT_LOWPOWER) return STATE_POWER_CHECK;
    return STATE_CRITICAL;
}

static EPS_State Run_Hibernate(void)
{
    if (s_vbat > VBAT_DEAD) return STATE_BOOT;
    return STATE_HIBERNATE;
}

static EPS_State Run_Error(void)
{
    s_error = 0;
    if (s_reset) { s_reset = 0; return STATE_BOOT; }
    return STATE_ERROR;
}

void FSM_Update(float vbat, float vsol)
{
    s_vbat = vbat;
    s_vsol = vsol;
    s_soc = VoltageToSOC(vbat);
    s_charging = (vsol > vbat + 0.3f) ? 1 : 0;
    s_full_charge = (vbat >= VBAT_FULL) ? 1 : 0;

    EPS_State next = s_current;
    switch (s_current)
    {
        case STATE_BOOT: next = Run_Boot();
        break;
        case STATE_POWER_CHECK: next = Run_PowerCheck();
        break;
        case STATE_CHARGING: next = Run_Charging();
        break;
        case STATE_NORMAL: next = Run_Normal();
        break;
        case STATE_LOW_POWER: next = Run_LowPower();
        break;
        case STATE_CRITICAL: next = Run_Critical();
        break;
        case STATE_HIBERNATE: next = Run_Hibernate();
        break;
        case STATE_ERROR: next = Run_Error();
        break;
        default: next = STATE_ERROR;
        break;
    }

    if (next != s_current)
    {
        s_prev = s_current;
        s_current = next;
        switch (s_current)
        {
            case STATE_BOOT: Enter_Boot();
            break;
            case STATE_POWER_CHECK: Enter_PowerCheck();
            break;
            case STATE_CHARGING: Enter_Charging();
            break;
            case STATE_NORMAL: Enter_Normal();
            break;
            case STATE_LOW_POWER: Enter_LowPower();
            break;
            case STATE_CRITICAL: Enter_Critical();
            break;
            case STATE_HIBERNATE: Enter_Hibernate();
            break;
            case STATE_ERROR: Enter_Error();
            break;
            default: break;
        }
    }
}

EPS_State FSM_GetState(void) { return s_current; }
int FSM_GetSOC(void) { return s_soc; }
float FSM_GetBattVoltage(void) { return s_vbat; }
float FSM_GetSolarVoltage(void) { return s_vsol; }
uint8_t FSM_IsCharging(void) { return s_charging; }
uint8_t FSM_IsFullCharge(void) { return s_full_charge; }
uint8_t FSM_IsError(void) { return s_error; }

const char *FSM_GetStateName(void)
{
    if (s_current < STATE_COUNT)
        return STATE_NAMES[s_current];
    return "UNKNOWN";
}

void FSM_SetError(void) { s_error = 1; }
void FSM_SetReset(void) { s_reset = 1; }
