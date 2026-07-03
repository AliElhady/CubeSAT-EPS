/* ============================================================
 *  CubeSat EPS — Step 7: Independent Watchdog
 *
 *  Timeout: (2999 + 1) × 64 / 32000 = 6.0 sec
 *  LSI = ~32 kHz (±10%), actual range: 5.4 – 6.6 sec
 *  The main loop runs in ~500 ms — 10× margin
 *
 *  Call order in main():
 *    WDG_Init()— before while(1), immediately after HAL_Init()
 *    WDG_Refresh() — at the end of each while(1) iteration
 *    WDG_GetLastResetReason() — for telemetry
 * ============================================================ */

#include "eps_watchdog.h"

/* ----------------------------------------------------------
 *  Static variables are only visible within this file
 * ---------------------------------------------------------- */
static IWDG_HandleTypeDef  hiwdg;
static WatchdogResetReason s_reset_reason = WDG_REASON_COLD;

/* ----------------------------------------------------------
 *  WDG_GetResetReason
 *
 *  Reads the RCC->CSR register and returns the cause of the last
 *  reset. Must be called BEFORE __HAL_RCC_CLEAR_RESET_FLAGS() —
 *  otherwise the flags will already have been cleared and the function will always return COLD.
 * ---------------------------------------------------------- */
WatchdogResetReason WDG_GetResetReason(void)
{
    uint32_t csr = RCC->CSR;   /* Read the register once */

    if (csr & RCC_CSR_IWDGRSTF) { return WDG_REASON_IWDG;      }
    if (csr & RCC_CSR_PINRSTF) { return WDG_REASON_PIN;       }
    if (csr & RCC_CSR_BORRSTF) { return WDG_REASON_BROWNOUT;  }
    if (csr & RCC_CSR_SFTRSTF) { return WDG_REASON_SOFTWARE;  }

    return WDG_REASON_COLD;    /* No flags matched */
}

/* ----------------------------------------------------------
 *  WDG_Init
 *
 *  1. Save the reason for the reset BEFORE clearing the flags
 *  2. Clear the flags
 *  3. Configure and start the IWDG
 * ---------------------------------------------------------- */
void WDG_Init(void)
{
    s_reset_reason = WDG_GetResetReason();
    __HAL_RCC_CLEAR_RESET_FLAGS();

    hiwdg.Instance = IWDG;
//    hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
//     hiwdg.Init.Reload = 2999;   /* timeout ~6 sec */
    hiwdg.Init.Prescaler = IWDG_PRESCALER_256;  /* ← максимальный */
    hiwdg.Init.Reload    = 4095;                /* ← максимальный */
   hiwdg.Init.Window    = 4095;  

    HAL_IWDG_Init(&hiwdg);
}
/* ----------------------------------------------------------
 *  WDG_Refresh
 *
 *  “I'm alive” signal — resets the IWDG counter.
 *  Call this at the end of each iteration of the main loop.
 *  If not called within 6 seconds, the MCU will reboot.
 * ---------------------------------------------------------- */
void WDG_Refresh(void)
{
    HAL_IWDG_Refresh(&hiwdg);
}
/* ----------------------------------------------------------
 *  WDG_GetLastResetReason
 *
 *  Telemetry getter — returns the reason for the last
 *  reset, which was saved during initialization.
 * ---------------------------------------------------------- */
WatchdogResetReason WDG_GetLastResetReason(void)
{
    return s_reset_reason;
}
