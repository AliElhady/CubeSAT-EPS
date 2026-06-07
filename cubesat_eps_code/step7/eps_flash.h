#ifndef EPS_FLASH_H
#define EPS_FLASH_H
#include "main.h"
typedef enum
{
    BOOT_REASON_COLD = 0,
    BOOT_REASON_HIBERNATE,
    BOOT_REASON_CORRUPT_LOG,
} BootReason;

HAL_StatusTypeDef EPS_Flash_Save(uint8_t state,
                                 uint8_t soc,
                                 float   batteryVoltage,
                                 float   solarVoltage);

HAL_StatusTypeDef EPS_Flash_Clear(void);
void RestoreStateOnStartup(void);
BootReason EPS_GetBootReason(void);

#endif
