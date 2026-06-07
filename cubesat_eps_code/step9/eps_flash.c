#include "main.h"
#include "eps_flash.h"
#include "eps_fsm.h"
#include <string.h>
#define FLASH_BASE_ADDR 0x08000000UL
#define FLASH_PAGE_SIZE 2048U
#define FLASH_BANK FLASH_BANK_2
#define FLASH_PAGE_IN_BANK 127U
#define FLASH_PAGE_GLOBAL (128U + FLASH_PAGE_IN_BANK)
#define FLASH_LOG_ADDRESS (FLASH_BASE_ADDR + FLASH_PAGE_GLOBAL * FLASH_PAGE_SIZE)
#define LOG_MAGIC 0xEA51C0DEUL
typedef struct
{
    uint32_t magic;
    uint8_t state;
    uint8_t soc;
    uint8_t reserved[2];
    float batteryVoltage;
    float solarVoltage;
    uint32_t crc;
} SystemLog;

_Static_assert(sizeof(SystemLog) % 8 == 0,
               "SystemLog size must be a multiple of 8 bytes");

static uint32_t CRC32_Compute(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    for (uint32_t i = 0; i < len; i++)
    {
        crc ^= (uint32_t)data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc >> 1) ^ (0xEDB88320UL & -(crc & 1));
    }
    return crc ^ 0xFFFFFFFFUL;
}

static uint32_t Log_CRC(const SystemLog *log)
{
    return CRC32_Compute((const uint8_t *)log,
                         offsetof(SystemLog, crc));
}

static HAL_StatusTypeDef Flash_ErasePage(void)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t pageError = 0;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK;
    erase.Page = FLASH_PAGE_IN_BANK;
    erase.NbPages = 1;

    return (HAL_FLASHEx_Erase(&erase, &pageError) == HAL_OK &&
            pageError == 0xFFFFFFFFU)
           ? HAL_OK : HAL_ERROR;
}

static HAL_StatusTypeDef Flash_WriteLog(const SystemLog *log)
{
    const uint8_t *raw = (const uint8_t *)log;
    uint32_t size = sizeof(SystemLog);

    for (uint32_t i = 0; i < size; i += 8)
    {
        uint64_t chunk = 0xFFFFFFFFFFFFFFFFULL;
        uint32_t bytes = (size - i >= 8) ? 8 : (size - i);
        memcpy(&chunk, raw + i, bytes);

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                              FLASH_LOG_ADDRESS + i,
                              chunk) != HAL_OK)
            return HAL_ERROR;
    }
    return HAL_OK;
}
HAL_StatusTypeDef EPS_Flash_Save(uint8_t state,
                                 uint8_t soc,
                                 float   batteryVoltage,
                                 float   solarVoltage)
{
    SystemLog log;
    memset(&log, 0xFF, sizeof(log));

    log.magic = LOG_MAGIC;
    log.state = state;
    log.soc = soc;
    log.batteryVoltage = batteryVoltage;
    log.solarVoltage = solarVoltage;
    log.crc = Log_CRC(&log);

    HAL_FLASH_Unlock();

    HAL_StatusTypeDef status = Flash_ErasePage();
    if (status == HAL_OK)
        status = Flash_WriteLog(&log);

    HAL_FLASH_Lock();
    return status;
}
HAL_StatusTypeDef EPS_Flash_Load(SystemLog *out_log)
{
    SystemLog log;
    memcpy(&log, (const void *)FLASH_LOG_ADDRESS, sizeof(log));
    if (log.magic != LOG_MAGIC)
        return HAL_ERROR;
    if (log.crc != Log_CRC(&log))
        return HAL_ERROR;
    if (log.batteryVoltage < 5.0f || log.batteryVoltage > 9.0f)
        return HAL_ERROR;
    if (log.soc > 100)
        return HAL_ERROR;
    *out_log = log;
    return HAL_OK;
}
HAL_StatusTypeDef EPS_Flash_Clear(void)
{
    HAL_FLASH_Unlock();
    HAL_StatusTypeDef status = Flash_ErasePage();
    HAL_FLASH_Lock();
    return status;
}
static BootReason s_bootReason = BOOT_REASON_COLD;

void RestoreStateOnStartup(void)
{
    SystemLog log;
    HAL_StatusTypeDef result = EPS_Flash_Load(&log);

    if (result != HAL_OK)
    {
        SystemLog raw;
        memcpy(&raw, (const void *)FLASH_LOG_ADDRESS, sizeof(raw));

        s_bootReason = (raw.magic == LOG_MAGIC)
                       ? BOOT_REASON_CORRUPT_LOG
                       : BOOT_REASON_COLD;
        return;
    }
    if (log.state == STATE_HIBERNATE)
    {
        s_bootReason = BOOT_REASON_HIBERNATE;
        FSM_Update(log.batteryVoltage, log.solarVoltage);

        EPS_Flash_Clear();
    }
    else
    {
        s_bootReason = BOOT_REASON_COLD;
        EPS_Flash_Clear();
    }
}
BootReason EPS_GetBootReason(void) { return s_bootReason; }
