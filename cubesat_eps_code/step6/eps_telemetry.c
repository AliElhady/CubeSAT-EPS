#include "eps_telemetry.h"
#include "eps_fsm.h"
#include "eps_gpio.h"
#include "eps_flash.h"
#include <stdio.h>
#include <string.h>
#define TELEM_MODE_ASCII
#define TELEM_INTERVAL_NORMAL_MS 5000U
#define TELEM_INTERVAL_LOWPOWER_MS 10000U
#define TELEM_INTERVAL_CRITICAL_MS 30000U
#define TELEM_ASCII_BUF_SIZE  128U

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

static uint8_t s_packet_id = 0;
static uint32_t s_uptime_s  = 0;
static uint32_t s_last_tick = 0;
static volatile uint8_t s_dma_busy = 0;
static uint8_t s_ascii_buf[TELEM_ASCII_BUF_SIZE];

#pragma pack(push, 1)
typedef struct
{
    uint8_t start[2];
    uint8_t packet_id;
    uint8_t state;
    uint8_t soc;
    uint8_t boot_reason;
    uint16_t vbat_raw;
    uint16_t vsol_raw;
    uint8_t flags;
    uint8_t loads;
    uint16_t uptime_s;
    uint8_t checksum;
    uint8_t stop;
} TelemetryPacket;
#pragma pack(pop)

_Static_assert(sizeof(TelemetryPacket) == 16,
               "TelemetryPacket must be exactly 16 bytes");

static TelemetryPacket s_bin_packet;
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1 || huart == &huart2)
        s_dma_busy = 0;
}
static uint16_t FloatToU16BE(float v)
{
    uint16_t raw = (uint16_t)(v * 100.0f);
    return (uint16_t)((raw >> 8) | (raw << 8));
}

static uint32_t GetInterval(void)
{
    switch (FSM_GetState())
    {
        case STATE_BOOT:
        case STATE_POWER_CHECK:
        case STATE_CHARGING:
        case STATE_NORMAL: return TELEM_INTERVAL_NORMAL_MS;
        case STATE_LOW_POWER: return TELEM_INTERVAL_LOWPOWER_MS;
        case STATE_CRITICAL:
        case STATE_ERROR: return TELEM_INTERVAL_CRITICAL_MS;
        case STATE_HIBERNATE: return 0;
        default: return TELEM_INTERVAL_NORMAL_MS;
    }
}

static void Send_ASCII(void)
{
    if (s_dma_busy) return;

    static const char *STATE_NAMES[] = {
        "BOOT", "PWR_CHK", "CHARGING", "NORMAL",
        "LOW_PWR", "CRITICAL", "HIBERNATE", "ERROR"
    };
    static const char *BOOT_NAMES[] = {
        "COLD", "HIBERNATE", "CORRUPT"
    };

    EPS_State  state = FSM_GetState();
    BootReason br = EPS_GetBootReason();

    int len = snprintf(
        (char *)s_ascii_buf,
        TELEM_ASCII_BUF_SIZE,
        "$EPS,%03u,%s,%d%%,%.2fV,%.2fV,%s,ch1=%d,ch2=%d,chg=%d,up=%lu\r\n",
        s_packet_id,
        (state < STATE_COUNT) ? STATE_NAMES[state] : "?",
        FSM_GetSOC(),
        FSM_GetBattVoltage(),
        FSM_GetSolarVoltage(),
        (br <= BOOT_REASON_CORRUPT_LOG) ? BOOT_NAMES[br] : "?",
        GPIO_GetLoadCh1State(),
        GPIO_GetLoadCh2State(),
        GPIO_GetChargeState(),
        s_uptime_s
    );

    if (len > 0 && len < (int)TELEM_ASCII_BUF_SIZE)
    {
        s_dma_busy = 1;
        HAL_UART_Transmit_DMA(&huart1, s_ascii_buf, (uint16_t)len);
    }
}

static void Send_Binary(void)
{
    if (s_dma_busy) return;

    TelemetryPacket *p = &s_bin_packet;

    p->start[0] = 0xAA;
    p->start[1] = 0x55;
    p->packet_id = s_packet_id;
    p->state = (uint8_t)FSM_GetState();
    p->soc = (uint8_t)FSM_GetSOC();
    p->boot_reason = (uint8_t)EPS_GetBootReason();
    p->vbat_raw = FloatToU16BE(FSM_GetBattVoltage());
    p->vsol_raw = FloatToU16BE(FSM_GetSolarVoltage());
    p->flags = (FSM_IsCharging() ? 0x01 : 0x00)
                   | (FSM_IsError() ? 0x02 : 0x00)
                   | (FSM_IsFullCharge() ? 0x04 : 0x00);
    p->loads = (GPIO_GetLoadCh1State() ? 0x01 : 0x00)
                   | (GPIO_GetLoadCh2State() ? 0x02 : 0x00)
                   | (GPIO_GetChargeState() ? 0x04 : 0x00);
    p->uptime_s = (uint16_t)(s_uptime_s & 0xFFFF);
    p->stop = 0xFF;
    uint8_t xor_val = 0;
    uint8_t *bytes = (uint8_t *)p;
    for (int i = 0; i < (int)(sizeof(TelemetryPacket) - 2); i++)
        xor_val ^= bytes[i];
    p->checksum = xor_val;

    s_dma_busy = 1;
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)p, sizeof(TelemetryPacket));
}

void Telem_Init(void)
{
    s_packet_id = 0;
    s_uptime_s = 0;
    s_last_tick = HAL_GetTick();
    s_dma_busy = 0;
}

void Telem_TickSecond(void)
{
    s_uptime_s++;
}

void Telem_Update(void)
{
    uint32_t interval = GetInterval();
    if (interval == 0) return;

    uint32_t now = HAL_GetTick();
    if ((now - s_last_tick) < interval) return;

    s_last_tick = now;

#ifdef TELEM_MODE_ASCII
    Send_ASCII();
#else
    Send_Binary();
#endif

    s_packet_id++;
}

void Telem_SendNow(void)
{
    if (s_dma_busy) return;

#ifdef TELEM_MODE_ASCII
    Send_ASCII();
#else
    Send_Binary();
#endif

    s_packet_id++;
    s_last_tick = HAL_GetTick();
}
