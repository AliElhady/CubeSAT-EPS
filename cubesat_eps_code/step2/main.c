/* ============================================================
 *  CubeSat EPS — Step 2: ADC + SOC
 *
 *  What has changed since Step 1:
 *    + eps_soc.h has been included
 *    + VoltageToSOC(vbat) called after reading the ADC
 *    + SOC added to the output string
 *
 *  Readiness criteria:
 *    when Vbat ≈ 7.85V, the terminal displays SOC ≈ 80%
 *    when Vbat ≈ 6.60V, the terminal displays SOC ≈ 20%
 * ============================================================ */

#include "main.h"
#include "eps_soc.h"
#include <stdio.h>
#include <string.h>

ADC_HandleTypeDef  hadc;
UART_HandleTypeDef huart1;

#define R_TOP 100000.0f
#define R_BOT 47000.0f
#define V_REF 3.3f
#define ADC_MAX 4095.0f
#define DIVIDER_FACTOR ((R_TOP + R_BOT) / R_BOT)
#define ADC_SAMPLES 16

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_ADC_Init(void);
void MX_USART1_UART_Init(void);

static float Read_Voltage(uint32_t adc_channel)
{
    ADC_ChannelConfTypeDef config = {0};
    config.Channel = adc_channel;
    config.Rank = ADC_REGULAR_RANK_1;
    config.SamplingTime = ADC_SAMPLETIME_39CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc, &config);

    uint32_t sum = 0;
    for (int i = 0; i < ADC_SAMPLES; i++)
    {
        HAL_ADC_Start(&hadc);
        HAL_ADC_PollForConversion(&hadc, 10);
        sum += HAL_ADC_GetValue(&hadc);
        HAL_ADC_Stop(&hadc);
        HAL_Delay(2);
    }

    float adc_avg = (float)sum / (float)ADC_SAMPLES;
    float voltage_adc = adc_avg * V_REF / ADC_MAX;
    return voltage_adc * DIVIDER_FACTOR;
}

static void UART_Print(const char *str)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)str,
                      (uint16_t)strlen(str), 100);
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_ADC_Init();
    MX_USART1_UART_Init();

    UART_Print("\r\n=== CubeSat EPS — Step 2: ADC + SOC ===\r\n");

    char buf[64];
    uint32_t counter = 0;

    while (1)
    {
        INA226_Data bat = INA226_Read_OnBus(&hi2c2, INA226_ADDR_U3);
INA226_Data sol = INA226_Read_OnBus(&hi2c3, INA226_ADDR_U4);
float vbat = bat.voltage_V;
float vsol = sol.voltage_V;

        /* ← New: Calculating SOC from battery voltage */
        int soc = VoltageToSOC(vbat);

        snprintf(buf, sizeof(buf),
                 "[%04lu] Vbat=%.3fV  SOC=%3d%%  Vsol=%.3fV\r\n",
                 counter++, vbat, soc, vsol);
        UART_Print(buf);

        HAL_Delay(500);
    }
}
