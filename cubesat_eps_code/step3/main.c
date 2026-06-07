/* ============================================================
 *  CubeSat EPS — Step 3: ADC + SOC + FSM
 *
 *  What has changed since Step 2:
 *    + eps_fsm.h has been included
 *    + FSM_Update(vbat, vsol) is called in the main loop
 *    + The name of the current state has been added to the UART
 *    + When the state changes — a separate line in the terminal
 *
 *  Readiness criteria:
 *    Change Vbat via the power supply — we see state changes in the UART
 *    according to the diagram. Check all transitions from the table below.
 * ============================================================ */

#include "main.h"
#include "eps_soc.h"
#include "eps_fsm.h"
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
    float v_adc = adc_avg * V_REF / ADC_MAX;
    return v_adc * DIVIDER_FACTOR;
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

    UART_Print("\r\n=== CubeSat EPS — Step 3: ADC + SOC + FSM ===\r\n");
    UART_Print("Vbat[V]  SOC[%]  State\r\n");
    UART_Print("-----------------------------\r\n");

    char buf[80];
    uint32_t counter = 0;
    EPS_State prev_state = STATE_COUNT;

    while (1)
    {
INA226_Data bat = INA226_Read_OnBus(&hi2c2, INA226_ADDR_U3);
INA226_Data sol = INA226_Read_OnBus(&hi2c3, INA226_ADDR_U4);
float vbat = bat.voltage_V;
float vsol = sol.voltage_V;

        FSM_Update(vbat, vsol);

        EPS_State state = FSM_GetState();
        int soc = FSM_GetSOC();
        const char *name = FSM_GetStateName();

        if (state != prev_state)
        {
            snprintf(buf, sizeof(buf),
                     "\r\n>>> TRANSITION: %s\r\n\r\n",
                     name);
            UART_Print(buf);
            prev_state = state;
        }

        snprintf(buf, sizeof(buf),
                 "[%04lu] %.3fV  %3d%%  %s\r\n",
                 counter++, vbat, soc, name);
        UART_Print(buf);

        HAL_Delay(500);
    }
}
