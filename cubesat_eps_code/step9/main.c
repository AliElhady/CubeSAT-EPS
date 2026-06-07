#include "main.h"
#include "eps_soc.h"
#include "eps_fsm.h"
#include "eps_gpio.h"
#include "eps_flash.h"
#include "eps_telemetry.h"

ADC_HandleTypeDef hadc;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim6;

#define R_TOP 100000.0f
#define R_BOT 47000.0f
#define V_REF 3.3f
#define ADC_MAX 4095.0f
#define DIVIDER_FACTOR ((R_TOP + R_BOT) / R_BOT)
#define ADC_SAMPLES 16
#define ADC_CH_VBAT ADC_CHANNEL_0
#define ADC_CH_VSOL ADC_CHANNEL_1

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_ADC_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_DMA_Init(void);
void MX_TIM6_Init(void);

static float Read_Voltage(uint32_t channel)
{
    ADC_ChannelConfTypeDef cfg = {0};
    cfg.Channel = channel;
    cfg.Rank = ADC_REGULAR_RANK_1;
    cfg.SamplingTime = ADC_SAMPLETIME_39CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc, &cfg);

    uint32_t sum = 0;
    for (int i = 0; i < ADC_SAMPLES; i++)
    {
        HAL_ADC_Start(&hadc);
        HAL_ADC_PollForConversion(&hadc, 10);
        sum += HAL_ADC_GetValue(&hadc);
        HAL_ADC_Stop(&hadc);
        HAL_Delay(2);
    }
    float v_adc = ((float)sum / ADC_SAMPLES) * V_REF / ADC_MAX;
    return v_adc * DIVIDER_FACTOR;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim6)
        Telem_TickSecond();
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_DMA_Init();
    MX_GPIO_Init();
    MX_ADC_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_TIM6_Init();

    GPIO_Loads_Init();
    Telem_Init();

    HAL_TIM_Base_Start_IT(&htim6);

    while (1)
    {
INA226_Data bat = INA226_Read_OnBus(&hi2c2, INA226_ADDR_U3);
INA226_Data sol = INA226_Read_OnBus(&hi2c3, INA226_ADDR_U4);
float vbat = bat.voltage_V;
float vsol = sol.voltage_V;

        FSM_Update(vbat, vsol);

        Telem_Update();

        HAL_Delay(500);
    }
}
