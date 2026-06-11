#include "main.h"
#include "eps_ina226.h"
#include <stdio.h>
#include <string.h>

I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;
UART_HandleTypeDef huart1;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_I2C2_Init(void);
void MX_I2C3_Init(void);
void MX_USART1_UART_Init(void);

static void UART_Print(const char *str)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)str,
                      strlen(str), 100);
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C2_Init();
    MX_I2C3_Init();
    MX_USART1_UART_Init();

    UART_Print("\r\n=== Step 1: INA226 voltage test ===\r\n");

    uint8_t u3_ok = INA226_Check(1);
    uint8_t u4_ok = INA226_Check(0);

    char buf[64];
    snprintf(buf, sizeof(buf),
             "U3 (VBAT): %s\r\nU4 (Solar): %s\r\n",
             u3_ok ? "OK" : "FAIL",
             u4_ok ? "OK" : "FAIL");
    UART_Print(buf);

    INA226_Init(1);
    INA226_Init(0);

    uint32_t counter = 0;
    while (1)
    {
        INA226_Data bat = INA226_Read(1);
        INA226_Data sol = INA226_Read(0);

        snprintf(buf, sizeof(buf),
                 "[%04lu] Vbat=%.3fV  Vsol=%.3fV\r\n",
                 counter++, bat.voltage_V, sol.voltage_V);
        UART_Print(buf);

        HAL_Delay(500);
    }
}
