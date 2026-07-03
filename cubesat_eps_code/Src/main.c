/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "eps_ina226.h"
#include "eps_soc.h"
#include "eps_fsm.h"
#include "eps_gpio.h"
#include "eps_flash.h"
#include "eps_watchdog.h"
#include "eps_imu.h"
#include "LoRa.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */
float g_vbat = 0.0f;
float g_vsol = 0.0f;
float g_ibat_mA = 0.0f;
uint8_t g_u3_ok = 0;
uint8_t g_u4_ok = 0;
uint32_t g_loop_count = 0;
int g_soc = 0;
uint8_t g_fsm_state = 0;
uint8_t g_charge_en = 0;
uint8_t g_boot_reason = 0;
uint8_t g_flash_save_result = 0xFF;
uint32_t g_flash_magic = 0;
uint8_t g_flash_state = 0xFF;
uint8_t g_flash_soc = 0xFF;
float g_flash_vbat = 0.0f;
uint32_t g_flash_crc_stored = 0;
uint32_t g_flash_crc_calc = 0;
uint8_t g_crc_ok = 0;
uint8_t g_wdg_reason = 0;
uint32_t g_loop_time_ms = 0;
uint32_t g_ina_time_ms  = 0;
char g_telem_buf[128];
uint32_t g_telem_packet_id = 0;
uint32_t g_telem_uptime = 0;
const char *g_telem_str = g_telem_buf;
float g_accel_x = 0.0f;
float g_accel_y = 0.0f;
float g_accel_z = 0.0f;
float g_gyro_x = 0.0f;
float g_gyro_y = 0.0f;
float g_gyro_z = 0.0f;
uint8_t g_imu1_ok   = 0;
uint8_t g_imu2_ok   = 0;
uint8_t g_imu_fault = 0;
uint8_t g_imu1_whoami = 0;
uint8_t g_imu2_whoami = 0;
uint8_t g_imu_at_68 = 0;
uint8_t g_imu_at_69 = 0;
uint8_t g_scan_found = 0;
uint8_t g_scan_addr = 0;
LoRa myLoRa;
uint8_t g_lora_ok = 0;
uint32_t g_lora_sent = 0;
uint16_t g_lora_init_result = 0;
uint8_t g_lora_version = 0;
uint8_t g_lora_tx_result = 0;
uint8_t g_lora_frmsb = 0;
uint8_t g_lora_frmid = 0;
uint8_t g_lora_frlsb = 0;
float g_pbat_mW = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C3_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C3_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,  GPIO_PIN_SET);  /* LORA_CS  */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);  /* IMU1_CS  */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);  /* IMU2_CS  */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);  /* CHARGE_EN */

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);   /* TXEN HIGH */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
  HAL_Delay(10);

  GPIO_Loads_Init();
  g_u3_ok = INA226_Check(1);
  if (g_u3_ok) INA226_Init(1);

  myLoRa = newLoRa();
  myLoRa.hSPIx = &hspi1;
  myLoRa.CS_port = GPIOA;
  myLoRa.CS_pin = GPIO_PIN_4;
  myLoRa.reset_port = GPIOB;
  myLoRa.reset_pin = GPIO_PIN_0;
  myLoRa.DIO0_port = GPIOB;
  myLoRa.DIO0_pin = GPIO_PIN_2;
  myLoRa.frequency = 433;
  myLoRa.spredingFactor = SF_7;
  myLoRa.bandWidth = BW_125KHz;
  myLoRa.crcRate = CR_4_5;
  myLoRa.power = POWER_20db;
  myLoRa.overCurrentProtection = 100;
  myLoRa.preamble = 8;
  //g_lora_ok = (LoRa_init(&myLoRa) == LORA_OK);
//  __HAL_RCC_SPI2_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);  /* RST LOW */
  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);    /* RST HIGH */
  HAL_Delay(100);

  g_lora_version = LoRa_read(&myLoRa, 0x42);
//  uint8_t spi1_tx = 0x42;
//  uint8_t spi1_rx = 0x00;
//  HAL_StatusTypeDef spi1_status;

//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
//  spi1_status = HAL_SPI_TransmitReceive(&hspi1, &spi1_tx, &spi1_rx, 1, 100);
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

 // g_lora_version = spi1_rx;
//  g_imu1_whoami = (uint8_t)spi1_status;
  g_lora_init_result = LoRa_init(&myLoRa);
  g_lora_ok = (g_lora_init_result == LORA_OK);

  g_lora_frmsb = LoRa_read(&myLoRa, 0x06);
  g_lora_frmid = LoRa_read(&myLoRa, 0x07);
  g_lora_frlsb = LoRa_read(&myLoRa, 0x08);

  WDG_Init();
  g_wdg_reason = (uint8_t)WDG_GetLastResetReason();

//  g_imu1_ok = IMU_Check(1);
//  g_imu2_ok = IMU_Check(2);
//  if (g_imu1_ok) IMU_Init(1);
//  if (g_imu2_ok) IMU_Init(2);
 // g_imu1_whoami = IMU_ReadReg_Debug(1);
//  g_imu2_whoami = IMU_ReadReg_Debug(2);

//  HAL_StatusTypeDef spi_status = HAL_SPI_GetState(&hspi2);
//  g_imu1_whoami = IMU_ReadWhoAmI(1);
//  g_imu2_whoami = IMU_ReadWhoAmI(2);

//  g_imu1_ok = (g_imu1_whoami == 0x47) ? 1 : 0;
//  g_imu2_ok = (g_imu2_whoami == 0x47) ? 1 : 0;
//  if (g_imu1_ok) IMU_Init(1);
//  if (g_imu2_ok) IMU_Init(2);
//  g_i2c_scan[128] = {0};
//  for (uint8_t addr = 1; addr < 127; addr++)
//  {
//      if (HAL_I2C_IsDeviceReady(&hi2c2, addr << 1, 1, 10) == HAL_OK)
//          g_i2c_scan[addr] = 1;
//  }

//  g_scan_found = 0;
//  g_scan_addr = 0;
//  for (uint8_t addr = 1; addr < 127; addr++)
//  {
//      if (HAL_I2C_IsDeviceReady(&hi2c2, addr << 1, 1, 10) == HAL_OK)
//      {
//          g_scan_found++;
//          g_scan_addr = addr;  /* запомним последний найденный */
//      }
//  }

//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);  /* CS LOW — IMU1 */
//  HAL_SPI_TransmitReceive(&hspi2, &spi_tx, &spi_rx, 1, 100);
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);    /* CS HIGH */

//  g_imu1_whoami = spi_rx;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  WDG_Refresh();
	  uint32_t t_start = HAL_GetTick();
	    INA226_Data bat = INA226_Read(1);
//	    INA226_Data sol = INA226_Read(0);
//	    if (g_u4_ok) sol = INA226_Read(0);
	    uint32_t t_after_ina = HAL_GetTick();
	    g_vbat = bat.voltage_V;
//	    g_vsol = sol.voltage_V;
	    g_ibat_mA = bat.current_mA;
	    g_pbat_mW = g_vbat * g_ibat_mA;

	    FSM_Update(g_vbat, g_vsol);

	    snprintf(g_telem_buf, sizeof(g_telem_buf),
	        "$EPS,%03lu,%d%%,%.2fV,st=%d,chg=%d,wdg=%d",
			g_telem_packet_id,
	        g_soc,
	        g_vbat,
	        g_fsm_state,
	        g_charge_en,
	        g_wdg_reason);
	    g_telem_packet_id++;
	    g_telem_uptime = HAL_GetTick() / 1000;

	    IMU_System imu = IMU_Update();
	    g_accel_x = imu.imu1.accel_x;
	    g_accel_y = imu.imu1.accel_y;
	    g_accel_z = imu.imu1.accel_z;
	    g_gyro_x = imu.imu1.gyro_x;
	    g_gyro_y = imu.imu1.gyro_y;
	    g_gyro_z = imu.imu1.gyro_z;
	    g_imu_fault = imu.fault;

	    g_soc = FSM_GetSOC();
	    g_fsm_state = (uint8_t)FSM_GetState();
	    g_charge_en = GPIO_GetChargeState();
	    g_boot_reason = (uint8_t)EPS_GetBootReason();
//	    g_crc_ok = EPS_Flash_DiagCRC();
	    g_loop_count++;
	    WDG_Refresh();
	    HAL_Delay(500);
	    static uint32_t last_lora = 0;
	    if (HAL_GetTick() - last_lora > 5000)
	    {
	        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	        char buf[64];
	        snprintf(buf, sizeof(buf),
	                 "$EPS,%03lu,%d%%,%d.%02dV,%dmW,st=%s",
	                 g_lora_sent, g_soc,
	                 (int)g_vbat,
	                 (int)(g_vbat * 100) % 100,
	                 (int)g_pbat_mW,
	                 FSM_GetStateName());
	        g_lora_tx_result = LoRa_transmit(&myLoRa, (uint8_t*)buf, strlen(buf), 100);
	        g_lora_sent++;
	        last_lora = HAL_GetTick();
	    }
	    g_loop_time_ms = HAL_GetTick() - t_start;
	    g_ina_time_ms  = t_after_ina - t_start;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00503D58;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00503D58;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_12|GPIO_PIN_5
                          |GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB2 PB12 PB5
                           PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_12|GPIO_PIN_5
                          |GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
