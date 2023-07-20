/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "ble_status.h"
#include "SPBTLE_RF.h"
#include "bluenrg_gatt_server.h"
#include "sensor_service.h"
#include "bluenrg_utils.h"
#include "bluenrg_l2cap_aci.h"
#include <mxconstants.h>
#include <ahrs.h>
#include <flight_control.h>
#include <pid_control.h>
#include <remote_control.h>
#include <sensor_management.h>

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
volatile uint32_t HCI_ProcessEvent=0;

ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart1;

/* BLE module */
extern int is_Ble_Connected;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM9_Init(void);
static void MX_USART1_UART_Init(void);
static void Error_Handler(void);


/* USER CODE BEGIN 0 */
/* BLE */
extern uint8_t set_Connectable;
uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
uint32_t ConnectionBleStatus=0;
uint8_t BufferToWrite[256];
int32_t BytesToWrite;
/* 1ms System timer */
uint8_t sys_Prd_Clk = FALSE;
/* USER CODE END 0 */

 int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */

  /* Initialize TIM2 for External Remocon RF receiver PWM Input*/
  HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_4);

  /* Initialize TIM4 for Motors PWM Output*/
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);

  /* Initialize General purpose TIM9 50Hz*/
  HAL_TIM_Base_Start_IT(&htim9);

  /* BLE communication */
  PRINTF("BLE communication initialization...\n\n");
  BlueNRG_Init();
  Init_BlueNRG_Custom_Services();

  /*Motor init*/
  set_All_Motor_Stop();

  /*Senors init*/
  all_Sensor_Init();

  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    if(HCI_ProcessEvent)
    {
          HCI_ProcessEvent=0;
          HCI_Process();
    }

    if(set_Connectable)
    {
          /* Now update the BLE advertize data and make the Board connectable */
          setConnectable();
          set_Connectable = FALSE;
    }

    //1.8ms oscillator
    if(sys_Prd_Clk == TRUE)
    {
    	/*BLE update sensors data*/
		AccGyroMag_Update(&accelBleSentValue_st, &gyroBleSentValue_st, &magBleSentValue_st);
		/*BLE update battery and barometer data*/
		SendBattEnvData(baroBleSentValue_st.PRESSURE, baroBleSentValue_st.TEMP, vBat_ADC_Value);
    }
    /*sensor get value handler*/
    sensorBleUpdateValue(sys_Prd_Clk);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;

  HAL_RCC_OscConfig(&RCC_OscInitStruct);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);


}

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION12b;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc1);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

/* SPI2 init function */
void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */
	__SPI2_CLK_ENABLE();
	__GPIOB_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_1LINE;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */
//  SPI_1LINE_TX(&hspi2);
  __HAL_SPI_ENABLE(&hspi2);
  /* USER CODE END SPI2_Init 2 */

}

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 20;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 32767;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim2);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

  HAL_TIM_IC_Init(&htim2);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1);

  HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2);

  HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3);

  HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4);

}

/* TIM4 init function */
void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
#ifdef DC_MOTOR
  htim4.Init.Prescaler = 84;                                    /* DC motor configuration - Freq 494Hz*/
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1999;
#endif/*DC_MOTOR*/
#ifdef BRSHLESS_MOTOR
  htim4.Init.Prescaler = 100;                                    /* ESC motor configuration - Freq 400Hz*/
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2075;
#endif/*BRSHLESS_MOTOR*/

  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim4);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim4);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2);

  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3);

  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4);

}

/* TIM9 init function */
void MX_TIM9_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 51;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 1999;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim9);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim9, &sMasterConfig) != HAL_OK)
  {
	  Error_Handler();
  }
}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through
        * the Code Generation settings)
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB2 PB10 PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC13*/
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED3_Port, LED3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED2_Port, LED2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LIS2MDL_CS_Port, LIS2MDL_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LPS22H_CS_Port, LPS22H_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LSM6DS33_CS_Port, LSM6DS33_CS_Pin, GPIO_PIN_SET);

}

/* USER CODE BEGIN 4 */
/*
 *  Handle Timer9 interrupt @ 800Hz
 *  Set the event flag and increase time index
 */
void SPI_Read(SPI_HandleTypeDef* xSpiHandle, uint8_t *val)
{
  /* In master RX mode the clock is automaticaly generated on the SPI enable.
  So to guarantee the clock generation for only one data, the clock must be
  disabled after the first bit and before the latest bit */
  /* Interrupts should be disabled during this operation */

  __disable_irq();
  //GPIOA->BSRR = (uint32_t)GPIO_PIN_8 << 16U;
  __HAL_SPI_ENABLE(xSpiHandle);

  	__asm("dsb\n");__asm("dsb\n");
    __asm("dsb\n");__asm("dsb\n");
    __asm("dsb\n");__asm("dsb\n");
    __asm("dsb\n");__asm("dsb\n");
    __asm("dsb\n");__asm("dsb\n");
    __asm("dsb\n");__asm("dsb\n");
    __asm("dsb\n");__asm("dsb\n");
    __asm("dsb\n");__asm("dsb\n");
    __asm("dsb\n");__asm("dsb\n");
    __asm("dsb\n");__asm("dsb\n");
    __asm("dsb\n");__asm("dsb\n");
    __asm("dsb\n");__asm("dsb\n");
    __asm("dsb\n");__asm("dsb\n");
    __asm("dsb\n");__asm("dsb\n");
    __asm("dsb\n");__asm("dsb\n");
    __asm("dsb\n");__asm("dsb\n");
    __asm("dsb\n");__asm("dsb\n");
    __asm("dsb\n");__asm("dsb\n");
    __asm("dsb\n");__asm("dsb\n");
    __asm("dsb\n");__asm("dsb\n");
    __asm("dsb\n");__asm("dsb\n");
    __asm("dsb\n");__asm("dsb\n");
    __asm("dsb\n");__asm("dsb\n");
    __asm("dsb\n");__asm("dsb\n");
    __asm("dsb\n");__asm("dsb\n");
    __asm("dsb\n");__asm("dsb\n");
    __asm("dsb\n");__asm("dsb\n");
    __asm("dsb\n");__asm("dsb\n");
    __asm("dsb\n");__asm("dsb\n");
    __asm("dsb\n");
    __asm("dsb\n");
    __asm("dsb\n");
    __asm("dsb\n");
    __asm("dsb\n");
    __asm("dsb\n");
    __asm("dsb\n");
    __asm("dsb\n");
    __asm("dsb\n");


  __HAL_SPI_DISABLE(xSpiHandle);

  __enable_irq();

  while ((xSpiHandle->Instance->SR & SPI_FLAG_RXNE) != SPI_FLAG_RXNE);
  /* read the received data */
  *val = *(__IO uint8_t *) &xSpiHandle->Instance->DR;
  while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY);
}

void SPI_Read_nBytes(SPI_HandleTypeDef* xSpiHandle, uint8_t *val, uint8_t size)
{
  /* Interrupts should be disabled during this operation */
  __disable_irq();
  __HAL_SPI_ENABLE(xSpiHandle);

  /* Transfer loop */
  while (size > 1U)
  {
    /* Check the RXNE flag */
    if (xSpiHandle->Instance->SR & SPI_FLAG_RXNE)
    {
      /* read the received data */
      *val = *(__IO uint8_t *) &xSpiHandle->Instance->DR;
      val += sizeof(uint8_t);
      size--;
    }
  }
  __enable_irq();
}

void SPI_Write(SPI_HandleTypeDef* xSpiHandle, uint8_t val)
{
  /* check TXE flag */
  while ((xSpiHandle->Instance->SR & SPI_FLAG_TXE) != SPI_FLAG_TXE);

  /* Write the data */
  *((__IO uint8_t*) &xSpiHandle->Instance->DR) = val;

  /* Wait BSY flag */
  while ((xSpiHandle->Instance->SR & SPI_SR_TXE) != SPI_SR_TXE);
  while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim9.Instance)
	{
		sys_Prd_Clk = TRUE;
	}
	else
	{
		sys_Prd_Clk = FALSE;
	}
}

uint32_t vBat_ADC_Value(void)
{
	uint32_t vBat_ADC=0;
	HAL_ADC_Start(&hadc1);
	if (HAL_ADC_PollForConversion(&hadc1, 1000000) == HAL_OK)
	{
		vBat_ADC = HAL_ADC_GetValue(&hadc1);
	}
	HAL_ADC_Stop(&hadc1);
	return vBat_ADC;
}
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
#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
