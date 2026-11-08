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
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
	uint8_t bmp_initialized = 0;
	uint8_t bmp_chip_id = 0;
	uint8_t bmp_address = 0x76 << 1;
	float temperature_c = 0.0f;
	float humidity_percent = 0.0f;
	float pressure_hpa = 0.0f;
	float reference_pressure_hpa = 0.0f;
	float altitude_m = 0.0f;
	uint8_t aht_data[7];
	uint8_t aht_found = 0;
	uint8_t bmp_found = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct
{
    uint16_t T1;
    int16_t T2, T3;
    uint16_t P1;
    int16_t P2, P3, P4, P5, P6, P7, P8, P9;
} BMP280_Calibration;

BMP280_Calibration bmp_cal;
int32_t bmp_t_fine;

HAL_StatusTypeDef BMP280_Init(void)
{
    uint8_t calibration[24];
    uint8_t config = 0x27;

    // Read factory calibration coefficients
    if (HAL_I2C_Mem_Read(&hi2c1, bmp_address,
                        0x88, I2C_MEMADD_SIZE_8BIT,
                        calibration, 24, 100) != HAL_OK)
        return HAL_ERROR;

    bmp_cal.T1 = calibration[0] | (calibration[1] << 8);
    bmp_cal.T2 = (int16_t)(calibration[2] | (calibration[3] << 8));
    bmp_cal.T3 = (int16_t)(calibration[4] | (calibration[5] << 8));

    bmp_cal.P1 = calibration[6] | (calibration[7] << 8);
    bmp_cal.P2 = (int16_t)(calibration[8] | (calibration[9] << 8));
    bmp_cal.P3 = (int16_t)(calibration[10] | (calibration[11] << 8));
    bmp_cal.P4 = (int16_t)(calibration[12] | (calibration[13] << 8));
    bmp_cal.P5 = (int16_t)(calibration[14] | (calibration[15] << 8));
    bmp_cal.P6 = (int16_t)(calibration[16] | (calibration[17] << 8));
    bmp_cal.P7 = (int16_t)(calibration[18] | (calibration[19] << 8));
    bmp_cal.P8 = (int16_t)(calibration[20] | (calibration[21] << 8));
    bmp_cal.P9 = (int16_t)(calibration[22] | (calibration[23] << 8));

    if (bmp_cal.P1 == 0)
        return HAL_ERROR;

    // Temperature x1, pressure x1, normal mode
    return HAL_I2C_Mem_Write(&hi2c1, bmp_address,
                            0xF4, I2C_MEMADD_SIZE_8BIT,
                            &config, 1, 100);
}

HAL_StatusTypeDef BMP280_ReadPressure(float *pressure)
{
    uint8_t data[6];

    if (HAL_I2C_Mem_Read(&hi2c1, bmp_address,
                        0xF7, I2C_MEMADD_SIZE_8BIT,
                        data, 6, 100) != HAL_OK)
        return HAL_ERROR;

    int32_t adc_P = ((uint32_t)data[0] << 12) |
                    ((uint32_t)data[1] << 4) |
                    (data[2] >> 4);

    int32_t adc_T = ((uint32_t)data[3] << 12) |
                    ((uint32_t)data[4] << 4) |
                    (data[5] >> 4);

    int32_t var1 = ((((adc_T >> 3) -
                     ((int32_t)bmp_cal.T1 << 1))) *
                     (int32_t)bmp_cal.T2) >> 11;

    int32_t var2 = (((((adc_T >> 4) -
                      (int32_t)bmp_cal.T1) *
                     ((adc_T >> 4) -
                      (int32_t)bmp_cal.T1)) >> 12) *
                     (int32_t)bmp_cal.T3) >> 14;

    bmp_t_fine = var1 + var2;

    int64_t pvar1 = (int64_t)bmp_t_fine - 128000;
    int64_t pvar2 = pvar1 * pvar1 * bmp_cal.P6;
    pvar2 += (pvar1 * bmp_cal.P5) << 17;
    pvar2 += ((int64_t)bmp_cal.P4) << 35;

    pvar1 = ((pvar1 * pvar1 * bmp_cal.P3) >> 8) +
            ((pvar1 * bmp_cal.P2) << 12);

    pvar1 = (((((int64_t)1) << 47) + pvar1) *
             bmp_cal.P1) >> 33;

    if (pvar1 == 0)
        return HAL_ERROR;

    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - pvar2) * 3125) / pvar1;

    pvar1 = (bmp_cal.P9 * (p >> 13) * (p >> 13)) >> 25;
    pvar2 = (bmp_cal.P8 * p) >> 19;

    p = ((p + pvar1 + pvar2) >> 8) +
        (((int64_t)bmp_cal.P7) << 4);

    *pressure = (float)p / 25600.0f;

    return HAL_OK;
}
HAL_StatusTypeDef AHT20_Read(float *temperature, float *humidity)
{
    uint8_t trigger[3] = {0xAC, 0x33, 0x00};
    uint8_t data[7];

    if (HAL_I2C_Master_Transmit(&hi2c1, 0x38 << 1,
                                trigger, 3, 100) != HAL_OK)
        return HAL_ERROR;

    HAL_Delay(80);

    if (HAL_I2C_Master_Receive(&hi2c1, 0x38 << 1,
                               data, 7, 100) != HAL_OK)
        return HAL_ERROR;

    if (data[0] & 0x80)
        return HAL_BUSY;

    uint32_t raw_humidity =
        ((uint32_t)data[1] << 12) |
        ((uint32_t)data[2] << 4) |
        (data[3] >> 4);

    uint32_t raw_temperature =
        ((uint32_t)(data[3] & 0x0F) << 16) |
        ((uint32_t)data[4] << 8) |
        data[5];

    *humidity = (raw_humidity * 100.0f) / 1048576.0f;
    *temperature = (raw_temperature * 200.0f) / 1048576.0f - 50.0f;

    return HAL_OK;
}
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED2);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */

      // Check AHT20
      aht_found = (
          HAL_I2C_IsDeviceReady(
              &hi2c1, 0x38 << 1, 3, 100
          ) == HAL_OK
      );

      // Check BMP280
      bmp_found = (
          HAL_I2C_IsDeviceReady(
              &hi2c1, 0x76 << 1, 3, 100
          ) == HAL_OK
      ) || (
          HAL_I2C_IsDeviceReady(
              &hi2c1, 0x77 << 1, 3, 100
          ) == HAL_OK
      );



      if (aht_found)
      {
          AHT20_Read(&temperature_c, &humidity_percent);
      }

      if (bmp_found)
      {
          if (HAL_I2C_IsDeviceReady(&hi2c1, 0x76 << 1, 3, 100) == HAL_OK)
              bmp_address = 0x76 << 1;
          else
              bmp_address = 0x77 << 1;

          HAL_I2C_Mem_Read(&hi2c1, bmp_address,
                           0xD0, I2C_MEMADD_SIZE_8BIT,
                           &bmp_chip_id, 1, 100);
      }

      if (bmp_found)
      {
          if (!bmp_initialized)
          {
              bmp_initialized = (BMP280_Init() == HAL_OK);
          }

          if (bmp_initialized)
          {
              if (BMP280_ReadPressure(&pressure_hpa) == HAL_OK)
              {
                  if (reference_pressure_hpa == 0.0f)
                  {
                      reference_pressure_hpa = pressure_hpa;
                  }

                  altitude_m = 44330.0f *
                      (1.0f - powf(pressure_hpa / reference_pressure_hpa,
                                    0.1903f));
              }
          }
      }
      else
      {
          bmp_initialized = 0;
      }

      if (aht_found && bmp_found)
      {
          BSP_LED_Toggle(LED2);
          HAL_Delay(250);
      }
      else
      {
          BSP_LED_Toggle(LED2);
          HAL_Delay(1000);
      }
  }
  /* USER CODE END 3 */

  }
// End of main()

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
#ifdef USE_FULL_ASSERT
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
