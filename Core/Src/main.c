/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include "arm_math.h"
#include "Typedef.h"
#include "arm_const_structs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define a1_reset (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0) == RESET)
#define a2_reset (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1) == RESET)
#define a3_reset (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) == RESET)
#define a4_reset (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) == RESET)
#define a5_reset (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4) == RESET)
#define a6_reset (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5) == RESET)

#define a1_set (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0) == SET)
#define a2_set (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1) == SET)
#define a3_set (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) == SET)
#define a4_set (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) == SET)
#define a5_set (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4) == SET)
#define a6_set (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5) == SET)

#define threshold 10000
#define data_len 320
#define FFT_LEN 128
#define overlap 64
#define win_num ((data_len-FFT_LEN)/(FFT_LEN-overlap))+1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
extern TsOUT* dnn_compute(TsIN*, TsInt*);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int ring_arr[6][data_len] = {0}, a1, a2, a3, a4, a5, a6, hx711_ready = 0, data_ready = 0, hx711_cnt = 0, ring_cnt = 0, avg_ready = 0, avg_cnt = 0, avg_arr[6] = {0},
		i, j, m, n, avg_start = 1, uart_start = 0, stop_bit = 300, start_bit = 0, th_cnt = 0, sample_flag = 0;
float32_t data_arr[6][data_len];
float32_t fft_arr[FFT_LEN<<1] = {0}, input_arr[6][win_num][FFT_LEN>>1] = {0}, abs_arr[FFT_LEN];
float32_t max_value, max_arr[win_num], min_value, min_arr[win_num];
uint32_t max_index;
int final_arr[6][data_len];
char tx[48];

TsOUT *result_ptr;
TsInt errorcode;
extern TsOUT dnn_buffer2[320];
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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(a1_reset && a2_reset && a3_reset && a4_reset && a5_reset && a6_reset && (hx711_ready == 0)) 
		{
			hx711_ready = 1;
			a1 = a2 = a3 = a4 = a5 = a6 = 0;
			TIM3->CNT = 0;
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		}
		if(data_ready && hx711_ready)
		{
			data_ready = 0;
			if(hx711_cnt < 24)
			{
				a1 = a1 << 1;
				a2 = a2 << 1;
				a3 = a3 << 1;
				a4 = a4 << 1;
				a5 = a5 << 1;
				a6 = a6 << 1;
				a1 = a1 + a1_set;
				a2 = a2 + a2_set;
				a3 = a3 + a3_set;
				a4 = a4 + a4_set;
				a5 = a5 + a5_set;
				a6 = a6 + a6_set;
			}
			else if(hx711_cnt == 24)
			{
				a1 = a1 | (0xFF000000*((a1&0x800000) == 0x800000));
				a2 = a2 | (0xFF000000*((a2&0x800000) == 0x800000));
				a3 = a3 | (0xFF000000*((a3&0x800000) == 0x800000));
				a4 = a4 | (0xFF000000*((a4&0x800000) == 0x800000));
				a5 = a5 | (0xFF000000*((a5&0x800000) == 0x800000));
				a6 = a6 | (0xFF000000*((a6&0x800000) == 0x800000));
			}
			else if(hx711_cnt == 26)
			{
				hx711_ready = 0;
				hx711_cnt = 0;
				ring_arr[0][ring_cnt] = a1;
				ring_arr[1][ring_cnt] = a2;
				ring_arr[2][ring_cnt] = a3;
				ring_arr[3][ring_cnt] = a4;
				ring_arr[4][ring_cnt] = a5;
				ring_arr[5][ring_cnt] = a6;
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
				ring_cnt++;
				ring_cnt = (ring_cnt < data_len) * ring_cnt;
				if((avg_ready == 0)&&(avg_start == 1))
				{
					if(avg_cnt == 0)
					{
						for(i=0; i<6; i++)
							avg_arr[i] = 0;
					}
					avg_arr[0] += a1;
					avg_arr[1] += a2;
					avg_arr[2] += a3;
					avg_arr[3] += a4;
					avg_arr[4] += a5;
					avg_arr[5] += a6;
					avg_cnt++;
					if(avg_cnt >= 256)
					{
						for(i=0; i<6; i++)
						{
							avg_arr[i] = avg_arr[i]>>8;
						}
						avg_ready = 1;
						avg_cnt = 0;
					}
				}
				if(avg_ready)
				{
					if(((abs(a1-avg_arr[0])>threshold) || (abs(a2-avg_arr[1])>threshold) || (abs(a3-avg_arr[2])>threshold) || (abs(a4-avg_arr[3])>threshold) || (abs(a6-avg_arr[5])>threshold)) && (sample_flag == 0))	//|| (abs(a5-avg_arr[4])>threshold) 
					{
						th_cnt++;
						sample_flag = th_cnt > 30;
					}
					else
					{
						th_cnt = 0;
					}
					if(stop_bit == 500 && sample_flag)
					{
						th_cnt = 0;
						sample_flag = 0;
						start_bit = ring_cnt-119;
						stop_bit = ring_cnt + 200;
						start_bit = start_bit + (start_bit<0)*data_len;
						stop_bit = stop_bit - (stop_bit>=data_len)*data_len;
					}
					if(stop_bit == ring_cnt)
					{
						stop_bit = 500;
						uart_start = 1;
						avg_start = 0;
						avg_ready = 0;
					}
				}
			}
			hx711_cnt++;
		}
		if(uart_start)
		{
			sample_flag = 0;
			for(j=start_bit; j<start_bit+data_len; j++)
			{
				i = j - (j>=data_len)*data_len;
				for(n=0; n<6; n++)
				{
					final_arr[n][j-start_bit] = ring_arr[n][i];
				}
				//sprintf(tx, "%07d\t%07d\t%07d\t%07d\t%07d\t%07d\n" ,ring_arr[0][i], ring_arr[1][i], ring_arr[2][i], ring_arr[3][i], ring_arr[4][i], ring_arr[5][i]);
				//HAL_UART_Transmit(&huart1, (uint8_t*)tx, 48, 100);
			}
			for(i=0; i<6; i++)
			{
				for(j=1; j<319; j++)
				{
					if(abs(final_arr[i][j-1] - final_arr[i][j]) > 3000)
					{
						final_arr[i][j] = (final_arr[i][j-1]+final_arr[i][j+1])/2;
					}
				}
			}
			for(i=0; i<6; i++)
			{
				for(j=0; j<data_len; j++)
				{
					data_arr[i][j] = (float32_t)final_arr[i][j];
				}
			}
			/*for(i=0; i<320; i++)
			{
				sprintf(tx, "%07d\t%07d\t%07d\t%07d\t%07d\t%07d\n" ,final_arr[0][i], final_arr[1][i], final_arr[2][i], final_arr[3][i], final_arr[4][i], final_arr[5][i]);
				HAL_UART_Transmit(&huart1, (uint8_t*)tx, 48, 100);
			}*/
			for(i=0; i<6; i++)
			{
				for(n=0; n<(data_len-FFT_LEN+1); n+=(FFT_LEN-overlap))
				{
					for(j=n; j<n+FFT_LEN; j++)
					{
						fft_arr[(j-n)*2] = data_arr[i][j];
						fft_arr[(j-n)*2+1] = 0;
					}
					arm_cfft_f32(&arm_cfft_sR_f32_len128, fft_arr, 0, 1);
					for(m=0; m<FFT_LEN; m++)
					{
						abs_arr[m] = fft_arr[m+2];
					}
					arm_cmplx_mag_f32(abs_arr, input_arr[i][n/(FFT_LEN-overlap)], FFT_LEN>>1);
				}
			}
			for(i=0; i<6; i++)
			{
				for(j=0; j<win_num; j++)
				{
					for(n=0; n<(FFT_LEN>>1); n++)
					{
						input_arr[i][j][n] /= FFT_LEN;
					}
				}
			}
			for(i=0; i<6; i++)
			{
				for(j=0; j<win_num; j++)
				{
					arm_max_f32(input_arr[i][j], (FFT_LEN>>1), max_arr+j, &max_index);
					arm_min_f32(input_arr[i][j], (FFT_LEN>>1), min_arr+j, &max_index);
				}
				arm_max_f32(max_arr, win_num, &max_value, &max_index);
				arm_min_f32(min_arr, win_num, &min_value, &max_index);
				for(j=0; j<win_num; j++)
				{
					for(n=0; n<(FFT_LEN>>1); n++)
					{
						input_arr[i][j][n] -= min_value;
						input_arr[i][j][n] /= (max_value-min_value);
					}
				}
			}
			result_ptr = dnn_compute(&input_arr[0][0][0], &errorcode);
			arm_max_f32(dnn_buffer2, 7, &max_value, &max_index);
			sprintf(tx, "%1d,%04f", max_index, max_value);
			HAL_UART_Transmit(&huart1, (uint8_t*)tx, 6, 100);
			//HAL_Delay(350);
			uart_start = 0;
			avg_start = 1;
		}
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 5;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : PD0 PD1 PD2 PD3
                           PD4 PD5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
