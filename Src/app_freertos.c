/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE BEGIN Header_StartPrintTask */
/**
  * @brief  Function implementing the Print_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartPrintTask */
void StartPrintTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  static int sec = 0;
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);

    qd_f_t Iqd_ref = MC_GetIqdrefMotor1_F();
    qd_f_t Iqd = MC_GetIqdMotor1_F();

    printf("IPM TEMP : %d, ", (uint8_t)IPM_temp);
    printf("Fault code : %d.\n\n", MC_GetOccurredFaultsMotor1());
    printf("Current Speed : %d, ", (int)MC_GetAverageMecSpeedMotor1_F());
    printf("Speed Target : %d.\n\n", (int)MC_GetMecSpeedReferenceMotor1_F());
    // printf("Power : %d, \n\n",(int)MC_GetAveragePowerMotor1_F());
    sprintf(float_buffer, "Iq_ref : %.2f, Iq : %.2f.\n\nId_ref : %.2f, Id : %.2f.\n\n", Iqd_ref.q, Iqd.q, Iqd_ref.d, Iqd.d);
    printf(float_buffer);
    printf("----- Run time : %d ------\n\n", sec+=1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartModbusTask */
/**
* @brief Function implementing the Modbus_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartModbusTask */
void StartModbusTask(void *argument)
{
  /* USER CODE BEGIN StartModbusTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(10);
    /* Enable UART */
    detec_uart();
  }
  /* USER CODE END StartModbusTask */
}

/* USER CODE BEGIN Header_StartTemperatureTask */
/**
* @brief Function implementing the Temperature_Tas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTemperatureTask */
void StartTemperatureTask(void *argument)
{
  /* USER CODE BEGIN StartTemperatureTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(100);
    if (HAL_ADC_Start(&hadc2) != HAL_OK)
    {
    	// Handle ADC start error
    	Error_Handler();
    }

    // Poll for ADC conversion to be completed
    if (HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY) == HAL_OK)
    {
      temp_adc = HAL_ADC_GetValue(&hadc2);
    }
    Temp_Average(temp_adc, &IPM_temp);
  }
  /* USER CODE END StartTemperatureTask */
}

int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}
/* USER CODE END Application */
