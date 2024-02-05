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
uint16_t sec = 0;
uint16_t Curr_adc[ADC_BUFFER_SIZE] = {0};
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void Logging_ADCvalue(ADC_HandleTypeDef hadc, uint16_t buffer[], uint16_t size);
void Logging_ADCvalue(ADC_HandleTypeDef hadc, uint16_t buffer[], uint16_t size)
{
  if (HAL_ADC_Start(&hadc) != HAL_OK)
  {
    // Handle ADC start error
    Error_Handler();
  }
    // Poll for ADC conversion to be completed
  if (HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY) == HAL_OK)
  {
    static int i = 0 ;
    buffer[i++] = HAL_ADC_GetValue(&hadc);
    if(i > size - 1)
    {
      i = 0;
    }
  }
}
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

  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);

    qd_f_t Iqd_ref = MC_GetIqdrefMotor1_F();
    qd_f_t Iqd = MC_GetIqdMotor1_F();
    int16_t Phase_Peak = MCI_GetPhaseCurrentAmplitude(&Mci[M1]);

    if(!MC_GetCurrentFaultsMotor1())
    {
      printf("\n\nPast Fault code : %u, ", MC_GetOccurredFaultsMotor1());
      printf("Current Fault code : %u.\n\n", MC_GetCurrentFaultsMotor1());
      // printf("Phase Peak : %d, ",(int)Phase_Peak);
      printf("Power : %d, ",(int)MC_GetAveragePowerMotor1_F());
      printf("IPM TEMP : %u.\n\n", (uint8_t)IPM_temp);
      printf("Current Speed : %d, ", (int)MC_GetAverageMecSpeedMotor1_F());
      printf("Speed Target : %d.\n\n", (int)MC_GetMecSpeedReferenceMotor1_F());
      sprintf(float_buffer, "Iq_ref : %.2f, Iq : %.2f.\n\nId_ref : %.2f, Id : %.2f.\n\n", Iqd_ref.q, Iqd.q, Iqd_ref.d, Iqd.d);
      printf(float_buffer);
      printf("----- Run time : %u ------\n\n", sec+=1);
    }
    else
    {
      //suspend printf task
    }
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
    osDelay(1);
    /* Enable UART */
    detec_uart();

    /*logging adc value when no MC fault*/
    if(!MC_GetCurrentFaultsMotor1())
    {
      Logging_ADCvalue(hadc3, (uint16_t *)&Curr_adc, ADC_BUFFER_SIZE);
    }
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
  ctrl_rs485_pin(&U1, SET);
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  // ctrl_rs485_pin(&U1, RESET);
  return ch;
}
/* USER CODE END Application */

