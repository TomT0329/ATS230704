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
#include "math.h"
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
float PFC_current[AC_PERIOD] = {0.0};
float PFC_voltage_total = 0.0;
float PFC_voltage_rms = 0.0;
float PFC_voltage[AC_PERIOD] = {0.0};
float PFC_current_total = 0.0;
float PFC_current_rms = 0.0;
float PFC_power = 0.0;
uint16_t sec = 0;
uint16_t Curr_adc[ADC_BUFFER_SIZE] = {0};
float Error_buffer[ERROR_BUFFER_SIZE] = {0};
float Current_Speed;
float Speed_Target;
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
    static uint32_t i = 0 ;
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
    int16_t Phase_Peak_S16A = MCI_GetPhaseCurrentAmplitude(&Mci[M1]);
    float Current_Amp = (Phase_Peak_S16A * 3.3) / (65536 * RSHUNT * AMPLIFICATION_GAIN);
    printf("\n\nRamp Speed Target : %d. ", (int)MC_GetLastRampFinalSpeedM1_F());
    printf("Ramp Command Completed: %d.\n\n", MC_HasRampCompletedMotor1());
    printf("Power : %d, ",(int)MC_GetAveragePowerMotor1_F());
    printf("IPM TEMP : %u.\n\n", (uint8_t)IPM_temp);
    printf("Current Speed : %d, ", (int)Current_Speed);
    printf("Speed Target : %d.\n\n", (int)Speed_Target);
    printf("PFC voltage : %d. PFC current : %d.\n\n", (int)PFC_voltage_rms, (int)PFC_current_rms);
    sprintf(float_buffer, "Iq_ref : %.2f, Iq : %.2f, Ia_pk : %.1f.\n\nId_ref : %.2f, Id : %.2f.\n\n", Iqd_ref.q, Iqd.q, Current_Amp, Iqd_ref.d, Iqd.d);
    printf(float_buffer);
    printf("----- Run time : %u ------\n\n", sec+=1);

    modbus_slave_value_update();
    System_Alarm_Handler();
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
    
  }
  /* USER CODE END StartModbusTask */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the StartSensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument)
{
  /* USER CODE BEGIN StartSensorTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
    Temp_Average(temp_adc[0], &IPM_temp);

    static uint32_t j = 0;
    static uint32_t k = 0;

    PFC_current[j++] = PFC_GetCurrent(temp_adc[1]);
    PFC_voltage[k++] = PFC_GetVoltage(temp_adc[2]);
    PFC_current_total = PFC_current_total + powf(PFC_current[j],2.0);
    PFC_voltage_total = PFC_voltage_total + powf(PFC_voltage[k],2.0);

    if(j == AC_PERIOD -1)
    {
      PFC_current_rms = sqrtf(PFC_current_total / AC_PERIOD);
      PFC_voltage_rms = sqrtf(PFC_voltage_total / AC_PERIOD);
      
      PFC_power = PFC_current_rms * PFC_voltage_rms;
      
      PFC_current_total = 0;
      PFC_voltage_total = 0;
      j = 0;
      k = 0;
    }

    static uint32_t i = 0;

    if(MCI_GetSTMState(&Mci[M1]) == RUN)
    {
      Current_Speed = MC_GetAverageMecSpeedMotor1_F();
      Speed_Target = MC_GetMecSpeedReferenceMotor1_F();
      Error_buffer[i++] = Speed_Target - Current_Speed;
      if(i > ERROR_BUFFER_SIZE -1)
      {
        i=0;
      }
    }

  }
  /* USER CODE END StartSensorTask */
}

/* USER CODE END Application */

