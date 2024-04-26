/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

#include "motorcontrol.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include "app_Modbus_RTU.h"
#include "app_temperature.h"
#include "flash_if.h"
#include "app_System_Protect.h"
#include "app_SBSFU.h"
#include <string.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern char float_buffer[];
extern UART_HandleTypeDef huart1;
extern float IPM_temp;
// extern uint16_t Curr_adc[];
extern float Error_buffer[];
extern uint16_t ACC_Time;
extern const float Init_Ramp_Speed;
extern float PFC_voltage_rms;
extern float PFC_current_rms;
extern uint32_t OneShunt_ADC;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern const uint16_t Init_Ramp_Time;
extern const float ACC_Value;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern int __io_putchar(int ch);
extern void StartReception(void);
extern void Logging_ADCvalue(ADC_HandleTypeDef hadc, uint16_t buffer[], uint16_t size);
void ADC2_DMA_Init(uint32_t *AdcValue);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define M1_OCP_Pin GPIO_PIN_13
#define M1_OCP_GPIO_Port GPIOC
#define DEBUG_LED_RED_Pin GPIO_PIN_14
#define DEBUG_LED_RED_GPIO_Port GPIOC
#define DEBUG_LED_GREEN_Pin GPIO_PIN_15
#define DEBUG_LED_GREEN_GPIO_Port GPIOC
#define M1_IPM_TEMP_Pin GPIO_PIN_2
#define M1_IPM_TEMP_GPIO_Port GPIOC
#define M1_BUS_VOLTAGE_Pin GPIO_PIN_2
#define M1_BUS_VOLTAGE_GPIO_Port GPIOA
#define VAC_VSEN_Pin GPIO_PIN_4
#define VAC_VSEN_GPIO_Port GPIOA
#define PFC_I_CS_Pin GPIO_PIN_5
#define PFC_I_CS_GPIO_Port GPIOA
#define M1_CURR_AMPL_Pin GPIO_PIN_1
#define M1_CURR_AMPL_GPIO_Port GPIOB
#define PFC_EN_Pin GPIO_PIN_2
#define PFC_EN_GPIO_Port GPIOB
#define M1_PWM_UL_Pin GPIO_PIN_13
#define M1_PWM_UL_GPIO_Port GPIOB
#define M1_PWM_VL_Pin GPIO_PIN_14
#define M1_PWM_VL_GPIO_Port GPIOB
#define M1_PWM_WL_Pin GPIO_PIN_15
#define M1_PWM_WL_GPIO_Port GPIOB
#define M1_PWM_UH_Pin GPIO_PIN_8
#define M1_PWM_UH_GPIO_Port GPIOA
#define M1_PWM_VH_Pin GPIO_PIN_9
#define M1_PWM_VH_GPIO_Port GPIOA
#define M1_PWM_WH_Pin GPIO_PIN_10
#define M1_PWM_WH_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define U1_DIR_Pin GPIO_PIN_12
#define U1_DIR_GPIO_Port GPIOC
#define U2_DIR_Pin GPIO_PIN_2
#define U2_DIR_GPIO_Port GPIOD
#define USER_TX_Pin GPIO_PIN_3
#define USER_TX_GPIO_Port GPIOB
#define USER_RX_Pin GPIO_PIN_4
#define USER_RX_GPIO_Port GPIOB
#define DEBUG_TX_Pin GPIO_PIN_6
#define DEBUG_TX_GPIO_Port GPIOB
#define DEBUG_RX_Pin GPIO_PIN_7
#define DEBUG_RX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define RX_BUFFER_SIZE   252 // /*data bytes*/ + 12
#define DATA_BYTES      240
#define FLASH_DATA_BYTES      ADC_BUFFER_SIZE * 2
#define ADC_BUFFER_SIZE 2500 // uint16_t
#define ERROR_BUFFER_SIZE 500 // uint16_t
#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))
#define FLOAT_BUFFER_SIZE 100

#define MAX_ACC_TIME	(uint16_t)100000
#define MIN_ACC_TIME	(uint16_t)1250

#define CLAMP(x, min_val, max_val) do { \
                                        if ((x) < (min_val)) { \
                                            (x) = (min_val); \
                                        } \
                                        else if ((x) > (max_val)) { \
                                            (x) = (max_val); \
                                        } \
                                    } while(0)


#define MAX_SPEED_01HZ	820
#define MIN_SPEED_01HZ	250
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
