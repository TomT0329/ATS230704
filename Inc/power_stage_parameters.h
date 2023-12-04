
/**
  ******************************************************************************
  * @file    power_stage_parameters.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the parameters needed for the Motor Control SDK
  *          in order to configure a power stage.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef POWER_STAGE_PARAMETERS_H
#define POWER_STAGE_PARAMETERS_H

/************************
 *** Motor Parameters ***
 ************************/

/************* PWM Driving signals section **************/
#define HW_DEAD_TIME_NS               8000 /*!< Dead-time inserted
                                                         by HW if low side signals
                                                         are not used */
/*********** Bus voltage sensing section ****************/
#define VBUS_PARTITIONING_FACTOR      0.006493506493506494 /*!< It expresses how
                                                       much the Vbus is attenuated
                                                       before being converted into
                                                       digital value */
#define NOMINAL_BUS_VOLTAGE_V         400
/******** Current reading parameters section ******/
/*** Topology ***/
#define SINGLE_SHUNT_PHASE_SHIFT

#define RSHUNT                        0.01175

/*  ICSs gains in case of isolated current sensors,
        amplification gain for shunts based sensing */
#define AMPLIFICATION_GAIN            3.01

/*** Noise parameters ***/
#define TNOISE_NS                     5000
#define TRISE_NS                      5750
#define MAX_TNTR_NS TRISE_NS

/************ Temperature sensing section ***************/
/* V[V]=V0+dV/dT[V/Celsius]*(T-T0)[Celsius]*/
#define V0_V                          2.2 /*!< in Volts */
#define T0_C                          25 /*!< in Celsius degrees */
#define dV_dT                         0.028 /*!< V/Celsius degrees */
#define T_MAX                         100 /*!< Sensor measured
                                                       temperature at maximum
                                                       power stage working
                                                       temperature, Celsius degrees */

#endif /*POWER_STAGE_PARAMETERS_H*/
/******************* (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/
