/***********************************************************************************************//**
 * @file        app_EEPROM_Control.c
 * @brief       app_EEPROM_Control
 * @date        AUG 29, 2022
 * @version     1.0.0
 * @author      KARRY.WANG
 *
 * Revision History
 *------------------------------------------------------------------------------------------------
 *|Version   |Date       |Author         |Change ID      |Description                            |
 *|----------|-----------|---------------|---------------|---------------------------------------|
 *|1.0.0     |2022.08.29 |KARRY.WANG     |00000000000    |Initial version created                |
 **************************************************************************************************/
/*================================================================================================*=
 * INCLUDE FILES
 *================================================================================================*/
#include "app_EEPROM.h"

/*================================================================================================*=
 * LOCAL CONSTANTS
 *================================================================================================*/


/*================================================================================================*=
 * LOCAL VARIABLES
 *================================================================================================*/

/*================================================================================================*=
 * LOCAL MACROS
 *================================================================================================*/


/*================================================================================================*=
 * GLOBAL CONSTANTS
 *================================================================================================*/


/*================================================================================================*=
 * GLOBAL VARIABLES
 *================================================================================================*/
EE_DATA EE;

/*================================================================================================*=
 * LOCAL FUNCTIONS PROTOTYPE
 *================================================================================================*/


/*================================================================================================*=
 * LOCAL FUNCTIONS
 *================================================================================================*/
static void EE_W_Byte(uint16_t addr,uint8_t data)
{
	HAL_StatusTypeDef STATE;
	uint8_t tx_buff[2];

	tx_buff[0] = addr; /* write address */
	tx_buff[1] = data; /* write address */

	STATE = HAL_I2C_Master_Transmit(&hi2c1, EE_W_ADDRESS, tx_buff, 2, 1000);

	(void)STATE;
}

static void EE_R_Byte(uint16_t addr,uint8_t length ,uint8_t *data)
{
	HAL_StatusTypeDef STATE;
	uint8_t rx_buff[250]={0},cmd[2],cnt;
	cmd[0] = addr+1;
	//cmd[0] = addr;
	STATE = HAL_I2C_Master_Transmit(&hi2c1, EE_W_ADDRESS, cmd, 1 , 1000);

	if(STATE == HAL_OK){
		cnt = 5;
		while(HAL_I2C_Master_Receive(&hi2c1, EE_R_ADDRESS, data, length, 1000) != HAL_OK)
		{
			cnt --;
			if(cnt == 0) return;
		}
	}
	(void)rx_buff;
}
/*================================================================================================*=
 * GLOBAL FUNCTIONS
 *================================================================================================*/
void EE_Init_Read(void)
{
	/* EEPROM Initial Variable */
//	uint16_t AutoFlowSet = 0;
//	uint16_t AutoPresSet = 0;
//	uint16_t AutoTempSet = 0;

	EE_READ_BYTE(0,29,(uint8_t*)&EE);
#if 0
	/* flow rate set */
	AutoFlowSet |= EE.AutoFlow_L;
	AutoFlowSet |= EE.AutoFlow_H << 8;
	FlowInfo.TargetFlow = (float)AutoFlowSet / 10;
	modbus_slave_value_update(AUTO_FLOW_RATE_TAR_SET,&AutoFlowSet,1);
	/* pressure set */
	AutoPresSet |= EE.AutoPres_L;
	AutoPresSet |= EE.AutoPres_H << 8;
	PresInfo.Target_Pres = (float)AutoPresSet / 10;
	modbus_slave_value_update(AUTO_PRES_TAR_SET,&AutoPresSet,1);
	/* temp set */
	AutoTempSet |= EE.AutoTemp_L;
	AutoTempSet |= EE.AutoTemp_H << 8;
	TempInfo.Target_Temp = (float)AutoTempSet / 10;
	modbus_slave_value_update(AUTO_TEMPERATURE_TAR_SET,&AutoTempSet,1);
	/* pump duty set */
	PumpInfo.MAN_Duty = (float)EE.ManualPumpDuty;
	modbus_slave_value_update(MANUAL_PUMP_DUTY_SET,&PumpInfo.MAN_Duty,1);
	/* three way duty set */
	ThreeWayInfo.MAN_Duty = (float)EE.ManualThreeWayDuty;
	modbus_slave_value_update(MANUAL_VALVE_DUTY_SET,&ThreeWayInfo.MAN_Duty,1);
    /* Water leakage set switch */
	Water.Leak_EnableSW.NO._1 =  EE.Leakage_SET_SW & 1;
	Water.Leak_EnableSW.NO._2 = (EE.Leakage_SET_SW >> 1 ) & 1;
	Water.Leak_EnableSW.NO._3 = (EE.Leakage_SET_SW >> 2 ) & 1;
	Water.Leak_EnableSW.NO._4 = (EE.Leakage_SET_SW >> 3 ) & 1;
	Water.Leak_EnableSW.NO._5 = (EE.Leakage_SET_SW >> 4 ) & 1;
	Water.Leak_EnableSW.NO._6 = (EE.Leakage_SET_SW >> 5 ) & 1;
	stModb.bitCoil.bit.leakageSetSW1 = Water.Leak_EnableSW.NO._1;
	stModb.bitCoil.bit.leakageSetSW2 = Water.Leak_EnableSW.NO._2;
	stModb.bitCoil.bit.leakageSetSW3 = Water.Leak_EnableSW.NO._3;
	stModb.bitCoil.bit.leakageSetSW4 = Water.Leak_EnableSW.NO._4;
	stModb.bitCoil.bit.leakageSetSW5 = Water.Leak_EnableSW.NO._5;
	stModb.bitCoil.bit.leakageSetSW6 = Water.Leak_EnableSW.NO._6;
	/* Water leakage alarm switch */
	Water.Leak_AlarmSW.NO._1 =  EE.Leakage_ALM_SW & 1;
	Water.Leak_AlarmSW.NO._2 = (EE.Leakage_ALM_SW >> 1 ) & 1;
	Water.Leak_AlarmSW.NO._3 = (EE.Leakage_ALM_SW >> 2 ) & 1;
	Water.Leak_AlarmSW.NO._4 = (EE.Leakage_ALM_SW >> 3 ) & 1;
	Water.Leak_AlarmSW.NO._5 = (EE.Leakage_ALM_SW >> 4 ) & 1;
	Water.Leak_AlarmSW.NO._6 = (EE.Leakage_ALM_SW >> 5 ) & 1;
	stModb.bitCoil.bit.leakageAlarmSW1 = Water.Leak_AlarmSW.NO._1;
	stModb.bitCoil.bit.leakageAlarmSW2 = Water.Leak_AlarmSW.NO._2;
	stModb.bitCoil.bit.leakageAlarmSW3 = Water.Leak_AlarmSW.NO._3;
	stModb.bitCoil.bit.leakageAlarmSW4 = Water.Leak_AlarmSW.NO._4;
	stModb.bitCoil.bit.leakageAlarmSW5 = Water.Leak_AlarmSW.NO._5;
	stModb.bitCoil.bit.leakageAlarmSW6 = Water.Leak_AlarmSW.NO._6;
	/* Pump redundant day set */
	if(EE.PumpRedundantDay == 0xff) EE.PumpRedundantDay = 0;
	PumpInfo.SetWorkDay = (int16_t)EE.PumpRedundantDay;
	modbus_slave_value_update(PUMP_REDUNDANT_INTERVAL,&PumpInfo.SetWorkDay,1);
	/* Pump1 status */
	if(EE.Pump1_Status == 0xff) EE.Pump1_Status = PUMP_REDUNDANT;
	PumpInfo.Pump1 = EE.Pump1_Status;
	modbus_slave_value_update(PUMP_1_STATUS,&PumpInfo.Pump1,1);
	/* Pump2 status */
	if(EE.Pump2_Status == 0xff) EE.Pump2_Status = PUMP_ENABLE;
	PumpInfo.Pump2 = EE.Pump2_Status;
	modbus_slave_value_update(PUMP_2_STATUS,&PumpInfo.Pump2,1);
	/* Pump redundant switch */
	if(EE.PumpRedundantSW == 0xff) EE.PumpRedundantSW = 0;
	PumpInfo.RED_Switch = EE.PumpRedundantSW;
	stModb.bitCoil.bit.pumpRedSW = PumpInfo.RED_Switch;
#endif
}

void EE_WRITE_BYTE(uint8_t AddStr , uint8_t *data)
{
  uint8_t W_Data = 0,index = 0;
  if(AddStr == 0) index = 0;
  else index = AddStr;
  W_Data = *(data + index);
  AddStr = AddStr+1;
  EE_W_Byte(AddStr ,W_Data);
//  Delay_Func(10);
}

void EE_READ_BYTE(uint8_t AddStr ,uint8_t length ,uint8_t *data)
{
	EE_R_Byte(AddStr ,length ,data);
}

void EE_READ_ALL(void)
{
	HAL_StatusTypeDef STATE;
	uint8_t rx_buff[250]={0},cmd[2],cnt;
	cmd[0] = 1;
	STATE = HAL_I2C_Master_Transmit(&hi2c1, EE_W_ADDRESS, cmd, 1 , 1000);
	if(STATE == HAL_OK){
		cnt = 5;
		while(HAL_I2C_Master_Receive(&hi2c1, EE_R_ADDRESS, (uint8_t *)&EE, 12, 1000) != HAL_OK)
		{
			cnt --;
			if(cnt == 0) return;
		}
	}
	(void)rx_buff;
}
/*================================================================================================*=
 * END OF FILE
 *================================================================================================*/
