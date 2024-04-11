/***********************************************************************************************//**
 * @file        app_System_Protect.c
 * @brief       app_System_Protect
 * @date        April.2, 2024
 * @version     1.0.0
 * @author      TOM.TANG
 *
 * Revision History
 *------------------------------------------------------------------------------------------------
 *|Version   |Date       |Author         |Change ID      |Description                            |
 *|----------|-----------|---------------|---------------|---------------------------------------|
 *|1.0.0     |2024.04.02  |TOM.TANG       |00000000000    |Initial version created               |
 **************************************************************************************************/
/*================================================================================================*=
 * INCLUDE FILES
 *================================================================================================*/
#include <string.h>
#include <math.h>
#include <stdint.h>
#include "app_System_Protect.h"
#include "bus_voltage_sensor.h"
#include "mc_config.h"
#include "main.h"
/*================================================================================================*=
 * GLOBAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
 *================================================================================================*/

/*================================================================================================*=
 * GLOBAL MACROS
 *================================================================================================*/

/*================================================================================================*=
 * GLOBAL CONSTANTS
 *================================================================================================*/

/*================================================================================================*=
 * GLOBAL VARIABLES
 *================================================================================================*/
SYSCTRL SysCtrl;
ALARM_INFO Alarm;
/*================================================================================================*=
 * LOCAL CONSTANTS
 *================================================================================================*/

/*================================================================================================*=
 * LOCAL FUNCTIONS PROTOTYPE
 *================================================================================================*/

/*================================================================================================*=
 * LOCAL FUNCTIONS
 *================================================================================================*/

void CompressOverCurrent()
{
    //TBD
}
void IpmOverCurrent()
{
    if(MC_GetOccurredFaultsMotor1() == MC_OVER_CURR)
    {
        Alarm.IpmOverCurrent = SET;
    }
    else
    {
        Alarm.IpmOverCurrent = RESET;
    }
}
void ADCoffsetAbnormal()
{
    if(MCI_GetSTMState(&Mci[M1]) == IDLE)
    {
        if((OneShunt_ADC > ADCvalue_H
		|| OneShunt_ADC < ADCvalue_L)
        && (Alarm.ADCoffsetAbnormal == RESET))
        {
            if(SysCtrl.count_protect[eADCoffsetAbnormal].last1 > ADCoffsetAbnormal_Last)
            {
                Alarm.ADCoffsetAbnormal = SET;
            }
            else
            {
                SysCtrl.count_protect[eADCoffsetAbnormal].last1++;
            }
        }
        else
        {
            SysCtrl.count_protect[eADCoffsetAbnormal].last1 = 0;
        }

        //no resume mechanism for now.
    }
}
void OutputLosePhase()
{
    //detect Ia current in the starup state
    if(MCI_GetSTMState(&Mci[M1]) == START 
    && SysCtrl.count_protect[eOutputLosePhase].timer < OutputLosePhase_Timer)
    {
    	ab_f_t Iab = MC_GetIabMotor1_F();

        if(Iab.a < PhaseLocateCurrent_L 
        && Alarm.OutputLosePhase == RESET)
        {
            if(SysCtrl.count_protect[eOutputLosePhase].last1 > OutputLosePhase_Last)
            {
                Alarm.OutputLosePhase = SET;
                Mci[M1].PastFaults = MC_START_UP;
                MC_StopSpeedRampMotor1();
                MC_StopMotor1();
                //execute init speed ramp to 3000rpm to protect compressor
                MCI_ExecSpeedRamp_F(&Mci[M1],Ramp_Speed,Ramp_Time);
            }
            else
            {
                SysCtrl.count_protect[eOutputLosePhase].last1 ++;
            }
        }
        else
        {
            SysCtrl.count_protect[eOutputLosePhase].last1 = 0;
            //only detect the first 3 sec
            SysCtrl.count_protect[eOutputLosePhase].timer ++;
        }

        //no resume mechanism for now.
    }
    else
    {
        SysCtrl.count_protect[eOutputLosePhase].timer = 0;
    }
}
void CompStarupFail()
{
    if(MC_GetOccurredFaultsMotor1() == MC_START_UP)
    {
        Alarm.CompStarupFail = SET;
    }
    else
    {
        Alarm.CompStarupFail = RESET;
    }
}
void CompMisalignment()
{
    if(MC_GetOccurredFaultsMotor1() &
    (MC_SPEED_FDBK | MC_DURATION | MC_SW_ERROR | MC_DP_FAULT))
    {
        Alarm.CompMisalignment = SET;
    }
    else
    {
        Alarm.CompMisalignment = RESET;
    }
}
void IpmOverTemp()
{

	if(Alarm.IpmNTCFault)
		return;

	if(IPM_temp > HIGHTEMP_H && Alarm.IpmOverTemp == RESET)
    {
        if(SysCtrl.count_protect[eIpmOverTemp].last1 > IpmOverTemp_Last)
        {
            Alarm.IpmOverTemp = SET;
        }
        else
        {
            SysCtrl.count_protect[eIpmOverTemp].last1 ++;
        }
    }
    else
    {
        SysCtrl.count_protect[eIpmOverTemp].last1 = 0;
    }

    if(IPM_temp < HIGHTEMP_L && Alarm.IpmOverTemp == SET)
    {
        if(SysCtrl.count_protect[eIpmOverTemp].resume1 > IpmOverTemp_Resume)
        {
            Alarm.IpmOverTemp = RESET;
        }
        else
        {
            SysCtrl.count_protect[eIpmOverTemp].resume1 ++;
        }
    }
    else
    {
        SysCtrl.count_protect[eIpmOverTemp].resume1 = 0;
    }
}
void IpmNTCFault()
{
    if(IPM_temp == IPM_NTCFAULT_L && Alarm.IpmNTCFault == RESET)
    {
        if(SysCtrl.count_protect[eIpmNTCFault].last1 > IpmNTCFault_Last)
        {
            Alarm.IpmNTCFault = SET;
        }
        else
        {
            SysCtrl.count_protect[eIpmNTCFault].last1 ++;
        }
    }
    else
    {
        SysCtrl.count_protect[eIpmNTCFault].last1 = 0;
    }

    if(IPM_temp < IPM_NTCFAULT_H 
    && IPM_NTCFAULT_L < IPM_temp
    && Alarm.IpmNTCFault == SET)
    {
        if(SysCtrl.count_protect[eIpmNTCFault].resume1 > IpmNTCFault_Resume)
        {
            Alarm.IpmNTCFault = RESET;
        }
        else
        {
            SysCtrl.count_protect[eIpmNTCFault].resume1 ++;
        }
    }
    else
    {
        SysCtrl.count_protect[eIpmNTCFault].resume1 = 0;
    }
}
void DCunderVoltage()
{
    BusVoltageSensor_Handle_t* BusVoltage= &BusVoltageSensor_M1._Super;
    
    if(VBS_GetAvBusVoltage_V(BusVoltage) < UnderVoltage)
    {
        Alarm.DCunderVoltage = SET;
    }
    else
    {
        Alarm.DCunderVoltage = RESET;
    }   
}
void DCoverVoltage(void)
{
    BusVoltageSensor_Handle_t* BusVoltage= &BusVoltageSensor_M1._Super;
    
    if(VBS_GetAvBusVoltage_V(BusVoltage) > OverVoltage)
    {
        Alarm.DCoverVoltage = SET;
    }
    else
    {
        Alarm.DCoverVoltage = RESET;
    }   

}
void LostCommunication(void)
{
    SysCtrl.count_protect[eLostCommunication].last1 ++;

    if(Alarm.LostCommunication == RESET)
    {
        if(SysCtrl.count_protect[eLostCommunication].last1 > UART_TIMEOUT)
        {
            Alarm.LostCommunication = SET;
        }
    }

    if(Alarm.LostCommunication == SET)
    {
        if(SysCtrl.count_protect[eLostCommunication].resume1 > UART_RESUME)
        {
            if(SysCtrl.count_protect[eLostCommunication].last1 <= UART_TIMEOUT)
            {
                Alarm.LostCommunication = RESET;
            }
            else
            {
                SysCtrl.count_protect[eLostCommunication].resume1 = RESET;
            }
        }
        else
        {
            SysCtrl.count_protect[eLostCommunication].resume1 ++;
        }
    }
}
/*================================================================================================*=
 * GLOBAL FUNCTIONS
 *================================================================================================*/
void System_Alarm_Handler()
{
    CompressOverCurrent();
    IpmOverCurrent();
    ADCoffsetAbnormal();
    // OutputLosePhase();
    CompStarupFail();
    CompMisalignment();
    IpmOverTemp();
    IpmNTCFault();
    DCunderVoltage(); // place in higher frequency task
    DCoverVoltage();// place in higher frequency task
    LostCommunication();

    if(Alarm.All)
    {
        HAL_GPIO_WritePin(DEBUG_LED_RED_GPIO_Port, DEBUG_LED_RED_Pin, GPIO_PIN_SET);
    }
}




/*================================================================================================*=
 * END OF FILE
 *================================================================================================*/
