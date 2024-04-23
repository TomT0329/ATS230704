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

void Comp_Software_OCP()
{
    // int16_t Phase_Peak_S16A = MCI_GetPhaseCurrentAmplitude(&Mci[M1]);
    // float Phase_Peak = (Phase_Peak_S16A * 3.3) / (65536 * RSHUNT * AMPLIFICATION_GAIN);
    qd_f_t Iqd_ref = MC_GetIqdrefMotor1_F();

    if(Iqd_ref.q > Phase_Curr_SpeedReduce_H 
    && Alarm.Fault1.Comp_Software_OCP == RESET)
    {
        if(SysCtrl.count_protect[eComp_Software_OCP].last1 > Comp_Software_OCP_Last)
        {
            Alarm.Fault1.Comp_Software_OCP = SET;
            MC_StopSpeedRampMotor1();
        }
        else
        {
            SysCtrl.count_protect[eComp_Software_OCP].last1 ++;
        }
    }
    else
    {
        SysCtrl.count_protect[eComp_Software_OCP].last1 = 0;
    }


    if(Iqd_ref.q < Phase_Curr_SpeedReduce_L 
    && MC_GetMecSpeedAverageMotor1() < Middle_OP_SPEED
    && Alarm.Fault1.Comp_Software_OCP == SET)
    {
        if(SysCtrl.count_protect[eComp_Software_OCP].resume1 > Comp_Software_OCP_Resume)
        {
            Alarm.Fault1.Comp_Software_OCP = RESET;
        }
        else
        {
            SysCtrl.count_protect[eComp_Software_OCP].resume1 ++;
        }
    }
    else
    {
        SysCtrl.count_protect[eComp_Software_OCP].resume1 = 0;
    }
}

void PowerLimitProtect()
{
    if(Alarm.Fault1.Comp_Software_OCP == SET)
    {
        Alarm.Fault1.PowerLimitProtect = RESET;
        SysCtrl.count_protect[ePowerLimitProtect] = (COUNT){0};
        return;
    }

    if(PFC_power > Power_Limit_H 
    && Alarm.Fault1.PowerLimitProtect == RESET)
    {
        if(SysCtrl.count_protect[ePowerLimitProtect].last1 > PowerLimitProtect_Last)
        {
            Alarm.Fault1.PowerLimitProtect = SET;

            //Stop Speed command
            MC_StopSpeedRampMotor1();
        }
        else
        {
            SysCtrl.count_protect[ePowerLimitProtect].last1 ++;
        }
    }
    else
    {
        SysCtrl.count_protect[ePowerLimitProtect].last1 = RESET;
    }

    //if motor speed > 820 or speed command < 820 then exit
    if((MC_GetMecSpeedAverageMotor1() > Power_Limit_Speed
    || stModb.wordReg1.wds[DRIVER_FREQ] < 820)
    && (Alarm.Fault1.PowerLimitProtect == SET))
    {
        if(SysCtrl.count_protect[ePowerLimitProtect].resume1 > PowerLimitProtect_Resume)
        {
            Alarm.Fault1.PowerLimitProtect = RESET;

            //Stop Speed command which over Speed_Limit 
            //and will execute speeed command from control board instead
            MC_StopSpeedRampMotor1();
        }
        else
        {
            SysCtrl.count_protect[ePowerLimitProtect].resume1 ++;
        }
    }
    else
    {
        SysCtrl.count_protect[ePowerLimitProtect].resume1 = RESET;
    }
}

void PowerLimitControl()
{

    //Hardware protect
    if(Alarm.Fault1.IPM_Hardware_OCP
    || Alarm.Fault1.ADCoffsetAbnormal
    || Alarm.Fault1.OutputLosePhase
    || Alarm.Fault1.CompStarupFail
    || Alarm.Fault1.CompMisalignment
    || Alarm.Fault1.IpmOverTemp
    || Alarm.Fault1.IpmNTCFault
    || Alarm.Fault1.DCoverVoltage
    || Alarm.Fault1.DCunderVoltage
    || Alarm.Fault1.LostCommunication)  
    {
        MC_StopSpeedRampMotor1();
        MC_StopMotor1();
        //execute init speed ramp to 3000rpm for the next MC_StartMotor1
        MCI_ExecSpeedRamp_F(&Mci[M1],Init_Ramp_Speed,Init_Ramp_Time);
        return;
    }

    if(Alarm.Fault1.Comp_Software_OCP
    && MC_HasRampCompletedMotor1())
    {
        //force speed ramp 3000rpm to protect ipm
        MCI_ExecSpeedRamp_F(&Mci[M1],Init_Ramp_Speed,Init_Ramp_Time);
        return;
    }

    int16_t freq_command = MC_GetMecSpeedAverageMotor1();//rps(01hz)

    if(Alarm.Fault1.PowerLimitProtect
    && MC_HasRampCompletedMotor1())
    {
        if(PFC_power >= Power_Limit_H)
        {
            freq_command-=SPEED_err;
            CLAMP(freq_command,MIN_SPEED_01HZ,Power_Control_Speed);
            MCI_ExecSpeedRamp(&Mci[M1],(int16_t)freq_command,Power_ControlACC_Time);
        }
        else 
        {
            freq_command+=SPEED_err;
            CLAMP(freq_command,MIN_SPEED_01HZ,Power_Control_Speed);
            MCI_ExecSpeedRamp(&Mci[M1],(int16_t)freq_command,Power_ControlACC_Time);
        }
    }

}

void IPM_Hardware_OCP()
{
    if(MC_GetOccurredFaultsMotor1() == MC_OVER_CURR)
    {
        Alarm.Fault1.IPM_Hardware_OCP = SET;
        MC_StopSpeedRampMotor1();
    }
    else
    {
        Alarm.Fault1.IPM_Hardware_OCP = RESET;
    }
}
void ADCoffsetAbnormal()
{
    if(MCI_GetSTMState(&Mci[M1]) == IDLE)
    {
        if((OneShunt_ADC > ADCvalue_H
		|| OneShunt_ADC < ADCvalue_L)
        && (Alarm.Fault1.ADCoffsetAbnormal == RESET))
        {
            if(SysCtrl.count_protect[eADCoffsetAbnormal].last1 > ADCoffsetAbnormal_Last)
            {
                Alarm.Fault1.ADCoffsetAbnormal = SET;
                MC_StopSpeedRampMotor1();
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
        && Alarm.Fault1.OutputLosePhase == RESET)
        {
            if(SysCtrl.count_protect[eOutputLosePhase].last1 > OutputLosePhase_Last)
            {
                Alarm.Fault1.OutputLosePhase = SET;
                Mci[M1].PastFaults = MC_START_UP;
                MC_StopSpeedRampMotor1();
                MC_StopMotor1();
                //execute init speed ramp to 3000rpm to protect compressor
                MCI_ExecSpeedRamp_F(&Mci[M1],Init_Ramp_Speed,Init_Ramp_Time);
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
        Alarm.Fault1.CompStarupFail = SET;
        MC_StopSpeedRampMotor1();
    }
    else
    {
        Alarm.Fault1.CompStarupFail = RESET;
    }
}
void CompMisalignment()
{
    if(MC_GetOccurredFaultsMotor1() &
    (MC_SPEED_FDBK | MC_DURATION | MC_SW_ERROR | MC_DP_FAULT))
    {
        Alarm.Fault1.CompMisalignment = SET;
        MC_StopSpeedRampMotor1();
    }
    else
    {
        Alarm.Fault1.CompMisalignment = RESET;
    }
}
void IpmOverTemp()
{

	if(Alarm.Fault1.IpmNTCFault == SET)
    {
        Alarm.Fault1.IpmOverTemp = RESET;
        SysCtrl.count_protect[eIpmOverTemp] = (COUNT){0};
        return;
    }

	if(IPM_temp > HIGHTEMP_H && Alarm.Fault1.IpmOverTemp == RESET)
    {
        if(SysCtrl.count_protect[eIpmOverTemp].last1 > IpmOverTemp_Last)
        {
            Alarm.Fault1.IpmOverTemp = SET;
            MC_StopSpeedRampMotor1();
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

    if(IPM_temp < HIGHTEMP_L && Alarm.Fault1.IpmOverTemp == SET)
    {
        if(SysCtrl.count_protect[eIpmOverTemp].resume1 > IpmOverTemp_Resume)
        {
            Alarm.Fault1.IpmOverTemp = RESET;
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
    if(IPM_temp == IPM_NTCFAULT_L && Alarm.Fault1.IpmNTCFault == RESET)
    {
        if(SysCtrl.count_protect[eIpmNTCFault].last1 > IpmNTCFault_Last)
        {
            Alarm.Fault1.IpmNTCFault = SET;
            MC_StopSpeedRampMotor1();
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
    && Alarm.Fault1.IpmNTCFault == SET)
    {
        if(SysCtrl.count_protect[eIpmNTCFault].resume1 > IpmNTCFault_Resume)
        {
            Alarm.Fault1.IpmNTCFault = RESET;
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
        Alarm.Fault1.DCunderVoltage = SET;
        MC_StopSpeedRampMotor1();
    }
    else
    {
        Alarm.Fault1.DCunderVoltage = RESET;
    }   
}
void DCoverVoltage(void)
{
    BusVoltageSensor_Handle_t* BusVoltage= &BusVoltageSensor_M1._Super;
    
    if(VBS_GetAvBusVoltage_V(BusVoltage) > OverVoltage)
    {
        Alarm.Fault1.DCoverVoltage = SET;
        MC_StopSpeedRampMotor1();
    }
    else
    {
        Alarm.Fault1.DCoverVoltage = RESET;
    }   

}
void LostCommunication(void)
{
    SysCtrl.count_protect[eLostCommunication].last1 ++;

    if(Alarm.Fault1.LostCommunication == RESET)
    {
        if(SysCtrl.count_protect[eLostCommunication].last1 > UART_TIMEOUT)
        {
            Alarm.Fault1.LostCommunication = SET;
            MC_StopSpeedRampMotor1();
        }
    }

    if(Alarm.Fault1.LostCommunication == SET)
    {
        if(SysCtrl.count_protect[eLostCommunication].resume1 > UART_RESUME)
        {
            if(SysCtrl.count_protect[eLostCommunication].last1 <= UART_TIMEOUT)
            {
                Alarm.Fault1.LostCommunication = RESET;
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
    Comp_Software_OCP();
    IPM_Hardware_OCP();
    // ADCoffsetAbnormal();
    OutputLosePhase();
    CompStarupFail();
    CompMisalignment();
    IpmOverTemp();
    IpmNTCFault();
    // DCunderVoltage();
    // DCoverVoltage();
    LostCommunication();

    PowerLimitProtect();
    PowerLimitControl();
    
    if(Alarm.Fault1.All)
    {
        HAL_GPIO_WritePin(DEBUG_LED_RED_GPIO_Port, DEBUG_LED_RED_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(DEBUG_LED_RED_GPIO_Port, DEBUG_LED_RED_Pin, GPIO_PIN_RESET);
    }
}




/*================================================================================================*=
 * END OF FILE
 *================================================================================================*/
