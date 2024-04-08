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
//protect fun
void CompressOverCurrent();
void IpmOverCurrent();
void ADoffsetAbnormal();
void OutputLosePhase();
void CompStarupFail();
void IpmOverTemp();
void IpmNTCFault();
void DCunderVoltage();
void DCoverVoltage();
void LostCommunication();
/*================================================================================================*=
 * LOCAL FUNCTIONS
 *================================================================================================*/

void CompressOverCurrent()
{
    //TBD
}
void IpmOverCurrent()
{

}
void ADoffsetAbnormal()
{

}
void OutputLosePhase()
{

}
void CompStarupFail()
{

}
void IpmOverTemp()
{

}
void IpmNTCFault()
{

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
    ADoffsetAbnormal();
    OutputLosePhase();
    CompStarupFail();
    IpmOverTemp();
    IpmNTCFault();
    DCunderVoltage();
    DCoverVoltage();
    LostCommunication();
}




/*================================================================================================*=
 * END OF FILE
 *================================================================================================*/
