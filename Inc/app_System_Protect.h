/***********************************************************************************************//**
 * @file        app_System_Protect.h
 * @brief       app_System_Protect
 * @date        April. 02, 2024
 * @version     1.0.0
 * @author      TOM.TANG
 *
 * Revision History
 *------------------------------------------------------------------------------------------------
 *|Version   |Date       |Author         |Change ID      |Description                            |
 *|----------|-----------|---------------|---------------|---------------------------------------|
 *|1.0.0     |2024.04.02  |TOM.TANG       |00000000000    |Initial version created                |
 **************************************************************************************************/
#ifndef INC_APP_SYSTEM_PROTECT_H_
#define INC_APP_SYSTEM_PROTECT_H_
/*================================================================================================*=
 * INCLUDE FILES
 *================================================================================================*/
#include "drive_parameters.h"
/*================================================================================================*=
 * GLOBAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
 *================================================================================================*/
enum{
        eComp_Software_OCP = 0 ,
		ePowerLimitProtect,
        eReserved2,
        eIPM_Hardware_OCP,
		eADCoffsetAbnormal ,
		eOutputLosePhase ,
		eComStarupFail ,
		eCompMisalignment ,//not use
		eReserved3 ,
		eReserved4 ,
		eIpmOverTemp ,
		eIpmNTCFault ,
		eDCunderVoltage ,
		eDCoverVoltage,
		eChargeCircuitFault,//not use
        eLostCommunication
};


typedef struct{
	uint16_t last1;
	uint16_t last2;
	uint16_t timer;
	uint16_t resume1;
	uint16_t resume2;
}COUNT;

typedef struct{
	COUNT	count_protect[16];
}SYSCTRL;


typedef struct ALARM_INFO_t
{
    /* data */
    union 
    {
        uint16_t All;
        struct 
        {
            unsigned int HighOutputCurrent : 1;
            unsigned int HighIPMTemp : 1;
            unsigned int reserved1 : 1;
            unsigned int reserved2 : 1;
            unsigned int reserved3  : 1;
            unsigned int reserved4  : 1;
            unsigned int reserved5  : 1;
            unsigned int reserved6 : 1; //not use
            unsigned int reserved7  : 1;
            unsigned int reserved8  : 1;
            unsigned int HighACcurrent  : 1;
            unsigned int HighPFCcurrent  : 1;
            unsigned int LowACvoltage  : 1;
            unsigned int LowDCvoltage : 1;
            unsigned int HighACpower : 1;
            unsigned int LostCommunication : 1;
        };
        
    }SL_SR;

    union 
    {
        uint16_t All;
        struct 
        {
            unsigned int Comp_Software_OCP : 1;
            unsigned int PowerLimitProtect : 1;
            unsigned int Reserved2 : 1;
            unsigned int IPM_Hardware_OCP : 1;
            unsigned int ADCoffsetAbnormal  : 1;
            unsigned int OutputLosePhase  : 1;
            unsigned int CompStarupFail  : 1;
            unsigned int CompMisalignment : 1; //not use
            unsigned int Reserved3  : 1;
            unsigned int Reserved4  : 1;
            unsigned int IpmOverTemp  : 1;
            unsigned int IpmNTCFault  : 1;
            unsigned int DCunderVoltage  : 1;
            unsigned int DCoverVoltage : 1;
            unsigned int ChargeCircuitFault : 1;//not use
            unsigned int LostCommunication : 1;
        };
        
    }Fault1;
    union 
    {
        uint16_t All;
        struct 
        {//TBD
            unsigned int ACoverCurrent : 1;
            unsigned int reserved : 1;
            unsigned int PFCHardwareOCP: 1;
            unsigned int DCHardwareOVP : 1;
            unsigned int ACHardwarelostPhase  : 1;
            unsigned int ACSoftwarelostPhase  : 1;
            unsigned int reserved1  : 1;
            unsigned int reserved2: 1; //not use
            unsigned int HPSprotect : 1;
            unsigned int DLTprotect : 1;
            unsigned int PFCoverTemp : 1;
            unsigned int PFCsensorFault : 1;
            unsigned int AC_UVP : 1;
            unsigned int AC_OVP : 1;
            unsigned int EEPROM_fault : 1;//not use
            unsigned int MCU_fault : 1;
        };
        
    }Fault2;
}ALARM_INFO;

/*================================================================================================*=
 * GLOBAL MACROS
 *================================================================================================*/

//PowerLimitProtect
#define Power_Limit_H 4500 // watt
#define Power_Limit_Speed MAX_SPEED_01HZ
#define PowerLimitProtect_Last 2 // sec
#define PowerLimitProtect_Resume 1 // sec
//PowerLimitControl
#define Power_Control_Speed (MAX_SPEED_01HZ + 15)
#define SPEED_err       4 // rps(01hz) = 24rpm
#define Power_ControlACC_Time        400//ms

//Comp_Software_OCP
#define Phase_Curr_SpeedReduce_H 21// Ampere
#define Phase_Curr_SpeedReduce_L 12// Ampere
#define Middle_OP_SPEED 520// rps 01HZ
#define Comp_Software_OCP_Last 1 //sec
#define Comp_Software_OCP_Resume 30 //sec

//CompStarupFail

// ADCoffsetAbnormal
#define ADCvalue_H 2048// to be define
#define ADCvalue_L 1948// to be define
#define ADCoffsetAbnormal_Last 1 //sec

//OutputLosePhase
#define PhaseLocateCurrent_L (PHASE2_FINAL_CURRENT_A - 2)//AMPERE
// #define PhaseLocateCurrent_H 0//AMPERE
#define OutputLosePhase_Last 1000 //sec
#define OutputLosePhase_Timer 3000 //sec

//Ipm over temperature
#define HIGHTEMP_H 80
#define HIGHTEMP_L 50
#define IpmOverTemp_Last 3
#define IpmOverTemp_Resume 3

//IpmNTCFault
#define IPM_NTCFAULT_H 130
#define IPM_NTCFAULT_L -40
#define IpmNTCFault_Last 5
#define IpmNTCFault_Resume 5

//DCunderVoltage
#define UnderVoltage 350 //V

#define OverVoltage 450 //V

#define UART_TIMEOUT 10 //sec
#define UART_RESUME 10 //sec

//LostCommunication
#define LostCommunication_Resume 10//sec
#define LostComm_Speed 500 // rps(01Hz)
#define LostComm_RampTime  60000// msec



/*================================================================================================*=
 * GLOBAL CONSTANTS
 *================================================================================================*/
extern SYSCTRL SysCtrl;
extern ALARM_INFO Alarm;
/*================================================================================================*=
 * GLOBAL VARIABLES
 *================================================================================================*/

/*================================================================================================*=
 * GLOBAL FUNCTIONS
 *================================================================================================*/
void System_Alarm_Handler(void);
//protect fun
void Comp_Software_OCP();
void Comp_Software_OCP();
void PowerLimitProtect();
void IPM_Hardware_OCP();
void ADCoffsetAbnormal();
void OutputLosePhase();
void CompStarupFail();
void CompMisalignment();
void IpmOverTemp();
void IpmNTCFault();
void DCunderVoltage();
void DCoverVoltage();
void LostCommunication();

//Power Limit control
void PowerLimitProtect();
void PowerLimitControl();
/*================================================================================================*=
 * END OF FILE
 *================================================================================================*/
#endif /* INC_APP_SYSTEM_PROTECT_H_ */
