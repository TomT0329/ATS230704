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
        eCompressOverCurrent = 0 ,
		eReserved1,
        eReserved2,
        eIpmOverCurrent,
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
            unsigned int CompressOverCurrent : 1;
            unsigned int Reserved1 : 1;
            unsigned int Reserved2 : 1;
            unsigned int IpmOverCurrent : 1;
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
        
    };
    
}ALARM_INFO;

/*================================================================================================*=
 * GLOBAL MACROS
 *================================================================================================*/


//CompStarupFail

// ADCoffsetAbnormal
#define ADCvalue_H 2048// to be define
#define ADCvalue_L 1975// to be define
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

#define LostCommunication_Resume 10//sec



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
void CompressOverCurrent();
void IpmOverCurrent();
void ADCoffsetAbnormal();
void OutputLosePhase();
void CompStarupFail();
void CompMisalignment();
void IpmOverTemp();
void IpmNTCFault();
void DCunderVoltage();
void DCoverVoltage();
void LostCommunication();
/*================================================================================================*=
 * END OF FILE
 *================================================================================================*/
#endif /* INC_APP_SYSTEM_PROTECT_H_ */
