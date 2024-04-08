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

/*================================================================================================*=
 * GLOBAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
 *================================================================================================*/
enum{
        eCompressOverCurrent = 0 ,
		eReserved1,
        eReserved2,
        eIpmOverCurrent,
		eADoffsetAbnormal ,
		eOutputLosePhase ,
		eComStarupFail ,
		eCompMisalignment ,//not use
		eReserved3 ,
		eReserved4 ,
		eIpmOverTemp ,
		eIpmSensorFault ,
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
            unsigned int ADoffsetAbnormal  : 1;
            unsigned int OutputLosePhase  : 1;
            unsigned int CompStarupFail  : 1;
            unsigned int CompMisalignment : 1; //not use
            unsigned int Reserved3  : 1;
            unsigned int Reserved4  : 1;
            unsigned int IpmOverTemp  : 1;
            unsigned int IpmSensorFault  : 1;
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
#define UART_TIMEOUT 10 //sec
#define UART_RESUME 10 //sec

#define LostCommunication_Resume 10//sec

#define OverVoltage 450 //V

#define UnderVoltage 300 //V


/*================================================================================================*=
 * GLOBAL CONSTANTS
 *================================================================================================*/
extern SYSCTRL SysCtrl;

/*================================================================================================*=
 * GLOBAL VARIABLES
 *================================================================================================*/

/*================================================================================================*=
 * GLOBAL FUNCTIONS
 *================================================================================================*/
void System_Alarm_Handler(void);
/*================================================================================================*=
 * END OF FILE
 *================================================================================================*/
#endif /* INC_APP_SYSTEM_PROTECT_H_ */
