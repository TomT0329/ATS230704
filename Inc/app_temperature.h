/***********************************************************************************************//**
 * @file        app_Modbus_RTU.c
 * @brief       app_Modbus_RTU.c
 * @date        SEP 11, 2023
 * @version     1.0.0
 * @author      TOM.TANG
 *
 * Revision History
 *------------------------------------------------------------------------------------------------
 *|Version   |Date       |Author         |Change ID      |Description                            |
 *|----------|-----------|---------------|---------------|---------------------------------------|
 *|1.0.0     |2023.09.11 |TOM.TANG       |00000000000    |Initial version created                |
 **************************************************************************************************/
#ifndef INC_APP_TEMPERATURE_H_
#define INC_APP_TEMPERATURE_H_
/*================================================================================================*=
 * INCLUDE FILES
 *================================================================================================*/
#include <stdint.h>
/*================================================================================================*=
 * LOCAL CONSTANTS
 *================================================================================================*/
static const uint16_t IpmTempTable[191] = {
305,	323,	342,	362,	383,	404,	427,	450,	475,	500,	
526,	553,	581,	610,	640,	671,	703,	736,	769,	804,	
839,	875,	912,	950,	989,	1028,	1068,	1109,	1150,	1192,	
1235,	1278,	1322,	1366,	1410,	1455,	1500,	1545,	1590,	1636,	
1681,	1727,	1773,	1818,	1864,	1909,	1954,	1999,	2044,	2088,	
2132,	2176,	2219,	2262,	2304,	2346,	2387,	2428,	2468,	2507,	
2546,	2584,	2622,	2659,	2695,	2731,	2766,	2800,	2833,	2866,	
2898,	2929,	2960,	2990,	3019,	3048,	3076,	3103,	3129,	3155,	
3181,	3205,	3229,	3252,	3275,	3297,	3319,	3340,	3360,	3380,	
3399,	3418,	3436,	3454,	3471,	3488,	3504,	3520,	3536,	3551,	
3565,	3579,	3593,	3606,	3619,	3632,	3644,	3656,	3667,	3678,	
3689,	3700,	3710,	3720,	3730,	3739,	3748,	3757,	3766,	3774,	
3782,	3790,	3797,	3805,	3812,	3819,	3826,	3833,	3839,	3845,	
3851,	3857,	3863,	3869,	3874,	3879,	3884,	3889,	3894,	3899,	
3904,	3908,	3912,	3917,	3921,	3925,	3929,	3932,	3936,	3940,	
3943,	3947,	3950,	3953,	3956,	3959,	3962,	3965,	3968,	3971,	
3973,	3976,	3979,	3981,	3983,	3986,	3988,	3990,	3993,	3995,	
3997,	3999,	4001,	4003,	4005,	4006,	4008,	4010,	4012,	4013,	
4015,	4016,	4018,	4020,	4021,	4022,	4024,	4025,	4027,	4028,	
4029
};

/*================================================================================================*=
 * LOCAL VARIABLES
 *================================================================================================*/
/*================================================================================================*=
 * LOCAL MACROS
 *================================================================================================*/
#define AVG_NUM 100

/*================================================================================================*=
 * GLOBAL CONSTANTS
 *================================================================================================*/


/*================================================================================================*=
 * GLOBAL VARIABLES
 *================================================================================================*/
extern uint32_t temp_adc;
/*================================================================================================*=
 * GLOBAL FUNCTIONS PROTOTYPE
 *================================================================================================*/
void Temp_Average(uint32_t  DigitalValue, float* IpmTemp);

#endif /* INC_APP_TEMPERATURE_H_ */