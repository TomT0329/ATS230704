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
/*================================================================================================*=
 * INCLUDE FILES
 *================================================================================================*/
#include "app_temperature.h"
#include <stdio.h>
#include <math.h>
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
float PFC_current_total = 0.0;
float PFC_voltage_total = 0.0;
float PFC_voltage[AC_PERIOD] = {0.0};
float PFC_current[AC_PERIOD] = {0.0};
/*================================================================================================*=
 * LOCAL FUNCTIONS PROTOTYPE
 *================================================================================================*/
float IpmTemp_Table( uint32_t  DigitalValue );

/*================================================================================================*=
 * LOCAL FUNCTIONS
 *================================================================================================*/
/*  ADC轉換值與溫度對照表 之演算法  - IPM 用  */
float IpmTemp_Table( uint32_t  DigitalValue )
{
    // 區域變數 宣告
    /*  NTC 計算 變數宣告 (可改為區域變數)  */
    float Tcalc = 0.0f; // 計算出(回傳)之值
    float Tmin = 0.0f; // 最小值 = Item - 40
    float DecimalPoint = 0.0f; // 小數點
    uint16_t Item = 0; // 陣列的 index
    uint16_t Dmin_uint16 = 0; // ADC轉換後之最小值
    float Dmin_float = 0.0f; // ADC轉換後之最小值(float)
    uint16_t Dmax_uint16 = 0; // ADC轉換後之最大值
    float Dmax_float = 0.0f; // ADC轉換後之最大值(float)
    uint16_t Dref = 0; // 參考值
    float DigitalValue_float = 0; // ADC轉換出來的值(float)

    //  限制範圍, 代表 -40 ~ 150 度
    if( 305 <= DigitalValue && DigitalValue <= 4000 )
    {
        // Item: 0~190  共191筆
        for( Item = 0 ; Item < 191 ; Item ++ ) // 尋找對應的Item
        {
              Dref =  IpmTempTable[Item]; // 取出

              if( Dref >= DigitalValue )  // 尋找Tmax
              {
                  break; // 找到,跳出迴圈
              }
        }

        Item = Item - 1; // 因 Dref >= DigitalValue, 故減1後是 Tmin

        Tmin = ((float)Item * 1.0f)  - 40.0f ; //index to temperature

        Dmin_uint16 = IpmTempTable[Item]; // 取出digital value最小值

        Dmax_uint16 = IpmTempTable[Item+1]; // 取出digital value最大值

        Dmin_float = (float)Dmin_uint16; // Dmin_uint16 轉成浮點數

        Dmax_float = (float)Dmax_uint16; // Dmax_uint16 轉成浮點數

        DigitalValue_float = (float)DigitalValue ;   //  DigitalValue 轉成浮點數

        // 內差法計算小數點 ( 刻度為1度, 故要乘上1 )
        DecimalPoint = ( ( DigitalValue_float - Dmin_float )  / ( Dmax_float - Dmin_float ) ) * 1.0f ;

        if(DecimalPoint > 1.0f){
            DecimalPoint = 1.0f;
        }
        if(DecimalPoint < 0.0f){
            DecimalPoint = 0.0f;
        }

        Tcalc =  Tmin +  DecimalPoint ; // 整數 加 小數

        return Tcalc;
    }
    else
    {
    	return -40.0;
    }
}

/*================================================================================================*=
 * GOBAL FUNCTIONS PROTOTYPE
 *================================================================================================*/

void ADC2_DMA_Init(uint32_t *AdcValue)
{
	if(HAL_ADC_Start_DMA(&hadc2,(uint32_t *)AdcValue,3) != HAL_OK)
	{
	  Error_Handler();
	}
}

void Temp_Average(uint32_t  DigitalValue, float* IpmTemp)
{
    static uint8_t index01 = 0;
    static float _Tipm[AVG_NUM] = {0};
    float TipmTotal = 0;

    // ADC_ConvertedValueLocal_1 = (float)((((float)DigitalValue) / 4096.0f )* 3.3f ); // 輸入給MCU的電壓

    // Ripm = ((10.0f*3.3f) / ADC_ConvertedValueLocal_1) - 10.0f ;

    _Tipm[index01]  =  IpmTemp_Table( DigitalValue );

    // 加總 - 將每一筆資料加起來
    for( int ind = 0 ; ind < AVG_NUM ; ind++ )
    {
        TipmTotal += _Tipm[ind]; // TcompTotal = TcompTotal + _Tcomp[ind];
    }

    *IpmTemp = TipmTotal / AVG_NUM  ; //平均(128筆)

    // 將加總值歸零(重要,必加)
    TipmTotal = 0.0f;

    if(  index01 >= AVG_NUM )
    {
        index01 = 0;
    }
    else
    {
        index01++;
    }
}

float PFC_GetCurrent(uint32_t  DigitalValue)
{
    return (float)(((((float)(DigitalValue + PFC_Curr_Cali)/ADC12BIT) * 3.3) - 0.7) * MCUref2CURR);
}
float PFC_GetVoltage(uint32_t  DigitalValue)
{
    return (float)(((((float)DigitalValue - ADC12BITREF)/ADC12BIT) * 3.3 ) * MCUref2VOLT);
}
void PFC_GetRMS()
{
    static uint32_t j = 0;
    static uint32_t k = 0;

    PFC_current[j++] = PFC_GetCurrent(temp_adc[1]);
    PFC_voltage[k++] = PFC_GetVoltage(temp_adc[2]);
    PFC_current_total = PFC_current_total + powf(PFC_current[j],2.0);
    PFC_voltage_total = PFC_voltage_total + powf(PFC_voltage[k],2.0);

    if(j == AC_PERIOD -1)
    {
      PFC_current_rms = sqrtf(PFC_current_total / AC_PERIOD);
      PFC_voltage_rms = sqrtf(PFC_voltage_total / AC_PERIOD);
      PFC_power = (PFC_current_rms) * (PFC_voltage_rms);

      PFC_current_total = 0;
      PFC_voltage_total = 0;
      j = 0;
      k = 0;
    }
}


