/***********************************************************************************************//**
 * @file        app_Modbus_RTU.h
 * @brief       app_Modbus_RTU.h
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
#ifndef INC_APP_MODBUS_RTU_H_
#define INC_APP_MODBUS_RTU_H_
/*================================================================================================*=
 * INCLUDE FILES
 *================================================================================================*/
#include "stdint.h"
#include "stdio.h"
#include "stm32G4xx_hal.h"
#include "main.h"
/*================================================================================================*=
 * LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
 *================================================================================================*/
// Control register
typedef enum
{
	_ID_SET_WORD_MIN   		 = 0x0000,							// Start

	DRIVER_CTRL   			 = _ID_SET_WORD_MIN,				// RPU MODBUS device ID
	DRIVER_PARA,
	DRIVER_EEPROM,
	DRIVER_FREQ,
	HEATER_CURRENT,
	DRIVER_OVERTEMP,
	DRIVER_RESERVED1,
	DRIVER_RESERVED2,
	DRIVER_SECURITY1,
	DRIVER_SECURITY2,

	DRIVER_STATUS			= 0x10,
	DRIVER_RESERVED3,
	EEPROM_STATUS,
	COMP_FREQ,
	COMP_VOLTAGE,
	COMP_CURRENT,
	COMP_POWER,
	COMP_TORQUE,
	IPM_TEMP,
	IPM_PWMFREQ,
	DCBUS_VOLTAGE,
	AC_VOLTAGE,
	AC_CURRENT,
	AC_POWER,
	VDC_RIPPLE,
	PFC_TEMP,
	PFCPWM_FREQ,
	PCB_TEMP,
	DLT_VOLTAGE,
	IF_VOLTAGE,
	DRIVER_RESERVED4,

	_ID_SET_WORD_MAX   	   = DRIVER_RESERVED4,
} _MODBUS_SLAVE_ADDR;

/* Information */
enum _MODBUS_INPUT_REG
{
	_ID_INPUT_REG_MIN 	= 0x0300,
	TEMP_W1    			= _ID_INPUT_REG_MIN,
	TEMP_W2    			= TEMP_W1 + 1,
	TEMP_AIR     		= TEMP_W2  + 1,
	TEMP_R4  			= TEMP_AIR  + 1,
	TEMP_R3  			= TEMP_R4  + 1,
	TEMP_R2  			= TEMP_R3  + 1,
	TEMP_R1     		= TEMP_R2 + 1,
	TEMP_TAMBIENT  		= TEMP_R1  + 1,
	PRES_W1				= TEMP_TAMBIENT + 1,
	PRES_W2				= PRES_W1 + 1,
	PRES_W3				= PRES_W2 + 1,
	PRES_R1				= PRES_W3 + 1,
	PUMP1_RPM_FB		= PRES_R1 + 1,
	PUMP1_DUTY_FB	    = PUMP1_RPM_FB + 1,
	COMP_2100			= PUMP1_DUTY_FB +1,
	COMP_2101			= COMP_2100 +1,
	COMP_2102			= COMP_2101 +1,
	COMP_2103			= COMP_2102 +1,
	COMP_2110			= COMP_2103 +1,
	COMP_2121			= COMP_2110 +1,
	_ID_INPUT_REG_MID   = COMP_2121,
	// Alarm
	_ID_ALERT_INDEX_BASE= 0x0400,						// Alarm status base
	RPU_FAN_STATUS      = _ID_ALERT_INDEX_BASE,			// RPU internal fan alarm status
	RPU_PUMP_STATUS 	= RPU_FAN_STATUS + 1,			// RPU pump alarm status
	RPU_SENSOR_ABNORMAL = RPU_PUMP_STATUS + 1,			// RPU sensor alarm status
	LEAKAGE_ABNORMAL    = RPU_SENSOR_ABNORMAL + 1,		// Leakage abnormal status
	HX_SENSOR_ABNORMAL  = LEAKAGE_ABNORMAL + 1,			// HX Sensor abnormal status
	HX_FAN_ABNORMAL  	= HX_SENSOR_ABNORMAL + 1,		// HX Fan abnormal status
	HX_FAN_2_ABNORMAL  	= HX_FAN_ABNORMAL + 1,			// HX Fan 2 abnormal status

	// Firmware verion
	_ID_FW_INDEX_BASE  = 0x0500,						// Firmware version base
	FW_VERSION_1       = _ID_FW_INDEX_BASE,				// Firmware version 1
	FW_VERSION_2       = FW_VERSION_1 + 1,				// Firmware version 2
	FW_VERSION_3       = FW_VERSION_2 + 1,				// Firmware version 3
	FW_VERSION_4       = FW_VERSION_3 + 1,				// Firmware version 4
	FW_VERSION_5       = FW_VERSION_4 + 1,				// Firmware version 5
	_ID_INPUT_REG_MAX  = FW_VERSION_5,
};

enum _MODBUS_BIT
{
	_ID_SET_BIT_MIN   		= 0x800,
	/* bit0 */
	_ID_WATER_HEATER_SWITCH = _ID_SET_BIT_MIN,
	_ID_PUMP_HEATER_SWITCH  = 0x801,
	_ID_PUMP_ALARM_STOP     = 0x802,
	_ID_PUMP_ALARM_OV_UV  	= 0x803,
	_ID_PUMP_ALARM_OC    	= 0x804,
	_ID_PUMP_ALARM_OT  		= 0x805,
	_ID_PUMP_ALARM_AL  		= 0x806,
	_ID_PUMP_ALARM_NL 		= 0x807,
	/* bit1 */
	_ID_PUMP2_STA 		    = 0x808,
	_ID_WATER_HEATER_STA    = 0x809,
	_ID_FAN1_1_STA          = 0x80A,
	_ID_FAN1_2_STA          = 0x80B,
	_ID_FAN1_3_STA          = 0x80C,
	_ID_FAN2_1_STA          = 0x80D,
	_ID_FAN2_2_STA          = 0x80E,
	_ID_RESERVE             = 0x80F,
	/* bit2 0x810 - 0x817 */
	/* bit3 0x818 - 0x81F */
	/* bit4 0x820 - 0x827 */
	/* bit5 0x828 - 0x82F */
	/* bit6 0x830 - 0x837 */
	/* bit7 0x838 - 0x83F */
	/* bit8 0x840 - 0x847 */
	/* bit9 */
	_ID_COMP_2100_DRIVER_STATUS  = 0x848,
	_ID_COMP_2100_PFC_STA     	 = 0x849,
	_ID_COMP_2100_CHG_FIN_STA 	 = 0x84A,
	_ID_COMP_2100_STOP_PROT    	 = 0x84B,
	_ID_COMP_2100_ALARM_PROT   	 = 0x84C,
    /**/

	_ID_SET_BIT_MAX_1 = _ID_COMP_2100_ALARM_PROT,
	_ID_SET_BIT_MAX = _ID_SET_BIT_MAX_1,
};
typedef enum{
	FIRST_DATA = 0,
	SECOND_DATA  = 1,
	THIRD_DATA  = 2,
}UART_RX_ST;

typedef struct
{
	uint8_t rbuf[8];
	uint8_t buf[260];
	int8_t ok;
	int8_t reserved[2];
	int16_t size;
	int16_t cnt;
} UART_BUFF_STR;

typedef struct
{
	//int8_t slaveID;
	int8_t timeOut;
	int8_t errCode;  //1:funErr, 2:AddrErr, 3:DataErr, 4:DevicErr, 5:command busy, 6:system busy
	//int8_t reserved;
	UART_BUFF_STR tx;
	UART_BUFF_STR rx;
} UART_STR;

typedef union
{
    int16_t wds[DRIVER_SECURITY2 - _ID_SET_WORD_MIN + 1];
    struct
    {
    	uint16_t Ctrl;      	 // 0x00 Auto tune Coolant Flow Rate Target Set
    	uint16_t Para;   		 // 0x01 Auto tune Coolant Pressure Target Set
    	uint16_t EEPROM;         // 0x02 Manual Control Pump duty set
    	int16_t Freq; 	     	 // 0x03 Manual Control Pump on/off
    	uint16_t SH_current;     // 0x04 Manual Control Valve duty set
    	int16_t OAT;             // 0x05 Manual Control Valve on/off
		uint16_t Reserved1;		 // 0X06
		uint16_t Reserved2;		 // 0X07
		uint16_t Security1;		 // 0X08
		uint16_t Security2;		 // 0X09
    } Driver;
} _MEM_REG_1;




typedef union
{
	uint16_t all[1];
	struct
	{
        uint16_t outlet_T_IL_Set: 1;// 1
        uint16_t outlet_T_IH_Set: 1;// 2
        uint16_t outlet_T_PL_Set: 1;// 3
        uint16_t outlet_T_PH_Set: 1;// 4
        uint16_t outlet_P_IL_Set: 1;// 5
        uint16_t outlet_P_IH_Set: 1;// 6
        uint16_t reserve: 		  10;// 5~16
	} flag;
} controlFlag;

typedef struct
{
	union
	{
		int8_t bits[15];
		struct
		{
			/* bits[0] */
			uint8_t WaterHeater_SW:    1; //0x00
			uint8_t PumpHeater_SW :    1; //0x01
			uint8_t Pump_Alarm_Stop:   1; //0x02
			uint8_t Pump_Alarm_OV_UV:  1; //0x03
			uint8_t Pump_Alarm_OC:     1; //0x04
			uint8_t Pump_Alarm_OT:     1; //0x05
			uint8_t Pump_Alarm_AL:     1; //0x06
			uint8_t Pump_Alarm_NL:     1; //0x07
			/* bits[1] */
			uint8_t Pump2_STA:         1; //0x08
			uint8_t WaterHeater_STA:   1; //0x09
			uint8_t Fan1_1_STA:   	   1; //0x0A
			uint8_t Fan1_2_STA:   	   1; //0x0B
			uint8_t Fan1_3_STA:   	   1; //0x0C
			uint8_t Fan2_1_STA:   	   1; //0x0D
			uint8_t Fan2_2_STA:   	   1; //0x0E
			uint8_t reserve:   	       1; //0x0F
			/* bits[2] */
			uint8_t reserve1:   	   8;
			/* bits[3] */
			uint8_t reserve2:   	   8;
			/* bits[4] */
			uint8_t reserve3:   	   8;
			/* bits[5] */
			uint8_t reserve4:   	   8;
			/* bits[6] */
			uint8_t reserve5:   	   8;
			/* bits[7] */
			uint8_t reserve6:   	   8;
			/* bits[8] */
			uint8_t reserve7:   	   8;
			/* bits[9] : Comp.2100*/
			uint8_t COMP_2100_Driver_STA:   1;
			uint8_t COMP_2100_PFC_STA:      1;
			uint8_t COMP_2100_CHG_FIN_Flag: 1;
			uint8_t COMP_2100_Stop_PROT:	1;
			uint8_t COMP_2100_Alarm_PROT:   1;
			uint8_t COMP_2100_Reserve:      3;
			/* bits[10] : Comp.2101*/
		} bit;
	};
} MEM_COILS;

typedef union
{
    int16_t wds[260];
    struct
    {
    	int16_t  Temp;   		// 0x0000
    } inputReg;
} MEM_INPUT_REG_1;

// Alarm register
typedef union
{
    uint16_t wds[2];
    struct
    {
        uint16_t filesize_high;  	
        uint16_t filesize_low;
    } inputReg;
} MEM_INPUT_REG_2;

// Firmware version register
typedef union
{
    int16_t wds[FW_VERSION_5 - _ID_FW_INDEX_BASE + 1];
    struct
    {
        uint16_t fwVer_1;  	// 0x0200, Firmware version 1
        uint16_t fwVer_2; 	// 0x0101, Firmware version 2
        uint16_t fwVer_3; 	// 0x0102, Firmware version 3
        uint16_t fwVer_4;   // 0x0103, Firmware version 4
        uint16_t fwVer_5;   // 0x0104, Firmware version 5
    } inputReg;
} MEM_INPUT_REG_3;

typedef struct _MODBUS_STR
{
	controlFlag		ctrlFlag;
	MEM_COILS       bitCoil;
	_MEM_REG_1 		wordReg1;
	MEM_INPUT_REG_1 inputReg1;
	MEM_INPUT_REG_2 inputReg2;
	MEM_INPUT_REG_3 inputReg3;
} MODBUS_STR;

typedef union
{
    int16_t w;
    struct
    {
        int8_t lo;
        int8_t hi;
    } b;
}TRANS_TYPE;

/*================================================================================================*=
 * GLOBAL MACROS
 *================================================================================================*/
#define	OFF 0x00
#define ON  0xff
#define GET_BIT(x, bit) ((x & (1<<bit)) >> bit)
#define RX_DATA_SIZE    200

#define DRIVER_SLAVE_ID		0x01
#define USART_TIMEOUT_CNT   8
#define ERR_MODBUS_Func    1
#define ERR_MODBUS_ADDR    2
#define ERR_MODBUS_PARA    3
#define ERR_MODBUS_WRITE   4

#define U2_485_DIR_PORT		U2_DIR_GPIO_Port // GPIOD
#define U2_485_DIR_PIN		U2_DIR_Pin // GPIO_PIN_2
#define U2_485_PIN_HIGH     NULL
#define U2_485_PIN_LOW      NULL
#define U2_TX_BUFF          NULL
#define U2_RX_BUFF          NULL

#define CTRL_FLAG           stModb.ctrlFlag.all[0]
#define MEM_REG_1           stModb.wordReg1.Driver
#define INPUT_REG_1         stModb.inputReg1.inputReg
#define INPUT_REG_2         stModb.inputReg2.inputReg
#define INPUT_REG_3         stModb.inputReg3.inputReg
/*================================================================================================*=
 * GLOBAL CONSTANTS
 *================================================================================================*/
static const char auchCRCHi[] =
{
	0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,0x01,0xc0,0x80,0x41,0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,
	0x00,0xc1,0x81,0x40,0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,0x01,0xc0,0x80,0x41,0x00,0xc1,0x81,0x40,
	0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,0x01,0xc0,0x80,0x41,
	0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,0x00,0xc1,0x81,0x40,0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,
	0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,0x01,0xc0,0x80,0x41,0x00,0xc1,0x81,0x40,0x00,0xc1,0x81,0x40,
	0x01,0xc0,0x80,0x41,0x01,0xc0,0x80,0x41,0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,0x00,0xc1,0x81,0x40,
	0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,0x01,0xc0,0x80,0x41,0x00,0xc1,0x81,0x40,0x00,0xc1,0x81,0x40,
	0x01,0xc0,0x80,0x41,0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,0x01,0xc0,0x80,0x41,0x00,0xc1,0x81,0x40,
	0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,0x01,0xc0,0x80,0x41,0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,
	0x00,0xc1,0x81,0x40,0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,
	0x01,0xc0,0x80,0x41,0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,0x00,0xc1,0x81,0x40,0x00,0xc1,0x81,0x40,
	0x01,0xc0,0x80,0x41,0x01,0xc0,0x80,0x41,0x00,0xc1,0x81,0x40,0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,
	0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,0x01,0xc0,0x80,0x41,0x00,0xc1,0x81,0x40
};

static const char auchCRCLo[] =
{
	0x00,0xc0,0xc1,0x01,0xc3,0x03,0x02,0xc2,0xc6,0x06,0x07,0xc7,0x05,0xc5,0xc4,0x04,0xcc,0x0c,0x0d,0xcd,
	0x0f,0xcf,0xce,0x0e,0x0a,0xca,0xcb,0x0b,0xc9,0x09,0x08,0xc8,0xd8,0x18,0x19,0xd9,0x1b,0xdb,0xda,0x1a,
	0x1e,0xde,0xdf,0x1f,0xdd,0x1d,0x1c,0xdc,0x14,0xd4,0xd5,0x15,0xd7,0x17,0x16,0xd6,0xd2,0x12,0x13,0xd3,
	0x11,0xd1,0xd0,0x10,0xf0,0x30,0x31,0xf1,0x33,0xf3,0xf2,0x32,0x36,0xf6,0xf7,0x37,0xf5,0x35,0x34,0xf4,
	0x3c,0xfc,0xfd,0x3d,0xff,0x3f,0x3e,0xfe,0xfa,0x3a,0x3b,0xfb,0x39,0xf9,0xf8,0x38,0x28,0xe8,0xe9,0x29,
	0xeb,0x2b,0x2a,0xea,0xee,0x2e,0x2f,0xef,0x2d,0xed,0xec,0x2c,0xe4,0x24,0x25,0xe5,0x27,0xe7,0xe6,0x26,
	0x22,0xe2,0xe3,0x23,0xe1,0x21,0x20,0xe0,0xa0,0x60,0x61,0xa1,0x63,0xa3,0xa2,0x62,0x66,0xa6,0xa7,0x67,
	0xa5,0x65,0x64,0xa4,0x6c,0xac,0xad,0x6d,0xaf,0x6f,0x6e,0xae,0xaa,0x6a,0x6b,0xab,0x69,0xa9,0xa8,0x68,
	0x78,0xb8,0xb9,0x79,0xbb,0x7b,0x7a,0xba,0xbe,0x7e,0x7f,0xbf,0x7d,0xbd,0xbc,0x7c,0xb4,0x74,0x75,0xb5,
	0x77,0xb7,0xb6,0x76,0x72,0xb2,0xb3,0x73,0xb1,0x71,0x70,0xb0,0x50,0x90,0x91,0x51,0x93,0x53,0x52,0x92,
	0x96,0x56,0x57,0x97,0x55,0x95,0x94,0x54,0x9c,0x5c,0x5d,0x9d,0x5f,0x9f,0x9e,0x5e,0x5a,0x9a,0x9b,0x5b,
	0x99,0x59,0x58,0x98,0x88,0x48,0x49,0x89,0x4b,0x8b,0x8a,0x4a,0x4e,0x8e,0x8f,0x4f,0x8d,0x4d,0x4c,0x8c,
	0x44,0x84,0x85,0x45,0x87,0x47,0x46,0x86,0x82,0x42,0x43,0x83,0x41,0x81,0x80,0x40
};

/*================================================================================================*=
 * GLOBAL VARIABLES
 *================================================================================================*/
extern UART_HandleTypeDef huart2;
extern UART_STR U2;
extern MODBUS_STR stModb;
extern uint8_t Rx_data[RX_DATA_SIZE];  //  creating a buffer of RX_DATA_SIZE bytes
extern uint8_t Rx_count;
/*================================================================================================*=
 * GLOBAL FUNCTIONS
 *================================================================================================*/
void Modbus_Slave_init();
void detec_uart(void);
void modbus_slave_value_update(int16_t addr, int16_t *pData, uint8_t dataLen);
void uart_timOut(void);
void UartResponse(void);
void ctrl_rs485_pin(UART_STR* Ux, int8_t flag);
/*================================================================================================*=
 * END OF FILE
 *================================================================================================*/
#endif /* INC_APP_MODBUS_RTU_H_ */
