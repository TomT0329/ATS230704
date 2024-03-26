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
#include "app_Modbus_RTU.h"
#include "main.h"
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
UART_STR U2;
UART_STR U1;
MODBUS_STR stModb;
void *Destination = (void *)DWL_SLOT_START;
/*================================================================================================*=
 * LOCAL FUNCTIONS PROTOTYPE
 *================================================================================================*/
static uint16_t get_crc(uint8_t *ptr, uint8_t len);
static char crc_check(uint8_t *ptr, unsigned char size);
static void send_err_code (UART_STR* Ux, UART_HandleTypeDef *pUartHandle, uint8_t exception);
static int16_t get_syst_para(int16_t index);
static void send_Ux(UART_STR* Ux, UART_HandleTypeDef *pUartHandle, uint8_t *pData, uint16_t Size);
void set_syst_para(int16_t index, int16_t dat);
void remoteFun(UART_STR* Ux, UART_HandleTypeDef *pUartHandle);
void modbus03(UART_STR* Ux, UART_HandleTypeDef *pUartHandle);
void modbus04(UART_STR* Ux, UART_HandleTypeDef *pUartHandle);
void modbus06(UART_STR* Ux, UART_HandleTypeDef *pUartHandle);
void modbus10(UART_STR* Ux, UART_HandleTypeDef *pUartHandle);
void modbus15(UART_STR* Ux, UART_HandleTypeDef *pUartHandle);
void Modbus_CtrlReg_Set(void);

/*================================================================================================*=
 * LOCAL FUNCTIONS
 *================================================================================================*/
/*************************************************************************
 * Function Name: get_crc(uint8_t *ptr, uint8_t len)
 *
 * Parameters: uint8_t *, int8_t
 *
 * Return: uint16_t
 *
 * Description: Culcate CRC
 ************************************************************************/
static uint16_t get_crc(uint8_t *ptr, uint8_t len)
{
	unsigned char uchCRCHi = 0xFF; /*CRC high byte*/
	unsigned char uchCRCLo = 0xFF; /*CRC low byte*/
	unsigned uIndex;

	while(len--)
	{ /*pass through message buffer*/
		uIndex = uchCRCHi ^* ptr++; /*calculate the CRC*/
		uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
		uchCRCLo = auchCRCLo[uIndex];
	}

	return (uchCRCHi << 8 | uchCRCLo);
}
/*************************************************************************
 * Function Name: crc_check(uint8_t *ptr, uint8_t len)
 *
 * Parameters: uint8_t *, int8_t
 *
 * Return: uint8_t
 *
 * Description: Check CRC
 ************************************************************************/
static char crc_check(uint8_t *ptr, unsigned char size)
{
	uint16_t calCrc = get_crc(ptr, size - 2);
	uint16_t bufCrc = (*(ptr + size - 2)) << 8;

	bufCrc |= *(ptr + size - 1);
	if(calCrc == bufCrc)
	{
		return 0;
	}

#if ENABLE_TEST_MODE
	if(0xFFFF == bufCrc)
	{
		return 0;
	}
#endif
	return 1;
}
/*************************************************************************
 * Function Name: send_err_code (UART_STR* Ux, uint8_t exception)
 *
 * Parameters: UART_STR* Ux, uint8_t
 *
 * Return: void
 *
 * Description: Modbus Error Function response
 ************************************************************************/
static void  send_err_code (UART_STR* Ux, UART_HandleTypeDef *pUartHandle, uint8_t exception)
{
	UART_BUFF_STR * tx = &Ux->tx;
	UART_BUFF_STR * rx = &Ux->rx;
	uint16_t crcTmp;


	tx->buf[0] = DRIVER_SLAVE_ID;

	tx->buf[1] = rx->buf[1] | 0x80;
	tx->buf[2] = exception;

	crcTmp = get_crc((uint8_t*)tx->buf, 3);
	tx->buf[3] = crcTmp>>8;
	tx->buf[4] = crcTmp;
	tx->size = 5;

	send_Ux(Ux, pUartHandle, tx->buf, tx->size);
}
/*************************************************************************
 * Function Name: get_syst_para
 *
 * Parameters: int16_t*, int16_t
 *
 * Return: int16_t
 *
 * Description: Get modbus holding register data
 ************************************************************************/
static int16_t get_syst_para(int16_t index)
{
	if((index >= _ID_SET_WORD_MIN) && (index <= _ID_SET_WORD_MAX))
	{
		return stModb.wordReg1.wds[index - _ID_SET_WORD_MIN];
	}
	return 0;
}
/*************************************************************************
 * Function Name: send_Ux(UART_STR* Ux,  int8_t *pData, uint16_t Size)
 *
 * Parameters: UART_STR, int8_t, uint16_t
 *
 * Return: void
 *
 * Description: Send modbus package, and to start TX interrput
 ************************************************************************/
static void send_Ux(UART_STR* Ux, UART_HandleTypeDef *pUartHandle, uint8_t *pData, uint16_t Size)
{
//	uint8_t count;
	UART_BUFF_STR * tx = &Ux->tx;
//	UART_BUFF_STR * rx = &Ux->rx;

	if ((pData == NULL) || (Size == 0U))
	{
		return;
	}
	ctrl_rs485_pin(Ux, SET);

	if(Ux == &U2)
	{
		tx->ok = SET;

		if(HAL_UART_Transmit_DMA(&huart2,(uint8_t *)pData,Size))
		{
			Error_Handler();
		}

	}

}
/*************************************************************************
* Function Name: void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
*
* Parameters: UART_HandleTypeDef *huart
*
* Return: None
*
* Description: Rx Transfer completed callback
************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart2)
	{
		U2.tx.ok = RESET;
		U2.rx.ok = RESET;
		U2.rx.size = RESET;
		ctrl_rs485_pin(&U2, RESET);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		__HAL_UNLOCK(&huart1);
//		HAL_UART_Receive_IT(&huart1, (uint8_t *)&uart1_rx_buffer, 1);
	}
	else if(huart == &huart2)
	{
		__HAL_UNLOCK(&huart2);
		StartReception();
	}
}
/*************************************************************************
 * Function Name: set_syst_para
 *
 * Parameters: int16_t, int16_t
 *
 * Return: int16_t
 *
 * Description: Set modbus holding register data
 ************************************************************************/
void set_syst_para(int16_t index, int16_t dat)
{
	if((index >= _ID_SET_WORD_MIN) && (index <= DRIVER_SECURITY2))
	{
		stModb.wordReg1.wds[index - _ID_SET_WORD_MIN] = dat;
	}
}

void set_file_temp(int16_t index, int16_t dat)
{
		stModb.inputReg1.wds[index - 0x0007] = dat;
	return;
}
/*================================================================================================*=
 * GLOBAL FUNCTIONS
 *================================================================================================*/

/*************************************************************************
 * Function Name: ctrl_rs485_pin(UART_STR* Ux, int8_t flag)
 *
 * Parameters: UART_STR, int8_t
 *
 * Return: int8_t
 *
 * Description: Control RS485 dir pin
 ************************************************************************/
void ctrl_rs485_pin(UART_STR* Ux, int8_t flag)
{
	uint8_t i;

	if(Ux == NULL)return;

	if(Ux == &U2)
	{
		HAL_GPIO_WritePin(U2_485_DIR_PORT, U2_485_DIR_PIN, flag);

		for (i = 0; i < 24; i++)
		{};
	}
	if(Ux == &U1)
	{
		HAL_GPIO_WritePin(U1_485_DIR_PORT, U1_485_DIR_PIN, flag);

		for (i = 0; i < 24; i++)
		{};
	}
}

/*************************************************************************
 * Function Name: uart_timOut(void)
 *
 * Parameters: UART_STR
 *
 * Return: int8_t
 *
 * Description: For Timer Interrupt
 ************************************************************************/
void uart_timOut(void)
{
	// UART_STR* Ux = &U2;

	// 	if(Ux->timeOut > 1)
	// 	{
	// 		Ux->timeOut--;
	// 	}
	// 	else if(Ux->timeOut == 1)
	// 	{
	// 		if((Ux->rx.buf[0] != 0) && (Ux->rx.buf[0] != DRIVER_SLAVE_ID))
	// 		{
	// 			Ux->rx.cnt = 0;
	// 		}
	// 		else if(Ux->rx.ok == 0)
	// 		{
	// 			if(Ux->rx.cnt <= 3)
	// 			{
	// 				Ux->rx.cnt = 0;
	// 			}
	// 			else
	// 			{
	// 				Ux->rx.ok = 1;
	// 				Ux->rx.size = Ux->rx.cnt;
	// 			}
	// 		}

	// 		Ux->timeOut = 0;
	// 	}
}
/*************************************************************************
 * Function Name: detec_uart(void)
 *
 * Parameters: UART_STR
 *
 * Return: int8_t
 *
 * Description: For main loop to moniting RS485 modbus package
 ************************************************************************/
void detec_uart(void)
{
	if(U2.tx.ok == RESET)
	{
		remoteFun(&U2, &huart2);
	}
}
/*************************************************************************
 * Function Name: remoteFun
 *
 * Parameters: UART_STR* Ux, UART_HandleTypeDef *pUartHandle,
 *
 * Return: void
 *
 * Description: Remote communication function in here
 ************************************************************************/
void remoteFun(UART_STR* Ux, UART_HandleTypeDef *pUartHandle)
{
	UART_BUFF_STR * rx = &Ux->rx;
	if(rx->ok)
	{
		if(rx->buf[0] != DRIVER_SLAVE_ID)
			return;

		if( !((rx->buf[1] == 0x06) || (rx->buf[1] == 0x03)
				||  (rx->buf[1] == 0x04) || (rx->buf[1] == 0x10) || (rx->buf[1] == 0x15)) )
			return;

		if (crc_check((uint8_t*)rx->buf, rx->size))
		{
			rx->size = RESET;
			printf("RX CRC Fail\n\r");
			return;
		}
		switch(rx->buf[1])
		{
			case 0x03: // Read Holding Registers
				modbus03(&U2, &huart2);
				break;

			case 0x04: // Write Intput Registers
				modbus04(&U2, &huart2);
				break;

			case 0x06: // Write Single Register
				modbus06(&U2, &huart2);
				break;
			case 0x10:
				modbus10(&U2, &huart2);
				break;
			case 0x15:
				modbus15(&U2, &huart2);
		}
		Modbus_CtrlReg_Set();
	}

}

/*************************************************************************
 * Function Name: modbus03
 *
 * Parameters: UART_STR* Ux
 *
 * Return: void
 *
 * Description: Modbus function code 0x03 - Read Holding Registers
 * Request  ex: 11 03 006B 0003 7687
 * Response ex: 11 03 06 AE41 5652 4340 49AD
 ************************************************************************/
void modbus03(UART_STR* Ux, UART_HandleTypeDef *pUartHandle)
{
	UART_BUFF_STR * tx = &Ux->tx;
	UART_BUFF_STR * rx = &Ux->rx;
	//int8_t* pSendBuf = &tx->buf[3];
	TRANS_TYPE addr, count, dat;
	uint16_t crcTmp, dptr = 0;

	addr.b.hi  = rx->buf[2];
	addr.b.lo  = rx->buf[3];
	count.b.hi = rx->buf[4];
	count.b.lo = rx->buf[5];

	if (rx->size != 8)
	{
		send_err_code(Ux, pUartHandle, ERR_MODBUS_PARA);
		rx->size = RESET;
		return;
	}
	else if(count.w < 1 || count.w > 125)
	{
		send_err_code(Ux, pUartHandle, ERR_MODBUS_PARA);
		rx->size = RESET;
		return;
	}

	else if(addr.w < _ID_SET_WORD_MIN || addr.w > (_ID_SET_WORD_MAX - count.w + 1))
	{
		send_err_code(Ux, pUartHandle, ERR_MODBUS_ADDR);
		rx->size = RESET;
		return;
	}

	while (count.w--)
	{
		dat.w = get_syst_para(addr.w + dptr);
		tx->buf[3 + dptr + dptr] = dat.b.hi;
		tx->buf[4 + dptr + dptr] = dat.b.lo;
		dptr++;
	}

	tx->buf[0] = rx->buf[0];
	tx->buf[1] = rx->buf[1];
	tx->buf[2] = dptr + dptr;
	crcTmp = get_crc( (uint8_t*)tx->buf , ( 3 + dptr + dptr ));
	tx->buf[3 + dptr + dptr] = crcTmp >> 8;
	tx->buf[4 + dptr + dptr] = crcTmp & 0xFF;
	tx->size = 5 + dptr + dptr;

	send_Ux(Ux, pUartHandle, tx->buf, tx->size);

}
/*************************************************************************
 * Function Name: modbus04
 *
 * Parameters: UART_STR* Ux
 *
 * Return: void
 *
 * Description: function code 0x04 - Read Input Registers
 * Request  ex: 11 04 0000 0001 31CA
 * Response ex: 11 04 02 dataLen
 ************************************************************************/
void modbus04(UART_STR* Ux, UART_HandleTypeDef *pUartHandle)
{
	//uint8_t dptr = 0;
	uint8_t uDataLen = 0, dptr = 0;
	UART_BUFF_STR * tx = &Ux->tx;
	UART_BUFF_STR * rx = &Ux->rx;
	TRANS_TYPE dat, addr, count;
	uint16_t crcTmp;

	addr.b.hi  = rx->buf[2];
	addr.b.lo  = rx->buf[3];
	count.b.hi = rx->buf[4];
	count.b.lo = rx->buf[5];

	if (rx->size != 8)
	{
		send_err_code(Ux, pUartHandle, ERR_MODBUS_PARA);
		return;
	}
	else if(count.w < 1 || count.w > 125)
	{
		send_err_code(Ux, pUartHandle, ERR_MODBUS_PARA);
		return;
	}
	else if(addr.w < _ID_SET_WORD_MIN || addr.w > (_ID_SET_WORD_MAX - count.w + 1))
	{
		send_err_code(Ux, pUartHandle, ERR_MODBUS_ADDR);
		return;
	}

    // Get input register data
	while(count.w--)
	{
		dat.w = get_syst_para(addr.w + dptr);
		tx->buf[3 + dptr + dptr] = dat.b.hi;
		tx->buf[4 + dptr + dptr] = dat.b.lo;
		dptr++;
		uDataLen += dptr + dptr;
	}

	tx->buf[0] = rx->buf[0];
	tx->buf[1] = rx->buf[1];
	tx->buf[2] = uDataLen;
	crcTmp = get_crc( (uint8_t*)tx->buf , (3 + uDataLen));
	tx->buf[3 + dptr + dptr] = crcTmp >> 8;
	tx->buf[4 + dptr + dptr] = crcTmp & 0xFF;
	tx->size = 5 + uDataLen;

	send_Ux(Ux, pUartHandle, tx->buf, tx->size);
}
/*************************************************************************
 * Function Name: modbus06
 *
 * Parameters: UART_STR* Ux
 *
 * Return: void
 *
 * Description: function code 0x06 - Write Single Register(Byte)
 * Request  ex: 11 06 0001 0003 9A9B
 * Response ex: 11 06 0001 0003 9A9B
 ************************************************************************/
void modbus06(UART_STR* Ux, UART_HandleTypeDef *pUartHandle)
{
	uint8_t i;
	UART_BUFF_STR * tx = &Ux->tx;
	UART_BUFF_STR * rx = &Ux->rx;
	TRANS_TYPE addr, dat;

	addr.b.hi = rx->buf[2];
	addr.b.lo = rx->buf[3];
	dat.b.hi  = rx->buf[4];
	dat.b.lo  = rx->buf[5];

	if(rx->size != 8)
	{
		send_err_code(Ux, pUartHandle, ERR_MODBUS_PARA);
		return;
	}

	if( (addr.w >= _ID_SET_WORD_MIN) && (addr.w <= DRIVER_SECURITY2) )
	{
		set_syst_para(addr.w, dat.w);
	}
	else if(addr.w == 0x2000 || addr.w == 0x3000 ) //SBSFU boot code
	{

	}
	else
	{
		send_err_code(Ux, pUartHandle, ERR_MODBUS_ADDR);
		return;
	}

	if(rx->buf[0] == 0)
		return;

	for(i = 0; i < 8; i++)
	{
		tx->buf[i] = rx->buf[i];
	}

	tx->size = rx->size;
	send_Ux(Ux, pUartHandle, tx->buf, tx->size);
}
/*************************************************************************
 * Function Name: modbus10
 *
 * Parameters: UART_STR* Ux
 *
 * Return: void
 *
 * Description: function code 0x10 - Write Multiple Registers(Byte)
 * Request  ex: 11 10 0001 0002 04 000A 0102 C6F0
 * Response ex: 11 10 0001 0002 1298
 ************************************************************************/
void modbus10(UART_STR* Ux, UART_HandleTypeDef *pUartHandle)
{
	UART_BUFF_STR * tx = &Ux->tx;
	UART_BUFF_STR * rx = &Ux->rx;
	uint16_t crcTmp, dptr = 0;
	uint8_t bytes;
	TRANS_TYPE addr, count, dat;

	addr.b.hi = rx->buf[2];
	addr.b.lo = rx->buf[3];
	count.b.hi = rx->buf[4];
	count.b.lo = rx->buf[5];
	bytes = count.w *2 ;

	if(rx->buf[0] != DRIVER_SLAVE_ID)
		return;
	else if(rx->buf[6] != bytes)
		return;
	else if(count.w < 1 || count.w > 125)//check boundary
	{
		send_err_code(Ux, pUartHandle, ERR_MODBUS_PARA);
		rx->size = RESET;
		return;
	}
	else if(addr.w < _ID_SET_WORD_MIN || addr.w > (_ID_SET_WORD_MAX - count.w + 1))
	{
		send_err_code(Ux, pUartHandle, ERR_MODBUS_ADDR);
		rx->size = RESET;
		return;
	}

	while (count.w--)
	{
		dat.b.hi  = rx->buf[7 + dptr + dptr];
		dat.b.lo  = rx->buf[8 + dptr + dptr];
		set_syst_para(addr.w + dptr, dat.w);
		dptr++;
	}

	tx->buf[0] = rx->buf[0];
	tx->buf[1] = rx->buf[1];
	tx->buf[2] = rx->buf[2];
	tx->buf[3] = rx->buf[3];
	tx->buf[4] = rx->buf[4];
	tx->buf[5] = rx->buf[5];
	crcTmp = get_crc( (uint8_t*)tx->buf , 6);
	tx->buf[6] = crcTmp >> 8;
	tx->buf[7] = crcTmp & 0xFF;
	tx->size = 8;

	send_Ux(Ux, pUartHandle, tx->buf, tx->size);
}
/*************************************************************************
 * Function Name: modbus15
 *
 * Parameters: UART_STR* Ux
 *
 * Return: void
 *
 * Description: function code 0x0f -Write File Record
 * Request  ex: 01 15 0D 06 0004 0007 0003 0101 0202 0303 73FC
 * Response ex: 01 15 0D 06 0004 0007 0003 0101 0202 0303 73FC
 ************************************************************************/
void modbus15(UART_STR* Ux, UART_HandleTypeDef *pUartHandle)
{
	UART_BUFF_STR * rx = &Ux->rx;
	uint8_t len, reference;
	TRANS_TYPE record, file;

	len = rx->buf[2];
	reference = rx->buf[3];
	file.b.hi = rx->buf[4];
	file.b.lo = rx->buf[5];
	record.b.hi = rx->buf[6];
	record.b.lo = rx->buf[7];
//	count.b.hi = rx->buf[8];
//	count.b.lo = rx->buf[9];

	if(rx->buf[0] != DRIVER_SLAVE_ID)
		return;
	else if(rx->buf[1] != 0x15)
	{
		send_err_code(Ux, pUartHandle, ERR_MODBUS_Func);
		rx->size = RESET;
		return;
	}
	else if(len < 0x09 || len > 0xfb)
	{
		send_err_code(Ux, pUartHandle, ERR_MODBUS_PARA);
		rx->size = RESET;
		return;
	}
	else if(reference != 0x06 && file.w == 0x0004 && record.w == 0x0007)
	{
		send_err_code(Ux, pUartHandle, ERR_MODBUS_ADDR);
		rx->size = RESET;
		return;
	}


//	while (count.w--)
//	{
//		dat.b.hi  = rx->buf[10 + dptr + dptr];
//		dat.b.lo  = rx->buf[11 + dptr + dptr];
//		set_file_temp(record.w + dptr, dat.w);
//		dptr++;
//	}

	send_Ux(Ux, pUartHandle, rx->buf, rx->size);
}
/*************************************************************************
 * Function Name: modbus_slave_value_update()
 *
 * Parameters: 
 *
 * Return: None
 *
 * Description: Update MODBUS data
 ************************************************************************/
void modbus_slave_value_update()
{
	BusVoltageSensor_Handle_t* BusVoltageSensor= &BusVoltageSensor_M1._Super;

	if(MCI_GetSTMState(&Mci[M1]) == RUN)
	{
		MODBUS_SET_BIT(stModb.wordReg1.wds[DRIVER_STATUS],0);
	}
	else
	{
		MODBUS_CLEAR_BIT(stModb.wordReg1.wds[DRIVER_STATUS],0);
	}

	if(MC_GetOccurredFaultsMotor1())
	{
		MODBUS_SET_BIT(stModb.wordReg1.wds[DRIVER_STATUS],7);
	}
	else
	{
		MODBUS_CLEAR_BIT(stModb.wordReg1.wds[DRIVER_STATUS],7);
	}

	stModb.wordReg1.wds[COMP_FREQ] = (uint16_t)MC_GetMecSpeedAverageMotor1();

	stModb.wordReg1.wds[COMP_POWER] = (uint16_t)MC_GetAveragePowerMotor1_F();

	stModb.wordReg1.wds[IPM_TEMP] = (uint16_t)IPM_temp;

	stModb.wordReg1.wds[IPM_PWMFREQ] = (uint16_t)PWM_FREQUENCY;

	stModb.wordReg1.wds[DCBUS_VOLTAGE] = (int16_t)VBS_GetAvBusVoltage_V(BusVoltageSensor);

	stModb.wordReg1.wds[AC_VOLTAGE] = (int16_t)PFC_voltage_rms;

}

void Modbus_CtrlReg_Set(void)
{
	UART_BUFF_STR * rx = &U2.rx;
	TRANS_TYPE addr;
	addr.b.hi = rx->buf[2];
	addr.b.lo = rx->buf[3];
	if(rx->buf[1] == 0x06)
	{
		switch (addr.w)
		{
			case DRIVER_CTRL:

			if(MODBUS_GET_BIT(stModb.wordReg1.wds[DRIVER_CTRL],0))
			{
				MCI_StartMotor(pMCI[0]);
			}else MCI_StopMotor(pMCI[0]);

			if(MODBUS_GET_BIT(stModb.wordReg1.wds[DRIVER_CTRL],2))
			{
				//PFC
			}else;

			if(MODBUS_GET_BIT(stModb.wordReg1.wds[DRIVER_CTRL],3))
			{
				//Clear restarup
			}else;

			if(MODBUS_GET_BIT(stModb.wordReg1.wds[DRIVER_CTRL],7))
			{				
				//Clear fault Bit
				MCI_FaultAcknowledged(pMCI[0]);
			}else;

			break;
			case DRIVER_PARA:

			break;
			case DRIVER_EEPROM:

			break;
			case DRIVER_FREQ:

			if(MC_HasRampCompletedMotor1() && stModb.wordReg1.wds[DRIVER_FREQ])
			{
				int16_t spd_err = (stModb.wordReg1.wds[DRIVER_FREQ] - MC_GetMecSpeedAverageMotor1());
				
				if(spd_err < 0)
				{
					spd_err = -spd_err;
				}

				ACC_Time = (uint16_t)(((float)spd_err) / ACC_Value);
				
				if(ACC_Time < MIN_ACC_TIME)
				{
					ACC_Time = MIN_ACC_TIME;
				}
				if(ACC_Time > MAX_ACC_TIME)
				{
					ACC_Time = MAX_ACC_TIME;
				}
				MCI_ExecSpeedRamp(&Mci[M1],(int16_t)(stModb.wordReg1.wds[DRIVER_FREQ]),ACC_Time);
			}
			break;
			case HEATER_SHCURRENT:

			break;
			case DRIVER_OAT:
			break;

			case DRIVER_RESERVED1:

			for(int i = ERROR_BUFFER_SIZE -1 ; i >0 ; i--)
			{
				printf("%d, ", (int)Error_buffer[i]);
			}
			printf("\n\nPast Fault code : %u, ", MC_GetOccurredFaultsMotor1());
			printf("Current Fault code : %u.\n\n", MC_GetCurrentFaultsMotor1());

			break;
			case DRIVER_SECURITY1:

			break;
			case DRIVER_SECURITY2:

			break;
		}
	}
	else if(rx->buf[1] == 0x10 
	&& rx->buf[23] == 0x00
	&& rx->buf[24] == 0x00
	&& rx->buf[25] == 0x00
	&& rx->buf[26] == 0x00)
	{
		if(MODBUS_GET_BIT(stModb.wordReg1.wds[DRIVER_CTRL],0) && stModb.wordReg1.wds[DRIVER_FREQ])
		{
			MC_StartMotor1();
		}
		else
		{
			MC_StopSpeedRampMotor1();
			MC_StopMotor1();
		}

		if(MODBUS_GET_BIT(stModb.wordReg1.wds[DRIVER_CTRL],2))
		{
			HAL_GPIO_WritePin(PFC_EN_GPIO_Port,PFC_EN_Pin,GPIO_PIN_RESET);
		}else HAL_GPIO_WritePin(PFC_EN_GPIO_Port,PFC_EN_Pin,GPIO_PIN_SET);

		if(MC_HasRampCompletedMotor1() && stModb.wordReg1.wds[DRIVER_FREQ])
		{
			int16_t spd_err = (stModb.wordReg1.wds[DRIVER_FREQ] - MC_GetMecSpeedAverageMotor1());
			
			if(spd_err < 0)
			{
				spd_err = -spd_err;
			}

			ACC_Time = (uint16_t)(((float)spd_err) / ACC_Value);
			
			if(ACC_Time < MIN_ACC_TIME)
			{
				ACC_Time = MIN_ACC_TIME;
			}
			if(ACC_Time > MAX_ACC_TIME)
			{
				ACC_Time = MAX_ACC_TIME;
			}
			if(stModb.wordReg1.wds[DRIVER_FREQ] > MAX_SPEED_01HZ)
			{
				stModb.wordReg1.wds[DRIVER_FREQ] = MAX_SPEED_01HZ;
			}
			if(stModb.wordReg1.wds[DRIVER_FREQ] < MIN_SPEED_01HZ)
			{
				stModb.wordReg1.wds[DRIVER_FREQ] = MIN_SPEED_01HZ;
			}
			
			MCI_ExecSpeedRamp(&Mci[M1],(int16_t)(stModb.wordReg1.wds[DRIVER_FREQ]),ACC_Time);
		}

		if(MC_GetOccurredFaultsMotor1() && stModb.wordReg1.wds[DRIVER_FREQ] == 0)
		{
			//save the error code first!
			HAL_NVIC_SystemReset();
		}


	}
}

/*================================================================================================*=
 * END OF FILE
 *================================================================================================*/
