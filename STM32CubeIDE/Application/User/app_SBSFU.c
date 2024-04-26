/***********************************************************************************************//**
 * @file        app_SBSFU.c
 * @brief       app_SBSFU
 * @date        APR 17, 2024
 * @version     1.0.0
 * @author      TOM.TANG
 *
 * Revision History
 *------------------------------------------------------------------------------------------------
 *|Version   |Date       |Author         |Change ID      |Description                            |
 *|----------|-----------|---------------|---------------|---------------------------------------|
 *|1.0.0     |2024.04.17 |TOM.TANG       |00000000000    |Initial version created                |
 **************************************************************************************************/
/*================================================================================================*=
 * INCLUDE FILES
 *================================================================================================*/
#include "main.h"
#include "app_SBSFU.h"
/*================================================================================================*=
 * LOCAL CONSTANTS
 *================================================================================================*/


/*================================================================================================*=
 * LOCAL VARIABLES
 *================================================================================================*/
//flash
uint32_t Data_Length;
void *Destination = (void *)DWL_SLOT_START;
uint8_t Source[DATA_BYTES] __attribute__((__aligned__(4)));
uint8_t fw_header_dwl_slot[SE_FW_HEADER_TOT_LEN];
uint32_t flash_write_fail = 0;
uint32_t flash_erase_fail = 0;
/*================================================================================================*=
 * LOCAL MACROS
 *================================================================================================*/


/*================================================================================================*=
 * GLOBAL CONSTANTS
 *================================================================================================*/


/*================================================================================================*=
 * GLOBAL VARIABLES
 *================================================================================================*/

/*================================================================================================*=
 * LOCAL FUNCTIONS PROTOTYPE
 *================================================================================================*/
//falsh
HAL_StatusTypeDef SFU_APP_InstallAtNextReset(uint8_t *fw_header);
static HAL_StatusTypeDef WriteInstallHeader(uint8_t *pfw_header);

/*================================================================================================*=
 * LOCAL FUNCTIONS
 *================================================================================================*/

// DMA transfer complete callback function
void HAL_UART2_TxCpltCallback(UART_HandleTypeDef *huart)
{
	U2.tx.ok = RESET;
	U2.rx.ok = RESET;
	U2.rx.size = RESET;
	ctrl_rs485_pin(&U2, RESET);

	if(U2.rx.buf[1] == 0x15 
	&& U2.rx.buf[0] == 0x02
	&& stModb.wordReg2.updateInfoReg.process == 0x0001)
	{
		//update data length in the first
		Data_Length = U2.rx.buf[9] * 2;

		if(Data_Length == 0)
		return;
		
		memcpy(Source, &U2.rx.buf[10], Data_Length);

		__disable_irq();

		//Download .sfb to the flash
		if(HAL_OK != FLASH_If_Write(Destination, Source, Data_Length))
		{
			flash_write_fail++;
		}
		else
		{
			Destination += 0xf0;
		}
		__enable_irq();
	}

	// setting installation header and reboot
	if(stModb.wordReg2.updateInfoReg.header == 0x0001)
	{
		__disable_irq();
		/* Read header in dwl slot */
		(void)FLASH_If_Read(fw_header_dwl_slot, (void *)DWL_SLOT_START, SE_FW_HEADER_TOT_LEN);

		/* Ask for installation at next reset */
		(void)SFU_APP_InstallAtNextReset((uint8_t *) fw_header_dwl_slot);
		__enable_irq();

		/* System Reboot*/
		// printf("  -- Image correctly downloaded - reboot\r\n\n");
		NVIC_SystemReset();
	}
}

static HAL_StatusTypeDef WriteInstallHeader(uint8_t *pfw_header)
{
  HAL_StatusTypeDef ret = HAL_OK;

  ret = FLASH_If_Erase_Size((void *) SWAP_SLOT_START, SFU_IMG_IMAGE_OFFSET);
  if (ret == HAL_OK)
  {
    ret = FLASH_If_Write((void *) SWAP_SLOT_START, pfw_header, SE_FW_HEADER_TOT_LEN);
  }
  return ret;
}

HAL_StatusTypeDef SFU_APP_InstallAtNextReset(uint8_t *fw_header)
{
#if  !defined(SFU_NO_SWAP)
  if (fw_header == NULL)
  {
    return HAL_ERROR;
  }
  if (WriteInstallHeader(fw_header) != HAL_OK)
  {
    return HAL_ERROR;
  }
  return HAL_OK;
#else
  return HAL_OK;                   /* Nothing to do */
#endif /* !SFU_NO_SWAP */
}


/*================================================================================================*=
 * GLOBAL FUNCTIONS
 *================================================================================================*/

/*================================================================================================*=
 * END OF FILE
 *================================================================================================*/
