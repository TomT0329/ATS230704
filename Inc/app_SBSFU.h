/***********************************************************************************************//**
 * @file        app_SBSFU.h
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
#ifndef INC_APP_SBSFU_H_
#define INC_APP_SBSFU_H_
/*================================================================================================*=
 * INCLUDE FILES
 *================================================================================================*/
#include "flash_if.h"
/*================================================================================================*=
 * GLOBAL MACROS
 *================================================================================================*/
//Flash Erase
extern uint32_t _PageError;
#define SWAP_SLOP_PAGE 70
#define ERASE_PAGES   58

#define SE_FW_HEADER_TOT_LEN 320 // sizeof(SE_FwRawHeaderTypeDef)

//mem address
#define DWL_SLOT_START (uint32_t) 0x08048000
#define DWL_SLOT_END (uint32_t) 0x0807DFFF
#define SWAP_SLOT_START (uint32_t) 0x08046000
extern void *Destination;
/**
  * Image starting offset to add to the  address of 1st block
  */
/* For G4 as active slot(s) header is(are) protected by Secure user memory, OFFSET is aligned on sector size : 4096 */
#define SFU_IMG_IMAGE_OFFSET ((uint32_t)4096U)
/*================================================================================================*=
 * LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
 *================================================================================================*/


/*================================================================================================*=
 * GLOBAL CONSTANTS
 *================================================================================================*/


/*================================================================================================*=
 * GLOBAL VARIABLES
 *================================================================================================*/


/*================================================================================================*=
 * GLOBAL FUNCTIONS
 *================================================================================================*/

/*================================================================================================*=
 * END OF FILE
 *================================================================================================*/
#endif /* INC_APP_SBSFU_H_ */
