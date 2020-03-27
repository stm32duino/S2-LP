/**
 * @file    S2LP_Types.c
 * @author  ST Microelectronics
 * @version 1.3.0
 * @date    June, 2019
 * @brief   This file provides functions to manage S2-LP debug.
 * @details
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * THIS SOURCE CODE IS PROTECTED BY A LICENSE.
 * FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
 * IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
 *
 * <h2><center>&copy; COPYRIGHT 2019 STMicroelectronics</center></h2>
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "S2LP.h"


/** @addtogroup S2LP_Libraries
 * @{
 */


/** @addtogroup S2LP_Types
 * @{
 */


/** @defgroup Types_Private_TypesDefinitions    Types Private Types Definitions
 * @{
 */


/**
 * @}
 */



/** @defgroup Types_Private_Defines             Types Private Defines
 * @{
 */


/**
 * @}
 */



/** @defgroup Types_Private_Macros               Types Private Macros
 * @{
 */


/**
 * @}
 */



/** @defgroup Types_Private_Variables             Types Private Variables
 * @{
 */



/** @defgroup Types_Private_FunctionPrototypes       Types Private FunctionPrototypes
 * @{
 */



/**
 * @}
 */



/** @defgroup Types_Private_Functions                 Types Private Functions
 * @{
 */


/**
 * @brief  Updates the gState (the global variable used to maintain memory of S2LP Status)
 *         reading the MC_STATE register of S2LP.
 * @param  None
 * @retval None
 */
void S2LP::S2LPRefreshStatus(void)
{  
  uint8_t tempRegValue;
  /* Read the MC_STATE both from register and from SPI header and exit when they match.
      This will protect against possible transition state changes */
  do
  {
    /* Reads the MC_STATE register to update the g_xStatus */
    g_xStatus = S2LPSpiReadRegisters(MC_STATE0_ADDR, 1, &tempRegValue);
  }
  while((tempRegValue>>1)!=g_xStatus.MC_STATE);
}

/**
 * @}
 */



/**
 * @}
 */



/**
 * @}
 */



/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
