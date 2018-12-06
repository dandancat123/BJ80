/*  BEGIN_FILE_HDR
 ************************************************************************************************
 *   NOTICE
 *   This software is the property of IntelliDrive Co.,Ltd.. Any information contained in this
 *   doc should not be reproduced, or used, or disclosed without the written authorization from
 *   IntelliDrive Co.,Ltd..
 ************************************************************************************************
 *   File Name       : COP_Drv.h
 *   Project         : LdwpvMcuApplication
 *   Processor       : S12XEP100
 *   Description        :
 *         This file implements the Computer Operating Properly (COP)
 *         module initialization according to the Peripheral
 *         Initialization Component settings.
 *         The (COP) module contains a free-running counter that generates
 *         a reset if allowed to overflow. The COP module
 *         helps software recover from runaway code. Prevent a COP reset by
 *         clearing the COP counter periodically.
 *   Component        :
 *         COP_Drv_Init
 *         COP_FeedDog
 *         COP_SwReset
 ************************************************************************************************
 *   Revision History:
 *
 *   Version     Date          Initials      CR#          Descriptions
 *   ---------   ----------    -----------   ----------   ---------------
 *   1.0         2013/10/12    SongPing      N/A          Original
 *
 ************************************************************************************************
 *    END_FILE_HDR*/

/*
 ** ===================================================================
 **     Include files.
 ** ===================================================================
 */
#include "COP_Drv.h"


/*
 ** ===================================================================
 **     Precompile.
 ** ===================================================================
 */
#pragma DATA_SEG DEFAULT               /* Select data segment "DEFAULT" */
#pragma CODE_SEG DEFAULT


/*
 ** ===================================================================
 **     Method      :  COP_Init (component COP)
 **
 **     Description :
 **         This function initializes registers of the COP module
 **         according to this Peripheral Initialization settings. Call
 **         this method in the user code to initialize the module.
 **         Clock setting
 **           Clock source                               : OSCCLK[8MHz]
 **           Time-out period prescaler                  : 6
 **           Time-out period                            : 1048.576 ms
 **     Parameters  : None
 **     Returns     : Nothing
 ** ===================================================================
 */
void COP_Init(void)
{
    COPCTL = 0x46U;                     /* COP Period = 2 ^ 23 / 8MHz = 1.04s */
}


/*
 ** ===================================================================
 **     Method      :  COP_FeedDog (component COP)
 **
 **     Description :
 **         This function restart the COP time-out period by writting
 **         0x55 followed by 0xAA to the register CPMUARMCOP.
 **     Parameters  : None
 **     Returns     : Nothing
 ** ===================================================================
 */
void COP_FeedDog(void)
{
    ARMCOP = 0x55U, ARMCOP = 0xAAU;
}


/*
 ** ===================================================================
 **     Method      :  COP_SwReset (component COP)
 **
 **     Description :
 **         This function causes a COP reset by writing 0x00 to
 **         the register CPMUARMCOP.
 **     Parameters  : None
 **     Returns     : Nothing
 ** ===================================================================
 */
void COP_SwReset(void)
{
    ARMCOP = 0x00U;
}



/* EOF */
