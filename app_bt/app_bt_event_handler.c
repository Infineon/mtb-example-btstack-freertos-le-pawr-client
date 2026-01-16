/*******************************************************************************
* File Name: app_bt_event_handler.c
*
* Description: This file contains the bluetooth event handler that processes
*              the bluetooth events from host stack.
*
* Related Document: See README.md
*
*
********************************************************************************
 * (c) 2021-2026, Infineon Technologies AG, or an affiliate of Infineon
 * Technologies AG. All rights reserved.
 * This software, associated documentation and materials ("Software") is
 * owned by Infineon Technologies AG or one of its affiliates ("Infineon")
 * and is protected by and subject to worldwide patent protection, worldwide
 * copyright laws, and international treaty provisions. Therefore, you may use
 * this Software only as provided in the license agreement accompanying the
 * software package from which you obtained this Software. If no license
 * agreement applies, then any use, reproduction, modification, translation, or
 * compilation of this Software is prohibited without the express written
 * permission of Infineon.
 *
 * Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
 * THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
 * SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
 * Infineon reserves the right to make changes to the Software without notice.
 * You are responsible for properly designing, programming, and testing the
 * functionality and safety of your intended application of the Software, as
 * well as complying with any legal requirements related to its use. Infineon
 * does not guarantee that the Software will be free from intrusion, data theft
 * or loss, or other breaches ("Security Breaches"), and Infineon shall have
 * no liability arising out of any Security Breaches. Unless otherwise
 * explicitly approved by Infineon, the Software may not be used in any
 * application where a failure of the Product or any consequences of the use
 * thereof can reasonably be expected to result in personal injury.
*******************************************************************************/

/*******************************************************************************
* Header Files
 ******************************************************************************/

#include "inttypes.h"
#include <FreeRTOS.h>
#include <task.h>
#include <string.h>
#include "cycfg_gap.h"
#include "app_bt_utils.h"
#include "app_bt_event_handler.h"
#include "app_bt_utils.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#ifdef ENABLE_BT_SPY_LOG
#include "cybt_debug_uart.h"
#endif

#include "wiced_bt_trace.h"
#include "cybt_platform_trace.h"
#include "pawr_app.h"

/*******************************************************************************
 * Macro Definitions
*******************************************************************************/

/*******************************************************************************
 * Variable Definitions
*******************************************************************************/

/*******************************************************************************
 * Function Definitions
******************************************************************************/
/**************************************************************************************************
* Function Name: app_bt_management_callback
***************************************************************************************************
* Function Description:
* @brief
* This function is a Bluetooth stack event handler function to receive management events
* from the Bluetooth stack and process as per the application.
* @param event        , Bluetooth LE event code of one byte length.
* @param p_event_data , Pointer to Bluetooth LE management event structures.
* @return wiced_result_t Error code from WICED_RESULT_LIST or BT_RESULT_LIST.
*/
wiced_result_t app_bt_management_callback(wiced_bt_management_evt_t event,
                                          wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t result  = WICED_BT_ERROR;
    printf("Bluetooth Management Event:0x%x\r\n",event);
    switch (event)
    {
        case BTM_ENABLED_EVT:
            /* Bluetooth Controller and Host Stack Enabled */
            if (WICED_BT_SUCCESS == p_event_data->enabled.status)
            {
                app_central_init();
                result = WICED_BT_SUCCESS;
            }
            else
            {
                printf( "Failed to initialize Bluetooth controller and stack\n");
            }
        break;
        default:
            printf( "unknown:evt:0x%x\n",event);
        break;
    }
    return result;
}

/* END OF FILE [] */
