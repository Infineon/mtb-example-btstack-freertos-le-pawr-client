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
* Copyright 2021-2025, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
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
