/******************************************************************************
* File Name:   pawr.h
*
* Description: This file consists of the inteface for PAwR control.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2021-2023, Cypress Semiconductor Corporation (an Infineon company) or
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

#ifndef PAWR_H_
#define PAWR_H_
/******************************************************************************
* Header Files
*******************************************************************************/
#include "wiced_bt_ble.h"

/*******************************************************************************
* Macro Definitions
*******************************************************************************/
/* enum for Extended ADV parameters */
enum extend_adv_param_e
{
    EXT_ADV_EVENT_PROP      = 0x00, /* extended adv event_properties */
    EXT_ADV_CHANNEL_MAP     = 0x07, /* primary_adv_channel_map 37/38/39 */
    EXT_ADV_TX_POWER        = 0x7F, /* extended adv tx power */
    EXT_ADVSEC_ADV_MAX_SKIP = 0x00, /* extended adv secondary_adv_max_skip */
    EXT_ADV_SET_ID          = 0x00, /* extended adv sid data in the subfield of the ADI field */
    EXT_ADV_EVT_PERIOD      = 800   /* extended adv period *0.625 = 500ms */
};

/* enum for PAwR parameters */
enum pawr_param_e
{
    PAWR_EXT_ADV_EVENT_PROP = 0x00, /* extended adv properties */
    PAWR_SUB_EVT_NUM        = 16,   /* PAwR sub event number */
    PAWR_EVT_PERIOD         = 4000, /* PAwR event period *1.25  = 5000ms */
    PAWR_SUB_EVT_PERIOD     = 250,  /* PAWR_EVT_PERIOD/PAWR_SUB_EVT_NUM 312.5ms */
    PAWR_RESPONSE_DELAY     = 20,   /* PAwR response delay *1.25  = 25ms */
    PAWR_RESPONSE_SPACE     = 40,   /* PAwR response space  *0.125 = 5ms */
    PAWR_RESPONSE_SLOTS     = 8,    /* PAwR response slot number */
    PAWR_ADV_HANDLE         = 0x01  /* PAwR adv handle */
};

/*******************************************************************************
 * Variable Definitions
*******************************************************************************/

/******************************************************************************
 * Function Prototypes
*******************************************************************************/
typedef void (pawr_se_ind_cb_t)(uint8_t adv_handle,uint8_t subevent_start,uint8_t total_subevent);
typedef void (pawr_se_rsp_report_cb_t)(uint8_t adv_handle,uint8_t subevent,uint8_t response_slot,uint8_t data_len,uint8_t *data);
void pawr_reg_se_ind_cb(pawr_se_ind_cb_t *callback);
void pawr_reg_se_rsp_report_cb(pawr_se_rsp_report_cb_t *callback);
void pawr_set_central_addr(const uint8_t *addr);
void pawr_init(void);
#endif /* PAWR_H_ */

