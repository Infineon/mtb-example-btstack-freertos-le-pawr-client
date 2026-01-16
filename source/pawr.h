/******************************************************************************
* File Name:   pawr.h
*
* Description: This file consists of the inteface for PAwR control.
*
* Related Document: See README.md
*
*
*******************************************************************************
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

