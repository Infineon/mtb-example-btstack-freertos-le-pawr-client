/******************************************************************************
* File Name:   pawr.c
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

/*******************************************************************************
* Header Files
*******************************************************************************/
#include <string.h>
#include "cyabs_rtos.h"
#include "cybsp.h"
#include "cyhal.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_trace.h"
#include "wiced_timer.h"
#include "wiced_memory.h"
#include "wiced_bt_cfg.h"
#include "cybt_platform_trace.h"
#include "pawr.h"
#ifdef ENABLE_BT_SPY_LOG
#include "cybt_debug_uart.h"
#else
#include "cy_retarget_io.h"
#endif
#include "app_bt_utils.h"

/*******************************************************************************
* Macro Definitions
*******************************************************************************/

/*******************************************************************************
* Variable Definitions
*******************************************************************************/
static wiced_bt_device_address_t NULL_BDA           = {0};
static uint8_t pawr_central_address[BD_ADDR_LEN]    = {0x00};
pawr_se_ind_cb_t        * pawr_se_ind_cb            = NULL;
pawr_se_rsp_report_cb_t * pawr_se_rsp_report_cb     = NULL;

/******************************************************************************
* Function Definitions
******************************************************************************/
/**************************************************************************************************
* Function Name: pawr_reg_se_ind_cb()
***************************************************************************************************
* Function Description:
* @brief
* This function reg PAwR sub event indication callback.
* @param[in] callback, callback function for sub event indication.
* @return void.
**************************************************************************************************/
void pawr_reg_se_ind_cb(pawr_se_ind_cb_t *callback)
{
    printf("pawr_reg_subevt_ind_cb\n");
    if (callback)
    {
        pawr_se_ind_cb = callback;
    }
}

/**************************************************************************************************
* Function Name: pawr_inform_se_ind_app()
***************************************************************************************************
* Function Description:
* @brief
* This function inform the subevent indication to app.
*  @param[in] adv_handle     , handle for synchronized advertising train.
*  @param[in] subevent_start , PAwR ind subevent start.
*  @param[in] total_subevent , PAwR ind subevent total number.
*  @return void
**************************************************************************************************/
static void pawr_inform_se_ind_snd_app(uint8_t adv_handle,uint8_t subevent_start,uint8_t total_subevent)
{
    if (pawr_se_ind_cb)
    {
        pawr_se_ind_cb(adv_handle,subevent_start,total_subevent);
    }
}

/**************************************************************************************************
* Function Name: pawr_reg_se_rsp_report_cb()
***************************************************************************************************
* Function Description:
* @brief
* This function reg PAwR sub event response report callback.
* @param[in] callback, callback function for sub event response report callback.
* @return void.
**************************************************************************************************/
void pawr_reg_se_rsp_report_cb(pawr_se_rsp_report_cb_t *callback)
{
    printf("pawr_reg_subevt_rsp_report_cb\n");
    if (callback)
    {
        pawr_se_rsp_report_cb = callback;
    }
}

/**************************************************************************************************
* Function Name: pawr_inform_se_rsp_report_app()
***************************************************************************************************
* Function Description:
* @brief
* This function inform the PAwR sub event response report to app.
* @param[in] adv_handle    , handle for synchronized advertising train.
* @param[in] subevent      , PAwR subevent response report subevent.
* @param[in] response_slot , PAwR subevent response report slot.
* @param[in] data_len      , PAwR subevent response report data len.
* @param[in] data          , PAwR subevent response report data point.
* @return void.
**************************************************************************************************/
static void pawr_inform_se_rsp_report_app(uint8_t adv_handle,
                                          uint8_t subevent,
                                          uint8_t response_slot,
                                          uint8_t data_len,
                                          uint8_t *data)
{
    if (pawr_se_rsp_report_cb)
    {
        pawr_se_rsp_report_cb(adv_handle,subevent,response_slot,data_len,data);
    }
}

/**************************************************************************************************
* Function Name: pawr_finish_creating_pawr_network()
***************************************************************************************************
* Function Description:
* @brief
* This function indicate create the PAwR network finished.
* @param[in] void.
* @return    void.
**************************************************************************************************/
static void pawr_finish_creating_pawr_network(void)
{
    printf("pawr_finish_creating_pawr_network\n");
    wiced_ble_ext_adv_duration_config_t ext_cfg = {PAWR_ADV_HANDLE, 0, 0};
    /* Set periodic ADV enable */
    printf("wiced_bt_ble_start_periodic_adv\n");
    wiced_ble_padv_enable_adv(PAWR_ADV_HANDLE, WICED_TRUE);
    printf("wiced_bt_ble_start_ext_adv\n");
    /* Set extended ADV enable */
    wiced_ble_ext_adv_enable(WICED_TRUE, 1, &ext_cfg);
    printf("pawr start\n");
}

/**************************************************************************************************
* Function Name: pawr_create_pawr_network()
***************************************************************************************************
* Function Description:
* @brief
* This function create a PAwR network.
* @param[in] void.
* @return    void.
**************************************************************************************************/
static void pawr_create_pawr_network(void)
{
    wiced_bt_dev_status_t status = WICED_BT_ERROR;
    wiced_ble_ext_adv_params_t ext_adv_params;
    wiced_ble_padv_params_t periodic_adv_params;

    ext_adv_params.event_properties = EXT_ADV_EVENT_PROP;
    ext_adv_params.primary_adv_int_min = EXT_ADV_EVT_PERIOD;
    ext_adv_params.primary_adv_int_max = EXT_ADV_EVT_PERIOD;
    ext_adv_params.primary_adv_channel_map = EXT_ADV_CHANNEL_MAP;
    ext_adv_params.own_addr_type = WICED_BLE_OWN_ADDR_PUBLIC;
    ext_adv_params.peer_addr_type = BLE_ADDR_PUBLIC;
    memcpy(ext_adv_params.peer_addr, NULL_BDA, 6);
    ext_adv_params.adv_filter_policy = BTM_BLE_ADV_POLICY_ACCEPT_CONN_AND_SCAN;
    ext_adv_params.adv_tx_power = EXT_ADV_TX_POWER;
    ext_adv_params.primary_adv_phy = WICED_BLE_EXT_ADV_PHY_1M;
    ext_adv_params.secondary_adv_max_skip = EXT_ADVSEC_ADV_MAX_SKIP;
    ext_adv_params.secondary_adv_phy = WICED_BLE_EXT_ADV_PHY_2M;
    ext_adv_params.adv_sid = EXT_ADV_SET_ID;
    ext_adv_params.scan_request_not = WICED_BLE_EXT_ADV_SCAN_REQ_NOTIFY_DISABLE;
    ext_adv_params.primary_phy_opts = WICED_BLE_EXT_ADV_PHY_OPTIONS_NO_PREFERENCE;
    ext_adv_params.secondary_phy_opts = WICED_BLE_EXT_ADV_PHY_OPTIONS_NO_PREFERENCE;

    /* set the Extended Adv paramameters */
    status = wiced_ble_ext_adv_set_params(PAWR_ADV_HANDLE, &ext_adv_params);
    printf("wiced_bt_ble_set_ext_adv_parameters returned:%d\n",status);
    periodic_adv_params.adv_int_min       = PAWR_EVT_PERIOD;
    periodic_adv_params.adv_int_max       = PAWR_EVT_PERIOD;
    periodic_adv_params.adv_properties    = PAWR_EXT_ADV_EVENT_PROP;
    periodic_adv_params.subevent_num      = PAWR_SUB_EVT_NUM;
    periodic_adv_params.subevent_interval = PAWR_SUB_EVT_PERIOD;
    periodic_adv_params.rsp_slot_delay    = PAWR_RESPONSE_DELAY;
    periodic_adv_params.rsp_slot_spacing  = PAWR_RESPONSE_SPACE;
    periodic_adv_params.rsp_slot_num      = PAWR_RESPONSE_SLOTS;

    /* set the Periodic ADV Parameters (v2) */
    status = wiced_ble_padv_set_adv_params(PAWR_ADV_HANDLE, &periodic_adv_params);
    printf("set_pawr_params:status:%d, pawr_evt_period:%d, sub_evt_period:%d, sub_evt_num:%d\n",
           status,
           PAWR_EVT_PERIOD,
           PAWR_SUB_EVT_PERIOD,
           PAWR_SUB_EVT_NUM);
    pawr_finish_creating_pawr_network();
}

/**************************************************************************************************
* Function Name: pawr_ext_adv_callback()
***************************************************************************************************
* Function Description:
* @brief
* This function process Extended ADV events.
* @param[in] event , The extended adv event code.
* @param[in] p_data, Event data refer to wiced_bt_ble_adv_ext_event_data_t.
* @return    void.
**************************************************************************************************/
static void pawr_ext_adv_callback(wiced_ble_ext_adv_event_t event, wiced_ble_ext_adv_event_data_t *p_data)
{
    switch (event)
    {
        case WICED_BT_BLE_PAWR_SUBEVENT_DATA_REQ_EVENT:
            pawr_inform_se_ind_snd_app(p_data->pawr_data_req.adv_handle,
                                       p_data->pawr_data_req.subevent_start,
                                       p_data->pawr_data_req.subevent_start_data_count);
        break;
        case WICED_BT_BLE_PAWR_RSP_REPORT_EVENT:
            if ((p_data->pawr_rsp_report.data_status == 0) && (p_data->pawr_rsp_report.data_length != 0))
            {
                pawr_inform_se_rsp_report_app(p_data->pawr_rsp_report.adv_handle,
                                              p_data->pawr_rsp_report.subevent,
                                              p_data->pawr_rsp_report.response_slot,
                                              p_data->pawr_rsp_report.data_length,
                                              p_data->pawr_rsp_report.p_data);
            }
        break;
        case WICED_BLE_PERIODIC_ADV_SYNC_LOST_EVENT:
            printf("peripheral sub event pawr conn down\n");
        break;
        case WICED_BLE_PERIODIC_ADV_SYNC_ESTABLISHED_EVENT:
            printf("peripheral sub event pawr conn up\n");
        break;
        case WICED_BLE_PERIODIC_ADV_REPORT_EVENT:
            printf("peripheral sub event pawr receive data ind from central\n");
        break;
        default:
        break;
    }
}

/**************************************************************************************************
* Function Name: pawr_set_central_addr()
***************************************************************************************************
* Function Description:
* @brief
* This function set central adv address.
* @param[in] addr, app set central address.
* @return    void.
**************************************************************************************************/
void pawr_set_central_addr(const uint8_t *addr)
{
    memcpy(pawr_central_address,addr,6);
    printf("pawr_set_central_addr: ");
    app_bt_util_print_bd_address(pawr_central_address);
}

/**************************************************************************************************
* Function Name: pawr_init()
***************************************************************************************************
* Function Description:
* @brief
* This function initialize pwar central.
* @param[in]       void.
* @return          void.
**************************************************************************************************/
void pawr_init(void)
{
    wiced_bt_ble_observe(WICED_FALSE, 0, NULL);
    wiced_ble_ext_adv_register_cback(pawr_ext_adv_callback);
    pawr_create_pawr_network();
}

/* [] END OF FILE */

