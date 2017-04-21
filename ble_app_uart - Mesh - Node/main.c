/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "nrf_ble_gatt.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "bsp_btn_ble.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define CONN_CFG_TAG                    1                                           /**< A tag that refers to the BLE stack configuration we set with @ref sd_ble_cfg_set. Default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define DEVICE_NAME                     "Nordic_UART"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */


//===MESH==============
#include "rbc_mesh.h"
#define MESH_ACCESS_ADDR        (RBC_MESH_ACCESS_ADDRESS_BLE_ADV)   /**< Access address for the mesh to operate on. */
#define MESH_INTERVAL_MIN_MS    (100)                               /**< Mesh minimum advertisement interval in milliseconds. */
#define MESH_CHANNEL            (38)                                /**< BLE channel to operate on. Single channel only. */
#define COMMISSION_HANDLE        0
#define SENSOR_MAX_LEN             22
#if defined BOARD_PCA10056 

#define LED_START 							13

#endif
//nrf_nvic_state_t nrf_nvic_state = {0};
static nrf_clock_lf_cfg_t m_clock_cfg = 
{
    .source = NRF_CLOCK_LF_SRC_XTAL,    
    .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_75_PPM
};

#define MESH_CLOCK_SOURCE       (m_clock_cfg)    /**< Clock source used by the Softdevice. For calibrating timeslot time. */


#define EXAMPLE_DFU_BANK_ADDR   (0x40000)

/**@brief Operation states.
 * IDs that uniquely identify a GAP option.
 */
enum OPERATION_STATES
{
  UNASSIGNED_ID_STATE  = 0,       /**< Channel Map. @ref ble_gap_opt_ch_map_t  */
  REQUESTING_ID_STATE,               /**< Local connection latency. @ref ble_gap_opt_local_conn_latency_t */
  NORMAL_OP_STATE
};

/**@brief Commissioning Opcode.
 * IDs that uniquely identify a GAP option.
 */
enum COMMISSIONING_OPCODES
{
  PING  = 0,       /**< Channel Map. @ref ble_gap_opt_ch_map_t  */
  REQUEST_HANDLE_OPCODE,               /**< Local connection latency. @ref ble_gap_opt_local_conn_latency_t */
  ASSIGN_HANDLE_OPCODE
};



APP_TIMER_DEF(m_one_sec_timer_id);                        /**< Battery timer. */
#define ONE_SECOND_INTERVAL      APP_TIMER_TICKS(1000)                      /**< Battery level measurement interval (ticks). */
uint16_t Handle_ID;
uint8_t current_state					=UNASSIGNED_ID_STATE;
uint16_t CHIP_ID ;

//======================
uint8_t sensor_data[23];
uint8_t button_value=0;
#define MAX_NODE 10
/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}









/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = softdevice_app_ram_start_get(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Overwrite some of the default configurations for the BLE stack.
    ble_cfg_t ble_cfg;

    // Configure the maximum number of connections.
    memset(&ble_cfg, 0, sizeof(ble_cfg));
    ble_cfg.gap_cfg.role_count_cfg.periph_role_count  = BLE_GAP_ROLE_COUNT_PERIPH_DEFAULT;
    ble_cfg.gap_cfg.role_count_cfg.central_role_count = 0;
    ble_cfg.gap_cfg.role_count_cfg.central_sec_count  = 0;
    err_code = sd_ble_cfg_set(BLE_GAP_CFG_ROLE_COUNT, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);

    // Configure the maximum ATT MTU.
    memset(&ble_cfg, 0x00, sizeof(ble_cfg));
    ble_cfg.conn_cfg.conn_cfg_tag                 = CONN_CFG_TAG;
    ble_cfg.conn_cfg.params.gatt_conn_cfg.att_mtu = NRF_BLE_GATT_MAX_MTU_SIZE;
    err_code = sd_ble_cfg_set(BLE_CONN_CFG_GATT, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);

    // Configure the maximum event length.
    memset(&ble_cfg, 0x00, sizeof(ble_cfg));
    ble_cfg.conn_cfg.conn_cfg_tag                     = CONN_CFG_TAG;
    ble_cfg.conn_cfg.params.gap_conn_cfg.event_length = 320;
    ble_cfg.conn_cfg.params.gap_conn_cfg.conn_count   = BLE_GAP_CONN_COUNT_DEFAULT;
    err_code = sd_ble_cfg_set(BLE_CONN_CFG_GAP, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);


		
    // Enable BLE stack.
    err_code = softdevice_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Subscribe for BLE events.
    //err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    //APP_ERROR_CHECK(err_code);
}



/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\r') || (index >= (SENSOR_MAX_LEN)))
            {
                NRF_LOG_DEBUG("Ready to send\r\n");
                NRF_LOG_HEXDUMP_DEBUG(data_array, index);
                memcpy(&sensor_data[1],data_array,SENSOR_MAX_LEN);
                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [LED Initialization] */
void leds_buttons_init(void)
{
        nrf_gpio_cfg_output(LED_1);
        nrf_gpio_cfg_output(LED_2);
        nrf_gpio_cfg_output(LED_4);
        nrf_gpio_cfg_output(LED_3);
        nrf_gpio_pin_set(LED_1);
        nrf_gpio_pin_set(LED_2);
        nrf_gpio_pin_set(LED_3);
        nrf_gpio_pin_set(LED_4);
        nrf_gpio_cfg_input(BUTTON_1, NRF_GPIO_PIN_PULLUP);
        nrf_gpio_cfg_input(BUTTON_2, NRF_GPIO_PIN_PULLUP);
        nrf_gpio_cfg_input(BUTTON_3, NRF_GPIO_PIN_PULLUP);
        nrf_gpio_cfg_input(BUTTON_4, NRF_GPIO_PIN_PULLUP);
}



/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
}


/**
* @brief RBC_MESH framework event handler. Defined in rbc_mesh.h. Handles
*   events coming from the mesh. Sets LEDs according to data
*
* @param[in] evt RBC event propagated from framework
*/
static void rbc_mesh_event_handler(rbc_mesh_event_t* p_evt)
{   uint32_t error_code;
    switch (p_evt->type)
    {
        case RBC_MESH_EVENT_TYPE_CONFLICTING_VAL:
						NRF_LOG_INFO("\r\nCONFLITING VAL!\r\n");
        case RBC_MESH_EVENT_TYPE_NEW_VAL:
						NRF_LOG_INFO("\r\nNEW VAL!\r\n");
						
        case RBC_MESH_EVENT_TYPE_UPDATE_VAL:
						NRF_LOG_INFO("\r\nUpdate VAL!\r\n");
            break;

        case RBC_MESH_EVENT_TYPE_TX:
            break;

        case RBC_MESH_EVENT_TYPE_INITIALIZED:
					  /* init BLE gateway softdevice application: */
            //nrf_adv_conn_init();
            break;
        default:
            break;
    }
	switch (p_evt->params.rx.value_handle)
	{
			
		case COMMISSION_HANDLE:
            if (current_state==UNASSIGNED_ID_STATE) //Commision handle version is udpated, start requesting Handle_ID
            {
                current_state=REQUESTING_ID_STATE;
                printf("\r\nState now is REQUESTING_ID_STATE!\r\n");
                break; 
            }  
            if ( p_evt->params.rx.p_data[0]==ASSIGN_HANDLE_OPCODE)
            {
                uint16_t chip_id= ((uint16_t)p_evt->params.rx.p_data[1] << 8) | p_evt->params.rx.p_data[2];
                
                if (chip_id==CHIP_ID){ //Handle ID is assigned, Switch to normal operation
                    Handle_ID=((uint16_t)p_evt->params.rx.p_data[3] << 8) | p_evt->params.rx.p_data[4];
                    current_state=NORMAL_OP_STATE;
                    error_code= rbc_mesh_value_enable(Handle_ID);
                    APP_ERROR_CHECK(error_code);

                    printf("\r\nState now is NORMAL_OP! Handle= %d\r\n",Handle_ID);
                }
                    
            }
            break;
        default:
            break;
            
    } 
                
}
/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    ret_code_t err_code;

    // Start application timers.
    err_code = app_timer_start(m_one_sec_timer_id, ONE_SECOND_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

   }


/**@brief Function for handling the one second timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void one_sec_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
	uint8_t mesh_data[3];
    uint32_t error_code=0;
	if (button_value==0){
        nrf_gpio_pin_toggle(LED_1);
    }else {
        if (button_value==1){                      
            nrf_gpio_pin_clear(LED_1);
        }else{
            nrf_gpio_pin_set(LED_1);
            
        }
    }
        
    switch (current_state)
    {
        case UNASSIGNED_ID_STATE:
		 
            printf("\r\nSending COMMISSION HANDLE Enable\r\n");
			error_code = rbc_mesh_value_enable(COMMISSION_HANDLE);
            break;
		
		case NORMAL_OP_STATE:
			if (button_value) 
            {
                sensor_data[0]=button_value;
            }else{
                sensor_data[0]=!nrf_gpio_pin_out_read(LED_1);
            }
		    error_code= rbc_mesh_value_set(Handle_ID,sensor_data,23);
            break;
		case REQUESTING_ID_STATE:
			mesh_data[0]=REQUEST_HANDLE_OPCODE;
			mesh_data[1]=CHIP_ID>>8;
			mesh_data[2]=(uint8_t)CHIP_ID;
			error_code=rbc_mesh_value_set(COMMISSION_HANDLE,mesh_data,3);
            APP_ERROR_CHECK(error_code);
            break;
        default:
            break;
    }
}
/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    ret_code_t err_code;

    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.
		err_code = app_timer_create(&m_one_sec_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                one_sec_timeout_handler);
    APP_ERROR_CHECK(err_code);

}


/**@brief Application main function.
 */
int main(void)
{
    uint32_t err_code;
    rbc_mesh_event_t evt;
	leds_buttons_init();
    //Assign Unique CHIP_ID
    CHIP_ID = (uint16_t)NRF_FICR->DEVICEID[0];
    // Initialize.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
	timers_init();
    uart_init();
    log_init();

    //buttons_leds_init(&erase_bonds);
    ble_stack_init();
   // gap_params_init();
   // gatt_init();
   // services_init();
   // advertising_init();
   // conn_params_init();

		rbc_mesh_init_params_t init_params;

    init_params.access_addr = MESH_ACCESS_ADDR;
    init_params.interval_min_ms = MESH_INTERVAL_MIN_MS;
    init_params.channel = MESH_CHANNEL;
    init_params.lfclksrc = MESH_CLOCK_SOURCE;
    init_params.tx_power = RBC_MESH_TXPOWER_0dBm ;
    
    uint32_t error_code = rbc_mesh_init(init_params);
     /* enable handle ID for all nodes*/
    for (uint32_t i = 0; i < MAX_NODE; ++i)
    {
        err_code = rbc_mesh_value_enable(i);
        APP_ERROR_CHECK(err_code);
    }   
    APP_ERROR_CHECK(error_code);
		timers_init();
    application_timers_start();
    //current_state= REQUESTING_ID_STATE;
	//Enable Commission handle
    printf("\r\nUART Start!\r\n");
    NRF_LOG_INFO("UART Start!\r\n");
   // err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
   // APP_ERROR_CHECK(err_code);

    // Enter main loop.
    for (;;)
    {
        if (rbc_mesh_event_get(&evt) == NRF_SUCCESS)
        {   
            rbc_mesh_event_handler(&evt);
            rbc_mesh_event_release(&evt);
        }
        if(nrf_gpio_pin_read(BUTTON_1) == 0){
            button_value=1;            
            nrf_gpio_pin_clear(LED_1);

        }
         if(nrf_gpio_pin_read(BUTTON_2) == 0){
            button_value=2;
            nrf_gpio_pin_set(LED_1);

        }
         if(nrf_gpio_pin_read(BUTTON_3) == 0){
            button_value=0;
        

        }
                
           
        
    }
}


/**
 * @}
 */
