
#include <bluetooth.h>

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"


esp_bt_controller_config_t bt_config;

static Status_t bluetooth_init_process(Failure_t* failure);
static Status_t bluetooth_init_and_enable_controller(Failure_t* failure);
static Status_t bluetooth_init_and_enable_host(Failure_t* failure);
static Status_t bluetooth_init_gatts(Failure_t* failure);

static void bluetooth_gatts_event_cb(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void bluetooth_gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

/** ***************************************************************************
 *  @brief  initialization of the bluetooth controller
 *  @param[in]  failure - pointer to the structure which is used for the
 *              error tracking
 *  @retval SUCCESS         - successful execution of the function
 *  @retval LIB_ERROR       - an error occur during using a library function
 *****************************************************************************/
Status_t Bluetooth_init(Failure_t* failure) {
    Status_t ret;

    ret = bluetooth_init_process(failure);
    if (ret != SUCCESS) {
        return ret;
    }
    ret = bluetooth_init_and_enable_controller(failure);
    if (ret != SUCCESS) {
        return ret;
    }
    ret = bluetooth_init_and_enable_host(failure);
    if (ret != SUCCESS) {
        return ret;
    }
    ret = bluetooth_init_gatts(failure);
    if (ret != SUCCESS) {
        return ret;
    }

    return SUCCESS;
}

static Status_t bluetooth_init_process(Failure_t* failure) {
    esp_err_t ret;

    // Initialize NVS - non volatile storage
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ret = nvs_flash_erase();
        if (ret != ESP_OK) {
            failure->status = LIB_ERROR;
            failure->failure_code = ret;
            failure->fn_pointer = (uint32_t*) bluetooth_init_process;
            return LIB_ERROR;
        }
        ret = nvs_flash_init();
        if (ret != ESP_OK) {
            failure->status = LIB_ERROR;
            failure->failure_code = ret;
            failure->fn_pointer = (uint32_t*) bluetooth_init_process;
            return LIB_ERROR;
        }
    }
    return SUCCESS;
}

static Status_t bluetooth_init_and_enable_controller(Failure_t* failure) {
    esp_err_t ret;

    // initialize the bt controller, must be called before the esp_bt_controller_enable
    // function is called, other modes are ESP_BT_MODE_BLE, ESP_BT_MODE_BTDM
    ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    bt_config = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    ret = esp_bt_controller_init(&bt_config);
    if (ret != ESP_OK) {
        failure->status = LIB_ERROR;
        failure->failure_code = ret;
        failure->fn_pointer = (uint32_t*) bluetooth_init_and_enable_controller;
        return LIB_ERROR;
    }
    // enable the controller
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
        failure->status = LIB_ERROR;
        failure->failure_code = ret;
        failure->fn_pointer = (uint32_t*) bluetooth_init_and_enable_controller;
        return LIB_ERROR;
    }
    return SUCCESS;
}

static Status_t bluetooth_init_and_enable_host(Failure_t* failure) {
    esp_err_t ret;

    // bluedroid is the controller which is connected to the bluetooth
    // controller via an virtual host control interface. the bluedroid must
    // be enabled in the menu configuration
    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        failure->status = LIB_ERROR;
        failure->failure_code = ret;
        failure->fn_pointer = (uint32_t*) bluetooth_init_and_enable_host;
        return LIB_ERROR;
    }
    // Enable the bluedroid application
    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        failure->status = LIB_ERROR;
        failure->failure_code = ret;
        failure->fn_pointer = (uint32_t*) bluetooth_init_and_enable_host;
    }
    return SUCCESS;
}

static Status_t bluetooth_init_gatts(Failure_t* failure) {
    esp_err_t ret;
    esp_err_t local_mtu_ret;

    // register gatts event callback
    ret = esp_ble_gatts_register_callback(bluetooth_gatts_event_cb);
    if (ret != ESP_OK) {
        failure->status = LIB_ERROR;
        failure->failure_code = ret;
        failure->fn_pointer = (uint32_t*) bluetooth_init_gatts;
        return LIB_ERROR;
    }
    ret = esp_ble_gap_register_callback(bluetooth_gap_event_handler);
    if (ret != ESP_OK) {
        failure->status = LIB_ERROR;
        failure->failure_code = ret;
        failure->fn_pointer = (uint32_t*) bluetooth_init_gatts;
        return LIB_ERROR;
    }
    ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
    if (ret){
        failure->status = LIB_ERROR;
        failure->failure_code = ret;
        failure->fn_pointer = (uint32_t*) bluetooth_init_gatts;
        return LIB_ERROR;
    }
    ret = esp_ble_gatts_app_register(PROFILE_B_APP_ID);
    if (ret){
        failure->status = LIB_ERROR;
        failure->failure_code = ret;
        failure->fn_pointer = (uint32_t*) bluetooth_init_gatts;
        return LIB_ERROR;
    }
    local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        failure->status = LIB_ERROR;
        failure->failure_code = ret;
        failure->fn_pointer = (uint32_t*) bluetooth_init_gatts;
        return LIB_ERROR;
    }
    return SUCCESS;
}
