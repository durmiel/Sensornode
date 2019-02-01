
#include <bt.h>

esp_bt_controller_config_t bt_config;


/** ***************************************************************************
 *  @brief  initialization of the bluetooth controller
 *  @param[in]  failure - pointer to the structure which is used for the
 *              error tracking
 *  @retval SUCCESS         - successful execution of the function
 *  @retval LIB_ERROR       - an error occur during using a library function
 *****************************************************************************/
Status_t Bluetooth_init(Failure_t* failure) {
    esp_err_t ret;

    // Initialize NVS - non volatile storage
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ret = nvs_flash_erase();
        if (ret != ESP_OK) {
            failure->status = LIB_ERROR;
            failure->failure_code = ret;
            failure->fn_pointer = (uint32_t*) Bluetooth_init;
            return LIB_ERROR;
        }
        ret = nvs_flash_init();
        if (ret != ESP_OK) {
            failure->status = LIB_ERROR;
            failure->failure_code = ret;
            failure->fn_pointer = (uint32_t*) Bluetooth_init;
            return LIB_ERROR;
        }
    }
    // initialize the bt controller, must be called before the esp_bt_controller_enable
    // function is called, other modes are ESP_BT_MODE_BLE, ESP_BT_MODE_BTDM
    ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    bt_config = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    ret = esp_bt_controller_init(&bt_config);
    if (ret != ESP_OK) {
        failure->status = LIB_ERROR;
        failure->failure_code = ret;
        failure->fn_pointer = (uint32_t*) Bluetooth_init;
        return LIB_ERROR;
    }
    // enable the controller
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
        failure->status = LIB_ERROR;
        failure->failure_code = ret;
        failure->fn_pointer = (uint32_t*) Bluetooth_init;
        return LIB_ERROR;
    }
    // bluedroid is the controller which is connected to the bluetooth
    // controller via an virtual host control interface. the bluedroid must
    // be enabled in the menu configuration
    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        failure->status = LIB_ERROR;
        failure->failure_code = ret;
        failure->fn_pointer = (uint32_t*) Bluetooth_init;
        return LIB_ERROR;
    }
    // Enable the bluedroid application
    
}
