
#include <bt.h>


/** ***************************************************************************
 *  @brief  initialization of the bluetooth controller
 *  @param[in]  failure - pointer to the structure which is used for the
 *              error tracking
 *  @retval SUCCESS         - successful execution of the function
 *  @retval LIB_ERROR       - an error occur during using a library function
 *****************************************************************************/
Status_t Bluetooth_init(Failure_t* failure) {
    esp_err_t ret;

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );





}
