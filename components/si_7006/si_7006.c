
#include <si_7006.h>
#include <driver/i2c.h>

/** ***************************************************************************
 * Define the commands of the SI 7006
 *****************************************************************************/
#define READ_HUMIDITY_HOLD_MASTER               0xE5
#define READ_HUMIDITY_NO_HOLD_MASTER            0xF5
#define READ_TEMPERATURE_HOLD_MASTER            0xE3
#define READ_TEMPERATURE_NO_HOLD_MASTER         0xF3
#define RESET                                   0xFE
#define WRITE_HEATER_CONTROL_REG                0x51
#define READ_HEATER_CONTROL_REG                 0x11
#define READ_ELECTRONIC_ID_1ST_0                0xFA
#define READ_ELECTRONIC_ID_1ST_1                0x0F
#define READ_ELECTRONIC_ID_2ND_0                0xFC
#define READ_ELECTRONIC_ID_2ND_1                0xC9
#define READ_FIRMWARE_REVISION_0                0x84
#define READ_FIRMWARE_REVISION_1                0xB8

/** ***************************************************************************
 * Define the slave address of the SI 7006
 *****************************************************************************/
#define SI_7006_ADDR                            0x40

/** ***************************************************************************
 * Defines for the pinning, the used freuqency and the buffer size
 *****************************************************************************/
#define I2C_MASTER_SCL_IO                   19                                        /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO                   18                                        /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM                      1                                         /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ                  100000                                    /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE           0                                         /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE           0                                         /*!< I2C master doesn't need buffer */

#define WRITE_BIT                           I2C_MASTER_WRITE                          /*!< I2C master write */
#define READ_BIT                            I2C_MASTER_READ                           /*!< I2C master read */
#define ACK_CHECK_EN                        0x1                                       /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                       0x0                                       /*!< I2C master will not check ack from slave */
#define ACK_VAL                             0x0                                       /*!< I2C ack value */
#define NACK_VAL                            0x1                                       /*!< I2C nack value */

#define DATA_LENGTH                         512                                       /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH                      128                                       /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS         1000                                      /*!< delay time between different test items */

/** ***************************************************************************
 *  @fn     Si7006_init(Failure_t* failure)
 *  @brief  i2c master initialization
 *          The master 1 will be set as master. The frequency is set to 100kHz
 *          and the used pins are
 *              SDA - GPIO 18
 *              SCL - GPIO 19
 *  @param[in]  failure - pointer to the structure which is used for the
 *              error tracking
 *  @retval SUCCESS         - successful execution of the function
 *  @retval LIB_ERROR       - an error occur during using a library function
 *****************************************************************************/
Status_t Si7006_init(Failure_t* failure) {
    int i2c_master_port;
    i2c_config_t conf;
    esp_err_t ret;

    // setup configuration sturcture and master port
    i2c_master_port       = I2C_MASTER_NUM;
    conf.mode             = I2C_MODE_MASTER;
    conf.sda_io_num       = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.scl_io_num       = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    i2c_param_config(i2c_master_port, &conf);
    ret = i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (ret != ESP_OK) {
        failure->status = LIB_ERROR;
        failure->failure_code = ret;
        failure->fn_pointer = (uint32_t*) Si7006_init;
        return LIB_ERROR;
    }

    return SUCCESS;
}

/** ***************************************************************************
 *  @fn     Si7006_readFirmwareVersion(Failure_t* failure, uint8_t* firmware_version)
 *  @brief  Read the firmware version out of the SI 7006 sensor.
 *  @param[in]  failure - pointer to the structure which is used for the
 *              error tracking
 *  @param[out] firmware_version - pointer to the variable which stores
 *              the read firmware version
 *  @retval SUCCESS         - successful execution of the function
 *  @retval LIB_ERROR       - an error occur during using a library function
 *  @retval INTERNAL_ERROR  - an error occur during using an internal function
 *****************************************************************************/
Status_t Si7006_readFirmwareVersion(Failure_t* failure, uint8_t* firmware_version) {
    i2c_cmd_handle_t i2c_handle;
    esp_err_t ret;
    
    // write the command to read the firware version
    i2c_handle = i2c_cmd_link_create();
    i2c_master_start(i2c_handle);
    i2c_master_write_byte(i2c_handle, (SI_7006_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(i2c_handle, READ_FIRMWARE_REVISION_0, ACK_CHECK_EN);
    i2c_master_write_byte(i2c_handle, READ_FIRMWARE_REVISION_1, ACK_CHECK_EN);
    i2c_master_stop(i2c_handle);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, i2c_handle, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(i2c_handle);
    if (ret != ESP_OK) {
        failure->status = LIB_ERROR;
        failure->failure_code = ret;
        failure->fn_pointer = (uint32_t*) Si7006_readFirmwareVersion;
        return LIB_ERROR;
    }
    vTaskDelay(1 / portTICK_RATE_MS);

    // read the firmware version
    i2c_handle = i2c_cmd_link_create();
    i2c_master_start(i2c_handle);
    i2c_master_write_byte(i2c_handle, (SI_7006_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(i2c_handle, firmware_version, ACK_CHECK_DIS);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, i2c_handle, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(i2c_handle);
    if (ret != ESP_OK) {
        failure->status = LIB_ERROR;
        failure->failure_code = ret;
        failure->fn_pointer = (uint32_t*) Si7006_readFirmwareVersion;
        return LIB_ERROR;
    }
    return SUCCESS;
}

/** ***************************************************************************
 *  @fn     Si7006_readElectronicId(Failure_t* failure, uint32_t* serial_nr_a, uint32_t* serial_nr_b)
 *  @brief  Read the electronic id out of the SI 7006 sensor
 *  @param[in]  failure - pointer to the structure which is used for the
 *              error tracking
 *  @param[out] serial_nr_a - pointer which store the first 4 bytes of the 
 *                            electronic id
 *  @param[out] serial_nr_b - pointer which store the second 4 bytes of the 
 *                            electronic id
 *  @retval SUCCESS         - successful execution of the function
 *  @retval LIB_ERROR       - an error occur during using a library function
 *  @retval INTERNAL_ERROR  - an error occur during using an internal function
 *****************************************************************************/
Status_t Si7006_readElectronicId(Failure_t* failure, uint32_t* serial_nr_a, uint32_t* serial_nr_b) {
    i2c_cmd_handle_t i2c_handle;
    esp_err_t ret;
    uint8_t buf[8];
    uint8_t i;

    // write the command to read electronic id first part
    i2c_handle = i2c_cmd_link_create();
    i2c_master_start(i2c_handle);
    i2c_master_write_byte(i2c_handle, (SI_7006_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(i2c_handle, READ_ELECTRONIC_ID_1ST_0, ACK_CHECK_EN);
    i2c_master_write_byte(i2c_handle, READ_ELECTRONIC_ID_1ST_1, ACK_CHECK_EN);
    i2c_master_stop(i2c_handle);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, i2c_handle, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(i2c_handle);
    if (ret != ESP_OK) {
        failure->status = LIB_ERROR;
        failure->failure_code = ret;
        failure->fn_pointer = (uint32_t*) Si7006_readFirmwareVersion;
        return LIB_ERROR;
    }
    vTaskDelay(1 / portTICK_RATE_MS);

    // read the electronic id first part
    i2c_handle = i2c_cmd_link_create();
    i2c_master_start(i2c_handle);
    i2c_master_write_byte(i2c_handle, (SI_7006_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    for (i = 0; i < 7; ++i) {
        i2c_master_read_byte(i2c_handle, &buf[i], ACK_CHECK_EN);
    }
    i2c_master_read_byte(i2c_handle, &buf[7], ACK_CHECK_DIS);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, i2c_handle, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(i2c_handle);
    if (ret != ESP_OK) {
        failure->status = LIB_ERROR;
        failure->failure_code = ret;
        failure->fn_pointer = (uint32_t*) Si7006_readFirmwareVersion;
        return LIB_ERROR;
    }

    *serial_nr_a = (buf[0] << 24) + (buf[2] << 16) + (buf[4] << 8) + (buf[6] << 0);

    // write the command to read electronic id second part
    i2c_handle = i2c_cmd_link_create();
    i2c_master_start(i2c_handle);
    i2c_master_write_byte(i2c_handle, (SI_7006_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(i2c_handle, READ_ELECTRONIC_ID_2ND_0, ACK_CHECK_EN);
    i2c_master_write_byte(i2c_handle, READ_ELECTRONIC_ID_2ND_1, ACK_CHECK_EN);
    i2c_master_stop(i2c_handle);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, i2c_handle, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(i2c_handle);
    if (ret != ESP_OK) {
        failure->status = LIB_ERROR;
        failure->failure_code = ret;
        failure->fn_pointer = (uint32_t*) Si7006_readFirmwareVersion;
        return LIB_ERROR;
    }
    vTaskDelay(1 / portTICK_RATE_MS);

    // read the electronic id second part
    i2c_handle = i2c_cmd_link_create();
    i2c_master_start(i2c_handle);
    i2c_master_write_byte(i2c_handle, (SI_7006_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    for (i = 0; i < 7; ++i) {
        i2c_master_read_byte(i2c_handle, &buf[i], ACK_CHECK_EN);
    }
    i2c_master_read_byte(i2c_handle, &buf[7], ACK_CHECK_DIS);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, i2c_handle, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(i2c_handle);
    if (ret != ESP_OK) {
        failure->status = LIB_ERROR;
        failure->failure_code = ret;
        failure->fn_pointer = (uint32_t*) Si7006_readFirmwareVersion;
        return LIB_ERROR;
    }

    *serial_nr_b = (buf[0] << 24) + (buf[2] << 16) + (buf[4] << 8) + (buf[6] << 0);

    return SUCCESS;
}

/** ***************************************************************************
 *  @fn     Si7006_readHumidity(Failure_t* failure, uint8_t* humidity, SI7006_READ_MODES_t mode)
 *  @brief  Measure the humidity and read the value out of the sensor
 *  @param[in]  failure - pointer to the structure which is used for the
 *              error tracking
 *  @param[out] humidity - pointer to a variable which stores the read humidity
 *  @param[in]  mode - mode which should be used for the reading.
 *  @retval SUCCESS         - successful execution of the function
 *  @retval LIB_ERROR       - an error occur during using a library function
 *  @retval INTERNAL_ERROR  - an error occur during using an internal function
 *****************************************************************************/
Status_t Si7006_readHumidity(Failure_t* failure, uint8_t* humidity, SI7006_READ_MODES_t mode) {
    return SUCCESS;
}

/** ***************************************************************************
 *  @fn     Si7006_readTemperature(Failure_t* failure, uint8_t* temperature, SI7006_READ_MODES_t mode)
 *  @brief  Measure the temperature and read the value out of the sensor
 *  @param[in]  failure - pointer to the structure which is used for the
 *              error tracking
 *  @param[out] temperature - pointer to a variable which stores the read temperature
 *  @param[in]  mode - mode which should be used for the reading.
 *  @retval SUCCESS         - successful execution of the function
 *  @retval LIB_ERROR       - an error occur during using a library function
 *  @retval INTERNAL_ERROR  - an error occur during using an internal function
 *****************************************************************************/
Status_t Si7006_readTemperature(Failure_t* failure, uint8_t* temperature, SI7006_READ_MODES_t mode) {
    return SUCCESS;
}

/** ***************************************************************************
 *  @fn     Si7006_readHeaterConfig(Failure_t* failure, uint8_t* heater_config)
 *  @brief  Read the heater configuration of the SI 7006
 *  @param[in]  failure - pointer to the structure which is used for the
 *              error tracking
 *  @param[out] heater_config - pointer to the variable which stores
 *              the read heater configuration
 *  @retval SUCCESS         - successful execution of the function
 *  @retval LIB_ERROR       - an error occur during using a library function
 *  @retval INTERNAL_ERROR  - an error occur during using an internal function
 *****************************************************************************/
Status_t Si7006_readHeaterConfig(Failure_t* failure, uint8_t* heater_config) {
    return SUCCESS;
}

/** ***************************************************************************
 *  @fn     Si7006_writeHeaterConfig(Failure_t* failure, uint8_t heater_config)
 *  @brief  Write the heater configuration of the SI 7006
 *  @param[in]  failure - pointer to the structure which is used for the
 *              error tracking
 *  @param[out] heater_config - contains the new heater configuration
 *  @retval SUCCESS         - successful execution of the function
 *  @retval LIB_ERROR       - an error occur during using a library function
 *  @retval INTERNAL_ERROR  - an error occur during using an internal function
 *****************************************************************************/
Status_t Si7006_writeHeaterConfig(Failure_t* failure, uint8_t heater_config) {
    return SUCCESS;
}

/** ***************************************************************************
 *  @fn     Si7006_readUserRegister(Failure_t* failure, uint8_t* user_config)
 *  @brief  Read the user configuration register of the SI 7006
 *  @param[in]  failure - pointer to the structure which is used for the
 *              error tracking
 *  @param[out] user_config - pointer to the variable which stores
 *              the read user register configuration
 *  @retval SUCCESS         - successful execution of the function
 *  @retval LIB_ERROR       - an error occur during using a library function
 *  @retval INTERNAL_ERROR  - an error occur during using an internal function
 *****************************************************************************/
Status_t Si7006_readUserRegister(Failure_t* failure, uint8_t* user_config) {
    return SUCCESS;
}

/** ***************************************************************************
 *  @fn     Si7006_writeUserRegister(Failure_t* failure, uint8_t user_config)
 *  @brief  Write the user configuration register of the SI 7006
 *  @param[in]  failure - pointer to the structure which is used for the
 *              error tracking
 *  @param[out] user_config - contains the new user register configuration
 *  @retval SUCCESS         - successful execution of the function
 *  @retval LIB_ERROR       - an error occur during using a library function
 *  @retval INTERNAL_ERROR  - an error occur during using an internal function
 *****************************************************************************/
Status_t Si7006_writeUserRegister(Failure_t* failure, uint8_t user_config) {
    return SUCCESS;
}
