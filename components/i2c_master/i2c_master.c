
#include <global.h>
#include <i2c_master.h>
#include "driver/i2c.h"

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
 *  @fn     I2cMaster_init(Failure_t* failure)   
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
Status_t I2cMaster_init(Failure_t* failure) {
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
        failure->fn_pointer = (uint32_t*) I2cMaster_init;
        return LIB_ERROR;
    }

    return SUCCESS;
}

/** ***************************************************************************
 *  @fn     I2cMaster_read(Failure_t* failure, i2c_port_t i2c_num, uint8_t slave_address, uint8_t *data_rd, size_t size)
 *  @brief  I2C master read out of the slave some data. The number of data, the
 *          slave address are configurable
 *  _______________________________________________________________________________________
 *  | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 *  --------|--------------------------|----------------------|--------------------|------|
 * 
 *  @param[in]  failure - pointer to the structure which is used for the
 *              error tracking
 *  @param[in]  i2c_num - selection of the wished i2c master number. The number
 *              will be configured during initialization
 *  @param[in]  slave_address - address of the slave normally the address has
 *              a width of 7-bit
 *  @param[out] data_rd - buffer in which the read data will be stored
 *  @param[in]  size    - number of bytes to read
 *  @retval SUCCESS         - successful execution of the function
 *  @retval LIB_ERROR       - an error occur during using a library function
 *  @retval INTERNAL_ERROR  - an internal error occur during checking the parameter
 *****************************************************************************/
Status_t I2cMaster_read(Failure_t* failure, I2cCommand_t* cmd,  i2c_port_t i2c_num, uint8_t slave_address, uint8_t *data_rd, size_t size) {
    uint8_t i;
    //TODO: check the command structure

    for (i = 0; i < cmd->cmd_size; ++i) {
        i2c_master_write_byte(i2c_handle, cmd->cmd_buf[i], ACK_CHECK_EN);
    }
    

    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */
Status_t I2cMaster_write(Failure_t* failure, i2c_port_t i2c_num, uint8_t slave_address, uint8_t *data_wr, size_t size) {
        // Do a size check
    if (size == 0) {
        failure->status = INTERNAL_ERROR;
        failure->failure_code = SIZE_ERROR;
        failure->fn_pointer = (uint32_t*) I2cMaster_write;
        return INTERNAL_ERROR;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slave_address << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1);
    i2c_cmd_link_delete(cmd);
    return ret;
}
