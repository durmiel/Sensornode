#ifndef I2C_MASTER_H
#define I2C_MASTER_H

#include <global.h>
#include <driver/i2c.h>

/** ***************************************************************************
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
Status_t I2C_master_init(Failure_t* failure);

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
Status_t I2cMaster_read(Failure_t* failure, i2c_port_t i2c_num, uint8_t slave_address, uint8_t *data_rd, size_t size);

Status_t I2cMaster_write(Failure_t* failure, i2c_port_t i2c_num, uint8_t slave_address, uint8_t *data_wr, size_t size);

#endif // I2C_MASTER_H
