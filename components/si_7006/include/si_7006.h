
#ifndef SI_7006_H
#define SI_7006_H

#include <stdint.h>
#include <global.h>

/** ***************************************************************************
 * Enumeration of the read mode. In read mode "HOLD" the master waits for the
 * answer of the SI 7006. In "NO_HOLD" the value of the measurement will be
 * read later
 *****************************************************************************/
typedef enum SI7006_READ_MODES {
    HOLD,                           /*!< Use the hold functionality and wait for the answer of the SI 7006 */
    NO_HOLD                         /*!< Use the hold functionality and don't wait for the answer of the SI 7006 */
}SI7006_READ_MODES_t;

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
Status_t Si7006_init(Failure_t* failure);

/** ***************************************************************************
 *  @brief  Read the firmware version out of the SI 7006 sensor.
 *  @param[in]  failure - pointer to the structure which is used for the
 *              error tracking
 *  @param[out] firmware_version - pointer to the variable which stores
 *              the read firmware version
 *  @retval SUCCESS         - successful execution of the function
 *  @retval LIB_ERROR       - an error occur during using a library function
 *  @retval INTERNAL_ERROR  - an error occur during using an internal function
 *****************************************************************************/
Status_t Si7006_readFirmwareVersion(Failure_t* failure, uint8_t* firmware_version);

/** ***************************************************************************
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
Status_t Si7006_readElectronicId(Failure_t* failure, uint32_t* serial_nr_a, uint32_t* serial_nr_b);

/** ***************************************************************************
 *  @brief  Measure the humidity and read the value out of the sensor
 *  @param[in]  failure - pointer to the structure which is used for the
 *              error tracking
 *  @param[out] humidity - pointer to a variable which stores the read humidity
 *  @param[in]  mode - mode which should be used for the reading.
 *  @retval SUCCESS         - successful execution of the function
 *  @retval LIB_ERROR       - an error occur during using a library function
 *  @retval INTERNAL_ERROR  - an error occur during using an internal function
 *****************************************************************************/
Status_t Si7006_readHumidity(Failure_t* failure, uint8_t* humidity, SI7006_READ_MODES_t mode);

/** ***************************************************************************
 *  @brief  Measure the temperature and read the value out of the sensor
 *  @param[in]  failure - pointer to the structure which is used for the
 *              error tracking
 *  @param[out] temperature - pointer to a variable which stores the read temperature
 *  @param[in]  mode - mode which should be used for the reading.
 *  @retval SUCCESS         - successful execution of the function
 *  @retval LIB_ERROR       - an error occur during using a library function
 *  @retval INTERNAL_ERROR  - an error occur during using an internal function
 *****************************************************************************/
Status_t Si7006_readTemperature(Failure_t* failure, uint8_t* temperature, SI7006_READ_MODES_t mode);

/** ***************************************************************************
 *  @brief  Read the heater configuration of the SI 7006
 *  @param[in]  failure - pointer to the structure which is used for the
 *              error tracking
 *  @param[out] heater_config - pointer to the variable which stores
 *              the read heater configuration
 *  @retval SUCCESS         - successful execution of the function
 *  @retval LIB_ERROR       - an error occur during using a library function
 *  @retval INTERNAL_ERROR  - an error occur during using an internal function
 *****************************************************************************/
Status_t Si7006_readHeaterConfig(Failure_t* failure, uint8_t* heater_config);

/** ***************************************************************************
 *  @brief  Write the heater configuration of the SI 7006
 *  @param[in]  failure - pointer to the structure which is used for the
 *              error tracking
 *  @param[out] heater_config - contains the new heater configuration
 *  @retval SUCCESS         - successful execution of the function
 *  @retval LIB_ERROR       - an error occur during using a library function
 *  @retval INTERNAL_ERROR  - an error occur during using an internal function
 *****************************************************************************/
Status_t Si7006_writeHeaterConfig(Failure_t* failure, uint8_t heater_config);

/** ***************************************************************************
 *  @brief  Read the user configuration register of the SI 7006
 *  @param[in]  failure - pointer to the structure which is used for the
 *              error tracking
 *  @param[out] user_config - pointer to the variable which stores
 *              the read user register configuration
 *  @retval SUCCESS         - successful execution of the function
 *  @retval LIB_ERROR       - an error occur during using a library function
 *  @retval INTERNAL_ERROR  - an error occur during using an internal function
 *****************************************************************************/
Status_t Si7006_readUserRegister(Failure_t* failure, uint8_t* user_config);

/** ***************************************************************************
 *  @brief  Write the user configuration register of the SI 7006
 *  @param[in]  failure - pointer to the structure which is used for the
 *              error tracking
 *  @param[out] user_config - contains the new user register configuration
 *  @retval SUCCESS         - successful execution of the function
 *  @retval LIB_ERROR       - an error occur during using a library function
 *  @retval INTERNAL_ERROR  - an error occur during using an internal function
 *****************************************************************************/
Status_t Si7006_writeUserRegister(Failure_t* failure, uint8_t user_config);

#endif // SI_7006_H