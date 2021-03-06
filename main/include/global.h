
#ifndef GLOBAL_H
#define GLOBAL_H

#include <stdint.h>

/** ***************************************************************************
 * This enum contains all possible return values of the application
 *****************************************************************************/
typedef enum Status {
    SUCCESS,                            /*!< Successful execution */
    LIB_ERROR,                          /*!< Error in a library function */
    INTERNAL_ERROR                      /*!< Error generated by the application */
} Status_t;

/** ***************************************************************************
 * This struct is used for error tracking
 *****************************************************************************/
typedef struct Failure {
    Status_t    status;                 /*!< Stores the status code */
    uint32_t    failure_code;           /*!< Stores the failure code */
    uint32_t*   fn_pointer;             /*!< Stores the address of the function where the error occur */
    uint8_t     parameter[64];          /*!< Storing of the used parameter */
} Failure_t;

/** ***************************************************************************
 * This struct contains the error codes for the internal errors
 *****************************************************************************/
typedef enum ErrorCode {
    SIZE_ERROR,                         /*!< A read or write size is not in range */
    RESERVED
}ErrorCode_t;

#endif