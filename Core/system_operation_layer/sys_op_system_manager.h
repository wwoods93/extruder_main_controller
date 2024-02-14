/***********************************************************************************************************************
 * Main_Controller
 * sys_op_system_manager.h
 *
 * wilson
 * 2/10/24
 * 9:10 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

#ifndef MAIN_CONTROLLER_SYS_OP_SYSTEM_MANAGER_H
#define MAIN_CONTROLLER_SYS_OP_SYSTEM_MANAGER_H

/* c/c++ includes */
#include <cstdint>
#include <vector>
/* stm32 includes */

/* third-party includes */

/* hal includes */

/* driver includes */

/* rtos abstraction includes */

/* system includes */
#include "../system_operation_layer/sys_op_general.h"


//#define MAX_DRIVER_LEVEL_USERS      32U
//#define MAX_HAL_RESOURCES           32U
//
//typedef enum
//{
//    USER_TYPE_NULL                          = 0x00,
//    USER_TYPE_SCREW_MOTOR_CONTROLLER        = 0x01,
//    USER_TYPE_DC_MOTOR_CONTROLLER           = 0x02,
//    USER_TYPE_RTD                           = 0x03,
//    USER_TYPE_TEMPERATURE_CONTROLLER        = 0x04,
//    USER_TYPE_TOUCHSCREEN                   = 0x05,
//
//} user_t;
//
//typedef enum
//{
//    RESOURCE_TYPE_NULL                      = 0x00,
//    RESOURCE_TYPE_ADC                       = 0x01,
//    RESOURCE_TYPE_CAN                       = 0x02,
//    RESOURCE_TYPE_I2C                       = 0x03,
//    RESOURCE_TYPE_SPI                       = 0x04,
//    RESOURCE_TYPE_PWM                       = 0x05,
//    RESOURCE_TYPE_UART                      = 0x06,
//
//} resource_t;
//
//typedef struct
//{
//    user_t user_type;
//    uint8_t global_id;
//
//} user_manifest_entry_t;
//
//typedef struct
//{
//    resource_t resource_type;
//    uint8_t global_id;
//
//} resource_manifest_entry_t;
//
//uint8_t claim_next_available_global_user_id();
//uint8_t claim_next_available_global_resource_id();
//uint8_t register_new_user_to_user_manifest(user_t _user_type);
//uint8_t register_new_resource_to_resource_manifest(resource_t _resource_type);



#endif //MAIN_CONTROLLER_SYS_OP_SYSTEM_MANAGER_H
