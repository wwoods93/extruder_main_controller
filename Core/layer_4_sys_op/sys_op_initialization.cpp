/***********************************************************************************************************************
 * Main_Controller
 * system_operation_initialization.cpp
 *
 * wilson
 * 11/6/22
 * 3:49 PM
 *
 * Description:
 *
 **********************************************************************************************************************/
#include <cstdint>
#include "stm32f4xx.h"
#include "cmsis_os2.h"
#include "../layer_0/rtosal_globals.h"
#include "../meta_structure/meta_structure_system_manager.h"
#include "sys_op_initialization.h"

#define INITIALIZATION_TASK_STATE_INITIALIZE      0
#define INITIALIZATION_TASK_STATE_RUN             1

namespace sys_op::initialization
{

    #define DCM_0_ADDRESS       0x14
    #define DCM_1_ADDRESS       0x00

    osEventFlagsId_t initialization_event_flags_handle = nullptr;
    osMessageQueueId_t initialization_messaging_queue_handle = nullptr;

    resource_config_t   RESOURCE_SPI_2;
    resource_config_t   I2C_0;

    device_config_t     RTD_0;
    device_config_t     RTD_1;
    device_config_t     RTD_2;
    device_config_t     DCM_0;
    device_config_t     DCM_1;

    user_config_t       RTD_DRIVER_0;
    user_config_t       RTD_DRIVER_1;
    user_config_t       RTD_DRIVER_2;
    user_config_t       DCM_DRIVER_0;
    user_config_t       DCM_DRIVER_1;

    device_config_t null_device;

//    char spi_0_name[META_STRUCTURE_NAME_LENGTH_MAX]        = "SPI_RESOURCE_0    \0";
//    char i2c_0_name[META_STRUCTURE_NAME_LENGTH_MAX]        = "I2C_RESOURCE_0    \0";
//
//    char rtd_0_name[META_STRUCTURE_NAME_LENGTH_MAX]        = "RTD_ZONE_0        \0";
//    char rtd_1_name[META_STRUCTURE_NAME_LENGTH_MAX]        = "RTD_ZONE_1        \0";
//    char rtd_2_name[META_STRUCTURE_NAME_LENGTH_MAX]        = "RTD_ZONE_2        \0";
//    char dcm_0_name[META_STRUCTURE_NAME_LENGTH_MAX]        = "DCM_DEVICE_0      \0";
//    char dcm_1_name[META_STRUCTURE_NAME_LENGTH_MAX]        = "DCM_DEVICE_1      \0";
//
//    char rtd_driver_0_name[META_STRUCTURE_NAME_LENGTH_MAX] = "RTD_DRIVER_0      \0";
//    char rtd_driver_1_name[META_STRUCTURE_NAME_LENGTH_MAX] = "RTD_DRIVER_1      \0";
//    char rtd_driver_2_name[META_STRUCTURE_NAME_LENGTH_MAX] = "RTD_DRIVER_2      \0";
//    char dcm_driver_0_name[META_STRUCTURE_NAME_LENGTH_MAX] = "DCM_DRIVER_0      \0";
//    char dcm_driver_1_name[META_STRUCTURE_NAME_LENGTH_MAX] = "DCM_DRIVER_1      \0";

    void task_state_machine()
    {
        static uint8_t initialization_state = INITIALIZATION_TASK_STATE_INITIALIZE;

        switch (initialization_state)
        {
            case INITIALIZATION_TASK_STATE_INITIALIZE:
            {
                initialization_event_flags_handle = get_initialization_task_queue_handle();
                initialization_messaging_queue_handle = get_initialization_task_queue_handle();

//                meta_structure::initialize_system_manifests();
//                meta_structure::create_resource_config(RESOURCE_SPI_2, spi_0_name, RESOURCE_TYPE_SPI);
//                meta_structure::create_resource_config(I2C_0, i2c_0_name, RESOURCE_TYPE_I2C);
//                id_number_t rtd_0_id = meta_structure::create_device_config(RTD_0, rtd_0_name, DEVICE_TYPE_RTD, RESOURCE_TYPE_SPI, 4U, 2U, ADDRESS_NULL_8_BIT, PORT_B, GPIO_PIN_14);
//                id_number_t rtd_1_id = meta_structure::create_device_config(RTD_1, rtd_1_name, DEVICE_TYPE_RTD, RESOURCE_TYPE_SPI, 4U, 2U, ADDRESS_NULL_8_BIT, PORT_NULL, PIN_NULL);
//                id_number_t rtd_2_id = meta_structure::create_device_config(RTD_2, rtd_2_name, DEVICE_TYPE_RTD, RESOURCE_TYPE_SPI, 4U, 2U, ADDRESS_NULL_8_BIT, PORT_NULL, PIN_NULL);
//                id_number_t dcm_0_id = meta_structure::create_device_config(DCM_0, dcm_0_name, DEVICE_TYPE_DCM, RESOURCE_TYPE_I2C, 3U, 3U, DCM_0_ADDRESS, PORT_NULL, PIN_NULL);
//                id_number_t dcm_1_id = meta_structure::create_device_config(DCM_1, dcm_1_name, DEVICE_TYPE_DCM, RESOURCE_TYPE_I2C, 3U, 3U, DCM_1_ADDRESS, PORT_NULL, PIN_NULL);
//                meta_structure::create_user_config(RTD_DRIVER_0, rtd_driver_0_name, USER_TYPE_RTD, 1U, rtd_0_id, ID_INVALID, ID_INVALID, ID_INVALID);
//                meta_structure::create_user_config(RTD_DRIVER_1, rtd_driver_1_name, USER_TYPE_RTD, 1U, rtd_1_id, ID_INVALID, ID_INVALID, ID_INVALID);
//                meta_structure::create_user_config(RTD_DRIVER_2, rtd_driver_2_name, USER_TYPE_RTD, 1U, rtd_2_id, ID_INVALID, ID_INVALID, ID_INVALID);
//                meta_structure::create_user_config(DCM_DRIVER_0, dcm_driver_0_name, USER_TYPE_DCM, 2U, dcm_0_id, ID_INVALID, ID_INVALID, ID_INVALID);
//                meta_structure::create_user_config(DCM_DRIVER_1, dcm_driver_1_name, USER_TYPE_DCM, 1U, dcm_1_id, ID_INVALID, ID_INVALID, ID_INVALID);
                osEventFlagsSet(initialization_event_flags_handle, READY_FOR_RESOURCE_INIT_FLAG);
                initialization_state = INITIALIZATION_TASK_STATE_RUN;
                break;
            }
            case INITIALIZATION_TASK_STATE_RUN:
            {

                break;
            }
            default:
                break;
        }

        // in main
        //      1. declare mutex/queue handles, attributes
        //          - init task to and from comms handler task
        //          - init task to and from preparation task
        //          - init task to and from extrusion task
        //          - comms handler to and from preparation task
        //          - comms handler to and from extrusion task
        //          - comms handler to and from spooling task
        //
        // in init task
        //      1. declare and populate init structures - this will list all users/devices/resources in a single place
        //      2. initialize manifests
        //      3. add resource_init structs to resource_manifest
        //      4. add device_init structs to device manifest
        //      5. add user_init structs to user manifest
        //          - user_init structs specify which devices they own
        //          - user_init structs specify which resources to which they would like to have access

        // in comms handler task
        //      1. declare resource objects
        //      2. initialize resource objects from resource init structs - chip selects, addresses, module config
        //      3. each resource opens channel based on request flag in user init struct, channel id returned to user init struct

        // in preparation/extrusion/spooling tasks
        //      1. initialize user objects with device ids, resource ids, channel ids








    }
}
