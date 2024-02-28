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
#include "cmsis_os2.h"
#include "../rtos_abstraction_layer/rtos_globals.h"
#include "sys_op_initialization.h"

#define INITIALIZATION_TASK_STATE_INITIALIZE      0
#define INITIALIZATION_TASK_STATE_RUN             1

namespace sys_op
{

    osMessageQueueId_t initialization_messaging_queue_handle = nullptr;

    // user_init structs

    // device_init structs

    // resource_init structs






    void initialization_state_machine()
    {
        static uint8_t initialization_state = INITIALIZATION_TASK_STATE_INITIALIZE;


        switch (initialization_state)
        {
            case INITIALIZATION_TASK_STATE_INITIALIZE:
            {
                initialization_messaging_queue_handle = get_initialization_task_queue_handle();

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
