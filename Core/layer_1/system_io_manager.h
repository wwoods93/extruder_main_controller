/***********************************************************************************************************************
 * Main_Controller
 * system_io_manager.h
 *
 * wilson
 * 10/13/24
 * 10:09 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

#ifndef MAIN_CONTROLLER_SYSTEM_IO_MANAGER_H
#define MAIN_CONTROLLER_SYSTEM_IO_MANAGER_H

/* c/c++ includes */
#include <cstdint>
#include <queue>
/* stm32 includes */

/* third-party includes */

/* layer_0_hal includes */

/* layer_1_rtosal includes */
#include "../layer_0/rtosal.h"
/* layer_2_device includes */

/* layer_3_control includes */

/* layer_3 includes */

/* layer_n_meta_structure includes */




class system_io_manager
{
    public:
        static constexpr uint8_t MESSAGE_LENGTH_MAX = 50U;
        static constexpr uint8_t DEVICE_NAME_LENGTH_MAX = 50U;
        static constexpr int16_t DEVICE_0 = 0U;
        static constexpr int16_t DEVICE_1 = 1U;
        static constexpr int16_t DEVICE_2 = 2U;


        typedef struct
        {
            uint8_t global_id;
            uint32_t uint32_data;
            float float_data;
            char message[MESSAGE_LENGTH_MAX];
        } packet_t;

        typedef struct
        {
            int16_t id;
            char name[DEVICE_NAME_LENGTH_MAX];
            rtosal::message_queue_handle_t tx_buffer_handle;
        } device_t;


        uint32_t initialize(rtosal::message_queue_handle_t arg_intertask_buffer_queue_handle);
        uint32_t add_device(rtosal::message_queue_handle_t arg_device_tx_buffer_queue_handle);
        uint32_t process_device_tx_buffers();



    private:
        rtosal::message_queue_handle_t intertask_buffer_queue_handle;

        int16_t next_available_device_id = 0U;
        device_t device_0;
        device_t device_1;
        device_t device_2;



};


#endif //MAIN_CONTROLLER_SYSTEM_IO_MANAGER_H
