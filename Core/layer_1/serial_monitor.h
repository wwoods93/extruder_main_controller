/***********************************************************************************************************************
 * Main_Controller
 * device_serial_monitor.h
 *
 * wilson
 * 10/8/24
 * 12:02 AM
 *
 * Description:
 *
 **********************************************************************************************************************/

#ifndef MAIN_CONTROLLER_SERIAL_MONITOR_H
#define MAIN_CONTROLLER_SERIAL_MONITOR_H

/* c/c++ includes */
#include <cstdint>
/* stm32 includes */
#include "stm32f4xx.h"
/* third-party includes */
#include "../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.h"
/* layer_0 includes */

/* layer_1_rtosal includes */
#include "../layer_0/rtosal.h"
/* layer_1 includes */

/* layer_3_control includes */

/* layer_4_sys_op includes */

/* layer_n_meta_structure includes */




class serial_monitor
{
    public:


        static constexpr uint8_t USE_SERIAL_MONITOR = 1U;
        static constexpr uint8_t NAME_LENGTH_MAX = 15U;
        static constexpr uint8_t MESSAGE_LENGTH_MAX = 50U;

        typedef struct
        {
            char message[MESSAGE_LENGTH_MAX];
        } packet_t;

        uint32_t initialize(UART_HandleTypeDef* arg_usart_module, rtosal::message_queue_id_t arg_message_queue_handle);
        uint32_t print(const char* arg_output_string);
        uint32_t process_send_buffer();



    private:
        char name[NAME_LENGTH_MAX] = "serial_monitor";
        char message[MESSAGE_LENGTH_MAX];
        char time_stamp[9] = "";
        UART_HandleTypeDef* usart_module;
        rtosal::message_queue_id_t usart_message_queue_send_handle;
        rtosal::message_queue_id_t usart_message_queue_receive_handle;
};


#endif //MAIN_CONTROLLER_SERIAL_MONITOR_H
