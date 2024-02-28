/***********************************************************************************************************************
 * Main_Controller
 * system_operation_extrusion_process.cpp
 *
 * wilson
 * 11/6/22
 * 3:51 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

#include <cstdint>
#include "cmsis_os2.h"
#include "../rtos_abstraction_layer/rtos_globals.h"
#include "../rtos_abstraction_layer/rtos_abstraction_layer.h"
#include "sys_op_extrusion_process.h"

#define EXTRUSION_PROCESS_STATE_INITIALIZE      0
#define EXTRUSION_PROCESS_STATE_RUN             1

namespace sys_op
{
    static uint32_t extrusion_process_iteration_tick;
    uint32_t kernel_tick_frequency_hz;

    static uint8_t packet_added = false;
    static uint16_t success_counter = 0;


    osMutexId_t extrusion_process_spi_tx_data_buffer_mutex = nullptr;

    osMessageQueueId_t extrusion_task_spi_tx_from_extrusion_queue_handle = nullptr;

    uint8_t tx[8] = {0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02 };
    void extrusion_process_intitialize()
    {

    }

    void extrusion_process_state_machine()
    {
        static uint8_t extrusion_process_state = EXTRUSION_PROCESS_STATE_INITIALIZE;

        switch (extrusion_process_state)
        {
            case EXTRUSION_PROCESS_STATE_INITIALIZE:
            {
                extrusion_process_iteration_tick = 0;
                kernel_tick_frequency_hz = osKernelGetTickFreq() * 2;
                extrusion_task_spi_tx_from_extrusion_queue_handle = get_extrusion_task_spi_tx_queue_handle();
                extrusion_process_spi_tx_data_buffer_mutex = get_spi_tx_buffer_mutex();
                extrusion_process_state = EXTRUSION_PROCESS_STATE_RUN;
                break;
            }
            case EXTRUSION_PROCESS_STATE_RUN:
            {
                if (osKernelGetTickCount() - extrusion_process_iteration_tick > 50U/*kernel_tick_frequency_hz*/)
                {
                    common_packet_t packet;
                    rtos_al::build_common_packet(packet, 0, tx);

                    if ( osMessageQueuePut(extrusion_task_spi_tx_from_extrusion_queue_handle, &packet, 0, 50U) == osOK)
                    {
                        packet_added = true;
                    }



//                    if (osMutexAcquire(extrusion_process_spi_tx_data_buffer_mutex, 10U) == osOK)
//                    {
//                        packet_added = rtos_al::add_packet_to_common_packet_array(packet);
//                        osMutexRelease(extrusion_process_spi_tx_data_buffer_mutex);
//                    }
//                    rtos_al::increment_packet_add_index();

                    if (packet_added)
                    {
                        ++success_counter;
                        packet_added = false;
                    }
                    extrusion_process_iteration_tick = osKernelGetTickCount();
                }
                break;
            }
            default:
                break;
        }
    }
}
