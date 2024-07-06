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
#include "../layer_2_rtosal/rtosal_globals.h"
#include "../layer_2_rtosal/rtosal.h"
#include "../layer_1_driver/driver_rtd.h"
#include "sys_op_extrusion_process.h"

#define EXTRUSION_PROCESS_STATE_INITIALIZE                          0
#define EXTRUSION_PROCESS_STATE_WAIT_FOR_SYSTEM_INITIALIZATION      1
#define EXTRUSION_PROCESS_STATE_CONFIGURE_USERS                     2
#define EXTRUSION_PROCESS_STATE_RUN                                 3

namespace driver
{
    rtd rtd_zone_0;
//    rtd rtd_zone_1;
//    rtd rtd_zone_2;
}

namespace sys_op::extrusion
{
    osEventFlagsId_t initialization_event_flags_handle = nullptr;
    osMessageQueueId_t spi_tx_queue_handle = nullptr;

    static uint32_t extrusion_process_iteration_tick;
    uint32_t kernel_tick_frequency_hz;

    static uint8_t packet_added = false;
    static uint16_t success_counter = 0;


//    osMutexId_t extrusion_process_spi_tx_data_buffer_mutex = nullptr;



    uint8_t tx[8] = { 0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x0F };

    void task_intitialize()
    {

    }

    void task_state_machine()
    {
        static uint8_t extrusion_process_state = EXTRUSION_PROCESS_STATE_INITIALIZE;

        switch (extrusion_process_state)
        {
            case EXTRUSION_PROCESS_STATE_INITIALIZE:
            {
                initialization_event_flags_handle = get_initialization_event_flags_handle();

                driver::rtd_zone_0.initialize(rtd::READ_RATE_10_HZ);
                driver::rtd_zone_0.start_read_requests();
                extrusion_process_iteration_tick = 0;
                kernel_tick_frequency_hz = rtosal::get_rtos_kernel_tick_frequency() * 2;
                spi_tx_queue_handle = get_spi_tx_queue_handle();
//                initialization_event_flags_handle = get_initialization_task_queue_handle();

//                extrusion_process_spi_tx_data_buffer_mutex = get_spi_tx_buffer_mutex();
                extrusion_process_state = EXTRUSION_PROCESS_STATE_WAIT_FOR_SYSTEM_INITIALIZATION;
                break;
            }
            case EXTRUSION_PROCESS_STATE_WAIT_FOR_SYSTEM_INITIALIZATION:
            {
                osEventFlagsWait(initialization_event_flags_handle, READY_FOR_USER_INIT_FLAG, osFlagsWaitAny, osWaitForever);
                extrusion_process_state = EXTRUSION_PROCESS_STATE_CONFIGURE_USERS;
                break;
            }
            case EXTRUSION_PROCESS_STATE_CONFIGURE_USERS:
            {

                extrusion_process_state = EXTRUSION_PROCESS_STATE_RUN;
                break;
            }
            case EXTRUSION_PROCESS_STATE_RUN:
            {

                driver::rtd_zone_0.handle_sensor_state();
                if (rtosal::get_rtos_kernel_tick_count() - extrusion_process_iteration_tick > 50U /*kernel_tick_frequency_hz*/)
                {
                    common_packet_t packet;
//                    rtosal::build_common_packet(packet, 0, tx);

                    if (driver::rtd_zone_0.send_request_if_flag_set(packet))
                    {
                        if (osMessageQueuePut(spi_tx_queue_handle, &packet, 0, 50U) == osOK)
                        {
                            packet_added = true;
                        }
                        driver::rtd_zone_0.clear_send_new_request_flag();
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
