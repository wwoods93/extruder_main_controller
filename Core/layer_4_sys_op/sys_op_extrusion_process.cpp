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

#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "cmsis_os2.h"

#include "../layer_0/hal_timer.h"
#include "system_clock.h"
#include "gpio.h"

#include "../layer_0/hal.h"
#include "../layer_0/hal_general.h"
#include "../layer_0/hal_wrapper.h"
#include "../layer_0/hal_callback.h"
#include "../layer_0/hal.h"
#include "../layer_0/rtosal_globals.h"
#include "../layer_0/rtosal.h"
#include "../layer_1/device.h"
#include "../layer_1/rtd.h"
#include "../layer_1/band_heater.h"

#include "../application/extruder.h"
#include "sys_op_extrusion_process.h"

#define EXTRUSION_PROCESS_STATE_INITIALIZE                          0
#define EXTRUSION_PROCESS_STATE_WAIT_FOR_SYSTEM_INITIALIZATION      1
#define EXTRUSION_PROCESS_STATE_CONFIGURE_USERS                     2
#define EXTRUSION_PROCESS_STATE_RUN                                 3


namespace sys_op::extrusion
{
    rtosal::event_flag_handle_t initialization_event_flags_handle = nullptr;
    rtosal::message_queue_handle_t spi_tx_queue_handle = nullptr;
    rtosal::message_queue_handle_t spi_rx_queue_handle = nullptr;
    rtosal::message_queue_handle_t comms_handler_output_data_queue_handle = nullptr;

    // TODO: rename
    static uint32_t extrusion_process_iteration_tick;

    // TODO: get rid of this
    static uint16_t success_counter = 0;

    common_packet_t tx_common_packet;
    common_packet_t rx_common_packet;
    common_float_data_t rtd_reading;



    uint8_t tx[8] = { 0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x0F };

    void task_intitialize()
    {

    }

    void task_state_machine()
    {
        static uint8_t extrusion_process_state = EXTRUSION_PROCESS_STATE_INITIALIZE;
        static uint32_t counter = 0;
        static uint8_t flag = 0;
        switch (extrusion_process_state)
        {
            case EXTRUSION_PROCESS_STATE_INITIALIZE:
            {
                initialization_event_flags_handle = get_initialization_event_flags_handle();

                extrusion_process_iteration_tick = 0U;

                extrusion_process_state = EXTRUSION_PROCESS_STATE_WAIT_FOR_SYSTEM_INITIALIZATION;
                break;
            }
            case EXTRUSION_PROCESS_STATE_WAIT_FOR_SYSTEM_INITIALIZATION:
            {
                rtosal::event_flag_wait(initialization_event_flags_handle, READY_FOR_USER_INIT_FLAG, rtosal::OS_FLAGS_ANY, rtosal::OS_WAIT_FOREVER);
                extrusion_process_state = EXTRUSION_PROCESS_STATE_CONFIGURE_USERS;
                break;
            }
            case EXTRUSION_PROCESS_STATE_CONFIGURE_USERS:
            {
                spi_tx_queue_handle = get_spi_2_extrusion_task_tx_queue_handle();
                spi_rx_queue_handle = get_spi_2_extrusion_task_rx_queue_handle();
                comms_handler_output_data_queue_handle = get_comms_handler_output_data_queue_handle();

                // TODO: wrap / rename
                MX_TIM6_Init();
                // TODO: create wrapper
                HAL_TIM_Base_Start(get_timer_6_handle());

                device::rtd_zone_0.initialize(rtd::READ_RATE_10_HZ, 0);
                device::rtd_zone_1.initialize(rtd::READ_RATE_10_HZ, 1);
                device::rtd_zone_2.initialize(rtd::READ_RATE_10_HZ, 2);

                // TODO: figure out this and handle_sensor_state()
                device::rtd_zone_0.start_read_requests();
                device::rtd_zone_1.start_read_requests();
                device::rtd_zone_2.start_read_requests();

                extrusion_process_state = EXTRUSION_PROCESS_STATE_RUN;
                break;
            }
            case EXTRUSION_PROCESS_STATE_RUN:
            {
                device::zone_1_band_heater.set_period(8000);
                device::zone_2_band_heater.set_period(8000);
                device::zone_3_band_heater.set_period(8000);

                // TODO: wrap Instance->CNT
                if (get_timer_6_handle()->Instance->CNT - extrusion_process_iteration_tick > 500U)
                {
                    // TODO: figure out this and start_read_requests()
                    device::rtd_zone_0.handle_sensor_state();
                    device::rtd_zone_1.handle_sensor_state();
                    device::rtd_zone_2.handle_sensor_state();
                    extrusion_process_iteration_tick = get_timer_6_handle()->Instance->CNT;
                }

                if (device::rtd_zone_0.send_request_if_flag_set(tx_common_packet))
                {
                    if (rtosal::message_queue_send(spi_tx_queue_handle, &tx_common_packet, 0U) == rtosal::OS_OK)
                    {
                        ++success_counter;
                    }
                    device::rtd_zone_0.clear_send_new_request_flag();
                }

                if (device::rtd_zone_1.send_request_if_flag_set(tx_common_packet))
                {
                    if (rtosal::message_queue_send(spi_tx_queue_handle, &tx_common_packet, 0U) == rtosal::OS_OK)
                    {
                        ++success_counter;
                    }
                    device::rtd_zone_1.clear_send_new_request_flag();
                }
                if (device::rtd_zone_2.send_request_if_flag_set(tx_common_packet))
                {
                    if (rtosal::message_queue_send(spi_tx_queue_handle, &tx_common_packet, 0U) == rtosal::OS_OK)
                    {
                        ++success_counter;
                    }
                    device::rtd_zone_2.clear_send_new_request_flag();
                }

                // TODO: make this cleaner
                uint8_t count = 0U;
                while (rtosal::message_queue_receive( spi_rx_queue_handle, &rx_common_packet, 50U) == rtosal::OS_OK && count < 3)
                {
                    rtd_reading.id = rx_common_packet.channel_id;

                    switch (rx_common_packet.channel_id)
                    {
                        case 0:
                        {
                            device::rtd_zone_0.read_rtd_and_calculate_temperature(rx_common_packet);
                            rtd_reading.value = device::rtd_zone_0.compute_temperature_moving_average();
                            break;
                        }
                        case 1:
                        {
                            device::rtd_zone_1.read_rtd_and_calculate_temperature(rx_common_packet);
                            rtd_reading.value = device::rtd_zone_1.compute_temperature_moving_average();
                            break;
                        }
                        case 2:
                        {
                            device::rtd_zone_2.read_rtd_and_calculate_temperature(rx_common_packet);
                            rtd_reading.value = device::rtd_zone_2.compute_temperature_moving_average();
                            break;
                        }
                        default:
                        {
                            break;
                        }

                    }

                    if (rtosal::message_queue_send(comms_handler_output_data_queue_handle, &rtd_reading, 50U) == rtosal::OS_OK)
                    {
                        ++success_counter;
                    }
                    ++count;
                }

                break;
            }
            default:
                break;
        }
    }
}
