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

#include "../layer_0/hal_general.h"
#include "../layer_0/hal_wrapper.h"
#include "../layer_0/hal_callback.h"
#include "../layer_0/hal.h"
#include "../layer_0/rtosal_globals.h"
#include "../layer_0/rtosal.h"
#include "../layer_1/device.h"
#include "../layer_1/device_rtd.h"
#include "../layer_1/band_heater.h"

#include "../application/extruder.h"
#include "sys_op_extrusion_process.h"

#define EXTRUSION_PROCESS_STATE_INITIALIZE                          0
#define EXTRUSION_PROCESS_STATE_WAIT_FOR_SYSTEM_INITIALIZATION      1
#define EXTRUSION_PROCESS_STATE_CONFIGURE_USERS                     2
#define EXTRUSION_PROCESS_STATE_RUN                                 3

namespace device
{
    rtd rtd_zone_0;
    rtd rtd_zone_1;
    rtd rtd_zone_2;

//    band_heater zone_1_band_heater;
//    band_heater zone_2_band_heater;
//    band_heater zone_3_band_heater;

    float average_temp_zone_0 = 0;
    float average_temp_zone_1 = 0;
    float average_temp_zone_2 = 0;

    uint8_t value_updated_zone_0 = 0;
    uint8_t value_updated_zone_1 = 0;
    uint8_t value_updated_zone_2 = 0;
}

//void timer_1_input_capture_zero_crossing_pulse_detected_callback(TIM_HandleTypeDef *htim)
//{
//    device::band_heater::zero_crossing_pulse_restart();
//    output_pulse_restart(&device::zone_1_band_heater, 1000);
//    output_pulse_restart(&device::zone_2_band_heater, 4500);
//    output_pulse_restart(&device::zone_3_band_heater, 800);
//}

//TIM_HandleTypeDef* device::band_heater::zero_crossing_pulse_timer_module = get_timer_1_handle();





namespace sys_op::extrusion
{
    osEventFlagsId_t initialization_event_flags_handle = nullptr;
    osMessageQueueId_t spi_tx_queue_handle = nullptr;
    osMessageQueueId_t spi_rx_queue_handle = nullptr;
    osMessageQueueId_t i2c_tx_queue_handle = nullptr;

    static uint32_t extrusion_process_iteration_tick;
    uint32_t kernel_tick_frequency_hz;

    static uint8_t packet_added = false;
    static uint8_t rx_buffer_accessed = 0U;
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

                extrusion_process_iteration_tick = 0;
                kernel_tick_frequency_hz = rtosal::get_rtos_kernel_tick_frequency() * 2;

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
                spi_tx_queue_handle = get_spi_2_extrusion_task_tx_queue_handle();
                spi_rx_queue_handle = get_spi_2_extrusion_task_rx_queue_handle();
                i2c_tx_queue_handle = get_i2c_tx_queue_handle();

//                device::zone_1_band_heater.initialize(TEMPERATURE_ZONE_1, TIMER_10_ID);
//                device::zone_2_band_heater.initialize(TEMPERATURE_ZONE_2, TIMER_13_ID);
//                device::zone_3_band_heater.initialize(TEMPERATURE_ZONE_3, TIMER_14_ID);
//                HAL_TIM_RegisterCallback(get_timer_1_handle(),  HAL_TIM_IC_CAPTURE_CB_ID, timer_1_input_capture_zero_crossing_pulse_detected_callback);
//                HAL_TIM_IC_Start_IT(get_timer_1_handle(), TIM_CHANNEL_2);

                device::rtd_zone_0.initialize(rtd::READ_RATE_10_HZ, 0);
                device::rtd_zone_1.initialize(rtd::READ_RATE_10_HZ, 1);
                device::rtd_zone_2.initialize(rtd::READ_RATE_10_HZ, 2);
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

                if (rtosal::get_rtos_kernel_tick_count() - extrusion_process_iteration_tick > 5U /*kernel_tick_frequency_hz*/)
                {
                    device::rtd_zone_0.handle_sensor_state();
                    device::rtd_zone_1.handle_sensor_state();
                    device::rtd_zone_2.handle_sensor_state();
                    extrusion_process_iteration_tick = osKernelGetTickCount();
                }

                if (device::rtd_zone_0.send_request_if_flag_set(tx_common_packet))
                {
                    if (osMessageQueuePut(spi_tx_queue_handle, &tx_common_packet, 0, 0U) == osOK)
                    {
                        ++success_counter;
                    }
                    device::rtd_zone_0.clear_send_new_request_flag();
                }

                if (device::rtd_zone_1.send_request_if_flag_set(tx_common_packet))
                {
                    if (osMessageQueuePut(spi_tx_queue_handle, &tx_common_packet, 0, 0U) == osOK)
                    {
                        ++success_counter;
                    }
                    device::rtd_zone_1.clear_send_new_request_flag();
                }
                if (device::rtd_zone_2.send_request_if_flag_set(tx_common_packet))
                {
                    if (osMessageQueuePut(spi_tx_queue_handle, &tx_common_packet, 0, 0U) == osOK)
                    {
                        ++success_counter;
                    }
                    device::rtd_zone_2.clear_send_new_request_flag();
                }

                rx_buffer_accessed = 0U;
                uint8_t count = 0U;
                while (osMessageQueueGet( spi_rx_queue_handle, &rx_common_packet, nullptr, 50U) == osOK && count < 3)
                {
                    switch (rx_common_packet.channel_id)
                    {
                        case 0:
                        {
                            device::rtd_zone_0.read_rtd_and_calculate_temperature(rx_common_packet);
                            device::average_temp_zone_0 = device::rtd_zone_0.compute_temperature_moving_average();
                            device::value_updated_zone_0 = 1;
                            break;
                        }
                        case 1:
                        {
                            device::rtd_zone_1.read_rtd_and_calculate_temperature(rx_common_packet);
                            device::average_temp_zone_1 = device::rtd_zone_1.compute_temperature_moving_average();
                            device::value_updated_zone_1 = 1;
                            break;
                        }
                        case 2:
                        {
                            device::rtd_zone_2.read_rtd_and_calculate_temperature(rx_common_packet);
                            device::average_temp_zone_2 = device::rtd_zone_2.compute_temperature_moving_average();
                            device::value_updated_zone_2 = 1;
                            break;
                        }
                        default:
                        {
                            break;
                        }

                    }
                    ++count;
                    rx_buffer_accessed = 1U;
                }

                if (device::value_updated_zone_0 == 1U)
                {
                    rtd_reading.id = 0;
                    rtd_reading.value = device::average_temp_zone_0;
                    if (osMessageQueuePut(i2c_tx_queue_handle, &rtd_reading, 0, 0U) == osOK)
                    {
                        ++success_counter;
                    }
                    rtd_reading.id = 0;
                    rtd_reading.value = 0;
                    device::value_updated_zone_0 = 0U;
                }

                if (device::value_updated_zone_1 == 1U)
                {
                    rtd_reading.id = 1;
                    rtd_reading.value = device::average_temp_zone_1;
                    if (osMessageQueuePut(i2c_tx_queue_handle, &rtd_reading, 0, 0U) == osOK)
                    {
                        ++success_counter;
                    }
                    rtd_reading.id = 0;
                    rtd_reading.value = 0;
                    device::value_updated_zone_1 = 0U;
                }

                if (device::value_updated_zone_2 == 1U)
                {
                    rtd_reading.id = 2;
                    rtd_reading.value = device::average_temp_zone_2;
                    if (osMessageQueuePut(i2c_tx_queue_handle, &rtd_reading, 0, 0U) == osOK)
                    {
                        ++success_counter;
                    }
                    rtd_reading.id = 0;
                    rtd_reading.value = 0;
                    device::value_updated_zone_2 = 0U;
                }

                break;
            }
            default:
                break;
        }
    }
}
