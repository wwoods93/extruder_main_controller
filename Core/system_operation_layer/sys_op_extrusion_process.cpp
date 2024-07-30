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
#include "../layer_1_driver/driver_dc_motor_controller.h"
#include "sys_op_extrusion_process.h"

#define EXTRUSION_PROCESS_STATE_INITIALIZE                          0
#define EXTRUSION_PROCESS_STATE_WAIT_FOR_SYSTEM_INITIALIZATION      1
#define EXTRUSION_PROCESS_STATE_CONFIGURE_USERS                     2
#define EXTRUSION_PROCESS_STATE_RUN                                 3

namespace driver
{
    rtd rtd_zone_0;
    rtd rtd_zone_1;
    rtd rtd_zone_2;

    float average_temp_zone_0 = 0;
    float average_temp_zone_1 = 0;
    float average_temp_zone_2 = 0;

    uint8_t value_updated_zone_0 = 0;
    uint8_t value_updated_zone_1 = 0;
    uint8_t value_updated_zone_2 = 0;
//    dc_motor_controller motor_controller_1;
}

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


//    osMutexId_t extrusion_process_spi_tx_data_buffer_mutex = nullptr;



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
                spi_tx_queue_handle = get_spi_tx_queue_handle();
                spi_rx_queue_handle = get_spi_rx_queue_handle();
                i2c_tx_queue_handle = get_i2c_tx_queue_handle();

//                driver::motor_controller_1.initialize_controller(0x0C, dc_motor_controller::F_3921Hz, true, false);
                driver::rtd_zone_0.initialize(rtd::READ_RATE_10_HZ, 0);
                driver::rtd_zone_1.initialize(rtd::READ_RATE_10_HZ, 1);
                driver::rtd_zone_2.initialize(rtd::READ_RATE_10_HZ, 2);
                driver::rtd_zone_0.start_read_requests();
                driver::rtd_zone_1.start_read_requests();
                driver::rtd_zone_2.start_read_requests();

//                driver::motor_controller_1.set_speed(dc_motor_controller::M1, 150);
                extrusion_process_state = EXTRUSION_PROCESS_STATE_RUN;
                break;
            }
            case EXTRUSION_PROCESS_STATE_RUN:
            {

                if (rtosal::get_rtos_kernel_tick_count() - extrusion_process_iteration_tick > 5U /*kernel_tick_frequency_hz*/)
                {
                    driver::rtd_zone_0.handle_sensor_state();
                    driver::rtd_zone_1.handle_sensor_state();
                    driver::rtd_zone_2.handle_sensor_state();
                    extrusion_process_iteration_tick = osKernelGetTickCount();
//                    if (counter > 100)
//                    {
//                        if (flag == 0U)
//                        {
//                            driver::motor_controller_1.set_speed(dc_motor_controller::M1, 50);
//                            flag = 1U;
//                        }
//                        else
//                        {
//                            driver::motor_controller_1.set_speed(dc_motor_controller::M1, 200);
//                            flag = 1U;
//                        }
//                        counter = 0;
//                    }
//                    ++counter;
                }

                if (driver::rtd_zone_0.send_request_if_flag_set(tx_common_packet))
                {
                    if (osMessageQueuePut(spi_tx_queue_handle, &tx_common_packet, 0, 0U) == osOK)
                    {
                        ++success_counter;
                    }
                    driver::rtd_zone_0.clear_send_new_request_flag();
                }

                if (driver::rtd_zone_1.send_request_if_flag_set(tx_common_packet))
                {
                    if (osMessageQueuePut(spi_tx_queue_handle, &tx_common_packet, 0, 0U) == osOK)
                    {
                        ++success_counter;
                    }
                    driver::rtd_zone_1.clear_send_new_request_flag();
                }
                if (driver::rtd_zone_2.send_request_if_flag_set(tx_common_packet))
                {
                    if (osMessageQueuePut(spi_tx_queue_handle, &tx_common_packet, 0, 0U) == osOK)
                    {
                        ++success_counter;
                    }
                    driver::rtd_zone_2.clear_send_new_request_flag();
                }

                rx_buffer_accessed = 0U;
                uint8_t count = 0U;
                while (osMessageQueueGet( spi_rx_queue_handle, &rx_common_packet, nullptr, 50U) == osOK && count < 3)
                {
                    switch (rx_common_packet.channel_id)
                    {
                        case 0:
                        {
                            driver::rtd_zone_0.read_rtd_and_calculate_temperature(rx_common_packet);
                            driver::average_temp_zone_0 = driver::rtd_zone_0.compute_temperature_moving_average();
                            driver::value_updated_zone_0 = 1;
                            break;
                        }
                        case 1:
                        {
                            driver::rtd_zone_1.read_rtd_and_calculate_temperature(rx_common_packet);
                            driver::average_temp_zone_1 = driver::rtd_zone_1.compute_temperature_moving_average();
                            driver::value_updated_zone_1 = 1;
                            break;
                        }
                        case 2:
                        {
                            driver::rtd_zone_2.read_rtd_and_calculate_temperature(rx_common_packet);
                            driver::average_temp_zone_2 = driver::rtd_zone_2.compute_temperature_moving_average();
                            driver::value_updated_zone_2 = 1;
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

                if (driver::value_updated_zone_0 == 1U)
                {
                    rtd_reading.id = 0;
                    rtd_reading.value = driver::average_temp_zone_0;
                    if (osMessageQueuePut(i2c_tx_queue_handle, &rtd_reading, 0, 0U) == osOK)
                    {
                        ++success_counter;
                    }
                    rtd_reading.id = 0;
                    rtd_reading.value = 0;
                    driver::value_updated_zone_0 = 0U;
                }

                if (driver::value_updated_zone_1 == 1U)
                {
                    rtd_reading.id = 1;
                    rtd_reading.value = driver::average_temp_zone_1;
                    if (osMessageQueuePut(i2c_tx_queue_handle, &rtd_reading, 0, 0U) == osOK)
                    {
                        ++success_counter;
                    }
                    rtd_reading.id = 0;
                    rtd_reading.value = 0;
                    driver::value_updated_zone_1 = 0U;
                }

                if (driver::value_updated_zone_2 == 1U)
                {
                    rtd_reading.id = 2;
                    rtd_reading.value = driver::average_temp_zone_2;
                    if (osMessageQueuePut(i2c_tx_queue_handle, &rtd_reading, 0, 0U) == osOK)
                    {
                        ++success_counter;
                    }
                    rtd_reading.id = 0;
                    rtd_reading.value = 0;
                    driver::value_updated_zone_2 = 0U;
                }


                break;
            }
            default:
                break;
        }
    }
}
