/***********************************************************************************************************************
 * Main_Controller
 * system_operation_comms_handler.cpp
 *
 * wilson
 * 11/6/22
 * 3:46 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

/* c/c++ includes */
#include <cstdint>
#include <cstring>
/* stm32 includes */
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
/* 3rd-party includes */
#include "cmsis_os2.h"
/* hal includes */
#include "../hardware_abstraction_layer/hal_general.h"
#include "../hardware_abstraction_layer/hal_callbacks.h"
#include "../hardware_abstraction_layer/hal_spi.h"
#include "../hardware_abstraction_layer/hal_i2c.h"
#include "mcu_clock_timers.h"
#include "system_clock.h"
#include "gpio.h"
#include "spi.h"
/* driver includes */
#include "../driver_layer/driver_dc_motor_controller.h"
#include "../driver_layer/driver_rtd.h"
/* system includes */
/* rtos includes */
#include "../rtos_abstraction_layer/rtos_globals.h"
#include "../rtos_abstraction_layer/rtos_abstraction_layer.h"
/* system_operation_comms_handler header */
#include "sys_op_comms_handler.h"

#include "../meta_structure/meta_structure_system_manager.h"

#define COMMS_HANDLER_STATE_INITIALIZE      0
#define COMMS_HANDLER_STATE_RUN             1

spi::handle_t spi_2_handle;
i2c::handle_t i2c_2_handle;

void SPI2_IRQHandler()
{
    spi_irq_handler(get_spi_object());
}

namespace driver
{
    dc_motor_controller motor_controller_1;
    rtd rtd_1;
}

namespace hal
{
    spi spi_2;
}

spi* get_spi_object()
{
    return &hal::spi_2;
}

spi::handle_t* get_spi_handle()
{
    return &spi_2_handle;
}

rtd* get_rtd_object()
{
    return &driver::rtd_1;
}

namespace sys_op
{
    static uint32_t comms_handler_iteration_tick;
    uint32_t rtos_kernel_tick_frequency_hz;

    static uint8_t buffer_accessed;
    static uint8_t common_array_accessed;

    static spi::channel_t channel;
    common_packet_t packet;
    static osMutexId_t comms_handler_spi_tx_data_buffer_mutex;
    osMessageQueueId_t comms_handler_task_spi_tx_from_extrusion_queue_handle = nullptr;

    void comms_handler_intitialize()
    {

    }

    void comms_handler_state_machine()
    {
        static uint8_t comms_handler_state = COMMS_HANDLER_STATE_INITIALIZE;
        static uint8_t counter = 0;
        static uint8_t packet_valid = false;
        static id_number_t channel_id;
        uint8_t tx_d[8] = { 0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x0F };
        uint8_t rx_d[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

        switch (comms_handler_state)
        {
            case COMMS_HANDLER_STATE_INITIALIZE:
            {
                comms_handler_iteration_tick = 0;
                rtos_kernel_tick_frequency_hz = osKernelGetTickFreq();
                rtos_kernel_tick_frequency_hz = rtos_kernel_tick_frequency_hz;

                buffer_accessed = false;
                common_array_accessed = false;

                comms_handler_spi_tx_data_buffer_mutex = get_spi_tx_buffer_mutex();
                comms_handler_task_spi_tx_from_extrusion_queue_handle = get_extrusion_task_spi_tx_queue_handle();
                initialize_system_manifests();
                register_new_device_to_device_manifest(DEVICE_TYPE_RTD_SENSOR, "arduino");
                hal::spi_2.initialize(&spi_2_handle, spi::SPI_TX_RX_COMPLETE_CALLBACK_ID, HAL_SPI_TxRxCplt_Callback, spi::SPI_TX_RX_COMPLETE_CALLBACK_ID, HAL_SPI_Error_Callback);
                hal::spi_2.create_channel(channel_id, 8, 2, PORT_B, GPIO_PIN_14);
                hal::spi_2.get_channel_by_channel_id(channel, channel_id);

                comms_handler_state = COMMS_HANDLER_STATE_RUN;
                break;
            }
            case COMMS_HANDLER_STATE_RUN:
            {

                if (osKernelGetTickCount() - comms_handler_iteration_tick > 25U/*rtos_kernel_tick_frequency_hz*/)
                {

                    if (osMessageQueueGet( comms_handler_task_spi_tx_from_extrusion_queue_handle, &packet, nullptr, 50U) == osOK)
                    {
                        common_array_accessed = true;
                    }

                    if (packet.status == 0xFF) // need XOR checksum or something
                    {
                        packet_valid = true;
                    }
//                    if (osMutexAcquire(comms_handler_spi_tx_data_buffer_mutex, 50U) == osOK)
//                    {
//                        common_array_accessed = rtos_al::remove_packet_from_common_packet_array(packet);
//                        osMutexRelease(comms_handler_spi_tx_data_buffer_mutex);
//                    }
//                    rtos_al::increment_packet_remove_index();

                    if (common_array_accessed && packet_valid)
                    {
                        hal::spi_2.transmit(channel_id, channel.packet_size, channel.tx_size, packet.bytes);
                        common_array_accessed = false;
                        packet_valid = false;
                    }

                    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
                    comms_handler_iteration_tick = osKernelGetTickCount();
                }

                hal::spi_2.process_send_buffer();
                buffer_accessed = hal::spi_2.process_return_buffer(channel_id, rx_d);
                if (buffer_accessed)
                {
                    counter++;
                    buffer_accessed = false;
                }

                break;
            }
            default:
                break;
        }

    }
}
