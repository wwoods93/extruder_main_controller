/***********************************************************************************************************************
 * Main_Controller
 * sys_op_comms_handler.cpp
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
#include "../layer_0_hal//hal_general.h"
#include "../layer_0_hal//hal_callbacks.h"
#include "../layer_0_hal//hal_spi.h"
#include "../layer_0_hal//hal_i2c.h"
#include "../Inc/peripheral_common.h"
#include "mcu_clock_timers.h"
#include "system_clock.h"
#include "gpio.h"
#include "spi.h"
/* driver includes */
#include "../layer_1_driver/driver_dc_motor_controller.h"
#include "../layer_1_driver/driver_rtd.h"
/* system includes */
/* rtos includes */
#include "../layer_2_rtosal/rtosal_globals.h"
#include "../layer_2_rtosal/rtosal.h"
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
//    dc_motor_controller motor_controller_1;
//    rtd rtd_1;
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


const char user_data[29] = "The application is running\r\n"; //demo data for transmission
uint8_t send_data[6] = {(uint8_t)'h', (uint8_t)'e', (uint8_t)'l', (uint8_t)'l', (uint8_t)'o', '\r' };
uint8_t recvd_data; // receive buffer
uint8_t data_buffer[100]; // data buffer
uint32_t cnt = 0;

//UART 2 transmission complete callback
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
//    memset(&user_data, '\0' , strlen(user_data)); //empty the transmission data buffer
}
//UART 2 receive complete callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(recvd_data == '\r') //when enter is pressed go to this condition
    {
        data_buffer[cnt++]='\r';
        HAL_UART_Transmit(huart, send_data, cnt,HAL_MAX_DELAY); //transmit the full sentence again
        memset(data_buffer, 0, cnt); // enpty the data buffer
    }
    else
    {
        data_buffer[cnt++] = recvd_data; // every time when interrput is happen, received 1 byte of data
    }
    HAL_UART_Receive_IT(get_usart_2_handle(), &recvd_data,1); //start next data receive interrupt

}

namespace sys_op::comms_handler
{
    osEventFlagsId_t  initialization_event_flags_handle = nullptr;
    osMessageQueueId_t initialization_queue_handle = nullptr;
    osMessageQueueId_t spi_tx_queue_handle = nullptr;

    static uint32_t comms_handler_iteration_tick;
    uint32_t rtos_kernel_tick_frequency_hz;

    static uint8_t buffer_accessed;
    static uint8_t common_array_accessed;

    static spi::channel_t channel;
    id_number_t _channel_id = ID_INVALID;
    common_packet_t packet;
//    static osMutexId_t comms_handler_spi_tx_data_buffer_mutex;


    void task_intitialize()
    {

    }

    void task_state_machine()
    {
        static uint8_t comms_handler_state = COMMS_HANDLER_STATE_INITIALIZE;
        static uint8_t counter = 0;
        static uint8_t uart_counter = 0;
        static uint8_t packet_valid = false;
        static id_number_t channel_id;
        uint8_t tx_d[8] = { 0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x0F };
        uint8_t rx_d[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

        switch (comms_handler_state)
        {
            case COMMS_HANDLER_STATE_INITIALIZE:
            {
                initialization_event_flags_handle   = get_initialization_event_flags_handle();
                spi_tx_queue_handle                 = get_spi_tx_queue_handle();
                initialization_queue_handle         = get_initialization_task_queue_handle();
                osEventFlagsWait(initialization_event_flags_handle, READY_FOR_RESOURCE_INIT_FLAG, osFlagsWaitAny, osWaitForever);

                comms_handler_iteration_tick = 0;
                rtos_kernel_tick_frequency_hz = osKernelGetTickFreq();
                rtos_kernel_tick_frequency_hz = rtos_kernel_tick_frequency_hz;
                buffer_accessed = false;
                common_array_accessed = false;

//                comms_handler_spi_tx_data_buffer_mutex = get_spi_tx_buffer_mutex();
//                comms_handler_task_spi_tx_from_extrusion_queue_handle = get_extrusion_task_spi_tx_queue_handle();
//                initialize_system_manifests();
//                register_new_device_to_device_manifest(DEVICE_TYPE_RTD_SENSOR, "arduino");
                hal::spi_2.initialize(&spi_2_handle, SPI_2, 0U, spi::SPI_TX_RX_COMPLETE_CALLBACK_ID, HAL_SPI_TxRxCplt_Callback, spi::SPI_TX_RX_COMPLETE_CALLBACK_ID, HAL_SPI_Error_Callback);
                device_config_t device;
                for (uint8_t index = 0; index < meta_structure::get_device_manifest_size(); ++index)
                {
                    memset(&device, '\0', sizeof(device_config_t));
                    meta_structure::get_device_from_device_manifest(device, index);
                    if (device.device_id != ID_INVALID)
                    {
                        switch (device.device_resource_type)
                        {
                            case RESOURCE_TYPE_ADC:
                            {

                                break;
                            }
                            case RESOURCE_TYPE_CAN:
                            {

                                break;
                            }
                            case RESOURCE_TYPE_I2C:
                            {

                                break;
                            }
                            case RESOURCE_TYPE_SPI:
                            {

                                hal::spi_2.create_channel(_channel_id, device.packet_size, device.tx_size,
                                                          device.device_port, device.device_pin);
                                meta_structure::set_channel_id_for_device_in_manifest(_channel_id, index);
                                break;
                            }
                            case RESOURCE_TYPE_PWM:
                            {

                                break;
                            }
                            case RESOURCE_TYPE_UART:
                            {

                                break;
                            }
                            default:
                                break;
                        }
                    }
                }
//                hal::spi_2.create_channel(channel_id, 8, 2, PORT_B, GPIO_PIN_14);
//                char name_of_rtd_0[NAME_LENGTH_MAX]        = "RTD_ZONE_0        \0";
                _channel_id = meta_structure::get_channel_id_by_device_name(rtd_0_name);
                hal::spi_2.get_channel_by_channel_id(channel, _channel_id);
                osEventFlagsSet(initialization_event_flags_handle, READY_FOR_USER_INIT_FLAG);
                comms_handler_state = COMMS_HANDLER_STATE_RUN;
                break;
            }
            case COMMS_HANDLER_STATE_RUN:
            {

                if (osKernelGetTickCount() - comms_handler_iteration_tick > 25U/*rtos_kernel_tick_frequency_hz*/)
                {
                    if (uart_counter > 20)
                    {
                        HAL_UART_Transmit_IT(get_usart_2_handle(), (uint8_t*)user_data,strlen(user_data)); //Transmit data in interrupt mode
                        HAL_UART_Receive_IT(get_usart_2_handle(), &recvd_data,1); //receive data from data buffer interrupt mode
                        uart_counter = 0;
                    }
                    ++uart_counter;


                    if (osMessageQueueGet( spi_tx_queue_handle, &packet, nullptr, 50U) == osOK)
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
