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
#include <string>
#include <queue>
/* stm32 includes */
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
/* 3rd-party includes */
#include "cmsis_os2.h"
/* hal includes */
#include "../layer_0/hal_general.h"
#include "../layer_0/hal_callback.h"
#include "../layer_0/hal_wrapper.h"
#include "../layer_0/hal_spi.h"
#include "../layer_0/hal_i2c.h"
#include "../layer_0/hal.h"
#include "../layer_0/hal_irq.h"
#include "../layer_0/hal_timer.h"
#include "system_clock.h"
#include "gpio.h"
#include "spi.h"
/* driver includes */
#include "../layer_1/device.h"
#include "../layer_1/rtd.h"
#include "../layer_1/band_heater.h"
#include "../layer_1/serial_monitor.h"
/* system includes */
/* rtos includes */
#include "../layer_0/rtosal_globals.h"
#include "../layer_0/rtosal_wrapper.h"
#include "../layer_0/rtosal.h"
/* system_operation_comms_handler header */
#include "sys_op_comms_handler.h"

#include "../application/extruder.h"
#include "../meta_structure/meta_structure_system_manager.h"

#define COMMS_HANDLER_STATE_INITIALIZE      0
#define COMMS_HANDLER_STATE_RUN             1

spi::module_t spi_2_handle;
spi::module_t spi_1_handle;
i2c::handle_t i2c_2_handle;

uint32_t spi_process_send_buffer_tick_counts[200];
uint16_t spi_timer_count = 0U;
void timer_1_input_capture_zero_crossing_pulse_detected_callback(TIM_HandleTypeDef *htim)
{
    device::band_heater::zero_crossing_pulse_restart();
    output_pulse_restart(&device::zone_1_band_heater);
    output_pulse_restart(&device::zone_2_band_heater);
    output_pulse_restart(&device::zone_3_band_heater);
}

typedef union
{
    uint8_t byte[4];
    float value;
} converter_float_to_bytes_t;

typedef union
{
    uint8_t byte[4];
    uint32_t value;
} converter_uint32_to_bytes_t;



spi::module_t* get_spi_handle()
{
    return &spi_2_handle;
}

uint32_t cnt = 0;

static uint8_t i2c_count = 0U;
uint8_t current_i2c_state = 0U;
static uint32_t i2c_interval_tick = 0U;
static constexpr uint16_t I2C_INTERVAL_US = 500U;

std::queue<float> zone_1_rtd_reading_queue;
std::queue<float> zone_2_rtd_reading_queue;
std::queue<float> zone_3_rtd_reading_queue;
float zone_1_rtd_reading = 0;
float zone_2_rtd_reading = 0;
float zone_3_rtd_reading = 0;

uint8_t zone_1_rtd_reading_updated = 0U;
uint8_t zone_2_rtd_reading_updated = 0U;
uint8_t zone_3_rtd_reading_updated = 0U;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
//    memset(&user_data, '\0' , strlen(user_data)); //empty the transmission data buffer
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//    if(recvd_data == '\r')
//    {
//        data_buffer[cnt++]='\r';
//        HAL_UART_Transmit(huart, send_data, cnt,HAL_MAX_DELAY);
//        memset(data_buffer, 0, cnt);
//    }
//    else
//    {
//        data_buffer[cnt++] = recvd_data;
//    }
//    HAL_UART_Receive_IT(get_usart_2_handle(), &recvd_data,1);

}

TIM_HandleTypeDef* device::band_heater::zero_crossing_pulse_timer_module = get_timer_1_handle();

namespace sys_op::comms_handler
{
    osEventFlagsId_t  initialization_event_flags_handle = nullptr;
    rtosal::message_queue_id_t initialization_queue_handle = nullptr;
    rtosal::message_queue_id_t spi_tx_queue_handle = nullptr;
    rtosal::message_queue_id_t spi_rx_queue_handle = nullptr;
    rtosal::message_queue_id_t comms_handler_output_data_queue_handle = nullptr;
    rtosal::message_queue_id_t serial_monitor_usart_queue_handle = nullptr;

    static uint32_t i2c_iteration_tick;
    static uint32_t spi_send_iteration_tick;
    static uint32_t spi_receive_iteration_tick;
    uint32_t kernel_tick_frequency_hz;

    int16_t rtd_0_channel_id = ID_INVALID;
    int16_t rtd_1_channel_id = ID_INVALID;
    int16_t rtd_2_channel_id = ID_INVALID;

    int16_t spi_1_test_channel_id = ID_INVALID;
    common_packet_t spi_1_test_packet;
    uint8_t spi_1_test_tx_bytes[8] = { 0x01, 0x03, 0x05, 0x07, 0x00, 0x00, 0x00, 0x00  };
    uint8_t spi_t_test_bytes_per_tx[8] = {4, 0, 0, 0, 0, 0, 0, 0 };
    uint8_t spi_1_test_rx_result_array[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    common_packet_t tx_common_packet;

    common_float_data_t rtd_reading;
    uint8_t rx_d[TX_SIZE_MAX] = {0, 0, 0, 0, 0, 0, 0, 0 };
    uint8_t i2c_data[5] = { 0, 0, 0, 0, 0 };
    converter_float_to_bytes_t float_converter;
    converter_uint32_to_bytes_t uint32_converter;

    void task_intitialize()
    {

    }

    void task_state_machine()
    {
        static uint8_t comms_handler_state = COMMS_HANDLER_STATE_INITIALIZE;

        char time_stamp[9];

        spi::packet_t spi_rx_packet;
        spi::packet_t spi_1_test_rx_packet;
        volatile uint32_t spi_timer = 0U;

        switch (comms_handler_state)
        {
            case COMMS_HANDLER_STATE_INITIALIZE:
            {
                initialization_event_flags_handle   = get_initialization_event_flags_handle();
                initialization_queue_handle         = get_initialization_task_queue_handle();

                spi_tx_queue_handle                 = get_spi_2_extrusion_task_tx_queue_handle();
                spi_rx_queue_handle                 = get_spi_2_extrusion_task_rx_queue_handle();
                comms_handler_output_data_queue_handle                 = get_comms_handler_output_data_queue_handle();
                serial_monitor_usart_queue_handle   = get_serial_monitor_usart_queue_handle();

                i2c_iteration_tick = 0U;
                spi_send_iteration_tick = 0U;
                spi_receive_iteration_tick = 0U;
                kernel_tick_frequency_hz = rtosal::get_rtos_kernel_tick_frequency() * 2;

                osEventFlagsWait(initialization_event_flags_handle, READY_FOR_RESOURCE_INIT_FLAG, osFlagsWaitAny, osWaitForever);

                device::debug_serial_monitor.initialize(get_usart_2_handle(), serial_monitor_usart_queue_handle);

                hal::spi_2.initialize(&spi_2_handle, SPI_2_ID, get_timer_2_handle(), FREQUENCY_1_MHZ);
                hal::spi_2.register_callback(spi::TX_RX_COMPLETE_CALLBACK_ID, hal_callback_spi_2_tx_rx_complete);
                hal::spi_2.register_callback(spi::ERROR_CALLBACK_ID, hal_callback_spi_2_error);
                hal::spi_2.create_channel(rtd_0_channel_id, GPIO_PORT_B, GPIO_PIN_14, spi_tx_queue_handle, spi_rx_queue_handle);
                hal::spi_2.create_channel(rtd_1_channel_id, GPIO_PORT_B, GPIO_PIN_15, spi_tx_queue_handle, spi_rx_queue_handle);
                hal::spi_2.create_channel(rtd_2_channel_id, GPIO_PORT_B, GPIO_PIN_1, spi_tx_queue_handle, spi_rx_queue_handle);

                HAL_I2C_RegisterCallback(get_i2c_2_handle(), HAL_I2C_MASTER_TX_COMPLETE_CB_ID, hal_callback_i2c_controller_tx_complete);
                HAL_I2C_RegisterCallback(get_i2c_2_handle(), HAL_I2C_ERROR_CB_ID,hal_callback_i2c_controller_error);

                device::zone_1_band_heater.initialize(TEMPERATURE_ZONE_1, TIMER_10_ID, get_zone_1_band_heater_mutex_handle());
                device::zone_2_band_heater.initialize(TEMPERATURE_ZONE_2, TIMER_13_ID, get_zone_2_band_heater_mutex_handle());
                device::zone_3_band_heater.initialize(TEMPERATURE_ZONE_3, TIMER_14_ID, get_zone_3_band_heater_mutex_handle());
                HAL_TIM_RegisterCallback(get_timer_1_handle(),  HAL_TIM_IC_CAPTURE_CB_ID, timer_1_input_capture_zero_crossing_pulse_detected_callback);
                HAL_TIM_IC_Start_IT(get_timer_1_handle(), TIM_CHANNEL_2);


                MX_TIM2_Init();
                HAL_TIM_Base_Start(get_timer_2_handle());


                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);

                osEventFlagsSet(initialization_event_flags_handle, READY_FOR_USER_INIT_FLAG);



                comms_handler_state = COMMS_HANDLER_STATE_RUN;
                break;
            }
            case COMMS_HANDLER_STATE_RUN:
            {

                if (get_timer_2_handle()->Instance->CNT - spi_timer > 10U)
                {
                    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
                    spi_timer = get_timer_2_handle()->Instance->CNT;
                }

                device::debug_serial_monitor.process_send_buffer();

                device::zone_1_band_heater.update_period();
                device::zone_2_band_heater.update_period();
                device::zone_3_band_heater.update_period();

                hal::spi_2.receive_inter_task_transaction_requests(spi_tx_queue_handle, tx_common_packet);
                hal::spi_2.process_send_buffer();
                hal::spi_2.process_return_buffer(spi_rx_packet, 0, rx_d);

                HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

                hal::rtc_get_time_stamp(time_stamp);

                if (osMessageQueueGet(comms_handler_output_data_queue_handle, &rtd_reading, nullptr, 50) == osOK)
                {

                    float_converter.value = rtd_reading.value;
                    i2c_data[0] = rtd_reading.id;
                    i2c_data[1] = float_converter.byte[0];
                    i2c_data[2] = float_converter.byte[1];
                    i2c_data[3] = float_converter.byte[2];
                    i2c_data[4] = float_converter.byte[3];

                    HAL_I2C_Master_Transmit_IT(get_i2c_2_handle(), (0x14 << 1), i2c_data, 5);
                }

                if (get_timer_2_handle()->Instance->CNT - i2c_iteration_tick > 500000U)
                {

                    if (i2c_count == 0U)
                    {
                        uint32_converter.value = hal::spi_2.get_packets_requested_count();
                        i2c_data[0] = 0x07;
                        i2c_data[1] = uint32_converter.byte[0];
                        i2c_data[2] = uint32_converter.byte[1];
                        i2c_data[3] = uint32_converter.byte[2];
                        i2c_data[4] = uint32_converter.byte[3];
                        HAL_I2C_Master_Transmit_IT(get_i2c_2_handle(), (0x14 << 1), i2c_data, 5);
                        i2c_count = 1U;
                    }
                    else if (i2c_count == 1U && get_timer_2_handle()->Instance->CNT - i2c_iteration_tick > 600000U)
                    {
                        uint32_converter.value = hal::spi_2.get_packets_received_count();
                        i2c_data[0] = 0x06;
                        i2c_data[1] = uint32_converter.byte[0];
                        i2c_data[2] = uint32_converter.byte[1];
                        i2c_data[3] = uint32_converter.byte[2];
                        i2c_data[4] = uint32_converter.byte[3];
                        HAL_I2C_Master_Transmit_IT(get_i2c_2_handle(), (0x14 << 1), i2c_data, 5);
                        i2c_count = 0U;
                        i2c_iteration_tick = get_timer_2_handle()->Instance->CNT;
                    }
                }


                spi_process_send_buffer_tick_counts[spi_timer_count] = get_timer_2_handle()->Instance->CNT - spi_timer;
                if (spi_timer_count < 199)
                {
                    ++spi_timer_count;
                }
                else
                {
                    spi_timer_count = 0;
                }

                break;
            }
            default:
            {
                break;
            }

        }

    }
}

void TIM1_CC_IRQHandler()
{
    HAL_TIM_IRQHandler(get_timer_1_handle());

}

void TIM1_UP_TIM10_IRQHandler()
{
    HAL_TIM_IRQHandler(get_timer_1_handle());
    HAL_TIM_IRQHandler(get_timer_10_handle());
}

void TIM8_UP_TIM13_IRQHandler()
{
    HAL_TIM_IRQHandler(get_timer_13_handle());
}

void TIM8_TRG_COM_TIM14_IRQHandler()
{
    HAL_TIM_IRQHandler(get_timer_14_handle());
}

void SPI1_IRQHandler()
{
    spi_irq_handler(hal::get_spi_1_object());
}

void SPI2_IRQHandler()
{
    spi_irq_handler(hal::get_spi_2_object());
}

void USART2_IRQHandler()
{
    HAL_UART_IRQHandler(get_usart_2_handle());
}

void CAN1_TX_IRQHandler()
{
    HAL_CAN_IRQHandler(get_can_1_handle());
}

void CAN1_RX0_IRQHandler()
{
    HAL_CAN_IRQHandler(get_can_1_handle());
}

void CAN1_RX1_IRQHandler()
{
    HAL_CAN_IRQHandler(get_can_1_handle());
}

void CAN1_SCE_IRQHandler()
{
    HAL_CAN_IRQHandler(get_can_1_handle());
}

void I2C1_EV_IRQHandler()
{
    HAL_I2C_EV_IRQHandler(get_i2c_1_handle());
}

void I2C1_ER_IRQHandler()
{
    HAL_I2C_ER_IRQHandler(get_i2c_1_handle());
}

void I2C2_EV_IRQHandler()
{
    HAL_I2C_EV_IRQHandler(get_i2c_2_handle());
}

void I2C2_ER_IRQHandler()
{
    HAL_I2C_ER_IRQHandler(get_i2c_2_handle());
}
