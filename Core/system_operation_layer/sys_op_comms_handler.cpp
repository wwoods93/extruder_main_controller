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

void MX_TIM10_reinit(TIM_HandleTypeDef *htim);
void MX_TIM13_reinit(TIM_HandleTypeDef *htim);
void MX_TIM14_reinit(TIM_HandleTypeDef *htim);

void timer_1_input_capture_zero_crossing_pulse_detected_callback(TIM_HandleTypeDef *htim)
{
    HAL_TIM_IC_Start_IT(get_timer_1_handle(), TIM_CHANNEL_2);
    MX_TIM10_reinit(get_timer_10_handle());
    MX_TIM13_reinit(get_timer_13_handle());
    MX_TIM14_reinit(get_timer_14_handle());
    HAL_TIM_OC_Start_IT(get_timer_10_handle(), TIM_CHANNEL_1);
    HAL_TIM_OC_Start_IT(get_timer_13_handle(), TIM_CHANNEL_1);
    HAL_TIM_OC_Start_IT(get_timer_14_handle(), TIM_CHANNEL_1);
}

void timer_1_input_capture_zero_crossing_pulse_half_detected_callback(TIM_HandleTypeDef *htim)
{
//    HAL_TIM_IC_Start_IT(get_timer_1_handle(), TIM_CHANNEL_2);
//    HAL_TIM_OC_Start_IT(get_timer_10_handle(), TIM_CHANNEL_1);
//    HAL_TIM_OC_Start_IT(get_timer_13_handle(), TIM_CHANNEL_1);
//    HAL_TIM_OC_Start_IT(get_timer_14_handle(), TIM_CHANNEL_1);
}


void timer_10_output_compare_delay_elapsed_callback(TIM_HandleTypeDef *htim)
{
//    HAL_TIM_OC_Stop_IT(get_timer_10_handle(), TIM_CHANNEL_1);

//    HAL_TIM_OC_Start_IT(get_timer_10_handle(), TIM_CHANNEL_1);
}

void timer_13_output_compare_delay_elapsed_callback(TIM_HandleTypeDef *htim)
{
//    HAL_TIM_OC_Stop_IT(get_timer_13_handle(), TIM_CHANNEL_1);
//    HAL_TIM_OC_Start_IT(get_timer_13_handle(), TIM_CHANNEL_1);
}

void timer_14_output_compare_delay_elapsed_callback(TIM_HandleTypeDef *htim)
{
//    HAL_TIM_OC_Stop_IT(get_timer_14_handle(), TIM_CHANNEL_1);
//    HAL_TIM_OC_Start_IT(get_timer_14_handle(), TIM_CHANNEL_1);
}



typedef union
{
    uint8_t buffer[4];
    float numeric_param_input;
} FloatToBytes;

void TIM1_CC_IRQHandler(void)
{
    HAL_TIM_IRQHandler(get_timer_1_handle());

}

void TIM1_UP_TIM10_IRQHandler(void)
{
    /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

    /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
    HAL_TIM_IRQHandler(get_timer_1_handle());
    HAL_TIM_IRQHandler(get_timer_10_handle());






    /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

    /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

void TIM8_UP_TIM13_IRQHandler(void)
{
    /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 0 */

    /* USER CODE END TIM8_UP_TIM13_IRQn 0 */
    HAL_TIM_IRQHandler(get_timer_13_handle());
    /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 1 */


    /* USER CODE END TIM8_UP_TIM13_IRQn 1 */
}

/**
  * @brief This function handles TIM8 trigger and commutation interrupts and TIM14 global interrupt.
  */
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
    /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 0 */

    /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 0 */
    HAL_TIM_IRQHandler(get_timer_14_handle());
    /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 1 */

    /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 1 */
}

void SPI2_IRQHandler()
{
    spi_irq_handler(hal::spi_object_ptr());
}

void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(get_usart_2_handle());
}

void CAN1_TX_IRQHandler(void)
{
    HAL_CAN_IRQHandler(get_can_1_handle());
}

void CAN1_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(get_can_1_handle());
}

void CAN1_RX1_IRQHandler(void)
{
    HAL_CAN_IRQHandler(get_can_1_handle());
}

void CAN1_SCE_IRQHandler(void)
{
    HAL_CAN_IRQHandler(get_can_1_handle());
}

void I2C1_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(get_i2c_1_handle());
}

void I2C1_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(get_i2c_1_handle());
}

void I2C2_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(get_i2c_1_handle());
}

void I2C2_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(get_i2c_1_handle());
}


void MX_TIM10_reinit(TIM_HandleTypeDef *htim)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim->Instance = TIM10;
    htim->Init.Prescaler = 64-1;
    htim->Init.CounterMode = TIM_COUNTERMODE_DOWN;
    htim->Init.Period = 1000;
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(htim) != HAL_OK)
    {
        Error_Handler();
    }
//    if (HAL_TIM_OC_Init(htim) != HAL_OK)
//    {
//        Error_Handler();
//    }
//    if (HAL_TIM_OnePulse_Init(htim, TIM_OPMODE_SINGLE) != HAL_OK)
//    {
//        Error_Handler();
//    }
    sConfigOC.OCMode = TIM_OCMODE_PWM2;
    sConfigOC.Pulse = 750;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_OC_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
}

void MX_TIM13_reinit(TIM_HandleTypeDef *htim)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim->Instance = TIM13;
    htim->Init.Prescaler = 32-1;
    htim->Init.CounterMode = TIM_COUNTERMODE_DOWN;
    htim->Init.Period = 1000;
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(htim) != HAL_OK)
    {
        Error_Handler();
    }
//    if (HAL_TIM_OC_Init(htim) != HAL_OK)
//    {
//        Error_Handler();
//    }
//    if (HAL_TIM_OnePulse_Init(htim, TIM_OPMODE_SINGLE) != HAL_OK)
//    {
//        Error_Handler();
//    }
    sConfigOC.OCMode = TIM_OCMODE_PWM2;
    sConfigOC.Pulse = 750;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_OC_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
}

void MX_TIM14_reinit(TIM_HandleTypeDef *htim)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim->Instance = TIM14;
    htim->Init.Prescaler = 32-1;
    htim->Init.CounterMode = TIM_COUNTERMODE_DOWN;
    htim->Init.Period = 1000;
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(htim) != HAL_OK)
    {
        Error_Handler();
    }
//    if (HAL_TIM_OC_Init(htim) != HAL_OK)
//    {
//        Error_Handler();
//    }
//    if (HAL_TIM_OnePulse_Init(htim, TIM_OPMODE_SINGLE) != HAL_OK)
//    {
//        Error_Handler();
//    }
    sConfigOC.OCMode = TIM_OCMODE_PWM2;
    sConfigOC.Pulse = 750;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_OC_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
}



namespace driver
{
//    dc_motor_controller motor_controller_1;
//    rtd rtd_1;
}

namespace hal
{
    spi spi_2;

    spi* spi_object_ptr()
    {
        return &spi_2;
    }
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
uint32_t pulse_count = 0;
uint8_t pulse_set = 0;

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
    osMessageQueueId_t spi_rx_queue_handle = nullptr;

    static uint32_t comms_handler_iteration_tick;
    uint32_t rtos_kernel_tick_frequency_hz;

    static volatile uint8_t buffer_accessed;
    static uint8_t common_array_accessed;

    id_number_t rtd_0_channel_id = ID_INVALID;
    id_number_t rtd_1_channel_id = ID_INVALID;
    id_number_t rtd_2_channel_id = ID_INVALID;
    common_packet_t tx_common_packet;
    common_packet_t rx_common_packet;
    uint8_t rx_d[TX_SIZE_MAX] = {0, 0, 0, 0, 0, 0, 0, 0 };

    FloatToBytes converter;
    float test_temp = 27.16;
    uint8_t b1 = 0;
    uint8_t b2 = 0;
    uint8_t b3 = 0;
    uint8_t b4 = 0;

    void task_intitialize()
    {

    }

    void task_state_machine()
    {
        static uint8_t comms_handler_state = COMMS_HANDLER_STATE_INITIALIZE;

        uint8_t rtd_config_reg = 0x00 | 0x80;
        uint8_t rtd_config_byte = 0xD3;
        uint8_t tx_data_1[2] = { 0x01 & 0x7F, DUMMY_BYTE };
        uint8_t tx_data_2[2] = { 0x02 & 0x7F, DUMMY_BYTE };

        converter.numeric_param_input = test_temp;
        b1 = converter.buffer[0];
        b2 = converter.buffer[1];
        b3 = converter.buffer[2];
        b4 = converter.buffer[3];
        uint8_t i2c_test_data[5] = { 0x01, b1, b2, b3, b4};



        uint8_t packet_added = 0U;
        uint32_t rtd_reading_count = 0;

        static uint8_t uart_counter = 0;
        static uint32_t buffer_access_counter = 0;


        switch (comms_handler_state)
        {
            case COMMS_HANDLER_STATE_INITIALIZE:
            {
                initialization_event_flags_handle   = get_initialization_event_flags_handle();
                spi_tx_queue_handle                 = get_spi_tx_queue_handle();
                spi_rx_queue_handle                 = get_spi_rx_queue_handle();
                initialization_queue_handle         = get_initialization_task_queue_handle();
                osEventFlagsWait(initialization_event_flags_handle, READY_FOR_RESOURCE_INIT_FLAG, osFlagsWaitAny, osWaitForever);

                hal::spi_2.configure_module(&spi_2_handle);
                hal::spi_2.spi_register_callback(spi::SPI_TX_RX_COMPLETE_CALLBACK_ID, hal_callback_spi_rx_tx_complete);
                hal::spi_2.spi_register_callback(spi::SPI_ERROR_CALLBACK_ID, HAL_SPI_Error_Callback);
                hal::spi_2.create_channel(rtd_0_channel_id, PORT_B, GPIO_PIN_14);
                hal::spi_2.create_channel(rtd_1_channel_id, PORT_B, GPIO_PIN_15);
                hal::spi_2.create_channel(rtd_2_channel_id, PORT_B, GPIO_PIN_1);
                comms_handler_iteration_tick = 0;
                rtos_kernel_tick_frequency_hz = osKernelGetTickFreq();
                rtos_kernel_tick_frequency_hz = rtos_kernel_tick_frequency_hz;
                buffer_accessed = false;
                common_array_accessed = false;

                HAL_TIM_RegisterCallback(get_timer_1_handle(), HAL_TIM_IC_CAPTURE_CB_ID, timer_1_input_capture_zero_crossing_pulse_detected_callback);
                HAL_TIM_RegisterCallback(get_timer_1_handle(), HAL_TIM_IC_CAPTURE_HALF_CB_ID, timer_1_input_capture_zero_crossing_pulse_half_detected_callback);

                HAL_TIM_RegisterCallback(get_timer_10_handle(), HAL_TIM_OC_DELAY_ELAPSED_CB_ID, timer_10_output_compare_delay_elapsed_callback);
                HAL_TIM_RegisterCallback(get_timer_13_handle(), HAL_TIM_OC_DELAY_ELAPSED_CB_ID, timer_13_output_compare_delay_elapsed_callback);
                HAL_TIM_RegisterCallback(get_timer_14_handle(), HAL_TIM_OC_DELAY_ELAPSED_CB_ID, timer_14_output_compare_delay_elapsed_callback);


                HAL_TIM_IC_Start_IT(get_timer_1_handle(), TIM_CHANNEL_2);

//                HAL_TIM_OC_Start_IT(get_timer_10_handle(), TIM_CHANNEL_1);
//                HAL_TIM_OC_Start_IT(get_timer_13_handle(), TIM_CHANNEL_1);
//                HAL_TIM_OC_Start_IT(get_timer_14_handle(), TIM_CHANNEL_1);

                osEventFlagsSet(initialization_event_flags_handle, READY_FOR_USER_INIT_FLAG);
                comms_handler_state = COMMS_HANDLER_STATE_RUN;
                break;
            }
            case COMMS_HANDLER_STATE_RUN:
            {



                HAL_I2C_Master_Transmit(get_i2c_2_handle(), (0x14 << 1), i2c_test_data, 5, 200);

                if (pulse_count > 5 && pulse_set == 0)
                {
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
                    pulse_set = 1;
                }

                if (pulse_count > 10)
                {
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
                    pulse_set = 0;
                    pulse_count = 0;
                }
                pulse_count++;

                    if (uart_counter > 50)
                    {


                        HAL_UART_Transmit_IT(get_usart_2_handle(), (uint8_t *) user_data,strlen(user_data)); //Transmit data in interrupt mode
                        HAL_UART_Receive_IT(get_usart_2_handle(), &recvd_data,1); //receive data from data buffer interrupt mode
                        uart_counter = 0;

                     }
                    ++uart_counter;

                if (osMessageQueueGet( spi_tx_queue_handle, &tx_common_packet, nullptr, 50) == osOK)
                {
                    common_array_accessed = true;
                }

                if (common_array_accessed)
                {
                    hal::spi_2.create_packet_and_add_to_send_buffer(tx_common_packet.channel_id, tx_common_packet.total_byte_count, tx_common_packet.tx_byte_count, tx_common_packet.bytes, tx_common_packet.bytes_per_tx);
                    common_array_accessed = false;
                }

                HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

                osDelay(10);
                hal::spi_2.process_send_buffer();
                packet_added = 0U;
                spi::packet_t spi_rx_packet;
                buffer_accessed = hal::spi_2.process_return_buffer(spi_rx_packet, 0, rx_d);
                buffer_accessed = hal::spi_2.process_return_buffer(spi_rx_packet, 1, rx_d);
                buffer_accessed = hal::spi_2.process_return_buffer(spi_rx_packet, 2, rx_d);

                if (buffer_accessed)
                {
                    rtosal::build_common_packet(rx_common_packet, spi_rx_packet.channel_id, spi_rx_packet.rx_bytes, spi_rx_packet.bytes_per_tx, spi_rx_packet.total_byte_count, spi_rx_packet.tx_byte_count);
                    if (osMessageQueuePut(spi_rx_queue_handle, &rx_common_packet, 0, 0U) == osOK)
                    {
                        packet_added = 1U;
                    }

                    buffer_access_counter++;
                    buffer_accessed = 0U;
                }
                break;
            }
            default:
                break;
        }

    }
}
