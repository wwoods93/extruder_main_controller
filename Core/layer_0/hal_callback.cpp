/***********************************************************************************************************************
 * Main_Controller
 * hal_callbacks.cpp
 *
 * wilson
 * 11/4/22
 * 12:38 AM
 *
 * Description:
 *
 **********************************************************************************************************************/

/* c/c++ includes */

/* stm32 includes */
#include "stm32f4xx_hal.h"
/* third-party includes */

/* layer_0 includes */
#include "hal.h"
/* layer_1_rtosal includes */

/* layer_1 includes */

/* layer_3_control includes */

/* layer_3 includes */

/* layer_n_meta_structure includes */

/* hal_callbacks header */
#include "hal_callback.h"

static uint32_t consecutive_i2c_bus_error_count = 0;

void hal_callback_spi_1_tx_rx_complete(spi *arg_object)
{

    if (hal::gpio_read_pin(arg_object->module->chip_select_port, arg_object->module->chip_select_pin) == GPIO_PIN_RESET)
    {
        hal::gpio_write_pin(arg_object->module->chip_select_port, arg_object->module->chip_select_pin, GPIO_PIN_SET);
    }

    arg_object->module->rx_data_ready_flag = 1U;

}

void hal_callback_spi_1_error(spi *arg_object)
{
    if (hal::gpio_read_pin(arg_object->module->chip_select_port, arg_object->module->chip_select_pin) == GPIO_PIN_RESET)
    {
        hal::gpio_write_pin(arg_object->module->chip_select_port, arg_object->module->chip_select_pin, GPIO_PIN_SET);
    }

    arg_object->module->rx_data_ready_flag = 1U;
}

void hal_callback_spi_2_tx_rx_complete(spi *arg_object)
{

    if (hal::gpio_read_pin(arg_object->module->chip_select_port, arg_object->module->chip_select_pin) == GPIO_PIN_RESET)
    {
        hal::gpio_write_pin(arg_object->module->chip_select_port, arg_object->module->chip_select_pin, GPIO_PIN_SET);
    }

    arg_object->module->rx_data_ready_flag = 1U;

}

void hal_callback_spi_2_error(spi *arg_object)
{
    if (hal::gpio_read_pin(arg_object->module->chip_select_port, arg_object->module->chip_select_pin) == GPIO_PIN_RESET)
    {
        hal::gpio_write_pin(arg_object->module->chip_select_port, arg_object->module->chip_select_pin, GPIO_PIN_SET);
    }

    arg_object->module->rx_data_ready_flag = 1U;
}


void hal_callback_i2c_controller_tx_complete(I2C_HandleTypeDef *hi2c)
{
    consecutive_i2c_bus_error_count = 0;
}

void hal_callback_i2c_controller_error(I2C_HandleTypeDef *hi2c)
{
    if ((hi2c->Instance->SR2 & I2C_SR2_BUSY) == I2C_SR2_BUSY)
    {
        consecutive_i2c_bus_error_count++;
    }
//    if (consecutive_i2c_bus_error_count > 10)
//    {
////        hi2c->Instance->CR1 |= I2C_CR1_SWRST;
////        hi2c->Instance->CR1 &= ~I2C_CR1_SWRST;
//        HAL_I2C_Master_Abort_IT(hi2c, (0x14 << 1));
//    }

    HAL_I2C_Master_Abort_IT(hi2c, (0x14 << 1));

}

void hal_callback_uart_tx_complete(UART_HandleTypeDef *huart)
{
//    memset(&user_data, '\0' , strlen(user_data)); //empty the transmission data buffer
}

void hal_callback_uart_rx_complete_callback(UART_HandleTypeDef *huart)
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


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        HAL_IncTick();
    }
}
