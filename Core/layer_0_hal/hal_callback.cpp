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

/* hal includes */
#include "hal_general.h"
/* driver includes */

/* rtos abstraction includes */

/* sys op includes */

/* meta structure includes */

/* hal_callbacks header */
#include "hal_callback.h"



void hal_callback_spi_rx_tx_complete(spi::module_t *arg_module)
{

    if (hal::gpio_read_pin(arg_module->chip_select_port, arg_module->chip_select_pin) == GPIO_PIN_RESET)
    {
        hal::gpio_write_pin(arg_module->chip_select_port, arg_module->chip_select_pin, GPIO_PIN_SET);
    }

    arg_module->rx_data_ready_flag = 1U;

}

void hal_callback_spi_error(spi::module_t *arg_module)
{
    if (hal::gpio_read_pin(arg_module->chip_select_port, arg_module->chip_select_pin) == GPIO_PIN_RESET)
    {
        hal::gpio_write_pin(arg_module->chip_select_port, arg_module->chip_select_pin, GPIO_PIN_SET);
    }

    arg_module->rx_data_ready_flag = 1U;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        HAL_IncTick();
    }
}
