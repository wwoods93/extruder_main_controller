/***********************************************************************************************************************
 * Main_Controller
 * hal_interrupt.cpp
 *
 * wilson
 * 10/31/24
 * 10:53 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

/* c/c++ includes */

/* stm32 includes */

/* third-party includes */

/* layer_0_hal includes */
#include "hal.h"
/* layer_1_rtosal includes */

/* layer_2_device includes */

/* layer_3_control includes */

/* layer_4_sys_op includes */

/* layer_n_meta_structure includes */

/* hal_interrupt header */
#include "stm32f4xx_it.h"

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
//    spi_irq_handler(hal::get_spi_1_object());
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
