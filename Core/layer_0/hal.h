/***********************************************************************************************************************
 * Main_Controller
 * peripheral_initialization.h
 *
 * wilson
 * 6/30/22
 * 7:27 PM
 *
 * Description:
 *
 **********************************************************************************************************************/


#ifndef MAIN_CONTROLLER_HAL_H
#define MAIN_CONTROLLER_HAL_H

/* c/c++ includes */

/* stm32 includes */
//#include "stm32f4xx_hal_uart.h"
/* third-party includes */

/* layer_0 includes */
#include "hal_spi.h"
/* layer_1_rtosal includes */

/* layer_1 includes */

/* layer_3_control includes */

/* layer_4_sys_op includes */

/* layer_n_meta_structure includes */


namespace hal
{
    extern spi spi_2;

    spi* get_spi_2_object();
    void i2c_build_packet_array_from_converted_bytes(uint8_t* arg_i2c_packet_array, uint8_t arg_global_id, const uint8_t* arg_converted_bytes);

    void timer_2_initialize();
}


void error_handler();
void Error_Handler();

SPI_HandleTypeDef* get_spi_1_handle();

RTC_HandleTypeDef* get_rtc_handle();
TIM_HandleTypeDef* get_timer_1_handle();
TIM_HandleTypeDef* get_timer_2_handle();
TIM_HandleTypeDef* get_timer_6_handle();
TIM_HandleTypeDef* get_timer_10_handle();
TIM_HandleTypeDef* get_timer_13_handle();
TIM_HandleTypeDef* get_timer_14_handle();

uint32_t get_timer_2_count();

CAN_HandleTypeDef* get_can_1_handle();
I2C_HandleTypeDef* get_i2c_1_handle();
I2C_HandleTypeDef* get_i2c_2_handle();
UART_HandleTypeDef* get_usart_2_handle();
void initialize_peripherals();
void can_1_initialize();
void MX_I2C1_Init();
void i2c_2_initialize();
void MX_IWDG_Init();
void MX_WWDG_Init();
void MX_USART2_UART_Init();

void MX_RTC_Init();
void MX_TIM1_Init();
void MX_TIM6_Init();
void MX_TIM10_Init();
void MX_TIM13_Init();
void MX_TIM14_Init();



#endif //MAIN_CONTROLLER_HAL_H
