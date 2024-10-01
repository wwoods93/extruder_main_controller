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


#ifndef MAIN_CONTROLLER_HAL_PERIPHERAL_H
#define MAIN_CONTROLLER_HAL_PERIPHERAL_H

#ifdef __cplusplus
extern "C" {
#endif

/* c/c++ includes */

/* stm32 includes */
#include "stm32f4xx_hal_uart.h"
/* third-party includes */

/* layer_0_hal includes */

/* layer_1_rtosal includes */

/* layer_2_device includes */

/* layer_3_control includes */

/* layer_4_sys_op includes */

/* layer_n_meta_structure includes */


void error_handler(void);
void Error_Handler(void);

SPI_HandleTypeDef* get_spi_1_handle(void);

RTC_HandleTypeDef* get_rtc_handle(void);
TIM_HandleTypeDef* get_timer_1_handle(void);
TIM_HandleTypeDef* get_timer_2_handle(void);
TIM_HandleTypeDef* get_timer_10_handle(void);
TIM_HandleTypeDef* get_timer_13_handle(void);
TIM_HandleTypeDef* get_timer_14_handle(void);

uint32_t get_timer_2_count(void);

CAN_HandleTypeDef* get_can_1_handle(void);
I2C_HandleTypeDef* get_i2c_1_handle(void);
I2C_HandleTypeDef* get_i2c_2_handle(void);
UART_HandleTypeDef* get_usart_2_handle(void);
void initialize_peripherals(void);
void MX_CAN1_Init(void);
void MX_I2C1_Init(void);
void MX_I2C2_Init();
void MX_WWDG_Init(void);
void MX_USART2_UART_Init(void);

void MX_RTC_Init(void);
void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM10_Init(void);
void MX_TIM13_Init(void);
void MX_TIM14_Init(void);

#ifdef __cplusplus
}
#endif

#endif //MAIN_CONTROLLER_HAL_PERIPHERAL_H
