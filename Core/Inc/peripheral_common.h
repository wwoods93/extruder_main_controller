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

//
// Created by wilson on 6/30/22.
//

#ifndef MAIN_CONTROLLER_PERIPHERAL_COMMON_H
#define MAIN_CONTROLLER_PERIPHERAL_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

void error_handler(void);
void Error_Handler(void);
UART_HandleTypeDef* get_usart_2_handle(void);
void initialize_peripherals(void);
void MX_WWDG_Init(void);
void MX_USART2_UART_Init(void);
#ifdef __cplusplus
}
#endif

#endif //MAIN_CONTROLLER_PERIPHERAL_COMMON_H
