/***********************************************************************************************************************
 * Main_Controller
 * peripheral_common.c
 *
 * wilson
 * 6/30/22
 * 7:27 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

#include "stm32f4xx.h"
#include "system_clock.h"
#include "gpio.h"
#include "mcu_clock_timers.h"
#include "pwm.h"
#include "icap.h"
#include "adc.h"
#include "uart.h"
#include "can.h"
#include "spi.h"
#include "quad_spi.h"
#include "peripheral_common.h"

WWDG_HandleTypeDef hwwdg;
UART_HandleTypeDef huart2;

UART_HandleTypeDef* get_usart_2_handle(void)
{
    return &huart2;
}

void error_handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}


void initialize_peripherals(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_TIM6_Init();
    MX_TIM7_Init();
    MX_TIM10_Init();
    MX_TIM11_Init();
    MX_TIM13_Init();
    MX_TIM14_Init();
//    MX_ADC1_Init();
//    MX_UART4_Init();
//    MX_CAN1_Init();
//    MX_SPI3_Init();
//    MX_QUADSPI_Init();
//    MX_WWDG_Init();
    MX_USART2_UART_Init();
}

//void MX_WWDG_Init(void)
//{
//
//    /* USER CODE BEGIN WWDG_Init 0 */
//
//    /* USER CODE END WWDG_Init 0 */
//
//    /* USER CODE BEGIN WWDG_Init 1 */
//
//    /* USER CODE END WWDG_Init 1 */
//    hwwdg.Instance = WWDG;
//    hwwdg.Init.Prescaler = WWDG_PRESCALER_1;
//    hwwdg.Init.Window = 64;
//    hwwdg.Init.Counter = 64;
//    hwwdg.Init.EWIMode = WWDG_EWI_ENABLE;
//    if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
//    {
//        Error_Handler();
//    }
//    /* USER CODE BEGIN WWDG_Init 2 */
//
//    /* USER CODE END WWDG_Init 2 */
//
//}

void MX_WWDG_Init(void)
{

    /* USER CODE BEGIN WWDG_Init 0 */
////////////////
    /* USER CODE END WWDG_Init 0 */

    /* USER CODE BEGIN WWDG_Init 1 */
////////////////
    /* USER CODE END WWDG_Init 1 */
    hwwdg.Instance = WWDG;
    hwwdg.Init.Prescaler = WWDG_PRESCALER_1;
    hwwdg.Init.Window = 64;
    hwwdg.Init.Counter = 64;
    hwwdg.Init.EWIMode = WWDG_EWI_ENABLE;
    if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN WWDG_Init 2 */
////////////////
    /* USER CODE END WWDG_Init 2 */

}

void MX_USART2_UART_Init(void)
{

    /* USER CODE BEGIN USART2_Init 0 */
//
    /* USER CODE END USART2_Init 0 */

    /* USER CODE BEGIN USART2_Init 1 */
//
    /* USER CODE END USART2_Init 1 */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */
//
    /* USER CODE END USART2_Init 2 */

}
