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

CAN_HandleTypeDef hcan1;
WWDG_HandleTypeDef hwwdg;
UART_HandleTypeDef huart2;
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

TIM_HandleTypeDef* get_timer_1_handle(void)
{
    return &htim1;
}

TIM_HandleTypeDef* get_timer_10_handle(void)
{
    return &htim10;
}

TIM_HandleTypeDef* get_timer_13_handle(void)
{
    return &htim13;
}

TIM_HandleTypeDef* get_timer_14_handle(void)
{
    return &htim14;
}

CAN_HandleTypeDef* get_can_1_handle(void)
{
    return &hcan1;
}

I2C_HandleTypeDef* get_i2c_1_handle(void)
{
    return &hi2c1;
}

I2C_HandleTypeDef* get_i2c_2_handle(void)
{
    return &hi2c2;
}

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
    MX_TIM1_Init();
    MX_TIM10_Init();
    MX_TIM11_Init();
    MX_TIM13_Init();
    MX_TIM14_Init();
//    MX_ADC1_Init();
//    MX_UART4_Init();
    MX_CAN1_Init();
//    MX_SPI3_Init();
//    MX_QUADSPI_Init();
//    MX_WWDG_Init();
    MX_USART2_UART_Init();
    MX_I2C2_Init();
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


void MX_CAN1_Init(void)
{

    /* USER CODE BEGIN CAN1_Init 0 */

    /* USER CODE END CAN1_Init 0 */

    /* USER CODE BEGIN CAN1_Init 1 */

    /* USER CODE END CAN1_Init 1 */
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 16;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = DISABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = DISABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN CAN1_Init 2 */

    /* USER CODE END CAN1_Init 2 */

}

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

void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}


void MX_I2C2_Init()
{
    hi2c2.Instance = I2C2;
    hi2c2.Init.ClockSpeed = 100000;
    hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK)
        Error_Handler();
}


void MX_TIM1_Init(void)
{

    /* USER CODE BEGIN TIM1_Init 0 */

    /* USER CODE END TIM1_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_IC_InitTypeDef sConfigIC = {0};

    /* USER CODE BEGIN TIM1_Init 1 */

    /* USER CODE END TIM1_Init 1 */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 64-1;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 10000-1;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM1_Init 2 */

    /* USER CODE END TIM1_Init 2 */

}

void MX_TIM10_Init(void)
{

    /* USER CODE BEGIN TIM10_Init 0 */
//////////////////////////////////////////////////////////
    /* USER CODE END TIM10_Init 0 */

    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM10_Init 1 */
//////////////////////////////////////////////////////////
    /* USER CODE END TIM10_Init 1 */
    htim10.Instance = TIM10;
    htim10.Init.Prescaler = 64-1;
    htim10.Init.CounterMode = TIM_COUNTERMODE_DOWN;
    htim10.Init.Period = 2250;
    htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_OC_Init(&htim10) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_OnePulse_Init(&htim10, TIM_OPMODE_SINGLE) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM2;
    sConfigOC.Pulse = 2000;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_OC_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM10_Init 2 */
//////////////////////////////////////////////////////////
    /* USER CODE END TIM10_Init 2 */
    HAL_TIM_MspPostInit(&htim10);

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM13_Init(void)
{

    /* USER CODE BEGIN TIM13_Init 0 */
//////////////////////////////////////////////////////////
    /* USER CODE END TIM13_Init 0 */

    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM13_Init 1 */
//////////////////////////////////////////////////////////
    /* USER CODE END TIM13_Init 1 */
    htim13.Instance = TIM13;
    htim13.Init.Prescaler = 32-1;
    htim13.Init.CounterMode = TIM_COUNTERMODE_DOWN;
    htim13.Init.Period = 2250;
    htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_OC_Init(&htim13) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_OnePulse_Init(&htim13, TIM_OPMODE_SINGLE) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM2;
    sConfigOC.Pulse = 2000;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_OC_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM13_Init 2 */
//////////////////////////////////////////////////////////
    /* USER CODE END TIM13_Init 2 */
    HAL_TIM_MspPostInit(&htim13);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM14_Init(void)
{

    /* USER CODE BEGIN TIM14_Init 0 */
//////////////////////////////////////////////////////////
    /* USER CODE END TIM14_Init 0 */

    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM14_Init 1 */
//////////////////////////////////////////////////////////
    /* USER CODE END TIM14_Init 1 */
    htim14.Instance = TIM14;
    htim14.Init.Prescaler = 32-1;
    htim14.Init.CounterMode = TIM_COUNTERMODE_DOWN;
    htim14.Init.Period = 3250;
    htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_OC_Init(&htim14) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_OnePulse_Init(&htim14, TIM_OPMODE_SINGLE) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM2;
    sConfigOC.Pulse = 2000;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_OC_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM14_Init 2 */
//////////////////////////////////////////////////////////
    /* USER CODE END TIM14_Init 2 */
    HAL_TIM_MspPostInit(&htim14);

}
