/***********************************************************************************************************************
 * Main_Controller
 * hal_peripheral.cpp
 *
 * wilson
 * 10/11/24
 * 1:52 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

/* c/c++ includes */

/* stm32 includes */
#include "stm32f4xx.h"
/* third-party includes */

/* layer_0 includes */
#include "hal.h"
#include "hal_spi.h"
/* layer_1 includes */

/* layer_2 includes */

/* layer_3 includes */



#include "system_clock.h"
#include "gpio.h"
#include "pwm.h"


CAN_HandleTypeDef hcan1;
WWDG_HandleTypeDef hwwdg;
IWDG_HandleTypeDef hiwdg;
UART_HandleTypeDef huart2;
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
SPI_HandleTypeDef hspi1;
RTC_HandleTypeDef hrtc;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim11;

static TIM_HandleTypeDef ms_timer_base_timer_handle;
static TIM_HandleTypeDef us_base_timer_handle;

namespace hal
{
    void initialize_peripherals()
    {
        HAL_Init();
        SystemClock_Config();

        MX_GPIO_Init();
        rtc_initialize();
        tim_1_initialize();
        tim_7_initialize();
        tim_10_initialize();
        tim_11_initialize();
        tim_13_initialize();
        tim_14_initialize();
        can_1_initialize();
        usart_2_initialize();
        i2c_2_initialize();
    }

    spi spi_2;

    spi* get_spi_2_object()
    {
        return &spi_2;
    }

    void i2c_build_packet_array_from_uint8_array(uint8_t* arg_i2c_packet_array, uint8_t arg_global_id, const uint8_t* arg_converted_bytes)
    {
        arg_i2c_packet_array[0] = arg_global_id;
        arg_i2c_packet_array[1] = arg_converted_bytes[0];
        arg_i2c_packet_array[2] = arg_converted_bytes[1];
        arg_i2c_packet_array[3] = arg_converted_bytes[2];
        arg_i2c_packet_array[4] = arg_converted_bytes[3];
    }

    void tim_1_initialize()
    {
        TIM_MasterConfigTypeDef sMasterConfig = {0};
        TIM_IC_InitTypeDef sConfigIC = {0};

        htim1.Instance = TIM1;
        htim1.Init.Prescaler = 64-1;
        htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
        htim1.Init.Period = 10000-1;
        htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        htim1.Init.RepetitionCounter = 0;
        htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
        if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
        {
            error_handler();
        }
        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
        {
            error_handler();
        }
        sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
        sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
        sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
        sConfigIC.ICFilter = 0;
        if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
        {
            error_handler();
        }
    }

    void tim_2_initialize()
    {
        TIM_ClockConfigTypeDef sClockSourceConfig = {0};
        TIM_MasterConfigTypeDef sMasterConfig = {0};

        htim2.Instance = TIM2;
        htim2.Init.Prescaler = 31U;
        htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
        htim2.Init.Period = 4294967295U;
        htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
        if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
        {
            error_handler();
        }
        sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
        if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
        {
            error_handler();
        }
        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
        {
            error_handler();
        }
    }

    void tim_6_initialize()
    {
        TIM_MasterConfigTypeDef sMasterConfig = {0};

        htim6.Instance = TIM6;
        htim6.Init.Prescaler = 32000U - 1U;
        htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
        htim6.Init.Period = 65535U;
        htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
        if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
        {
            error_handler();
        }
        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
        {
            error_handler();
        }
    }

    void tim_7_initialize()
    {
        TIM_MasterConfigTypeDef sMasterConfig = {0};

        us_base_timer_handle.Instance = TIM7;
        us_base_timer_handle.Init.Prescaler = 32;
        us_base_timer_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
        us_base_timer_handle.Init.Period = 65535;
        us_base_timer_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
        if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
        {
            error_handler();
        }
        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
        {
            error_handler();
        }
    }

    void tim_10_initialize()
    {
        TIM_OC_InitTypeDef sConfigOC = {0};

        htim10.Instance = TIM10;
        htim10.Init.Prescaler = 64-1;
        htim10.Init.CounterMode = TIM_COUNTERMODE_DOWN;
        htim10.Init.Period = 1000;
        htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

        if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
        {
            error_handler();
        }
        if (HAL_TIM_OC_Init(&htim10) != HAL_OK)
        {
            error_handler();
        }

        if (HAL_TIM_OnePulse_Init(&htim10, TIM_OPMODE_SINGLE) != HAL_OK)
        {
            error_handler();
        }

        sConfigOC.OCMode = TIM_OCMODE_PWM2;
        sConfigOC.Pulse = 750;
        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
        sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

        if (HAL_TIM_OC_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
        {
            error_handler();
        }

        HAL_TIM_MspPostInit(&htim10);
    }

    void tim_11_initialize()
    {
        htim11.Instance = TIM11;
        htim11.Init.Prescaler = 64;
        htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
        htim11.Init.Period = 65535;
        htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
        if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
        {
            error_handler();
        }
    }

    void tim_13_initialize()
    {
        TIM_OC_InitTypeDef sConfigOC = {0};

        htim13.Instance = TIM13;
        htim13.Init.Prescaler = 32-1;
        htim13.Init.CounterMode = TIM_COUNTERMODE_DOWN;
        htim13.Init.Period = 1000;
        htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

        if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
        {
            error_handler();
        }
        if (HAL_TIM_OC_Init(&htim13) != HAL_OK)
        {
            error_handler();
        }
        if (HAL_TIM_OnePulse_Init(&htim13, TIM_OPMODE_SINGLE) != HAL_OK)
        {
            error_handler();
        }

        sConfigOC.OCMode = TIM_OCMODE_PWM2;
        sConfigOC.Pulse = 750;
        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
        sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

        if (HAL_TIM_OC_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
        {
            error_handler();
        }

        HAL_TIM_MspPostInit(&htim13);

    }

    void tim_14_initialize()
    {
        TIM_OC_InitTypeDef sConfigOC = {0};

        htim14.Instance = TIM14;
        htim14.Init.Prescaler = 32-1;
        htim14.Init.CounterMode = TIM_COUNTERMODE_DOWN;
        htim14.Init.Period = 1000;
        htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

        if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
        {
            error_handler();
        }
        if (HAL_TIM_OC_Init(&htim14) != HAL_OK)
        {
            error_handler();
        }
        if (HAL_TIM_OnePulse_Init(&htim14, TIM_OPMODE_SINGLE) != HAL_OK)
        {
            error_handler();
        }

        sConfigOC.OCMode = TIM_OCMODE_PWM2;
        sConfigOC.Pulse = 750;
        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
        sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

        if (HAL_TIM_OC_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
        {
            error_handler();
        }

        HAL_TIM_MspPostInit(&htim14);
    }


    timer_handle_t* tim_1_get_handle()
    {
        return static_cast<hal::timer_handle_t*>(&htim1);
    }

    timer_handle_t* tim_2_get_handle()
    {
        return static_cast<hal::timer_handle_t*>(&htim2);
    }

    timer_handle_t* tim_6_get_handle()
    {
        return static_cast<hal::timer_handle_t*>(&htim6);
    }

    timer_handle_t* tim_10_get_handle()
    {
        return static_cast<hal::timer_handle_t*>(&htim10);
    }

    timer_handle_t* tim_13_get_handle()
    {
        return static_cast<hal::timer_handle_t*>(&htim13);
    }

    timer_handle_t* tim_14_get_handle()
    {
        return static_cast<hal::timer_handle_t*>(&htim14);
    }

    uint32_t timer_2_get_count()
    {
        return htim2.Instance->CNT;
    }

    uint32_t timer_6_get_count()
    {
        return htim6.Instance->CNT;
    }

    uint32_t timer_get_count(hal::timer_handle_t* arg_timer_handle)
    {
        return arg_timer_handle->Instance->CNT;
    }

}

SPI_HandleTypeDef* spi_1_get_handle()
{
    return &hspi1;
}

RTC_HandleTypeDef* rtc_get_handle()
{
    return &hrtc;
}

CAN_HandleTypeDef* can_1_get_handle()
{
    return &hcan1;
}

I2C_HandleTypeDef* i2c_1_get_handle()
{
    return &hi2c1;
}

I2C_HandleTypeDef* i2c_2_get_handle()
{
    return &hi2c2;
}

UART_HandleTypeDef* usart_2_get_handle()
{
    return &huart2;
}

void error_handler()
{
    __disable_irq();
    while (1)
    {
    }
}

void Error_Handler()
{
    __disable_irq();
    while (1)
    {
    }
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
//        error_handler();
//    }
//    /* USER CODE BEGIN WWDG_Init 2 */
//
//    /* USER CODE END WWDG_Init 2 */
//
//}


void can_1_initialize()
{
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
        error_handler();
    }
}

void MX_IWDG_Init()
{
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
    hiwdg.Init.Reload = 4095;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
    {
        error_handler();
    }
}

void MX_WWDG_Init()
{
    hwwdg.Instance = WWDG;
    hwwdg.Init.Prescaler = WWDG_PRESCALER_1;
    hwwdg.Init.Window = 64;
    hwwdg.Init.Counter = 64;
    hwwdg.Init.EWIMode = WWDG_EWI_ENABLE;
    if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
    {
        error_handler();
    }
}

void usart_2_initialize()
{
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
        error_handler();
    }
}

void i2c_1_initialize()
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
        error_handler();
    }
}

void i2c_2_initialize()
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
    {
        error_handler();
    }
}

void rtc_initialize()
{
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    hrtc.Instance = RTC;
    hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
    hrtc.Init.AsynchPrediv = 127;
    hrtc.Init.SynchPrediv = 255;
    hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
    hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;

    if (HAL_RTC_Init(&hrtc) != HAL_OK)
    {
        error_handler();
    }

    sTime.Hours = 0x10;
    sTime.Minutes = 0x0;
    sTime.Seconds = 0x0;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;

    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
    {
        error_handler();
    }

    sDate.WeekDay = RTC_WEEKDAY_MONDAY;
    sDate.Month = RTC_MONTH_JANUARY;
    sDate.Date = 0x1;
    sDate.Year = 24;

    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
    {
        error_handler();
    }
}


void assert_failed(uint8_t *file, uint32_t line)
{

}
