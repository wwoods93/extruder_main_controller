/***********************************************************************************************************************
 * Main_Controller
 * hal_wrapper.cpp
 *
 * wilson
 * 8/26/24
 * 12:28 AM
 *
 * Description:
 *
 **********************************************************************************************************************/

/* c/c++ includes */
#include <string>
#include <cstring>
#include <cstdio>
/* stm32 includes */
#include "stm32f4xx.h"
/* third-party includes */

/* layer_0 includes */
#include "hal.h"
/* layer_1_rtosal includes */

/* layer_1 includes */

/* layer_3_control includes */

/* layer_3 includes */

/* layer_n_meta_structure includes */

/* hal_wrapper header */
#include "hal_wrapper.h"


namespace hal
{


    status_t timer_register_callback(timer_handle_t* arg_timer_handle, timer_callback_id_t arg_callback_id_t, timer_callback_t arg_callback)
    {
        return (status_t) HAL_TIM_RegisterCallback((TIM_HandleTypeDef*)arg_timer_handle, (HAL_TIM_CallbackIDTypeDef)arg_callback_id_t, (pTIM_CallbackTypeDef) arg_callback);
    }

    status_t timer_input_capture_start_interrupt(timer_handle_t* arg_timer_handle, uint32_t arg_channel)
    {
        return (status_t)HAL_TIM_IC_Start_IT((TIM_HandleTypeDef*) arg_timer_handle, arg_channel);
    }

    status_t timer_time_base_start(timer_handle_t* arg_timer_handle)
    {
        return (status_t) HAL_TIM_Base_Start((TIM_HandleTypeDef*) arg_timer_handle);
    }





    status_t i2c_register_callback(i2c_handle_t* arg_i2c_handle, i2c_callback_id_t arg_callback_id, i2c_callback_t arg_callback)
    {
        return (status_t)HAL_I2C_RegisterCallback((I2C_HandleTypeDef*)arg_i2c_handle, (HAL_I2C_CallbackIDTypeDef)arg_callback_id, (pI2C_CallbackTypeDef)arg_callback);
    }

    status_t i2c_controller_transmit_interrupt(i2c_handle_t* arg_i2c_handle, uint16_t arg_address, uint8_t* arg_data, uint16_t arg_size)
    {
        return (status_t)HAL_I2C_Master_Transmit_IT((I2C_HandleTypeDef*)arg_i2c_handle, arg_address, arg_data, arg_size);
    }



    void gpio_write_pin(gpio_t* arg_port_name, uint16_t arg_gpio_pin, uint8_t arg_pin_state)
    {
        HAL_GPIO_WritePin((GPIO_TypeDef *)arg_port_name, arg_gpio_pin, (GPIO_PinState)arg_pin_state);
    }

    uint8_t gpio_read_pin(gpio_t* arg_port_name, uint16_t arg_gpio_pin)
    {
        return (uint8_t)HAL_GPIO_ReadPin(arg_port_name, arg_gpio_pin);
    }

    void gpio_toggle_pin(gpio_t* arg_port_name, uint16_t arg_gpio_pin)
    {
        HAL_GPIO_TogglePin((GPIO_TypeDef *)arg_port_name, arg_gpio_pin);
    }


    void rtc_get_time_stamp(char arg_time_stamp_string[9])
    {
        RTC_DateTypeDef current_date;
        RTC_TimeTypeDef current_time;


        uint8_t leading_zero = 0;
        char hours_str[3];
        char minutes_str[3];
        char seconds_str[3];
        char colon[2] = ":";

        memset(&current_time, '\0', sizeof(RTC_TimeTypeDef));
        HAL_RTC_GetTime(get_rtc_handle(), &current_time, RTC_FORMAT_BCD);
        HAL_RTC_GetDate(get_rtc_handle(), &current_date, RTC_FORMAT_BCD);

        uint8_t hours = RTC_Bcd2ToByte(current_time.Hours);
        uint8_t minutes = RTC_Bcd2ToByte(current_time.Minutes);
        uint8_t seconds = RTC_Bcd2ToByte(current_time.Seconds);

        if (hours < 10U)
        {
            sprintf(hours_str, "%d%d", leading_zero, hours);
        }
        else
        {
            sprintf(hours_str, "%d", hours);
        }

        if (minutes < 10U)
        {
            sprintf(minutes_str, "%d%d", leading_zero, minutes);
        }
        else
        {
            sprintf(minutes_str, "%d", minutes);
        }

        if (seconds < 10U)
        {
            sprintf(seconds_str, "%d%d", leading_zero, seconds);
        }
        else
        {
            sprintf(seconds_str, "%d", seconds);
        }

        sprintf(arg_time_stamp_string, "%s:%s:%s", hours_str, minutes_str, seconds_str);
    }

    void spi_1_msp_initialize()
    {
        GPIO_InitTypeDef GPIO_InitStruct = {0};

        __HAL_RCC_SPI1_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(SPI1_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(SPI1_IRQn);
}


    void spi_2_msp_initialize()
    {
        GPIO_InitTypeDef GPIO_InitStruct = {0};

        __HAL_RCC_SPI2_CLK_ENABLE();

        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_SPI2;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_2;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_13;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(SPI2_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(SPI2_IRQn);

    }

}
