/***********************************************************************************************************************
 * Main_Controller
 * mcu_clock_timers.c
 *
 * wilson
 * 6/15/22
 * 8:26 PM
 *
 * Description:
 *
 **********************************************************************************************************************/
#include "stdint.h"
#include "stm32f4xx.h"
#include "mcu_clock_timers.h"

static uint32_t ms_count = 0;

void timer_enable(TIM_HandleTypeDef *timer_handle)
{
    __HAL_TIM_ENABLE(timer_handle);
}

void timer_disable(TIM_HandleTypeDef *timer_handle)
{
    __HAL_TIM_DISABLE(timer_handle);
}

void timer_start(TIM_HandleTypeDef *timer_handle)
{
    HAL_TIM_Base_Start(timer_handle);
}

void timer_stop(TIM_HandleTypeDef *timer_handle)
{
    HAL_TIM_Base_Stop(timer_handle);
}


void timer_reset(TIM_HandleTypeDef *timer_handle)
{
    timer_handle->Instance->CNT = 0;
}

uint32_t timer_get_count(TIM_HandleTypeDef *timer_handle)
{
    return timer_handle->Instance->CNT;
}

void ms_timer(TIM_HandleTypeDef *timer_handle)
{
    if (timer_get_count(timer_handle) >= 1000)
        ms_count++;
}

uint32_t get_ms_timer_count()
{
    return ms_count;
}
