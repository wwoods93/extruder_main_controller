/***********************************************************************************************************************
 * Main_Controller
 * mcu_clock_timers.h
 *
 * wilson
 * 6/15/22
 * 8:26 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

#include "stm32f4xx.h"

#ifndef MAIN_CONTROLLER_MCU_CLOCK_TIMERS_H
#define MAIN_CONTROLLER_MCU_CLOCK_TIMERS_H

void timer_enable(TIM_HandleTypeDef *timer_handle);
void timer_disable(TIM_HandleTypeDef *timer_handle);
void timer_start(TIM_HandleTypeDef *timer_handle);
void timer_stop(TIM_HandleTypeDef *timer_handle);
void timer_reset(TIM_HandleTypeDef *timer_handle);
uint32_t timer_get_count(TIM_HandleTypeDef *timer_handle);
void ms_timer(TIM_HandleTypeDef *timer_handle);
uint32_t get_ms_timer_count();

#endif //MAIN_CONTROLLER_MCU_CLOCK_TIMERS_H
