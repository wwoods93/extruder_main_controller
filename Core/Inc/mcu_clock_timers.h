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

void MX_TIM6_Init(void);
void MX_TIM7_Init(void);
void MX_TIM11_Init(void);

void ms_timer_base_timer_initialize(TIM_HandleTypeDef base_timer_handle);
void ms_timer_base_timer_enable(void);
void ms_timer_base_timer_disable(void);
void ms_timer_base_timer_start(void);
void ms_timer_base_timer_stop(void);
void ms_timer_base_timer_reset(void);
uint32_t ms_timer_base_timer_get_us_count(void);
void ms_timer_initialize(TIM_HandleTypeDef base_timer_handle);
void ms_timer_deinitialize(void);
uint32_t ms_timer(void);
uint32_t ms_timer_get_count(void);

void us_timer_base_timer_initialize(TIM_HandleTypeDef base_timer_handle);
void us_timer_base_timer_enable(void);
void us_timer_base_timer_disable(void);
void us_timer_base_timer_start(void);
void us_timer_base_timer_stop(void);
void us_timer_base_timer_reset(void);
uint32_t us_timer_base_timer_get_us_count(void);
void us_timer_initialize(TIM_HandleTypeDef base_timer_handle);
void us_timer_deinitialize(void);
uint32_t us_timer(void);
uint32_t us_timer_get_count(void);

void us_delay(uint16_t microseconds);

void timers_initialize(void);


#endif //MAIN_CONTROLLER_MCU_CLOCK_TIMERS_H
