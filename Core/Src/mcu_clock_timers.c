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

#include "peripheral_initialization.h"
#include "mcu_clock_timers.h"

#define TIMER_RESET             0
#define MS_PER_1_SECOND         1000
#define US_DELAY_FACTOR         11

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim11;

static TIM_HandleTypeDef ms_timer_base_timer_handle;
static TIM_HandleTypeDef us_timer_base_timer_handle;

static uint32_t ms_count = 0;
static uint32_t us_count = 0;

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM6_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 32;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 65535;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM7_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim7.Instance = TIM7;
    htim7.Init.Prescaler = 32;
    htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim7.Init.Period = 65535;
    htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}


/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM11_Init(void)
{
    htim11.Instance = TIM11;
    htim11.Init.Prescaler = 64;
    htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim11.Init.Period = 65535;
    htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
    {
        Error_Handler();
    }
}

void ms_timer_base_timer_initialize(TIM_HandleTypeDef base_timer_handle)
{
    ms_timer_base_timer_handle = base_timer_handle;
}

void ms_timer_base_timer_enable(void)
{
    __HAL_TIM_ENABLE(&ms_timer_base_timer_handle);
}

void ms_timer_base_timer_disable(void)
{
    __HAL_TIM_DISABLE(&ms_timer_base_timer_handle);
}

void ms_timer_base_timer_start(void)
{
    HAL_TIM_Base_Start(&ms_timer_base_timer_handle);
}

void ms_timer_base_timer_stop(void)
{
    HAL_TIM_Base_Stop(&ms_timer_base_timer_handle);
}


void ms_timer_base_timer_reset(void)
{
    ms_timer_base_timer_handle.Instance->CNT = TIMER_RESET;
}

uint32_t ms_timer_base_timer_get_us_count()
{
    return ms_timer_base_timer_handle.Instance->CNT;
}

void ms_timer_initialize(TIM_HandleTypeDef base_timer_handle)
{
    ms_timer_base_timer_initialize(base_timer_handle);
    ms_timer_base_timer_enable();
    ms_timer_base_timer_start();
}

void ms_timer_deinitialize(void)
{
    ms_timer_base_timer_stop();
    ms_timer_base_timer_disable();
}

uint32_t ms_timer(void)
{
    if (ms_timer_base_timer_get_us_count() >= MS_PER_1_SECOND)
    {
        ms_timer_base_timer_reset();
        ms_count++;
    }
    return ms_count;
}

uint32_t ms_timer_get_count(void)
{
    return ms_count;
}

void us_timer_base_timer_initialize(TIM_HandleTypeDef base_timer_handle)
{
    us_timer_base_timer_handle = base_timer_handle;
}

void us_timer_base_timer_enable(void)
{
    __HAL_TIM_ENABLE(&us_timer_base_timer_handle);
}

void us_timer_base_timer_disable(void)
{
    __HAL_TIM_DISABLE(&us_timer_base_timer_handle);
}

void us_timer_base_timer_start(void)
{
    HAL_TIM_Base_Start(&us_timer_base_timer_handle);
}

void us_timer_base_timer_stop(void)
{
    HAL_TIM_Base_Stop(&us_timer_base_timer_handle);
}

void us_timer_base_timer_reset(void)
{
    us_timer_base_timer_handle.Instance->CNT = TIMER_RESET;
}

uint32_t us_timer_base_timer_get_us_count(void)
{
    return us_timer_base_timer_handle.Instance->CNT;
}

void us_timer_initialize(TIM_HandleTypeDef base_timer_handle)
{
    us_timer_base_timer_initialize(base_timer_handle);
    us_timer_base_timer_enable();
    us_timer_base_timer_start();
}

void us_timer_deinitialize(void)
{
    us_timer_base_timer_stop();
    us_timer_base_timer_disable();
}

uint32_t us_timer(void)
{
    us_count = us_timer_get_count();
    return us_count;
}

uint32_t us_timer_get_count(void)
{
    return us_timer_base_timer_get_us_count();
}

void us_delay(uint16_t microseconds)
{
    uint16_t delay = microseconds * US_DELAY_FACTOR;
    for (uint16_t i = 0; i <= delay; ++i);
}

void timers_initialize(void)
{
    us_timer_initialize(htim7);
    ms_timer_initialize(htim6);
}
