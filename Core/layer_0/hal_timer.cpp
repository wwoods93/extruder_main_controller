/***********************************************************************************************************************
 * Main_Controller
 * hal_timer.cpp
 *
 * wilson
 * 10/11/24
 * 2:11 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

/* c/c++ includes */
#include <cstdint>
/* stm32 includes */
#include "stm32f4xx.h"
/* third-party includes */

/* layer_0 includes */
#include "hal.h"
#include "hal_timer.h"
/* layer_1_rtosal includes */

/* layer_1 includes */

/* layer_3_control includes */

/* layer_4_sys_op includes */

/* layer_n_meta_structure includes */

/* hal_timer header */

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim11;

static TIM_HandleTypeDef ms_timer_base_timer_handle;
static TIM_HandleTypeDef us_base_timer_handle;


void MX_TIM6_Init()
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

void MX_TIM7_Init()
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    us_base_timer_handle.Instance = TIM7;
    us_base_timer_handle.Init.Prescaler = 32;
    us_base_timer_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    us_base_timer_handle.Init.Period = 65535;
    us_base_timer_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
void MX_TIM11_Init()
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

void timers_initialize()
{
    MX_TIM6_Init();
    MX_TIM7_Init();
    MX_TIM11_Init();
}
