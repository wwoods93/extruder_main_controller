/***********************************************************************************************************************
 * Main_Controller
 * device_band_heater.cpp
 *
 * wilson
 * 9/17/24
 * 10:39 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

/* c/c++ includes */

/* stm32 includes */
#include "stm32f4xx.h"
/* third-party includes */

/* layer_0_hal includes */
#include "../layer_0_hal/hal_general.h"
#include "../layer_0_hal/hal_peripheral.h"
/* layer_1_rtosal includes */

/* layer_2_device includes */

/* layer_3_control includes */

/* layer_4_sys_op includes */

/* layer_n_meta_structure includes */


#include "../application/extruder.h"

/* device_band_heater header */
#include "heating_element.h"


/*
 *
period  image 1     image 2     image 3
 8300   26          27
 8250   28          29
 8200   30          31
 8150   32          33
 8100   34          35
 8000   36          37
 7900   38          39
 7800   40          41
 7700   42          43
 7600   44          45
 7500   46          47
 7400   48          49
 7300   50          51
 7200   52          53
 7100   54          55
 7000   56          57
 6900   58          59
 6800   60          61
 6700   62          63
 6600   64          65
 6500   66          67
 6400   68          69
 6300   70          71
 6200   72          73
 6100   74          75
 6000   76          77
 5900   78          79
 5800   80          81
 5700   82          83
 5600   84          85
 5500   86          87
 5400   88          89
 5300   90          91
 5200   92          93
 5100   94          95
 5000   96          97
 4900   98          99
 4800   100         101
 4700   102         103
 4600   104         105
 4500   106         107
 4400   108         109
 4300   110         111
 4200   112         114
 4100   115         116
 4000   117         118
 3900   119         120
 3800   121         122
 3700   123         125
 3600   126         127
 3500   128         129
 3400   130         131
 3300   132         133
 3200   134         135
 3100   136         137
 3000   138         139
 2900   140         141
 2800   142         143
 2700   144         145
 2600   146         147
 2500   148         149
 2400   150         151
 2300   152         153
 2200   154         155
 2100   156         157
 2000   158         159         160
 1900   161         162         163
 1800   164         165         166
 1700   167         168         169
 1600   170         171         172
 1500   173         174     `   175
 1400   176         177         178
 1300   179         181         182
 1200   183         184         185
 1100   186         187         188
 1000   189         190         191
 900    192         193         194
 800    195         196         197
 700    198         199         200
 600    201         202         203
 500    // unstable at 00



 */

uint32_t heating_element::initialize(uint8_t arg_zero_crossing_pulse_timer_id, uint8_t arg_zone_1_output_pulse_timer_id, uint8_t arg_zone_2_output_pulse_timer_id, uint8_t arg_zone_3_output_pulse_timer_id)
{
    if (arg_zero_crossing_pulse_timer_id == TIMER_1_ID)
    {
        MX_TIM1_Init();
        zero_crossing_pulse_timer_module = get_timer_1_handle();
    }

    initialize_output_pulse_timer(TEMPERATURE_ZONE_1, arg_zone_1_output_pulse_timer_id);
    initialize_output_pulse_timer(TEMPERATURE_ZONE_2, arg_zone_2_output_pulse_timer_id);
    initialize_output_pulse_timer(TEMPERATURE_ZONE_3, arg_zone_3_output_pulse_timer_id);

    return 0;
}

uint32_t heating_element::initialize_output_pulse_timer(uint8_t arg_temperature_zone_id, uint8_t arg_output_pulse_timer_id)
{
    switch (arg_output_pulse_timer_id)
    {
        case TIMER_10_ID:
        {
            MX_TIM10_Init();
            switch (arg_temperature_zone_id)
            {
                case TEMPERATURE_ZONE_1:
                {
                    zone_1_output_pulse_timer_module = get_timer_10_handle();
                    break;
                }
                case TEMPERATURE_ZONE_2:
                {
                    zone_2_output_pulse_timer_module = get_timer_10_handle();
                    break;
                }
                case TEMPERATURE_ZONE_3:
                {
                    zone_3_output_pulse_timer_module = get_timer_10_handle();
                    break;
                }
                default:
                {
                    break;
                }
            }
            break;
        }
        case TIMER_13_ID:
        {
            MX_TIM13_Init();
            switch (arg_temperature_zone_id)
            {
                case TEMPERATURE_ZONE_1:
                {
                    zone_1_output_pulse_timer_module = get_timer_13_handle();
                    break;
                }
                case TEMPERATURE_ZONE_2:
                {
                    zone_2_output_pulse_timer_module = get_timer_13_handle();
                    break;
                }
                case TEMPERATURE_ZONE_3:
                {
                    zone_3_output_pulse_timer_module = get_timer_13_handle();
                    break;
                }
                default:
                {
                    break;
                }
            }
            break;
        }
        case TIMER_14_ID:
        {
            MX_TIM14_Init();
            switch (arg_temperature_zone_id)
            {
                case TEMPERATURE_ZONE_1:
                {
                    zone_1_output_pulse_timer_module = get_timer_14_handle();
                    break;
                }
                case TEMPERATURE_ZONE_2:
                {
                    zone_2_output_pulse_timer_module = get_timer_14_handle();
                    break;
                }
                case TEMPERATURE_ZONE_3:
                {
                    zone_3_output_pulse_timer_module = get_timer_14_handle();
                    break;
                }
                default:
                {
                    break;
                }
            }
            break;
        }
        default:
        {
            break;
        }
    }

    return 0;
}


void output_pulse_restart(heating_element* arg_heating_element, uint8_t arg_temperature_zone_id, uint16_t arg_period)
{


    TIM_OC_InitTypeDef sConfigOC = {0};

    sConfigOC.OCMode = TIM_OCMODE_PWM2;
    sConfigOC.Pulse = arg_period - heating_element::OUTPUT_PULSE_WIDTH;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    switch (arg_temperature_zone_id)
    {
        case TEMPERATURE_ZONE_1:
        {
            arg_heating_element->zone_1_output_pulse_timer_module->Init.Period = arg_period;

            if (HAL_TIM_Base_Init(arg_heating_element->zone_1_output_pulse_timer_module) != HAL_OK)
            {
                Error_Handler();
            }

            if (HAL_TIM_OC_ConfigChannel(arg_heating_element->zone_1_output_pulse_timer_module, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
            {
                Error_Handler();
            }

            break;
        }
        case TEMPERATURE_ZONE_2:
        {

            arg_heating_element->zone_2_output_pulse_timer_module->Init.Period = arg_period;
            if (HAL_TIM_Base_Init(arg_heating_element->zone_2_output_pulse_timer_module) != HAL_OK)
            {
                Error_Handler();
            }

            if (HAL_TIM_OC_ConfigChannel(arg_heating_element->zone_2_output_pulse_timer_module, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
            {
                Error_Handler();
            }
            break;
        }
        case TEMPERATURE_ZONE_3:
        {
            arg_heating_element->zone_3_output_pulse_timer_module->Init.Period = arg_period;

            if (HAL_TIM_Base_Init(arg_heating_element->zone_3_output_pulse_timer_module) != HAL_OK)
            {
                Error_Handler();
            }

            if (HAL_TIM_OC_ConfigChannel(arg_heating_element->zone_3_output_pulse_timer_module, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
            {
                Error_Handler();
            }

            break;
        }
        default:
        {
            break;
        }
    }
}

TIM_HandleTypeDef* heating_element::get_zero_crossing_pulse_timer_module()
{
    return zero_crossing_pulse_timer_module;
}

TIM_HandleTypeDef* heating_element::get_zone_1_output_pulse_timer_module()
{
    return zone_1_output_pulse_timer_module;
}

TIM_HandleTypeDef* heating_element::get_zone_2_output_pulse_timer_module()
{
    return zone_2_output_pulse_timer_module;
}

TIM_HandleTypeDef* heating_element::get_zone_3_output_pulse_timer_module()
{
    return zone_3_output_pulse_timer_module;
}
