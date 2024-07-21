/***********************************************************************************************************************
 * Main_Controller
 * adc.c
 *
 * wilson
 * 7/12/22
 * 8:39 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

//
// Created by wilson on 7/12/22.
//

#include "stm32f4xx.h"

#include "peripheral_common.h"

#include "adc.h"

//ADC_HandleTypeDef hadc1;
//
//void MX_ADC1_Init(void)
//{
//
//    /* USER CODE BEGIN ADC1_Init 0 */
//
//    /* USER CODE END ADC1_Init 0 */
//
//    ADC_ChannelConfTypeDef sConfig = {0};
//
//    /* USER CODE BEGIN ADC1_Init 1 */
//
//    /* USER CODE END ADC1_Init 1 */
//
//    /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
//    */
//    hadc1.Instance = ADC1;
//    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
//    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
//    hadc1.Init.ScanConvMode = DISABLE;
//    hadc1.Init.ContinuousConvMode = DISABLE;
//    hadc1.Init.DiscontinuousConvMode = DISABLE;
//    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
//    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
//    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
//    hadc1.Init.NbrOfConversion = 1;
//    hadc1.Init.DMAContinuousRequests = DISABLE;
//    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
//    if (HAL_ADC_Init(&hadc1) != HAL_OK)
//    {
//        Error_Handler();
//    }
//
//    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//    */
//    sConfig.Channel = ADC_CHANNEL_1;
//    sConfig.Rank = 1;
//    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
//    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//    {
//        Error_Handler();
//    }
//    /* USER CODE BEGIN ADC1_Init 2 */
//
//    /* USER CODE END ADC1_Init 2 */
//
//}
