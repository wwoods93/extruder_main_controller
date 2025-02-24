/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "extruder_main.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim4;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles RCC global interrupt.
  */
void RCC_IRQHandler(void)
{
  /* USER CODE BEGIN RCC_IRQn 0 */

  /* USER CODE END RCC_IRQn 0 */
  /* USER CODE BEGIN RCC_IRQn 1 */

  /* USER CODE END RCC_IRQn 1 */
}

/**
  * @brief This function handles CAN1 TX interrupt.
  */
//void CAN1_TX_IRQHandler(void)
//{
//  /* USER CODE BEGIN CAN1_TX_IRQn 0 */
//////////////////////
//  /* USER CODE END CAN1_TX_IRQn 0 */
//  HAL_CAN_IRQHandler(&hcan1);
//  /* USER CODE BEGIN CAN1_TX_IRQn 1 */
//////////////////////
//  /* USER CODE END CAN1_TX_IRQn 1 */
//}
//
///**
//  * @brief This function handles CAN1 RX0 interrupt.
//  */
//void CAN1_RX0_IRQHandler(void)
//{
//  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */
//////////////////////
//  /* USER CODE END CAN1_RX0_IRQn 0 */
//  HAL_CAN_IRQHandler(&hcan1);
//  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */
//////////////////////
//  /* USER CODE END CAN1_RX0_IRQn 1 */
//}
//
///**
//  * @brief This function handles CAN1 RX1 interrupt.
//  */
//void CAN1_RX1_IRQHandler(void)
//{
//  /* USER CODE BEGIN CAN1_RX1_IRQn 0 */
//////////////////////
//  /* USER CODE END CAN1_RX1_IRQn 0 */
//  HAL_CAN_IRQHandler(&hcan1);
//  /* USER CODE BEGIN CAN1_RX1_IRQn 1 */
//////////////////////
//  /* USER CODE END CAN1_RX1_IRQn 1 */
//}
//
///**
//  * @brief This function handles CAN1 SCE interrupt.
//  */
//void CAN1_SCE_IRQHandler(void)
//{
//  /* USER CODE BEGIN CAN1_SCE_IRQn 0 */
//////////////////////
//  /* USER CODE END CAN1_SCE_IRQn 0 */
//  HAL_CAN_IRQHandler(&hcan1);
//  /* USER CODE BEGIN CAN1_SCE_IRQn 1 */
//////////////////////
//  /* USER CODE END CAN1_SCE_IRQn 1 */
//}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
//void TIM1_UP_TIM10_IRQHandler(void)
//{
//  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */
////////////////
//  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
//  HAL_TIM_IRQHandler(&htim1);
//  HAL_TIM_IRQHandler(&htim10);
//  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */
////////////////
//  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
//}
//
///**
//  * @brief This function handles TIM1 capture compare interrupt.
//  */
//void TIM1_CC_IRQHandler(void)
//{
//  /* USER CODE BEGIN TIM1_CC_IRQn 0 */
//////////////////
//  /* USER CODE END TIM1_CC_IRQn 0 */
//  HAL_TIM_IRQHandler(&htim1);
//  /* USER CODE BEGIN TIM1_CC_IRQn 1 */
//////////////////
//  /* USER CODE END TIM1_CC_IRQn 1 */
//}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles I2C1 event interrupt.
  */
//void I2C1_EV_IRQHandler(void)
//{
//  /* USER CODE BEGIN I2C1_EV_IRQn 0 */
////////////////////////
//  /* USER CODE END I2C1_EV_IRQn 0 */
//  HAL_I2C_EV_IRQHandler(&hi2c1);
//  /* USER CODE BEGIN I2C1_EV_IRQn 1 */
////////////////////////
//  /* USER CODE END I2C1_EV_IRQn 1 */
//}
//
///**
//  * @brief This function handles I2C1 error interrupt.
//  */
//void I2C1_ER_IRQHandler(void)
//{
//  /* USER CODE BEGIN I2C1_ER_IRQn 0 */
////////////////////////
//  /* USER CODE END I2C1_ER_IRQn 0 */
//  HAL_I2C_ER_IRQHandler(&hi2c1);
//  /* USER CODE BEGIN I2C1_ER_IRQn 1 */
////////////////////////
//  /* USER CODE END I2C1_ER_IRQn 1 */
//}
//
///**
//  * @brief This function handles I2C2 event interrupt.
//  */
//void I2C2_EV_IRQHandler(void)
//{
//  /* USER CODE BEGIN I2C2_EV_IRQn 0 */
////////////////////////
//  /* USER CODE END I2C2_EV_IRQn 0 */
//  HAL_I2C_EV_IRQHandler(&hi2c2);
//  /* USER CODE BEGIN I2C2_EV_IRQn 1 */
////////////////////////
//  /* USER CODE END I2C2_EV_IRQn 1 */
//}
//
///**
//  * @brief This function handles I2C2 error interrupt.
//  */
//void I2C2_ER_IRQHandler(void)
//{
//  /* USER CODE BEGIN I2C2_ER_IRQn 0 */
////////////////////////
//  /* USER CODE END I2C2_ER_IRQn 0 */
//  HAL_I2C_ER_IRQHandler(&hi2c2);
//  /* USER CODE BEGIN I2C2_ER_IRQn 1 */
////////////////////////
//  /* USER CODE END I2C2_ER_IRQn 1 */
//}
//
///**
//  * @brief This function handles SPI1 global interrupt.
//  */
//void SPI1_IRQHandler(void)
//{
//  /* USER CODE BEGIN SPI1_IRQn 0 */
//////
//  /* USER CODE END SPI1_IRQn 0 */
//  HAL_SPI_IRQHandler(&hspi1);
//  /* USER CODE BEGIN SPI1_IRQn 1 */
//////
//  /* USER CODE END SPI1_IRQn 1 */
//}
//
///**
//  * @brief This function handles SPI2 global interrupt.
//  */
//void SPI2_IRQHandler(void)
//{
//  /* USER CODE BEGIN SPI2_IRQn 0 */
//////////////////////////////////
//  /* USER CODE END SPI2_IRQn 0 */
//  HAL_SPI_IRQHandler(&hspi2);
//  /* USER CODE BEGIN SPI2_IRQn 1 */
//////////////////////////////////
//  /* USER CODE END SPI2_IRQn 1 */
//}
//
///**
//  * @brief This function handles USART2 global interrupt.
//  */
//void USART2_IRQHandler(void)
//{
//  /* USER CODE BEGIN USART2_IRQn 0 */
//////////////////////
//  /* USER CODE END USART2_IRQn 0 */
//  HAL_UART_IRQHandler(&huart2);
//  /* USER CODE BEGIN USART2_IRQn 1 */
//////////////////////
//  /* USER CODE END USART2_IRQn 1 */
//}
//
///**
//  * @brief This function handles TIM8 update interrupt and TIM13 global interrupt.
//  */
//void TIM8_UP_TIM13_IRQHandler(void)
//{
//  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 0 */
////////////////
//  /* USER CODE END TIM8_UP_TIM13_IRQn 0 */
//  HAL_TIM_IRQHandler(&htim13);
//  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 1 */
////////////////
//  /* USER CODE END TIM8_UP_TIM13_IRQn 1 */
//}
//
///**
//  * @brief This function handles TIM8 trigger and commutation interrupts and TIM14 global interrupt.
//  */
//void TIM8_TRG_COM_TIM14_IRQHandler(void)
//{
//  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 0 */
////////////////
//  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 0 */
//  HAL_TIM_IRQHandler(&htim14);
//  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 1 */
////////////////
//  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 1 */
//}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
