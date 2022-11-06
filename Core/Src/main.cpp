/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

#include "main.h"
#include "cmsis_os.h"
#include "../driver_layer/driver_dc_motor_controller.h"
#include "../hardware_abstraction_layer/hal_spi.h"
#include "../hardware_abstraction_layer/hal_i2c.h"
#include "../driver_layer/driver_rtd.h"
#include "can.h"
#include "uart.h"
#include "spi.h"
#include "quad_spi.h"
#include "adc.h"
#include "pwm.h"
#include "icap.h"
#include "gpio.h"
#include "../rtos_abstraction_layer/rtos_task_driver.h"
#include "system_clock.h"
#include "mcu_clock_timers.h"

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
osThreadId_t initialization_taskHandle;
const osThreadAttr_t initialization_task_attributes =
{
    .name = "initialization_task",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityNormal4,
};

osThreadId_t preparation_process_taskHandle;
const osThreadAttr_t preparation_process_task_attributes =
{
    .name = "preparation_process_task",
    .stack_size = 384 * 4,
    .priority = (osPriority_t) osPriorityNormal4,
};

osThreadId_t extrusion_process_taskHandle;
const osThreadAttr_t extrusion_process_task_attributes =
{
    .name = "extrusion_process_task",
    .stack_size = 384 * 4,
    .priority = (osPriority_t) osPriorityNormal4,
};

osThreadId_t spooling_process_taskHandle;
const osThreadAttr_t spooling_process_task_attributes =
{
    .name = "spooling_process_task",
    .stack_size = 384 * 4,
    .priority = (osPriority_t) osPriorityNormal4,
};

osThreadId_t comms_updater_taskHandle;
const osThreadAttr_t comms_updater_task_attributes =
{
    .name = "comms_updater_task",
    .stack_size = 384 * 4,
    .priority = (osPriority_t) osPriorityNormal4,
};

/* Definitions for spi_tx_data_buffer_mutex */
osMutexId_t spi_tx_data_buffer_mutexHandle;
const osMutexAttr_t spi_tx_data_buffer_mutex_attributes = {
        .name = "spi_tx_data_buffer_mutex"
};
/* Definitions for spi_rx_data_buffer_mutex */
osMutexId_t spi_rx_data_buffer_mutexHandle;
const osMutexAttr_t spi_rx_data_buffer_mutex_attributes = {
        .name = "spi_rx_data_buffer_mutex"
};
/* Definitions for i2c_tx_data_buffer_mutex */
osMutexId_t i2c_tx_data_buffer_mutexHandle;
const osMutexAttr_t i2c_tx_data_buffer_mutex_attributes = {
        .name = "i2c_tx_data_buffer_mutex"
};
/* Definitions for i2c_rx_data_buffer_mutex */
osMutexId_t i2c_rx_data_buffer_mutexHandle;
const osMutexAttr_t i2c_rx_data_buffer_mutex_attributes = {
        .name = "i2c_rx_data_buffer_mutex"
};

osThreadId_t preparationProcessTaskHandle;
osThreadId_t extrusionProcessTaskHandle;
osThreadId_t spoolingProcessTaskHandle;
osThreadId_t commsUpdaterTaskHandle;





void start_initialization_task(void *argument);
void start_preparation_process_task(void *argument);
void start_extrusion_process_task(void *argument);
void start_spooling_process_task(void *argument);
void start_comms_updater_task(void *argument);

int main()
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_CAN1_Init();
    MX_SPI3_Init();
    MX_TIM6_Init();
    MX_TIM7_Init();
    MX_TIM10_Init();
    MX_UART4_Init();
    MX_QUADSPI_Init();
    MX_TIM11_Init();
    MX_TIM13_Init();
    MX_TIM14_Init();
    osKernelInitialize();

    initialization_taskHandle       = osThreadNew(start_initialization_task,        nullptr, &initialization_task_attributes);
    preparation_process_taskHandle  = osThreadNew(start_preparation_process_task,   nullptr, &preparation_process_task_attributes);
    extrusion_process_taskHandle    = osThreadNew(start_extrusion_process_task,     nullptr, &extrusion_process_task_attributes);
    spooling_process_taskHandle     = osThreadNew(start_spooling_process_task,      nullptr, &spooling_process_task_attributes);
    comms_updater_taskHandle        = osThreadNew(start_comms_updater_task,         nullptr, &comms_updater_task_attributes);

    spi_tx_data_buffer_mutexHandle  = osMutexNew(&spi_tx_data_buffer_mutex_attributes);
    spi_rx_data_buffer_mutexHandle  = osMutexNew(&spi_rx_data_buffer_mutex_attributes);
    i2c_tx_data_buffer_mutexHandle  = osMutexNew(&i2c_tx_data_buffer_mutex_attributes);
    i2c_rx_data_buffer_mutexHandle  = osMutexNew(&i2c_rx_data_buffer_mutex_attributes);

    osKernelStart();

    while (true);
}

void start_initialization_task(void *argument)
{
    for(;;)
    {


        osDelay(1);
    }
}

void start_preparation_process_task(void *argument)
{
    run_preparation_process_task_functions();
}

void start_extrusion_process_task(void *argument)
{
    run_extrusion_process_task_functions();
}

void start_spooling_process_task(void *argument)
{
    run_spooling_process_task_functions();
}


void start_comms_updater_task(void *argument)
{
    run_comms_updater_task_functions();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        HAL_IncTick();
    }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

#pragma clang diagnostic pop
