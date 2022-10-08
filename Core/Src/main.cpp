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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "motor.h"
#include "hal_i2c.h"
#include "can.h"
#include "uart.h"
#include "spi.h"
#include "quad_spi.h"
#include "adc.h"
#include "pwm.h"
#include "icap.h"
#include "gpio.h"
#include "os_abstraction_layer.h"
#include "system_clock.h"
#include "mcu_clock_timers.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/


/* Definitions for initialization_task */
osThreadId_t initialization_taskHandle;
const osThreadAttr_t initialization_task_attributes = {
        .name = "initialization_task",
        .stack_size = 256 * 4,
        .priority = (osPriority_t) osPriorityNormal4,
};
/* Definitions for preparation_process_task */
osThreadId_t preparation_process_taskHandle;
const osThreadAttr_t preparation_process_task_attributes = {
        .name = "preparation_process_task",
        .stack_size = 384 * 4,
        .priority = (osPriority_t) osPriorityNormal4,
};
/* Definitions for extrusion_process_task */
osThreadId_t extrusion_process_taskHandle;
const osThreadAttr_t extrusion_process_task_attributes = {
        .name = "extrusion_process_task",
        .stack_size = 384 * 4,
        .priority = (osPriority_t) osPriorityNormal4,
};
/* Definitions for spooling_process_task */
osThreadId_t spooling_process_taskHandle;
const osThreadAttr_t spooling_process_task_attributes = {
        .name = "spooling_process_task",
        .stack_size = 384 * 4,
        .priority = (osPriority_t) osPriorityNormal4,
};
/* Definitions for comms_updater_task */
osThreadId_t comms_updater_taskHandle;
const osThreadAttr_t comms_updater_task_attributes = {
        .name = "comms_updater_task",
        .stack_size = 384 * 4,
        .priority = (osPriority_t) osPriorityNormal4,
};
/* USER CODE BEGIN PV */
osThreadId_t preparationProcessTaskHandle;
osThreadId_t extrusionProcessTaskHandle;
osThreadId_t spoolingProcessTaskHandle;
osThreadId_t commsUpdaterTaskHandle;


void start_initialization_task(void *argument);
void start_preparation_process_task(void *argument);
void start_extrusion_process_task(void *argument);
void start_spooling_process_task(void *argument);
void start_comms_updater_task(void *argument);


/**
  * @brief  The application entry point.
  * @retval int
  */
int main()
{
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_CAN1_Init();
    //MX_I2C2_Init();
    MX_SPI2_Init();
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

    initialization_taskHandle         = osThreadNew(start_initialization_task,        NULL, &initialization_task_attributes);
    preparation_process_taskHandle    = osThreadNew(start_preparation_process_task,   NULL, &preparation_process_task_attributes);
    extrusion_process_taskHandle      = osThreadNew(start_extrusion_process_task,     NULL, &extrusion_process_task_attributes);
    spooling_process_taskHandle       = osThreadNew(start_spooling_process_task,      NULL, &spooling_process_task_attributes);
    comms_updater_taskHandle          = osThreadNew(start_comms_updater_task,         NULL, &comms_updater_task_attributes);

    osKernelStart();

    while (1)
    {

    }
}


/**
  * @brief  Function implementing the initialization_task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_start_initialization_task */
void start_initialization_task(void *argument)
{
    /* USER CODE BEGIN 5 */
    /* Infinite loop */
    for(;;)
    {


        osDelay(1);
    }
    // run_initialization_task_functions();
    /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_start_preparation_process_task */
/**
* @brief Function implementing the preparation_process_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_preparation_process_task */
void start_preparation_process_task(void *argument)
{
    /* USER CODE BEGIN start_preparation_process_task */
    /* Infinite loop */
    run_preparation_process_task_functions();
    /* USER CODE END start_preparation_process_task */
}

/* USER CODE BEGIN Header_start_extrusion_process_task */
/**
* @brief Function implementing the extrusion_process_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_extrusion_process_task */
void start_extrusion_process_task(void *argument)
{
    /* USER CODE BEGIN start_extrusion_process_task */
    /* Infinite loop */
    run_extrusion_process_task_functions();
    /* USER CODE END start_extrusion_process_task */
}

/* USER CODE BEGIN Header_start_spooling_process_task */
/**
* @brief Function implementing the spooling_process_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_spooling_process_task */
void start_spooling_process_task(void *argument)
{
    /* USER CODE BEGIN start_spooling_process_task */
    /* Infinite loop */
    run_spooling_process_task_functions();
    /* USER CODE END start_spooling_process_task */
}

/* USER CODE BEGIN Header_start_comms_updater_task */
/**
* @brief Function implementing the comms_updater_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_comms_updater_task */
void start_comms_updater_task(void *argument)
{
    /* USER CODE BEGIN start_comms_updater_task */
    /* Infinite loop */
    // uint8_t data_to_send[4] = { 0x01, 0x02, 0x03, 0x04 };

    timers_initialize();
    uint32_t count = 0;
    uint8_t motor_command_data[3] = { 0x05, 0x07, 0x02 };
    i2c_motor_init();
    static uint32_t led_timer = 0;
    for(;;)
    {
        //i2c_motor_set_speed(0, 100);
        if (ms_timer() - led_timer > 1000)
        {
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);

            if (count % 2 == 0)
                 i2c_motor_set_speed(0, 100);
            else
                i2c_motor_set_speed(0, 25);

            count++;
            led_timer = ms_timer();
        }

//        i2c_motor_set_speed(0, 100);
//        ms_delay(250);
//        i2c_motor_stop(0);
//        ms_delay(250);
    }
    // run_comms_updater_task_functions();
    /* USER CODE END start_comms_updater_task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
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
