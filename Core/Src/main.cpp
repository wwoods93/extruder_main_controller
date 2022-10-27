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

SPI_HandleTypeDef hspi2;

void HAL_SPI_TxRxCpltCallback(spi::SPI_HandleTypeDef *hspi)
{
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET)
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
}

void HAL_SPI_ErrorCallback(spi::SPI_HandleTypeDef *hspi)
{
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET)
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
}

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

namespace hal
{
    spi spi_2;
}
namespace driver
{
    dc_motor_controller motor_controller_1;
    //rtd rtd_1(&hspi2);
}

void SPI2_IRQHandler()
{
    /* USER CODE BEGIN SPI2_IRQn 0 */

    /* USER CODE END SPI2_IRQn 0 */
    spi_irq_handler(&hal::spi_2);
    /* USER CODE BEGIN SPI2_IRQn 1 */

    /* USER CODE END SPI2_IRQn 1 */
}


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
//    driver::rtd_1.MX_SPI2_Init();
    MX_SPI3_Init();
    MX_TIM6_Init();
    MX_TIM7_Init();
    MX_TIM10_Init();
    MX_UART4_Init();
    MX_QUADSPI_Init();
    MX_TIM11_Init();
    MX_TIM13_Init();
    MX_TIM14_Init();

    hal::spi_2.configure_module(reinterpret_cast<spi::SPI_HandleTypeDef *>(&hspi2));
    hal::spi_2.spi_register_callback((spi::spi_callback_id_t )HAL_SPI_TX_RX_COMPLETE_CB_ID, HAL_SPI_TxRxCpltCallback);
    hal::spi_2.spi_register_callback((spi::spi_callback_id_t )HAL_SPI_TX_RX_COMPLETE_CB_ID, HAL_SPI_ErrorCallback);
    osKernelInitialize();

    initialization_taskHandle         = osThreadNew(start_initialization_task,        nullptr, &initialization_task_attributes);
    preparation_process_taskHandle    = osThreadNew(start_preparation_process_task,   nullptr, &preparation_process_task_attributes);
    extrusion_process_taskHandle      = osThreadNew(start_extrusion_process_task,     nullptr, &extrusion_process_task_attributes);
    spooling_process_taskHandle       = osThreadNew(start_spooling_process_task,      nullptr, &spooling_process_task_attributes);
    comms_updater_taskHandle          = osThreadNew(start_comms_updater_task,         nullptr, &comms_updater_task_attributes);

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

    uint8_t spi_byte = 0xC2;
    uint8_t rx_data = 0;

    float temp_1 = 0;
    timers_initialize();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
   // driver::motor_controller_1.initialize_controller(0x0C, dc_motor_controller::F_3921Hz, true, false);
    uint32_t count = 0;
    uint8_t motor_command_data[3] = { 0x05, 0x07, 0x02 };
    //i2c_motor_init();
    //driver::motor_controller_1.set_speed(dc_motor_controller::M1, 20);
    static uint32_t led_timer = 0;
    for(;;)
    {
        //i2c_motor_set_speed(0, 100);
        if (ms_timer() - led_timer > 999)
        {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            hal::spi_2.spi_transmit_receive_interrupt(&spi_byte, &rx_data, 1);
            //HAL_SPI_TransmitReceive_IT(&hspi2, &spi_byte, &rx_data, 1);
//            temp_1 = driver::rtd_1.read_rtd_and_calculate_temperature();
                //driver::motor_controller_1.nudge_speed_up(dc_motor_controller::M1, 10);
//                driver::motor_controller_1.set_speed(dc_motor_controller::M1, 255);
//            else
//                driver::motor_controller_1.set_speed(dc_motor_controller::M1, 50);
            //count++;
            led_timer = ms_timer();
        }
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
