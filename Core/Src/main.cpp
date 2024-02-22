/***********************************************************************************************************************
 * Main_Controller
 * main.cpp
 *
 * wilson
 * 11/4/22
 * 12:38 AM
 *
 * Description:
 *
 **********************************************************************************************************************/

#include "extruder_main.h"
#include "main.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "peripheral_common.h"
#include "mcu_clock_timers.h"
#include "../system_operation_layer/sys_op_comms_handler.h"
#include "../hardware_abstraction_layer/hal_spi.h"

#include "../hardware_abstraction_layer/hal_callbacks.h"

#define KERNEL_TICKS_PER_1_SECOND       1019

osThreadId_t initialization_taskHandle;
osThreadId_t preparation_process_taskHandle;
osThreadId_t extrusion_process_taskHandle;
osThreadId_t spooling_process_taskHandle;
osThreadId_t comms_handler_taskHandle;

osMutexId_t spi_tx_data_buffer_mutexHandle;
osMutexId_t spi_rx_data_buffer_mutexHandle;
osMutexId_t i2c_tx_data_buffer_mutexHandle;
osMutexId_t i2c_rx_data_buffer_mutexHandle;

const osThreadAttr_t initialization_task_attributes = { .name = "initialization_task",      .stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal, };
const osThreadAttr_t preparation_task_attributes    = { .name = "preparation_process_task", .stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal, };
const osThreadAttr_t extrusion_task_attributes      = { .name = "extrusion_process_task",   .stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal, };
const osThreadAttr_t spooling_task_attributes       = { .name = "spooling_process_task",    .stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal, };
const osThreadAttr_t comms_handler_task_attributes  = { .name = "comms_handler_task",       .stack_size = 700 * 4, .priority = (osPriority_t) osPriorityNormal4, };

const osMutexAttr_t spi_tx_data_buffer_mutex_attributes = { .name = "spi_tx_data_buffer_mutex" };
const osMutexAttr_t spi_rx_data_buffer_mutex_attributes = { .name = "spi_rx_data_buffer_mutex" };
const osMutexAttr_t i2c_tx_data_buffer_mutex_attributes = { .name = "i2c_tx_data_buffer_mutex" };
const osMutexAttr_t i2c_rx_data_buffer_mutex_attributes = { .name = "i2c_rx_data_buffer_mutex" };

void start_initialization_task(void *argument);
void start_preparation_process_task(void *argument);
void start_extrusion_process_task(void *argument);
void start_spooling_process_task(void *argument);
void start_comms_handler_task(void *argument);

int main()
{
    initialize_peripherals();
    timers_initialize();
    osKernelInitialize();

    spi_tx_data_buffer_mutexHandle  = osMutexNew(&spi_tx_data_buffer_mutex_attributes);
    spi_rx_data_buffer_mutexHandle  = osMutexNew(&spi_rx_data_buffer_mutex_attributes);
    i2c_tx_data_buffer_mutexHandle  = osMutexNew(&i2c_tx_data_buffer_mutex_attributes);
    i2c_rx_data_buffer_mutexHandle  = osMutexNew(&i2c_rx_data_buffer_mutex_attributes);

    initialization_taskHandle       = osThreadNew(start_initialization_task,        nullptr, &initialization_task_attributes);
    preparation_process_taskHandle  = osThreadNew(start_preparation_process_task,   nullptr, &preparation_task_attributes);
    extrusion_process_taskHandle    = osThreadNew(start_extrusion_process_task,     nullptr, &extrusion_task_attributes);
    spooling_process_taskHandle     = osThreadNew(start_spooling_process_task,      nullptr, &spooling_task_attributes);
    comms_handler_taskHandle        = osThreadNew(start_comms_handler_task,         nullptr, &comms_handler_task_attributes);

    osKernelStart();
    while (true);
}

void start_initialization_task(void *argument)
{

}

void start_preparation_process_task(void *argument)
{

}

void start_extrusion_process_task(void *argument)
{

}

void start_spooling_process_task(void *argument)
{
    uint32_t spooling_process_iteration_tick = 0;
    uint32_t kernel_tick = 0;

//    while (true)
//    {
//        kernel_tick = osKernelGetTickCount();
//        if (kernel_tick - spooling_process_iteration_tick > KERNEL_TICKS_PER_1_SECOND)
//        {
//            //sys_op::spooling_process_state_machine();
//            spooling_process_iteration_tick = kernel_tick;
//        }
//    }
}

void start_comms_handler_task(void *argument)
{

//    uint32_t kernel_tick = 0;

    while (true)
    {
//        if (osKernelGetTickCount() - comms_handler_iteration_tick > RTOS_KERNEL_TICK_FREQUENCY_HZ)
//        {
            sys_op::comms_handler_state_machine();
//            comms_handler_iteration_tick = osKernelGetTickCount();
//        }
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
