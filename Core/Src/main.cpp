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

/* c/c++ includes */

/* stm32 includes */
#include "stm32f4xx.h"
/* third-party includes */
#include "cmsis_os.h"
#include "cmsis_os2.h"
/* layer_0 includes */
#include "../layer_0/hal.h"
/* layer_1_rtosal includes */
#include "../layer_0/rtosal_globals.h"
#include "../layer_0/rtosal.h"
/* layer_1 includes */

/* layer_3_control includes */

/* layer_4_sys_op includes */
#include "../layer_4_sys_op/sys_op_initialization.h"
#include "../layer_4_sys_op/sys_op_comms_handler.h"
#include "../layer_4_sys_op/sys_op_preparation_process.h"
#include "../layer_4_sys_op/sys_op_extrusion_process.h"
#include "../layer_4_sys_op/sys_op_spooling_process.h"
/* layer_n_meta_structure includes */

/* main header */
#include "main.h"
#include "extruder_main.h"

#include "task.h"




const osEventFlagsAttr_t initialization_event_flags_attributes = { .name = "initialization_event_flags" };

osEventFlagsId_t initialization_event_flags_handle;
osMessageQueueId_t initialization_task_queue_handle;
const osMessageQueueAttr_t initialization_task_queue_attributes = { .name = "initialization_task_queue" };





osThreadId_t initialization_taskHandle;
osThreadId_t comms_handler_taskHandle;
osThreadId_t preparation_process_taskHandle;
osThreadId_t extrusion_process_taskHandle;
osThreadId_t spooling_process_taskHandle;
osThreadId_t heartbeat_taskHandle;

const osThreadAttr_t initialization_task_attributes = { .name = "initialization_task",      .stack_size = 200 * 4, .priority = (osPriority_t) osPriorityNormal, };
const osThreadAttr_t comms_handler_task_attributes  = { .name = "comms_handler_task",       .stack_size = 320 * 4, .priority = (osPriority_t) osPriorityAboveNormal2, };
const osThreadAttr_t preparation_task_attributes    = { .name = "preparation_process_task", .stack_size = 96  * 4, .priority = (osPriority_t) osPriorityNormal, };
const osThreadAttr_t extrusion_task_attributes      = { .name = "extrusion_process_task",   .stack_size = 256 * 4, .priority = (osPriority_t) osPriorityAboveNormal1, };
const osThreadAttr_t spooling_task_attributes       = { .name = "spooling_process_task",    .stack_size = 96  * 4, .priority = (osPriority_t) osPriorityNormal, };
const osThreadAttr_t heartbeat_task_attributes      = { .name = "heartbeat_task",           .stack_size = 96  * 4, .priority = (osPriority_t) osPriorityNormal, };

void start_initialization_task(void *argument);
void start_comms_handler_task(void *argument);
void start_preparation_process_task(void *argument);
void start_extrusion_process_task(void *argument);
void start_spooling_process_task(void *argument);
void start_heartbeat_task(void *argument);



int main()
{
    initialize_peripherals();
//    timers_initialize();
    osKernelInitialize();

    initialization_event_flags_handle = osEventFlagsNew(&initialization_event_flags_attributes);

    rtosal::rtosal_initialize();
    initialization_task_queue_handle = osMessageQueueNew((uint32_t)QUEUE_LENGTH_MAX, (uint32_t)sizeof(common_packet_t), &initialization_task_queue_attributes);



    initialization_taskHandle       = osThreadNew(start_initialization_task,        nullptr, &initialization_task_attributes);
    comms_handler_taskHandle        = osThreadNew(start_comms_handler_task,         nullptr, &comms_handler_task_attributes);
    preparation_process_taskHandle  = osThreadNew(start_preparation_process_task,   nullptr, &preparation_task_attributes);
    extrusion_process_taskHandle    = osThreadNew(start_extrusion_process_task,     nullptr, &extrusion_task_attributes);
    spooling_process_taskHandle     = osThreadNew(start_spooling_process_task,      nullptr, &spooling_task_attributes);
    heartbeat_taskHandle            = osThreadNew(start_heartbeat_task,             nullptr, &heartbeat_task_attributes);

    osEventFlagsClear(initialization_event_flags_handle, 0xFFFFFFFF);
    osKernelStart();
    while (true);
}

void start_heartbeat_task(void *argument)
{
    while (true)
    {
        osDelay(1);
    }
}

void start_initialization_task(void *argument)
{

    while (true)
    {
        sys_op::initialization::task_state_machine();
    }
}

void start_comms_handler_task(void *argument)
{

//    static unsigned long last_ten_high_water_marks[10];
//    static unsigned long highest = 0;
//    static uint8_t count = 0;

    while (true)
    {
        sys_op::comms_handler::task_state_machine();
//        unsigned long high_water_mark =  uxTaskGetStackHighWaterMark(nullptr);
//        last_ten_high_water_marks[count] = high_water_mark;
//        count++;
//        if (count >= 10)
//        {
//            highest = last_ten_high_water_marks[0];
//            for (uint8_t i = 1; i < 10; ++i)
//            {
//                if (last_ten_high_water_marks[i] > highest)
//                    highest = last_ten_high_water_marks[i];
//            }
//            count = 0;
//        }
    }
}

void start_preparation_process_task(void *argument)
{
    while (true)
    {
        sys_op::preparation::task_state_machine();
    }
}

void start_extrusion_process_task(void *argument)
{
//    static unsigned long last_ten_high_water_marks[10];
//    static unsigned long highest = 0;
//    static uint8_t count = 0;

    while (true)
    {
        sys_op::extrusion::task_state_machine();
//        unsigned long high_water_mark =  uxTaskGetStackHighWaterMark(nullptr);
//        last_ten_high_water_marks[count] = high_water_mark;
//        count++;
//        if (count >= 10)
//        {
//            highest = last_ten_high_water_marks[0];
//            for (uint8_t i = 1; i < 10; ++i)
//            {
//                if (last_ten_high_water_marks[i] < highest)
//                    highest = last_ten_high_water_marks[i];
//            }
//            count = 0;
//        }
    }
}

void start_spooling_process_task(void *argument)
{
    while (true)
    {
        sys_op::spooling::task_state_machine();
    }
}

osMessageQueueId_t get_initialization_task_queue_handle()
{
    return initialization_task_queue_handle;
}

//osMessageQueueId_t get_spi_2_extrusion_task_tx_queue_handle()
//{
//    return spi_tx_queue_handle;
//}

//osMessageQueueId_t get_spi_2_extrusion_task_rx_queue_handle()
//{
//    return spi_rx_queue_handle;
//}



osEventFlagsId_t get_initialization_event_flags_handle()
{
    return initialization_event_flags_handle;
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
