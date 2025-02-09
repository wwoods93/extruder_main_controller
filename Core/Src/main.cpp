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

/* layer_3 includes */
#include "../layer_3/sys_op_heartbeat.h"
#include "../layer_3/sys_op_initialization.h"
#include "../layer_3/sys_op_user_comms.h"
#include "../layer_3/sys_op_power_management.h"
#include "../layer_3/sys_op_speed_control.h"
#include "../layer_3/sys_op_temp_control.h"
/* layer_n_meta_structure includes */

/* main header */
#include "main.h"
#include "extruder_main.h"

#include "task.h"


const osEventFlagsAttr_t initialization_event_flags_attributes = { .name = "initialization_event_flags" };

osEventFlagsId_t initialization_event_flags_handle;
osMessageQueueId_t initialization_task_queue_handle;
const osMessageQueueAttr_t initialization_task_queue_attributes = { .name = "initialization_task_queue" };


osThreadId_t initialization_task_handle;
osThreadId_t user_comms_task_handle;
osThreadId_t speed_control_task_handle;
osThreadId_t temp_control_task_handle;
osThreadId_t heartbeat_task_handle;

const osThreadAttr_t initialization_task_attributes     = { .name = "initialization_task",      .stack_size = 200 * 4, .priority = (osPriority_t) osPriorityNormal, };
const osThreadAttr_t user_comms_task_attributes         = { .name = "comms_handler_task",       .stack_size = 320 * 4, .priority = (osPriority_t) osPriorityNormal, };
const osThreadAttr_t speed_control_task_attributes      = { .name = "preparation_process_task", .stack_size = 96  * 4, .priority = (osPriority_t) osPriorityNormal, };
const osThreadAttr_t temp_control_task_attributes       = { .name = "extrusion_process_task",   .stack_size = 256 * 4, .priority = (osPriority_t) osPriorityNormal, };
const osThreadAttr_t heartbeat_task_attributes          = { .name = "heartbeat_task",           .stack_size = 96  * 4, .priority = (osPriority_t) osPriorityNormal, };

void start_initialization_task(void *argument);
void start_user_comms_task(void *argument);
void start_speed_control_task(void *argument);
void start_temp_control_task(void *argument);
void start_heartbeat_task(void *argument);



int main()
{
    initialize_peripherals();
    osKernelInitialize();

    initialization_event_flags_handle = osEventFlagsNew(&initialization_event_flags_attributes);

    rtosal::rtosal_initialize();
    initialization_task_queue_handle = osMessageQueueNew((uint32_t)QUEUE_LENGTH_MAX, (uint32_t)sizeof(common_packet_t), &initialization_task_queue_attributes);

    initialization_task_handle  = osThreadNew(start_initialization_task, nullptr, &initialization_task_attributes);
    user_comms_task_handle      = osThreadNew(start_user_comms_task, nullptr, &user_comms_task_attributes);
    speed_control_task_handle   = osThreadNew(start_speed_control_task, nullptr, &speed_control_task_attributes);
    temp_control_task_handle    = osThreadNew(start_temp_control_task, nullptr, &temp_control_task_attributes);
    heartbeat_task_handle       = osThreadNew(start_heartbeat_task, nullptr, &heartbeat_task_attributes);

    osEventFlagsClear(initialization_event_flags_handle, 0xFFFFFFFF);
    osKernelStart();
    while (true);
}

void start_heartbeat_task(void *argument)
{
    while (true)
    {
        sys_op::heartbeat::task_state_machine();
    }
}

void start_initialization_task(void *argument)
{

    while (true)
    {
        sys_op::initialization::task_state_machine();
    }
}

void start_user_comms_task(void *argument)
{

//    static unsigned long last_ten_high_water_marks[10];
//    static unsigned long highest = 0;
//    static uint8_t count = 0;

    while (true)
    {
        sys_op::user_comms::task_state_machine();
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

void start_speed_control_task(void *argument)
{
    while (true)
    {
        sys_op::speed_control::task_state_machine();
    }
}

void start_temp_control_task(void *argument)
{
//    static unsigned long last_ten_high_water_marks[10];
//    static unsigned long highest = 0;
//    static uint8_t count = 0;

    while (true)
    {
        sys_op::temp_control::task_state_machine();
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

osMessageQueueId_t get_initialization_task_queue_handle()
{
    return initialization_task_queue_handle;
}

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
