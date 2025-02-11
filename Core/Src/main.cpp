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

/* third-party includes */
#include "cmsis_os2.h"
/* layer_0 includes */
#include "../layer_0/hal.h"
/* layer_1_rtosal includes */
#include "../layer_0/rtosal.h"
/* layer_1 includes */

/* layer_2 includes */

/* layer_3 includes */
#include "../layer_3/sys_op_heartbeat.h"
#include "../layer_3/sys_op_user_comms.h"
#include "../layer_3/sys_op_power_management.h"
#include "../layer_3/sys_op_speed_control.h"
#include "../layer_3/sys_op_temp_control.h"


rtosal::thread_handle_t user_comms_task_handle;
rtosal::thread_handle_t speed_control_task_handle;
rtosal::thread_handle_t temp_control_task_handle;
rtosal::thread_handle_t heartbeat_task_handle;

const rtosal::thread_attr_t user_comms_task_attributes      = { .name = "comms_handler_task",       .stack_size = 320 * 4, .priority = (osPriority_t) osPriorityNormal, };
const rtosal::thread_attr_t speed_control_task_attributes   = { .name = "preparation_process_task", .stack_size = 96  * 4, .priority = (osPriority_t) osPriorityNormal, };
const rtosal::thread_attr_t temp_control_task_attributes    = { .name = "extrusion_process_task",   .stack_size = 256 * 4, .priority = (osPriority_t) osPriorityNormal, };
const rtosal::thread_attr_t heartbeat_task_attributes       = { .name = "heartbeat_task",           .stack_size = 96  * 4, .priority = (osPriority_t) osPriorityNormal, };

void start_user_comms_task(void *argument);
void start_speed_control_task(void *argument);
void start_temp_control_task(void *argument);
void start_heartbeat_task(void *argument);

uint8_t system_run = 1U;

int main()
{
    initialize_peripherals();
    rtosal::kernel_init();
    rtosal::rtosal_resource_init();

    user_comms_task_handle      = rtosal::thread_create(start_user_comms_task,      &user_comms_task_attributes);
    speed_control_task_handle   = rtosal::thread_create(start_speed_control_task,   &speed_control_task_attributes);
    temp_control_task_handle    = rtosal::thread_create(start_temp_control_task,    &temp_control_task_attributes);
    heartbeat_task_handle       = rtosal::thread_create(start_heartbeat_task,       &heartbeat_task_attributes);

    rtosal::kernel_start();
    while (system_run);
}

void start_heartbeat_task(void *argument)
{
    while (system_run)
    {
        sys_op::heartbeat::task_state_machine();
    }
}

void start_user_comms_task(void *argument)
{
    sys_op::user_comms::task_intitialize();
    while (system_run)
    {
        sys_op::user_comms::task_state_machine();
    }
}

void start_speed_control_task(void *argument)
{
    while (system_run)
    {
        sys_op::speed_control::task_state_machine();
    }
}

void start_temp_control_task(void *argument)
{
    while (system_run)
    {
        sys_op::temp_control::task_state_machine();
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
