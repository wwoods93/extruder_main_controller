/***********************************************************************************************************************
 * Main_Controller
 * rtos_abstraction_layer.h
 *
 * wilson
 * 11/4/22
 * 12:11 AM
 *
 * Description:
 *
 **********************************************************************************************************************/

#ifndef MAIN_CONTROLLER_RTOS_TASK_DRIVER_H
#define MAIN_CONTROLLER_RTOS_TASK_DRIVER_H

void SPI2_IRQHandler();

void run_initialization_task_functions();

void run_preparation_process_task_functions();

void run_extrusion_process_task_functions();

void run_spooling_process_task_functions();

void run_comms_updater_task_functions();

#endif //MAIN_CONTROLLER_RTOS_TASK_DRIVER_H
