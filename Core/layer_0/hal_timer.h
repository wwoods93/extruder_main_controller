 /***********************************************************************************************************************
 * Main_Controller
 * mcu_clock_timers.h
 *
 * wilson
 * 6/15/22
 * 8:26 PM
 *
 * Description:
 *
 **********************************************************************************************************************/



#ifndef MAIN_CONTROLLER_MCU_CLOCK_TIMERS_H
#define MAIN_CONTROLLER_MCU_CLOCK_TIMERS_H

/* c/c++ includes */

/* stm32 includes */
#include "stm32f4xx.h"
/* third-party includes */

/* layer_0 includes */

/* layer_1_rtosal includes */

/* layer_1 includes */

/* layer_3_control includes */

/* layer_4_sys_op includes */

/* layer_n_meta_structure includes */


void MX_TIM6_Init(void);
void MX_TIM7_Init(void);
void MX_TIM11_Init(void);

void timers_initialize(void);



#endif //MAIN_CONTROLLER_MCU_CLOCK_TIMERS_H
