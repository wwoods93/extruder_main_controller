/***********************************************************************************************************************
 * Main_Controller
 * rtosal_wrapper.h
 *
 * wilson
 * 8/27/24
 * 12:21 AM
 *
 * Description:
 *
 **********************************************************************************************************************/

#ifndef MAIN_CONTROLLER_RTOSAL_WRAPPER_H
#define MAIN_CONTROLLER_RTOSAL_WRAPPER_H

/* c/c++ includes */

/* stm32 includes */

/* third-party includes */
#include "../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.h"
/* hal includes */

/* driver includes */

/* rtos abstraction includes */

/* sys op includes */

/* meta structure includes */

typedef osMessageQueueId_t cmsis_message_queue_id_t ;
typedef osEventFlagsId_t cmsis_os2_event_flag_id;


    static constexpr int32_t CMSIS_OS2_STATUS_OK                 =  0;
    static constexpr int32_t CMSIS_OS2_STATUS_ERROR              = -1;
    static constexpr int32_t CMSIS_OS2_STATUS_ERROR_TIMEOUT      = -2;
    static constexpr int32_t CMSIS_OS2_STATUS_ERROR_RESOURCE     = -3;
    static constexpr int32_t CMSIS_OS2_STATUS_ERROR_PARAMETER    = -4;
    static constexpr int32_t CMSIS_OS2_STATUS_ERROR_NO_MEMORY    = -5;
    static constexpr int32_t CMSIS_OS2_STATUS_ERROR_ISR          = -6;
    static constexpr int32_t CMSIS_OS2_STATUS_RESERVED           = 0x7FFFFFFF;



#endif //MAIN_CONTROLLER_RTOSAL_WRAPPER_H
