/***********************************************************************************************************************
 * Main_Controller
 * hal_general.h
 *
 * wilson
 * 10/21/22
 * 9:18 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

//
// Created by wilson on 10/21/22.
//

#ifndef MAIN_CONTROLLER_HAL_GENERAL_H
#define MAIN_CONTROLLER_HAL_GENERAL_H

#define HAL_SET_BIT(REG, BIT)     ((REG) |= (BIT))
#define HAL_CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))
#define HAL_READ_REG(REG)         ((REG))
#define HAL_UNUSED(X) (void)X      /* To avoid gcc/g++ warnings */

#define HAL_GENERAL_IS_BIT_SET(REG, BIT)         (((REG) & (BIT)) == (BIT))
#define HAL_GENERAL_IS_BIT_CLR(REG, BIT)         (((REG) & (BIT)) == 0U)

#if (USE_RTOS == 1U)
/* Reserved for future use */
  #error "USE_RTOS should be 0 in the current HAL release"
#else
    #define HAL_LOCK_MODULE(__HANDLE__)                                                             \
                                    do                                                              \
                                    {                                                               \
                                        if((__HANDLE__)->Lock == HAL_LOCKED) { return HAL_BUSY; }   \
                                        else { (__HANDLE__)->Lock = HAL_LOCKED; }                   \
                                    } while (0U)                                                    \

    #define HAL_UNLOCK_MODULE(__HANDLE__) do { (__HANDLE__)->Lock = HAL_UNLOCKED; } while (0U)

#endif /* USE_RTOS */

#endif //MAIN_CONTROLLER_HAL_GENERAL_H
