/***********************************************************************************************************************
 * Main_Controller
 * meta_structure_driver_level_user.h
 *
 * wilson
 * 2/11/24
 * 7:14 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

#ifndef MAIN_CONTROLLER_META_STRUCTURE_USER_H
#define MAIN_CONTROLLER_META_STRUCTURE_USER_H

/* c/c++ includes */
#include <cstdint>
/* stm32 includes */

/* third-party includes */

/* hal includes */

/* driver includes */

/* rtos abstraction includes */

/* system includes */
#include "../meta_structure/meta_structure_system_manager.h"

class user
{
    public:
        int16_t get_user_id();


    protected:
        user();
        int16_t user_id = ID_INVALID;
    private:


};

#endif //MAIN_CONTROLLER_META_STRUCTURE_USER_H
