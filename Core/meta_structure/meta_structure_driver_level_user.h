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

#ifndef MAIN_CONTROLLER_META_STRUCTURE_DRIVER_LEVEL_USER_H
#define MAIN_CONTROLLER_META_STRUCTURE_DRIVER_LEVEL_USER_H

/* c/c++ includes */
#include <cstdint>
/* stm32 includes */

/* third-party includes */

/* hal includes */

/* driver includes */

/* rtos abstraction includes */

/* system includes */
#include "../meta_structure/meta_structure_system_manager.h"

class driver_level_user
{
    public:
        id_number_t get_global_driver_level_user_id();


    protected:
        driver_level_user();
        id_number_t global_driver_level_user_id;
    private:


};

#endif //MAIN_CONTROLLER_META_STRUCTURE_DRIVER_LEVEL_USER_H
