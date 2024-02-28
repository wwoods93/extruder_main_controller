/***********************************************************************************************************************
 * Main_Controller
 * meta_structure_driver_level_user.cpp
 *
 * wilson
 * 2/11/24
 * 7:14 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

/* c/c++ includes */

/* stm32 includes */

/* third-party includes */

/* hal includes */

/* driver includes */

/* rtos abstraction includes */

/* system includes */
#include "meta_structure_system_manager.h"

/* meta_structure_driver_level_user header */
#include "meta_structure_user.h"


user::user()
{

}

id_number_t user::get_user_id()
{
    return user_id;
}
