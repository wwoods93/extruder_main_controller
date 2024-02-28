/***********************************************************************************************************************
 * Main_Controller
 * meta_structure_hal_level_resource.cpp
 *
 * wilson
 * 2/11/24
 * 7:13 PM
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

/* meta_structure_hal_level_resource header */
#include "meta_structure_resource.h"

resource::resource()
{

}

id_number_t resource::get_resource_id()
{
    return resource_id;
}
