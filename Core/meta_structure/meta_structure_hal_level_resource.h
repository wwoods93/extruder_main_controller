/***********************************************************************************************************************
 * Main_Controller
 * meta_structure_hal_level_resource.h
 *
 * wilson
 * 2/11/24
 * 7:13 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

#ifndef MAIN_CONTROLLER_META_STRUCTURE_HAL_LEVEL_RESOURCE_H
#define MAIN_CONTROLLER_META_STRUCTURE_HAL_LEVEL_RESOURCE_H

/* c/c++ includes */
#include <cstdint>
/* stm32 includes */

/* third-party includes */

/* hal includes */

/* driver includes */

/* rtos abstraction includes */

/* system includes */
#include "meta_structure_system_manager.h"

class hal_level_resource
{
    public:
        id_number_t get_global_hal_level_resource_id();
    protected:
        hal_level_resource();
        id_number_t global_hal_level_resource_id;
    private:


};

#endif //MAIN_CONTROLLER_META_STRUCTURE_HAL_LEVEL_RESOURCE_H
