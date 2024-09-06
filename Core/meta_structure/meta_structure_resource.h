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

#ifndef MAIN_CONTROLLER_META_STRUCTURE_RESOURCE_H
#define MAIN_CONTROLLER_META_STRUCTURE_RESOURCE_H

/* c/c++ includes */
#include <cstdint>
/* stm32 includes */

/* third-party includes */

/* hal includes */

/* driver includes */

/* rtos abstraction includes */

/* system includes */
#include "meta_structure_system_manager.h"

class resource
{
    public:
        int16_t get_resource_id();
    protected:
        resource();
        int16_t resource_id;
    private:

};

#endif //MAIN_CONTROLLER_META_STRUCTURE_RESOURCE_H
