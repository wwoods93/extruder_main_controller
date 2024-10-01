/***********************************************************************************************************************
 * Main_Controller
 * sys_op_system_manager.cpp
 *
 * wilson
 * 2/10/24
 * 9:10 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

/* c/c++ includes */
#include <cstdint>
#include <vector>
#include <utility>
/* stm32 includes */

/* third-party includes */

/* hal includes */

/* driver includes */

/* rtos abstraction includes */

/* system includes */
#include "sys_op_general.h"

/* sys_op_system_manager header */
#include "sys_op_system_manager.h"

//user_manifest_entry_t temp_user = {USER_TYPE_NULL, 0 };
//resource_manifest_entry_t temp_resource = {RESOURCE_TYPE_NULL, 0 };
//
//static std::vector<user_manifest_entry_t *> user_manifest;
//static std::vector<resource_manifest_entry_t *> resource_manifest;
//static uint8_t next_available_global_user_id;
//static uint8_t next_available_global_resource_id;
//
//void initialize_system_manifests()
//{
//    user_manifest.reserve(MAX_DRIVER_LEVEL_USERS);
//    resource_manifest.reserve(MAX_HAL_RESOURCES);
//}
//
//
//uint8_t claim_next_available_global_user_id()
//{
//    uint8_t claimed_id = next_available_global_user_id;
//    next_available_global_user_id++;
//    return claimed_id;
//}
//
//uint8_t claim_next_available_global_resource_id()
//{
//    uint8_t claimed_id = next_available_global_resource_id;
//    next_available_global_resource_id++;
//    return claimed_id;
//}
//
//uint8_t register_new_user_to_user_manifest(user_t _user_type)
//{
//    user_manifest_entry_t entry;
//    uint8_t manifest_index = claim_next_available_global_user_id();
//    entry.global_id = manifest_index;
//    entry.user_type = _user_type;
//    user_manifest.push_back(&entry);
//    return manifest_index;
//}
//
//uint8_t register_new_resource_to_resource_manifest(resource_t _resource_type)
//{
//    resource_manifest_entry_t entry;
//    uint8_t manifest_index = claim_next_available_global_resource_id();
//    entry.global_id = manifest_index;
//    entry.resource_type = _resource_type;
//    resource_manifest.push_back(&entry);
//    return manifest_index;
//}
