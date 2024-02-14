/***********************************************************************************************************************
 * Main_Controller
 * meta_structure_system_manager.cpp
 *
 * wilson
 * 2/11/24
 * 9:05 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

/* c/c++ includes */
#include <cstdint>
#include <vector>
#include <string>
/* stm32 includes */

/* third-party includes */

/* hal includes */

/* driver includes */

/* rtos abstraction includes */

/* system includes */


/* meta_structure_system_manager header */
#include "meta_structure_system_manager.h"

user_manifest_entry_t temp_user = {USER_TYPE_NULL, "", 0 };
resource_manifest_entry_t temp_resource = {RESOURCE_TYPE_NULL, "", 0 };

//static std::vector<user_manifest_entry_t *> user_manifest;
//static std::vector<device_manifest_entry_t *> device_manifest;
//static std::vector<resource_manifest_entry_t *> resource_manifest;

static uint8_t next_available_global_user_id;
static uint8_t next_available_global_device_id;
static uint8_t next_available_global_resource_id;

//void add_user(user_manifest_entry_t entry)
//{
//    user_manifest.push_back(&entry);
//}
//
//void add_device(user_manifest_entry)

void initialize_system_manifests()
{
    user_manifest.reserve(MAX_DRIVER_LEVEL_USERS);
    device_manifest.reserve(MAX_PERIPHERAL_DEVICES);
    resource_manifest.reserve(MAX_HAL_RESOURCES);
}


id_number_t claim_next_available_global_user_id()
{
    id_number_t claimed_id = next_available_global_user_id;
    next_available_global_user_id++;
    return claimed_id;
}

id_number_t claim_next_available_global_device_id()
{
    id_number_t claimed_id = next_available_global_device_id;
    next_available_global_device_id++;
    return claimed_id;
}

id_number_t claim_next_available_global_resource_id()
{
    id_number_t claimed_id = next_available_global_resource_id;
    next_available_global_resource_id++;
    return claimed_id;
}

id_number_t register_new_user_to_user_manifest(user_t _user_type, std::string const& _user_name)
{
    user_manifest_entry_t entry;
    id_number_t manifest_index = claim_next_available_global_user_id();
    entry.global_id = manifest_index;
    entry.user_type = _user_type;
    entry.user_name = _user_name;
    user_manifest.push_back(&entry);
    return manifest_index;
}

id_number_t register_new_device_to_device_manifest(device_t _device_type, std::string const& _device_name)
{
    device_manifest_entry_t entry;
    id_number_t manifest_index = claim_next_available_global_device_id();
    entry.global_id = manifest_index;
    entry.device_type = _device_type;
    entry.device_name = _device_name;
    device_manifest.push_back(&entry);
    return manifest_index;
}

id_number_t register_new_resource_to_resource_manifest(resource_t _resource_type, std::string const& _resource_name)
{
    resource_manifest_entry_t entry;
    id_number_t manifest_index = claim_next_available_global_resource_id();
    entry.global_id = manifest_index;
    entry.resource_type = _resource_type;
    entry.resource_name = _resource_name;
    resource_manifest.push_back(&entry);
    return manifest_index;
}
