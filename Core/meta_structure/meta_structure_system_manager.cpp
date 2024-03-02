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
#include <cstring>
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

namespace meta_structure
{
    static uint8_t next_available_user_id;
    static uint8_t next_available_device_id;
    static uint8_t next_available_resource_id;

    void initialize_system_manifests()
    {
        user_manifest.reserve(SYSTEM_MANAGER_USERS_MAX);
        device_manifest.reserve(MAX_DEVICES);
        resource_manifest.reserve(MAX_HAL_RESOURCES);
    }

    id_number_t claim_next_available_user_id()
    {
        id_number_t claimed_id = ID_INVALID;
        if (next_available_user_id <= SYSTEM_MANAGER_USERS_MAX)
        {
            claimed_id = next_available_user_id;
        }
        ++next_available_user_id;
        return claimed_id;
    }

    id_number_t claim_next_available_device_id()
    {
        id_number_t claimed_id = next_available_device_id;
        next_available_device_id++;
        return claimed_id;
    }

    id_number_t claim_next_available_resource_id()
    {
        id_number_t claimed_id = next_available_resource_id;
        next_available_resource_id++;
        return claimed_id;
    }

    id_number_t create_resource_config(resource_config_t& _resource_config, char* _resource_name, resource_t _type)
    {
        memset(&_resource_config, '\0', sizeof(resource_config_t));
        _resource_config.resource_id = claim_next_available_resource_id();
        memcpy(&_resource_config.resource_name, _resource_name, NAME_LENGTH_MAX);
        _resource_config.resource_type = _type;
        resource_manifest.push_back(&_resource_config);
        return _resource_config.resource_id;
    }

    id_number_t create_device_config(device_config_t& _device_config, char* _device_name, device_t _device_type, resource_t _device_resource_type, uint8_t _packet_size, uint8_t _tx_size, uint8_t _device_address, port_name_t _device_port, uint16_t _device_pin)
    {
        memset(&_device_config, '\0', sizeof(device_config_t));
        _device_config.device_id = claim_next_available_device_id();
        memcpy(&_device_config.device_name, _device_name, NAME_LENGTH_MAX);
        _device_config.device_type = _device_type;
        _device_config.device_resource_type = _device_resource_type;
        _device_config.packet_size = _packet_size;
        _device_config.tx_size = _tx_size;
        _device_config.device_address = _device_address;
        _device_config.device_port = _device_port;
        _device_config.device_pin = _device_pin;
        device_manifest.push_back(&_device_config);
        return _device_config.device_id;
    }

    id_number_t create_user_config(user_config_t& _user_config, char* _user_name, user_t _type, uint8_t _device_count, id_number_t _device_id_0, id_number_t _device_id_1, id_number_t _device_id_2, id_number_t _device_id_3)
    {
        memset(&_user_config, '\0', sizeof (user_config_t));
        _user_config.user_id = claim_next_available_user_id();
        memcpy(&_user_config.user_name, _user_name, NAME_LENGTH_MAX);
        _user_config.user_type = _type;
        _user_config.device_count = _device_count;
        _user_config.device_ids[0] = _device_id_0;
        _user_config.device_ids[1] = _device_id_1;
        _user_config.device_ids[2] = _device_id_2;
        _user_config.device_ids[3] = _device_id_3;
        user_manifest.push_back(&_user_config);
        return _user_config.user_id;
    }

    void get_device_from_device_manifest(device_config_t& _device, uint8_t _index)
    {
        memset(&_device, '\0', sizeof(device_config_t));
        memcpy(&_device, &device_manifest[_index], sizeof(device_config_t));
    }

    void set_channel_id_for_device_in_manifest(id_number_t _channel_id, uint8_t _index)
    {
        device_manifest[_index]->channel_id = _channel_id;
    }
}
