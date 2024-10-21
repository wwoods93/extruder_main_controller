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

char spi_0_name[META_STRUCTURE_NAME_LENGTH_MAX]        = "SPI_RESOURCE_0    \0";
char i2c_0_name[META_STRUCTURE_NAME_LENGTH_MAX]        = "I2C_RESOURCE_0    \0";

char rtd_0_name[META_STRUCTURE_NAME_LENGTH_MAX]        = "RTD_ZONE_0        \0";
char rtd_1_name[META_STRUCTURE_NAME_LENGTH_MAX]        = "RTD_ZONE_1        \0";
char rtd_2_name[META_STRUCTURE_NAME_LENGTH_MAX]        = "RTD_ZONE_2        \0";
char dcm_0_name[META_STRUCTURE_NAME_LENGTH_MAX]        = "DCM_DEVICE_0      \0";
char dcm_1_name[META_STRUCTURE_NAME_LENGTH_MAX]        = "DCM_DEVICE_1      \0";

char rtd_driver_0_name[META_STRUCTURE_NAME_LENGTH_MAX] = "RTD_DRIVER_0      \0";
char rtd_driver_1_name[META_STRUCTURE_NAME_LENGTH_MAX] = "RTD_DRIVER_1      \0";
char rtd_driver_2_name[META_STRUCTURE_NAME_LENGTH_MAX] = "RTD_DRIVER_2      \0";
char dcm_driver_0_name[META_STRUCTURE_NAME_LENGTH_MAX] = "DCM_DRIVER_0      \0";
char dcm_driver_1_name[META_STRUCTURE_NAME_LENGTH_MAX] = "DCM_DRIVER_1      \0";

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

    int16_t claim_next_available_user_id()
    {
        int16_t claimed_id = ID_INVALID;
        if (next_available_user_id <= SYSTEM_MANAGER_USERS_MAX)
        {
            claimed_id = next_available_user_id;
        }
        ++next_available_user_id;
        return claimed_id;
    }

    int16_t claim_next_available_device_id()
    {
        int16_t claimed_id = next_available_device_id;
        next_available_device_id++;
        return claimed_id;
    }

    int16_t claim_next_available_resource_id()
    {
        int16_t claimed_id = next_available_resource_id;
        next_available_resource_id++;
        return claimed_id;
    }

    int16_t create_resource_config(resource_config_t& _resource_config, char* _resource_name, resource_t _type)
    {
        memset(&_resource_config, '\0', sizeof(resource_config_t));
        _resource_config.resource_id = claim_next_available_resource_id();
        memcpy(&_resource_config.resource_name, _resource_name, META_STRUCTURE_NAME_LENGTH_MAX);
        _resource_config.resource_type = _type;
        resource_manifest.push_back(&_resource_config);
        return _resource_config.resource_id;
    }

    int16_t create_device_config(device_config_t& _device_config, char* _device_name, device_t _device_type, resource_t _device_resource_type, uint8_t _packet_size, uint8_t _tx_size, uint8_t _device_address, port_name_t _device_port, uint16_t _device_pin)
    {
        memset(&_device_config, '\0', sizeof(device_config_t));
        _device_config.device_id = claim_next_available_device_id();
        memcpy(&_device_config.device_name, _device_name, META_STRUCTURE_NAME_LENGTH_MAX);
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

    int16_t create_user_config(user_config_t& _user_config, char* _user_name, user_t _type, uint8_t _device_count, int16_t _device_id_0, int16_t _device_id_1, int16_t _device_id_2, int16_t _device_id_3)
    {
        memset(&_user_config, '\0', sizeof (user_config_t));
        _user_config.user_id = claim_next_available_user_id();
        memcpy(&_user_config.user_name, _user_name, META_STRUCTURE_NAME_LENGTH_MAX);
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
        memcpy(&_device, device_manifest[_index], sizeof(device_config_t));
    }

    void set_channel_id_for_device_in_manifest(int16_t _channel_id, uint8_t _index)
    {
        device_manifest[_index]->channel_id = _channel_id;
    }

    uint8_t get_device_manifest_size()
    {
        return (uint8_t)device_manifest.size();
    }
    int16_t get_channel_id_by_device_name(char* _device_name)
    {
        int16_t channel_id = ID_INVALID;
        for (uint8_t index = 0; index < (uint8_t)device_manifest.size(); ++index)
        {
            if (!strcmp( device_manifest[index]->device_name, _device_name))
            {
                channel_id = device_manifest[index]->channel_id;
            }
        }
        return channel_id;
    }

    void get_user_config_from_user_manifest(user_config_t& _user, uint8_t _index)
    {
        memset(&_user, '\0', sizeof(user_config_t));
        memcpy(&_user, device_manifest[_index], sizeof(user_config_t));
    }

    void get_user_config_by_user_name(user_config_t& _user, char* _user_name)
    {
        for (uint8_t index = 0; index < (uint8_t)user_manifest.size(); ++index)
        {
            if (!strcmp( user_manifest[index]->user_name, _user_name))
            {
                memset(&_user, '\0', sizeof(user_config_t));
                memcpy(&_user, user_manifest[index], sizeof(user_config_t));
                break;
            }
        }
    }
}
