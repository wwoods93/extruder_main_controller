/***********************************************************************************************************************
 * Main_Controller
 * meta_structure_system_manager.h
 *
 * wilson
 * 2/11/24
 * 9:05 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

#ifndef MAIN_CONTROLLER_META_STRUCTURE_SYSTEM_MANAGER_H
#define MAIN_CONTROLLER_META_STRUCTURE_SYSTEM_MANAGER_H

/* c/c++ includes */
#include <cstdint>
#include <string>
#include <vector>
/* stm32 includes */

/* third-party includes */

/* hal includes */

/* driver includes */

/* rtos abstraction includes */

/* system includes */

#define DEFAULT_MAX_MANIFEST_ENTRIES        8U

#define SYSTEM_MANAGER_USERS_MAX            DEFAULT_MAX_MANIFEST_ENTRIES
#define MAX_DEVICES                         DEFAULT_MAX_MANIFEST_ENTRIES
#define MAX_HAL_RESOURCES                   DEFAULT_MAX_MANIFEST_ENTRIES

#define DEVICES_PER_USER_MAX                4U

#define NAME_LENGTH_MAX                     20U
#define PIN_NULL                            0xFFFF
#define ADDRESS_NULL_8_BIT                  0xFF
typedef int16_t id_number_t;

constexpr id_number_t ID_INVALID = -1;

const char default_name[NAME_LENGTH_MAX] = "default_name\0";

typedef enum
{
    USER_TYPE_SCREW_MOTOR   = 0x00,
    USER_TYPE_DCM           = 0x01,
    USER_TYPE_RTD           = 0x02,
    USER_TYPE_TEMP_CTRL     = 0x03,
    USER_TYPE_TFT           = 0x04,
    USER_TYPE_NULL          = 0xFF,
} user_t;

typedef enum
{
    DEVICE_TYPE_SCREW_MOTOR = 0x00,
    DEVICE_TYPE_DCM         = 0x01,
    DEVICE_TYPE_RTD         = 0x02,
    DEVICE_TYPE_TEMP_CTRL   = 0x03,
    DEVICE_TYPE_NULL        = 0xFF,
} device_t;

typedef enum
{
    RESOURCE_TYPE_ADC       = 0x00,
    RESOURCE_TYPE_CAN       = 0x01,
    RESOURCE_TYPE_I2C       = 0x02,
    RESOURCE_TYPE_SPI       = 0x03,
    RESOURCE_TYPE_PWM       = 0x04,
    RESOURCE_TYPE_UART      = 0x05,
    RESOURCE_TYPE_NULL      = 0xFF,
} resource_t;

typedef enum
{
    PORT_A      = 0x00,
    PORT_B      = 0x01,
    PORT_C      = 0x02,
    PORT_D      = 0x03,
    PORT_E      = 0x04,
    PORT_F      = 0x05,
    PORT_G      = 0x06,
    PORT_H      = 0x07,
    PORT_NULL   = 0xFF,

} port_name_t;

struct resource_config_t
{
    id_number_t resource_id = ID_INVALID;
    char resource_name[NAME_LENGTH_MAX] = "default_name\0";
    resource_t resource_type = RESOURCE_TYPE_NULL;
};

struct device_config_t
{
    id_number_t device_id = ID_INVALID;
    id_number_t channel_id = ID_INVALID;
    char device_name[NAME_LENGTH_MAX] = "default_name\0";
    device_t device_type = DEVICE_TYPE_NULL;
    resource_t device_resource_type = RESOURCE_TYPE_NULL;
    uint8_t packet_size = 0U;
    uint8_t tx_size = 0U;
    uint8_t device_address = ADDRESS_NULL_8_BIT;
    port_name_t device_port = PORT_NULL;
    uint16_t device_pin = PIN_NULL;
};

struct user_config_t
{
    id_number_t user_id = ID_INVALID;
    char user_name[NAME_LENGTH_MAX] = "default_name\0";
    user_t user_type = USER_TYPE_NULL;
    uint8_t device_count = 0;
    id_number_t device_ids[DEVICES_PER_USER_MAX] = { ID_INVALID, ID_INVALID, ID_INVALID, ID_INVALID };
};



namespace meta_structure
{
    static std::vector<user_config_t *> user_manifest;
    static std::vector<device_config_t *> device_manifest;
    static std::vector<resource_config_t *> resource_manifest;

    void initialize_system_manifests();
    id_number_t claim_next_available_user_id();
    id_number_t claim_next_available_device_id();
    id_number_t claim_next_available_resource_id();

    id_number_t create_resource_config(resource_config_t& _resource_config, char* _resource_name, resource_t _type);
    id_number_t create_device_config(device_config_t& _device_config, char* _device_name, device_t _device_type, resource_t _device_resource_type, uint8_t _packet_size, uint8_t _tx_size, uint8_t _device_address, port_name_t _device_port, uint16_t _device_pin);
    id_number_t create_user_config(user_config_t& _user_config, char* _user_name, user_t _type, uint8_t _device_count, id_number_t _device_id_0, id_number_t _device_id_1, id_number_t _device_id_2, id_number_t _device_id_3);
    void get_device_from_device_manifest(device_config_t& _device, uint8_t _index);
    void set_channel_id_for_device_in_manifest(id_number_t _channel_id, uint8_t _index);
    uint8_t get_device_manifest_size();
    id_number_t get_channel_id_by_device_name(char* _device_name);
}


#endif //MAIN_CONTROLLER_META_STRUCTURE_SYSTEM_MANAGER_H
