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
#include "../layer_0/hal_wrapper.h"
/* driver includes */

/* rtos abstraction includes */

/* system includes */

//#define DEFAULT_MAX_MANIFEST_ENTRIES        8U

//#define SYSTEM_MANAGER_USERS_MAX            DEFAULT_MAX_MANIFEST_ENTRIES
//#define MAX_DEVICES                         DEFAULT_MAX_MANIFEST_ENTRIES
//#define MAX_HAL_RESOURCES                   DEFAULT_MAX_MANIFEST_ENTRIES
//
//#define DEVICES_PER_USER_MAX                4U
//
//#define META_STRUCTURE_NAME_LENGTH_MAX                     20U
//#define PIN_NULL                            0xFFFF
//#define ADDRESS_NULL_8_BIT                  0xFF

#ifndef ID_INVALID
#define ID_INVALID (-1)
#endif

//const char default_name[META_STRUCTURE_NAME_LENGTH_MAX] = "default_name\0";
//
//extern char spi_0_name[META_STRUCTURE_NAME_LENGTH_MAX];
//extern char i2c_0_name[META_STRUCTURE_NAME_LENGTH_MAX];
//
//extern char rtd_0_name[META_STRUCTURE_NAME_LENGTH_MAX];
//extern char rtd_1_name[META_STRUCTURE_NAME_LENGTH_MAX];
//extern char rtd_2_name[META_STRUCTURE_NAME_LENGTH_MAX];
//extern char dcm_0_name[META_STRUCTURE_NAME_LENGTH_MAX];
//extern char dcm_1_name[META_STRUCTURE_NAME_LENGTH_MAX];
//
//extern char rtd_driver_0_name[META_STRUCTURE_NAME_LENGTH_MAX];
//extern char rtd_driver_1_name[META_STRUCTURE_NAME_LENGTH_MAX];
//extern char rtd_driver_2_name[META_STRUCTURE_NAME_LENGTH_MAX];
//extern char dcm_driver_0_name[META_STRUCTURE_NAME_LENGTH_MAX];
//extern char dcm_driver_1_name[META_STRUCTURE_NAME_LENGTH_MAX];

//typedef enum
//{
//    USER_TYPE_SCREW_MOTOR   = 0x00,
//    USER_TYPE_DCM           = 0x01,
//    USER_TYPE_RTD           = 0x02,
//    USER_TYPE_TEMP_CTRL     = 0x03,
//    USER_TYPE_TFT           = 0x04,
//    USER_TYPE_NULL          = 0xFF,
//} user_t;
//
//typedef enum
//{
//    DEVICE_TYPE_SCREW_MOTOR = 0x00,
//    DEVICE_TYPE_DCM         = 0x01,
//    DEVICE_TYPE_RTD         = 0x02,
//    DEVICE_TYPE_TEMP_CTRL   = 0x03,
//    DEVICE_TYPE_NULL        = 0xFF,
//} device_t;
//
//typedef enum
//{
//    RESOURCE_TYPE_ADC       = 0x00,
//    RESOURCE_TYPE_CAN       = 0x01,
//    RESOURCE_TYPE_I2C       = 0x02,
//    RESOURCE_TYPE_SPI       = 0x03,
//    RESOURCE_TYPE_PWM       = 0x04,
//    RESOURCE_TYPE_UART      = 0x05,
//    RESOURCE_TYPE_NULL      = 0xFF,
//} resource_t;

//struct resource_config_t
//{
//    int16_t resource_id = ID_INVALID;
//    char resource_name[META_STRUCTURE_NAME_LENGTH_MAX] = "default_name\0";
//    resource_t resource_type = RESOURCE_TYPE_NULL;
//};

//struct device_config_t
//{
//    int16_t device_id = ID_INVALID;
//    int16_t channel_id = ID_INVALID;
//    char device_name[META_STRUCTURE_NAME_LENGTH_MAX] = "default_name\0";
//    device_t device_type = DEVICE_TYPE_NULL;
//    resource_t device_resource_type = RESOURCE_TYPE_NULL;
//    uint8_t packet_size = 0U;
//    uint8_t tx_size = 0U;
//    uint8_t device_address = ADDRESS_NULL_8_BIT;
//    port_name_t device_port = PORT_NULL;
//    uint16_t device_pin = PIN_NULL;
//};
//
//struct user_config_t
//{
//    int16_t user_id = ID_INVALID;
//    char user_name[META_STRUCTURE_NAME_LENGTH_MAX] = "default_name\0";
//    user_t user_type = USER_TYPE_NULL;
//    uint8_t device_count = 0;
//    int16_t device_ids[DEVICES_PER_USER_MAX] = { ID_INVALID, ID_INVALID, ID_INVALID, ID_INVALID };
//};



namespace meta_structure
{
//    static std::vector<user_config_t *> user_manifest;
//    static std::vector<device_config_t *> device_manifest;
//    static std::vector<resource_config_t *> resource_manifest;

//    void initialize_system_manifests();
//    int16_t claim_next_available_user_id();
//    int16_t claim_next_available_device_id();
//    int16_t claim_next_available_resource_id();
//
//    int16_t create_resource_config(resource_config_t& _resource_config, char* _resource_name, resource_t _type);
//    int16_t create_device_config(device_config_t& _device_config, char* _device_name, device_t _device_type, resource_t _device_resource_type, uint8_t _packet_size, uint8_t _tx_size, uint8_t _device_address, port_name_t _device_port, uint16_t _device_pin);
//    int16_t create_user_config(user_config_t& _user_config, char* _user_name, user_t _type, uint8_t _device_count, int16_t _device_id_0, int16_t _device_id_1, int16_t _device_id_2, int16_t _device_id_3);
//    void get_device_from_device_manifest(device_config_t& _device, uint8_t _index);
//    void set_channel_id_for_device_in_manifest(int16_t _channel_id, uint8_t _index);
//    uint8_t get_device_manifest_size();
//    int16_t get_channel_id_by_device_name(char* _device_name);
//    void get_user_config_from_user_manifest(user_config_t& _user, uint8_t _index);
//    void get_user_config_by_user_name(user_config_t& _user, char* _user_name);
}


#endif //MAIN_CONTROLLER_META_STRUCTURE_SYSTEM_MANAGER_H
