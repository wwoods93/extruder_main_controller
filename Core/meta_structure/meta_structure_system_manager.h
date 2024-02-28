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

#define DEFAULT_MAX_MANIFEST_ENTRIES        32U

#define SYSTEM_MANAGER_USERS_MAX                 DEFAULT_MAX_MANIFEST_ENTRIES
#define MAX_PERIPHERAL_DEVICES              DEFAULT_MAX_MANIFEST_ENTRIES
#define MAX_HAL_RESOURCES                   DEFAULT_MAX_MANIFEST_ENTRIES

#define NAME_LENGTH_MAX                     16U
#define USER_NAME_LENGTH_MAX                NAME_LENGTH_MAX
#define DEVICE_NAME_LENGTH_MAX              NAME_LENGTH_MAX
#define RESOURCE_NAME_LENGTH_MAX            NAME_LENGTH_MAX

typedef int16_t id_number_t;

constexpr id_number_t ID_INVALID = -1;

typedef enum
{
    USER_TYPE_NULL                                  = 0x00,
    USER_TYPE_SCREW_MOTOR_INTERFACE                 = 0x01,
    USER_TYPE_DC_MOTOR_INTERFACE                    = 0x02,
    USER_TYPE_RTD_INTERFACE                         = 0x03,
    USER_TYPE_TEMPERATURE_CONTROL_INTERFACE         = 0x04,
    USER_TYPE_TOUCHSCREEN_INTERFACE                 = 0x05,

} user_t;

typedef enum
{
    DEVICE_TYPE_NULL                                = 0x00,
    DEVICE_TYPE_SCREW_MOTOR_CONTROLLER              = 0x01,
    DEVICE_TYPE_DC_MOTOR_CONTROLLER                 = 0x02,
    DEVICE_TYPE_RTD_SENSOR                          = 0x03,
    DEVICE_TYPE_TEMP_CONTROLLER                     = 0x04,

} device_t;

typedef enum
{
    RESOURCE_TYPE_NULL                              = 0x00,
    RESOURCE_TYPE_ADC                               = 0x01,
    RESOURCE_TYPE_CAN                               = 0x02,
    RESOURCE_TYPE_I2C                               = 0x03,
    RESOURCE_TYPE_SPI                               = 0x04,
    RESOURCE_TYPE_PWM                               = 0x05,
    RESOURCE_TYPE_UART                              = 0x06,

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

} port_name_t;

typedef struct
{
    id_number_t user_id;
    std::string user_name;
    user_t user_type;
} user_manifest_entry_t;

typedef struct
{
    id_number_t device_id;
    std::string device_name;
    device_t device_type;
} device_manifest_entry_t;

typedef struct
{
    id_number_t resource_id;
    std::string resource_name;
    resource_t resource_type;
} resource_manifest_entry_t;



static std::vector<user_manifest_entry_t *> user_manifest;
static std::vector<device_manifest_entry_t *> device_manifest;
static std::vector<resource_manifest_entry_t *> resource_manifest;

void initialize_system_manifests();
id_number_t claim_next_available_user_id();
id_number_t claim_next_available_global_device_id();
id_number_t claim_next_available_global_resource_id();

id_number_t register_new_user_to_user_manifest(user_t _user_type, std::string const& _user_name);
id_number_t register_new_device_to_device_manifest(device_t _device_type, std::string const& _device_name);
id_number_t register_new_resource_to_resource_manifest(resource_t _resource_type, std::string const& _resource_name);


#endif //MAIN_CONTROLLER_META_STRUCTURE_SYSTEM_MANAGER_H
