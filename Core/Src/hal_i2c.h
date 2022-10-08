/***********************************************************************************************************************
 * Main_Controller
 * hal_i2c.h
 *
 * wilson
 * 10/6/22
 * 10:06 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

//
// Created by wilson on 10/6/22.
//

#ifndef MAIN_CONTROLLER_HAL_I2C_H
#define MAIN_CONTROLLER_HAL_I2C_H

#include "stm32f4xx.h"

class i2c
{
    public:
        typedef enum
        {
            I2C_STATUS_OK       = 0x00U,
            I2C_STATUS_ERROR    = 0x01U,
            I2C_STATUS_BUSY     = 0x02U,
            I2C_STATUS_TIMEOUT  = 0x03U
        } status_t;

        typedef enum
        {
            FLAG_CLEAR = 0x00U,
            FLAG_SET = 0x01U
        } flag_t;

        typedef enum
        {
            I2C_STATE_CHECK_IF_PERIPHERAL_IS_INITIALIZED = 0,
            I2C_STATE_CHECK_IF_BUS_IS_BUSY,
            I2C_STATE_PREPARE_AND_REQUEST_TRANSFER,
            I2C_STATE_TRANSFER,
            I2C_STATE_FINISH_TRANSFER,
            I2C_STATE_PERIPHERAL_BUSY,
            I2C_STATE_PERIPHERAL_ERROR,

        } CONTROLLER_WRITE_STATES;

        typedef struct
        {
            CONTROLLER_WRITE_STATES state;
        } CONTROLLER_WRITE_PROCEDURE;

        explicit i2c(I2C_HandleTypeDef* handle);
        status_t controller_send(uint16_t target_address, uint8_t *data_buffer_pointer, uint16_t size, uint32_t timeout);

    private:

        I2C_HandleTypeDef* peripheral_handle;
        uint8_t i2c_controller_error_message_code;
        CONTROLLER_WRITE_PROCEDURE i2c_controller_write_procedure{};

        void initialize_peripheral();

        I2C_HandleTypeDef* get_peripheral_handle();
        void enable_peripheral();
        void disable_peripheral();
        volatile HAL_I2C_StateTypeDef get_peripheral_state();
        void set_peripheral_state(volatile HAL_I2C_StateTypeDef i2c_state);
        volatile uint32_t get_peripheral_previous_state();
        uint16_t get_transfer_size();
        volatile uint32_t get_error_code();
        void set_error_code(volatile uint32_t error_code);
        uint32_t get_addressing_mode();
        void set_device_mode(volatile HAL_I2C_ModeTypeDef i2c_device_mode);
        void write_data_register(volatile uint32_t data_register_value);
        bool check_flag(uint32_t flag);
        void clear_flag(uint32_t flag);
        void clear_address_flag();
        status_t lock_peripheral();
        void unlock_peripheral();
        void set_control_register_bit(uint32_t control_register_bit);
        void clear_control_register_bit(uint32_t control_register_bit);
        uint32_t read_control_register_bit(uint32_t control_register_bit);
        void generate_start_bit();
        void generate_stop_bit();
        void set_error_state(uint32_t error);
        void set_transfer_state();
        void set_transfer_parameters(uint8_t *data_buffer_pointer, uint16_t size, volatile uint32_t transfer_options);
        void write_next_byte_to_tx_register();
        status_t check_for_nack();
        status_t wait_for_flag(uint32_t flag, flag_t status, uint32_t timeout, uint32_t tick_start);
        status_t wait_for_controller_address_flag(uint32_t flag, uint32_t timeout, uint32_t tick_start);
        status_t controller_request_send(volatile uint32_t current_transfer_options, uint16_t target_address, uint32_t timeout, uint32_t tick_start);
};

#endif //MAIN_CONTROLLER_HAL_I2C_H
