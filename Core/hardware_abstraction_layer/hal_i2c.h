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
#ifndef MAIN_CONTROLLER_HAL_I2C_H
#define MAIN_CONTROLLER_HAL_I2C_H

#include "stm32f4xx.h"

class i2c
{
    public:
        /****************************************** compile time constants ********************************************/
        /* device modes */
        static constexpr uint8_t I2C_MODE_NONE                             = (uint8_t) HAL_I2C_MODE_NONE;
        static constexpr uint8_t I2C_MODE_CONTROLLER                       = (uint8_t) HAL_I2C_MODE_MASTER;
        static constexpr uint8_t I2C_MODE_PERIPHERAL                       = (uint8_t) HAL_I2C_MODE_SLAVE;
        static constexpr uint8_t I2C_MODE_MEMORY                           = (uint8_t) HAL_I2C_MODE_MEM;
        /* module states */
        static constexpr uint8_t I2C_STATE_RESET                           = (uint8_t) HAL_I2C_STATE_RESET;
        static constexpr uint8_t I2C_STATE_READY                           = (uint8_t) 0x20U;
        static constexpr uint8_t I2C_STATE_BUSY                            = (uint8_t) HAL_I2C_STATE_BUSY;
        static constexpr uint8_t I2C_STATE_BUSY_TRANSFERRING               = (uint8_t) HAL_I2C_STATE_BUSY_TX;
        static constexpr uint8_t I2C_STATE_BUSY_RECEIVING                  = (uint8_t) HAL_I2C_STATE_BUSY_RX;
        static constexpr uint8_t I2C_STATE_LISTENING                       = (uint8_t) HAL_I2C_STATE_LISTEN;
        static constexpr uint8_t I2C_STATE_BUSY_LISTENING_AND_TRANSFERRING = (uint8_t) HAL_I2C_STATE_BUSY_TX_LISTEN;
        static constexpr uint8_t I2C_STATE_BUSY_LISTENING_AND_RECEIVING    = (uint8_t) HAL_I2C_STATE_BUSY_RX_LISTEN;
        static constexpr uint8_t I2C_STATE_ABORTING_USER_REQUEST           = (uint8_t) HAL_I2C_STATE_ABORT;
        static constexpr uint8_t I2C_STATE_TIMEOUT                         = (uint8_t) HAL_I2C_STATE_TIMEOUT;
        static constexpr uint8_t I2C_STATE_ERROR                           = (uint8_t) HAL_I2C_STATE_ERROR;
        /* constants for PreviousState member of I2C_HandleTypeDef struct */
        static constexpr uint32_t I2C_STATE_MASK = ((uint32_t)((uint32_t)((uint32_t)I2C_STATE_BUSY_TRANSFERRING | (uint32_t)I2C_STATE_BUSY_RECEIVING) & (uint32_t)(~((uint32_t)I2C_STATE_READY))));
        static constexpr uint32_t I2C_STATE_NONE = ((uint32_t)(I2C_MODE_NONE));
        static constexpr uint32_t I2C_STATE_CONTROLLER_TRANSMITTING    = ((uint32_t)(((uint32_t)I2C_STATE_BUSY_TRANSFERRING & I2C_STATE_MASK) | (uint32_t)I2C_MODE_CONTROLLER));
        static constexpr uint32_t I2C_STATE_CONTROLLER_RECEIVING       = ((uint32_t)(((uint32_t)I2C_STATE_BUSY_RECEIVING & I2C_STATE_MASK) | (uint32_t)I2C_MODE_CONTROLLER));
        static constexpr uint32_t I2C_STATE_TARGET_TRANSMITTING        = ((uint32_t)(((uint32_t)I2C_STATE_BUSY_TRANSFERRING & I2C_STATE_MASK) | (uint32_t)I2C_MODE_PERIPHERAL));
        static constexpr uint32_t I2C_STATE_TARGET_RECEIVING           = ((uint32_t)(((uint32_t)I2C_STATE_BUSY_RECEIVING & I2C_STATE_MASK) | (uint32_t)I2C_MODE_PERIPHERAL));
        /* flags */
        static constexpr uint32_t I2C_FLAG_BUS_BUSY                    = I2C_FLAG_BUSY;
        static constexpr uint32_t I2C_FLAG_START_BIT_SET               = I2C_FLAG_SB;
        static constexpr uint32_t I2C_FLAG_10_BIT_HEADER_SENT          = I2C_FLAG_ADD10;
        static constexpr uint32_t I2C_FLAG_ADDRESS_SENT                = I2C_FLAG_ADDR;
        static constexpr uint32_t I2C_FLAG_ACKNOWLEDGE_FAILED          = I2C_FLAG_AF;
        static constexpr uint32_t I2C_FLAG_TRANSMIT_BUFFER_EMPTY       = I2C_FLAG_TXE;
        static constexpr uint32_t I2C_FLAG_BYTE_TRANSFER_FINISHED      = I2C_FLAG_BTF;
        /* module error codes */
        static constexpr uint8_t I2C_ERROR_NONE                        = (uint8_t) HAL_I2C_ERROR_NONE;
        static constexpr uint8_t I2C_ERROR_TIMEOUT                     = (uint8_t) HAL_I2C_ERROR_TIMEOUT;
        static constexpr uint8_t I2C_ERROR_ACKNOWLEDGE_FAILED          = (uint8_t) HAL_I2C_ERROR_AF;
        static constexpr uint8_t I2C_WRONG_START                       = (uint8_t) HAL_I2C_WRONG_START;
        static constexpr uint8_t I2C_ERROR_BUSY                        = (uint8_t) HAL_I2C_ERROR_BERR;
        static constexpr uint8_t I2C_ERROR_ARBITRATION_LOST            = (uint8_t) HAL_I2C_ERROR_ARLO;
        static constexpr uint8_t I2C_ERROR_OVER_OR_UNDER_RUN           = (uint8_t) HAL_I2C_ERROR_OVR;
        static constexpr uint8_t I2C_ERROR_DMA_TRANSFER                = (uint8_t) HAL_I2C_ERROR_DMA;
        static constexpr uint8_t I2C_ERROR_DMA_PARAMETER               = (uint8_t) HAL_I2C_ERROR_DMA_PARAM;
        static constexpr uint8_t I2C_ERROR_SIZE_MANAGEMENT             = (uint8_t) HAL_I2C_ERROR_SIZE;
        /* control register bit masks */
        static constexpr uint32_t I2C_CR1_REG_PERIPHERAL_ENABLE_BIT    = I2C_CR1_PE;
        static constexpr uint32_t I2C_CR1_REG_POSITION_ENABLE_BIT      = I2C_CR1_POS;
        static constexpr uint32_t I2C_CR1_REG_GENERATION_START_BIT     = I2C_CR1_START;
        static constexpr uint32_t I2C_CR1_REG_GENERATION_STOP_BIT      = I2C_CR1_STOP;
        /* i2c timeout constants */
        static constexpr uint32_t I2C_TIMEOUT_35_MS                    = 35U;
        static constexpr uint32_t I2C_TIMEOUT_BUSY_25_MS               = 25U;
        static constexpr uint32_t I2C_TIMEOUT_STOP_5_MS                = 5U;
        /* default for XferOptions member of I2C_HandleTypeDef struct */
        static constexpr uint32_t I2C_TRANSFER_OPTIONS_DEFAULT         = 0xFFFF0000U;
        /* state machine error codes */
        static constexpr uint8_t ERROR_MESSAGE_CODE_NO_ERROR                        = 0x00;
        static constexpr uint8_t START_BIT_SET_FAILURE                              = 0x01;
        static constexpr uint8_t ADDRESS_SEND_FAILURE                               = 0x02;
        static constexpr uint8_t PERIPHERAL_NOT_READY                               = 0x03;
        static constexpr uint8_t BUS_BUSY_TIMEOUT                                   = 0x04;
        static constexpr uint8_t TRANSMIT_BUFFER_NOT_EMPTY                          = 0x05;
        static constexpr uint8_t TRANSMIT_BUFFER_NOT_EMPTY_AND_ACK_FAILURE          = 0x06;
        static constexpr uint8_t BYTE_TRANSFER_NOT_FINISHED                         = 0x07;
        static constexpr uint8_t BYTE_TRANSFER_NOT_FINISHED_AND_ACK_FAILURE         = 0x08;
        /********************************************* type definitions ***********************************************/
        typedef enum
        {
            I2C_STATUS_OK       = 0x00U,
            I2C_STATUS_ERROR    = 0x01U,
            I2C_STATUS_BUSY     = 0x02U,
            I2C_STATUS_TIMEOUT  = 0x03U
        } i2c_status_t;

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

        typedef volatile HAL_I2C_StateTypeDef       stm32_i2c_state_t;
        typedef volatile HAL_I2C_ModeTypeDef        stm32_i2c_mode_t;
        /****************************************** public member functions *******************************************/
        explicit i2c(I2C_HandleTypeDef* handle);
        i2c_status_t controller_send(uint16_t target_address, uint8_t *data_buffer_pointer, uint16_t size, uint32_t timeout);

    private:
        /********************************************** private objects ***********************************************/
        I2C_HandleTypeDef* i2c_module_handle;
        uint8_t i2c_controller_error_message_code;
        CONTROLLER_WRITE_PROCEDURE i2c_controller_write_procedure{};
        /****************************************** private member functions ******************************************/
        void initialize_i2c_module();
        I2C_HandleTypeDef* get_i2c_module_handle();
        void enable_i2c_module();
        void disable_i2c_module();
        uint8_t get_i2c_module_state();
        void set_i2c_module_state(uint8_t i2c_state);
        volatile uint32_t get_i2c_module_previous_state();
        uint16_t get_transfer_size();
        volatile uint32_t get_error_code();
        void set_error_code(volatile uint32_t error_code);
        uint32_t get_addressing_mode();
        void set_device_mode(uint8_t i2c_device_mode);
        void write_data_register(volatile uint32_t data_register_value);
        bool check_flag(uint32_t flag);
        void clear_flag(uint32_t flag);
        void clear_address_flag();
        i2c_status_t lock_i2c_module();
        void unlock_i2c_module();
        void set_control_register_bit(uint32_t control_register_bit);
        void clear_control_register_bit(uint32_t control_register_bit);
        uint32_t read_control_register_bit(uint32_t control_register_bit);
        void generate_start_bit();
        void generate_stop_bit();
        void set_error_state(uint32_t error);
        void set_transfer_state();
        void set_transfer_parameters(uint8_t *data_buffer_pointer, uint16_t size, volatile uint32_t transfer_options);
        void write_next_byte_to_tx_register();
        i2c_status_t check_for_nack();
        i2c_status_t wait_for_flag(uint32_t flag, flag_t status, uint32_t timeout, uint32_t tick_start);
        i2c_status_t wait_for_controller_address_flag(uint32_t flag, uint32_t timeout, uint32_t tick_start);
        i2c_status_t controller_request_send(volatile uint32_t current_transfer_options, uint16_t target_address, uint32_t timeout, uint32_t tick_start);
};

#endif //MAIN_CONTROLLER_HAL_I2C_H
