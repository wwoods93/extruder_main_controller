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

/* c/c++ includes */
#include <cstdint>
/* stm32 includes */
#include "stm32f4xx.h"
/* third-party includes */

/* hal includes */
#include "hal_general.h"
/* driver includes */

/* rtos abstraction includes */

/* sys op includes */

/* meta structure includes */
#include "../meta_structure/meta_structure_resource.h"



class i2c : public resource
{
    public:
        /* type definitions */
        typedef enum
        {
            I2C_STATUS_OK                   = 0x00U,
            I2C_STATUS_ERROR                = 0x01U,
            I2C_STATUS_BUSY                 = 0x02U,
            I2C_STATUS_TIMEOUT              = 0x03U
        } status_t;

        typedef enum
        {
            I2C_STATE_RESET                 = 0x00U,
            I2C_STATE_READY                 = 0x20U,
            I2C_STATE_BUSY                  = 0x24U,
            I2C_STATE_BUSY_TX               = 0x21U,
            I2C_STATE_BUSY_RX               = 0x22U,
            I2C_STATE_LISTEN                = 0x28U,
            I2C_STATE_BUSY_TX_AND_LISTENING = 0x29U,
            I2C_STATE_BUSY_RX_AND_LISTENING = 0x2AU,
            I2C_STATE_ABORT                 = 0x60U,
            I2C_STATE_TIMEOUT               = 0xA0U,
            I2C_STATE_ERROR                 = 0xE0U
        } state_t;

        typedef enum
        {
            I2C_MODE_NONE                   = 0x00U,
            I2C_MODE_CONTROLLER             = 0x10U,
            I2C_MODE_PERIPHERAL             = 0x20U,
            I2C_MODE_MEMORY                 = 0x40U
        } mode_t;

        /* constants */
        static constexpr uint32_t I2C_STATE_MASK                        = ((uint32_t)((uint32_t)((uint32_t)I2C_STATE_BUSY_TX | (uint32_t)I2C_STATE_BUSY_RX)& (uint32_t)(~((uint32_t)I2C_STATE_READY))));
        static constexpr uint32_t I2C_STATE_NONE                        = ((uint32_t)(I2C_MODE_NONE));
        static constexpr uint32_t I2C_STATE_CONTROLLER_TRANSMITTING     = ((uint32_t)(((uint32_t)I2C_STATE_BUSY_TX & I2C_STATE_MASK) | (uint32_t)I2C_MODE_CONTROLLER));
        static constexpr uint32_t I2C_STATE_CONTROLLER_RECEIVING        = ((uint32_t)(((uint32_t)I2C_STATE_BUSY_RX & I2C_STATE_MASK) | (uint32_t)I2C_MODE_CONTROLLER));
        static constexpr uint32_t I2C_STATE_TARGET_TRANSMITTING         = ((uint32_t)(((uint32_t)I2C_STATE_BUSY_TX & I2C_STATE_MASK) | (uint32_t)I2C_MODE_PERIPHERAL));
        static constexpr uint32_t I2C_STATE_TARGET_RECEIVING            = ((uint32_t)(((uint32_t)I2C_STATE_BUSY_RX & I2C_STATE_MASK) | (uint32_t)I2C_MODE_PERIPHERAL));

        static constexpr uint32_t I2C_FLAG_BUS_BUSY                     = STM_HAL_I2C_FLAG_BUS_BUSY;
        static constexpr uint32_t I2C_FLAG_START_BIT_SET                = STM_HAL_I2C_FLAG_START_BIT_SET;
        static constexpr uint32_t I2C_FLAG_10_BIT_HEADER_SENT           = STM_HAL_I2C_FLAG_10_BIT_HEADER_SENT;
        static constexpr uint32_t I2C_FLAG_ADDRESS_SENT                 = STM_HAL_I2C_FLAG_ADDRESS_SENT;
        static constexpr uint32_t I2C_FLAG_ACKNOWLEDGE_FAILED           = STM_HAL_I2C_FLAG_ACKNOWLEDGE_FAILED;
        static constexpr uint32_t I2C_FLAG_TRANSMIT_BUFFER_EMPTY        = STM_HAL_I2C_TRANSMIT_BUFFER_EMPTY;
        static constexpr uint32_t I2C_FLAG_BYTE_TRANSFER_FINISHED       = STM_HAL_I2C_FLAG_BYTE_TRANSFER_FINISHED;

        static constexpr uint8_t I2C_ERROR_NONE                         = (uint8_t) STM_HAL_I2C_ERROR_NONE;
        static constexpr uint8_t I2C_ERROR_TIMEOUT                      = (uint8_t) HAL_I2C_ERROR_TIMEOUT;
        static constexpr uint8_t I2C_ERROR_ACKNOWLEDGE_FAILED           = (uint8_t) STM_HAL_I2C_ERROR_ACKNOWLEDGE_FAILED;
        static constexpr uint8_t I2C_WRONG_START                        = (uint8_t) STM_HAL_I2C_WRONG_START;
        static constexpr uint8_t I2C_ERROR_BUSY                         = (uint8_t) STM_HAL_I2C_ERROR_BUSY;
        static constexpr uint8_t I2C_ERROR_ARBITRATION_LOST             = (uint8_t) STM_HAL_I2C_ERROR_ARBITRATION_LOST;
        static constexpr uint8_t I2C_ERROR_OVERRUN                      = (uint8_t) STM_HAL_I2C_ERROR_OVERRUN;
        static constexpr uint8_t I2C_ERROR_DMA_TRANSFER                 = (uint8_t) STM_HAL_I2C_ERROR_DMA_TRANSFER;
        static constexpr uint8_t I2C_ERROR_DMA_PARAMETER                = (uint8_t) STM_HAL_I2C_ERROR_DMA_PARAMETER;
        static constexpr uint8_t I2C_ERROR_SIZE                         = (uint8_t) STM_HAL_I2C_ERROR_SIZE;

        static constexpr uint32_t I2C_CR1_REG_PERIPHERAL_ENABLE_BIT     = I2C_CR1_MODULE_ENABLE;
        static constexpr uint32_t I2C_CR1_REG_POSITION_ENABLE_BIT       = I2C_CR1_POS;
        static constexpr uint32_t I2C_CR1_REG_GENERATION_START_BIT      = I2C_CR1_START;
        static constexpr uint32_t I2C_CR1_REG_GENERATION_STOP_BIT       = I2C_CR1_STOP;

        static constexpr uint32_t I2C_TIMEOUT_35_MS                     = 35U;
        static constexpr uint32_t I2C_TIMEOUT_BUSY_25_MS                = 25U;
        static constexpr uint32_t I2C_TIMEOUT_STOP_5_MS                 = 5U;
        static constexpr uint32_t I2C_TRANSFER_OPTIONS_DEFAULT          = 0xFFFF0000U;

        static constexpr uint8_t ERROR_MESSAGE_CODE_NO_ERROR                = 0x00;
        static constexpr uint8_t START_BIT_SET_FAILURE                      = 0x01;
        static constexpr uint8_t ADDRESS_SEND_FAILURE                       = 0x02;
        static constexpr uint8_t PERIPHERAL_NOT_READY                       = 0x03;
        static constexpr uint8_t BUS_BUSY_TIMEOUT                           = 0x04;
        static constexpr uint8_t TRANSMIT_BUFFER_NOT_EMPTY                  = 0x05;
        static constexpr uint8_t TRANSMIT_BUFFER_NOT_EMPTY_AND_ACK_FAILURE  = 0x06;
        static constexpr uint8_t BYTE_TRANSFER_NOT_FINISHED                 = 0x07;
        static constexpr uint8_t BYTE_TRANSFER_NOT_FINISHED_AND_ACK_FAILURE = 0x08;

        /* macros */
        #define I2C_ENABLE_MODULE(__HANDLE__)           STM_HAL_SET_BIT((__HANDLE__)->instance->CONTROL_REG_1, I2C_CR1_MODULE_ENABLE)
        #define I2C_DISABLE_MODULE(__HANDLE__)          STM_HAL_CLEAR_BIT((__HANDLE__)->instance->CONTROL_REG_1, I2C_CR1_MODULE_ENABLE)
        #define HAL_I2C_FLAG_MASK                       0x0000FFFFU
        #define I2C_GET_FLAG(__HANDLE__, __FLAG__)      ((((uint8_t)((__FLAG__) >> 16U)) == 0x01U) ? (((((__HANDLE__)->instance->STATUS_REG_1)          \
                                                        & ((__FLAG__) & HAL_I2C_FLAG_MASK)) == ((__FLAG__) & HAL_I2C_FLAG_MASK)) ? SET : RESET)         \
                                                        : (((((__HANDLE__)->instance->STATUS_REG_1) & ((__FLAG__) & HAL_I2C_FLAG_MASK)) == ((__FLAG__)  \
                                                        & HAL_I2C_FLAG_MASK)) ? SET : RESET))

        #define I2C_CLEAR_FLAG(__HANDLE__, __FLAG__)    ((__HANDLE__)->instance->STATUS_REG_1 = ~((__FLAG__) & HAL_I2C_FLAG_MASK))
        #define I2C_CLEAR_ADDRESS_FLAG(__HANDLE__)                  \
            do {                                                    \
                __IO uint32_t tmpreg = 0x00U;                       \
                tmpreg = (__HANDLE__)->instance->STATUS_REG_1;      \
                tmpreg = (__HANDLE__)->instance->STATUS_REG_2;      \
                UNUSED(tmpreg);                                     \
        }   while (0)


        /* structures */
        typedef struct
        {
            uint32_t clock_speed;
            uint32_t fast_mode_duty_cycle;
            uint32_t own_address_1;
            uint32_t addressing_mode;
            uint32_t dual_address_mode;
            uint32_t own_address_2;
            uint32_t general_call_mode;
            uint32_t no_stretch_mode;
        } init_t;

        #if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
            typedef struct _handle_t
        #else
            typedef struct
        #endif
            {
                hal_i2c_t               *instance;
                init_t              init;
                uint8_t                 *buffer_ptr;
                uint16_t                transfer_size;
                volatile uint16_t       transfer_counter;
                volatile uint32_t       transfer_options;
                volatile uint32_t       previous_state;
                dma_handle_t            *tx_dma_handle;
                dma_handle_t            *rx_dma_handle;
                hal_lock_t              lock;
                volatile state_t    state;
                volatile mode_t     mode;
                volatile uint32_t       error_code;
                volatile uint32_t       device_address;
                volatile uint32_t       memory_address;
                volatile uint32_t       memory_address_size;
                volatile uint32_t       event_counter;

                #if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
                    void (* MasterTxCpltCallback)(struct __I2C_HandleTypeDef *hi2c);
                    void (* MasterRxCpltCallback)(struct __I2C_HandleTypeDef *hi2c);
                    void (* SlaveTxCpltCallback)(struct __I2C_HandleTypeDef *hi2c);
                    void (* SlaveRxCpltCallback)(struct __I2C_HandleTypeDef *hi2c);
                    void (* ListenCpltCallback)(struct __I2C_HandleTypeDef *hi2c);
                    void (* MemTxCpltCallback)(struct __I2C_HandleTypeDef *hi2c);
                    void (* MemRxCpltCallback)(struct __I2C_HandleTypeDef *hi2c);
                    void (* ErrorCallback)(struct __I2C_HandleTypeDef *hi2c);
                    void (* AbortCpltCallback)(struct __I2C_HandleTypeDef *hi2c);
                    void (* AddrCallback)(struct __I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode);
                    void (* MspInitCallback)(struct __I2C_HandleTypeDef *hi2c);
                    void (* MspDeInitCallback)(struct __I2C_HandleTypeDef *hi2c);
                #endif
            } handle_t;
            #if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
                typedef enum
                {
                    HAL_I2C_MASTER_TX_COMPLETE_CALLBACK_ID    = 0x00U,
                    HAL_I2C_MASTER_RX_COMPLETE_CALLBACK_ID    = 0x01U,
                    HAL_I2C_SLAVE_TX_COMPLETE_CALLBACK_ID     = 0x02U,
                    HAL_I2C_SLAVE_RX_COMPLETE_CALLBACK_ID     = 0x03U,
                    HAL_I2C_LISTEN_COMPLETE_CALLBACK_ID       = 0x04U,
                    HAL_I2C_MEM_TX_COMPLETE_CALLBACK_ID       = 0x05U,
                    HAL_I2C_MEM_RX_COMPLETE_CALLBACK_ID       = 0x06U,
                    HAL_I2C_ERROR_CALLBACK_ID                 = 0x07U,
                    HAL_I2C_ABORT_CALLBACK_ID                 = 0x08U,
                    HAL_I2C_MSP_INIT_CALLBACK_ID              = 0x09U,
                    HAL_I2C_MSP_DE_INIT_CALLBACK_ID           = 0x0AU
                } HAL_I2C_CallbackIDTypeDef;

                typedef  void (*pI2C_CallbackTypeDef)       (I2C_HandleTypeDef *hi2c);
                typedef  void (*pI2C_AddrCallbackTypeDef)   (I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode);
            #endif

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

        typedef volatile state_t       stm32_state_t;
        typedef volatile mode_t        stm32_mode_t;
        /* public member functions */
        explicit i2c(handle_t* handle);
        status_t controller_send(uint16_t target_address, uint8_t *data_buffer_pointer, uint16_t size, uint32_t timeout);
        handle_t* get_i2c_module_handle() const;
        handle_t* i2c_module_handle;
        uint8_t i2c_controller_error_message_code;
        CONTROLLER_WRITE_PROCEDURE i2c_controller_write_procedure;
    private:
        /* private objects */
        status_t initialize_module(handle_t *hi2c);
        static void HAL_I2C_MspInit(handle_t* hi2c);

        /* private member functions */
        void set_configuration();
        void enable_i2c_module();
        void disable_i2c_module();
        uint8_t get_i2c_module_state();
        void set_i2c_module_state(state_t i2c_state);
        volatile uint32_t get_i2c_module_previous_state();
        uint16_t get_transfer_size();
        volatile uint32_t get_error_code();
        void set_error_code(volatile uint32_t error_code);
        uint32_t get_addressing_mode();
        void set_device_mode(mode_t i2c_device_mode);
        void write_data_register(volatile uint32_t data_register_value);
        bool check_flag(uint32_t flag);
        void clear_flag(uint32_t flag);
        void clear_address_flag();
        status_t lock_i2c_module();
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
        status_t check_for_nack();
        status_t wait_for_flag(uint32_t flag, flag_status_t status, uint32_t timeout, uint32_t tick_start);
        status_t wait_for_controller_address_flag(uint32_t flag, uint32_t timeout, uint32_t tick_start);
        status_t controller_request_send(volatile uint32_t current_transfer_options, uint16_t target_address, uint32_t timeout, uint32_t tick_start);
};

#endif //MAIN_CONTROLLER_HAL_I2C_H
