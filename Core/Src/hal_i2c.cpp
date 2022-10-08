/***********************************************************************************************************************
 * Main_Controller
 * hal_i2c.cpp
 *
 * wilson
 * 10/6/22
 * 10:06 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

#include "stm32f4xx.h"
#include "peripheral_initialization.h"
#include "hal_i2c.h"

/******************************************* renaming for stm32 constants *********************************************/
/* device modes */
#define I2C_MODE_NONE                               (volatile HAL_I2C_ModeTypeDef) 0x00U
#define I2C_MODE_CONTROLLER                         (volatile HAL_I2C_ModeTypeDef) 0x10U
#define I2C_MODE_PERIPHERAL                         (volatile HAL_I2C_ModeTypeDef) 0x20U
#define I2C_MODE_MEMORY                             (volatile HAL_I2C_ModeTypeDef) 0x40U
/* peripheral states */
#define I2C_STATE_RESET                             (volatile HAL_I2C_StateTypeDef) 0x00U
#define I2C_STATE_READY                             (volatile HAL_I2C_StateTypeDef) 0x20U
#define I2C_STATE_BUSY                              (volatile HAL_I2C_StateTypeDef) 0x24U
#define I2C_STATE_BUSY_TRANSFERRING                 (volatile HAL_I2C_StateTypeDef) 0x21U
#define I2C_STATE_BUSY_RECEIVING                    (volatile HAL_I2C_StateTypeDef) 0x22U
#define I2C_STATE_LISTENING                         (volatile HAL_I2C_StateTypeDef) 0x28U
#define I2C_STATE_BUSY_LISTENING_AND_TRANSFERRING   (volatile HAL_I2C_StateTypeDef) 0x29U
#define I2C_STATE_BUSY_LISTENING_AND_RECEIVING      (volatile HAL_I2C_StateTypeDef) 0x2AU
#define I2C_STATE_ABORTING_USER_REQUEST             (volatile HAL_I2C_StateTypeDef) 0x60U
#define I2C_STATE_TIMEOUT                           (volatile HAL_I2C_StateTypeDef) 0xA0U
#define I2C_STATE_ERROR                             (volatile HAL_I2C_StateTypeDef) 0xE0U
/* constants for PreviousState member of I2C_HandleTypeDef struct */
constexpr uint32_t I2C_STATE_MASK = ((uint32_t)((uint32_t)((uint32_t)I2C_STATE_BUSY_TRANSFERRING | (uint32_t)I2C_STATE_BUSY_RECEIVING) & (uint32_t)(~((uint32_t)I2C_STATE_READY))));
#define I2C_STATE_NONE                              ((uint32_t)(I2C_MODE_NONE))
#define I2C_STATE_CONTROLLER_TRANSMITTING           ((uint32_t)(((uint32_t)I2C_STATE_BUSY_TRANSFERRING & I2C_STATE_MASK) | (uint32_t)I2C_MODE_CONTROLLER))
#define I2C_STATE_CONTROLLER_RECEIVING              ((uint32_t)(((uint32_t)I2C_STATE_BUSY_RECEIVING & I2C_STATE_MASK) | (uint32_t)I2C_MODE_CONTROLLER))
#define I2C_STATE_TARGET_TRANSMITTING               ((uint32_t)(((uint32_t)I2C_STATE_BUSY_TRANSFERRING & I2C_STATE_MASK) | (uint32_t)I2C_MODE_PERIPHERAL))
#define I2C_STATE_TARGET_RECEIVING                  ((uint32_t)(((uint32_t)I2C_STATE_BUSY_RECEIVING & I2C_STATE_MASK) | (uint32_t)I2C_MODE_PERIPHERAL))
/* flags */
#define I2C_FLAG_BUS_BUSY                           I2C_FLAG_BUSY
#define I2C_FLAG_START_BIT_SET                      I2C_FLAG_SB
#define I2C_FLAG_10_BIT_HEADER_SENT                 I2C_FLAG_ADD10
#define I2C_FLAG_ADDRESS_SENT                       I2C_FLAG_ADDR
#define I2C_FLAG_ACKNOWLEDGE_FAILED                 I2C_FLAG_AF
#define I2C_FLAG_TRANSMIT_BUFFER_EMPTY              I2C_FLAG_TXE
#define I2C_FLAG_BYTE_TRANSFER_FINISHED             I2C_FLAG_BTF
/* errors */
#define I2C_ERROR_NONE                              HAL_I2C_ERROR_NONE
#define I2C_ERROR_TIMEOUT                           HAL_I2C_ERROR_TIMEOUT
#define I2C_ERROR_ACKNOWLEDGE_FAILED                HAL_I2C_ERROR_AF
#define I2C_WRONG_START                             HAL_I2C_WRONG_START
#define I2C_ERROR_BUSY                              HAL_I2C_ERROR_BERR
#define I2C_ERROR_ARBITRATION_LOST                  HAL_I2C_ERROR_ARLO
#define I2C_ERROR_OVER_OR_UNDER_RUN                 HAL_I2C_ERROR_OVR
#define I2C_ERROR_DMA_TRANSFER                      HAL_I2C_ERROR_DMA
#define I2C_ERROR_DMA_PARAMETER                     HAL_I2C_ERROR_DMA_PARAM
#define I2C_ERROR_SIZE_MANAGEMENT                   HAL_I2C_ERROR_SIZE
/* control register bit masks */
#define I2C_CR1_REG_PERIPHERAL_ENABLE_BIT           I2C_CR1_PE
#define I2C_CR1_REG_POSITION_ENABLE_BIT             I2C_CR1_POS
#define I2C_CR1_REG_GENERATION_START_BIT            I2C_CR1_START
#define I2C_CR1_REG_GENERATION_STOP_BIT             I2C_CR1_STOP
/* i2c timeout constants */
#define I2C_TIMEOUT_35_MS                           35U
#define I2C_TIMEOUT_BUSY_25_MS                      25U
#define I2C_TIMEOUT_STOP_5_MS                       5U
/* default for XferOptions member of I2C_HandleTypeDef struct */
#define I2C_TRANSFER_OPTIONS_DEFAULT                0xFFFF0000U

/**********************************************************************************************************************/

#define ERROR_MESSAGE_CODE_NO_ERROR                         0x00
#define START_BIT_SET_FAILURE                               0x01
#define ADDRESS_SEND_FAILURE                                0x02
#define PERIPHERAL_NOT_READY                                0x03
#define BUS_BUSY_TIMEOUT                                    0x04
#define TRANSMIT_BUFFER_NOT_EMPTY                           0x05
#define TRANSMIT_BUFFER_NOT_EMPTY_AND_ACK_FAILURE           0x06
#define BYTE_TRANSFER_NOT_FINISHED                          0x07
#define BYTE_TRANSFER_NOT_FINISHED_AND_ACK_FAILURE          0x08

i2c::i2c(I2C_HandleTypeDef* handle)
{
    peripheral_handle = handle;
    initialize_peripheral();
    i2c_controller_error_message_code = ERROR_MESSAGE_CODE_NO_ERROR;
    i2c_controller_write_procedure.state = I2C_STATE_CHECK_IF_PERIPHERAL_IS_INITIALIZED;
}

void i2c::initialize_peripheral()
{
    peripheral_handle->Instance = I2C2;
    peripheral_handle->Init.ClockSpeed = 100000U;
    peripheral_handle->Init.DutyCycle = I2C_DUTYCYCLE_2;
    peripheral_handle->Init.OwnAddress1 = 0;
    peripheral_handle->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    peripheral_handle->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    peripheral_handle->Init.OwnAddress2 = 0;
    peripheral_handle->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    peripheral_handle->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(peripheral_handle) != HAL_OK)
        Error_Handler();
}

i2c::status_t i2c::controller_send(uint16_t target_address, uint8_t *data_buffer_pointer, uint16_t size, uint32_t timeout)
{
    uint32_t tick_start = 0;
    i2c_controller_error_message_code = ERROR_MESSAGE_CODE_NO_ERROR;
    i2c_controller_write_procedure.state = I2C_STATE_CHECK_IF_PERIPHERAL_IS_INITIALIZED;

    switch(i2c_controller_write_procedure.state)
    {
        case I2C_STATE_CHECK_IF_PERIPHERAL_IS_INITIALIZED:
        {
            if (get_peripheral_state() != I2C_STATE_READY)
            {
                i2c_controller_error_message_code = PERIPHERAL_NOT_READY;
                i2c_controller_write_procedure.state = I2C_STATE_PERIPHERAL_BUSY;
            }
            else
                i2c_controller_write_procedure.state = I2C_STATE_CHECK_IF_BUS_IS_BUSY;
        }
        case I2C_STATE_CHECK_IF_BUS_IS_BUSY:
        {
            tick_start = HAL_GetTick();

            if (wait_for_flag(I2C_FLAG_BUS_BUSY, FLAG_SET, I2C_TIMEOUT_BUSY_25_MS, tick_start) != I2C_STATUS_OK)
            {
                i2c_controller_error_message_code = BUS_BUSY_TIMEOUT;
                i2c_controller_write_procedure.state = I2C_STATE_PERIPHERAL_BUSY;
            }
            else
                i2c_controller_write_procedure.state = I2C_STATE_PREPARE_AND_REQUEST_TRANSFER;
        }
        case I2C_STATE_PREPARE_AND_REQUEST_TRANSFER:
        {
            lock_peripheral();
            if (read_control_register_bit(I2C_CR1_REG_PERIPHERAL_ENABLE_BIT) != I2C_CR1_REG_PERIPHERAL_ENABLE_BIT)
                enable_peripheral();
            clear_control_register_bit(I2C_CR1_REG_POSITION_ENABLE_BIT);

            volatile uint32_t current_transfer_options = I2C_TRANSFER_OPTIONS_DEFAULT;
            set_transfer_state();
            set_transfer_parameters(data_buffer_pointer, size, current_transfer_options);

            status_t send_request_result = controller_request_send(current_transfer_options, target_address, timeout, tick_start);
            if (send_request_result != I2C_STATUS_OK)
            {
                switch (send_request_result)
                {
                    case I2C_STATUS_TIMEOUT:
                    {
                        i2c_controller_error_message_code = START_BIT_SET_FAILURE;
                        break;
                    }
                    case I2C_STATUS_ERROR:
                    {
                        i2c_controller_error_message_code = ADDRESS_SEND_FAILURE;
                        break;
                    }
                    default:
                        break;
                }
                i2c_controller_write_procedure.state = I2C_STATE_PERIPHERAL_ERROR;
            }
            else
                i2c_controller_write_procedure.state = I2C_STATE_TRANSFER;
        }
        case I2C_STATE_TRANSFER:
        {
            clear_address_flag();
            bool i2c_status_error_has_occurred = false;

            while (get_transfer_size() && !i2c_status_error_has_occurred)
            {
                if (wait_for_flag(I2C_FLAG_TRANSMIT_BUFFER_EMPTY, FLAG_CLEAR, timeout, tick_start) != I2C_STATUS_OK)
                {
                    if (get_error_code() == I2C_ERROR_ACKNOWLEDGE_FAILED)
                    {
                        generate_stop_bit();
                        i2c_controller_error_message_code = TRANSMIT_BUFFER_NOT_EMPTY_AND_ACK_FAILURE;
                    }
                    else
                        i2c_controller_error_message_code = TRANSMIT_BUFFER_NOT_EMPTY;
                    i2c_status_error_has_occurred = true;
                    break;
                }

                write_next_byte_to_tx_register();

                if ((check_flag(I2C_FLAG_BYTE_TRANSFER_FINISHED) == FLAG_SET) && (get_transfer_size() != 0U))
                    write_next_byte_to_tx_register();

                if (wait_for_flag(I2C_FLAG_BYTE_TRANSFER_FINISHED, FLAG_CLEAR, timeout, tick_start) != I2C_STATUS_OK)
                {
                    if (get_error_code() == I2C_ERROR_ACKNOWLEDGE_FAILED)
                    {
                        generate_stop_bit();
                        i2c_controller_error_message_code = BYTE_TRANSFER_NOT_FINISHED_AND_ACK_FAILURE;
                    }
                    else
                        i2c_controller_error_message_code = BYTE_TRANSFER_NOT_FINISHED;
                    i2c_status_error_has_occurred = true;
                    break;
                }
            }
            if (i2c_status_error_has_occurred)
                i2c_controller_write_procedure.state = I2C_STATE_PERIPHERAL_ERROR;
            else
                i2c_controller_write_procedure.state = I2C_STATE_FINISH_TRANSFER;
        }
        case I2C_STATE_FINISH_TRANSFER:
        {
            generate_stop_bit();
            set_peripheral_state(I2C_STATE_READY);
            set_device_mode(I2C_MODE_NONE);
            unlock_peripheral();
            return I2C_STATUS_OK;
        }
        case I2C_STATE_PERIPHERAL_BUSY:
        {
            /*
             * error handling for busy peripheral
             */
            return I2C_STATUS_BUSY;
        }
        case I2C_STATE_PERIPHERAL_ERROR:
        {
            /*
             * error handling for peripheral error
             */
            return I2C_STATUS_ERROR;
        }
        default:
            break;
    }
    return I2C_STATUS_OK;
}

I2C_HandleTypeDef* i2c::get_peripheral_handle()
{
    return peripheral_handle;
}

void i2c::enable_peripheral()
{
    __HAL_I2C_ENABLE(peripheral_handle);
}

void i2c::disable_peripheral()
{
    __HAL_I2C_DISABLE(peripheral_handle);
}

volatile HAL_I2C_StateTypeDef i2c::get_peripheral_state()
{
    return peripheral_handle->State;
}

void i2c::set_peripheral_state(volatile HAL_I2C_StateTypeDef i2c_state)
{
    peripheral_handle->State = i2c_state;
}

volatile uint32_t i2c::get_peripheral_previous_state()
{
    return peripheral_handle->PreviousState;
}

uint16_t i2c::get_transfer_size()
{
    return peripheral_handle->XferSize;
}

 volatile uint32_t i2c::get_error_code()
{
    return peripheral_handle->ErrorCode;
}

void i2c::set_error_code(volatile uint32_t error_code)
{
    peripheral_handle->ErrorCode = error_code;
}

uint32_t i2c::get_addressing_mode()
{
    return peripheral_handle->Init.AddressingMode;
}

void i2c::set_device_mode(volatile HAL_I2C_ModeTypeDef i2c_device_mode)
{
    peripheral_handle->Mode = i2c_device_mode;
}

void i2c::write_data_register(volatile uint32_t data_register_value)
{
    peripheral_handle->Instance->DR = data_register_value;
}

bool i2c::check_flag(uint32_t flag)
{
    return __HAL_I2C_GET_FLAG(peripheral_handle, flag);
}

void i2c::clear_flag(uint32_t flag)
{
    __HAL_I2C_CLEAR_FLAG(peripheral_handle, flag);
}

void i2c::clear_address_flag()
{
    __HAL_I2C_CLEAR_ADDRFLAG(peripheral_handle);
}

i2c::status_t i2c::lock_peripheral()
{
    if (peripheral_handle->Lock == HAL_LOCKED)
        return I2C_STATUS_BUSY;
    peripheral_handle->Lock = HAL_LOCKED;
    return I2C_STATUS_OK;
}

void i2c::unlock_peripheral()
{
    peripheral_handle->Lock = HAL_UNLOCKED;
}

void i2c::set_control_register_bit(uint32_t control_register_bit)
{
    SET_BIT(peripheral_handle->Instance->CR1, control_register_bit);
}

void i2c::clear_control_register_bit(uint32_t control_register_bit)
{
    CLEAR_BIT(peripheral_handle->Instance->CR1, control_register_bit);
}

uint32_t i2c::read_control_register_bit(uint32_t control_register_bit)
{
    return READ_BIT(peripheral_handle->Instance->CR1, control_register_bit);
}

void i2c::generate_start_bit()
{
    SET_BIT(peripheral_handle->Instance->CR1, I2C_CR1_REG_GENERATION_START_BIT);
}

void i2c::generate_stop_bit()
{
    SET_BIT(peripheral_handle->Instance->CR1, I2C_CR1_REG_GENERATION_STOP_BIT);
}

void i2c::set_error_state(uint32_t i2c_error)
{
    peripheral_handle->PreviousState = I2C_STATE_NONE;
    peripheral_handle->State         = I2C_STATE_READY;
    peripheral_handle->Mode          = I2C_MODE_NONE;
    peripheral_handle->ErrorCode     |= i2c_error;
}

void i2c::set_transfer_state()
{
    peripheral_handle->State = I2C_STATE_BUSY_TRANSFERRING;
    peripheral_handle->Mode = I2C_MODE_CONTROLLER;
    peripheral_handle->ErrorCode = I2C_ERROR_NONE;
}

void i2c::set_transfer_parameters(uint8_t *data_buffer_pointer, uint16_t size, volatile uint32_t transfer_options)
{
    peripheral_handle->pBuffPtr = data_buffer_pointer;
    peripheral_handle->XferCount = size;
    peripheral_handle->XferSize = peripheral_handle->XferCount;
    peripheral_handle->XferOptions = transfer_options;
}

void i2c::write_next_byte_to_tx_register()
{
    peripheral_handle->Instance->DR = *peripheral_handle->pBuffPtr;
    peripheral_handle->pBuffPtr++;
    peripheral_handle->XferCount--;
    peripheral_handle->XferSize--;
}

/**********************************************************************************************************************/

i2c::status_t i2c::check_for_nack()
{
    if (check_flag(I2C_FLAG_ACKNOWLEDGE_FAILED) == FLAG_SET)
    {
        clear_flag(I2C_FLAG_ACKNOWLEDGE_FAILED);
        set_error_state(I2C_ERROR_ACKNOWLEDGE_FAILED);
        unlock_peripheral();
        return I2C_STATUS_ERROR;
    }
    return I2C_STATUS_OK;
}

i2c::status_t i2c::wait_for_flag(uint32_t flag, flag_t status, uint32_t timeout, uint32_t tick_start)
{
    while (check_flag(flag) == status)
    {
        if ((flag == I2C_FLAG_TRANSMIT_BUFFER_EMPTY) && (check_for_nack() != I2C_STATUS_OK))
            return I2C_STATUS_ERROR;

        if (timeout != HAL_MAX_DELAY)
        {
            if (((HAL_GetTick() - tick_start) > timeout) || (timeout == 0U))
            {
                set_error_state(I2C_ERROR_TIMEOUT);

                unlock_peripheral();

                return I2C_STATUS_ERROR;
            }
        }
    }
    return I2C_STATUS_OK;
}

i2c::status_t i2c::wait_for_controller_address_flag(uint32_t flag, uint32_t timeout, uint32_t tick_start)
{
    while (check_flag(flag) == FLAG_CLEAR)
    {
        if (check_flag(I2C_FLAG_ACKNOWLEDGE_FAILED) == FLAG_SET)
        {
            generate_stop_bit();
            clear_flag(I2C_FLAG_ACKNOWLEDGE_FAILED);
            set_error_state(I2C_ERROR_ACKNOWLEDGE_FAILED);
            unlock_peripheral();
            return I2C_STATUS_ERROR;
        }
        if (timeout != HAL_MAX_DELAY)
        {
            if (((HAL_GetTick() - tick_start) > timeout) || (timeout == 0U))
            {
                set_error_state(I2C_ERROR_TIMEOUT);
                unlock_peripheral();
                return I2C_STATUS_ERROR;
            }
        }
    }
    return I2C_STATUS_OK;
}

i2c::status_t i2c::controller_request_send(volatile uint32_t current_transfer_options, uint16_t target_address, uint32_t timeout, uint32_t tick_start)
{
    // if this is the first frame, generate a start bit
    if (current_transfer_options == I2C_FIRST_AND_LAST_FRAME || current_transfer_options == I2C_FIRST_FRAME || current_transfer_options == I2C_TRANSFER_OPTIONS_DEFAULT)
        generate_start_bit();
        // if the controller has just finished receiving, generate a start bit
    else if (get_peripheral_previous_state() == I2C_STATE_CONTROLLER_RECEIVING)
        generate_start_bit();
    // wait until start bit is set
    if (wait_for_flag(I2C_FLAG_START_BIT_SET, FLAG_CLEAR, timeout, tick_start) != I2C_STATUS_OK)
    {
        // confirm start bit set, throw error if not
        if (read_control_register_bit(I2C_CR1_REG_GENERATION_START_BIT) == I2C_CR1_REG_GENERATION_START_BIT)
            set_error_code(I2C_WRONG_START);
        return I2C_STATUS_TIMEOUT;
    }
    // 7-bit mode write target address to data register
    if (get_addressing_mode() == I2C_ADDRESSINGMODE_7BIT)
    {
        volatile uint32_t address_write_7_bit = I2C_7BIT_ADD_WRITE(target_address);
        write_data_register(address_write_7_bit);
    }
    else
    {
        // 10-bit mode write header to data register
        volatile uint32_t header_write_10_bit = I2C_10BIT_HEADER_WRITE(target_address);
        write_data_register(header_write_10_bit);
        // 10-bit mode confirm header sent, return error otherwise
        if (wait_for_controller_address_flag(I2C_FLAG_10_BIT_HEADER_SENT, timeout, tick_start) != I2C_STATUS_OK)
            return I2C_STATUS_ERROR;
        // 10-bit mode write address to data register
        volatile uint32_t address_write_10_bit = I2C_10BIT_ADDRESS(target_address);
        write_data_register(address_write_10_bit);
    }
    // wait for address sent flag, return error otherwise
    if (wait_for_controller_address_flag(I2C_FLAG_ADDRESS_SENT, timeout, tick_start) != I2C_STATUS_OK)
        return I2C_STATUS_ERROR;
    return I2C_STATUS_OK;
}
