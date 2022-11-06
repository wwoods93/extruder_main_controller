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
#include "peripheral_common.h"
#include "hal_i2c.h"

i2c::i2c(I2C_HandleTypeDef* handle)
{
    i2c_module_handle = handle;
    initialize_i2c_module();
    i2c_controller_error_message_code = ERROR_MESSAGE_CODE_NO_ERROR;
    i2c_controller_write_procedure.state = I2C_STATE_CHECK_IF_PERIPHERAL_IS_INITIALIZED;
}

void i2c::initialize_i2c_module()
{
    i2c_module_handle->Instance = I2C2;
    i2c_module_handle->Init.ClockSpeed = 100000U;
    i2c_module_handle->Init.DutyCycle = I2C_DUTYCYCLE_2;
    i2c_module_handle->Init.OwnAddress1 = 0;
    i2c_module_handle->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    i2c_module_handle->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    i2c_module_handle->Init.OwnAddress2 = 0;
    i2c_module_handle->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    i2c_module_handle->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(i2c_module_handle) != HAL_OK)
        Error_Handler();
}

i2c::i2c_status_t i2c::controller_send(uint16_t target_address, uint8_t *data_buffer_pointer, uint16_t size, uint32_t timeout)
{
    uint32_t tick_start = 0;
    i2c_controller_error_message_code = ERROR_MESSAGE_CODE_NO_ERROR;
    i2c_controller_write_procedure.state = I2C_STATE_CHECK_IF_PERIPHERAL_IS_INITIALIZED;

    switch(i2c_controller_write_procedure.state)
    {
        case I2C_STATE_CHECK_IF_PERIPHERAL_IS_INITIALIZED:
        {
            if (get_i2c_module_state() != I2C_STATE_READY)
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
            lock_i2c_module();
            if (read_control_register_bit(I2C_CR1_REG_PERIPHERAL_ENABLE_BIT) != I2C_CR1_REG_PERIPHERAL_ENABLE_BIT)
                enable_i2c_module();
            clear_control_register_bit(I2C_CR1_REG_POSITION_ENABLE_BIT);
            volatile uint32_t current_transfer_options = I2C_TRANSFER_OPTIONS_DEFAULT;
            set_transfer_state();
            set_transfer_parameters(data_buffer_pointer, size, current_transfer_options);
            i2c_status_t request_result = controller_request_send(current_transfer_options, target_address, timeout, tick_start);

            if (request_result != I2C_STATUS_OK)
            {
                switch (request_result)
                {
                    case I2C_STATUS_TIMEOUT:
                        i2c_controller_error_message_code = START_BIT_SET_FAILURE;
                        break;
                    case I2C_STATUS_ERROR:
                        i2c_controller_error_message_code = ADDRESS_SEND_FAILURE;
                        break;
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
            set_i2c_module_state(I2C_STATE_READY);
            set_device_mode(I2C_MODE_NONE);
            unlock_i2c_module();
            return I2C_STATUS_OK;
        }
        case I2C_STATE_PERIPHERAL_BUSY:
        {
            /*
             * error handling for busy module
             */
            return I2C_STATUS_BUSY;
        }
        case I2C_STATE_PERIPHERAL_ERROR:
        {
            /*
             * error handling for module error
             */
            return I2C_STATUS_ERROR;
        }
        default:
            break;
    }
    return I2C_STATUS_OK;
}

I2C_HandleTypeDef* i2c::get_i2c_module_handle()
{
    return i2c_module_handle;
}

void i2c::enable_i2c_module()
{
    __HAL_I2C_ENABLE(i2c_module_handle);
}

void i2c::disable_i2c_module()
{
    __HAL_I2C_DISABLE(i2c_module_handle);
}

uint8_t i2c::get_i2c_module_state()
{
    return (uint8_t) i2c_module_handle->State;
}

void i2c::set_i2c_module_state(uint8_t i2c_state)
{
    i2c_module_handle->State = (stm32_i2c_state_t) i2c_state;
}

volatile uint32_t i2c::get_i2c_module_previous_state()
{
    return i2c_module_handle->PreviousState;
}

uint16_t i2c::get_transfer_size()
{
    return i2c_module_handle->XferSize;
}

 volatile uint32_t i2c::get_error_code()
{
    return i2c_module_handle->ErrorCode;
}

void i2c::set_error_code(volatile uint32_t error_code)
{
    i2c_module_handle->ErrorCode = error_code;
}

uint32_t i2c::get_addressing_mode()
{
    return i2c_module_handle->Init.AddressingMode;
}

void i2c::set_device_mode(uint8_t i2c_device_mode)
{
    i2c_module_handle->Mode = (volatile HAL_I2C_ModeTypeDef) i2c_device_mode;
}

void i2c::write_data_register(volatile uint32_t data_register_value)
{
    i2c_module_handle->Instance->DR = data_register_value;
}

bool i2c::check_flag(uint32_t flag)
{
    return __HAL_I2C_GET_FLAG(i2c_module_handle, flag);
}

void i2c::clear_flag(uint32_t flag)
{
    __HAL_I2C_CLEAR_FLAG(i2c_module_handle, flag);
}

void i2c::clear_address_flag()
{
    __HAL_I2C_CLEAR_ADDRFLAG(i2c_module_handle);
}

i2c::i2c_status_t i2c::lock_i2c_module()
{
    if (i2c_module_handle->Lock == HAL_LOCKED)
        return I2C_STATUS_BUSY;
    i2c_module_handle->Lock = HAL_LOCKED;
    return I2C_STATUS_OK;
}

void i2c::unlock_i2c_module()
{
    i2c_module_handle->Lock = HAL_UNLOCKED;
}

void i2c::set_control_register_bit(uint32_t control_register_bit)
{
    SET_BIT(i2c_module_handle->Instance->CR1, control_register_bit);
}

void i2c::clear_control_register_bit(uint32_t control_register_bit)
{
    CLEAR_BIT(i2c_module_handle->Instance->CR1, control_register_bit);
}

uint32_t i2c::read_control_register_bit(uint32_t control_register_bit)
{
    return READ_BIT(i2c_module_handle->Instance->CR1, control_register_bit);
}

void i2c::generate_start_bit()
{
    SET_BIT(i2c_module_handle->Instance->CR1, I2C_CR1_REG_GENERATION_START_BIT);
}

void i2c::generate_stop_bit()
{
    SET_BIT(i2c_module_handle->Instance->CR1, I2C_CR1_REG_GENERATION_STOP_BIT);
}

void i2c::set_error_state(uint32_t i2c_error)
{
    i2c_module_handle->PreviousState = I2C_STATE_NONE;
    i2c_module_handle->State         = (stm32_i2c_state_t) I2C_STATE_READY;
    i2c_module_handle->Mode          = (stm32_i2c_mode_t) I2C_MODE_NONE;
    i2c_module_handle->ErrorCode     |= i2c_error;
}

void i2c::set_transfer_state()
{
    i2c_module_handle->State = (stm32_i2c_state_t) I2C_STATE_BUSY_TRANSFERRING;
    i2c_module_handle->Mode = (stm32_i2c_mode_t) I2C_MODE_CONTROLLER;
    i2c_module_handle->ErrorCode = I2C_ERROR_NONE;
}

void i2c::set_transfer_parameters(uint8_t *data_buffer_pointer, uint16_t size, volatile uint32_t transfer_options)
{
    i2c_module_handle->pBuffPtr = data_buffer_pointer;
    i2c_module_handle->XferCount = size;
    i2c_module_handle->XferSize = i2c_module_handle->XferCount;
    i2c_module_handle->XferOptions = transfer_options;
}

void i2c::write_next_byte_to_tx_register()
{
    i2c_module_handle->Instance->DR = *i2c_module_handle->pBuffPtr;
    i2c_module_handle->pBuffPtr++;
    i2c_module_handle->XferCount--;
    i2c_module_handle->XferSize--;
}

i2c::i2c_status_t i2c::check_for_nack()
{
    if (check_flag(I2C_FLAG_ACKNOWLEDGE_FAILED) == FLAG_SET)
    {
        clear_flag(I2C_FLAG_ACKNOWLEDGE_FAILED);
        set_error_state(I2C_ERROR_ACKNOWLEDGE_FAILED);
        unlock_i2c_module();
        return I2C_STATUS_ERROR;
    }
    return I2C_STATUS_OK;
}

i2c::i2c_status_t i2c::wait_for_flag(uint32_t flag, flag_t status, uint32_t timeout, uint32_t tick_start)
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
                unlock_i2c_module();
                return I2C_STATUS_ERROR;
            }
        }
    }
    return I2C_STATUS_OK;
}

i2c::i2c_status_t i2c::wait_for_controller_address_flag(uint32_t flag, uint32_t timeout, uint32_t tick_start)
{
    while (check_flag(flag) == FLAG_CLEAR)
    {
        if (check_flag(I2C_FLAG_ACKNOWLEDGE_FAILED) == FLAG_SET)
        {
            generate_stop_bit();
            clear_flag(I2C_FLAG_ACKNOWLEDGE_FAILED);
            set_error_state(I2C_ERROR_ACKNOWLEDGE_FAILED);
            unlock_i2c_module();
            return I2C_STATUS_ERROR;
        }
        if (timeout != HAL_MAX_DELAY)
        {
            if (((HAL_GetTick() - tick_start) > timeout) || (timeout == 0U))
            {
                set_error_state(I2C_ERROR_TIMEOUT);
                unlock_i2c_module();
                return I2C_STATUS_ERROR;
            }
        }
    }
    return I2C_STATUS_OK;
}

i2c::i2c_status_t i2c::controller_request_send(volatile uint32_t current_transfer_options, uint16_t target_address, uint32_t timeout, uint32_t tick_start)
{
    // if this is the first frame or module has just received a byte, generate a start bit
    if (current_transfer_options == I2C_FIRST_AND_LAST_FRAME
     || current_transfer_options == I2C_FIRST_FRAME
     || current_transfer_options == I2C_TRANSFER_OPTIONS_DEFAULT
     || get_i2c_module_previous_state() == I2C_STATE_CONTROLLER_RECEIVING)
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
