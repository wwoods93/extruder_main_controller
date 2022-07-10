/***********************************************************************************************************************
 * Main_Controller
 * i2c.c
 *
 * wilson
 * 6/30/22
 * 7:26 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

#include <stdbool.h>
#include "stm32f4xx.h"

#include "peripheral_initialization.h"

#include "i2c.h"

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
#define I2C_STATE_MASK                              ((uint32_t)((uint32_t)((uint32_t)I2C_STATE_BUSY_TRANSFERRING | (uint32_t)I2C_STATE_BUSY_RECEIVING) & (uint32_t)(~((uint32_t)I2C_STATE_READY))))
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

typedef enum
{
    I2C_STATE_CHECK_IF_PERIPHERAL_IS_INITIALIZED = 0,
    I2C_STATE_CHECK_IF_BUS_IS_BUSY,
    I2C_STATE_PREPARE_AND_REQUEST_TRANSFER,
    I2C_STATE_TRANSFER,
    I2C_STATE_FINISH_TRANSFER,
    I2C_STATE_PERIPHERAL_BUSY,
    I2C_STATE_PERIPHERAL_ERROR,

} I2C_CONTROLLER_WRITE_STATES;

typedef struct
{
    I2C_CONTROLLER_WRITE_STATES state;
} I2C_CONTROLLER_WRITE_PROCEDURE;

#define ERROR_MESSAGE_CODE_NO_ERROR                         0x00
#define START_BIT_SET_FAILURE                               0x01
#define ADDRESS_SEND_FAILURE                                0x02
#define PERIPHERAL_NOT_READY                                0x03
#define BUS_BUSY_TIMEOUT                                    0x04
#define TRANSMIT_BUFFER_NOT_EMPTY                           0x05
#define TRANSMIT_BUFFER_NOT_EMPTY_AND_ACK_FAILURE           0x06
#define BYTE_TRANSFER_NOT_FINISHED                          0x07
#define BYTE_TRANSFER_NOT_FINISHED_AND_ACK_FAILURE          0x08

static uint8_t i2c_controller_error_message_code;
I2C_CONTROLLER_WRITE_PROCEDURE i2c_controller_write_procedure;

I2C_HandleTypeDef hi2c2;

void MX_I2C2_Init()
{
    hi2c2.Instance = I2C2;
    hi2c2.Init.ClockSpeed = 100000;
    hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != I2C_STATUS_OK)
        Error_Handler();
}

/******************************** functions that interact with the stm32 hal directly *********************************/

I2C_HandleTypeDef* i2c_get_peripheral(void)
{
    return &hi2c2;
}

static void i2c_enable_peripheral(void)
{
    I2C_HandleTypeDef* i2c_peripheral = i2c_get_peripheral();
    __HAL_I2C_ENABLE(i2c_peripheral);
}

static void i2c_disable_peripheral(void)
{
    I2C_HandleTypeDef* i2c_peripheral = i2c_get_peripheral();
    __HAL_I2C_DISABLE(i2c_peripheral);
}

static volatile HAL_I2C_StateTypeDef i2c_peripheral_get_state(void)
{
    I2C_HandleTypeDef* i2c_peripheral = i2c_get_peripheral();
    return i2c_peripheral->State;
}

static void i2c_peripheral_set_state(volatile HAL_I2C_StateTypeDef i2c_state)
{
    I2C_HandleTypeDef* i2c_peripheral = i2c_get_peripheral();
    i2c_peripheral->State = i2c_state;
}

static volatile uint32_t i2c_peripheral_get_previous_state(void)
{
    I2C_HandleTypeDef* i2c_peripheral = i2c_get_peripheral();
    return i2c_peripheral->PreviousState;
}

static uint16_t i2c_peripheral_get_current_transfer_size(void)
{
    I2C_HandleTypeDef* i2c_peripheral = i2c_get_peripheral();
    return i2c_peripheral->XferSize;
}

static volatile uint32_t i2c_peripheral_get_error_code(void)
{
    I2C_HandleTypeDef* i2c_peripheral = i2c_get_peripheral();
    return i2c_peripheral->ErrorCode;
}

static void i2c_peripheral_set_error_code(volatile uint32_t error_code)
{
    I2C_HandleTypeDef* i2c_peripheral = i2c_get_peripheral();
    i2c_peripheral->ErrorCode = error_code;
}

static uint32_t i2c_peripheral_get_addressing_mode(void)
{
    I2C_HandleTypeDef* i2c_peripheral = i2c_get_peripheral();
    return i2c_peripheral->Init.AddressingMode;
};

static void i2c_peripheral_set_device_mode(volatile HAL_I2C_ModeTypeDef i2c_device_mode)
{
    I2C_HandleTypeDef* i2c_peripheral = i2c_get_peripheral();
    i2c_peripheral->Mode = i2c_device_mode;
}

void i2c_peripheral_write_data_register(volatile uint32_t data_register_value)
{
    I2C_HandleTypeDef* i2c_peripheral = i2c_get_peripheral();
    i2c_peripheral->Instance->DR = data_register_value;
}

static bool i2c_check_flag(uint32_t flag)
{
    I2C_HandleTypeDef* i2c_peripheral = i2c_get_peripheral();
    return __HAL_I2C_GET_FLAG(i2c_peripheral, flag);
}

static void i2c_clear_flag(uint32_t flag)
{
    I2C_HandleTypeDef* i2c_peripheral = i2c_get_peripheral();
    __HAL_I2C_CLEAR_FLAG(i2c_peripheral, flag);
}

static void i2c_clear_address_flag(void)
{
    I2C_HandleTypeDef* i2c_peripheral = i2c_get_peripheral();
    __HAL_I2C_CLEAR_ADDRFLAG(i2c_peripheral);

}
static i2c_status_t i2c_lock_process(void)
{
    I2C_HandleTypeDef *i2c_peripheral = i2c_get_peripheral();
    if (i2c_peripheral->Lock == HAL_LOCKED)
        return I2C_STATUS_BUSY;
    i2c_peripheral->Lock = HAL_LOCKED;
    return I2C_STATUS_OK;
}

static void i2c_unlock_process(void)
{
    I2C_HandleTypeDef* i2c_peripheral = i2c_get_peripheral();
    i2c_peripheral->Lock = HAL_UNLOCKED;
}

static void i2c_set_control_register_bit(uint32_t control_register_bit)
{
    I2C_HandleTypeDef* i2c_peripheral = i2c_get_peripheral();
    SET_BIT(i2c_peripheral->Instance->CR1, control_register_bit);
}

uint32_t i2c_read_control_register_bit(uint32_t control_register_bit)
{
    I2C_HandleTypeDef* i2c_peripheral = i2c_get_peripheral();
    return READ_BIT(i2c_peripheral->Instance->CR1, control_register_bit);
}

static void i2c_clear_control_register_bit(uint32_t control_register_bit)
{
    I2C_HandleTypeDef* i2c_peripheral = i2c_get_peripheral();
    CLEAR_BIT(i2c_peripheral->Instance->CR1, control_register_bit);
}

static void i2c_generate_start_bit(void)
{
    I2C_HandleTypeDef* i2c_peripheral = i2c_get_peripheral();
    SET_BIT(i2c_peripheral->Instance->CR1, I2C_CR1_REG_GENERATION_START_BIT);
}

static void i2c_generate_stop_bit(void)
{
    I2C_HandleTypeDef* i2c_peripheral = i2c_get_peripheral();
    SET_BIT(i2c_peripheral->Instance->CR1, I2C_CR1_REG_GENERATION_STOP_BIT);
}

static void i2c_set_peripheral_error_state(uint32_t i2c_error)
{
    I2C_HandleTypeDef* i2c_peripheral = i2c_get_peripheral();
    i2c_peripheral->PreviousState = I2C_STATE_NONE;
    i2c_peripheral->State         = I2C_STATE_READY;
    i2c_peripheral->Mode          = I2C_MODE_NONE;
    i2c_peripheral->ErrorCode     |= i2c_error;
}

static void i2c_set_peripheral_transfer_state(void)
{
    I2C_HandleTypeDef* i2c_peripheral = i2c_get_peripheral();
    i2c_peripheral->State = I2C_STATE_BUSY_TRANSFERRING;
    i2c_peripheral->Mode = I2C_MODE_CONTROLLER;
    i2c_peripheral->ErrorCode = I2C_ERROR_NONE;
}

void i2c_set_peripheral_transfer_parameters(uint8_t *data_buffer_pointer, uint16_t size, volatile uint32_t transfer_options)
{
    I2C_HandleTypeDef* i2c_peripheral = i2c_get_peripheral();
    i2c_peripheral->pBuffPtr = data_buffer_pointer;
    i2c_peripheral->XferCount = size;
    i2c_peripheral->XferSize = i2c_peripheral->XferCount;
    i2c_peripheral->XferOptions = transfer_options;
}

static void write_next_byte_to_tx_register(void)
{
    I2C_HandleTypeDef* i2c_peripheral = i2c_get_peripheral();
    i2c_peripheral->Instance->DR = *i2c_peripheral->pBuffPtr;
    i2c_peripheral->pBuffPtr++;
    i2c_peripheral->XferCount--;
    i2c_peripheral->XferSize--;
}

/**********************************************************************************************************************/

static i2c_status_t i2c_check_for_nack(void)
{
    if (i2c_check_flag(I2C_FLAG_ACKNOWLEDGE_FAILED) == FLAG_SET)
    {
        i2c_clear_flag(I2C_FLAG_ACKNOWLEDGE_FAILED);
        i2c_set_peripheral_error_state(I2C_ERROR_ACKNOWLEDGE_FAILED);
        i2c_unlock_process();
        return I2C_STATUS_ERROR;
    }
    return I2C_STATUS_OK;
}

static i2c_status_t i2c_wait_for_flag(uint32_t flag, flag_status_t status, uint32_t timeout, uint32_t tick_start)
{
    while (i2c_check_flag(flag) == status)
    {
        if ((flag == I2C_FLAG_TRANSMIT_BUFFER_EMPTY) && (i2c_check_for_nack() != I2C_STATUS_OK))
            return I2C_STATUS_ERROR;

        if (timeout != HAL_MAX_DELAY)
        {
            if (((HAL_GetTick() - tick_start) > timeout) || (timeout == 0U))
            {
                i2c_set_peripheral_error_state(I2C_ERROR_TIMEOUT);

                i2c_unlock_process();

                return I2C_STATUS_ERROR;
            }
        }
    }
    return I2C_STATUS_OK;
}

static i2c_status_t i2c_wait_for_controller_address_flag(uint32_t flag, uint32_t timeout, uint32_t tick_start)
{
    while (i2c_check_flag(flag) == FLAG_CLEAR)
    {
        if (i2c_check_flag(I2C_FLAG_ACKNOWLEDGE_FAILED) == FLAG_SET)
        {
            i2c_generate_stop_bit();
            i2c_clear_flag(I2C_FLAG_ACKNOWLEDGE_FAILED);
            i2c_set_peripheral_error_state(I2C_ERROR_ACKNOWLEDGE_FAILED);
            i2c_unlock_process();
            return I2C_STATUS_ERROR;
        }

        if (timeout != HAL_MAX_DELAY)
        {
            if (((HAL_GetTick() - tick_start) > timeout) || (timeout == 0U))
            {
                i2c_set_peripheral_error_state(I2C_ERROR_TIMEOUT);
                i2c_unlock_process();
                return I2C_STATUS_ERROR;
            }
        }
    }
    return I2C_STATUS_OK;
}

static i2c_status_t i2c_controller_request_write(volatile uint32_t current_transfer_options, uint16_t target_address, uint32_t timeout, uint32_t tick_start)
{
    // if this is the first frame, generate a start bit
    if ((current_transfer_options == I2C_FIRST_AND_LAST_FRAME) || (current_transfer_options == I2C_FIRST_FRAME) || (current_transfer_options == I2C_TRANSFER_OPTIONS_DEFAULT))
        i2c_generate_start_bit();
    // if the controller has just finished receiving, generate a start bit
    else if (i2c_peripheral_get_previous_state() == I2C_STATE_CONTROLLER_RECEIVING)
        i2c_generate_start_bit();

    // wait until start bit is set
    if (i2c_wait_for_flag(I2C_FLAG_START_BIT_SET, FLAG_CLEAR, timeout, tick_start) != I2C_STATUS_OK)
    {
        // confirm start bit set, throw error if not
        if (i2c_read_control_register_bit(I2C_CR1_REG_GENERATION_START_BIT) == I2C_CR1_REG_GENERATION_START_BIT)
            i2c_peripheral_set_error_code(I2C_WRONG_START);

        return I2C_STATUS_TIMEOUT;
    }

    // 7-bit mode write target address to data register
    if (i2c_peripheral_get_addressing_mode() == I2C_ADDRESSINGMODE_7BIT)
    {
        volatile uint32_t address_write_7_bit = I2C_7BIT_ADD_WRITE(target_address);
        i2c_peripheral_write_data_register(address_write_7_bit);
    }
    else
    {
        // 10-bit mode write header to data register
        volatile uint32_t header_write_10_bit = I2C_10BIT_HEADER_WRITE(target_address);
        i2c_peripheral_write_data_register(header_write_10_bit);
        // 10-bit mode confirm header sent, return error otherwise
        if (i2c_wait_for_controller_address_flag(I2C_FLAG_10_BIT_HEADER_SENT, timeout, tick_start) != I2C_STATUS_OK)
            return I2C_STATUS_ERROR;
        // 10-bit mode write address to data register
        volatile uint32_t address_write_10_bit = I2C_10BIT_ADDRESS(target_address);
        i2c_peripheral_write_data_register(address_write_10_bit);
    }
    // wait for address sent flag, return error otherwise
    if (i2c_wait_for_controller_address_flag(I2C_FLAG_ADDRESS_SENT, timeout, tick_start) != I2C_STATUS_OK)
        return I2C_STATUS_ERROR;

    return I2C_STATUS_OK;
}

i2c_status_t i2c_controller_write(uint16_t target_address, uint8_t *data_buffer_pointer, uint16_t size, uint32_t timeout)
{
    uint32_t tick_start = 0;
    i2c_controller_error_message_code = ERROR_MESSAGE_CODE_NO_ERROR;
    i2c_controller_write_procedure.state = I2C_STATE_CHECK_IF_PERIPHERAL_IS_INITIALIZED;

    switch(i2c_controller_write_procedure.state)
    {
        case I2C_STATE_CHECK_IF_PERIPHERAL_IS_INITIALIZED:
        {
            if (i2c_peripheral_get_state() != I2C_STATE_READY)
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

            if (i2c_wait_for_flag(I2C_FLAG_BUS_BUSY, FLAG_SET, I2C_TIMEOUT_BUSY_25_MS, tick_start) != I2C_STATUS_OK)
            {
                i2c_controller_error_message_code = BUS_BUSY_TIMEOUT;
                i2c_controller_write_procedure.state = I2C_STATE_PERIPHERAL_BUSY;
            }
            else
                i2c_controller_write_procedure.state = I2C_STATE_PREPARE_AND_REQUEST_TRANSFER;
        }

        case I2C_STATE_PREPARE_AND_REQUEST_TRANSFER:
        {
            i2c_lock_process();

            if (i2c_read_control_register_bit(I2C_CR1_REG_PERIPHERAL_ENABLE_BIT) != I2C_CR1_REG_PERIPHERAL_ENABLE_BIT)
                i2c_enable_peripheral();

            i2c_clear_control_register_bit(I2C_CR1_REG_POSITION_ENABLE_BIT);

            volatile uint32_t current_transfer_options = I2C_TRANSFER_OPTIONS_DEFAULT;
            i2c_set_peripheral_transfer_state();
            i2c_set_peripheral_transfer_parameters(data_buffer_pointer, size, current_transfer_options);

            i2c_status_t write_request_result = i2c_controller_request_write(current_transfer_options, target_address, timeout, tick_start);
            if (write_request_result != I2C_STATUS_OK)
            {
                switch (write_request_result)
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
            i2c_clear_address_flag();
            bool i2c_status_error_has_occurred = false;

            while (i2c_peripheral_get_current_transfer_size() && (i2c_status_error_has_occurred == false))
            {
                if (i2c_wait_for_flag(I2C_FLAG_TRANSMIT_BUFFER_EMPTY, FLAG_CLEAR, timeout, tick_start) != I2C_STATUS_OK)
                {
                    if (i2c_peripheral_get_error_code() == I2C_ERROR_ACKNOWLEDGE_FAILED)
                    {
                        i2c_generate_stop_bit();
                        i2c_controller_error_message_code = TRANSMIT_BUFFER_NOT_EMPTY_AND_ACK_FAILURE;
                    }
                    else
                    {
                        i2c_controller_error_message_code = TRANSMIT_BUFFER_NOT_EMPTY;
                    }

                    i2c_status_error_has_occurred = true;
                    break;
                }

                write_next_byte_to_tx_register();

                if ((i2c_check_flag(I2C_FLAG_BYTE_TRANSFER_FINISHED) == FLAG_SET) && (i2c_peripheral_get_current_transfer_size() != 0U))
                    write_next_byte_to_tx_register();

                if (i2c_wait_for_flag(I2C_FLAG_BYTE_TRANSFER_FINISHED, FLAG_CLEAR, timeout, tick_start) != I2C_STATUS_OK)
                {
                    if (i2c_peripheral_get_error_code() == I2C_ERROR_ACKNOWLEDGE_FAILED)
                    {
                        i2c_generate_stop_bit();
                        i2c_controller_error_message_code = BYTE_TRANSFER_NOT_FINISHED_AND_ACK_FAILURE;
                    }
                    else
                    {
                        i2c_controller_error_message_code = BYTE_TRANSFER_NOT_FINISHED;
                    }

                    i2c_status_error_has_occurred = true;
                    break;
                }
            }
            if (i2c_status_error_has_occurred == true)
                i2c_controller_write_procedure.state = I2C_STATE_PERIPHERAL_ERROR;
            else
                i2c_controller_write_procedure.state = I2C_STATE_FINISH_TRANSFER;
        }

        case I2C_STATE_FINISH_TRANSFER:
        {
            i2c_generate_stop_bit();
            i2c_peripheral_set_state(I2C_STATE_READY);
            i2c_peripheral_set_device_mode(I2C_MODE_NONE);
            i2c_unlock_process();

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
}
