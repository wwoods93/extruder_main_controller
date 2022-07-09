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

#include "stm32f4xx.h"

#include "peripheral_initialization.h"

#include "i2c.h"

/******************************************* renaming for stm32 constants *********************************************/

#define I2C_MODE_NONE                           (__IO HAL_I2C_ModeTypeDef) 0x00U
#define I2C_MODE_CONTROLLER                     (__IO HAL_I2C_ModeTypeDef) 0x10U
#define I2C_MODE_PERIPHERAL                     (__IO HAL_I2C_ModeTypeDef) 0x20U
#define I2C_MODE_MEMORY                         (__IO HAL_I2C_ModeTypeDef) 0x40U

#define I2C_STATE_RESET                                             (__IO HAL_I2C_StateTypeDef) 0x00U
#define I2C_STATE_READY                                             (__IO HAL_I2C_StateTypeDef) 0x20U
#define I2C_STATE_BUSY                                              (__IO HAL_I2C_StateTypeDef) 0x24U
#define I2C_STATE_BUSY_TRANSFERRING                                 (__IO HAL_I2C_StateTypeDef) 0x21U
#define I2C_STATE_BUSY_RECEIVING                                    (__IO HAL_I2C_StateTypeDef) 0x22U
#define I2C_STATE_LISTENING                                         (__IO HAL_I2C_StateTypeDef) 0x28U
#define I2C_STATE_BUSY_LISTENING_AND_TRANSFERRING                   (__IO HAL_I2C_StateTypeDef) 0x29U
#define I2C_STATE_BUSY_LISTENING_AND_RECEIVING                      (__IO HAL_I2C_StateTypeDef) 0x2AU
#define I2C_STATE_ABORTING_USER_REQUEST                             (__IO HAL_I2C_StateTypeDef) 0x60U
#define I2C_STATE_TIMEOUT                                           (__IO HAL_I2C_StateTypeDef) 0xA0U
#define I2C_STATE_ERROR                                             (__IO HAL_I2C_StateTypeDef) 0xE0U

/* constants for PreviousState member of I2C_HandleTypeDef struct */
#define I2C_STATE_MASK                          ((uint32_t)((uint32_t)((uint32_t)I2C_STATE_BUSY_TRANSFERRING | (uint32_t)I2C_STATE_BUSY_RECEIVING) & (uint32_t)(~((uint32_t)I2C_STATE_READY))))
#define I2C_STATE_NONE                          ((uint32_t)(I2C_MODE_NONE))
#define I2C_STATE_CONTROLLER_TRANSMITTING       ((uint32_t)(((uint32_t)I2C_STATE_BUSY_TRANSFERRING & I2C_STATE_MASK) | (uint32_t)I2C_MODE_CONTROLLER))
#define I2C_STATE_CONTROLLER_RECEIVING          ((uint32_t)(((uint32_t)I2C_STATE_BUSY_RECEIVING & I2C_STATE_MASK) | (uint32_t)I2C_MODE_CONTROLLER))
#define I2C_STATE_TARGET_TRANSMITTING           ((uint32_t)(((uint32_t)I2C_STATE_BUSY_TRANSFERRING & I2C_STATE_MASK) | (uint32_t)I2C_MODE_PERIPHERAL))
#define I2C_STATE_TARGET_RECEIVING              ((uint32_t)(((uint32_t)I2C_STATE_BUSY_RECEIVING & I2C_STATE_MASK) | (uint32_t)I2C_MODE_PERIPHERAL))

/* flags */
#define I2C_FLAG_BUS_BUSY                               I2C_FLAG_BUSY
#define I2C_FLAG_START_BIT_SET                          I2C_FLAG_SB
#define I2C_FLAG_10_BIT_HEADER_SENT                     I2C_FLAG_ADD10
#define I2C_FLAG_ADDRESS_SENT                           I2C_FLAG_ADDR
#define I2C_FLAG_ACKNOWLEDGE_FAILED                     I2C_FLAG_AF
#define I2C_FLAG_TRANSMIT_BUFFER_EMPTY                  I2C_FLAG_TXE
#define I2C_FLAG_BYTE_TRANSFER_FINISHED                 I2C_FLAG_BTF

/* errors */
#define I2C_ERROR_NONE                                  HAL_I2C_ERROR_NONE
#define I2C_ERROR_TIMEOUT                               HAL_I2C_ERROR_TIMEOUT
#define I2C_ERROR_ACKNOWLEDGE_FAILED                    HAL_I2C_ERROR_AF
#define I2C_WRONG_START                                 HAL_I2C_WRONG_START
#define I2C_ERROR_BUSY                                  HAL_I2C_ERROR_BERR
#define I2C_ERROR_ARBITRATION_LOST                      HAL_I2C_ERROR_ARLO
#define I2C_ERROR_OVER_OR_UNDER_RUN                     HAL_I2C_ERROR_OVR
#define I2C_ERROR_DMA_TRANSFER                          HAL_I2C_ERROR_DMA
#define I2C_ERROR_DMA_PARAMETER                         HAL_I2C_ERROR_DMA_PARAM
#define I2C_ERROR_SIZE_MANAGEMENT                       HAL_I2C_ERROR_SIZE

/* control register bit masks */
#define I2C_CR1_REG_PERIPHERAL_ENABLE_BIT               I2C_CR1_PE
#define I2C_CR1_REG_POSITION_ENABLE_BIT                 I2C_CR1_POS
#define I2C_CR1_REG_GENERATION_START_BIT                I2C_CR1_START
#define I2C_CR1_REG_GENERATION_STOP_BIT                 I2C_CR1_STOP

/* i2c timeout constants */
#define I2C_TIMEOUT_35_MS                               35U
#define I2C_TIMEOUT_BUSY_25_MS                          25U
#define I2C_TIMEOUT_STOP_5_MS                           5U

/* default for XferOptions member of I2C_HandleTypeDef struct */
#define I2C_TRANSFER_OPTION_DEFAULT                     0xFFFF0000U


/**********************************************************************************************************************/

typedef enum
{
    I2C_STATE_CHECK_IF_PERIPHERAL_IS_INITIALIZED = 0,
    I2C_STATE_CHECK_IF_BUS_IS_BUSY,
    I2C_STATE_PREPARE_AND_REQUEST_TRANSFER,
    I2C_STATE_TRANSFER,
    I2C_STATE_FINISH_TRANSFER,

} I2C_CONTROLLER_WRITE_STATES;

typedef struct
{
    I2C_CONTROLLER_WRITE_STATES state;
} I2C_CONTROLLER_WRITE_PROCEDURE;

I2C_CONTROLLER_WRITE_PROCEDURE i2c_controller_write_procedure;

I2C_HandleTypeDef hi2c2;

I2C_HandleTypeDef* i2c_get_module(void)
{
    return &hi2c2;
}
/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
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
    if (HAL_I2C_Init(&hi2c2) != HAL_OK)
        Error_Handler();
}

static HAL_StatusTypeDef i2c_check_for_nack(I2C_HandleTypeDef *i2c_module)
{
    if (__HAL_I2C_GET_FLAG(i2c_module, I2C_FLAG_ACKNOWLEDGE_FAILED) == FLAG_SET)
    {
        __HAL_I2C_CLEAR_FLAG(i2c_module, I2C_FLAG_ACKNOWLEDGE_FAILED);

        i2c_module->PreviousState = I2C_STATE_NONE;
        i2c_module->State         = I2C_STATE_READY;
        i2c_module->Mode          = I2C_MODE_NONE;
        i2c_module->ErrorCode     |= I2C_ERROR_ACKNOWLEDGE_FAILED;

        __HAL_UNLOCK(i2c_module);
        return HAL_ERROR;
    }
    return HAL_OK;
}

static HAL_StatusTypeDef i2c_wait_for_flag(I2C_HandleTypeDef *i2c_module, uint32_t flag, flag_status_t status, uint32_t timeout, uint32_t tick_start)
{
    while (__HAL_I2C_GET_FLAG(i2c_module, flag) == status)
    {
        if ((flag == I2C_FLAG_TRANSMIT_BUFFER_EMPTY) && (i2c_check_for_nack(i2c_module) != HAL_OK))
            return HAL_ERROR;

        if (timeout != HAL_MAX_DELAY)
        {
            if (((HAL_GetTick() - tick_start) > timeout) || (timeout == 0U))
            {
                i2c_module->PreviousState = I2C_STATE_NONE;
                i2c_module->State         = (__IO HAL_I2C_StateTypeDef)0x20U;
                i2c_module->Mode          =  I2C_MODE_NONE;
                i2c_module->ErrorCode     |= I2C_ERROR_TIMEOUT;

                __HAL_UNLOCK(i2c_module);

                return HAL_ERROR;
            }
        }
    }
    return HAL_OK;
}

static HAL_StatusTypeDef i2c_wait_for_controller_address_flag(I2C_HandleTypeDef *i2c_module, uint32_t flag, uint32_t timeout, uint32_t tick_start)
{
    while (__HAL_I2C_GET_FLAG(i2c_module, flag) == FLAG_CLEAR)
    {
        if (__HAL_I2C_GET_FLAG(i2c_module, I2C_FLAG_ACKNOWLEDGE_FAILED) == FLAG_SET)
        {
            /* Generate Stop */
            SET_BIT(i2c_module->Instance->CR1, I2C_CR1_REG_GENERATION_STOP_BIT);

            /* Clear AF flag */
            __HAL_I2C_CLEAR_FLAG(i2c_module, I2C_FLAG_ACKNOWLEDGE_FAILED);

            i2c_module->PreviousState       = I2C_STATE_NONE;
            i2c_module->State               = I2C_STATE_READY;
            i2c_module->Mode                = I2C_MODE_NONE;
            i2c_module->ErrorCode           |= I2C_ERROR_ACKNOWLEDGE_FAILED;

            /* Process Unlocked */
            __HAL_UNLOCK(i2c_module);

            return HAL_ERROR;
        }

        /* Check for the timeout */
        if (timeout != HAL_MAX_DELAY)
        {
            if (((HAL_GetTick() - tick_start) > timeout) || (timeout == 0U))
            {
                i2c_module->PreviousState       = I2C_STATE_NONE;
                i2c_module->State               = I2C_STATE_READY;
                i2c_module->Mode                = I2C_MODE_NONE;
                i2c_module->ErrorCode           |= I2C_ERROR_TIMEOUT;

                /* Process Unlocked */
                __HAL_UNLOCK(i2c_module);

                return HAL_ERROR;
            }
        }
    }
    return HAL_OK;
}


static HAL_StatusTypeDef i2c_controller_request_write(I2C_HandleTypeDef *i2c_module, uint16_t target_address, uint32_t timeout, uint32_t tick_start)
{
    /* Declaration of temporary variable to prevent undefined behavior of volatile usage */
    uint32_t CurrentXferOptions = i2c_module->XferOptions;

    /* Generate Start condition if first transfer */
    if ((CurrentXferOptions == I2C_FIRST_AND_LAST_FRAME) || (CurrentXferOptions == I2C_FIRST_FRAME) || (CurrentXferOptions == I2C_TRANSFER_OPTION_DEFAULT))
    {
        SET_BIT(i2c_module->Instance->CR1, I2C_CR1_REG_GENERATION_START_BIT);
    }
    else if (i2c_module->PreviousState == I2C_STATE_CONTROLLER_RECEIVING)
    {
        SET_BIT(i2c_module->Instance->CR1, I2C_CR1_REG_GENERATION_START_BIT);
    }
    else
    {
        /* do nothing */
    }

    /* Wait until SB flag is set */
    if (i2c_wait_for_flag(i2c_module, I2C_FLAG_START_BIT_SET, FLAG_CLEAR, timeout, tick_start) != HAL_OK)
    {
        if (READ_BIT(i2c_module->Instance->CR1, I2C_CR1_REG_GENERATION_START_BIT) == I2C_CR1_REG_GENERATION_START_BIT)
        {
            i2c_module->ErrorCode = I2C_WRONG_START;
        }
        return HAL_TIMEOUT;
    }

    if (i2c_module->Init.AddressingMode == I2C_ADDRESSINGMODE_7BIT)
    {
        i2c_module->Instance->DR = I2C_7BIT_ADD_WRITE(target_address);
    }
    else
    {
        /* Send header of slave address */
        i2c_module->Instance->DR = I2C_10BIT_HEADER_WRITE(target_address);

        /* Wait until ADD10 flag is set */
        if (i2c_wait_for_controller_address_flag(i2c_module, I2C_FLAG_10_BIT_HEADER_SENT, timeout, tick_start) != HAL_OK)
        {
            return HAL_ERROR;
        }
        /* Send slave address */
        i2c_module->Instance->DR = I2C_10BIT_ADDRESS(target_address);
    }

    /* Wait until ADDR flag is set */
    if (i2c_wait_for_controller_address_flag(i2c_module, I2C_FLAG_ADDRESS_SENT, timeout, tick_start) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

static void write_tx_register(I2C_HandleTypeDef *i2c_module)
{
    i2c_module->Instance->DR = *i2c_module->pBuffPtr;

    i2c_module->pBuffPtr++;
    i2c_module->XferCount--;
    i2c_module->XferSize--;
}

HAL_StatusTypeDef i2c_controller_write(I2C_HandleTypeDef *i2c_module, uint16_t target_address, uint8_t *data_buffer_pointer, uint16_t Size, uint32_t timeout)
{
    uint32_t tick_start = 0;

    i2c_controller_write_procedure.state = I2C_STATE_CHECK_IF_PERIPHERAL_IS_INITIALIZED;

    switch(i2c_controller_write_procedure.state)
    {
        case I2C_STATE_CHECK_IF_PERIPHERAL_IS_INITIALIZED:
        {
            if (i2c_module->State != I2C_STATE_READY)
            {
                return HAL_BUSY;
            }
            i2c_controller_write_procedure.state = I2C_STATE_CHECK_IF_BUS_IS_BUSY;
        }

        case I2C_STATE_CHECK_IF_BUS_IS_BUSY:
        {
            tick_start = HAL_GetTick();

            if (i2c_wait_for_flag(i2c_module, I2C_FLAG_BUS_BUSY, FLAG_SET, I2C_TIMEOUT_BUSY_25_MS, tick_start) != HAL_OK)
            {
                return HAL_BUSY;
            }
            i2c_controller_write_procedure.state = I2C_STATE_PREPARE_AND_REQUEST_TRANSFER;
        }

        case I2C_STATE_PREPARE_AND_REQUEST_TRANSFER:
        {
            /* Process Locked */
            __HAL_LOCK(i2c_module);

            /* Check if the I2C is already enabled */
            if ((i2c_module->Instance->CR1 & I2C_CR1_REG_PERIPHERAL_ENABLE_BIT) != I2C_CR1_REG_PERIPHERAL_ENABLE_BIT)
                __HAL_I2C_ENABLE(i2c_module);

            /* Disable Pos */
            CLEAR_BIT(i2c_module->Instance->CR1, I2C_CR1_REG_POSITION_ENABLE_BIT);

            i2c_module->State = I2C_STATE_BUSY_TRANSFERRING;
            i2c_module->Mode = I2C_MODE_CONTROLLER;
            i2c_module->ErrorCode = I2C_ERROR_NONE;

            /* Prepare transfer parameters */
            i2c_module->pBuffPtr = data_buffer_pointer;
            i2c_module->XferCount = Size;
            i2c_module->XferSize = i2c_module->XferCount;
            i2c_module->XferOptions = I2C_TRANSFER_OPTION_DEFAULT;

            if (i2c_controller_request_write(i2c_module, target_address, timeout, tick_start) != HAL_OK)
            {
                return HAL_ERROR;
            }

            __HAL_I2C_CLEAR_ADDRFLAG(i2c_module);
            i2c_controller_write_procedure.state = I2C_STATE_TRANSFER;
        }

        case I2C_STATE_TRANSFER:
        {
            while (i2c_module->XferSize > 0U)
            {
                /* Wait until TXE flag is set */
                if (i2c_wait_for_flag(i2c_module, I2C_FLAG_TRANSMIT_BUFFER_EMPTY, FLAG_CLEAR, timeout, tick_start) !=
                    HAL_OK)
                {
                    if (i2c_module->ErrorCode == I2C_ERROR_ACKNOWLEDGE_FAILED)
                    {
                        /* Generate Stop */
                        SET_BIT(i2c_module->Instance->CR1, I2C_CR1_REG_GENERATION_STOP_BIT);
                    }
                    return HAL_ERROR;
                }

                write_tx_register(i2c_module);

                if ((__HAL_I2C_GET_FLAG(i2c_module, I2C_FLAG_BYTE_TRANSFER_FINISHED) == FLAG_SET) && (i2c_module->XferSize != 0U))
                {
                    write_tx_register(i2c_module);
                }

                /* Wait until BTF flag is set */
                if (i2c_wait_for_flag(i2c_module, I2C_FLAG_BYTE_TRANSFER_FINISHED, FLAG_CLEAR, timeout, tick_start) !=
                    HAL_OK)
                {
                    if (i2c_module->ErrorCode == I2C_ERROR_ACKNOWLEDGE_FAILED)
                    {
                        /* Generate Stop */
                        SET_BIT(i2c_module->Instance->CR1, I2C_CR1_REG_GENERATION_STOP_BIT);
                    }
                    return HAL_ERROR;
                }
            }
            i2c_controller_write_procedure.state = I2C_STATE_FINISH_TRANSFER;n
        }

        case I2C_STATE_FINISH_TRANSFER:
        {
            SET_BIT(i2c_module->Instance->CR1, I2C_CR1_REG_GENERATION_STOP_BIT);

            i2c_module->State = I2C_STATE_READY;
            i2c_module->Mode = I2C_MODE_NONE;

            __HAL_UNLOCK(i2c_module);

            return HAL_OK;
        }

        default:
            break;
    }
}
