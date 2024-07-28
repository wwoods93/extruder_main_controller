///***********************************************************************************************************************
// * Main_Controller
// * hal_i2c.cpp
// *
// * wilson
// * 10/6/22
// * 10:06 PM
// *
// * Description:
// *
// **********************************************************************************************************************/
//
//
///* c/c++ includes */
//
///* stm32 includes */
//#include "stm32f4xx.h"
///* third-party includes */
//
///* hal includes */
//#include "peripheral_common.h"
///* driver includes */
//
///* rtos abstraction includes */
//
///* sys op includes */
//
///* meta structure includes */
//#include "../meta_structure/meta_structure_resource.h"
///* hal_i2c header */
//#include "hal_i2c.h"
//
//i2c::i2c(handle_t* handle)
//{
//    i2c_module_handle = handle;
//    set_configuration();
//    i2c_controller_error_message_code = ERROR_MESSAGE_CODE_NO_ERROR;
//    i2c_controller_write_procedure.state = I2C_STATE_CHECK_IF_PERIPHERAL_IS_INITIALIZED;
//}
//
// i2c::status_t i2c::initialize_module(handle_t *hi2c)
//{
//    uint32_t freqrange;
//    uint32_t pclk1;
//
//    if (hi2c == nullptr)
//        return I2C_STATUS_ERROR;
//
//    assert_param(IS_I2C_ALL_INSTANCE(hi2c->instance));
//    assert_param(IS_I2C_CLOCK_SPEED(hi2c->init.clock_speed));
//    assert_param(IS_I2C_DUTY_CYCLE(hi2c->init.fast_mode_duty_cycle));
//    assert_param(IS_I2C_OWN_ADDRESS1(hi2c->init.own_address_1));
//    assert_param(IS_I2C_ADDRESSING_MODE(hi2c->init.addressing_mode));
//    assert_param(IS_I2C_DUAL_ADDRESS(hi2c->init.dual_address_mode));
//    assert_param(IS_I2C_OWN_ADDRESS2(hi2c->init.own_address_2));
//    assert_param(IS_I2C_GENERAL_CALL(hi2c->init.general_call_mode));
//    assert_param(IS_I2C_NO_STRETCH(hi2c->init.no_stretch_mode));
//
//    if (hi2c->state == I2C_STATE_RESET)
//    {
//        hi2c->lock = HAL_MODULE_UNLOCKED;
//
//        #if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
//
//            hi2c->MasterTxCpltCallback = HAL_I2C_MasterTxCpltCallback;
//            hi2c->MasterRxCpltCallback = HAL_I2C_MasterRxCpltCallback;
//            hi2c->SlaveTxCpltCallback  = HAL_I2C_SlaveTxCpltCallback;
//            hi2c->SlaveRxCpltCallback  = HAL_I2C_SlaveRxCpltCallback;
//            hi2c->ListenCpltCallback   = HAL_I2C_ListenCpltCallback;
//            hi2c->MemTxCpltCallback    = HAL_I2C_MemTxCpltCallback;
//            hi2c->MemRxCpltCallback    = HAL_I2C_MemRxCpltCallback;
//            hi2c->ErrorCallback        = HAL_I2C_ErrorCallback;
//            hi2c->AbortCpltCallback    = HAL_I2C_AbortCpltCallback;
//            hi2c->AddrCallback         = HAL_I2C_AddrCallback;
//
//            if (hi2c->MspInitCallback == NULL)
//                hi2c->MspInitCallback = HAL_I2C_MspInit;
//            hi2c->MspInitCallback(hi2c);
//        #else
//                HAL_I2C_MspInit(hi2c);
//        #endif
//    }
//
//    hi2c->state = I2C_STATE_BUSY;
//    I2C_DISABLE_MODULE(hi2c);
//    hi2c->instance->CONTROL_REG_1 |= I2C_CR1_SWRST;
//    hi2c->instance->CONTROL_REG_1 &= ~I2C_CR1_SWRST;
//    pclk1 = HAL_RCC_GetPCLK1Freq();
//
//    if (I2C_MIN_PCLK_FREQ(pclk1, hi2c->init.clock_speed) == 1U)
//        return I2C_STATUS_ERROR;
//
//    freqrange = I2C_FREQRANGE(pclk1);
//
//    MODIFY_REG(hi2c->instance->CONTROL_REG_2, I2C_CR2_FREQ, freqrange);
//    MODIFY_REG(hi2c->instance->RISE_TIME_REG, I2C_TRISE_TRISE, I2C_RISE_TIME(freqrange, hi2c->init.clock_speed));
//    MODIFY_REG(hi2c->instance->CLOCK_CONTROL_REG, (I2C_CCR_FS | I2C_CCR_DUTY | I2C_CCR_CCR), I2C_SPEED(pclk1, hi2c->init.clock_speed, hi2c->init.fast_mode_duty_cycle));
//    MODIFY_REG(hi2c->instance->CONTROL_REG_1, (I2C_CR1_ENGC | I2C_CR1_NOSTRETCH), (hi2c->init.general_call_mode | hi2c->init.no_stretch_mode));
//    MODIFY_REG(hi2c->instance->OWN_ADDRESS_REG_1, (I2C_OAR1_ADDMODE | I2C_OAR1_ADD8_9 | I2C_OAR1_ADD1_7 | I2C_OAR1_ADD0), (hi2c->init.addressing_mode | hi2c->init.own_address_1));
//    MODIFY_REG(hi2c->instance->OWN_ADDRESS_REG_2, (I2C_OAR2_ENDUAL | I2C_OAR2_ADD2), (hi2c->init.dual_address_mode | hi2c->init.own_address_2));
//
//    I2C_ENABLE_MODULE(hi2c);
//    hi2c->error_code = HAL_I2C_ERROR_NONE;
//    hi2c->state = I2C_STATE_READY;
//    hi2c->previous_state = I2C_STATE_NONE;
//    hi2c->mode = I2C_MODE_NONE;
//
//    return I2C_STATUS_OK;
//}
//
//void i2c::HAL_I2C_MspInit(handle_t* hi2c)
//{
//    GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//    if (hi2c->instance == I2C_2)
//    {
//        __HAL_RCC_GPIOB_CLK_ENABLE();       // I2C2_SCL     PB10
//        __HAL_RCC_GPIOC_CLK_ENABLE();       // I2C2_SDA     PC12
//
//        GPIO_InitStruct.Pin = GPIO_PIN_10;
//        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
//        GPIO_InitStruct.Pull = GPIO_PULLUP;
//        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//        GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
//        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//        GPIO_InitStruct.Pin = GPIO_PIN_12;
//        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
//        GPIO_InitStruct.Pull = GPIO_PULLUP;
//        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//        GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
//        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//
//        __HAL_RCC_I2C2_CLK_ENABLE();
//        HAL_NVIC_SetPriority(I2C2_EV_IRQn, 5, 0);
//        HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
//        HAL_NVIC_SetPriority(I2C2_ER_IRQn, 5, 0);
//        HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
//    }
//
//}
//
//void i2c::set_configuration()
//{
//    i2c_module_handle->instance = I2C_2;
//    i2c_module_handle->init.clock_speed = 100000U;
//    i2c_module_handle->init.fast_mode_duty_cycle = HAL_STM_I2C_DUTY_CYCLE_2;
//    i2c_module_handle->init.own_address_1 = 0;
//    i2c_module_handle->init.addressing_mode = HAL_STM_I2C_7_BIT_ADDRESS_MODE;
//    i2c_module_handle->init.dual_address_mode = HAL_STM_I2C_DUAL_ADDRESS_DISABLE;
//    i2c_module_handle->init.own_address_2 = 0;
//    i2c_module_handle->init.general_call_mode = HAL_STM_I2C_GENERAL_CALL_DISABLE;
//    i2c_module_handle->init.no_stretch_mode = HAL_STM_I2C_NO_STRETCH_DISABLE;
//    if (initialize_module(i2c_module_handle) != I2C_STATUS_OK)
//        Error_Handler();
//}
//
//i2c::status_t i2c::controller_send(uint16_t target_address, uint8_t *data_buffer_pointer, uint16_t size, uint32_t timeout)
//{
//    uint32_t tick_start = 0;
//    i2c_controller_error_message_code = ERROR_MESSAGE_CODE_NO_ERROR;
//    i2c_controller_write_procedure.state = I2C_STATE_CHECK_IF_PERIPHERAL_IS_INITIALIZED;
//
////    switch(i2c_controller_write_procedure.state)
////    {
////        case I2C_STATE_CHECK_IF_PERIPHERAL_IS_INITIALIZED:
////        {
//            if (get_i2c_module_state() != I2C_STATE_READY)
//            {
//                i2c_controller_error_message_code = PERIPHERAL_NOT_READY;
////                i2c_controller_write_procedure.state = I2C_STATE_PERIPHERAL_BUSY;
//                return I2C_STATUS_ERROR;
//            }
////            else
////                i2c_controller_write_procedure.state = I2C_STATE_CHECK_IF_BUS_IS_BUSY;
////        }
////        case I2C_STATE_CHECK_IF_BUS_IS_BUSY:
////        {
//            tick_start = HAL_GetTick();
//
//            if (wait_for_flag(I2C_FLAG_BUS_BUSY, FLAG_SET, I2C_TIMEOUT_BUSY_25_MS, tick_start) != I2C_STATUS_OK)
//            {
//                i2c_controller_error_message_code = BUS_BUSY_TIMEOUT;
////                i2c_controller_write_procedure.state = I2C_STATE_PERIPHERAL_BUSY;
//                return I2C_STATUS_ERROR;
//            }
//
////            else
////                i2c_controller_write_procedure.state = I2C_STATE_PREPARE_AND_REQUEST_TRANSFER;
////        }
////        case I2C_STATE_PREPARE_AND_REQUEST_TRANSFER:
////        {
//            lock_i2c_module();
//            if (read_control_register_bit(I2C_CR1_REG_PERIPHERAL_ENABLE_BIT) != I2C_CR1_REG_PERIPHERAL_ENABLE_BIT)
//                enable_i2c_module();
//            clear_control_register_bit(I2C_CR1_REG_POSITION_ENABLE_BIT);
//            volatile uint32_t current_transfer_options = I2C_TRANSFER_OPTIONS_DEFAULT;
//            set_transfer_state();
//            set_transfer_parameters(data_buffer_pointer, size, current_transfer_options);
//            status_t request_result = controller_request_send(current_transfer_options, target_address, timeout, tick_start);
//
//            if (request_result != I2C_STATUS_OK)
//            {
////                switch (request_result)
////                {
////                    case I2C_STATUS_TIMEOUT:
////                        i2c_controller_error_message_code = START_BIT_SET_FAILURE;
////                        break;
////                    case I2C_STATUS_ERROR:
////                        i2c_controller_error_message_code = ADDRESS_SEND_FAILURE;
////                        break;
////                    default:
////                        break;
////                }
////                i2c_controller_write_procedure.state = I2C_STATE_PERIPHERAL_ERROR;
//                return I2C_STATUS_ERROR;
//            }
////            else
////                i2c_controller_write_procedure.state = I2C_STATE_TRANSFER;
////        }
////        case I2C_STATE_TRANSFER:
////        {
//            clear_address_flag();
//            bool i2c_status_error_has_occurred = false;
//
//            while (get_transfer_size() && !i2c_status_error_has_occurred)
//            {
//                if (wait_for_flag(I2C_FLAG_TRANSMIT_BUFFER_EMPTY, FLAG_RESET, timeout, tick_start) != I2C_STATUS_OK)
//                {
//                    if (get_error_code() == I2C_ERROR_ACKNOWLEDGE_FAILED)
//                    {
//                        generate_stop_bit();
//                        i2c_controller_error_message_code = TRANSMIT_BUFFER_NOT_EMPTY_AND_ACK_FAILURE;
//                    }
//                    else
//                        i2c_controller_error_message_code = TRANSMIT_BUFFER_NOT_EMPTY;
//                    i2c_status_error_has_occurred = true;
//                    break;
//                }
//                write_next_byte_to_tx_register();
//
//                if ((check_flag(I2C_FLAG_BYTE_TRANSFER_FINISHED) == FLAG_SET) && (get_transfer_size() != 0U))
//                    write_next_byte_to_tx_register();
//
//                if (wait_for_flag(I2C_FLAG_BYTE_TRANSFER_FINISHED, FLAG_RESET, timeout, tick_start) != I2C_STATUS_OK)
//                {
//                    if (get_error_code() == I2C_ERROR_ACKNOWLEDGE_FAILED)
//                    {
//                        generate_stop_bit();
//                        i2c_controller_error_message_code = BYTE_TRANSFER_NOT_FINISHED_AND_ACK_FAILURE;
//                    }
//                    else
//                        i2c_controller_error_message_code = BYTE_TRANSFER_NOT_FINISHED;
//                    i2c_status_error_has_occurred = true;
//                    break;
//                }
//            }
//            if (i2c_status_error_has_occurred)
//            {
//                    return I2C_STATUS_ERROR;
////                i2c_controller_write_procedure.state = I2C_STATE_PERIPHERAL_ERROR;
//            }
////            else
////                i2c_controller_write_procedure.state = I2C_STATE_FINISH_TRANSFER;
////        }
////        case I2C_STATE_FINISH_TRANSFER:
////        {
//            generate_stop_bit();
//            set_i2c_module_state(I2C_STATE_READY);
//            set_device_mode(I2C_MODE_NONE);
//            unlock_i2c_module();
//            return I2C_STATUS_OK;
////        }
////        case I2C_STATE_PERIPHERAL_BUSY:
////        {
////            /*
////             * error handling for busy module
////             */
////            return I2C_STATUS_BUSY;
////        }
////        case I2C_STATE_PERIPHERAL_ERROR:
////        {
////            /*
////             * error handling for module error
////             */
////            return I2C_STATUS_ERROR;
////        }
////        default:
////            break;
////    }
//    return I2C_STATUS_OK;
//}
//
//i2c::handle_t* i2c::get_i2c_module_handle() const
//{
//    return i2c_module_handle;
//}
//
//void i2c::enable_i2c_module()
//{
//    I2C_ENABLE_MODULE(i2c_module_handle);
//}
//
//void i2c::disable_i2c_module()
//{
//    I2C_DISABLE_MODULE(i2c_module_handle);
//}
//
//uint8_t i2c::get_i2c_module_state()
//{
//    return (uint8_t) i2c_module_handle->state;
//}
//
//void i2c::set_i2c_module_state(state_t i2c_state)
//{
//    i2c_module_handle->state = i2c_state;
//}
//
//volatile uint32_t i2c::get_i2c_module_previous_state()
//{
//    return i2c_module_handle->previous_state;
//}
//
//uint16_t i2c::get_transfer_size()
//{
//    return i2c_module_handle->transfer_size;
//}
//
// volatile uint32_t i2c::get_error_code()
//{
//    return i2c_module_handle->error_code;
//}
//
//void i2c::set_error_code(volatile uint32_t error_code)
//{
//    i2c_module_handle->error_code = error_code;
//}
//
//uint32_t i2c::get_addressing_mode()
//{
//    return i2c_module_handle->init.addressing_mode;
//}
//
//void i2c::set_device_mode(mode_t i2c_device_mode)
//{
//    i2c_module_handle->mode = (volatile mode_t) i2c_device_mode;
//}
//
//void i2c::write_data_register(volatile uint32_t data_register_value)
//{
//    i2c_module_handle->instance->DATA_REG = data_register_value;
//}
//
//bool i2c::check_flag(uint32_t flag)
//{
//    return I2C_GET_FLAG(i2c_module_handle, flag);
//}
//
//void i2c::clear_flag(uint32_t flag)
//{
//    I2C_CLEAR_FLAG(i2c_module_handle, flag);
//}
//
//void i2c::clear_address_flag()
//{
//    I2C_CLEAR_ADDRESS_FLAG(i2c_module_handle);
//}
//
//i2c::status_t i2c::lock_i2c_module()
//{
//    if (i2c_module_handle->lock == HAL_MODULE_LOCKED)
//        return I2C_STATUS_BUSY;
//    i2c_module_handle->lock = HAL_MODULE_LOCKED;
//    return I2C_STATUS_OK;
//}
//
//void i2c::unlock_i2c_module()
//{
//    i2c_module_handle->lock = HAL_MODULE_UNLOCKED;
//}
//
//void i2c::set_control_register_bit(uint32_t control_register_bit)
//{
//    SET_BIT(i2c_module_handle->instance->CONTROL_REG_1, control_register_bit);
//}
//
//void i2c::clear_control_register_bit(uint32_t control_register_bit)
//{
//    CLEAR_BIT(i2c_module_handle->instance->CONTROL_REG_1, control_register_bit);
//}
//
//uint32_t i2c::read_control_register_bit(uint32_t control_register_bit)
//{
//    return READ_BIT(i2c_module_handle->instance->CONTROL_REG_1, control_register_bit);
//}
//
//void i2c::generate_start_bit()
//{
//    SET_BIT(i2c_module_handle->instance->CONTROL_REG_1, I2C_CR1_REG_GENERATION_START_BIT);
//}
//
//void i2c::generate_stop_bit()
//{
//    SET_BIT(i2c_module_handle->instance->CONTROL_REG_1, I2C_CR1_REG_GENERATION_STOP_BIT);
//}
//
//void i2c::set_error_state(uint32_t i2c_error)
//{
//    i2c_module_handle->previous_state = I2C_STATE_NONE;
//    i2c_module_handle->state         = (stm32_state_t) I2C_STATE_READY;
//    i2c_module_handle->mode          = (stm32_mode_t) I2C_MODE_NONE;
//    i2c_module_handle->error_code     |= i2c_error;
//}
//
//void i2c::set_transfer_state()
//{
//    i2c_module_handle->state = (stm32_state_t) I2C_STATE_BUSY_TX;
//    i2c_module_handle->mode = (stm32_mode_t) I2C_MODE_CONTROLLER;
//    i2c_module_handle->error_code = I2C_ERROR_NONE;
//}
//
//void i2c::set_transfer_parameters(uint8_t *data_buffer_pointer, uint16_t size, volatile uint32_t transfer_options)
//{
//    i2c_module_handle->buffer_ptr = data_buffer_pointer;
//    i2c_module_handle->transfer_counter = size;
//    i2c_module_handle->transfer_size = i2c_module_handle->transfer_counter;
//    i2c_module_handle->transfer_options = transfer_options;
//}
//
//void i2c::write_next_byte_to_tx_register()
//{
//    i2c_module_handle->instance->DATA_REG = *i2c_module_handle->buffer_ptr;
//    i2c_module_handle->buffer_ptr++;
//    i2c_module_handle->transfer_counter--;
//    i2c_module_handle->transfer_size--;
//}
//
//i2c::status_t i2c::check_for_nack()
//{
//    if (check_flag(I2C_FLAG_ACKNOWLEDGE_FAILED) == FLAG_SET)
//    {
//        clear_flag(I2C_FLAG_ACKNOWLEDGE_FAILED);
//        set_error_state(I2C_ERROR_ACKNOWLEDGE_FAILED);
//        unlock_i2c_module();
//        return I2C_STATUS_ERROR;
//    }
//    return I2C_STATUS_OK;
//}
//
//i2c::status_t i2c::wait_for_flag(uint32_t flag, flag_status_t status, uint32_t timeout, uint32_t tick_start)
//{
//    while (check_flag(flag) == status)
//    {
//        if ((flag == I2C_FLAG_TRANSMIT_BUFFER_EMPTY) && (check_for_nack() != I2C_STATUS_OK))
//            return I2C_STATUS_ERROR;
//
//        if (timeout != HAL_MAX_DELAY)
//        {
//            if (((HAL_GetTick() - tick_start) > timeout) || (timeout == 0U))
//            {
//                set_error_state(I2C_ERROR_TIMEOUT);
//                unlock_i2c_module();
//                return I2C_STATUS_ERROR;
//            }
//        }
//    }
//    return I2C_STATUS_OK;
//}
//
//i2c::status_t i2c::wait_for_controller_address_flag(uint32_t flag, uint32_t timeout, uint32_t tick_start)
//{
//    while (check_flag(flag) == FLAG_RESET)
//    {
//        if (check_flag(I2C_FLAG_ACKNOWLEDGE_FAILED) == FLAG_SET)
//        {
//            generate_stop_bit();
//            clear_flag(I2C_FLAG_ACKNOWLEDGE_FAILED);
//            set_error_state(I2C_ERROR_ACKNOWLEDGE_FAILED);
//            unlock_i2c_module();
//            return I2C_STATUS_ERROR;
//        }
//        if (timeout != HAL_MAX_DELAY)
//        {
//            if (((HAL_GetTick() - tick_start) > timeout) || (timeout == 0U))
//            {
//                set_error_state(I2C_ERROR_TIMEOUT);
//                unlock_i2c_module();
//                return I2C_STATUS_ERROR;
//            }
//        }
//    }
//    return I2C_STATUS_OK;
//}
//
//i2c::status_t i2c::controller_request_send(volatile uint32_t current_transfer_options, uint16_t target_address, uint32_t timeout, uint32_t tick_start)
//{
//    // if this is the first frame or module has just received a byte, generate a start bit
//    if (current_transfer_options == I2C_FIRST_AND_LAST_FRAME
//     || current_transfer_options == I2C_FIRST_FRAME
//     || current_transfer_options == I2C_TRANSFER_OPTIONS_DEFAULT
//     || get_i2c_module_previous_state() == I2C_STATE_CONTROLLER_RECEIVING)
//        generate_start_bit();
//    // wait until start bit is set
//    if (wait_for_flag(I2C_FLAG_START_BIT_SET, FLAG_RESET, timeout, tick_start) != I2C_STATUS_OK)
//    {
//        // confirm start bit set, throw error if not
//        if (read_control_register_bit(I2C_CR1_REG_GENERATION_START_BIT) == I2C_CR1_REG_GENERATION_START_BIT)
//            set_error_code(I2C_WRONG_START);
//        return I2C_STATUS_TIMEOUT;
//    }
//    // 7-bit mode write target address to data register
//    if (get_addressing_mode() == HAL_STM_I2C_7_BIT_ADDRESS_MODE)
//    {
//        volatile uint32_t address_write_7_bit = I2C_7BIT_ADD_WRITE(target_address);
//        write_data_register(address_write_7_bit);
//    }
//    else
//    {
//        // 10-bit mode write header to data register
//        volatile uint32_t header_write_10_bit = I2C_10BIT_HEADER_WRITE(target_address);
//        write_data_register(header_write_10_bit);
//        // 10-bit mode confirm header sent, return error otherwise
//        if (wait_for_controller_address_flag(I2C_FLAG_10_BIT_HEADER_SENT, timeout, tick_start) != I2C_STATUS_OK)
//            return I2C_STATUS_ERROR;
//        // 10-bit mode write address to data register
//        volatile uint32_t address_write_10_bit = I2C_10BIT_ADDRESS(target_address);
//        write_data_register(address_write_10_bit);
//    }
//    // wait for address sent flag, return error otherwise
//    if (wait_for_controller_address_flag(I2C_FLAG_ADDRESS_SENT, timeout, tick_start) != I2C_STATUS_OK)
//        return I2C_STATUS_ERROR;
//    return I2C_STATUS_OK;
//}
