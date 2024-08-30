/***********************************************************************************************************************
 * hal_spi.cpp
 *
 * wilson
 * 10/16/22
 * 9:41 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

/* c/c++ includes */
#include <cstdlib>
#include <cstring>
#include <memory>
/* stm32 includes */

/* third-party includes */

/* hal includes */
#include "hal_wrapper.h"
#include "hal_spi_definitions.h"
/* driver includes */

/* rtosal includes */

/* sys op includes */

/* meta structure includes */

/* hal_spi header */
#include "hal_spi.h"

void spi::chip_select_set_active(uint8_t arg_channel_id)
{
    channel_t channel;

    get_channel_by_channel_id(channel, (id_number_t)arg_channel_id);
    hal::gpio_write_pin(channel.chip_select.port, channel.chip_select.pin, (GPIO_PinState) CHIP_SELECT_SET);
}

void spi::chip_select_set_inactive(uint8_t arg_channel_id)
{
    channel_t channel;

    get_channel_by_channel_id(channel, (id_number_t)arg_channel_id);
    hal::gpio_write_pin(channel.chip_select.port, channel.chip_select.pin, (GPIO_PinState) CHIP_SELECT_RESET);
}


spi::procedure_status_t spi::initialize(module_t* arg_module, hal_spi_t* arg_instance, TIM_HandleTypeDef* arg_timeout_time_base, uint32_t arg_timeout_time_base_frequency)
{
    procedure_status_t status = PROCEDURE_STATUS_OK;

    timeout_time_base = arg_timeout_time_base;
    timeout_time_base_frequency = arg_timeout_time_base_frequency;

    module = arg_module;
    module->instance = arg_instance;
    module->rx_data_ready_flag = 0U;
    module->settings.mode = SPI_CONFIG_MODE_CONTROLLER;
    module->settings.direction = SPI_CONFIG_DIRECTION_2_LINE;
    module->settings.data_size = SPI_CONFIG_DATA_SIZE_8_BIT;
    module->settings.clock_polarity = SPI_CONFIG_CLOCK_POLARITY_LOW;
    module->settings.clock_phase = SPI_CONFIG_CLOCK_PHASE_TRAILING_EDGE;
    module->settings.chip_select_setting = SPI_CONFIG_CHIP_SELECT_SOFTWARE;
    module->settings.baud_rate_prescaler = SPI_CONFIG_BAUD_RATE_PRESCALER_16;
    module->settings.first_bit_setting = SPI_CONFIG_DATA_MSB_FIRST;
    module->settings.ti_mode = SPI_CONFIG_TI_MODE_DISABLE;
    module->settings.crc_calculation = SPI_CONFIG_CRC_CALCULATION_DISABLE;
    module->settings.crc_polynomial = 0U;

    hal::gpio_write_pin(GPIO_PORT_B, PIN_14, (GPIO_PinState) CHIP_SELECT_RESET);
    hal::gpio_write_pin(GPIO_PORT_B, PIN_15, (GPIO_PinState) CHIP_SELECT_RESET);
    hal::gpio_write_pin(GPIO_PORT_B, PIN_1,  (GPIO_PinState) CHIP_SELECT_RESET);

    if (module == nullptr)                                                              { status = PROCEDURE_STATUS_ERROR; }
    if (module->instance != SPI_1 && module->instance != SPI_2
        && module->instance != SPI_3 && module->instance != SPI_4)                      { status = PROCEDURE_STATUS_ERROR; }
    if (module->settings.mode != SPI_CONFIG_MODE_CONTROLLER
        && module->settings.mode != SPI_CONFIG_MODE_PERIPHERAL)                             { status = PROCEDURE_STATUS_ERROR; }
    if (module->settings.direction != SPI_CONFIG_DIRECTION_2_LINE
        && module->settings.direction != SPI_CONFIG_DIRECTION_2_LINE_RX_ONLY
        && module->settings.direction != SPI_CONFIG_DIRECTION_1_LINE)                       { status = PROCEDURE_STATUS_ERROR; }
    if (module->settings.data_size != SPI_CONFIG_DATA_SIZE_8_BIT
        && module->settings.data_size != SPI_CONFIG_DATA_SIZE_16_BIT)                       { status = PROCEDURE_STATUS_ERROR; }
    if (module->settings.clock_polarity != SPI_CONFIG_CLOCK_POLARITY_LOW
        && module->settings.clock_polarity != SPI_CONFIG_CLOCK_POLARITY_HIGH)               { status = PROCEDURE_STATUS_ERROR; }
    if (module->settings.clock_phase != SPI_CONFIG_CLOCK_PHASE_LEADING_EDGE
        && module->settings.clock_phase != SPI_CONFIG_CLOCK_PHASE_TRAILING_EDGE)            { status = PROCEDURE_STATUS_ERROR; }
    if (module->settings.chip_select_setting != SPI_CONFIG_CHIP_SELECT_SOFTWARE
        && module->settings.chip_select_setting != SPI_CONFIG_CHIP_SELECT_HARDWARE_INPUT
        && module->settings.chip_select_setting != SPI_CONFIG_CHIP_SELECT_HARDWARE_OUTPUT)  { status = PROCEDURE_STATUS_ERROR; }
    if (module->settings.baud_rate_prescaler != SPI_CONFIG_BAUD_RATE_PRESCALER_2
        && module->settings.baud_rate_prescaler != SPI_CONFIG_BAUD_RATE_PRESCALER_4
        && module->settings.baud_rate_prescaler != SPI_CONFIG_BAUD_RATE_PRESCALER_8
        && module->settings.baud_rate_prescaler != SPI_CONFIG_BAUD_RATE_PRESCALER_16
        && module->settings.baud_rate_prescaler != SPI_CONFIG_BAUD_RATE_PRESCALER_32
        && module->settings.baud_rate_prescaler != SPI_CONFIG_BAUD_RATE_PRESCALER_64
        && module->settings.baud_rate_prescaler != SPI_CONFIG_BAUD_RATE_PRESCALER_128
        && module->settings.baud_rate_prescaler != SPI_CONFIG_BAUD_RATE_PRESCALER_256)      { status = PROCEDURE_STATUS_ERROR; }
    if (module->settings.first_bit_setting != SPI_CONFIG_DATA_MSB_FIRST
        && module->settings.first_bit_setting != SPI_CONFIG_DATA_LSB_FIRST)                 { status = PROCEDURE_STATUS_ERROR; }
    if (module->settings.ti_mode != SPI_CONFIG_TI_MODE_DISABLE)                             { status = PROCEDURE_STATUS_ERROR; }
    if (module->settings.crc_calculation != SPI_CONFIG_CRC_CALCULATION_DISABLE)             { status = PROCEDURE_STATUS_ERROR; }

    if (module->status == MODULE_STATUS_RESET)
    {
        module->lock = MODULE_UNLOCKED;
        module->callbacks[TX_COMPLETE_CALLBACK_ID]          = nullptr;
        module->callbacks[TX_COMPLETE_CALLBACK_ID]          = nullptr;
        module->callbacks[RX_COMPLETE_CALLBACK_ID]          = nullptr;
        module->callbacks[TX_RX_COMPLETE_CALLBACK_ID]       = nullptr;
        module->callbacks[TX_HALF_COMPLETE_CALLBACK_ID]     = nullptr;
        module->callbacks[RX_HALF_COMPLETE_CALLBACK_ID]     = nullptr;
        module->callbacks[TX_RX_HALF_COMPLETE_CALLBACK_ID]  = nullptr;
        module->callbacks[ERROR_CALLBACK_ID]                = nullptr;
        module->callbacks[ABORT_CALLBACK_ID]                = nullptr;

        if (module->callbacks[MSP_INIT_CALLBACK_ID] == nullptr)
        {
            module->callbacks[MSP_INIT_CALLBACK_ID] = reinterpret_cast<void (*)(_handle_t *)>(HAL_SPI_MspInit);
        }

        module->callbacks[MSP_INIT_CALLBACK_ID](module);
    }

    module->status = MODULE_STATUS_BUSY;
    disable_module();

    set_bit_spi_register_32(CONTROL_REG_1_ID, (
        (module->settings.mode & (SPI_CR1_BIT_CONTROLLER_MODE | SPI_CR1_BIT_INTERNAL_CHIP_SELECT)) |
        (module->settings.direction & (SPI_CR1_BIT_RECEIVE_ONLY | SPI_CR1_BIT_BIDIRECTIONAL_MODE)) |
        (module->settings.data_size & SPI_CR1_BIT_DATA_FRAME_FORMAT) |
        (module->settings.clock_polarity & SPI_CR1_BIT_CLOCK_POLARITY) |
        (module->settings.clock_phase & SPI_CR1_BIT_CLOCK_PHASE) |
        (module->settings.chip_select_setting & SPI_CR1_BIT_SOFTWARE_CHIP_SELECT) |
        (module->settings.baud_rate_prescaler & SPI_CR1_BIT_BAUD_RATE) |
        (module->settings.first_bit_setting & SPI_CR1_BIT_LSB_FIRST) |
        (module->settings.crc_calculation & SPI_CR1_BIT_CRC_ENABLE)));

    set_bit_spi_register_32(CONTROL_REG_2_ID, (((module->settings.chip_select_setting >> 16U) & SPI_CR2_BIT_CHIP_SELECT_OUTPUT_ENABLE) |
                                                (module->settings.ti_mode & SPI_CR2_BIT_FRAME_FORMAT)));
    module->error_code   = SPI_ERROR_NONE;
    module->status        = MODULE_STATUS_READY;
    return PROCEDURE_STATUS_OK;
}

spi::procedure_status_t spi::spi_transmit_receive_interrupt(uint8_t *arg_tx_data_ptr, uint8_t *arg_rx_data_ptr, uint16_t arg_packet_size, hal::gpio_t* arg_chip_select_port, uint16_t arg_chip_select_pin)
{
    uint8_t spi_procedure_error = SPI_PROCEDURE_ERROR_NONE;

    if ((arg_tx_data_ptr == nullptr) || (arg_rx_data_ptr == nullptr) || (arg_packet_size == 0U))
    {
        spi_procedure_error = SPI_PROCEDURE_STATE_DATA_ERROR;
    }

    if (verify_communication_direction(SPI_CONFIG_DIRECTION_2_LINE) == PROCEDURE_STATUS_OK)
    {
        if (lock_module() == PROCEDURE_STATUS_OK)
        {
            if ((module->status != MODULE_STATUS_READY) && (module->settings.mode != SPI_CONFIG_MODE_CONTROLLER || module->settings.direction != SPI_CONFIG_DIRECTION_2_LINE || module->status != MODULE_STATUS_BUSY_RX))
            {
                spi_procedure_error = SPI_PROCEDURE_STATE_BUS_ERROR;
            }
            else if (module->status != MODULE_STATUS_BUSY_RX)
            {
                module->status = MODULE_STATUS_BUSY_TX_RX;
            }

            set_transaction_parameters(arg_tx_data_ptr, arg_rx_data_ptr, arg_packet_size);
            set_tx_and_rx_interrupt_service_routines();

            module->chip_select_port = arg_chip_select_port;
            module->chip_select_pin = arg_chip_select_pin;

            hal::gpio_write_pin(arg_chip_select_port, arg_chip_select_pin, GPIO_PIN_RESET);
            chip_select_set_active(active_packet.channel_id);
            enable_interrupts(SPI_CR2_BIT_TX_BUFFER_EMPTY_INTERRUPT_ENABLE | SPI_CR2_BIT_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE | SPI_CR2_BIT_ERROR_INTERRUPT_ENABLE);

            if ((module->instance->CONTROL_REG_1 & SPI_CR1_BIT_SPI_ENABLE) != SPI_CR1_BIT_SPI_ENABLE)
            {
                enable_module();
            }

            if (unlock_module() != PROCEDURE_STATUS_OK)
            {
                spi_procedure_error = SPI_PROCEDURE_STATE_BUS_ERROR;
            }
        }
        else
        {
            spi_procedure_error = SPI_PROCEDURE_STATE_BUS_ERROR;
        }
    }
    else
    {
        spi_procedure_error = SPI_PROCEDURE_STATE_DATA_ERROR;
    }

    if (spi_procedure_error == SPI_PROCEDURE_STATE_BUS_ERROR)  { return PROCEDURE_STATUS_BUSY;  }
    if (spi_procedure_error == SPI_PROCEDURE_STATE_DATA_ERROR) { return PROCEDURE_STATUS_ERROR; }

    return PROCEDURE_STATUS_OK;
}

spi::procedure_status_t spi::register_callback(callback_id_t arg_callback_id, spi_callback_ptr_t arg_callback_ptr) const
{
    procedure_status_t status = PROCEDURE_STATUS_OK;

    if (arg_callback_ptr == nullptr)
    {
        module->error_code |= ERROR_CALLBACK_ID;

        return PROCEDURE_STATUS_ERROR;
    }

    if (lock_module() == PROCEDURE_STATUS_OK)
    {
        if (module->status == MODULE_STATUS_READY)
        {
            if (arg_callback_id >= SPI_REGISTER_CALLBACK_MIN_ID && arg_callback_id <= SPI_REGISTER_CALLBACK_MAX_ID)
            {
                module->callbacks[arg_callback_id] = arg_callback_ptr;
            }
            else
            {
                set_error_bit(SPI_ERROR_CALLBACK_INVALID);
                status =  PROCEDURE_STATUS_ERROR;
            }
        }
        else if (module->status == MODULE_STATUS_RESET)
        {

            if (arg_callback_id == MSP_INIT_CALLBACK_ID || arg_callback_id == MSP_DEINIT_CALLBACK_ID)
            {
                module->callbacks[arg_callback_id] = arg_callback_ptr;
            }
            else
            {
                set_error_bit(SPI_ERROR_CALLBACK_INVALID);
                status =  PROCEDURE_STATUS_ERROR;
            }
        }
        else
        {
            set_error_bit(SPI_ERROR_CALLBACK_INVALID);
            status =  PROCEDURE_STATUS_ERROR;
        }
        if (unlock_module() != PROCEDURE_STATUS_OK)
        {
            status = PROCEDURE_STATUS_ERROR;
        }
    }
    else
    {
        status = PROCEDURE_STATUS_ERROR;
    }

    return status;
}

spi::procedure_status_t spi::unregister_callback(callback_id_t arg_callback_id) const
{
    procedure_status_t status = PROCEDURE_STATUS_OK;


    if (lock_module() == PROCEDURE_STATUS_OK)
    {
        if (module->status == MODULE_STATUS_READY)
        {
            if (arg_callback_id >= SPI_REGISTER_CALLBACK_MIN_ID && arg_callback_id <= SPI_REGISTER_CALLBACK_MAX_ID)
            {
                module->callbacks[arg_callback_id] = nullptr;
            }
            else
            {
                set_error_bit(SPI_ERROR_CALLBACK_INVALID);
                status =  PROCEDURE_STATUS_ERROR;
            }
        }
        else if (module->status == MODULE_STATUS_RESET)
        {

            if (arg_callback_id == MSP_INIT_CALLBACK_ID || arg_callback_id == MSP_DEINIT_CALLBACK_ID)
            {
                module->callbacks[arg_callback_id] = nullptr;
            }
            else
            {
                set_error_bit(SPI_ERROR_CALLBACK_INVALID);
                status =  PROCEDURE_STATUS_ERROR;
            }
        }
        else
        {
            set_error_bit(SPI_ERROR_CALLBACK_INVALID);
            status =  PROCEDURE_STATUS_ERROR;
        }

        if (unlock_module() != PROCEDURE_STATUS_OK)
        {
            status = PROCEDURE_STATUS_ERROR;
        }
    }
    else
    {
        status = PROCEDURE_STATUS_ERROR;
    }

    return status;
}

void tx_2_line_8_bit_isr(spi arg_object, struct spi::_handle_t *arg_module)
{
    *(volatile uint8_t *)&arg_module->instance->DATA_REG = (*arg_module->tx_buffer_ptr);
    arg_module->tx_buffer_ptr++;
    arg_module->tx_transfer_counter--;
    if (arg_module->tx_transfer_counter == 0U)
    {
        arg_object.disable_interrupts(SPI_CR2_BIT_TX_BUFFER_EMPTY_INTERRUPT_ENABLE);
        if (arg_module->rx_transfer_counter == 0U) { arg_object.close_isr(spi::TX_RX); }
    }
}

void rx_2_line_8_bit_isr(spi arg_object, struct spi::_handle_t *arg_module)
{
    *(arg_module->rx_buffer_ptr) = *((volatile uint8_t *)&arg_module->instance->DATA_REG);     // receive data in 8-bit mode
    arg_module->rx_buffer_ptr++;
    arg_module->rx_transfer_counter--;

    if (arg_module->rx_transfer_counter == 0U)
    {
        arg_object.disable_interrupts(SPI_CR2_BIT_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE | SPI_CR2_BIT_ERROR_INTERRUPT_ENABLE);
        if (arg_module->tx_transfer_counter == 0U) { arg_object.close_isr(spi::TX_RX); }
    }
}

void tx_2_line_16_bit_isr(spi arg_object, struct spi::_handle_t *arg_module)
{
    arg_module->instance->DATA_REG = *((uint16_t *)arg_module->tx_buffer_ptr);
    arg_module->tx_buffer_ptr += sizeof(uint16_t);
    arg_module->tx_transfer_counter--;

    if (arg_module->tx_transfer_counter == 0U)
    {
        arg_object.disable_interrupts(SPI_CR2_BIT_TX_BUFFER_EMPTY_INTERRUPT_ENABLE);
        if (arg_module->rx_transfer_counter == 0U) { arg_object.close_isr(spi::TX_RX); }
    }
}

void rx_2_line_16_bit_isr(spi arg_object, struct spi::_handle_t *arg_module)
{
    *((uint16_t *)arg_module->rx_buffer_ptr) = (uint16_t)(arg_module->instance->DATA_REG);
    arg_module->rx_buffer_ptr += sizeof(uint16_t);
    arg_module->rx_transfer_counter--;

    if (arg_module->rx_transfer_counter == 0U)
    {
        arg_object.disable_interrupts(SPI_CR2_BIT_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE);
        if (arg_module->tx_transfer_counter == 0U) { arg_object.close_isr(spi::TX_RX); }
    }
}

void spi_irq_handler(spi* arg_object)
{
    if ((arg_object->get_status_register_bit(SPI_SR_BIT_OVERRUN) != BIT_SET)
        && (arg_object->get_status_register_bit(SPI_SR_BIT_RX_BUFFER_NOT_EMPTY) == BIT_SET)
        && (arg_object->check_interrupt_source(SPI_CR2_BIT_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE) == BIT_SET))
    {
        arg_object->module->rx_isr_ptr(*arg_object, arg_object->module);
        return;
    }
    if ((arg_object->get_status_register_bit(SPI_SR_BIT_TX_BUFFER_EMPTY) == BIT_SET)
        && (arg_object->check_interrupt_source(SPI_CR2_BIT_TX_BUFFER_EMPTY_INTERRUPT_ENABLE) == BIT_SET))
    {
        arg_object->module->tx_isr_ptr(*arg_object, arg_object->module);
        return;
    }
    if (((arg_object->get_status_register_bit(SPI_SR_BIT_MODE_FAULT) == BIT_SET)
         || (arg_object->get_status_register_bit(SPI_SR_BIT_OVERRUN) == BIT_SET))
        && (arg_object->check_interrupt_source(SPI_CR2_BIT_ERROR_INTERRUPT_ENABLE) == BIT_SET))
    {
        if (arg_object->get_status_register_bit(SPI_SR_BIT_OVERRUN) == BIT_SET)
        {
            if (arg_object->module->status != spi::MODULE_STATUS_BUSY_TX)
            {
                arg_object->set_error_bit(SPI_ERROR_OVERRUN);
                arg_object->clear_overrun_flag();
            }
            else
            {
                arg_object->clear_overrun_flag();
                return;
            }
        }

        if (arg_object->get_status_register_bit(SPI_SR_BIT_MODE_FAULT) == BIT_SET)
        {
            arg_object->set_error_bit(SPI_ERROR_MODE_FAULT);
            arg_object->clear_mode_fault_flag();
        }

        if (arg_object->module->error_code != SPI_ERROR_NONE)
        {
            arg_object->disable_interrupts(SPI_CR2_BIT_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE | SPI_CR2_BIT_TX_BUFFER_EMPTY_INTERRUPT_ENABLE | SPI_CR2_BIT_ERROR_INTERRUPT_ENABLE);
            arg_object->module->status = spi::MODULE_STATUS_READY;
            arg_object->module->callbacks[spi::ERROR_CALLBACK_ID](arg_object->module);
        }
        return;
    }
}

void spi::set_tx_and_rx_interrupt_service_routines() const
{
    if (module->settings.data_size == SPI_CONFIG_DATA_SIZE_8_BIT)
    {
        module->rx_isr_ptr     = rx_2_line_8_bit_isr;
        module->tx_isr_ptr     = tx_2_line_8_bit_isr;
    }
    else if (module->settings.data_size == SPI_CONFIG_DATA_SIZE_16_BIT)
    {
        module->rx_isr_ptr     = rx_2_line_16_bit_isr;
        module->tx_isr_ptr     = tx_2_line_16_bit_isr;
    }
}

spi::procedure_status_t spi::verify_communication_direction(uint32_t arg_intended_direction) const
{
    procedure_status_t status = PROCEDURE_STATUS_OK;

    if (module->settings.direction != arg_intended_direction)
    {
        status = PROCEDURE_STATUS_ERROR;
    }

    return status;
}

void spi::set_transaction_parameters(uint8_t *arg_tx_data_ptr, uint8_t *arg_rx_data_ptr, uint16_t arg_packet_size) const
{
    module->error_code = SPI_ERROR_NONE;
    module->tx_buffer_ptr = (uint8_t *)arg_tx_data_ptr;
    module->rx_buffer_ptr = (uint8_t *)arg_rx_data_ptr;
    module->tx_transfer_counter = arg_packet_size;
    module->rx_transfer_counter = arg_packet_size;
}

spi::procedure_status_t spi::flag_timeout(uint32_t arg_status_reg_bit, bit_status_t arg_bit_status) const
{
    uint32_t start_time = timeout_time_base->Instance->CNT;
    uint16_t fallback_countdown = FALLBACK_COUNTDOWN;

    while (get_status_register_bit(arg_status_reg_bit) != arg_bit_status)
    {
        if (timeout_time_base->Instance->CNT - start_time >= FLAG_TIMEOUT || fallback_countdown == 0)
        {
            disable_interrupts(SPI_CR2_BIT_TX_BUFFER_EMPTY_INTERRUPT_ENABLE | SPI_CR2_BIT_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE | SPI_CR2_BIT_ERROR_INTERRUPT_ENABLE);

            if ((module->settings.mode == SPI_CONFIG_MODE_CONTROLLER) && ((module->settings.direction == SPI_CONFIG_DIRECTION_1_LINE) || (module->settings.direction == SPI_CONFIG_DIRECTION_2_LINE_RX_ONLY)))
            {
                disable_module();
            }

            module->status = MODULE_STATUS_READY;
            if (unlock_module() == PROCEDURE_STATUS_OK)
            {
                return PROCEDURE_STATUS_TIMEOUT;
            }
            else
            {
                return PROCEDURE_STATUS_ERROR;
            }
        }
        --fallback_countdown;
    }
    return PROCEDURE_STATUS_OK;
}

spi::procedure_status_t spi::wait_for_pending_flags_and_end_transaction(transaction_t arg_transaction_type)
{
    procedure_status_t status = PROCEDURE_STATUS_OK;
    uint32_t pending_flag = 0;

    if ((module->settings.mode == SPI_CONFIG_MODE_CONTROLLER)
        && ((module->settings.direction == SPI_CONFIG_DIRECTION_1_LINE)
            || (module->settings.direction == SPI_CONFIG_DIRECTION_2_LINE_RX_ONLY)))
        disable_module();

    if (module->settings.mode == SPI_CONFIG_MODE_CONTROLLER)
    {
        if (arg_transaction_type == TX_RX || module->settings.direction != SPI_CONFIG_DIRECTION_2_LINE_RX_ONLY)
        {
            pending_flag = SPI_SR_BIT_RESOURCE_BUSY;
        }
        else
        {
            pending_flag = SPI_SR_BIT_RX_BUFFER_NOT_EMPTY;
        }
    }
    else
    {
        pending_flag = SPI_SR_BIT_RX_BUFFER_NOT_EMPTY;
    }

    if (flag_timeout(pending_flag, BIT_CLEAR) != PROCEDURE_STATUS_OK)
    {
        set_error_bit(SPI_ERROR_WAITING_FOR_FLAG);
        status = PROCEDURE_STATUS_TIMEOUT;
    }

    return status;
}

void spi::close_isr(transaction_t arg_transaction_type)
{

    uint32_t active_interrupts = 0;

    switch(arg_transaction_type)
    {
        case TX_RX:
        {
            active_interrupts = SPI_CR2_BIT_ERROR_INTERRUPT_ENABLE;

            if (flag_timeout(SPI_SR_BIT_TX_BUFFER_EMPTY, BIT_SET))
            {
                set_error_bit(SPI_ERROR_WAITING_FOR_FLAG);
            }

            break;
        }
        case TX_ONLY:
        {
            active_interrupts = SPI_CR2_BIT_TX_BUFFER_EMPTY_INTERRUPT_ENABLE | SPI_CR2_BIT_ERROR_INTERRUPT_ENABLE;

            if (flag_timeout(SPI_SR_BIT_TX_BUFFER_EMPTY, BIT_SET) != PROCEDURE_STATUS_OK)
            {
                set_error_bit(SPI_ERROR_WAITING_FOR_FLAG);
            }

            break;
        }
        case RX_ONLY:
        {
            active_interrupts = SPI_CR2_BIT_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE | SPI_CR2_BIT_ERROR_INTERRUPT_ENABLE;

            break;

        }
        default:
        {
            break;
        }
    }

    disable_interrupts(active_interrupts);

    if (wait_for_pending_flags_and_end_transaction(arg_transaction_type) != PROCEDURE_STATUS_OK)
    {
        set_error_bit(SPI_ERROR_WAITING_FOR_FLAG);
    }

    if (module->settings.direction == SPI_CONFIG_DIRECTION_2_LINE)
    {
        clear_overrun_flag();
    }

    if (module->error_code != SPI_ERROR_NONE)
    {
        module->status = MODULE_STATUS_READY;
        module->callbacks[ERROR_CALLBACK_ID](module);
    }
    else
    {
        if (arg_transaction_type == RX_ONLY ||
            (arg_transaction_type == TX_RX && module->status == MODULE_STATUS_BUSY_RX))
        {
            module->status = MODULE_STATUS_READY;
            module->callbacks[RX_COMPLETE_CALLBACK_ID](module);
        }
        else if (arg_transaction_type == TX_ONLY)
        {
            module->status = MODULE_STATUS_READY;
            module->callbacks[TX_COMPLETE_CALLBACK_ID](module);
        }
        else if (arg_transaction_type == TX_RX)
        {
            module->status = MODULE_STATUS_READY;
            module->callbacks[TX_RX_COMPLETE_CALLBACK_ID](module);
        }
    }
}

void spi::transmit_and_get_result(uint8_t arg_packet_size, uint8_t* arg_tx_data)
{
    auto *rx_ptr = static_cast<uint8_t *>(malloc(arg_packet_size * sizeof(uint8_t)));
    spi_transmit_receive_interrupt(arg_tx_data, rx_ptr, arg_packet_size, active_packet.chip_select.port, active_packet.chip_select.pin);
    while (!module->rx_data_ready_flag);
    module->rx_data_ready_flag = 0U;

    for (uint8_t index = 0U; index < arg_packet_size; ++index)
    {
        rx_result[index] = *rx_ptr;
        if (index < arg_packet_size - 1U)
        {
            rx_ptr++;
        }
    }

    for (uint8_t index = 0U; index < arg_packet_size; ++index)
    {
        *rx_ptr = 0U;
        if (index < arg_packet_size - 1U)
        {
            rx_ptr--;
        }
    }
    free(rx_ptr);
}


spi::procedure_status_t spi::create_channel(id_number_t& arg_channel_id, hal::gpio_t* arg_chip_select_port, uint16_t arg_chip_select_pin)
{
    arg_channel_id = ID_INVALID;

    id_number_t new_channel_id = assign_next_available_channel_id();

    if (new_channel_id != ID_INVALID)
    {
        channel_t new_channel;
        memset(&new_channel, '\0', sizeof(channel_t));
        new_channel.channel_id = new_channel_id;

        GPIO_TypeDef* chip_select_port = nullptr;

//        switch (arg_chip_select_port)
//        {
//            case PORT_A:
//            {
//                chip_select_port = GPIOA;
//                break;
//            }
//            case PORT_B:
//            {
//                chip_select_port = GPIOB;
//                break;
//            }
//            case PORT_C:
//            {
//                chip_select_port = GPIOC;
//                break;
//            }
//            case PORT_D:
//            {
//                chip_select_port = GPIOD;
//                break;
//            }
//            case PORT_E:
//
//                chip_select_port = GPIOE;
//                break;
//
//            case PORT_F:
//            {
//                chip_select_port = GPIOF;
//                break;
//            }
//            case PORT_G:
//            {
//                chip_select_port = GPIOG;
//                break;
//            }
//            case PORT_H:
//            {
//                chip_select_port = GPIOH;
//                break;
//            }
//            default:
//            {
//                break;
//            }
//        }
        new_channel.chip_select.port = arg_chip_select_port;
        new_channel.chip_select.pin = arg_chip_select_pin;

        switch (new_channel_id)
        {
            case CHANNEL_0:
            {
                memset(&channel_list.channel_0, '\0', sizeof(channel_t));
                memcpy(&channel_list.channel_0, &new_channel, sizeof(channel_t));
                break;
            }
            case CHANNEL_1:
            {
                memset(&channel_list.channel_1, '\0', sizeof(channel_t));
                memcpy(&channel_list.channel_1, &new_channel, sizeof(channel_t));
                break;
            }
            case CHANNEL_2:
            {
                memset(&channel_list.channel_2, '\0', sizeof(channel_t));
                memcpy(&channel_list.channel_2, &new_channel, sizeof(channel_t));
                break;
            }
            case CHANNEL_3:
            {
                memset(&channel_list.channel_3, '\0', sizeof(channel_t));
                memcpy(&channel_list.channel_3, &new_channel, sizeof(channel_t));
                break;
            }
            case CHANNEL_4:
            {
                memset(&channel_list.channel_4, '\0', sizeof(channel_t));
                memcpy(&channel_list.channel_4, &new_channel, sizeof(channel_t));
                break;
            }
            case CHANNEL_5:
            {
                memset(&channel_list.channel_5, '\0', sizeof(channel_t));
                memcpy(&channel_list.channel_5, &new_channel, sizeof(channel_t));
                break;
            }
            case CHANNEL_6:
            {
                memset(&channel_list.channel_6, '\0', sizeof(channel_t));
                memcpy(&channel_list.channel_6, &new_channel, sizeof(channel_t));
                break;
            }
            case CHANNEL_7:
            {
                memset(&channel_list.channel_7, '\0', sizeof(channel_t));
                memcpy(&channel_list.channel_7, &new_channel, sizeof(channel_t));
                break;
            }
            default:
            {
                break;
            }
            chip_select_set_inactive(new_channel.channel_id);
        }
    }
    else
    {
        return PROCEDURE_STATUS_ERROR;
    }

    arg_channel_id = new_channel_id;

    return PROCEDURE_STATUS_OK;
}


id_number_t spi::assign_next_available_channel_id()
{
    id_number_t channel_id = ID_INVALID;

    if (next_available_channel_id <= SPI_USER_CHANNELS_MAX)
    {
        channel_id = next_available_channel_id;
        next_available_channel_id++;
    }

    return channel_id;
}


spi::procedure_status_t spi::create_packet_and_add_to_send_buffer(id_number_t arg_channel_id, uint8_t arg_total_byte_count, uint8_t arg_tx_byte_count, uint8_t (&arg_tx_bytes)[8], uint8_t (&arg_bytes_per_tx)[8])
{
    packet_t packet;
    channel_t channel;
    get_channel_by_channel_id(channel, arg_channel_id);

    memset(&packet, '\0', sizeof(packet_t));
    memcpy(&packet.tx_bytes, arg_tx_bytes, sizeof(packet.tx_bytes));
    memcpy(&packet.bytes_per_tx, arg_bytes_per_tx, sizeof(packet.bytes_per_tx));
    packet.channel_id = arg_channel_id;
    packet.total_byte_count = arg_total_byte_count;
    packet.tx_byte_count = arg_tx_byte_count;
    packet.chip_select.port = channel.chip_select.port;
    packet.chip_select.pin = channel.chip_select.pin;
    send_buffer_push(packet);

    return PROCEDURE_STATUS_OK;
}


void spi::get_channel_by_channel_id(channel_t& arg_channel, id_number_t arg_channel_id)
{
    memset(&arg_channel, '\0', sizeof(channel_t));

    switch (arg_channel_id)
    {
        case CHANNEL_0:
        {
            memcpy(&arg_channel, &channel_list.channel_0, sizeof(channel_t));
            break;
        }
        case CHANNEL_1:
        {
            memcpy(&arg_channel, &channel_list.channel_1, sizeof(channel_t));
            break;
        }
        case CHANNEL_2:
        {
            memcpy(&arg_channel, &channel_list.channel_2, sizeof(channel_t));
            break;
        }
        case CHANNEL_3:
        {
            memcpy(&arg_channel, &channel_list.channel_3, sizeof(channel_t));
            break;
        }
        case CHANNEL_4:
        {
            memcpy(&arg_channel, &channel_list.channel_4, sizeof(channel_t));
            break;
        }
        case CHANNEL_5:
        {
            memcpy(&arg_channel, &channel_list.channel_5, sizeof(channel_t));
            break;
        }
        case CHANNEL_6:
        {
            memcpy(&arg_channel, &channel_list.channel_6, sizeof(channel_t));
            break;
        }
        case CHANNEL_7:
        {
            memcpy(&arg_channel, &channel_list.channel_7, sizeof(channel_t));
            break;
        }
        default:
        {
            break;
        }
    }
}

void spi::send_buffer_push(packet_t& arg_packet)
{
    send_buffer.push(arg_packet);
}

void spi::send_buffer_pop()
{
    if (!send_buffer.empty())
    {
        send_buffer.pop();
    }
}

void spi::send_buffer_get_front(spi::packet_t& arg_packet)
{
    if (!send_buffer.empty())
    {
        memset(&arg_packet, '\0', sizeof(packet_t));
        memcpy(&arg_packet, &send_buffer.front(), sizeof(packet_t));
    }
}

void spi::set_active_packet_from_send_buffer()
{
    send_buffer_get_front(active_packet);
}

void spi::push_active_packet_to_return_buffer()
{
    switch(active_packet.channel_id)
    {
        case CHANNEL_0:
        {
            return_buffer_0.push(active_packet);
            break;
        }
        case CHANNEL_1:
        {
            return_buffer_1.push(active_packet);
            break;
        }
        case CHANNEL_2:
        {
            return_buffer_2.push(active_packet);
            break;
        }
        case CHANNEL_3:
        {
            return_buffer_3.push(active_packet);
            break;
        }
        case CHANNEL_4:
        {
            return_buffer_4.push(active_packet);
            break;
        }
        case CHANNEL_5:
        {
            return_buffer_5.push(active_packet);
            break;
        }
        case CHANNEL_6:
        {
            return_buffer_6.push(active_packet);
            break;
        }
        case CHANNEL_7:
        {
            return_buffer_7.push(active_packet);
            break;
        }
        default:
        {
            break;
        }
    }
}

spi::procedure_status_t spi::reset_active_packet()
{
    memset(&active_packet, '\0', sizeof(packet_t));
    active_packet.channel_id = ID_INVALID;

    return PROCEDURE_STATUS_OK;
}


uint8_t spi::process_return_buffer(packet_t& arg_packet, id_number_t arg_channel, uint8_t (&arg_rx_array)[TX_SIZE_MAX])
{
    uint8_t buffer_accessed = 0U;

    memset(&arg_packet, '\0', sizeof(packet_t));

    switch(arg_channel)
    {
        case CHANNEL_0:
        {
            if (!return_buffer_0.empty())
            {
                memcpy(&arg_packet, &return_buffer_0.front(), sizeof(packet_t));
                return_buffer_0.pop();
                buffer_accessed = 1U;
            }

            break;
        }
        case CHANNEL_1:
        {
            if (!return_buffer_1.empty())
            {
                memcpy(&arg_packet, &return_buffer_1.front(), sizeof(packet_t));
                return_buffer_1.pop();
                buffer_accessed = 1U;
            }

            break;
        }
        case CHANNEL_2:
        {
            if (!return_buffer_2.empty())
            {
                memcpy(&arg_packet, &return_buffer_2.front(), sizeof(packet_t));
                return_buffer_2.pop();
                buffer_accessed = 1U;
            }

            break;
        }
        case CHANNEL_3:
        {
            if (!return_buffer_3.empty())
            {
                memcpy(&arg_packet, &return_buffer_3.front(), sizeof(packet_t));
                return_buffer_3.pop();
                buffer_accessed = 1U;
            }

            break;
        }
        case CHANNEL_4:
        {
            if (!return_buffer_4.empty())
            {
                memcpy(&arg_packet, &return_buffer_4.front(), sizeof(packet_t));
                return_buffer_4.pop();
                buffer_accessed = 1U;
            }

            break;
        }
        case CHANNEL_5:
        {
            if (!return_buffer_5.empty())
            {
                memcpy(&arg_packet, &return_buffer_5.front(), sizeof(packet_t));
                return_buffer_5.pop();
                buffer_accessed = 1U;
            }

            break;
        }
        case CHANNEL_6:
        {
            if (!return_buffer_6.empty())
            {
                memcpy(&arg_packet, &return_buffer_6.front(), sizeof(packet_t));
                return_buffer_6.pop();
                buffer_accessed = 1U;
            }

            break;
        }
        case CHANNEL_7:
        {
            if (!return_buffer_7.empty())
            {
                memcpy(&arg_packet, &return_buffer_7.front(), sizeof(packet_t));
                return_buffer_7.pop();
                buffer_accessed = 1U;
            }

            break;
        }
        default:
        {
            break;
        }
    }

    if (buffer_accessed)
    {
        memcpy(&arg_rx_array, &arg_packet.rx_bytes, sizeof(arg_rx_array));
    }

    return buffer_accessed;
}

void spi::process_send_buffer()
{
    if (!send_buffer.empty())
    {
        set_active_packet_from_send_buffer();

        module->chip_select.port = active_packet.chip_select.port;
        module->chip_select.pin = active_packet.chip_select.pin;

        uint8_t tx_index = 0U;
        uint8_t byte_count_of_current_tx = 0;
        memset(&active_packet.rx_bytes, '\0', sizeof(active_packet.rx_bytes));

        for (uint8_t byte_count_array_index = 0U; byte_count_array_index < 8U; ++byte_count_array_index)
        {
            byte_count_of_current_tx = active_packet.bytes_per_tx[byte_count_array_index];
            if (byte_count_of_current_tx != 0U)
            {
                transmit_and_get_result(byte_count_of_current_tx, &active_packet.tx_bytes[tx_index]);
                for (uint8_t result_byte = 0U; result_byte < byte_count_of_current_tx; ++result_byte)
                {
                    active_packet.rx_bytes[tx_index++] = rx_result[result_byte];
                }
            }
        }

        send_buffer_pop();
        push_active_packet_to_return_buffer();
        reset_active_packet();
    }
}
