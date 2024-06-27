/***********************************************************************************************************************
 * Main_Controller
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
#include "hal_general.h"
#include "hal_callbacks.h"
#include "peripheral_common.h"
#include "mcu_clock_timers.h"
/* driver includes */

/* rtos abstraction includes */
#include "../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.h"
/* sys op includes */

/* meta structure includes */
#include "../meta_structure/meta_structure_system_manager.h"
/* hal_spi header */
#include "hal_spi.h"


/**************************************************** interface *******************************************************/

spi::spi()
{

}

void spi::initialize(handle_t* arg_module, hal_spi_t* arg_instance, uint8_t arg_use_crc, callback_id_t arg_complete_callback_id, spi_callback_ptr_t arg_complete_callback_ptr, callback_id_t arg_error_callback_id, spi_callback_ptr_t arg_error_callback_ptr)
{
    procedure_status_t status = PROCEDURE_STATUS_OK;

    use_crc = arg_use_crc;

    status = initialize_protocol(arg_module, arg_instance);
    if (status != PROCEDURE_STATUS_OK)
    {
        error_handler();
    }

    status = register_callback(arg_complete_callback_id, arg_complete_callback_ptr);
    if (status != PROCEDURE_STATUS_OK)
    {
        error_handler();
    }

    status = register_callback(arg_error_callback_id, arg_error_callback_ptr);
    if (status != PROCEDURE_STATUS_OK)
    {
        error_handler();
    }

    status = initialize_active_packet();
    if (status != PROCEDURE_STATUS_OK)
    {
        error_handler();
    }

    status = initialize_send_buffer();
    if (status != PROCEDURE_STATUS_OK)
    {
        error_handler();
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, (GPIO_PinState) CHIP_SELECT_RESET);
}

spi::procedure_status_t spi::initialize_protocol(handle_t* arg_module, hal_spi_t* arg_instance)
{
    procedure_status_t status = PROCEDURE_STATUS_OK;

    module = arg_module;

    module->state = SPI_STATE_RESET;
    module->instance = arg_instance;
    module->init.mode = SPI_CONFIG_MODE_CONTROLLER;
    module->init.direction = SPI_CONFIG_DIRECTION_2_LINE;
    module->init.data_size = SPI_CONFIG_DATA_SIZE_8_BIT;
    module->init.clock_polarity = SPI_CONFIG_CLOCK_POLARITY_LOW;
    module->init.clock_phase = SPI_PHASE_2EDGE;
    module->init.chip_select_setting = SPI_CONFIG_CHIP_SELECT_SOFTWARE;
    module->init.baud_rate_prescaler = SPI_CONFIG_BAUD_RATE_PRESCALER_16;
    module->init.first_bit_setting = SPI_CONFIG_DATA_MSB_FIRST;
    module->init.ti_mode = SPI_TI_MODE_DISABLE;
    module->init.crc_calculation = SPI_CRC_CALCULATION_DISABLE;
    module->init.crc_polynomial = 10U;

    if (module == nullptr)                                                      { status = PROCEDURE_STATUS_ERROR; }
    if (module->instance != SPI_1 && module->instance != SPI_2
        && module->instance != SPI_3 && module->instance != SPI_4)              { status = PROCEDURE_STATUS_ERROR; }
    if (module->init.mode != SPI_CONFIG_MODE_CONTROLLER
        && module->init.mode != SPI_CONFIG_MODE_PERIPHERAL)                            { status = PROCEDURE_STATUS_ERROR; }
    if (module->init.direction != SPI_CONFIG_DIRECTION_2_LINE
        && module->init.direction != SPI_CONFIG_DIRECTION_2_LINE_RX_ONLY
        && module->init.direction != SPI_CONFIG_DIRECTION_1_LINE)                      { status = PROCEDURE_STATUS_ERROR; }
    if (module->init.data_size != SPI_CONFIG_DATA_SIZE_8_BIT
        && module->init.data_size != SPI_CONFIG_DATA_SIZE_16_BIT)                      { status = PROCEDURE_STATUS_ERROR; }
    if (module->init.clock_polarity != SPI_CONFIG_CLOCK_POLARITY_LOW
        && module->init.clock_polarity != SPI_CONFIG_CLOCK_POLARITY_HIGH)              { status = PROCEDURE_STATUS_ERROR; }
    if (module->init.clock_phase != SPI_CONFIG_CLOCK_PHASE_LEADING_EDGE
        && module->init.clock_phase != SPI_CONFIG_CLOCK_PHASE_TRAILING_EDGE)           { status = PROCEDURE_STATUS_ERROR; }
    if (module->init.chip_select_setting != SPI_CONFIG_CHIP_SELECT_SOFTWARE
        && module->init.chip_select_setting != SPI_CONFIG_CHIP_SELECT_HARDWARE_INPUT
        && module->init.chip_select_setting != SPI_CONFIG_CHIP_SELECT_HARDWARE_OUTPUT) { status = PROCEDURE_STATUS_ERROR; }
    if (module->init.baud_rate_prescaler != SPI_CONFIG_BAUD_RATE_PRESCALER_2
        && module->init.baud_rate_prescaler != SPI_CONFIG_BAUD_RATE_PRESCALER_4
        && module->init.baud_rate_prescaler != SPI_CONFIG_BAUD_RATE_PRESCALER_8
        && module->init.baud_rate_prescaler != SPI_CONFIG_BAUD_RATE_PRESCALER_16
        && module->init.baud_rate_prescaler != SPI_CONFIG_BAUD_RATE_PRESCALER_32
        && module->init.baud_rate_prescaler != SPI_CONFIG_BAUD_RATE_PRESCALER_64
        && module->init.baud_rate_prescaler != SPI_CONFIG_BAUD_RATE_PRESCALER_128
        && module->init.baud_rate_prescaler != SPI_CONFIG_BAUD_RATE_PRESCALER_256)     { status = PROCEDURE_STATUS_ERROR; }
    if (module->init.first_bit_setting != SPI_CONFIG_DATA_MSB_FIRST
        && module->init.first_bit_setting != SPI_CONFIG_DATA_LSB_FIRST)                { status = PROCEDURE_STATUS_ERROR; }
    if (module->init.ti_mode != SPI_TI_MODE_DISABLE
        && module->init.ti_mode != SPI_CONFIG_TI_MODE_ENABLE)                          { status = PROCEDURE_STATUS_ERROR; }

    if (use_crc != 0U)
    {
        if (module->init.crc_calculation != SPI_CR1_BIT_CRC_ENABLE && module->init.crc_calculation != SPI_CRC_CALCULATION_DISABLE)
        {
            status = PROCEDURE_STATUS_ERROR;
        }

        if (module->init.crc_polynomial < SPI_CRC_POLYNOMIAL_MIN || module->init.crc_polynomial > SPI_CRC_POLYNOMIAL_MAX || (module->init.crc_polynomial & 0x01U) == 0U)
        {
            status = PROCEDURE_STATUS_ERROR;
        }
    }

    if (module->state == SPI_STATE_RESET)
    {
        module->lock = HAL_MODULE_UNLOCKED;

        module->callbacks[SPI_TX_RX_COMPLETE_CALLBACK_ID] = nullptr;
        module->callbacks[SPI_TX_COMPLETE_CALLBACK_ID] = nullptr;
        module->callbacks[SPI_RX_COMPLETE_CALLBACK_ID] = nullptr;

        module->callbacks[SPI_TX_RX_HALF_COMPLETE_CALLBACK_ID] = nullptr;
        module->callbacks[SPI_TX_HALF_COMPLETE_CALLBACK_ID] = nullptr;
        module->callbacks[SPI_RX_HALF_COMPLETE_CALLBACK_ID] = nullptr;

        module->callbacks[SPI_ERROR_CALLBACK_ID] = nullptr;
        module->callbacks[SPI_ABORT_CALLBACK_ID] = nullptr;

        module->callbacks[SPI_MSP_INIT_CALLBACK_ID] = nullptr;
        module->callbacks[SPI_MSP_DEINIT_CALLBACK_ID] = nullptr;

        module->callbacks[SPI_MSP_INIT_CALLBACK_ID] = reinterpret_cast<void (*)(_handle_t *)>(HAL_SPI_MspInit);

        module->callbacks[SPI_MSP_INIT_CALLBACK_ID](module);
    }

    module->state = SPI_STATE_BUSY;

    disable_module();

    WRITE_REG(module->instance->CONTROL_REG_1, (
        (module->init.mode & (SPI_CR1_BIT_CONTROLLER_MODE | SPI_CR1_BIT_INTERNAL_CHIP_SELECT)) |
        (module->init.direction & (SPI_CR1_BIT_RECEIVE_ONLY | SPI_CR1_BIT_BIDIRECTIONAL_MODE)) |
        (module->init.data_size & SPI_CR1_BIT_DATA_FRAME_FORMAT) |
        (module->init.clock_polarity & SPI_CR1_BIT_CLOCK_POLARITY) |
        (module->init.clock_phase & SPI_CR1_BIT_CLOCK_PHASE) |
        (module->init.chip_select_setting & SPI_CR1_BIT_SOFTWARE_CHIP_SELECT) |
        (module->init.baud_rate_prescaler & SPI_CR1_BIT_BAUD_RATE_MASK) |
        (module->init.first_bit_setting & SPI_CR1_BIT_LSB_FIRST) |
        (module->init.crc_calculation & SPI_CR1_BIT_CRC_ENABLE)));
    WRITE_REG(module->instance->CONTROL_REG_2, (((module->init.chip_select_setting >> 16U) & SPI_CR2_CHIP_SELECT_OUTPUT_ENABLE) | (module->init.ti_mode & SPI_CR2_FRAME_FORMAT)));

    if (use_crc != 0U && module->init.crc_calculation == SPI_CR1_BIT_CRC_ENABLE)
    {
        WRITE_REG(module->instance->CRC_POLYNOMIAL_REG, (module->init.crc_polynomial & SPI_CRC_POLYNOMIAL_REG));
    }

    module->error_code   = SPI_ERROR_NONE;
    module->state        = SPI_STATE_READY;

    return status;
}

spi::procedure_status_t spi::initialize_active_packet()
{
    return reset_active_packet();
}

spi::procedure_status_t spi::initialize_send_buffer()
{
    procedure_status_t status = PROCEDURE_STATUS_OK;

    while (!send_buffer.empty())
    {
        send_buffer.pop();
    }

    if (!send_buffer.empty())
    {
        status = PROCEDURE_STATUS_ERROR;
    }

    return status;
}

void spi::initialize_return_buffer()
{
//    return_buffer.reserve(SPI_BUFFER_MAX);
}

spi::procedure_status_t spi::create_channel(id_number_t& arg_channel_id, uint8_t arg_packet_size, uint8_t arg_tx_size, port_name_t arg_chip_select_port, uint16_t arg_chip_select_pin)
{
    arg_channel_id = ID_INVALID;

    id_number_t new_channel_id = assign_next_available_channel_id();

    if (new_channel_id != ID_INVALID)
    {
        channel_t new_user_channel;
        memset(&new_user_channel, '\0', sizeof(channel_t));
        new_user_channel.channel_id = new_channel_id;
        new_user_channel.packet_size = arg_packet_size;
        new_user_channel.tx_size = arg_tx_size;

        GPIO_TypeDef* chip_select_port = nullptr;

        switch (arg_chip_select_port)
        {
            case PORT_A:
            {
                chip_select_port = GPIOA;
                break;
            }
            case PORT_B:
            {
                chip_select_port = GPIOB;
                break;
            }
            case PORT_C:
            {
                chip_select_port = GPIOC;
                break;
            }
            case PORT_D:
            {
                chip_select_port = GPIOD;
                break;
            }
            case PORT_E:

                chip_select_port = GPIOE;
                break;

            case PORT_F:
            {
                chip_select_port = GPIOF;
                break;
            }
            case PORT_G:
            {
                chip_select_port = GPIOG;
                break;
            }
            case PORT_H:
            {
                chip_select_port = GPIOH;
                break;
            }
            default:
            {
                break;
            }
        }
        new_user_channel.chip_select.port = chip_select_port;
        new_user_channel.chip_select.pin = arg_chip_select_pin;

        switch (new_channel_id)
        {
            case CHANNEL_0:
            {
                memset(&user_list.channel_0, '\0', sizeof(channel_t));
                memcpy(&user_list.channel_0, &new_user_channel, sizeof(channel_t));
                break;
            }
            case CHANNEL_1:
            {
                memset(&user_list.channel_1, '\0', sizeof(channel_t));
                memcpy(&user_list.channel_1, &new_user_channel, sizeof(channel_t));
                break;
            }
            case CHANNEL_2:
            {
                memset(&user_list.channel_2, '\0', sizeof(channel_t));
                memcpy(&user_list.channel_2, &new_user_channel, sizeof(channel_t));
                break;
            }
            case CHANNEL_3:
            {
                memset(&user_list.channel_3, '\0', sizeof(channel_t));
                memcpy(&user_list.channel_3, &new_user_channel, sizeof(channel_t));
                break;
            }
            case CHANNEL_4:
            {
                memset(&user_list.channel_4, '\0', sizeof(channel_t));
                memcpy(&user_list.channel_4, &new_user_channel, sizeof(channel_t));
                break;
            }
            case CHANNEL_5:
            {
                memset(&user_list.channel_5, '\0', sizeof(channel_t));
                memcpy(&user_list.channel_5, &new_user_channel, sizeof(channel_t));
                break;
            }
            case CHANNEL_6:
            {
                memset(&user_list.channel_6, '\0', sizeof(channel_t));
                memcpy(&user_list.channel_6, &new_user_channel, sizeof(channel_t));
                break;
            }
            case CHANNEL_7:
            {
                memset(&user_list.channel_7, '\0', sizeof(channel_t));
                memcpy(&user_list.channel_7, &new_user_channel, sizeof(channel_t));
                break;
            }
            default:
            {
                break;
            }
        }
    }
    else
    {
        return PROCEDURE_STATUS_ERROR;
    }

    arg_channel_id = new_channel_id;

    return PROCEDURE_STATUS_OK;
}

void spi::get_channel_by_channel_id(channel_t& arg_channel, id_number_t arg_channel_id)
{
    memset(&arg_channel, '\0', sizeof(channel_t));

    switch (arg_channel_id)
    {
        case CHANNEL_0:
        {
            memcpy(&arg_channel, &user_list.channel_0, sizeof(channel_t));
            break;
        }
        case CHANNEL_1:
        {
            memcpy(&arg_channel, &user_list.channel_1, sizeof(channel_t));
            break;
        }
        case CHANNEL_2:
        {
            memcpy(&arg_channel, &user_list.channel_2, sizeof(channel_t));
            break;
        }
        case CHANNEL_3:
        {
            memcpy(&arg_channel, &user_list.channel_3, sizeof(channel_t));
            break;
        }
        case CHANNEL_4:
        {
            memcpy(&arg_channel, &user_list.channel_4, sizeof(channel_t));
            break;
        }
        case CHANNEL_5:
        {
            memcpy(&arg_channel, &user_list.channel_5, sizeof(channel_t));
            break;
        }
        case CHANNEL_6:
        {
            memcpy(&arg_channel, &user_list.channel_6, sizeof(channel_t));
            break;
        }
        case CHANNEL_7:
        {
            memcpy(&arg_channel, &user_list.channel_7, sizeof(channel_t));
            break;
        }
        default:
        {
            break;
        }
    }
}

spi::procedure_status_t spi::transmit(id_number_t arg_channel_id, uint8_t arg_total_byte_count, uint8_t arg_tx_size, const uint8_t* arg_tx_bytes)
{
    packet_t packet;
    channel_t channel;

    get_channel_by_channel_id(channel, arg_channel_id);

    memset(&packet, '\0', sizeof(packet_t));

    packet.channel_id = arg_channel_id;
    memcpy(&packet.tx_bytes, arg_tx_bytes, sizeof(packet.tx_bytes));

    send_buffer_push(packet);

    return PROCEDURE_STATUS_OK;
}

void spi::process_send_buffer()
{
    static uint8_t tx_sent;
    static uint8_t tx_index;
    static uint8_t last_tx_index;
    static uint8_t rx_size;
    static uint8_t transaction_size;
    static uint8_t num_transmissions;

    if (!send_buffer.empty())
    {
        switch (send_state)
        {
            case SPI_TRANSACTION_NOT_IN_PROGRESS:
            {
                set_active_packet_from_send_buffer();

                get_channel_by_channel_id(active_channel, active_packet.channel_id);

                module->chip_select.port = active_channel.chip_select.port;
                module->chip_select.pin = active_channel.chip_select.pin;

                tx_sent = false;
                tx_index = 0;
                last_tx_index = active_channel.packet_size - active_channel.tx_size + 1;
                rx_size = active_channel.tx_size;
                transaction_size = active_channel.packet_size;

                num_transmissions = calculate_number_of_transmissions_for_active_packet();

                send_state = SPI_TRANSACTION_IN_PROGRESS;

                break;
            }
            case SPI_TRANSACTION_IN_PROGRESS:
            {
                if (tx_index < last_tx_index)
                {
                    if (!tx_sent)
                    {
                        spi_transmit_receive_interrupt(module->rx_array, tx_index);
                        tx_sent = true;
                    }

                    if (tx_sent && hal_callbacks_get_spi_rx_data_ready_flag())
                    {
                        for (uint8_t rx_index = 0U; rx_index < rx_size; ++rx_index)
                        {
                            active_packet.rx_bytes[tx_index++] = module->rx_array[rx_index];
                        }

                        tx_sent = false;

                        hal_callbacks_set_spi_rx_data_ready_flag(0U);
                    }

                    if (tx_index >= transaction_size)
                    {
                        send_state = SPI_TRANSACTION_COMPLETE;
                    }
                }

                break;
            }
            case SPI_TRANSACTION_COMPLETE:
            {
                send_buffer_pop();
                push_active_packet_to_return_buffer();
                reset_active_packet();
                reset_active_channel();
                send_state = SPI_TRANSACTION_NOT_IN_PROGRESS;
                break;
            }
            default:
            {
                break;
            }
        }
    }
}

spi::procedure_status_t spi::spi_transmit_receive_interrupt(uint8_t *arg_rx_data_ptr, uint8_t arg_tx_index)
{
    procedure_status_t status = PROCEDURE_STATUS_OK;
    uint8_t error = SPI_PROCEDURE_ERROR_NONE;

    uint8_t* tx_data_ptr = &active_packet.tx_bytes[arg_tx_index];

    if ((arg_rx_data_ptr == nullptr) || (active_channel.tx_size == 0U))
    {
        error = SPI_PROCEDURE_STATE_DATA_ERROR;
    }

    assert_param(module->init.direction == SPI_CONFIG_DIRECTION_2_LINE);
    verify_communication_direction(SPI_CONFIG_DIRECTION_2_LINE);

    status = lock_module();
    if (status == PROCEDURE_STATUS_OK)
    {
        if (!((module->state == SPI_STATE_READY) || ((module->init.mode == SPI_CONFIG_MODE_CONTROLLER)
                                                     && (module->init.direction == SPI_CONFIG_DIRECTION_2_LINE) &&
                                                     (module->state == SPI_STATE_BUSY_RX))))
        {
            error = SPI_PROCEDURE_STATE_BUS_ERROR;
        }
        else if (module->state != SPI_STATE_BUSY_RX)
        {
            module->state = SPI_STATE_BUSY_TX_RX;
        }

        set_transaction_parameters(tx_data_ptr, arg_rx_data_ptr, active_channel.tx_size);
        set_rx_and_tx_interrupt_service_routines();
        reset_enabled_crc();

        assert_chip_select();

//        SPI_ENABLE_INTERRUPTS(module, (SPI_CONFIG_TX_BUFFER_EMPTY_INTERRUPT_ENABLE |
//                                       SPI_CONFIG_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE |
//                                       SPI_CONFIG_ERROR_INTERRUPT_ENABLE));
        enable_interrupts((SPI_CONFIG_TX_BUFFER_EMPTY_INTERRUPT_ENABLE
            | SPI_CONFIG_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE
            | SPI_CONFIG_ERROR_INTERRUPT_ENABLE));

        if ((module->instance->CONTROL_REG_1 & SPI_CR1_BIT_SPI_ENABLE) != SPI_CR1_BIT_SPI_ENABLE)
        {
            enable_module();
        }

        unlock_module();
    }

    if (error == SPI_PROCEDURE_STATE_BUS_ERROR)
    {
        status = PROCEDURE_STATUS_BUSY;
    }

    if (error == SPI_PROCEDURE_STATE_DATA_ERROR)
    {
        status = PROCEDURE_STATUS_ERROR;
    }

    return status;
}

uint8_t spi::process_return_buffer(id_number_t arg_channel, uint8_t (&arg_rx_array)[SPI_BYTE_COUNT_MAX] )
{
    uint8_t buffer_accessed = 0U;
    packet_t packet;

    memset(&packet, '\0', sizeof(packet_t));

    switch(arg_channel)
    {
        case CHANNEL_0:
        {
            if (!return_buffer_0.empty())
            {
                memcpy(&packet, &return_buffer_0.front(), sizeof(packet_t));
                return_buffer_0.pop();
                buffer_accessed = 1U;
            }

            break;
        }
        case CHANNEL_1:
        {
            if (!return_buffer_1.empty())
            {
                memcpy(&packet, &return_buffer_1.front(), sizeof(packet_t));
                return_buffer_1.pop();
                buffer_accessed = 1U;
            }

            break;
        }
        case CHANNEL_2:
        {
            if (!return_buffer_2.empty())
            {
                memcpy(&packet, &return_buffer_2.front(), sizeof(packet_t));
                return_buffer_2.pop();
                buffer_accessed = 1U;
            }

            break;
        }
        case CHANNEL_3:
        {
            if (!return_buffer_3.empty())
            {
                memcpy(&packet, &return_buffer_3.front(), sizeof(packet_t));
                return_buffer_3.pop();
                buffer_accessed = 1U;
            }

            break;
        }
        case CHANNEL_4:
        {
            if (!return_buffer_4.empty())
            {
                memcpy(&packet, &return_buffer_4.front(), sizeof(packet_t));
                return_buffer_4.pop();
                buffer_accessed = 1U;
            }

            break;
        }
        case CHANNEL_5:
        {
            if (!return_buffer_5.empty())
            {
                memcpy(&packet, &return_buffer_5.front(), sizeof(packet_t));
                return_buffer_5.pop();
                buffer_accessed = 1U;
            }

            break;
        }
        case CHANNEL_6:
        {
            if (!return_buffer_6.empty())
            {
                memcpy(&packet, &return_buffer_6.front(), sizeof(packet_t));
                return_buffer_6.pop();
                buffer_accessed = 1U;
            }

            break;
        }
        case CHANNEL_7:
        {
            if (!return_buffer_7.empty())
            {
                memcpy(&packet, &return_buffer_7.front(), sizeof(packet_t));
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
        memcpy(&arg_rx_array, &packet.rx_bytes, sizeof(arg_rx_array));
    }

    return buffer_accessed;
}

void spi::assert_chip_select() const
{
    HAL_GPIO_WritePin(active_channel.chip_select.port, active_channel.chip_select.pin, (GPIO_PinState) CHIP_SELECT_SET);
}
void spi::deassert_chip_select() const
{
    HAL_GPIO_WritePin(active_channel.chip_select.port, active_channel.chip_select.pin, (GPIO_PinState) CHIP_SELECT_RESET);
}

id_number_t spi::assign_next_available_channel_id()
{
    id_number_t user_channel_id = ID_INVALID;

    if (next_available_user_channel_id <= SPI_USER_CHANNELS_MAX)
    {
        user_channel_id = next_available_user_channel_id;
        next_available_user_channel_id++;
    }
    return user_channel_id;
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

void spi::reset_active_channel()
{
    memset(&active_channel, '\0', sizeof(packet_t));
    active_channel.channel_id = ID_INVALID;
}

uint8_t spi::calculate_number_of_transmissions_for_active_packet() const
{
    uint8_t num_transmissions = active_channel.packet_size / active_channel.tx_size;
    if (active_channel.packet_size % active_channel.tx_size != 0)
    {
        ++num_transmissions;
    }

    return num_transmissions;
}

spi::procedure_status_t spi::register_callback(callback_id_t arg_callback_id, spi_callback_ptr_t arg_callback_ptr) const
{
    procedure_status_t status = PROCEDURE_STATUS_OK;

    if (arg_callback_ptr == nullptr)
    {
        set_error_bit(SPI_ERROR_CALLBACK_ID);

        return PROCEDURE_STATUS_ERROR;
    }

    status = lock_module();

    if (status == PROCEDURE_STATUS_OK)
    {
        if (SPI_STATE_READY == module->state)
        {
            if (arg_callback_id >= SPI_REGISTER_CALLBACK_MIN_ID && arg_callback_id <= SPI_REGISTER_CALLBACK_MAX_ID)
            {
                module->callbacks[arg_callback_id] = arg_callback_ptr;
            }
            else
            {
                set_error_bit(SPI_ERROR_CALLBACK_ID);
                status = PROCEDURE_STATUS_ERROR;
            }
        }
        else if (SPI_STATE_RESET == module->state)
        {

            if (arg_callback_id == SPI_MSP_INIT_CALLBACK_ID || arg_callback_id == SPI_MSP_DEINIT_CALLBACK_ID)
            {
                module->callbacks[arg_callback_id] = arg_callback_ptr;
            }
            else
            {
                set_error_bit(SPI_ERROR_CALLBACK_ID);
                status = PROCEDURE_STATUS_ERROR;
            }
        }
        else
        {
            set_error_bit(SPI_ERROR_CALLBACK_ID);
            status = PROCEDURE_STATUS_ERROR;
        }

        unlock_module();
    }
    return status;
}

spi::procedure_status_t spi::unregister_callback(callback_id_t arg_callback_id) const
{
    procedure_status_t status = PROCEDURE_STATUS_OK;
    status = lock_module();

    if (status == PROCEDURE_STATUS_OK)
    {
        if (SPI_STATE_READY == module->state)
        {
            if (arg_callback_id >= SPI_REGISTER_CALLBACK_MIN_ID && arg_callback_id <= SPI_REGISTER_CALLBACK_MAX_ID)
            {
                module->callbacks[arg_callback_id] = nullptr;
            }
            else
            {
                set_error_bit(SPI_ERROR_CALLBACK_ID);
                status = PROCEDURE_STATUS_ERROR;
            }
        }
        else if (SPI_STATE_RESET == module->state)
        {

            if (arg_callback_id == SPI_MSP_INIT_CALLBACK_ID || arg_callback_id == SPI_MSP_DEINIT_CALLBACK_ID)
            {
                module->callbacks[arg_callback_id] = nullptr;
            }
            else
            {
                set_error_bit(SPI_ERROR_CALLBACK_ID);
                status = PROCEDURE_STATUS_ERROR;
            }
        }
        else
        {
            set_error_bit(SPI_ERROR_CALLBACK_ID);
            status = PROCEDURE_STATUS_ERROR;
        }

        unlock_module();
    }

    return status;
}

void spi_rx_2_line_8_bit_isr_with_crc(spi arg_object, struct spi::_handle_t *arg_module)
{
    volatile uint32_t register_contents = REGISTER_READ(arg_module->instance->DATA_REG);
    REGISTER_READ(register_contents);
    UNUSED_CAST_VOID(register_contents);

   arg_object.disable_interrupts((SPI_CONFIG_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE | SPI_CONFIG_ERROR_INTERRUPT_ENABLE));

    if (arg_module->tx_transfer_counter == 0U)
    {
        arg_object.close_tx_rx_isr();
    }
}

void spi_rx_2_line_16_bit_isr_with_crc(spi arg_object, struct spi::_handle_t *arg_module)
{
    volatile uint32_t register_contents = REGISTER_READ(arg_module->instance->DATA_REG);
    REGISTER_READ(register_contents);
    UNUSED_CAST_VOID(register_contents);
   arg_object.disable_interrupts((SPI_CONFIG_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE | SPI_CONFIG_ERROR_INTERRUPT_ENABLE));

    if (arg_module->tx_transfer_counter == 0U)
    {
        arg_object.close_tx_rx_isr();
    }
}

void spi_rx_1_line_8_bit_isr_with_crc(spi arg_object, struct spi::_handle_t *arg_module)
{
    volatile uint32_t register_contents = REGISTER_READ(arg_module->instance->DATA_REG);
    REGISTER_READ(register_contents);
    UNUSED_CAST_VOID(register_contents);

   arg_object.disable_interrupts((SPI_CONFIG_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE | SPI_CONFIG_ERROR_INTERRUPT_ENABLE));

    arg_object.close_rx_isr();
}

void spi_rx_1_line_16_bit_isr_with_crc(spi arg_object, struct spi::_handle_t *arg_module)
{
    volatile uint32_t register_contents = REGISTER_READ(arg_module->instance->DATA_REG);
    REGISTER_READ(register_contents);
    UNUSED_CAST_VOID(register_contents);

   arg_object.disable_interrupts((SPI_CONFIG_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE | SPI_CONFIG_ERROR_INTERRUPT_ENABLE));

    arg_object.close_rx_isr();
}

void spi_tx_2_line_8_bit_isr(spi arg_object, struct spi::_handle_t *arg_module)
{
    *(volatile uint8_t *)&arg_module->instance->DATA_REG = (*arg_module->tx_buffer_ptr);

    arg_module->tx_buffer_ptr++;
    arg_module->tx_transfer_counter--;

    if (arg_module->tx_transfer_counter == 0U)
    {
        if (arg_object.use_crc != 0U && arg_object.module->init.crc_calculation == SPI_CR1_BIT_CRC_ENABLE)
        {
            arg_object.set_bit_spi_register_32(spi::CONTROL_REG_1_ID, SPI_CR1_BIT_SEND_CRC_NEXT);
            arg_object.disable_interrupts(SPI_CONFIG_TX_BUFFER_EMPTY_INTERRUPT_ENABLE);
            return;
        }

        arg_object.disable_interrupts(SPI_CONFIG_TX_BUFFER_EMPTY_INTERRUPT_ENABLE);

        arg_object.close_tx_rx_isr();
    }
}

void spi_rx_2_line_8_bit_isr(spi arg_object, struct spi::_handle_t *arg_module)
{
    *arg_module->rx_buffer_ptr = *((volatile uint8_t *)&arg_module->instance->DATA_REG); // receive data in 8-bit mode

    arg_module->rx_buffer_ptr++;
    arg_module->rx_transfer_counter--;

    if (arg_module->rx_transfer_counter == 0U)
    {
        if (arg_object.use_crc != 0U && arg_module->init.crc_calculation == SPI_CR1_BIT_CRC_ENABLE)
        {
            arg_module->rx_isr_ptr =  spi_rx_2_line_8_bit_isr_with_crc;
            return;
        }

       arg_object.disable_interrupts((SPI_CONFIG_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE | SPI_CONFIG_ERROR_INTERRUPT_ENABLE));

        if (arg_module->tx_transfer_counter == 0U)
        {
            arg_object.close_tx_rx_isr();
        }
    }
}

void spi_tx_2_line_16_bit_isr(spi arg_object, struct spi::_handle_t *arg_module)
{
    arg_module->instance->DATA_REG = *((uint16_t *)arg_module->tx_buffer_ptr);

    arg_module->tx_buffer_ptr += sizeof(uint16_t);
    arg_module->tx_transfer_counter--;

    if (arg_module->tx_transfer_counter == 0U)
    {

        if (arg_object.use_crc != 0U && arg_module->init.crc_calculation == SPI_CR1_BIT_CRC_ENABLE)
        {
            arg_object.set_bit_spi_register_32(spi::CONTROL_REG_1_ID, SPI_CR1_BIT_SEND_CRC_NEXT);
            arg_object.disable_interrupts(SPI_CONFIG_TX_BUFFER_EMPTY_INTERRUPT_ENABLE);
            return;
        }

        arg_object.disable_interrupts(SPI_CONFIG_TX_BUFFER_EMPTY_INTERRUPT_ENABLE);

        if (arg_module->rx_transfer_counter == 0U)
        {
            arg_object.close_tx_rx_isr();
        }
    }
}

void spi_rx_2_line_16_bit_isr(spi arg_object, struct spi::_handle_t *arg_module)
{
    *((uint16_t *)arg_module->rx_buffer_ptr) = (uint16_t)(arg_module->instance->DATA_REG);

    arg_module->rx_buffer_ptr += sizeof(uint16_t);
    arg_module->rx_transfer_counter--;

    if (arg_module->rx_transfer_counter == 0U)
    {
        if (arg_object.use_crc != 0U && arg_module->init.crc_calculation == SPI_CR1_BIT_CRC_ENABLE)
        {
            arg_module->rx_isr_ptr =  spi_rx_2_line_16_bit_isr_with_crc;
            return;
        }

        arg_object.disable_interrupts(SPI_CONFIG_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE);

        if (arg_module->tx_transfer_counter == 0U)
        {
            arg_object.close_tx_rx_isr();
        }
    }
}

void spi::close_tx_rx_isr()
{
    volatile uint32_t count = SPI_DEFAULT_TIMEOUT_100_US * (SystemCoreClock / 24U / 1000U);

    disable_interrupts(SPI_CONFIG_ERROR_INTERRUPT_ENABLE);

    while ((module->instance->STATUS_REG & SPI_SR_BIT_TX_BUFFER_EMPTY) == FLAG_RESET && count > 0U)
    {
        count--;
    }

    if (count <= 0U)
    {
        set_error_bit(SPI_ERROR_WAITING_FOR_FLAG);
    }

    if (end_tx_rx_transaction(SPI_DEFAULT_TIMEOUT_100_US) != PROCEDURE_STATUS_OK)
    {
        set_error_bit(SPI_ERROR_WAITING_FOR_FLAG);
    }

    if (module->init.direction == SPI_CONFIG_DIRECTION_2_LINE)
    {
        clear_overrun_flag();
    }

    if (use_crc != 0U && get_status_register_bit(SPI_SR_BIT_CRC_ERROR) == BIT_SET)
    {
        module->state = SPI_STATE_READY;

        set_error_bit(SPI_ERROR_DURING_CRC_CALCULATION);
        clear_crc_error();

        module->callbacks[SPI_ERROR_CALLBACK_ID](module);
    }
    else
    {
        if (module->error_code == SPI_ERROR_NONE)
        {
            if (module->state == SPI_STATE_BUSY_RX)
            {
                module->state = SPI_STATE_READY;
                module->callbacks[SPI_RX_COMPLETE_CALLBACK_ID](module);
            }
            else
            {
                module->state = SPI_STATE_READY;
                module->callbacks[SPI_TX_RX_COMPLETE_CALLBACK_ID](module);
            }
        }
        else
        {
            module->state = SPI_STATE_READY;
            module->callbacks[SPI_ERROR_CALLBACK_ID](module);
        }
    }
}

void spi::close_tx_isr()
{
    volatile uint32_t count = SPI_DEFAULT_TIMEOUT_100_US * (SystemCoreClock / 24U / 1000U);

    while ((module->instance->STATUS_REG & SPI_SR_BIT_TX_BUFFER_EMPTY) == FLAG_RESET && count > 0U)
    {
        count--;
    }

    if (count <= 0U)
    {
        set_error_bit(SPI_ERROR_WAITING_FOR_FLAG);
    }

    disable_interrupts((SPI_CONFIG_TX_BUFFER_EMPTY_INTERRUPT_ENABLE | SPI_CONFIG_ERROR_INTERRUPT_ENABLE));

    if (end_tx_rx_transaction(SPI_DEFAULT_TIMEOUT_100_US) != PROCEDURE_STATUS_OK)
    {
        set_error_bit(SPI_ERROR_WAITING_FOR_FLAG);
    }

    if (module->init.direction == SPI_CONFIG_DIRECTION_2_LINE)
    {
        clear_overrun_flag();
    }

    module->state = SPI_STATE_READY;

    if (module->error_code != SPI_ERROR_NONE)
    {
        module->callbacks[SPI_ERROR_CALLBACK_ID](module);
    }
    else
    {
        module->callbacks[SPI_TX_COMPLETE_CALLBACK_ID](module);
    }
}

void spi::close_rx_isr()
{
    disable_interrupts((SPI_CONFIG_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE | SPI_CONFIG_ERROR_INTERRUPT_ENABLE));

    if (end_rx_transaction(SPI_DEFAULT_TIMEOUT_100_US) != PROCEDURE_STATUS_OK)
    {
        set_error_bit(SPI_ERROR_WAITING_FOR_FLAG);
    }

    if (module->init.direction == SPI_CONFIG_DIRECTION_2_LINE)
    {
        clear_overrun_flag();
    }

    module->state = SPI_STATE_READY;

    if (use_crc != 0U && get_status_register_bit(SPI_SR_BIT_CRC_ERROR) == BIT_SET)
    {
        set_error_bit(SPI_ERROR_DURING_CRC_CALCULATION);
        clear_crc_error();
        module->callbacks[SPI_ERROR_CALLBACK_ID](module);
    }
    else
    {
        if (module->error_code == SPI_ERROR_NONE)
        {
            module->callbacks[SPI_RX_COMPLETE_CALLBACK_ID](module);
        }
        else
        {
            module->callbacks[SPI_ERROR_CALLBACK_ID](module);
        }
    }
}

spi::procedure_status_t spi::end_tx_rx_transaction(uint32_t arg_timeout)
{
    volatile uint32_t count = SPI_BUSY_FLAG_WORK_AROUND_TIMEOUT_1000_US * (SystemCoreClock / 24U / 1000000U);

    if (module->init.mode == SPI_CONFIG_MODE_CONTROLLER)
    {
        if (wait_for_status_register_bit(SPI_SR_BIT_RESOURCE_BUSY, BIT_CLEAR, arg_timeout) != PROCEDURE_STATUS_OK)
        {
            set_error_bit(SPI_ERROR_WAITING_FOR_FLAG);

            return PROCEDURE_STATUS_TIMEOUT;
        }
    }
    else
    {
        while(get_status_register_bit(SPI_SR_BIT_RESOURCE_BUSY) == BIT_SET && count > 0U)
        {
            count--;
        }
    }
    return PROCEDURE_STATUS_OK;
}

spi::procedure_status_t spi::end_rx_transaction(uint32_t arg_timeout)
{
    if ((module->init.mode == SPI_MODE_MASTER) && ((module->init.direction == SPI_CONFIG_DIRECTION_1_LINE) || (module->init.direction == SPI_CONFIG_DIRECTION_2_LINE_RX_ONLY)))
    {
        disable_module();
    }
    if (module->init.mode == SPI_MODE_MASTER)
    {
        if (module->init.direction != SPI_CONFIG_DIRECTION_2_LINE_RX_ONLY)
        {
            if (wait_for_status_register_bit(SPI_SR_BIT_RESOURCE_BUSY, BIT_CLEAR, arg_timeout) != PROCEDURE_STATUS_OK)
            {
                set_error_bit(SPI_ERROR_WAITING_FOR_FLAG);

                return PROCEDURE_STATUS_TIMEOUT;
            }
        }
        else
        {
            if (wait_for_status_register_bit(SPI_SR_BIT_RX_BUFFER_NOT_EMPTY, BIT_CLEAR, arg_timeout) != PROCEDURE_STATUS_OK)
            {
                set_error_bit(SPI_ERROR_WAITING_FOR_FLAG);

                return PROCEDURE_STATUS_TIMEOUT;
            }
        }
    }
    else
    {
        if (wait_for_status_register_bit(SPI_SR_BIT_RX_BUFFER_NOT_EMPTY, BIT_CLEAR, arg_timeout) != PROCEDURE_STATUS_OK)
        {
            set_error_bit(SPI_ERROR_WAITING_FOR_FLAG);

            return PROCEDURE_STATUS_TIMEOUT;
        }
    }

    return PROCEDURE_STATUS_OK;
}

void spi::abort_tx_isr()
{
    clear_bit_spi_register_32(CONTROL_REG_2_ID, SPI_CONFIG_TX_BUFFER_EMPTY_INTERRUPT_ENABLE);
    disable_module();

    module->state = SPI_STATE_ABORT;
}

void spi::abort_rx_isr()
{
    volatile uint32_t register_contents = 0U;
    volatile uint32_t count = SPI_DEFAULT_TIMEOUT_100_US * (SystemCoreClock / 24U / 1000U);

    while ((module->instance->STATUS_REG & SPI_SR_BIT_TX_BUFFER_EMPTY) == FLAG_RESET && count > 0U)
    {
        count--;
    }

    if (count <= 0U)
    {
        set_error_bit(SPI_ERROR_DURING_ABORT);
    }

    disable_module();
    clear_bit_spi_register_32(CONTROL_REG_2_ID, (SPI_CONFIG_TX_BUFFER_EMPTY_INTERRUPT_ENABLE | SPI_CONFIG_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE | SPI_CONFIG_ERROR_INTERRUPT_ENABLE));

    register_contents = REGISTER_READ(module->instance->DATA_REG);
    UNUSED_CAST_VOID(register_contents);

    module->state = SPI_STATE_ABORT;
}

spi::procedure_status_t dma_abort_interrupt(spi* arg_object, dma_handle_t *arg_dma_handle)
{
    if(arg_dma_handle->state != DMA_STATE_BUSY)
    {
        arg_dma_handle->error_code = SPI_DMA_ERROR_NO_TRANSFER;
        return spi::PROCEDURE_STATUS_ERROR;
    }
    else
    {
        arg_dma_handle->state = DMA_STATE_ABORT;
        arg_object->disable_dma(arg_dma_handle);
    }

    return spi::PROCEDURE_STATUS_OK;
}

void dma_abort_on_error(dma_handle_t *arg_dma_handle)
{
    auto *spi_module = (spi::handle_t *)(((dma_handle_t *)arg_dma_handle)->parent);

    spi_module->rx_transfer_counter = 0U;
    spi_module->tx_transfer_counter = 0U;

    spi_module->callbacks[spi::SPI_ERROR_CALLBACK_ID](spi_module);
}

void spi::spi_irq_handler()
{
    uint32_t interrupt_source = module->instance->CONTROL_REG_2;
    uint32_t interrupt_flag   = module->instance->STATUS_REG;

    if ((get_status_register_bit(SPI_SR_BIT_OVERRUN) == BIT_CLEAR)
        && (get_status_register_bit(SPI_SR_BIT_RX_BUFFER_NOT_EMPTY) == BIT_SET)
        && (check_interrupt_source(SPI_CONFIG_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE) == BIT_SET))
    {
        module->rx_isr_ptr(*this, module);
        return;
    }

    if ((get_status_register_bit(SPI_SR_BIT_TX_BUFFER_EMPTY) == BIT_SET)
        && check_interrupt_source(SPI_CONFIG_TX_BUFFER_EMPTY_INTERRUPT_ENABLE) == BIT_SET)
    {
        module->tx_isr_ptr(*this, module);
        return;
    }

    if (((get_status_register_bit(SPI_SR_BIT_MODE_FAULT) == BIT_SET)
         || (get_status_register_bit(SPI_SR_BIT_OVERRUN) == BIT_SET)
         || (get_status_register_bit(SPI_SR_BIT_TI_MODE_FRAME_FORMAT_ERROR) == BIT_SET))
         && (check_interrupt_source(SPI_CONFIG_ERROR_INTERRUPT_ENABLE) == BIT_SET))
    {
        if (get_status_register_bit(SPI_SR_BIT_OVERRUN) == BIT_SET)
        {
            if (module->state != spi::SPI_STATE_BUSY_TX)
            {
                set_error_bit(SPI_ERROR_OVERRUN);
                clear_overrun_flag();
            }
            else
            {
                clear_overrun_flag();
                return;
            }
        }

        if (get_status_register_bit(SPI_SR_BIT_MODE_FAULT) == BIT_SET)
        {
            set_error_bit(SPI_ERROR_MODE_FAULT);
            clear_mode_fault_flag();
        }

        if (get_status_register_bit(SPI_SR_BIT_TI_MODE_FRAME_FORMAT_ERROR) == BIT_SET)
        {
            set_error_bit(SPI_ERROR_TI_MODE_FRAME_FORMAT);
            clear_ti_frame_format_error_flag();
        }

        if (module->error_code != SPI_ERROR_NONE)
        {
            disable_interrupts((SPI_CONFIG_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE | SPI_CONFIG_TX_BUFFER_EMPTY_INTERRUPT_ENABLE | SPI_CONFIG_ERROR_INTERRUPT_ENABLE));

            module->state = spi::SPI_STATE_READY;

            if ((REGISTER_CHECK_SET_BIT(interrupt_source, SPI_CR2_TX_BUFFER_DMA_ENABLE)) || (REGISTER_CHECK_SET_BIT(interrupt_source, SPI_CR2_RX_BUFFER_DMA_ENABLE)))
            {
                clear_bit_spi_register_32(spi::CONTROL_REG_2_ID, (SPI_CR2_TX_BUFFER_DMA_ENABLE | SPI_CR2_RX_BUFFER_DMA_ENABLE));

                if (module->rx_dma_handle != nullptr)
                {
                    module->rx_dma_handle->transfer_abort_callback = dma_abort_on_error;

                    if (dma_abort_interrupt(this, module->rx_dma_handle) != spi::PROCEDURE_STATUS_OK)
                    {
                        set_error_bit(SPI_ERROR_DURING_ABORT);
                    }
                }
                if (module->tx_dma_handle != nullptr)
                {
                    module->tx_dma_handle->transfer_abort_callback = dma_abort_on_error;

                    if (dma_abort_interrupt(this, module->tx_dma_handle) != spi::PROCEDURE_STATUS_OK)
                    {
                        set_error_bit(SPI_ERROR_DURING_ABORT);
                    }
                }
            }
            else
            {
                module->callbacks[spi::SPI_ERROR_CALLBACK_ID](module);
            }
        }
        return;
    }
}

spi::procedure_status_t spi::lock_module() const
{
    procedure_status_t status = PROCEDURE_STATUS_OK;

    if (module->lock == HAL_MODULE_LOCKED)
    {
        status = PROCEDURE_STATUS_BUSY;
    }
    else
    {
        module->lock = HAL_MODULE_LOCKED;
    }

    return status;
}

void spi::unlock_module() const
{
    module->lock = HAL_MODULE_UNLOCKED;
}

void spi::set_transaction_parameters(uint8_t *arg_tx_data_ptr, uint8_t *arg_rx_data_ptr, uint16_t arg_packet_size) const
{
    module->error_code = SPI_ERROR_NONE;

    module->tx_buffer_ptr = (uint8_t *)arg_tx_data_ptr;
    module->tx_transfer_size = arg_packet_size;
    module->tx_transfer_counter = arg_packet_size;
    module->rx_buffer_ptr = (uint8_t *)arg_rx_data_ptr;
    module->rx_transfer_size = arg_packet_size;
    module->rx_transfer_counter = arg_packet_size;
}

void spi::set_rx_and_tx_interrupt_service_routines() const
{
    if (module->init.data_size == SPI_CONFIG_DATA_SIZE_8_BIT)
    {
        module->rx_isr_ptr     = spi_rx_2_line_8_bit_isr;
        module->tx_isr_ptr     = spi_tx_2_line_8_bit_isr;
    }
    else
    {
        module->rx_isr_ptr     = spi_rx_2_line_16_bit_isr;
        module->tx_isr_ptr     = spi_tx_2_line_16_bit_isr;
    }
}

spi::comms_state_t spi::get_module_communication_state() const
{
    return module->state;
}

uint32_t spi::get_module_operating_mode() const
{
    return (uint32_t) module->init.mode;
}

void spi::verify_communication_direction(uint32_t arg_intended_direction) const
{
    uint32_t set_direction = module->init.direction;

    switch (arg_intended_direction)
    {
        case SPI_CONFIG_DIRECTION_2_LINE:
        {
            assert_param(set_direction == SPI_CONFIG_DIRECTION_2_LINE);
            break;
        }
        case SPI_CONFIG_DIRECTION_2_LINE_RX_ONLY:
        {
            assert_param(set_direction == SPI_CONFIG_DIRECTION_2_LINE_RX_ONLY);
            break;
        }
        case SPI_CONFIG_DIRECTION_1_LINE:
        {
            assert_param(set_direction == SPI_CONFIG_DIRECTION_1_LINE);
            break;
        }
        default:
        {
            break;
        }
    }
}

spi::procedure_status_t spi::wait_for_status_register_bit(uint32_t arg_bit, bit_status_t arg_bit_status, uint32_t arg_timeout) const
{
    uint32_t start_time_ = HAL_GetTick();

    while (get_status_register_bit(arg_bit) != arg_bit_status)
    {
        if (arg_timeout != HAL_MAX_DELAY)
        {
            if ((HAL_GetTick() - start_time_) >= arg_timeout)
            {
                disable_interrupts((SPI_CONFIG_TX_BUFFER_EMPTY_INTERRUPT_ENABLE | SPI_CONFIG_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE | SPI_CONFIG_ERROR_INTERRUPT_ENABLE));

                if ((module->init.mode == SPI_CONFIG_MODE_CONTROLLER) && ((module->init.direction == SPI_CONFIG_DIRECTION_1_LINE) || (module->init.direction == SPI_CONFIG_DIRECTION_2_LINE_RX_ONLY)))
                {
                    disable_module();
                }

                if (module->init.crc_calculation == SPI_CR1_BIT_CRC_ENABLE)
                {
                    reset_enabled_crc();
                }

                module->state = SPI_STATE_READY;
                unlock_module();

                return PROCEDURE_STATUS_TIMEOUT;
            }
        }
    }

    return PROCEDURE_STATUS_OK;
}

void spi::reset_enabled_crc() const
{
    if (module->init.crc_calculation == SPI_CR1_BIT_CRC_ENABLE)
    {
        clear_bit_spi_register_32(CONTROL_REG_1_ID, SPI_CR1_BIT_CRC_ENABLE);
        set_bit_spi_register_32(CONTROL_REG_1_ID, SPI_CR1_BIT_CRC_ENABLE);
    }
}
