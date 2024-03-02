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
/* header */
#include "hal_spi.h"


/**************************************************** interface *******************************************************/

spi::spi()
{

}

void spi::initialize(handle_t* spi_handle, callback_id_t complete_callback_id, spi_callback_ptr_t complete_callback_ptr, callback_id_t error_callback_id, spi_callback_ptr_t error_callback_ptr)
{
    const std::string spi_name = "SPI 2";
//    register_new_resource_to_resource_manifest(RESOURCE_TYPE_SPI, spi_name);
    configure_protocol(spi_handle);
    spi_register_callback(complete_callback_id, complete_callback_ptr);
    spi_register_callback(error_callback_id, error_callback_ptr);
    initialize_active_packet();
    initialize_send_buffer();
    if (initialize_protocol() != SPI_STATUS_OK) { Error_Handler(); }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, (GPIO_PinState) CHIP_SELECT_RESET);
}

spi::status_t spi::create_channel(id_number_t& _channel_id, uint8_t _packet_size, uint8_t _tx_size, port_name_t _chip_select_port, uint16_t _chip_select_pin)
{
    _channel_id = ID_INVALID;
    id_number_t new_channel_id = assign_next_available_channel_id();

    if (new_channel_id != ID_INVALID)
    {
        channel_t new_user_channel;
        memset(&new_user_channel, '\0', sizeof(channel_t));
        new_user_channel.channel_id = new_channel_id;
        new_user_channel.packet_size = _packet_size;
        new_user_channel.tx_size = _tx_size;
        GPIO_TypeDef* chip_select_port;
        switch (_chip_select_port)
        {
            case PORT_A:
                chip_select_port = GPIOA;
                break;
            case PORT_B:
                chip_select_port = GPIOB;
                break;
            case PORT_C:
                chip_select_port = GPIOC;
                break;
            case PORT_D:
                chip_select_port = GPIOD;
                break;
            case PORT_E:
                chip_select_port = GPIOE;
                break;
            case PORT_F:
                chip_select_port = GPIOF;
                break;
            case PORT_G:
                chip_select_port = GPIOG;
                break;
            case PORT_H:
                chip_select_port = GPIOH;
                break;
            default:
                break;
        }
        new_user_channel.chip_select.port = chip_select_port;
        new_user_channel.chip_select.pin = _chip_select_pin;

        switch (new_channel_id)
        {
            case CHANNEL_0:
                memset(&user_list.channel_0, '\0', sizeof(channel_t));
                memcpy(&user_list.channel_0, &new_user_channel, sizeof(channel_t));
                break;
            case CHANNEL_1:
                memset(&user_list.channel_1, '\0', sizeof(channel_t));
                memcpy(&user_list.channel_1, &new_user_channel, sizeof(channel_t));
                break;
            case CHANNEL_2:
                memset(&user_list.channel_2, '\0', sizeof(channel_t));
                memcpy(&user_list.channel_2, &new_user_channel, sizeof(channel_t));
                break;
            case CHANNEL_3:
                memset(&user_list.channel_3, '\0', sizeof(channel_t));
                memcpy(&user_list.channel_3, &new_user_channel, sizeof(channel_t));
                break;
            case CHANNEL_4:
                memset(&user_list.channel_4, '\0', sizeof(channel_t));
                memcpy(&user_list.channel_4, &new_user_channel, sizeof(channel_t));
                break;
            case CHANNEL_5:
                memset(&user_list.channel_5, '\0', sizeof(channel_t));
                memcpy(&user_list.channel_5, &new_user_channel, sizeof(channel_t));
                break;
            case CHANNEL_6:
                memset(&user_list.channel_6, '\0', sizeof(channel_t));
                memcpy(&user_list.channel_6, &new_user_channel, sizeof(channel_t));
                break;
            case CHANNEL_7:
                memset(&user_list.channel_7, '\0', sizeof(channel_t));
                memcpy(&user_list.channel_7, &new_user_channel, sizeof(channel_t));
                break;
            default:
                break;
        }
    }
    else
    {
        return SPI_STATUS_ERROR;
    }
    _channel_id = new_channel_id;
    return SPI_STATUS_OK;
}

spi::status_t spi::transmit(id_number_t _channel_id, uint8_t _total_byte_count, uint8_t _tx_size, const uint8_t* _tx_bytes)
{
    packet_t packet;
    channel_t channel;
    get_channel_by_channel_id(channel, _channel_id);
    memset(&packet, '\0', sizeof(packet_t));
    packet.channel_id = _channel_id;
    memcpy(&packet.tx_bytes, _tx_bytes, sizeof(packet.tx_bytes));
    send_buffer_push(packet);
    return SPI_STATUS_OK;
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
                        for (uint8_t rx_index = 0; rx_index < rx_size; ++rx_index)
                        {
                            active_packet.rx_bytes[tx_index++] = module->rx_array[rx_index];
                        }
                        tx_sent = false;
                        hal_callbacks_set_spi_rx_data_ready_flag(0);
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
                break;
        }
    }
}

uint8_t spi::process_return_buffer(id_number_t _channel, uint8_t (&_rx_array)[SPI_BYTE_COUNT_MAX] )
{
    uint8_t buffer_accessed = 0;
    packet_t p;
    memset(&p, '\0', sizeof(packet_t));
    switch(_channel)
    {
        case CHANNEL_0:
            if (!return_buffer_0.empty())
            {
                memcpy(&p, &return_buffer_0.front(), sizeof(packet_t));
                return_buffer_0.pop();
                buffer_accessed = 1;
            }
            break;
        case CHANNEL_1:
            if (!return_buffer_1.empty())
            {
                memcpy(&p, &return_buffer_1.front(), sizeof(packet_t));
                return_buffer_1.pop();
                buffer_accessed = 1;
            }
            break;
        case CHANNEL_2:
            if (!return_buffer_2.empty())
            {
                memcpy(&p, &return_buffer_2.front(), sizeof(packet_t));
                return_buffer_2.pop();
                buffer_accessed = 1;
            }
            break;
        case CHANNEL_3:
            if (!return_buffer_3.empty())
            {
                memcpy(&p, &return_buffer_3.front(), sizeof(packet_t));
                return_buffer_3.pop();
                buffer_accessed = 1;
            }
            break;
        case CHANNEL_4:
            if (!return_buffer_4.empty())
            {
                memcpy(&p, &return_buffer_4.front(), sizeof(packet_t));
                return_buffer_4.pop();
                buffer_accessed = 1;
            }
            break;
        case CHANNEL_5:
            if (!return_buffer_5.empty())
            {
                memcpy(&p, &return_buffer_5.front(), sizeof(packet_t));
                return_buffer_5.pop();
                buffer_accessed = 1;
            }
            break;
        case CHANNEL_6:
            if (!return_buffer_6.empty())
            {
                memcpy(&p, &return_buffer_6.front(), sizeof(packet_t));
                return_buffer_6.pop();
                buffer_accessed = 1;
            }
            break;
        case CHANNEL_7:
            if (!return_buffer_7.empty())
            {
                memcpy(&p, &return_buffer_7.front(), sizeof(packet_t));
                return_buffer_7.pop();
                buffer_accessed = 1;
                break;
            }
        default:
            break;
    }

    if (buffer_accessed)
    {
        memcpy(&_rx_array, &p.rx_bytes, sizeof(_rx_array));
    }
    return buffer_accessed;
}

/**********************************************************************************************************************/

void spi::configure_protocol(handle_t* spi_handle)
{
    module = spi_handle;
    module->instance = SPI_2;
    module->init.mode = SPI_MODE_MASTER;
    module->init.direction = SPI_DIRECTION_2_LINE;
    module->init.data_size = SPI_DATA_SIZE_8_BIT;
    module->init.clock_polarity = SPI_POLARITY_LOW;
    module->init.clock_phase = SPI_PHASE_2EDGE;
    module->init.chip_select_setting = SPI_CHIP_SELECT_SOFTWARE;
    module->init.baud_rate_prescaler = SPI_BAUD_RATE_PRESCALER_16;
    module->init.first_bit_setting = SPI_DATA_MSB_FIRST;
    module->init.ti_mode = SPI_TI_MODE_DISABLE;
    module->init.crc_calculation = SPI_CRC_CALCULATION_DISABLE;
    module->init.crc_polynomial = 10;
}

void spi::initialize_active_packet()
{
    reset_active_packet();
}

spi::status_t spi::initialize_send_buffer()
{
    while (!send_buffer.empty())
    {
        send_buffer.pop();
    }
    return SPI_STATUS_OK;
}

void spi::initialize_return_buffer()
{
//    return_buffer.reserve(SPI_BUFFER_MAX);
}

spi::status_t spi::initialize_protocol() const
{
    if (module == nullptr)
        return SPI_STATUS_ERROR;
    assert_param(SPI_VERIFY_VALID_INSTANCE(module->instance));
    assert_param(SPI_VERIFY_MODE(module->init.mode));
    assert_param(SPI_VERIFY_DIRECTION(module->init.direction));
    assert_param(SPI_VERIFY_DATA_SIZE(module->init.data_size));
    assert_param(SPI_VERIFY_CHIP_SELECT_MODE(module->init.chip_select_setting));
    assert_param(SPI_VERIFY_BAUD_RATE_PRESCALER(module->init.baud_rate_prescaler));
    assert_param(SPI_VERIFY_FIRST_BIT_SETTING(module->init.first_bit_setting));
    assert_param(SPI_VERIFY_TI_MODE(module->init.ti_mode));
    if (module->init.ti_mode == SPI_TI_MODE_DISABLE)
    {
        assert_param(SPI_VERIFY_CLOCK_POLARITY(module->init.clock_polarity));
        assert_param(SPI_VERIFY_CLOCK_PHASE(module->init.clock_phase));
        if (module->init.mode == SPI_MODE_MASTER)
            assert_param(SPI_VERIFY_BAUD_RATE_PRESCALER(module->init.baud_rate_prescaler));
        else
            module->init.baud_rate_prescaler = SPI_BAUD_RATE_PRESCALER_2;
    }
    else
    {
        assert_param(SPI_VERIFY_BAUD_RATE_PRESCALER(module->init.baud_rate_prescaler));
        module->init.clock_polarity = SPI_CLOCK_POLARITY_LOW;
        module->init.clock_phase    = SPI_CLOCK_PHASE_LEADING_EDGE;
    }
#if (SPI_USE_CRC != 0U)
    assert_param(IS_SPI_CRC_CALCULATION(spi_handle->init.crc_calculation));
        if (spi_handle->init.crc_calculation == SPI_CRCCALCULATION_ENABLE)
            assert_param(IS_SPI_CRC_POLYNOMIAL(spi_handle->init.crc_polynomial));
#else
    module->init.crc_calculation = SPI_CRC_CALCULATION_DISABLE;
#endif

    if (module->state == SPI_STATE_RESET)
    {
        module->lock = HAL_MODULE_UNLOCKED;
#if (SPI_USE_REGISTER_CALLBACKS == 1U)
        module->TxCpltCallback       = HAL_SPI_TxCpltCallback;
        module->RxCpltCallback       = HAL_SPI_RxCpltCallback;
        module->TxRxCpltCallback     = HAL_SPI_TxRxCplt_Callback;
        module->TxHalfCpltCallback   = HAL_SPI_TxHalfCpltCallback;
        module->RxHalfCpltCallback   = HAL_SPI_RxHalfCpltCallback;
        module->TxRxHalfCpltCallback = HAL_SPI_TxRxHalfCpltCallback;
        module->ErrorCallback        = HAL_SPI_Error_Callback;
        module->AbortCpltCallback    = HAL_SPI_AbortCpltCallback;

        if (module->MspInitCallback == nullptr)
            module->MspInitCallback = reinterpret_cast<void (*)(_handle_t *)>(HAL_SPI_MspInit);
        module->MspInitCallback(module);
#else
        HAL_SPI_MspInit(reinterpret_cast<SPI_HandleTypeDef *>(spi_module_handle));
#endif
    }
    module->state = SPI_STATE_BUSY;
    SPI_DISABLE_MODULE(module);

    WRITE_REG(module->instance->CONTROL_REG_1, (
            (module->init.mode & (SPI_CR1_MODE_CONTROLLER | SPI_CR1_INTERNAL_CHIP_SELECT)) |
            (module->init.direction & (SPI_CR1_RECEIVE_ONLY | SPI_CR1_BIDIRECTIONAL_MODE)) |
            (module->init.data_size & SPI_CR1_DATA_FRAME_FORMAT) |
            (module->init.clock_polarity & SPI_CR1_CLOCK_POLARITY) |
            (module->init.clock_phase & SPI_CR1_CLOCK_PHASE) |
            (module->init.chip_select_setting & SPI_CR1_SOFTWARE_CHIP_SELECT) |
            (module->init.baud_rate_prescaler & SPI_CR1_BAUD_RATE_CONTROL_MASK) |
            (module->init.first_bit_setting & SPI_CR1_LSB_FIRST) |
            (module->init.crc_calculation & SPI_CR1_CRC_ENABLE)));
    WRITE_REG(module->instance->CONTROL_REG_2, ((
                                                        (module->init.chip_select_setting >> 16U) & SPI_CR2_CHIP_SELECT_OUTPUT_ENABLE) |
                                                (module->init.ti_mode & SPI_CR2_FRAME_FORMAT)));

    #if (SPI_USE_CRC != 0U)
        if (spi_handle->init.crc_calculation == SPI_CRCCALCULATION_ENABLE)
            WRITE_REG(spi_handle->instance->CRC_POLYNOMIAL_REG, (spi_handle->init.crc_polynomial & SPI_CRCPR_CRCPOLY_Msk));
    #endif

    #if defined(SPI_I2S_MODE_SELECT)
        STM_HAL_CLEAR_BIT(spi_module_handle->instance->I2S_CONFIG_REG, SPI_I2S_MODE_SELECT);
    #endif

    module->error_code   = SPI_ERROR_NONE;
    module->state        = SPI_STATE_READY;
    return SPI_STATUS_OK;
}

spi::status_t spi::spi_transmit_receive_interrupt(uint8_t *rx_data_pointer, uint8_t _tx_index)
{
    status_t transmit_status = SPI_STATUS_OK;
    uint8_t error = SPI_PROCEDURE_ERROR_NONE;
    uint8_t* tx_data_pointer = &active_packet.tx_bytes[_tx_index];

    if ((rx_data_pointer == nullptr) || (active_channel.tx_size == 0U))
        error = SPI_PROCEDURE_STATE_DATA_ERROR;

    assert_param(SPI_VERIFY_DIRECTION_2_LINE(module->init.direction));
    verify_communication_direction(SPI_DIRECTION_2_LINE);
    lock_module();

    if (!((module->state == SPI_STATE_READY) || ((module->init.mode == SPI_MODE_CONTROLLER) && (module->init.direction == SPI_DIRECTION_2_LINE) && (module->state == SPI_STATE_BUSY_RX))))
        error = SPI_PROCEDURE_STATE_BUS_ERROR;
    else if (module->state != SPI_STATE_BUSY_RX)
        module->state = SPI_STATE_BUSY_TX_RX;
    set_transaction_parameters(tx_data_pointer, rx_data_pointer, active_channel.tx_size);
    set_rx_and_tx_interrupt_service_routines();
    reset_enabled_crc();
    assert_chip_select();
    SPI_ENABLE_INTERRUPTS(module, (SPI_TX_BUFFER_EMPTY_INTERRUPT_ENABLE | SPI_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE | SPI_ERROR_INTERRUPT_ENABLE));
    if ((module->instance->CONTROL_REG_1 & SPI_CR1_SPE) != SPI_CR1_SPE) { SPI_ENABLE_MODULE(module); }
    unlock_module();
    if (error == SPI_PROCEDURE_STATE_BUS_ERROR)  { transmit_status = SPI_STATUS_BUSY;  }
    if (error == SPI_PROCEDURE_STATE_DATA_ERROR) { transmit_status = SPI_STATUS_ERROR; }
    return transmit_status;
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

void spi::get_channel_by_channel_id(channel_t& channel, id_number_t channel_id)
{
    memset(&channel, '\0', sizeof(channel_t));
    switch (channel_id)
    {
        case CHANNEL_0:
            memcpy(&channel, &user_list.channel_0, sizeof(channel_t));
            break;
        case CHANNEL_1:
            memcpy(&channel, &user_list.channel_1, sizeof(channel_t));
            break;
        case CHANNEL_2:
            memcpy(&channel, &user_list.channel_2, sizeof(channel_t));
            break;
        case CHANNEL_3:
            memcpy(&channel, &user_list.channel_3, sizeof(channel_t));
            break;
        case CHANNEL_4:
            memcpy(&channel, &user_list.channel_4, sizeof(channel_t));
            break;
        case CHANNEL_5:
            memcpy(&channel, &user_list.channel_5, sizeof(channel_t));
            break;
        case CHANNEL_6:
            memcpy(&channel, &user_list.channel_6, sizeof(channel_t));
            break;
        case CHANNEL_7:
            memcpy(&channel, &user_list.channel_7, sizeof(channel_t));
            break;
        default:
            break;
    }
}

void spi::send_buffer_push(packet_t& packet)
{
    send_buffer.push(packet);
}

void spi::send_buffer_pop()
{
    if (!send_buffer.empty())
    {
        send_buffer.pop();
    }
}

void spi::send_buffer_get_front(spi::packet_t& packet)
{
    if (!send_buffer.empty())
    {
        memset(&packet, '\0', sizeof(packet_t));
        memcpy(&packet, &send_buffer.front(), sizeof(packet_t));
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
            return_buffer_0.push(active_packet);
            break;
        case CHANNEL_1:
            return_buffer_1.push(active_packet);
            break;
        case CHANNEL_2:
            return_buffer_2.push(active_packet);
            break;
        case CHANNEL_3:
            return_buffer_3.push(active_packet);
            break;
        case CHANNEL_4:
            return_buffer_4.push(active_packet);
            break;
        case CHANNEL_5:
            return_buffer_5.push(active_packet);
            break;
        case CHANNEL_6:
            return_buffer_6.push(active_packet);
            break;
        case CHANNEL_7:
            return_buffer_7.push(active_packet);
            break;
        default:
            break;
    }
}

void spi::reset_active_packet()
{
    memset(&active_packet, '\0', sizeof(packet_t));
    active_packet.channel_id = ID_INVALID;

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
        ++num_transmissions;
    return num_transmissions;
}

#if (SPI_USE_REGISTER_CALLBACKS == 1U)

    spi::status_t spi::spi_register_callback(callback_id_t callback_id, spi_callback_ptr_t callback_ptr) const
    {
        status_t status = SPI_STATUS_OK;
        if (callback_ptr == nullptr)
        {
            module->error_code |= SPI_ERROR_CALLBACK_ID;
            return SPI_STATUS_ERROR;
        }
        SPI_LOCK_MODULE(module);

        if (SPI_STATE_READY == module->state)
        {
            switch (callback_id)
            {
                case SPI_TX_COMPLETE_CALLBACK_ID:
                    module->TxCpltCallback = callback_ptr;
                    break;
                case SPI_RX_COMPLETE_CALLBACK_ID:
                    module->RxCpltCallback = callback_ptr;
                    break;
                case SPI_TX_RX_COMPLETE_CALLBACK_ID:
                    module->TxRxCpltCallback = callback_ptr;
                    break;
                case SPI_TX_HALF_COMPLETE_CALLBACK_ID:
                    module->TxHalfCpltCallback = callback_ptr;
                    break;
                case SPI_RX_HALF_COMPLETE_CALLBACK_ID:
                    module->RxHalfCpltCallback = callback_ptr;
                    break;
                case SPI_TX_RX_HALF_COMPLETE_CALLBACK_ID:
                    module->TxRxHalfCpltCallback = callback_ptr;
                    break;
                case SPI_ERROR_CALLBACK_ID:
                    module->ErrorCallback = callback_ptr;
                    break;
                case SPI_ABORT_CALLBACK_ID:
                    module->AbortCpltCallback = callback_ptr;
                    break;
                case SPI_MSP_INIT_CALLBACK_ID:
                    module->MspInitCallback = callback_ptr;
                    break;
                case SPI_MSP_DEINIT_CALLBACK_ID:
                    module->MspDeInitCallback = callback_ptr;
                    break;
                default:
                    STM_HAL_SET_BIT(module->error_code, SPI_ERROR_CALLBACK_ID);
                    status =  SPI_STATUS_ERROR;
                    break;
            }
        }
        else if (SPI_STATE_RESET == module->state)
        {
            switch (callback_id)
            {
                case SPI_MSP_INIT_CALLBACK_ID:
                    module->MspInitCallback = callback_ptr;
                    break;
                case SPI_MSP_DEINIT_CALLBACK_ID:
                    module->MspDeInitCallback = callback_ptr;
                    break;
                default:
                    STM_HAL_SET_BIT(module->error_code, SPI_ERROR_CALLBACK_ID);
                    status =  SPI_STATUS_ERROR;
                    break;
            }
        }
        else
        {
            STM_HAL_SET_BIT(module->error_code, SPI_ERROR_CALLBACK_ID);
            status =  SPI_STATUS_ERROR;
        }
        SPI_UNLOCK_MODULE(module);
        return status;
    }

    spi::status_t spi::spi_unregister_callback(callback_id_t callback_id) const
    {
        status_t status = SPI_STATUS_OK;
        SPI_LOCK_MODULE(module);

        if (SPI_STATE_READY == module->state)
        {
            switch (callback_id)
            {
                case SPI_TX_COMPLETE_CALLBACK_ID:
                    module->TxCpltCallback = HAL_SPI_TxCpltCallback;
                    break;
                case SPI_RX_COMPLETE_CALLBACK_ID:
                    module->RxCpltCallback = HAL_SPI_RxCpltCallback;
                    break;
                case SPI_TX_RX_COMPLETE_CALLBACK_ID:
                    module->TxRxCpltCallback = HAL_SPI_TxRxCplt_Callback;
                    break;
                case SPI_TX_HALF_COMPLETE_CALLBACK_ID:
                    module->TxHalfCpltCallback = HAL_SPI_TxHalfCpltCallback;
                    break;
                case SPI_RX_HALF_COMPLETE_CALLBACK_ID:
                    module->RxHalfCpltCallback = HAL_SPI_RxHalfCpltCallback;
                    break;
                case SPI_TX_RX_HALF_COMPLETE_CALLBACK_ID:
                    module->TxRxHalfCpltCallback = HAL_SPI_TxRxHalfCpltCallback;
                    break;
                case SPI_ERROR_CALLBACK_ID:
                    module->ErrorCallback = HAL_SPI_Error_Callback;
                    break;
                case SPI_ABORT_CALLBACK_ID:
                    module->AbortCpltCallback = HAL_SPI_AbortCpltCallback;
                    break;
                case SPI_MSP_INIT_CALLBACK_ID:
                    module->MspInitCallback = reinterpret_cast<void (*)(_handle_t *)>(HAL_SPI_MspInit);
                    break;
                case SPI_MSP_DEINIT_CALLBACK_ID:
                    module->MspDeInitCallback = reinterpret_cast<void (*)(_handle_t *)>(HAL_SPI_MspDeInit);
                    break;
                default :
                    STM_HAL_SET_BIT(module->error_code, SPI_ERROR_CALLBACK_ID);
                    status =  SPI_STATUS_ERROR;
                    break;
            }
        }
        else if (SPI_STATE_RESET == module->state)
        {
            switch (callback_id)
            {
                case SPI_MSP_INIT_CALLBACK_ID :
                    module->MspInitCallback = reinterpret_cast<void (*)(_handle_t *)>(HAL_SPI_MspInit);
                    break;
                case SPI_MSP_DEINIT_CALLBACK_ID :
                    module->MspDeInitCallback = reinterpret_cast<void (*)(_handle_t *)>(HAL_SPI_MspDeInit);
                    break;
                default :
                    STM_HAL_SET_BIT(module->error_code, SPI_ERROR_CALLBACK_ID);
                    status =  SPI_STATUS_ERROR;
                    break;
            }
        }
        else
        {
            STM_HAL_SET_BIT(module->error_code, SPI_ERROR_CALLBACK_ID);
            status =  SPI_STATUS_ERROR;
        }
        SPI_UNLOCK_MODULE(module);
        return status;
    }
#endif
/********************************************** interrupt service routines ********************************************/
void spi_tx_2_line_8_bit_isr(spi spi_object, struct spi::_handle_t *spi_handle)
{
    *(__IO uint8_t *)&spi_handle->instance->DATA_REG = (*spi_handle->tx_buffer_ptr);
    spi_handle->tx_buffer_ptr++;
    spi_handle->tx_transfer_counter--;
    if (spi_handle->tx_transfer_counter == 0U)
    {
#if (SPI_USE_CRC != 0U)
        if (spi_module_handle->init.crc_calculation == SPI_CRC_CALCULATION_ENABLE)
            {
                STM_HAL_SET_BIT(spi_handle->instance->CONTROL_REG_1, SPI_CR1_CRCNEXT);
                SPI_DISABLE_INTERRUPTS(spi_handle, SPI_TX_BUFFER_EMPTY_INTERRUPT_ENABLE);
                return;
            }
#endif
        SPI_DISABLE_INTERRUPTS(spi_handle, SPI_TX_BUFFER_EMPTY_INTERRUPT_ENABLE);
        if (spi_handle->rx_transfer_counter == 0U) { spi_object.close_rx_tx_isr(); }
    }
}

void spi_rx_2_line_8_bit_isr(spi spi_object, struct spi::_handle_t *spi_handle)
{
    *spi_handle->rx_buffer_ptr = *((__IO uint8_t *)&spi_handle->instance->DATA_REG);     // receive data in 8-bit mode
    spi_handle->rx_buffer_ptr++;
    spi_handle->rx_transfer_counter--;

    if (spi_handle->rx_transfer_counter == 0U)
    {
#if (SPI_USE_CRC != 0U)
        if (spi_handle->init.crc_calculation == SPI_CRC_CALCULATION_ENABLE)
            {
                spi_handle->rx_isr_pointer =  rx_2_line_8_bit_isrCRC;
                return;
            }
#endif
        SPI_DISABLE_INTERRUPTS(spi_handle, (SPI_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE | SPI_ERROR_INTERRUPT_ENABLE));
        if (spi_handle->tx_transfer_counter == 0U) { spi_object.close_rx_tx_isr(); }
    }
}

void spi_tx_2_line_16_bit_isr(spi spi_object, struct spi::_handle_t *spi_handle)
{
    spi_handle->instance->DATA_REG = *((uint16_t *)spi_handle->tx_buffer_ptr);
    spi_handle->tx_buffer_ptr += sizeof(uint16_t);
    spi_handle->tx_transfer_counter--;

    if (spi_handle->tx_transfer_counter == 0U)
    {
#if (SPI_USE_CRC != 0U)
        if (spi_module_handle->init.crc_calculation == SPI_CRC_CALCULATION_ENABLE)
            {
                STM_HAL_SET_BIT(spi_module_handle->instance->CONTROL_REG_1, SPI_CR1_CRCNEXT);
                SPI_DISABLE_INTERRUPTS(spi_module_handle, SPI_TX_BUFFER_EMPTY_INTERRUPT_ENABLE);
                return;
            }
#endif
        SPI_DISABLE_INTERRUPTS(spi_handle, SPI_TX_BUFFER_EMPTY_INTERRUPT_ENABLE);
        if (spi_handle->rx_transfer_counter == 0U) { spi_object.close_rx_tx_isr(); }
    }
}

void spi_rx_2_line_16_bit_isr(spi spi_object, struct spi::_handle_t *spi_handle)
{
    *((uint16_t *)spi_handle->rx_buffer_ptr) = (uint16_t)(spi_handle->instance->DATA_REG);
    spi_handle->rx_buffer_ptr += sizeof(uint16_t);
    spi_handle->rx_transfer_counter--;

    if (spi_handle->rx_transfer_counter == 0U)
    {
        #if (SPI_USE_CRC != 0U)
            if (spi_module_handle->init.crc_calculation == SPI_CRC_CALCULATION_ENABLE)
            {
                spi_module_handle->rx_isr_pointer =  rx_2_line_16_bit_isrCRC;
                return;
            }
        #endif
        SPI_DISABLE_INTERRUPTS(spi_handle, SPI_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE);
        if (spi_handle->tx_transfer_counter == 0U) { spi_object.close_rx_tx_isr(); }
    }
}
/*************************************************** friend functions *************************************************/
spi::handle_t* get_spi_handle(spi* spi_object)
{
    return spi_object->module;
}

spi::status_t dma_abort_interrupt(dma_handle_t *dma_handle)
{
    if(dma_handle->state != DMA_STATE_BUSY)
    {
        dma_handle->error_code = STM_HAL_DMA_ERROR_NO_XFER;
        return spi::SPI_STATUS_ERROR;
    }
    else
    {
        dma_handle->state = DMA_STATE_ABORT;
        STM_HAL_DMA_DISABLE(dma_handle);
    }
    return spi::SPI_STATUS_OK;
}

void dma_abort_on_error(dma_handle_t *dma_handle)
{
    auto *spi_handle = (spi::handle_t *)(((dma_handle_t *)dma_handle)->parent);
    spi_handle->rx_transfer_counter = 0U;
    spi_handle->tx_transfer_counter = 0U;
#if (SPI_USE_REGISTER_CALLBACKS == 1U)
    spi_handle->ErrorCallback(spi_handle);
#else
    HAL_SPI_ErrorCallback(spi_handle);
#endif
}

void spi_irq_handler(spi* spi_object)
{
    uint32_t interrupt_source = spi_object->module->instance->CONTROL_REG_2;
    uint32_t interrupt_flag   = spi_object->module->instance->STATUS_REG;
    if ((SPI_CHECK_FLAG_STATUS(interrupt_flag, SPI_FLAG_OVERRUN) == FLAG_RESET)
        && (SPI_CHECK_FLAG_STATUS(interrupt_flag, SPI_FLAG_RX_BUFFER_NOT_EMPTY) != FLAG_RESET)
        && (SPI_CHECK_INTERRUPT_SOURCE(interrupt_source, SPI_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE) != FLAG_RESET))
    {
        spi_object->module->rx_isr_ptr(*spi_object, spi_object->module);
        return;
    }
    if ((SPI_CHECK_FLAG_STATUS(interrupt_flag, SPI_FLAG_TX_BUFFER_EMPTY) != FLAG_RESET)
        && (SPI_CHECK_INTERRUPT_SOURCE(interrupt_source, SPI_TX_BUFFER_EMPTY_INTERRUPT_ENABLE) != FLAG_RESET))
    {
        spi_object->module->tx_isr_ptr(*spi_object, spi_object->module);
        return;
    }
    if (((SPI_CHECK_FLAG_STATUS(interrupt_flag, SPI_FLAG_MODE_FAULT) != FLAG_RESET)
         || (SPI_CHECK_FLAG_STATUS(interrupt_flag, SPI_FLAG_OVERRUN) != FLAG_RESET)
         || (SPI_CHECK_FLAG_STATUS(interrupt_flag, SPI_FLAG_TI_MODE_FRAME_FORMAT_ERROR) != FLAG_RESET))
        && (SPI_CHECK_INTERRUPT_SOURCE(interrupt_source, SPI_ERROR_INTERRUPT_ENABLE) != FLAG_RESET))
    {
        if (SPI_CHECK_FLAG_STATUS(interrupt_flag, SPI_FLAG_OVERRUN) != FLAG_RESET)
        {
            if (spi_object->module->state != spi::SPI_STATE_BUSY_TX)
            {
                STM_HAL_SET_BIT(spi_object->module->error_code, SPI_ERROR_OVERRUN);
                SPI_CLEAR_OVERRUN_FLAG(spi_object->module);
            }
            else
            {
                SPI_CLEAR_OVERRUN_FLAG(spi_object->module);
                return;
            }
        }
        if (SPI_CHECK_FLAG_STATUS(interrupt_flag, SPI_FLAG_MODE_FAULT) != FLAG_RESET)
        {
            STM_HAL_SET_BIT(spi_object->module->error_code, SPI_ERROR_MODE_FAULT);
            SPI_CLEAR_MODE_FAULT_FLAG(spi_object->module);
        }
        if (SPI_CHECK_FLAG_STATUS(interrupt_flag, SPI_FLAG_TI_MODE_FRAME_FORMAT_ERROR) != FLAG_RESET)
        {
            STM_HAL_SET_BIT(spi_object->module->error_code, SPI_ERROR_TI_MODE_FRAME_FORMAT);
            HAL_SPI_CLEAR_FORMAT_ERROR_FLAG(spi_object->module);
        }

        if (spi_object->module->error_code != SPI_ERROR_NONE)
        {
            SPI_DISABLE_INTERRUPTS(spi_object->module, SPI_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE
                                                       | SPI_TX_BUFFER_EMPTY_INTERRUPT_ENABLE | SPI_ERROR_INTERRUPT_ENABLE);
            spi_object->module->state = spi::SPI_STATE_READY;
            if ((STM_HAL_CHECK_FOR_BIT_STATE_SET(interrupt_source, SPI_CR2_TX_BUFFER_DMA_ENABLE))
                || (HAL_IS_BIT_SET(interrupt_source, SPI_CR2_RX_BUFFER_DMA_ENABLE)))
            {
                STM_HAL_CLEAR_BIT(spi_object->module->instance->CONTROL_REG_2,
                                  (SPI_CR2_TX_BUFFER_DMA_ENABLE | SPI_CR2_RX_BUFFER_DMA_ENABLE));
                if (spi_object->module->rx_dma_handle != nullptr)
                {
                    spi_object->module->rx_dma_handle->transfer_abort_callback = dma_abort_on_error;
                    if (spi::SPI_STATUS_OK != dma_abort_interrupt(spi_object->module->rx_dma_handle))
                        STM_HAL_SET_BIT(spi_object->module->error_code, SPI_ERROR_DURING_ABORT);
                }
                if (spi_object->module->tx_dma_handle != nullptr)
                {
                    spi_object->module->tx_dma_handle->transfer_abort_callback = dma_abort_on_error;
                    if (spi::SPI_STATUS_OK != dma_abort_interrupt(spi_object->module->tx_dma_handle))
                        STM_HAL_SET_BIT(spi_object->module->error_code, SPI_ERROR_DURING_ABORT);
                }
            }
            else
            {
                #if (SPI_USE_REGISTER_CALLBACKS == 1U)
                    spi_object->module->ErrorCallback(spi_object->module);
                #else
                    HAL_SPI_ErrorCallback(spi_object->spi_module_handle);
                #endif
            }
        }
        return;
    }
}

/********************************************* private member functions ***********************************************/

spi::status_t spi::lock_module() const
{
    if (module->lock == HAL_MODULE_LOCKED) { return SPI_STATUS_BUSY; }
    module->lock = HAL_MODULE_LOCKED;
    return SPI_STATUS_OK;
}

void spi::unlock_module() const
{
    module->lock = HAL_MODULE_UNLOCKED;
}

void spi::set_transaction_parameters(uint8_t *tx_data_pointer, uint8_t *rx_data_pointer, uint16_t packet_size) const
{
    module->error_code = SPI_ERROR_NONE;
    module->tx_buffer_ptr = (uint8_t *)tx_data_pointer;
    module->tx_transfer_size = packet_size;
    module->tx_transfer_counter = packet_size;
    module->rx_buffer_ptr = (uint8_t *)rx_data_pointer;
    module->rx_transfer_size = packet_size;
    module->rx_transfer_counter = packet_size;
}

void spi::set_rx_and_tx_interrupt_service_routines() const
{
    if (module->init.data_size == SPI_DATA_SIZE_8_BIT)
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

spi::state_t spi::get_module_communication_state() const
{
    return module->state;
}

uint32_t spi::get_module_operating_mode() const
{
    return (uint32_t) module->init.mode;
}

void spi::verify_communication_direction(uint32_t intended_direction) const
{
    uint32_t set_direction = module->init.direction;
    switch (intended_direction)
    {
        case SPI_DIRECTION_2_LINE:
            assert_param(SPI_VERIFY_DIRECTION_2_LINE(set_direction));
            break;
        case SPI_DIRECTION_2_LINE_RX_ONLY:
            assert_param(SPI_VERIFY_DIRECTION_2_LINE_RX_ONLY(set_direction));
            break;
        case SPI_DIRECTION_1_LINE:
            assert_param(SPI_VERIFY_DIRECTION_1_LINE(set_direction));
            break;
        default:
            break;
    }
}


spi::status_t spi::wait_for_flag_until_timeout(uint32_t flag, flag_status_t flag_status, uint32_t flag_timeout, uint32_t start_time) const
{
    __IO uint32_t count;
    uint32_t adjusted_timeout;
    uint32_t wait_start_time;

    adjusted_timeout    = flag_timeout - (HAL_GetTick() - start_time);
    wait_start_time          = HAL_GetTick();
    count               = adjusted_timeout * ((SystemCoreClock * 32U) >> 20U);

    while ((SPI_GET_FLAG_STATUS(module, flag) ? FLAG_SET : FLAG_RESET) != flag_status)
    {
        if (flag_timeout != HAL_MAX_DELAY)
        {
            if (((HAL_GetTick() - wait_start_time) >= adjusted_timeout) || (adjusted_timeout == 0U))
            {
                SPI_DISABLE_INTERRUPTS(module, (SPI_TX_BUFFER_EMPTY_INTERRUPT_ENABLE
                                                | SPI_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE | SPI_ERROR_INTERRUPT_ENABLE));
                if ((module->init.mode == SPI_MODE_CONTROLLER)
                        && ((module->init.direction == SPI_DIRECTION_1_LINE)
                        || (module->init.direction == SPI_DIRECTION_2_LINE_RX_ONLY)))
                    SPI_DISABLE_MODULE(module);
                if (module->init.crc_calculation == SPI_CRC_CALCULATION_ENABLE)
                    SPI_RESET_CRC_CALCULATION(module);
                module->state = SPI_STATE_READY;
                SPI_UNLOCK_MODULE(module);
                return SPI_STATUS_TIMEOUT;
            }
            if (count == 0U) { adjusted_timeout = 0U; }
            count--;
        }
    }
    return SPI_STATUS_OK;
}

spi::status_t spi::end_rx_transaction(uint32_t flag_timeout, uint32_t start_time)
{
    if ((module->init.mode == SPI_MODE_MASTER)
            && ((module->init.direction == SPI_DIRECTION_1_LINE)
            || (module->init.direction == SPI_DIRECTION_2_LINE_RX_ONLY)))
        SPI_DISABLE_MODULE(module);
    if (module->init.mode == SPI_MODE_MASTER)
    {
        if (module->init.direction != SPI_DIRECTION_2_LINE_RX_ONLY)
        {
            if (wait_for_flag_until_timeout(SPI_FLAG_BUSY, FLAG_RESET, flag_timeout, start_time) != SPI_STATUS_OK)
            {
                STM_HAL_SET_BIT(module->error_code, SPI_ERROR_WAITING_FOR_FLAG);
                return SPI_STATUS_TIMEOUT;
            }
        }
        else
        {
            if (wait_for_flag_until_timeout(SPI_FLAG_RX_BUFFER_NOT_EMPTY, FLAG_RESET, flag_timeout, start_time) != SPI_STATUS_OK)
            {
                STM_HAL_SET_BIT(module->error_code, SPI_ERROR_WAITING_FOR_FLAG);
                return SPI_STATUS_TIMEOUT;
            }
        }
    }
    else
    {
        if (wait_for_flag_until_timeout(SPI_FLAG_RX_BUFFER_NOT_EMPTY, FLAG_RESET, flag_timeout, start_time) != SPI_STATUS_OK)
        {
            STM_HAL_SET_BIT(module->error_code, SPI_ERROR_WAITING_FOR_FLAG);
            return SPI_STATUS_TIMEOUT;
        }
    }
    return SPI_STATUS_OK;
}

spi::status_t spi::end_rx_tx_transaction(uint32_t flag_timeout, uint32_t start_time)
{
    __IO uint32_t count = SPI_BUSY_FLAG_WORK_AROUND_TIMEOUT_1000_US * (SystemCoreClock / 24U / 1000000U);
    if (module->init.mode == SPI_MODE_CONTROLLER)
    {
        if (wait_for_flag_until_timeout(SPI_FLAG_BUSY, FLAG_RESET, flag_timeout, start_time) != SPI_STATUS_OK)
        {
            STM_HAL_SET_BIT(module->error_code, SPI_ERROR_WAITING_FOR_FLAG);
            return SPI_STATUS_TIMEOUT;
        }
    }
    else
    {
        do
        {
            if (count == 0U) { break; }
            count--;
        }   while (SPI_GET_FLAG_STATUS(module, SPI_FLAG_BUSY) != FLAG_RESET);
    }
    return SPI_STATUS_OK;
}

void spi::close_rx_tx_isr()
{
    uint32_t tickstart;
    __IO uint32_t count = SPI_DEFAULT_TIMEOUT_100_US * (SystemCoreClock / 24U / 1000U);
    tickstart = HAL_GetTick();
    SPI_DISABLE_INTERRUPTS(module, SPI_ERROR_INTERRUPT_ENABLE);
    do
    {
        if (count == 0U)
        {
            STM_HAL_SET_BIT(module->error_code, SPI_ERROR_WAITING_FOR_FLAG);
            break;
        }
        count--;
    }   while ((module->instance->STATUS_REG & SPI_FLAG_TX_BUFFER_EMPTY) == FLAG_RESET);

    if (end_rx_tx_transaction(SPI_DEFAULT_TIMEOUT_100_US, tickstart) != SPI_STATUS_OK)
        STM_HAL_SET_BIT(module->error_code, SPI_ERROR_WAITING_FOR_FLAG);
    if (module->init.direction == SPI_DIRECTION_2_LINE)
        SPI_CLEAR_OVERRUN_FLAG(module);

    #if (SPI_USE_CRC != 0U)
        if (SPI_GET_FLAG_STATUS(spi_module_handle, SPI_FLAG_CRC_ERROR) != FLAG_RESET)
        {
            spi_module_handle->state = SPI_STATE_READY;
            STM_HAL_SET_BIT(spi_module_handle->error_code, SPI_ERROR_DURING_CRC_CALCULATION);
            __HAL_SPI_CLEAR_CRCERRFLAG(spi_module_handle);
            #if (SPI_USE_REGISTER_CALLBACKS == 1U)
                spi_module_handle->ErrorCallback(spi_module_handle);
            #else
                HAL_SPI_ErrorCallback(spi_module_handle);
            #endif
        }
        else
        {
    #endif
            if (module->error_code == SPI_ERROR_NONE)
            {
                if (module->state == SPI_STATE_BUSY_RX)
                {
                    module->state = SPI_STATE_READY;
                    #if (SPI_USE_REGISTER_CALLBACKS == 1U)
                        module->RxCpltCallback(module);
                    #else
                        HAL_SPI_RxCpltCallback(spi_module_handle);
                    #endif
                }
                else
                {
                    module->state = SPI_STATE_READY;
                    #if (SPI_USE_REGISTER_CALLBACKS == 1U)
                        module->TxRxCpltCallback(module);
                    #else
                        HAL_SPI_TxRxCpltCallback(spi_module_handle);
                    #endif
                }
            }
            else
            {
                module->state = SPI_STATE_READY;
                #if (SPI_USE_REGISTER_CALLBACKS == 1U)
                    module->ErrorCallback(module);
                #else
                    HAL_SPI_ErrorCallback(spi_module_handle);
                #endif
            }
    #if (SPI_USE_CRC != 0U)
        }
    #endif
}

#if (SPI_USE_CRC != 0U)
    static void rx_2_line_16_bit_isrCRC(struct _handle_t *spi_module_handle)
    {
        __IO uint32_t register_contents = 0U;
        register_contents = STM_HAL_READ_REG(spi_module_handle->instance->DATA_REG);
        STM_HAL_UNUSED(register_contents);
        SPI_DISABLE_INTERRUPTS(spi_module_handle, SPI_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE);
        close_rx_tx_isr(spi_module_handle);
    }

    static void rx_2_line_8_bit_isrCRC(struct _handle_t *spi_module_handle)
    {
        __IO uint8_t  *pregister_contents8;
        __IO uint8_t  register_contents8 = 0;

        pregister_contents8 = (__IO uint8_t *)&spi_module_handle->instance->DATA_REG;
        register_contents8 = *pregister_contents8;
        STM_HAL_UNUSED(register_contents8);
        SPI_DISABLE_INTERRUPTS(spi_module_handle, (SPI_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE | SPI_ERROR_INTERRUPT_ENABLE));
        if (spi_module_handle->tx_transfer_counter == 0U)
            close_rx_tx_isr(spi_module_handle);
    }
#endif

void spi::close_rx_isr()
{
    SPI_DISABLE_INTERRUPTS(module, (SPI_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE | SPI_ERROR_INTERRUPT_ENABLE));
    if (end_rx_transaction(SPI_DEFAULT_TIMEOUT_100_US, HAL_GetTick()) != SPI_STATUS_OK)
        STM_HAL_SET_BIT(module->error_code, SPI_ERROR_WAITING_FOR_FLAG);
    if (module->init.direction == SPI_DIRECTION_2_LINE)
        SPI_CLEAR_OVERRUN_FLAG(module);
    module->state = SPI_STATE_READY;

    #if (SPI_USE_CRC != 0U)
        if (SPI_GET_FLAG_STATUS(spi_module_handle, SPI_FLAG_CRC_ERROR) != FLAG_RESET)
        {
            STM_HAL_SET_BIT(spi_module_handle->error_code, SPI_ERROR_DURING_CRC_CALCULATION);
            __HAL_SPI_CLEAR_CRCERRFLAG(spi_module_handle);
            #if (SPI_USE_REGISTER_CALLBACKS == 1U)
                spi_module_handle->ErrorCallback(spi_module_handle);
            #else
                HAL_SPI_ErrorCallback(spi_module_handle);
            #endif
        }
        else
        {
    #endif
            if (module->error_code == SPI_ERROR_NONE)
            {
                #if (SPI_USE_REGISTER_CALLBACKS == 1U)
                    module->RxCpltCallback(module);
                #else
                    HAL_SPI_RxCpltCallback(spi_module_handle);
                #endif
            }
            else
            {
                #if (SPI_USE_REGISTER_CALLBACKS == 1U)
                    module->ErrorCallback(module);
                #else
                    HAL_SPI_ErrorCallback(spi_module_handle);
                #endif
            }
    #if (SPI_USE_CRC != 0U)
        }
    #endif
}

void spi::close_tx_isr()
{
    uint32_t tickstart;
    __IO uint32_t count = SPI_DEFAULT_TIMEOUT_100_US * (SystemCoreClock / 24U / 1000U);
    tickstart = HAL_GetTick();
    do
    {
        if (count == 0U)
        {
            STM_HAL_SET_BIT(module->error_code, SPI_ERROR_WAITING_FOR_FLAG);
            break;
        }
        count--;
    }   while ((module->instance->STATUS_REG & SPI_FLAG_TX_BUFFER_EMPTY) == FLAG_RESET);

    SPI_DISABLE_INTERRUPTS(module, (SPI_TX_BUFFER_EMPTY_INTERRUPT_ENABLE | SPI_ERROR_INTERRUPT_ENABLE));
    if (end_rx_tx_transaction(SPI_DEFAULT_TIMEOUT_100_US, tickstart) != SPI_STATUS_OK)
        STM_HAL_SET_BIT(module->error_code, SPI_ERROR_WAITING_FOR_FLAG);
    if (module->init.direction == SPI_DIRECTION_2_LINE)
        SPI_CLEAR_OVERRUN_FLAG(module);
    module->state = SPI_STATE_READY;
    if (module->error_code != SPI_ERROR_NONE)
    {
        #if (SPI_USE_REGISTER_CALLBACKS == 1U)
            module->ErrorCallback(module);
        #else
            HAL_SPI_ErrorCallback(spi_module_handle);
        #endif
    }
    else
    {
        #if (SPI_USE_REGISTER_CALLBACKS == 1U)
            module->TxCpltCallback(module);
        #else
            HAL_SPI_TxCpltCallback(spi_module_handle);
        #endif
    }
}

void spi::abort_rx_isr()
{
    __IO uint32_t register_contents = 0U;
    __IO uint32_t count = SPI_DEFAULT_TIMEOUT_100_US * (SystemCoreClock / 24U / 1000U);
    do
    {
        if (count == 0U)
        {
            STM_HAL_SET_BIT(module->error_code, SPI_ERROR_DURING_ABORT );
            break;
        }
        count--;
    }   while ((module->instance->STATUS_REG & SPI_FLAG_TX_BUFFER_EMPTY) == FLAG_RESET);

    SPI_DISABLE_MODULE(module);
    STM_HAL_CLEAR_BIT(module->instance->CONTROL_REG_2, (SPI_TX_BUFFER_EMPTY_INTERRUPT_ENABLE
                                                        | SPI_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE | SPI_ERROR_INTERRUPT_ENABLE));
    register_contents = STM_HAL_READ_REG(module->instance->DATA_REG);
    STM_HAL_UNUSED(register_contents);
    module->state = SPI_STATE_ABORT;
}

void spi::abort_tx_isr()
{
    STM_HAL_CLEAR_BIT(module->instance->CONTROL_REG_2, (SPI_TX_BUFFER_EMPTY_INTERRUPT_ENABLE));
    SPI_DISABLE_MODULE(module);
    module->state = SPI_STATE_ABORT;
}

void spi::reset_enabled_crc()
{
#if (SPI_USE_CRC != 0U)
    if (spi_module_handle->init.crc_calculation == SPI_CRC_CALCULATION_ENABLE) { SPI_RESET_CRC_CALCULATION(spi_module_handle); }
#endif
}
