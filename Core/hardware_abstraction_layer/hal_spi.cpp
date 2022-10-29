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

#include "peripheral_initialization.h"
#include "mcu_clock_timers.h"
#include "hal_general.h"
#include "hal_spi.h"

HAL_StatusTypeDef spi::initialize_module()
{
    if (spi_module_handle == nullptr)
        return HAL_ERROR;
    assert_param(IS_SPI_ALL_INSTANCE(spi_module_handle->instance));
    assert_param(IS_SPI_MODE(spi_module_handle->init.mode));
    assert_param(IS_SPI_DIRECTION(spi_module_handle->init.direction));
    assert_param(IS_SPI_DATASIZE(spi_module_handle->init.data_size));
    assert_param(IS_SPI_NSS(spi_module_handle->init.chip_select_setting));
    assert_param(IS_SPI_BAUDRATE_PRESCALER(spi_module_handle->init.baud_rate_prescaler));
    assert_param(IS_SPI_FIRST_BIT(spi_module_handle->init.first_bit_setting));
    assert_param(IS_SPI_TIMODE(spi_module_handle->init.ti_mode));
    if (spi_module_handle->init.ti_mode == SPI_TI_MODE_DISABLE)
    {
        assert_param(IS_SPI_CPOL(spi_module_handle->init.clock_polarity));
        assert_param(IS_SPI_CPHA(spi_module_handle->init.clock_phase));
        if (spi_module_handle->init.mode == SPI_MODE_MASTER)
            assert_param(IS_SPI_BAUDRATE_PRESCALER(spi_module_handle->init.baud_rate_prescaler));
        else
            spi_module_handle->init.baud_rate_prescaler = SPI_BAUD_RATE_PRESCALER_2;
    }
    else
    {
        assert_param(IS_SPI_BAUDRATE_PRESCALER(spi_module_handle->init.baud_rate_prescaler));
        spi_module_handle->init.clock_polarity = SPI_CLOCK_POLARITY_LOW;
        spi_module_handle->init.clock_phase    = SPI_CLOCK_PHASE_LEADING_EDGE;
    }
    #if (SPI_USE_CRC != 0U)
        assert_param(IS_SPI_CRC_CALCULATION(spi_handle->init.crc_calculation));
        if (spi_handle->init.crc_calculation == SPI_CRCCALCULATION_ENABLE)
            assert_param(IS_SPI_CRC_POLYNOMIAL(spi_handle->init.crc_polynomial));
    #else
        spi_module_handle->init.crc_calculation = SPI_CRC_CALCULATION_DISABLE;
    #endif /* SPI_USE_CRC */

    if (spi_module_handle->state == SPI_STATE_RESET)
    {
        spi_module_handle->lock = HAL_MODULE_UNLOCKED;
        #if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
            spi_module_handle->TxCpltCallback       = reinterpret_cast<void (*)(_spi_handle_t *)>(HAL_SPI_TxCpltCallback);
            spi_module_handle->RxCpltCallback       = reinterpret_cast<void (*)(_spi_handle_t *)>(HAL_SPI_RxCpltCallback);
            spi_module_handle->TxRxCpltCallback     = reinterpret_cast<void (*)(_spi_handle_t *)>(HAL_SPI_TxRxCpltCallback);
            spi_module_handle->TxHalfCpltCallback   = reinterpret_cast<void (*)(_spi_handle_t *)>(HAL_SPI_TxHalfCpltCallback);
            spi_module_handle->RxHalfCpltCallback   = reinterpret_cast<void (*)(_spi_handle_t *)>(HAL_SPI_RxHalfCpltCallback);
            spi_module_handle->TxRxHalfCpltCallback = reinterpret_cast<void (*)(_spi_handle_t *)>(HAL_SPI_TxRxHalfCpltCallback);
            spi_module_handle->ErrorCallback        = reinterpret_cast<void (*)(_spi_handle_t *)>(HAL_SPI_ErrorCallback);
            spi_module_handle->AbortCpltCallback    = reinterpret_cast<void (*)(_spi_handle_t *)>(HAL_SPI_AbortCpltCallback);

            if (spi_module_handle->MspInitCallback == nullptr)
                spi_module_handle->MspInitCallback = reinterpret_cast<void (*)(_spi_handle_t *)>(HAL_SPI_MspInit);
            spi_module_handle->MspInitCallback(spi_module_handle);
        #else
            HAL_SPI_MspInit(spi_handle);
        #endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
    }
    spi_module_handle->state = SPI_STATE_BUSY;
    SPI_DISABLE_MODULE(spi_module_handle);

    WRITE_REG(spi_module_handle->instance->CONTROL_REG_1, (
            (spi_module_handle->init.mode & (SPI_CR1_MODE_CONTROLLER | SPI_CR1_INTERNAL_CHIP_SELECT)) |
            (spi_module_handle->init.direction & (SPI_CR1_RECEIVE_ONLY | SPI_CR1_BIDIRECTIONAL_MODE)) |
            (spi_module_handle->init.data_size & SPI_CR1_DATA_FRAME_FORMAT) |
            (spi_module_handle->init.clock_polarity & SPI_CR1_CLOCK_POLARITY) |
            (spi_module_handle->init.clock_phase & SPI_CR1_CLOCK_PHASE) |
            (spi_module_handle->init.chip_select_setting & SPI_CR1_SOFTWARE_CHIP_SELECT) |
            (spi_module_handle->init.baud_rate_prescaler & SPI_CR1_BAUD_RATE_CONTROL_MASK) |
            (spi_module_handle->init.first_bit_setting  & SPI_CR1_LSB_FIRST) |
            (spi_module_handle->init.crc_calculation & SPI_CR1_CRC_ENABLE)));
    WRITE_REG(spi_module_handle->instance->CONTROL_REG_2, ((
            (spi_module_handle->init.chip_select_setting >> 16U) & SPI_CR2_CHIP_SELECT_OUTPUT_ENABLE) |
            (spi_module_handle->init.ti_mode & SPI_CR2_FRAME_FORMAT)));

    #if (SPI_USE_CRC != 0U)
        if (spi_handle->init.crc_calculation == SPI_CRCCALCULATION_ENABLE)
            WRITE_REG(spi_handle->instance->CRC_POLYNOMIAL_REG, (spi_handle->init.crc_polynomial & SPI_CRCPR_CRCPOLY_Msk));
    #endif /* SPI_USE_CRC */

    #if defined(SPI_I2SCFGR_I2SMOD)
        HAL_GENERAL_CLEAR_BIT(spi_module_handle->instance->I2S_CONFIG_REG, SPI_I2SCFGR_I2SMOD);
    #endif /* SPI_I2SCFGR_I2SMOD */

    spi_module_handle->error_code = SPI_ERROR_NONE;
    spi_module_handle->state     = SPI_STATE_READY;

    return HAL_OK;
}

void spi::configure_module(spi_handle_t* spi_handle)
{
    spi_module_handle = spi_handle;
    spi_module_handle->instance = SPI_2;
    spi_module_handle->init.mode = SPI_MODE_MASTER;
    spi_module_handle->init.direction = SPI_DIRECTION_2LINES;
    spi_module_handle->init.data_size = SPI_DATASIZE_8BIT;
    spi_module_handle->init.clock_polarity = SPI_POLARITY_HIGH;
    spi_module_handle->init.clock_phase = SPI_PHASE_2EDGE;
    spi_module_handle->init.chip_select_setting = SPI_NSS_SOFT;
    spi_module_handle->init.baud_rate_prescaler = SPI_BAUD_RATE_PRESCALER_256;
    spi_module_handle->init.first_bit_setting = SPI_FIRSTBIT_MSB;
    spi_module_handle->init.ti_mode = SPI_TI_MODE_DISABLE;
    spi_module_handle->init.crc_calculation = SPI_CRC_CALCULATION_DISABLE;
    spi_module_handle->init.crc_polynomial = 10;
    if (initialize_module() != HAL_OK) { Error_Handler(); }
}

spi::spi_status_t spi::lock_module() const
{
    if (spi_module_handle->lock == HAL_MODULE_LOCKED) { return SPI_STATUS_BUSY; }
    spi_module_handle->lock = HAL_MODULE_LOCKED;
    return SPI_STATUS_OK;
}

void spi::unlock_module() const
{
    spi_module_handle->lock = HAL_MODULE_UNLOCKED;
}

uint8_t spi::get_module_communication_state() const
{
    return (uint8_t) spi_module_handle->state;
}

uint32_t spi::get_module_operating_mode() const
{
    return (uint32_t) spi_module_handle->init.mode;
}

void spi::verify_communication_direction(uint32_t intended_direction) const
{
    uint32_t set_direction = spi_module_handle->init.direction;
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

void spi::set_transaction_parameters(uint8_t *tx_data_pointer, uint8_t *rx_data_pointer, uint16_t packet_size) const
{
    spi_module_handle->error_code   = SPI_ERROR_NONE;
    spi_module_handle->tx_buffer_pointer  = (uint8_t *)tx_data_pointer;
    spi_module_handle->tx_transfer_size  = packet_size;
    spi_module_handle->tx_transfer_counter = packet_size;
    spi_module_handle->rx_buffer_pointer  = (uint8_t *)rx_data_pointer;
    spi_module_handle->rx_transfer_size  = packet_size;
    spi_module_handle->rx_transfer_counter = packet_size;
}

void spi::reset_enabled_crc()
{
    #if (SPI_USE_CRC != 0U)
        if (spi_module_handle->init.crc_calculation == SPI_CRC_CALCULATION_ENABLE) { SPI_RESET_CRC_CALCULATION(spi_module_handle); }
    #endif
}

HAL_StatusTypeDef spi::wait_for_flag_until_timeout(uint32_t flag, FlagStatus flag_status, uint32_t flag_timeout, uint32_t start_time_from_hal_tick)
{
    __IO uint32_t count;
    uint32_t adjusted_timeout;
    uint32_t start_time;

    adjusted_timeout    = flag_timeout - (HAL_GetTick() - start_time_from_hal_tick);    // adjust timeout value in case of end of transfer
    start_time          = HAL_GetTick();
    count               = adjusted_timeout * ((SystemCoreClock * 32U) >> 20U);              // calculate timeout based on software loop

    while ((SPI_GET_FLAG(spi_module_handle, flag) ? SET : RESET) != flag_status)
    {
        if (flag_timeout != HAL_MAX_DELAY)
        {
            if (((HAL_GetTick() - start_time) >= adjusted_timeout) || (adjusted_timeout == 0U))
            {
                SPI_DISABLE_INTERRUPTS(spi_module_handle, (SPI_TX_BUFFER_EMPTY_INTERRUPT_ENABLE
                        | SPI_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE | SPI_ERROR_INTERRUPT_ENABLE));
                if ((spi_module_handle->init.mode == SPI_MODE_CONTROLLER)
                        && ((spi_module_handle->init.direction == SPI_DIRECTION_1_LINE)
                        || (spi_module_handle->init.direction == SPI_DIRECTION_2_LINE_RX_ONLY)))
                    SPI_DISABLE_MODULE(spi_module_handle);
                if (spi_module_handle->init.crc_calculation == SPI_CRC_CALCULATION_ENABLE)
                    SPI_RESET_CRC_CALCULATION(spi_module_handle);
                spi_module_handle->state = SPI_STATE_READY;
                HAL_UNLOCK_MODULE(spi_module_handle);
                return HAL_TIMEOUT;
            }
            if (count == 0U) { adjusted_timeout = 0U; }
            count--;
        }
    }
    return HAL_OK;
}

HAL_StatusTypeDef spi::end_rx_transaction(uint32_t flag_timeout, uint32_t start_time_from_hal_tick)
{
    if ((spi_module_handle->init.mode == SPI_MODE_MASTER)
            && ((spi_module_handle->init.direction == SPI_DIRECTION_1_LINE)
            || (spi_module_handle->init.direction == SPI_DIRECTION_2_LINE_RX_ONLY)))
        SPI_DISABLE_MODULE(spi_module_handle);
    if (spi_module_handle->init.mode == SPI_MODE_MASTER)
    {
        if (spi_module_handle->init.direction != SPI_DIRECTION_2_LINE_RX_ONLY)
        {
            if (wait_for_flag_until_timeout(SPI_FLAG_BUSY, RESET, flag_timeout, start_time_from_hal_tick) != HAL_OK)
            {
                HAL_GENERAL_SET_BIT(spi_module_handle->error_code, SPI_ERROR_WAITING_FOR_FLAG);
                return HAL_TIMEOUT;
            }
        }
        else
        {
            if (wait_for_flag_until_timeout(SPI_FLAG_RX_BUFFER_NOT_EMPTY, RESET, flag_timeout, start_time_from_hal_tick) != HAL_OK)
            {
                HAL_GENERAL_SET_BIT(spi_module_handle->error_code, SPI_ERROR_WAITING_FOR_FLAG);
                return HAL_TIMEOUT;
            }
        }
    }
    else
    {
        if (wait_for_flag_until_timeout(SPI_FLAG_RX_BUFFER_NOT_EMPTY, RESET, flag_timeout, start_time_from_hal_tick) != HAL_OK)
        {
            HAL_GENERAL_SET_BIT(spi_module_handle->error_code, SPI_ERROR_WAITING_FOR_FLAG);
            return HAL_TIMEOUT;
        }
    }
    return HAL_OK;
}

HAL_StatusTypeDef spi::end_rx_tx_transaction(uint32_t flag_timeout, uint32_t start_time_from_hal_tick)
{
    __IO uint32_t count = SPI_BUSY_FLAG_WORK_AROUND_TIMEOUT_1000_US * (SystemCoreClock / 24U / 1000000U);
    if (spi_module_handle->init.mode == SPI_MODE_CONTROLLER)
    {
        if (wait_for_flag_until_timeout(SPI_FLAG_BUSY, RESET, flag_timeout, start_time_from_hal_tick) != HAL_OK)
        {
            HAL_GENERAL_SET_BIT(spi_module_handle->error_code, SPI_ERROR_WAITING_FOR_FLAG);
            return HAL_TIMEOUT;
        }
    }
    else
    {
        do
        {
            if (count == 0U) { break; }
            count--;
        }   while (SPI_GET_FLAG(spi_module_handle, SPI_FLAG_BUSY) != RESET);
    }
    return HAL_OK;
}

void spi::close_rx_tx_isr()
{
    uint32_t tickstart;
    __IO uint32_t count = SPI_DEFAULT_TIMEOUT_100_US * (SystemCoreClock / 24U / 1000U);
    tickstart = HAL_GetTick();      //timeout management
    SPI_DISABLE_INTERRUPTS(spi_module_handle, SPI_ERROR_INTERRUPT_ENABLE);
    do
    {
        if (count == 0U)
        {
            HAL_GENERAL_SET_BIT(spi_module_handle->error_code, SPI_ERROR_WAITING_FOR_FLAG);
            break;
        }
        count--;
    }   while ((spi_module_handle->instance->STATUS_REG & SPI_FLAG_TX_BUFFER_EMPTY) == RESET);

    if (end_rx_tx_transaction(SPI_DEFAULT_TIMEOUT_100_US, tickstart) != HAL_OK)
        HAL_GENERAL_SET_BIT(spi_module_handle->error_code, SPI_ERROR_WAITING_FOR_FLAG);
    if (spi_module_handle->init.direction == SPI_DIRECTION_2_LINE)
        SPI_CLEAR_OVERRUN_FLAG(spi_module_handle);

    #if (SPI_USE_CRC != 0U)
        if (SPI_GET_FLAG(spi_module_handle, SPI_FLAG_CRC_ERROR) != RESET)
        {
            spi_module_handle->state = SPI_STATE_READY;
            HAL_GENERAL_SET_BIT(spi_module_handle->error_code, SPI_ERROR_DURING_CRC_CALCULATION);
            __HAL_SPI_CLEAR_CRCERRFLAG(spi_module_handle);
            #if (SPI_USE_REGISTER_CALLBACKS == 1U)
                spi_module_handle->ErrorCallback(spi_module_handle);
            #else
                HAL_SPI_ErrorCallback(spi_module_handle);
            #endif /* SPI_USE_REGISTER_CALLBACKS */
        }
        else
        {
    #endif /* SPI_USE_CRC */
            if (spi_module_handle->error_code == SPI_ERROR_NONE)
            {
                if (spi_module_handle->state == SPI_STATE_BUSY_RX)
                {
                    spi_module_handle->state = SPI_STATE_READY;
                    #if (SPI_USE_REGISTER_CALLBACKS == 1U)
                        spi_module_handle->RxCpltCallback(spi_module_handle);
                    #else
                        HAL_SPI_RxCpltCallback(spi_module_handle);
                    #endif /* SPI_USE_REGISTER_CALLBACKS */
                }
                else
                {
                    spi_module_handle->state = SPI_STATE_READY;
                    #if (SPI_USE_REGISTER_CALLBACKS == 1U)
                        spi_module_handle->TxRxCpltCallback(spi_module_handle);
                    #else
                        HAL_SPI_TxRxCpltCallback(spi_module_handle);
                    #endif /* SPI_USE_REGISTER_CALLBACKS */
                }
            }
            else
            {
                spi_module_handle->state = SPI_STATE_READY;
                #if (SPI_USE_REGISTER_CALLBACKS == 1U)
                    spi_module_handle->ErrorCallback(spi_module_handle);
                #else
                    HAL_SPI_ErrorCallback(spi_module_handle);
                #endif /* SPI_USE_REGISTER_CALLBACKS */
            }
    #if (SPI_USE_CRC != 0U)
        }
    #endif /* SPI_USE_CRC */
}

#if (SPI_USE_CRC != 0U)
    static void rx_2_line_16_bit_isrCRC(struct _spi_handle_t *spi_module_handle)
    {
        __IO uint32_t register_contents = 0U;
        register_contents = HAL_GENERAL_READ_REG(spi_module_handle->instance->DATA_REG);      // read 16-bit crc to flush data register
        HAL_GENERAL_UNUSED(register_contents);
        SPI_DISABLE_INTERRUPTS(spi_module_handle, SPI_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE);
        close_rx_tx_isr(spi_module_handle);
    }
#endif /* SPI_USE_CRC */


#if (SPI_USE_CRC != 0U)
    static void rx_2_line_8_bit_isrCRC(struct _spi_handle_t *spi_module_handle)
    {
        __IO uint8_t  *pregister_contents8;
        __IO uint8_t  register_contents8 = 0;

        pregister_contents8 = (__IO uint8_t *)&spi_module_handle->instance->DATA_REG;
        register_contents8 = *pregister_contents8;        // read crc to flush data register
        HAL_GENERAL_UNUSED(register_contents8);
        SPI_DISABLE_INTERRUPTS(spi_module_handle, (SPI_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE | SPI_ERROR_INTERRUPT_ENABLE));
        if (spi_module_handle->tx_transfer_counter == 0U)
            close_rx_tx_isr(spi_module_handle);
    }
#endif /* SPI_USE_CRC */

HAL_StatusTypeDef spi::spi_transmit_receive_interrupt(uint8_t *tx_data_pointer, uint8_t *rx_data_pointer, uint16_t packet_size)
{
    uint8_t spi_procedure_error = SPI_PROCEDURE_ERROR_NONE;
    uint32_t spi_module_mode;
    spi_state_t spi_module_state;
    spi_module_state = (spi_state_t) get_module_communication_state();
    spi_module_mode = get_module_operating_mode();

    if ((tx_data_pointer == nullptr) || (rx_data_pointer == nullptr) || (packet_size == 0U))
        spi_procedure_error = SPI_PROCEDURE_STATE_DATA_ERROR;

    assert_param(IS_SPI_DIRECTION_2LINES(spi_module_handle->init.direction));
    verify_communication_direction(SPI_DIRECTION_2_LINE);
    lock_module();
    if (!((spi_module_state == SPI_STATE_READY)
            || ((spi_module_mode == SPI_MODE_CONTROLLER)
            && (spi_module_handle->init.direction == SPI_DIRECTION_2_LINE)
            && (spi_module_state == SPI_STATE_BUSY_RX))))
        spi_procedure_error = SPI_PROCEDURE_STATE_BUS_ERROR;
    else if (spi_module_handle->state != SPI_STATE_BUSY_RX)
        spi_module_handle->state = SPI_STATE_BUSY_TX_RX;

    set_transaction_parameters(tx_data_pointer, rx_data_pointer, packet_size);
    set_rx_and_tx_interrupt_service_routines();
    reset_enabled_crc();

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    SPI_ENABLE_INTERRUPTS(spi_module_handle, (SPI_TX_BUFFER_EMPTY_INTERRUPT_ENABLE
            | SPI_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE | SPI_ERROR_INTERRUPT_ENABLE));
    if ((spi_module_handle->instance->CONTROL_REG_1 & SPI_CR1_SPE) != SPI_CR1_SPE)
        SPI_ENABLE_MODULE(spi_module_handle);
    unlock_module();
    if (spi_procedure_error == SPI_PROCEDURE_STATE_BUS_ERROR) { return HAL_BUSY; }
    if (spi_procedure_error == SPI_PROCEDURE_STATE_DATA_ERROR) { return HAL_ERROR; }
    return HAL_OK;
}

void spi::close_rx_isr()
{
    SPI_DISABLE_INTERRUPTS(spi_module_handle, (SPI_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE | SPI_ERROR_INTERRUPT_ENABLE));
    if (end_rx_transaction(SPI_DEFAULT_TIMEOUT_100_US, HAL_GetTick()) != HAL_OK)
        HAL_GENERAL_SET_BIT(spi_module_handle->error_code, SPI_ERROR_WAITING_FOR_FLAG);
    if (spi_module_handle->init.direction == SPI_DIRECTION_2_LINE)
        SPI_CLEAR_OVERRUN_FLAG(spi_module_handle);
    spi_module_handle->state = SPI_STATE_READY;

    #if (SPI_USE_CRC != 0U)
        if (SPI_GET_FLAG(spi_module_handle, SPI_FLAG_CRC_ERROR) != RESET)
        {
            HAL_GENERAL_SET_BIT(spi_module_handle->error_code, SPI_ERROR_DURING_CRC_CALCULATION);
            __HAL_SPI_CLEAR_CRCERRFLAG(spi_module_handle);
            #if (SPI_USE_REGISTER_CALLBACKS == 1U)
                spi_module_handle->ErrorCallback(spi_module_handle);
            #else
                HAL_SPI_ErrorCallback(spi_module_handle);
            #endif /* SPI_USE_REGISTER_CALLBACKS */
        }
        else
        {
    #endif /* SPI_USE_CRC */
            if (spi_module_handle->error_code == SPI_ERROR_NONE)
            {
                #if (SPI_USE_REGISTER_CALLBACKS == 1U)
                    spi_module_handle->RxCpltCallback(spi_module_handle);
                #else
                    HAL_SPI_RxCpltCallback(spi_module_handle);
                #endif /* SPI_USE_REGISTER_CALLBACKS */
            }
            else
            {
                #if (SPI_USE_REGISTER_CALLBACKS == 1U)
                    spi_module_handle->ErrorCallback(spi_module_handle);
                #else
                    HAL_SPI_ErrorCallback(spi_module_handle);
                #endif /* SPI_USE_REGISTER_CALLBACKS */
            }
    #if (SPI_USE_CRC != 0U)
        }
    #endif /* SPI_USE_CRC */
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
            HAL_GENERAL_SET_BIT(spi_module_handle->error_code, SPI_ERROR_WAITING_FOR_FLAG);
            break;
        }
        count--;
    }   while ((spi_module_handle->instance->STATUS_REG & SPI_FLAG_TX_BUFFER_EMPTY) == RESET);

    SPI_DISABLE_INTERRUPTS(spi_module_handle, (SPI_TX_BUFFER_EMPTY_INTERRUPT_ENABLE | SPI_ERROR_INTERRUPT_ENABLE));
    if (end_rx_tx_transaction(SPI_DEFAULT_TIMEOUT_100_US, tickstart) != HAL_OK)
        HAL_GENERAL_SET_BIT(spi_module_handle->error_code, SPI_ERROR_WAITING_FOR_FLAG);
    if (spi_module_handle->init.direction == SPI_DIRECTION_2_LINE)
        SPI_CLEAR_OVERRUN_FLAG(spi_module_handle);
    spi_module_handle->state = SPI_STATE_READY;
    if (spi_module_handle->error_code != SPI_ERROR_NONE)
    {
        #if (SPI_USE_REGISTER_CALLBACKS == 1U)
            spi_module_handle->ErrorCallback(spi_module_handle);
        #else
            HAL_SPI_ErrorCallback(spi_module_handle);
        #endif /* SPI_USE_REGISTER_CALLBACKS */
    }
    else
    {
        #if (SPI_USE_REGISTER_CALLBACKS == 1U)
            spi_module_handle->TxCpltCallback(spi_module_handle);
        #else
            HAL_SPI_TxCpltCallback(spi_module_handle);
        #endif /* SPI_USE_REGISTER_CALLBACKS */
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
            HAL_GENERAL_SET_BIT(spi_module_handle->error_code, SPI_ERROR_DURING_ABORT );
            break;
        }
        count--;
    }   while ((spi_module_handle->instance->STATUS_REG & SPI_FLAG_TX_BUFFER_EMPTY) == RESET);

    SPI_DISABLE_MODULE(spi_module_handle);
    HAL_GENERAL_CLEAR_BIT(spi_module_handle->instance->CONTROL_REG_2, (SPI_TX_BUFFER_EMPTY_INTERRUPT_ENABLE
            | SPI_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE | SPI_ERROR_INTERRUPT_ENABLE));
    register_contents = HAL_GENERAL_READ_REG(spi_module_handle->instance->DATA_REG);     // flush data register
    HAL_GENERAL_UNUSED(register_contents);                                         // avoid compiler warning
    spi_module_handle->state = SPI_STATE_ABORT;
}

void spi::abort_tx_isr()
{
    HAL_GENERAL_CLEAR_BIT(spi_module_handle->instance->CONTROL_REG_2, (SPI_TX_BUFFER_EMPTY_INTERRUPT_ENABLE));
    SPI_DISABLE_MODULE(spi_module_handle);
    spi_module_handle->state = SPI_STATE_ABORT;
}

#if (SPI_USE_REGISTER_CALLBACKS == 1U)

    HAL_StatusTypeDef spi::spi_register_callback(spi_callback_id_t CallbackID, spi_callback_pointer_t pCallback)
    {
        HAL_StatusTypeDef status = HAL_OK;
        if (pCallback == nullptr)
        {
            spi_module_handle->error_code |= SPI_ERROR_CALLBACK_ID;
            return HAL_ERROR;
        }
        HAL_LOCK_MODULE(spi_module_handle);

        if (SPI_STATE_READY == spi_module_handle->state)
        {
            switch (CallbackID)
            {
                case SPI_TX_COMPLETE_CALLBACK_ID:
                    spi_module_handle->TxCpltCallback = pCallback;
                    break;
                case SPI_RX_COMPLETE_CALLBACK_ID:
                    spi_module_handle->RxCpltCallback = pCallback;
                    break;
                case SPI_TX_RX_COMPLETE_CALLBACK_ID:
                    spi_module_handle->TxRxCpltCallback = pCallback;
                    break;
                case SPI_TX_HALF_COMPLETE_CALLBACK_ID:
                    spi_module_handle->TxHalfCpltCallback = pCallback;
                    break;
                case SPI_RX_HALF_COMPLETE_CALLBACK_ID:
                    spi_module_handle->RxHalfCpltCallback = pCallback;
                    break;
                case SPI_TX_RX_HALF_COMPLETE_CALLBACK_ID:
                    spi_module_handle->TxRxHalfCpltCallback = pCallback;
                    break;
                case SPI_ERROR_CALLBACK_ID:
                    spi_module_handle->ErrorCallback = pCallback;
                    break;
                case SPI_ABORT_CALLBACK_ID:
                    spi_module_handle->AbortCpltCallback = pCallback;
                    break;
                case SPI_MSP_INIT_CALLBACK_ID:
                    spi_module_handle->MspInitCallback = pCallback;
                    break;
                case SPI_MSP_DEINIT_CALLBACK_ID:
                    spi_module_handle->MspDeInitCallback = pCallback;
                    break;
                default:
                    HAL_GENERAL_SET_BIT(spi_module_handle->error_code, SPI_ERROR_CALLBACK_ID);
                    status =  HAL_ERROR;
                break;
            }
        }
        else if (SPI_STATE_RESET == spi_module_handle->state)
        {
            switch (CallbackID)
            {
                case SPI_MSP_INIT_CALLBACK_ID:
                    spi_module_handle->MspInitCallback = pCallback;
                    break;
                case SPI_MSP_DEINIT_CALLBACK_ID:
                    spi_module_handle->MspDeInitCallback = pCallback;
                    break;
                default:
                    HAL_GENERAL_SET_BIT(spi_module_handle->error_code, SPI_ERROR_CALLBACK_ID);
                    status =  HAL_ERROR;
                    break;
            }
        }
        else
        {
            HAL_GENERAL_SET_BIT(spi_module_handle->error_code, SPI_ERROR_CALLBACK_ID);
            status =  HAL_ERROR;
        }
        HAL_UNLOCK_MODULE(spi_module_handle);
        return status;
    }

    HAL_StatusTypeDef spi::spi_unregister_callback(HAL_SPI_CallbackIDTypeDef CallbackID)
    {
        HAL_StatusTypeDef status = HAL_OK;
        HAL_LOCK_MODULE(spi_module_handle);

        if (SPI_STATE_READY == spi_module_handle->state)
        {
            switch (CallbackID)
            {
                case HAL_SPI_TX_COMPLETE_CB_ID:
                    spi_module_handle->TxCpltCallback = reinterpret_cast<void (*)(_spi_handle_t *)>(HAL_SPI_TxCpltCallback);
                    break;
                case HAL_SPI_RX_COMPLETE_CB_ID:
                    spi_module_handle->RxCpltCallback = reinterpret_cast<void (*)(_spi_handle_t *)>(HAL_SPI_RxCpltCallback);
                    break;
                case HAL_SPI_TX_RX_COMPLETE_CB_ID:
                    spi_module_handle->TxRxCpltCallback = reinterpret_cast<void (*)(_spi_handle_t *)>(HAL_SPI_TxRxCpltCallback);
                    break;
                case HAL_SPI_TX_HALF_COMPLETE_CB_ID:
                    spi_module_handle->TxHalfCpltCallback = reinterpret_cast<void (*)(_spi_handle_t *)>(HAL_SPI_TxHalfCpltCallback);
                    break;
                case HAL_SPI_RX_HALF_COMPLETE_CB_ID:
                    spi_module_handle->RxHalfCpltCallback = reinterpret_cast<void (*)(_spi_handle_t *)>(HAL_SPI_RxHalfCpltCallback);
                    break;
                case HAL_SPI_TX_RX_HALF_COMPLETE_CB_ID:
                    spi_module_handle->TxRxHalfCpltCallback = reinterpret_cast<void (*)(_spi_handle_t *)>(HAL_SPI_TxRxHalfCpltCallback);
                    break;
                case HAL_SPI_ERROR_CB_ID:
                    spi_module_handle->ErrorCallback = reinterpret_cast<void (*)(_spi_handle_t *)>(HAL_SPI_ErrorCallback);
                    break;
                case HAL_SPI_ABORT_CB_ID:
                    spi_module_handle->AbortCpltCallback = reinterpret_cast<void (*)(_spi_handle_t *)>(HAL_SPI_AbortCpltCallback);
                    break;
                case HAL_SPI_MSPINIT_CB_ID:
                    spi_module_handle->MspInitCallback = reinterpret_cast<void (*)(_spi_handle_t *)>(HAL_SPI_MspInit);
                    break;
                case HAL_SPI_MSPDEINIT_CB_ID:
                    spi_module_handle->MspDeInitCallback = reinterpret_cast<void (*)(_spi_handle_t *)>(HAL_SPI_MspDeInit);
                    break;
                default :
                    HAL_GENERAL_SET_BIT(spi_module_handle->error_code, SPI_ERROR_CALLBACK_ID);
                    status =  HAL_ERROR;
                    break;
            }
        }
        else if (SPI_STATE_RESET == spi_module_handle->state)
        {
            switch (CallbackID)
            {
                case HAL_SPI_MSPINIT_CB_ID :
                    spi_module_handle->MspInitCallback = reinterpret_cast<void (*)(_spi_handle_t *)>(HAL_SPI_MspInit);
                    break;
                case HAL_SPI_MSPDEINIT_CB_ID :
                    spi_module_handle->MspDeInitCallback = reinterpret_cast<void (*)(_spi_handle_t *)>(HAL_SPI_MspDeInit);
                    break;
                default :
                    HAL_GENERAL_SET_BIT(spi_module_handle->error_code, SPI_ERROR_CALLBACK_ID);
                    status =  HAL_ERROR;
                    break;
            }
        }
        else
        {
            HAL_GENERAL_SET_BIT(spi_module_handle->error_code, SPI_ERROR_CALLBACK_ID);
            status =  HAL_ERROR;
        }
        HAL_UNLOCK_MODULE(spi_module_handle);
        return status;
    }
#endif /* SPI_USE_REGISTER_CALLBACKS */

void spi_tx_2_line_8_bit_isr(spi spi_object, struct spi::_spi_handle_t *spi_handle)
{
    *(__IO uint8_t *)&spi_handle->instance->DATA_REG = (*spi_handle->tx_buffer_pointer);
    spi_handle->tx_buffer_pointer++;
    spi_handle->tx_transfer_counter--;
    if (spi_handle->tx_transfer_counter == 0U)
    {
        #if (SPI_USE_CRC != 0U)
            if (spi_module_handle->init.crc_calculation == SPI_CRC_CALCULATION_ENABLE)
            {
                HAL_GENERAL_SET_BIT(spi_handle->instance->CONTROL_REG_1, SPI_CR1_CRCNEXT);
                SPI_DISABLE_INTERRUPTS(spi_handle, SPI_TX_BUFFER_EMPTY_INTERRUPT_ENABLE);
                return;
            }
        #endif /* SPI_USE_CRC */
        SPI_DISABLE_INTERRUPTS(spi_handle, spi::SPI_TX_BUFFER_EMPTY_INTERRUPT_ENABLE);
        if (spi_handle->rx_transfer_counter == 0U) { spi_object.close_rx_tx_isr(); }
    }
}

void spi_rx_2_line_8_bit_isr(spi spi_object, struct spi::_spi_handle_t *spi_handle)
{
    *spi_handle->rx_buffer_pointer = *((__IO uint8_t *)&spi_handle->instance->DATA_REG);     // receive data in 8-bit mode
    spi_handle->rx_buffer_pointer++;
    spi_handle->rx_transfer_counter--;

    if (spi_handle->rx_transfer_counter == 0U)        // check for end of receive
    {
        #if (SPI_USE_CRC != 0U)
            if (spi_handle->init.crc_calculation == SPI_CRC_CALCULATION_ENABLE)
            {
                spi_handle->rx_isr_pointer =  rx_2_line_8_bit_isrCRC;
                return;
            }
        #endif /* SPI_USE_CRC */
        SPI_DISABLE_INTERRUPTS(spi_handle, (spi::SPI_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE | spi::SPI_ERROR_INTERRUPT_ENABLE));
        if (spi_handle->tx_transfer_counter == 0U) { spi_object.close_rx_tx_isr(); }
    }
}

void spi_tx_2_line_16_bit_isr(spi spi_object, struct spi::_spi_handle_t *spi_handle)
{
    spi_handle->instance->DATA_REG = *((uint16_t *)spi_handle->tx_buffer_pointer);
    spi_handle->tx_buffer_pointer += sizeof(uint16_t);
    spi_handle->tx_transfer_counter--;

    if (spi_handle->tx_transfer_counter == 0U)
    {
        #if (SPI_USE_CRC != 0U)
            if (spi_module_handle->init.crc_calculation == SPI_CRC_CALCULATION_ENABLE)
            {
                HAL_GENERAL_SET_BIT(spi_module_handle->instance->CONTROL_REG_1, SPI_CR1_CRCNEXT);      // set crc next bit to send crc
                SPI_DISABLE_INTERRUPTS(spi_module_handle, SPI_TX_BUFFER_EMPTY_INTERRUPT_ENABLE);
                return;
            }
        #endif /* SPI_USE_CRC */
        SPI_DISABLE_INTERRUPTS(spi_handle, spi::SPI_TX_BUFFER_EMPTY_INTERRUPT_ENABLE);
        if (spi_handle->rx_transfer_counter == 0U) { spi_object.close_rx_tx_isr(); }
    }
}

void spi_rx_2_line_16_bit_isr(spi spi_object, struct spi::_spi_handle_t *spi_handle)
{
    *((uint16_t *)spi_handle->rx_buffer_pointer) = (uint16_t)(spi_handle->instance->DATA_REG); // receive data in 16-bit mode
    spi_handle->rx_buffer_pointer += sizeof(uint16_t);
    spi_handle->rx_transfer_counter--;

    if (spi_handle->rx_transfer_counter == 0U)
    {
        #if (SPI_USE_CRC != 0U)
            if (spi_module_handle->init.crc_calculation == SPI_CRC_CALCULATION_ENABLE)
            {
                spi_module_handle->rx_isr_pointer =  rx_2_line_16_bit_isrCRC;
                return;
            }
        #endif /* SPI_USE_CRC */
        SPI_DISABLE_INTERRUPTS(spi_handle, spi::SPI_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE);
        if (spi_handle->tx_transfer_counter == 0U) { spi_object.close_rx_tx_isr(); }
    }
}

void SPI_DMAAbortOnError(spi::dma_handle_t *hdma)
{
    auto *spi_handle = (spi::spi_handle_t *)(((spi::dma_handle_t *)hdma)->parent); /* Derogation MISRAC2012-Rule-11.5 */
    spi_handle->rx_transfer_counter = 0U;
    spi_handle->tx_transfer_counter = 0U;
    #if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
        spi_handle->ErrorCallback(spi_handle);
    #else
        HAL_SPI_ErrorCallback(spi_handle);
    #endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
}

spi::spi_status_t dma_abort_interrupt(spi::dma_handle_t *hdma)
{
    if(hdma->state != spi::DMA_STATE_BUSY)
    {
        hdma->error_code = HAL_DMA_ERROR_NO_XFER;
        return spi::SPI_STATUS_ERROR;
    }
    else
    {
        /* Set Abort State  */
        hdma->state = spi::DMA_STATE_ABORT;

        /* Disable the stream */
        HAL_DMA_DISABLE(hdma);
    }
    return spi::SPI_STATUS_OK;
}

void spi::set_rx_and_tx_interrupt_service_routines() const
{
    if (spi_module_handle->init.data_size == SPI_DATA_SIZE_8_BIT)
    {
        spi_module_handle->rx_isr_pointer     = spi_rx_2_line_8_bit_isr;
        spi_module_handle->tx_isr_pointer     = spi_tx_2_line_8_bit_isr;
    }
    else
    {
        spi_module_handle->rx_isr_pointer     = spi_rx_2_line_16_bit_isr;
        spi_module_handle->tx_isr_pointer     = spi_tx_2_line_16_bit_isr;
    }
}

void spi_irq_handler(spi* spi_object)
{
    uint32_t interrupt_source = spi_object->spi_module_handle->instance->CONTROL_REG_2;
    uint32_t interrupt_flag   = spi_object->spi_module_handle->instance->STATUS_REG;
    // receiving
    if ((SPI_CHECK_FLAG(interrupt_flag, spi::SPI_FLAG_OVERRUN) == RESET)
            && (SPI_CHECK_FLAG(interrupt_flag, spi::SPI_FLAG_RX_BUFFER_NOT_EMPTY) != RESET)
            && (SPI_CHECK_IT_SOURCE(interrupt_source, spi::SPI_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE) != RESET))
    {
        spi_object->spi_module_handle->rx_isr_pointer(*spi_object, spi_object->spi_module_handle);
        return;
    }
    // sending
    if ((SPI_CHECK_FLAG(interrupt_flag, spi::SPI_FLAG_TX_BUFFER_EMPTY) != RESET)
            && (SPI_CHECK_IT_SOURCE(interrupt_source, spi::SPI_TX_BUFFER_EMPTY_INTERRUPT_ENABLE) != RESET))
    {
        spi_object->spi_module_handle->tx_isr_pointer(*spi_object, spi_object->spi_module_handle);
        return;
    }
    // error handling
    if (((SPI_CHECK_FLAG(interrupt_flag, spi::SPI_FLAG_MODE_FAULT) != RESET)
            || (SPI_CHECK_FLAG(interrupt_flag, spi::SPI_FLAG_OVERRUN) != RESET)
            || (SPI_CHECK_FLAG(interrupt_flag, spi::SPI_FLAG_TI_MODE_FRAME_FORMAT_ERROR) != RESET))
            && (SPI_CHECK_IT_SOURCE(interrupt_source, spi::SPI_ERROR_INTERRUPT_ENABLE) != RESET))
    {
        if (SPI_CHECK_FLAG(interrupt_flag, spi::SPI_FLAG_OVERRUN) != RESET)
        {
            if (spi_object->spi_module_handle->state != spi::SPI_STATE_BUSY_TX)
            {
                HAL_GENERAL_SET_BIT(spi_object->spi_module_handle->error_code, spi::SPI_ERROR_OVERRUN);
                SPI_CLEAR_OVERRUN_FLAG(spi_object->spi_module_handle);
            }
            else
            {
                SPI_CLEAR_OVERRUN_FLAG(spi_object->spi_module_handle);
                return;
            }
        }
        if (SPI_CHECK_FLAG(interrupt_flag, spi::SPI_FLAG_MODE_FAULT) != RESET)
        {
            HAL_GENERAL_SET_BIT(spi_object->spi_module_handle->error_code, spi::SPI_ERROR_MODE_FAULT);
            SPI_CLEAR_MODE_FAULT_FLAG(spi_object->spi_module_handle);
        }
        if (SPI_CHECK_FLAG(interrupt_flag, spi::SPI_FLAG_TI_MODE_FRAME_FORMAT_ERROR) != RESET)
        {
            HAL_GENERAL_SET_BIT(spi_object->spi_module_handle->error_code, spi::SPI_ERROR_TI_MODE_FRAME_FORMAT);
            HAL_SPI_CLEAR_FORMAT_ERROR_FLAG(spi_object->spi_module_handle);
        }

        if (spi_object->spi_module_handle->error_code != spi::SPI_ERROR_NONE)
        {
            SPI_DISABLE_INTERRUPTS(spi_object->spi_module_handle, spi::SPI_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE
                    | spi::SPI_TX_BUFFER_EMPTY_INTERRUPT_ENABLE | spi::SPI_ERROR_INTERRUPT_ENABLE);
            spi_object->spi_module_handle->state = spi::SPI_STATE_READY;
            if ((HAL_GENERAL_IS_BIT_SET(interrupt_source, spi::SPI_CR2_TX_BUFFER_DMA_ENABLE))
                    || (HAL_IS_BIT_SET(interrupt_source, spi::SPI_CR2_RX_BUFFER_DMA_ENABLE)))
            {
                HAL_GENERAL_CLEAR_BIT(spi_object->spi_module_handle->instance->CONTROL_REG_2,
                        (spi::SPI_CR2_TX_BUFFER_DMA_ENABLE | spi::SPI_CR2_RX_BUFFER_DMA_ENABLE));
                if (spi_object->spi_module_handle->rx_dma_handle != nullptr)
                {
                    spi_object->spi_module_handle->rx_dma_handle->transfer_abort_callback = SPI_DMAAbortOnError;
                    if (spi::SPI_STATUS_OK != dma_abort_interrupt(spi_object->spi_module_handle->rx_dma_handle))
                        HAL_GENERAL_SET_BIT(spi_object->spi_module_handle->error_code, spi::SPI_ERROR_DURING_ABORT);
                }
                if (spi_object->spi_module_handle->tx_dma_handle != nullptr)
                {
                    spi_object->spi_module_handle->tx_dma_handle->transfer_abort_callback = SPI_DMAAbortOnError;
                    if (spi::SPI_STATUS_OK != dma_abort_interrupt(spi_object->spi_module_handle->tx_dma_handle))
                        HAL_GENERAL_SET_BIT(spi_object->spi_module_handle->error_code, spi::SPI_ERROR_DURING_ABORT);
                }
            }
            else
            {
                #if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
                    spi_object->spi_module_handle->ErrorCallback(spi_object->spi_module_handle);
                #else
                    HAL_SPI_ErrorCallback(spi_handle);
                #endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
            }
        }
        return;
    }
}

__weak void HAL_SPI_TxCpltCallback(spi::spi_handle_t *spi_handle)
{
    HAL_GENERAL_UNUSED(spi_handle);
}

__weak void HAL_SPI_RxCpltCallback(spi::spi_handle_t *spi_handle)
{
    HAL_GENERAL_UNUSED(spi_handle);
}

__weak void HAL_SPI_TxRxCpltCallback(spi::spi_handle_t *spi_handle)
{
    HAL_GENERAL_UNUSED(spi_handle);
}

__weak void HAL_SPI_TxHalfCpltCallback(spi::spi_handle_t *spi_handle)
{
    HAL_GENERAL_UNUSED(spi_handle);
}

__weak void HAL_SPI_RxHalfCpltCallback(spi::spi_handle_t *spi_handle)
{
    HAL_GENERAL_UNUSED(spi_handle);
}

__weak void HAL_SPI_TxRxHalfCpltCallback(spi::spi_handle_t *spi_handle)
{
    HAL_GENERAL_UNUSED(spi_handle);
}

__weak void HAL_SPI_ErrorCallback(spi::spi_handle_t *spi_handle)
{
    HAL_GENERAL_UNUSED(spi_handle);
}

__weak void HAL_SPI_AbortCpltCallback(spi::spi_handle_t *spi_handle)
{
    HAL_GENERAL_UNUSED(spi_handle);
}
