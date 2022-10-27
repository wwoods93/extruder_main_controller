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

#if (HAL_SPI_USE_REGISTER_CALLBACKS == 1U)
    #define SPI_ERROR_NONE                              (0x00000000U)
    #define SPI_ERROR_MODE_FAULT                        (0x00000001U)
    #define SPI_ERROR_DURING_CRC_CALCULATION            (0x00000002U)
    #define SPI_ERROR_OVERRUN                           (0x00000004U)
    #define SPI_ERROR_TI_MODE_FRAME_FORMAT              (0x00000008U)
    #define SPI_ERROR_DMA_TRANSFER                      (0x00000010U)
    #define SPI_ERROR_WAITING_FOR_FLAG                  (0x00000020U)
    #define SPI_ERROR_DURING_ABORT                      (0x00000040U)
    #define SPI_ERROR_CALLBACK_INVALID                  (0x00000080U)
#endif

#define SPI_FLAG_RX_BUFFER_NOT_EMPTY                SPI_SR_RXNE   /* SPI status flag: Rx buffer not empty flag       */
#define SPI_FLAG_TX_BUFFER_EMPTY                    SPI_SR_TXE    /* SPI status flag: Tx buffer empty flag           */
#define SPI_FLAG_BUSY                               SPI_SR_BSY    /* SPI status flag: Busy flag                      */
#define SPI_FLAG_CRC_ERROR                          SPI_SR_CRCERR /* SPI Error flag: CRC error flag                  */
#define SPI_FLAG_MODE_FAULT                         SPI_SR_MODF   /* SPI Error flag: Mode fault flag                 */
#define SPI_FLAG_OVERRUN                            SPI_SR_OVR    /* SPI Error flag: Overrun flag                    */
#define SPI_FLAG_TI_MODE_FRAME_FORMAT_ERROR         SPI_SR_FRE    /* SPI Error flag: TI mode frame format error flag */
#define SPI_FLAG_BIT_MASK                           (SPI_SR_RXNE | SPI_SR_TXE | SPI_SR_BSY | SPI_SR_CRCERR\
                                                   | SPI_SR_MODF | SPI_SR_OVR | SPI_SR_FRE)

#define SPI_MODE_PERIPHERAL                         (0x00000000U)
#define SPI_MODE_CONTROLLER                         (SPI_CR1_MSTR | SPI_CR1_SSI)

#define SPI_DIRECTION_2_LINE                        (0x00000000U)
#define SPI_DIRECTION_2_LINE_RX_ONLY                SPI_CR1_RXONLY
#define SPI_DIRECTION_1_LINE                        SPI_CR1_BIDIMODE
#define SPI_DATA_SIZE_8_BIT                         (0x00000000U)
#define SPI_DATA_SIZE_16_BIT                        SPI_CR1_DFF
#define SPI_DATA_MSB_FIRST                          (0x00000000U)
#define SPI_DATA_LSB_FIRST                          SPI_CR1_LSBFIRST
#define SPI_CLOCK_POLARITY_LOW                      (0x00000000U)
#define SPI_CLOCK_POLARITY_HIGH                     SPI_CR1_CPOL
#define SPI_CLOCK_PHASE_LEADING_EDGE                (0x00000000U)
#define SPI_CLOCK_PHASE_TRAILING_EDGE               SPI_CR1_CPHA
#define SPI_CHIP_SELECT_SOFTWARE                    SPI_CR1_SSM
#define SPI_CHIP_SELECT_HARDWARE_INPUT              (0x00000000U)
#define SPI_CHIP_SELECT_HARDWARE_OUTPUT             (SPI_CR2_SSOE << 16U)

#define SPI_BAUD_RATE_PRESCALER_2                   (0x00000000U)
#define SPI_BAUD_RATE_PRESCALER_4                   (SPI_CR1_BR_0)
#define SPI_BAUD_RATE_PRESCALER_8                   (SPI_CR1_BR_1)
#define SPI_BAUD_RATE_PRESCALER_16                  (SPI_CR1_BR_1 | SPI_CR1_BR_0)
#define SPI_BAUD_RATE_PRESCALER_32                  (SPI_CR1_BR_2)
#define SPI_BAUD_RATE_PRESCALER_64                  (SPI_CR1_BR_2 | SPI_CR1_BR_0)
#define SPI_BAUD_RATE_PRESCALER_128                 (SPI_CR1_BR_2 | SPI_CR1_BR_1)
#define SPI_BAUD_RATE_PRESCALER_256                 (SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0)

#define SPI_TI_MODE_DISABLE                         (0x00000000U)
#define SPI_TI_MODE_ENABLE                          SPI_CR2_FRF

#define SPI_CRC_CALCULATION_DISABLE                 (0x00000000U)
#define SPI_CRC_CALCULATION_ENABLE                  SPI_CR1_CRCEN

#define SPI_TX_BUFFER_EMPTY_INTERRUPT_ENABLE        SPI_CR2_TXEIE
#define SPI_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE    SPI_CR2_RXNEIE
#define SPI_ERROR_INTERRUPT_ENABLE                  SPI_CR2_ERRIE

#define SPI_DEFAULT_TIMEOUT_100_US 100U
#define SPI_BUSY_FLAG_WORK_AROUND_TIMEOUT_1000_US 1000U /*!< flag_timeout 1000 µs             */

HAL_StatusTypeDef spi::initialize_module()
{
    if (spi_module_handle == nullptr)
        return HAL_ERROR;
    assert_param(IS_SPI_ALL_INSTANCE(spi_module_handle->Instance));
    assert_param(IS_SPI_MODE(spi_module_handle->Init.Mode));
    assert_param(IS_SPI_DIRECTION(spi_module_handle->Init.Direction));
    assert_param(IS_SPI_DATASIZE(spi_module_handle->Init.DataSize));
    assert_param(IS_SPI_NSS(spi_module_handle->Init.NSS));
    assert_param(IS_SPI_BAUDRATE_PRESCALER(spi_module_handle->Init.BaudRatePrescaler));
    assert_param(IS_SPI_FIRST_BIT(spi_module_handle->Init.FirstBit));
    assert_param(IS_SPI_TIMODE(spi_module_handle->Init.TIMode));
    if (spi_module_handle->Init.TIMode == SPI_TIMODE_DISABLE)
    {
        assert_param(IS_SPI_CPOL(spi_module_handle->Init.CLKPolarity));
        assert_param(IS_SPI_CPHA(spi_module_handle->Init.CLKPhase));
        if (spi_module_handle->Init.Mode == SPI_MODE_MASTER)
            assert_param(IS_SPI_BAUDRATE_PRESCALER(spi_module_handle->Init.BaudRatePrescaler));
        else
            spi_module_handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    }
    else
    {
        assert_param(IS_SPI_BAUDRATE_PRESCALER(spi_module_handle->Init.BaudRatePrescaler));
        spi_module_handle->Init.CLKPolarity = SPI_POLARITY_LOW;
        spi_module_handle->Init.CLKPhase    = SPI_PHASE_1EDGE;
    }
    #if (USE_SPI_CRC != 0U)
        assert_param(IS_SPI_CRC_CALCULATION(hspi->Init.CRCCalculation));
        if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
            assert_param(IS_SPI_CRC_POLYNOMIAL(hspi->Init.CRCPolynomial));
    #else
        spi_module_handle->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    #endif /* USE_SPI_CRC */

    if (spi_module_handle->State == HAL_SPI_STATE_RESET)
    {
        spi_module_handle->Lock = HAL_UNLOCKED;
        #if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
            spi_module_handle->TxCpltCallback       = reinterpret_cast<void (*)(__SPI_HandleTypeDef *)>(HAL_SPI_TxCpltCallback);
            spi_module_handle->RxCpltCallback       = reinterpret_cast<void (*)(__SPI_HandleTypeDef *)>(HAL_SPI_RxCpltCallback);
            spi_module_handle->TxRxCpltCallback     = reinterpret_cast<void (*)(__SPI_HandleTypeDef *)>(HAL_SPI_TxRxCpltCallback);
            spi_module_handle->TxHalfCpltCallback   = reinterpret_cast<void (*)(__SPI_HandleTypeDef *)>(HAL_SPI_TxHalfCpltCallback);
            spi_module_handle->RxHalfCpltCallback   = reinterpret_cast<void (*)(__SPI_HandleTypeDef *)>(HAL_SPI_RxHalfCpltCallback);
            spi_module_handle->TxRxHalfCpltCallback = reinterpret_cast<void (*)(__SPI_HandleTypeDef *)>(HAL_SPI_TxRxHalfCpltCallback);
            spi_module_handle->ErrorCallback        = reinterpret_cast<void (*)(__SPI_HandleTypeDef *)>(HAL_SPI_ErrorCallback);
            spi_module_handle->AbortCpltCallback    = reinterpret_cast<void (*)(__SPI_HandleTypeDef *)>(HAL_SPI_AbortCpltCallback);

            if (spi_module_handle->MspInitCallback == nullptr)
                spi_module_handle->MspInitCallback = reinterpret_cast<void (*)(__SPI_HandleTypeDef *)>(HAL_SPI_MspInit);
            spi_module_handle->MspInitCallback(spi_module_handle);
        #else
            /* Init the low level hardware : GPIO, CLOCK, NVIC... */
            HAL_SPI_MspInit(hspi);
        #endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
    }

    spi_module_handle->State = HAL_SPI_STATE_BUSY;
    __HAL_SPI_DISABLE(spi_module_handle);
    /*----------------------- SPIx CR1 & CR2 Configuration ---------------------*/
    /* Configure : SPI Mode, Communication Mode, Data size, Clock polarity and phase, NSS management,
    Communication speed, First bit and CRC calculation state */
    WRITE_REG(spi_module_handle->Instance->CR1, ((spi_module_handle->Init.Mode & (SPI_CR1_MSTR | SPI_CR1_SSI)) |
                                    (spi_module_handle->Init.Direction & (SPI_CR1_RXONLY | SPI_CR1_BIDIMODE)) |
                                    (spi_module_handle->Init.DataSize & SPI_CR1_DFF) |
                                    (spi_module_handle->Init.CLKPolarity & SPI_CR1_CPOL) |
                                    (spi_module_handle->Init.CLKPhase & SPI_CR1_CPHA) |
                                    (spi_module_handle->Init.NSS & SPI_CR1_SSM) |
                                    (spi_module_handle->Init.BaudRatePrescaler & SPI_CR1_BR_Msk) |
                                    (spi_module_handle->Init.FirstBit  & SPI_CR1_LSBFIRST) |
                                    (spi_module_handle->Init.CRCCalculation & SPI_CR1_CRCEN)));

    /* Configure : NSS management, TI Mode */
    WRITE_REG(spi_module_handle->Instance->CR2, (((spi_module_handle->Init.NSS >> 16U) & SPI_CR2_SSOE) | (spi_module_handle->Init.TIMode & SPI_CR2_FRF)));

    #if (USE_SPI_CRC != 0U)
        /*---------------------------- SPIx CRCPOLY Configuration ------------------*/
        /* Configure : CRC Polynomial */
        if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
        {
            WRITE_REG(hspi->Instance->CRCPR, (hspi->Init.CRCPolynomial & SPI_CRCPR_CRCPOLY_Msk));
        }
    #endif /* USE_SPI_CRC */

    #if defined(SPI_I2SCFGR_I2SMOD)
        /* Activate the SPI mode (Make sure that I2SMOD bit in I2SCFGR register is reset) */
        CLEAR_BIT(spi_module_handle->Instance->I2SCFGR, SPI_I2SCFGR_I2SMOD);
    #endif /* SPI_I2SCFGR_I2SMOD */

    spi_module_handle->ErrorCode = SPI_ERROR_NONE;
    spi_module_handle->State     = HAL_SPI_STATE_READY;

    return HAL_OK;
}

void spi::configure_module(SPI_HandleTypeDef* spi_handle)
{
    spi_module_handle = spi_handle;
    spi_module_handle->Instance = SPI2;
    spi_module_handle->Init.Mode = SPI_MODE_MASTER;
    spi_module_handle->Init.Direction = SPI_DIRECTION_2LINES;
    spi_module_handle->Init.DataSize = SPI_DATASIZE_8BIT;
    spi_module_handle->Init.CLKPolarity = SPI_POLARITY_HIGH;
    spi_module_handle->Init.CLKPhase = SPI_PHASE_2EDGE;
    spi_module_handle->Init.NSS = SPI_NSS_SOFT;
    spi_module_handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    spi_module_handle->Init.FirstBit = SPI_FIRSTBIT_MSB;
    spi_module_handle->Init.TIMode = SPI_TIMODE_DISABLE;
    spi_module_handle->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spi_module_handle->Init.CRCPolynomial = 10;
    if (initialize_module() != HAL_OK) { Error_Handler(); }
}

spi::spi_status_t spi::lock_module() const
{
    if (spi_module_handle->Lock == HAL_LOCKED) { return SPI_STATUS_BUSY; }
    spi_module_handle->Lock = HAL_LOCKED;
    return SPI_STATUS_OK;
}

void spi::unlock_module() const
{
    spi_module_handle->Lock = HAL_UNLOCKED;
}

uint8_t spi::get_module_communication_state() const
{
    return (uint8_t) spi_module_handle->State;
}

uint32_t spi::get_module_operating_mode() const
{
    return (uint32_t) spi_module_handle->Init.Mode;
}

void spi::verify_communication_direction(uint32_t intended_direction) const
{
    uint32_t set_direction = spi_module_handle->Init.Direction;
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

void spi::set_transaction_parameters(uint8_t *tx_data_ptr, uint8_t *rx_data_ptr, uint16_t packet_size) const
{
    spi_module_handle->ErrorCode   = SPI_ERROR_NONE;
    spi_module_handle->pTxBuffPtr  = (uint8_t *)tx_data_ptr;
    spi_module_handle->TxXferSize  = packet_size;
    spi_module_handle->TxXferCount = packet_size;
    spi_module_handle->pRxBuffPtr  = (uint8_t *)rx_data_ptr;
    spi_module_handle->RxXferSize  = packet_size;
    spi_module_handle->RxXferCount = packet_size;
}

void spi::reset_enabled_crc()
{
    #if (USE_SPI_CRC != 0U)
        if (spi_module_handle->Init.CRCCalculation == SPI_CRC_CALCULATION_ENABLE) { SPI_RESET_CRC(spi_module_handle); }
    #endif
}

HAL_StatusTypeDef spi::wait_for_flag_until_timeout(uint32_t flag, FlagStatus flag_status, uint32_t flag_timeout, uint32_t start_time_from_hal_get_tick)
{
    __IO uint32_t count;
    uint32_t adjusted_timeout;
    uint32_t start_time;

    adjusted_timeout    = flag_timeout - (HAL_GetTick() - start_time_from_hal_get_tick);    // adjust timeout value in case of end of transfer
    start_time          = HAL_GetTick();
    count               = adjusted_timeout * ((SystemCoreClock * 32U) >> 20U);              // calculate timeout based on software loop

    while ((SPI_GET_FLAG(spi_module_handle, flag) ? SET : RESET) != flag_status)
    {
        if (flag_timeout != HAL_MAX_DELAY)
        {
            if (((HAL_GetTick() - start_time) >= adjusted_timeout) || (adjusted_timeout == 0U))
            {
                SPI_DISABLE_INTERRUPTS(spi_module_handle, (SPI_TX_BUFFER_EMPTY_INTERRUPT_ENABLE | SPI_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE | SPI_ERROR_INTERRUPT_ENABLE));
                if ((spi_module_handle->Init.Mode == SPI_MODE_CONTROLLER) && ((spi_module_handle->Init.Direction == SPI_DIRECTION_1_LINE) || (spi_module_handle->Init.Direction == SPI_DIRECTION_2_LINE_RX_ONLY)))
                    SPI_DISABLE_MODULE(spi_module_handle);
                if (spi_module_handle->Init.CRCCalculation == SPI_CRC_CALCULATION_ENABLE)
                    SPI_RESET_CRC(spi_module_handle);
                spi_module_handle->State = HAL_SPI_STATE_READY;
                HAL_UNLOCK_MODULE(spi_module_handle);
                return HAL_TIMEOUT;
            }
            /* If Systick is disabled or not incremented, deactivate timeout to go in disable loop procedure */
            if (count == 0U) { adjusted_timeout = 0U; }
            count--;
        }
    }
    return HAL_OK;
}

HAL_StatusTypeDef spi::end_rx_transaction(uint32_t flag_timeout, uint32_t start_time_from_hal_get_tick)
{
    if ((spi_module_handle->Init.Mode == SPI_MODE_MASTER) && ((spi_module_handle->Init.Direction == SPI_DIRECTION_1_LINE) || (spi_module_handle->Init.Direction == SPI_DIRECTION_2_LINE_RX_ONLY)))
        SPI_DISABLE_MODULE(spi_module_handle);
    if (spi_module_handle->Init.Mode == SPI_MODE_MASTER)
    {
        if (spi_module_handle->Init.Direction != SPI_DIRECTION_2_LINE_RX_ONLY)
        {
            if (wait_for_flag_until_timeout(SPI_FLAG_BSY, RESET, flag_timeout, start_time_from_hal_get_tick) != HAL_OK)
            {
                HAL_SET_BIT(spi_module_handle->ErrorCode, SPI_ERROR_WAITING_FOR_FLAG);
                return HAL_TIMEOUT;
            }
        }
        else
        {
            if (wait_for_flag_until_timeout(SPI_FLAG_RXNE, RESET, flag_timeout, start_time_from_hal_get_tick) != HAL_OK)
            {
                HAL_SET_BIT(spi_module_handle->ErrorCode, SPI_ERROR_WAITING_FOR_FLAG);
                return HAL_TIMEOUT;
            }
        }
    }
    else
    {
        if (wait_for_flag_until_timeout(SPI_FLAG_RXNE, RESET, flag_timeout, start_time_from_hal_get_tick) != HAL_OK)
        {
            HAL_SET_BIT(spi_module_handle->ErrorCode, SPI_ERROR_WAITING_FOR_FLAG);
            return HAL_TIMEOUT;
        }
    }
    return HAL_OK;
}

HAL_StatusTypeDef spi::end_rx_tx_transaction(uint32_t flag_timeout, uint32_t start_time_from_hal_get_tick)
{
    __IO uint32_t count = SPI_BUSY_FLAG_WORK_AROUND_TIMEOUT_1000_US * (SystemCoreClock / 24U / 1000000U);
    if (spi_module_handle->Init.Mode == SPI_MODE_CONTROLLER)
    {
        if (wait_for_flag_until_timeout(SPI_FLAG_BUSY, RESET, flag_timeout, start_time_from_hal_get_tick) != HAL_OK)
        {
            HAL_SET_BIT(spi_module_handle->ErrorCode, SPI_ERROR_WAITING_FOR_FLAG);
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
            HAL_SET_BIT(spi_module_handle->ErrorCode, SPI_ERROR_WAITING_FOR_FLAG);
            break;
        }
        count--;
    }   while ((spi_module_handle->Instance->SR & SPI_FLAG_TX_BUFFER_EMPTY) == RESET);

    if (end_rx_tx_transaction(SPI_DEFAULT_TIMEOUT_100_US, tickstart) != HAL_OK)
        HAL_SET_BIT(spi_module_handle->ErrorCode, SPI_ERROR_WAITING_FOR_FLAG);
    if (spi_module_handle->Init.Direction == SPI_DIRECTION_2_LINE)
        SPI_CLEAR_OVERRUN_FLAG(spi_module_handle);

    #if (USE_SPI_CRC != 0U)
        if (SPI_GET_FLAG(spi_module_handle, SPI_FLAG_CRC_ERROR) != RESET)
        {
            spi_module_handle->State = SPI_STATE_READY;
            HAL_SET_BIT(spi_module_handle->ErrorCode, SPI_ERROR_DURING_CRC_CALCULATION);
            __HAL_SPI_CLEAR_CRCERRFLAG(spi_module_handle);
            #if (HAL_SPI_USE_REGISTER_CALLBACKS == 1U)
                spi_module_handle->ErrorCallback(spi_module_handle);
            #else
                HAL_SPI_ErrorCallback(spi_module_handle);
            #endif /* HAL_SPI_USE_REGISTER_CALLBACKS */
        }
        else
        {
    #endif /* USE_SPI_CRC */
            if (spi_module_handle->ErrorCode == SPI_ERROR_NONE)
            {
                if (spi_module_handle->State == HAL_SPI_STATE_BUSY_RX)
                {
                    spi_module_handle->State = HAL_SPI_STATE_READY;
                    #if (HAL_SPI_USE_REGISTER_CALLBACKS == 1U)
                        spi_module_handle->RxCpltCallback(spi_module_handle);
                    #else
                        HAL_SPI_RxCpltCallback(spi_module_handle);
                    #endif /* HAL_SPI_USE_REGISTER_CALLBACKS */
                }
                else
                {
                    spi_module_handle->State = HAL_SPI_STATE_READY;
                    #if (HAL_SPI_USE_REGISTER_CALLBACKS == 1U)
                        spi_module_handle->TxRxCpltCallback(spi_module_handle);
                    #else
                        HAL_SPI_TxRxCpltCallback(spi_module_handle);
                    #endif /* HAL_SPI_USE_REGISTER_CALLBACKS */
                }
            }
            else
            {
                spi_module_handle->State = HAL_SPI_STATE_READY;
                #if (HAL_SPI_USE_REGISTER_CALLBACKS == 1U)
                    spi_module_handle->ErrorCallback(spi_module_handle);
                #else
                    HAL_SPI_ErrorCallback(spi_module_handle);
                #endif /* HAL_SPI_USE_REGISTER_CALLBACKS */
            }
    #if (USE_SPI_CRC != 0U)
        }
    #endif /* USE_SPI_CRC */
}

#if (USE_SPI_CRC != 0U)
    static void rx_2_line_16_bit_isrCRC(struct __SPI_HandleTypeDef *spi_module_handle)
    {
        __IO uint32_t tmpreg = 0U;
        tmpreg = HAL_READ_REG(spi_module_handle->Instance->DR);      // read 16-bit crc to flush data register
        HAL_UNUSED(tmpreg);
        SPI_DISABLE_INTERRUPTS(spi_module_handle, SPI_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE);
        close_rx_tx_isr(spi_module_handle);
    }
#endif /* USE_SPI_CRC */


#if (USE_SPI_CRC != 0U)
    static void rx_2_line_8_bit_isrCRC(struct __SPI_HandleTypeDef *spi_module_handle)
    {
        __IO uint8_t  *ptmpreg8;
        __IO uint8_t  tmpreg8 = 0;

        ptmpreg8 = (__IO uint8_t *)&spi_module_handle->Instance->DR;
        tmpreg8 = *ptmpreg8;        // read crc to flush data register
        HAL_UNUSED(tmpreg8);
        SPI_DISABLE_INTERRUPTS(spi_module_handle, (SPI_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE | SPI_ERROR_INTERRUPT_ENABLE));
        if (spi_module_handle->TxXferCount == 0U)
            close_rx_tx_isr(spi_module_handle);
    }
#endif /* USE_SPI_CRC */

HAL_StatusTypeDef spi::spi_transmit_receive_interrupt(uint8_t *tx_data_ptr, uint8_t *rx_data_ptr, uint16_t packet_size)
{
    uint8_t spi_procedure_error = SPI_PROCEDURE_ERROR_NONE;
    uint32_t spi_module_mode;
    HAL_SPI_StateTypeDef spi_module_state;
    spi_module_state = (HAL_SPI_StateTypeDef) get_module_communication_state();
    spi_module_mode = get_module_operating_mode();

    if ((tx_data_ptr == nullptr) || (rx_data_ptr == nullptr) || (packet_size == 0U))
        spi_procedure_error = SPI_PROCEDURE_STATE_DATA_ERROR;

    assert_param(IS_SPI_DIRECTION_2LINES(spi_module_handle->Init.Direction));
    verify_communication_direction(SPI_DIRECTION_2_LINE);
    lock_module();
    if (!((spi_module_state == HAL_SPI_STATE_READY) || ((spi_module_mode == SPI_MODE_CONTROLLER) && (spi_module_handle->Init.Direction == SPI_DIRECTION_2_LINE) && (spi_module_state == HAL_SPI_STATE_BUSY_RX))))
        spi_procedure_error = SPI_PROCEDURE_STATE_BUS_ERROR;
    else if (spi_module_handle->State != HAL_SPI_STATE_BUSY_RX)
        spi_module_handle->State = HAL_SPI_STATE_BUSY_TX_RX;

    set_transaction_parameters(tx_data_ptr, rx_data_ptr, packet_size);
    set_rx_and_tx_interrupt_service_routines();
    reset_enabled_crc();

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    SPI_ENABLE_INTERRUPTS(spi_module_handle, (SPI_TX_BUFFER_EMPTY_INTERRUPT_ENABLE | SPI_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE | SPI_ERROR_INTERRUPT_ENABLE));
    if ((spi_module_handle->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
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
        HAL_SET_BIT(spi_module_handle->ErrorCode, SPI_ERROR_WAITING_FOR_FLAG);
    if (spi_module_handle->Init.Direction == SPI_DIRECTION_2_LINE)
        SPI_CLEAR_OVERRUN_FLAG(spi_module_handle);
    spi_module_handle->State = HAL_SPI_STATE_READY;

    #if (USE_SPI_CRC != 0U)
        if (SPI_GET_FLAG(spi_module_handle, SPI_FLAG_CRC_ERROR) != RESET)
        {
            HAL_SET_BIT(spi_module_handle->ErrorCode, SPI_ERROR_DURING_CRC_CALCULATION);
            __HAL_SPI_CLEAR_CRCERRFLAG(spi_module_handle);
            #if (HAL_SPI_USE_REGISTER_CALLBACKS == 1U)
                spi_module_handle->ErrorCallback(spi_module_handle);
            #else
                HAL_SPI_ErrorCallback(spi_module_handle);
            #endif /* HAL_SPI_USE_REGISTER_CALLBACKS */
        }
        else
        {
    #endif /* USE_SPI_CRC */
            if (spi_module_handle->ErrorCode == SPI_ERROR_NONE)
            {
                #if (HAL_SPI_USE_REGISTER_CALLBACKS == 1U)
                    spi_module_handle->RxCpltCallback(spi_module_handle);
                #else
                    HAL_SPI_RxCpltCallback(spi_module_handle);
                #endif /* HAL_SPI_USE_REGISTER_CALLBACKS */
            }
            else
            {
                #if (HAL_SPI_USE_REGISTER_CALLBACKS == 1U)
                    spi_module_handle->ErrorCallback(spi_module_handle);
                #else
                    HAL_SPI_ErrorCallback(spi_module_handle);
                #endif /* HAL_SPI_USE_REGISTER_CALLBACKS */
            }
    #if (USE_SPI_CRC != 0U)
        }
    #endif /* USE_SPI_CRC */
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
            HAL_SET_BIT(spi_module_handle->ErrorCode, SPI_ERROR_WAITING_FOR_FLAG);
            break;
        }
        count--;
    }   while ((spi_module_handle->Instance->SR & SPI_FLAG_TX_BUFFER_EMPTY) == RESET);

    SPI_DISABLE_INTERRUPTS(spi_module_handle, (SPI_TX_BUFFER_EMPTY_INTERRUPT_ENABLE | SPI_ERROR_INTERRUPT_ENABLE));
    if (end_rx_tx_transaction(SPI_DEFAULT_TIMEOUT_100_US, tickstart) != HAL_OK)
        HAL_SET_BIT(spi_module_handle->ErrorCode, SPI_ERROR_WAITING_FOR_FLAG);
    if (spi_module_handle->Init.Direction == SPI_DIRECTION_2_LINE)
        SPI_CLEAR_OVERRUN_FLAG(spi_module_handle);
    spi_module_handle->State = HAL_SPI_STATE_READY;
    if (spi_module_handle->ErrorCode != SPI_ERROR_NONE)
    {
        #if (HAL_SPI_USE_REGISTER_CALLBACKS == 1U)
            spi_module_handle->ErrorCallback(spi_module_handle);
        #else
            HAL_SPI_ErrorCallback(spi_module_handle);
        #endif /* HAL_SPI_USE_REGISTER_CALLBACKS */
    }
    else
    {
        #if (HAL_SPI_USE_REGISTER_CALLBACKS == 1U)
            spi_module_handle->TxCpltCallback(spi_module_handle);
        #else
            HAL_SPI_TxCpltCallback(spi_module_handle);
        #endif /* HAL_SPI_USE_REGISTER_CALLBACKS */
    }
}

void spi::abort_rx_isr()
{
    __IO uint32_t tmpreg = 0U;
    __IO uint32_t count = SPI_DEFAULT_TIMEOUT_100_US * (SystemCoreClock / 24U / 1000U);
    do
    {
        if (count == 0U)
        {
            HAL_SET_BIT(spi_module_handle->ErrorCode, SPI_ERROR_DURING_ABORT );
            break;
        }
        count--;
    }   while ((spi_module_handle->Instance->SR & SPI_FLAG_TX_BUFFER_EMPTY) == RESET);

    SPI_DISABLE_MODULE(spi_module_handle);
    HAL_CLEAR_BIT(spi_module_handle->Instance->CR2, (SPI_TX_BUFFER_EMPTY_INTERRUPT_ENABLE | SPI_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE | SPI_ERROR_INTERRUPT_ENABLE));
    tmpreg = HAL_READ_REG(spi_module_handle->Instance->DR);  // flush data register
    HAL_UNUSED(tmpreg);                         // avoid compiler warning
    spi_module_handle->State = HAL_SPI_STATE_ABORT;
}

void spi::abort_tx_isr()
{
    HAL_CLEAR_BIT(spi_module_handle->Instance->CR2, (SPI_TX_BUFFER_EMPTY_INTERRUPT_ENABLE));
    SPI_DISABLE_MODULE(spi_module_handle);
    spi_module_handle->State = HAL_SPI_STATE_ABORT;
}

#if (HAL_SPI_USE_REGISTER_CALLBACKS == 1U)

    HAL_StatusTypeDef spi::spi_register_callback(spi_callback_id_t CallbackID, spi_callback_ptr_t pCallback)
    {
        HAL_StatusTypeDef status = HAL_OK;
        if (pCallback == nullptr)
        {
            spi_module_handle->ErrorCode |= SPI_ERROR_CALLBACK_INVALID;
            return HAL_ERROR;
        }
        HAL_LOCK_MODULE(spi_module_handle);

        if (HAL_SPI_STATE_READY == spi_module_handle->State)
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
                    HAL_SET_BIT(spi_module_handle->ErrorCode, SPI_ERROR_CALLBACK_INVALID);
                    status =  HAL_ERROR;
                break;
            }
        }
        else if (HAL_SPI_STATE_RESET == spi_module_handle->State)
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
                    HAL_SET_BIT(spi_module_handle->ErrorCode, SPI_ERROR_CALLBACK_INVALID);
                    status =  HAL_ERROR;
                    break;
            }
        }
        else
        {
            HAL_SET_BIT(spi_module_handle->ErrorCode, SPI_ERROR_CALLBACK_INVALID);
            status =  HAL_ERROR;
        }
        HAL_UNLOCK_MODULE(spi_module_handle);
        return status;
    }

    HAL_StatusTypeDef spi::spi_unregister_callback(HAL_SPI_CallbackIDTypeDef CallbackID)
    {
        HAL_StatusTypeDef status = HAL_OK;
        HAL_LOCK_MODULE(spi_module_handle);

        if (HAL_SPI_STATE_READY == spi_module_handle->State)
        {
            switch (CallbackID)
            {
                case HAL_SPI_TX_COMPLETE_CB_ID:
                    spi_module_handle->TxCpltCallback = reinterpret_cast<void (*)(__SPI_HandleTypeDef *)>(HAL_SPI_TxCpltCallback);
                    break;
                case HAL_SPI_RX_COMPLETE_CB_ID:
                    spi_module_handle->RxCpltCallback = reinterpret_cast<void (*)(__SPI_HandleTypeDef *)>(HAL_SPI_RxCpltCallback);
                    break;
                case HAL_SPI_TX_RX_COMPLETE_CB_ID:
                    spi_module_handle->TxRxCpltCallback = reinterpret_cast<void (*)(__SPI_HandleTypeDef *)>(HAL_SPI_TxRxCpltCallback);
                    break;
                case HAL_SPI_TX_HALF_COMPLETE_CB_ID:
                    spi_module_handle->TxHalfCpltCallback = reinterpret_cast<void (*)(__SPI_HandleTypeDef *)>(HAL_SPI_TxHalfCpltCallback);
                    break;
                case HAL_SPI_RX_HALF_COMPLETE_CB_ID:
                    spi_module_handle->RxHalfCpltCallback = reinterpret_cast<void (*)(__SPI_HandleTypeDef *)>(HAL_SPI_RxHalfCpltCallback);
                    break;
                case HAL_SPI_TX_RX_HALF_COMPLETE_CB_ID:
                    spi_module_handle->TxRxHalfCpltCallback = reinterpret_cast<void (*)(__SPI_HandleTypeDef *)>(HAL_SPI_TxRxHalfCpltCallback);
                    break;
                case HAL_SPI_ERROR_CB_ID:
                    spi_module_handle->ErrorCallback = reinterpret_cast<void (*)(__SPI_HandleTypeDef *)>(HAL_SPI_ErrorCallback);
                    break;
                case HAL_SPI_ABORT_CB_ID:
                    spi_module_handle->AbortCpltCallback = reinterpret_cast<void (*)(__SPI_HandleTypeDef *)>(HAL_SPI_AbortCpltCallback);
                    break;
                case HAL_SPI_MSPINIT_CB_ID:
                    spi_module_handle->MspInitCallback = reinterpret_cast<void (*)(__SPI_HandleTypeDef *)>(HAL_SPI_MspInit);
                    break;
                case HAL_SPI_MSPDEINIT_CB_ID:
                    spi_module_handle->MspDeInitCallback = reinterpret_cast<void (*)(__SPI_HandleTypeDef *)>(HAL_SPI_MspDeInit);
                    break;
                default :
                    HAL_SET_BIT(spi_module_handle->ErrorCode, SPI_ERROR_CALLBACK_INVALID);
                    status =  HAL_ERROR;
                    break;
            }
        }
        else if (HAL_SPI_STATE_RESET == spi_module_handle->State)
        {
            switch (CallbackID)
            {
                case HAL_SPI_MSPINIT_CB_ID :
                    spi_module_handle->MspInitCallback = reinterpret_cast<void (*)(__SPI_HandleTypeDef *)>(HAL_SPI_MspInit);
                    break;
                case HAL_SPI_MSPDEINIT_CB_ID :
                    spi_module_handle->MspDeInitCallback = reinterpret_cast<void (*)(__SPI_HandleTypeDef *)>(HAL_SPI_MspDeInit);
                    break;
                default :
                    HAL_SET_BIT(spi_module_handle->ErrorCode, SPI_ERROR_CALLBACK_INVALID);
                    status =  HAL_ERROR;
                    break;
            }
        }
        else
        {
            HAL_SET_BIT(spi_module_handle->ErrorCode, SPI_ERROR_CALLBACK_INVALID);
            status =  HAL_ERROR;
        }
        HAL_UNLOCK_MODULE(spi_module_handle);
        return status;
    }
#endif /* HAL_SPI_USE_REGISTER_CALLBACKS */

void spi_tx_2_line_8_bit_isr(spi spi_object, struct spi::__SPI_HandleTypeDef *spi_handle)
{
    *(__IO uint8_t *)&spi_handle->Instance->DR = (*spi_handle->pTxBuffPtr);
    spi_handle->pTxBuffPtr++;
    spi_handle->TxXferCount--;
    if (spi_handle->TxXferCount == 0U)
    {
        #if (USE_SPI_CRC != 0U)
                if (spi_module_handle->Init.CRCCalculation == SPI_CRC_CALCULATION_ENABLE)
                    {
                        HAL_SET_BIT(spi_handle->Instance->CR1, SPI_CR1_CRCNEXT);
                        SPI_DISABLE_INTERRUPTS(spi_handle, SPI_TX_BUFFER_EMPTY_INTERRUPT_ENABLE);
                        return;
                    }
        #endif /* USE_SPI_CRC */
        SPI_DISABLE_INTERRUPTS(spi_handle, SPI_TX_BUFFER_EMPTY_INTERRUPT_ENABLE);
        if (spi_handle->RxXferCount == 0U) { spi_object.close_rx_tx_isr(); }
    }
}

void spi_rx_2_line_8_bit_isr(spi spi_object, struct spi::__SPI_HandleTypeDef *spi_handle)
{
    *spi_handle->pRxBuffPtr = *((__IO uint8_t *)&spi_handle->Instance->DR);     // receive data in 8-bit mode
    spi_handle->pRxBuffPtr++;
    spi_handle->RxXferCount--;

    if (spi_handle->RxXferCount == 0U)        // check for end of receive
    {
        #if (USE_SPI_CRC != 0U)
            if (spi_handle->Init.CRCCalculation == SPI_CRC_CALCULATION_ENABLE)
            {
                spi_handle->RxISR =  rx_2_line_8_bit_isrCRC;
                return;
            }
        #endif /* USE_SPI_CRC */
        SPI_DISABLE_INTERRUPTS(spi_handle, (SPI_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE | SPI_ERROR_INTERRUPT_ENABLE));
        if (spi_handle->TxXferCount == 0U) { spi_object.close_rx_tx_isr(); }
    }
}

void spi_tx_2_line_16_bit_isr(spi spi_object, struct spi::__SPI_HandleTypeDef *spi_handle)
{
    /* Transmit data in 16 Bit mode */
    spi_handle->Instance->DR = *((uint16_t *)spi_handle->pTxBuffPtr);
    spi_handle->pTxBuffPtr += sizeof(uint16_t);
    spi_handle->TxXferCount--;

    if (spi_handle->TxXferCount == 0U)
    {
        #if (USE_SPI_CRC != 0U)
            if (spi_module_handle->Init.CRCCalculation == SPI_CRC_CALCULATION_ENABLE)
            {
                HAL_SET_BIT(spi_module_handle->Instance->CR1, SPI_CR1_CRCNEXT);      // set crc next bit to send crc
                SPI_DISABLE_INTERRUPTS(spi_module_handle, SPI_TX_BUFFER_EMPTY_INTERRUPT_ENABLE);
                return;
            }
        #endif /* USE_SPI_CRC */

        SPI_DISABLE_INTERRUPTS(spi_handle, SPI_TX_BUFFER_EMPTY_INTERRUPT_ENABLE);
        if (spi_handle->RxXferCount == 0U) { spi_object.close_rx_tx_isr(); }
    }
}

void spi_rx_2_line_16_bit_isr(spi spi_object, struct spi::__SPI_HandleTypeDef *spi_handle)
{
    *((uint16_t *)spi_handle->pRxBuffPtr) = (uint16_t)(spi_handle->Instance->DR); // receive data in 16-bit mode
    spi_handle->pRxBuffPtr += sizeof(uint16_t);
    spi_handle->RxXferCount--;

    if (spi_handle->RxXferCount == 0U)
    {
        #if (USE_SPI_CRC != 0U)
            if (spi_module_handle->Init.CRCCalculation == SPI_CRC_CALCULATION_ENABLE)
            {
                spi_module_handle->RxISR =  rx_2_line_16_bit_isrCRC;
                return;
            }
        #endif /* USE_SPI_CRC */

        SPI_DISABLE_INTERRUPTS(spi_handle, SPI_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE);
        if (spi_handle->TxXferCount == 0U) { spi_object.close_rx_tx_isr(); }
    }
}

void SPI_DMAAbortOnError(DMA_HandleTypeDef *hdma)
{
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent); /* Derogation MISRAC2012-Rule-11.5 */
    hspi->RxXferCount = 0U;
    hspi->TxXferCount = 0U;

    /* Call user error callback */
    #if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
        hspi->ErrorCallback(hspi);
    #else
        HAL_SPI_ErrorCallback(hspi);
    #endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
}




void spi::set_rx_and_tx_interrupt_service_routines() const
{
    if (spi_module_handle->Init.DataSize == SPI_DATA_SIZE_8_BIT)
    {
        spi_module_handle->RxISR     = spi_rx_2_line_8_bit_isr;
        spi_module_handle->TxISR     = spi_tx_2_line_8_bit_isr;
    }
    else
    {
        spi_module_handle->RxISR     = spi_rx_2_line_16_bit_isr;
        spi_module_handle->TxISR     = spi_tx_2_line_16_bit_isr;
    }
}

void spi_irq_handler(spi* spi_object)
{
    uint32_t itsource = spi_object->spi_module_handle->Instance->CR2;
    uint32_t itflag   = spi_object->spi_module_handle->Instance->SR;
    // receiving
    if ((SPI_CHECK_FLAG(itflag, SPI_FLAG_OVR) == RESET) &&
        (SPI_CHECK_FLAG(itflag, SPI_FLAG_RXNE) != RESET) && (SPI_CHECK_IT_SOURCE(itsource, SPI_IT_RXNE) != RESET))
    {
        spi_object->spi_module_handle->RxISR(*spi_object, spi_object->spi_module_handle);
        return;
    }
    // sending
    if ((SPI_CHECK_FLAG(itflag, SPI_FLAG_TXE) != RESET) && (SPI_CHECK_IT_SOURCE(itsource, SPI_IT_TXE) != RESET))
    {
        spi_object->spi_module_handle->TxISR(*spi_object, spi_object->spi_module_handle);
        return;
    }
    // error handling
    if (((SPI_CHECK_FLAG(itflag, SPI_FLAG_MODF) != RESET) || (SPI_CHECK_FLAG(itflag, SPI_FLAG_OVR) != RESET)
         || (SPI_CHECK_FLAG(itflag, SPI_FLAG_FRE) != RESET)) && (SPI_CHECK_IT_SOURCE(itsource, SPI_IT_ERR) != RESET))
    {
        if (SPI_CHECK_FLAG(itflag, SPI_FLAG_OVR) != RESET)
        {
            if (spi_object->spi_module_handle->State != HAL_SPI_STATE_BUSY_TX)
            {
                SET_BIT(spi_object->spi_module_handle->ErrorCode, SPI_ERROR_OVERRUN);
                __HAL_SPI_CLEAR_OVRFLAG(spi_object->spi_module_handle);
            }
            else
            {
                __HAL_SPI_CLEAR_OVRFLAG(spi_object->spi_module_handle);
                return;
            }
        }
        if (SPI_CHECK_FLAG(itflag, SPI_FLAG_MODF) != RESET)
        {
            HAL_SET_BIT(spi_object->spi_module_handle->ErrorCode, SPI_ERROR_MODE_FAULT);
            SPI_CLEAR_MODE_FAULT_FLAG(spi_object->spi_module_handle);
        }
        if (SPI_CHECK_FLAG(itflag, SPI_FLAG_FRE) != RESET)
        {
            HAL_SET_BIT(spi_object->spi_module_handle->ErrorCode, SPI_ERROR_TI_MODE_FRAME_FORMAT);
            HAL_SPI_CLEAR_FORMAT_ERROR_FLAG(spi_object->spi_module_handle);
        }

        if (spi_object->spi_module_handle->ErrorCode != SPI_ERROR_NONE)
        {
            SPI_DISABLE_INTERRUPTS(spi_object->spi_module_handle, SPI_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE | SPI_TX_BUFFER_EMPTY_INTERRUPT_ENABLE | SPI_ERROR_INTERRUPT_ENABLE);

            spi_object->spi_module_handle->State = HAL_SPI_STATE_READY;
            if ((HAL_GENERAL_IS_BIT_SET(itsource, SPI_CR2_TXDMAEN)) || (HAL_IS_BIT_SET(itsource, SPI_CR2_RXDMAEN)))
            {
                HAL_CLEAR_BIT(spi_object->spi_module_handle->Instance->CR2, (SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN));

                if (spi_object->spi_module_handle->hdmarx != nullptr)
                {
                    spi_object->spi_module_handle->hdmarx->XferAbortCallback = SPI_DMAAbortOnError;
                    if (HAL_OK != HAL_DMA_Abort_IT(spi_object->spi_module_handle->hdmarx))
                    {
                        SET_BIT(spi_object->spi_module_handle->ErrorCode, SPI_ERROR_DURING_ABORT);
                    }
                }
                if (spi_object->spi_module_handle->hdmatx != nullptr)
                {
                    spi_object->spi_module_handle->hdmatx->XferAbortCallback = SPI_DMAAbortOnError;
                    if (HAL_OK != HAL_DMA_Abort_IT(spi_object->spi_module_handle->hdmatx))
                    {
                        SET_BIT(spi_object->spi_module_handle->ErrorCode, SPI_ERROR_DURING_ABORT);
                    }
                }
            }
            else
            {
                #if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
                    spi_object->spi_module_handle->ErrorCallback(spi_object->spi_module_handle);
                #else
                    HAL_SPI_ErrorCallback(hspi);
                #endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
            }
        }
        return;
    }
}
