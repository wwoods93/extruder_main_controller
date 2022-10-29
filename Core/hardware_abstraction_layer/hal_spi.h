/***********************************************************************************************************************
 * Main_Controller
 * hal_spi.h
 *
 * wilson
 * 10/16/22
 * 9:41 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

#ifndef MAIN_CONTROLLER_HAL_SPI_H
#define MAIN_CONTROLLER_HAL_SPI_H

#include "stm32f4xx.h"
#include "hal_general.h"

class spi
{
    public:

        #define SPI_USE_REGISTER_CALLBACKS                              1U
        #define SPI_USE_CRC                                             0U
        static constexpr uint8_t SPI_PROCEDURE_ERROR_NONE               = 0U;
        static constexpr uint8_t SPI_PROCEDURE_STATE_BUS_ERROR          = 1U;
        static constexpr uint8_t SPI_PROCEDURE_STATE_DATA_ERROR         = 2U;

        static constexpr uint32_t SPI_CR1_MODE_CONTROLLER               = SPI_CR1_MSTR;
        static constexpr uint32_t SPI_CR1_INTERNAL_CHIP_SELECT          = SPI_CR1_SSI;
        static constexpr uint32_t SPI_CR1_RECEIVE_ONLY                  = SPI_CR1_RXONLY;
        static constexpr uint32_t SPI_CR1_BIDIRECTIONAL_MODE            = SPI_CR1_BIDIMODE;
        static constexpr uint32_t SPI_CR1_DATA_FRAME_FORMAT             = SPI_CR1_DFF;
        static constexpr uint32_t SPI_CR1_CLOCK_POLARITY                = SPI_CR1_CPOL;
        static constexpr uint32_t SPI_CR1_CLOCK_PHASE                   = SPI_CR1_CPHA;
        static constexpr uint32_t SPI_CR1_SOFTWARE_CHIP_SELECT          = SPI_CR1_SSM;
        static constexpr uint32_t SPI_CR1_BAUD_RATE_CONTROL_MASK        = SPI_CR1_BR_Msk;
        static constexpr uint32_t SPI_CR1_LSB_FIRST                     = SPI_CR1_LSBFIRST;
        static constexpr uint32_t SPI_CR1_CRC_ENABLE                    = SPI_CR1_CRCEN;

        static constexpr uint32_t SPI_CR2_CHIP_SELECT_OUTPUT_ENABLE     = SPI_CR2_SSOE;
        static constexpr uint32_t SPI_CR2_FRAME_FORMAT                  = SPI_CR2_FRF;

        static constexpr uint8_t SPI_CR2_RX_BUFFER_DMA_ENABLE_POSITION  = (0U);
        static constexpr uint32_t SPI_CR2_RX_BUFFER_DMA_ENABLE_MASK     = 0x1UL << SPI_CR2_RX_BUFFER_DMA_ENABLE_POSITION;
        static constexpr uint32_t SPI_CR2_RX_BUFFER_DMA_ENABLE          = SPI_CR2_RX_BUFFER_DMA_ENABLE_MASK;
        static constexpr uint8_t SPI_CR2_TX_BUFFER_DMA_ENABLE_POSITION  = (1U);
        static constexpr uint32_t SPI_CR2_TX_BUFFER_DMA_ENABLE_MASK     = 0x1UL << SPI_CR2_TX_BUFFER_DMA_ENABLE_POSITION;
        static constexpr uint32_t SPI_CR2_TX_BUFFER_DMA_ENABLE          = SPI_CR2_TX_BUFFER_DMA_ENABLE_MASK;
        #if (SPI_USE_REGISTER_CALLBACKS == 1U)
            static constexpr uint8_t SPI_ERROR_NONE                     = (0x00000000U);
            static constexpr uint8_t SPI_ERROR_MODE_FAULT               = (0x00000001U);
            static constexpr uint8_t SPI_ERROR_DURING_CRC_CALCULATION   = (0x00000002U);
            static constexpr uint8_t SPI_ERROR_OVERRUN                  = (0x00000004U);
            static constexpr uint8_t SPI_ERROR_TI_MODE_FRAME_FORMAT     = (0x00000008U);
            static constexpr uint8_t SPI_ERROR_DMA_TRANSFER             = (0x00000010U);
            static constexpr uint8_t SPI_ERROR_WAITING_FOR_FLAG         = (0x00000020U);
            static constexpr uint8_t SPI_ERROR_DURING_ABORT             = (0x00000040U);
            static constexpr uint8_t SPI_ERROR_CALLBACK_INVALID         = (0x00000080U);
        #endif
        static constexpr uint32_t SPI_FLAG_RX_BUFFER_NOT_EMPTY          = SPI_SR_RXNE;
        static constexpr uint32_t SPI_FLAG_TX_BUFFER_EMPTY              = SPI_SR_TXE;
        static constexpr uint32_t SPI_FLAG_BUSY                         = SPI_SR_BSY;
        static constexpr uint32_t SPI_FLAG_CRC_ERROR                    = SPI_SR_CRCERR;
        static constexpr uint32_t SPI_FLAG_MODE_FAULT                   = SPI_SR_MODF;
        static constexpr uint32_t SPI_FLAG_OVERRUN                      = SPI_SR_OVR;
        static constexpr uint32_t SPI_FLAG_TI_MODE_FRAME_FORMAT_ERROR   = SPI_SR_FRE;
        static constexpr uint32_t SPI_FLAG_BIT_MASK                     = (SPI_SR_RXNE | SPI_SR_TXE | SPI_SR_BSY | SPI_SR_CRCERR \
                                                                         | SPI_SR_MODF | SPI_SR_OVR | SPI_SR_FRE);
        static constexpr uint32_t SPI_MODE_PERIPHERAL                   = (0x00000000U);
        static constexpr uint32_t SPI_MODE_CONTROLLER                   = (SPI_CR1_MSTR | SPI_CR1_SSI);
        static constexpr uint32_t SPI_DIRECTION_2_LINE                  = (0x00000000U);
        static constexpr uint32_t SPI_DIRECTION_2_LINE_RX_ONLY          = SPI_CR1_RXONLY;
        static constexpr uint32_t SPI_DIRECTION_1_LINE                  = SPI_CR1_BIDIMODE;
        static constexpr uint32_t SPI_DATA_SIZE_8_BIT                   = (0x00000000U);
        static constexpr uint32_t SPI_DATA_SIZE_16_BIT                  = SPI_CR1_DFF;
        static constexpr uint32_t SPI_DATA_MSB_FIRST                    = (0x00000000U);
        static constexpr uint32_t SPI_DATA_LSB_FIRST                    = SPI_CR1_LSBFIRST;
        static constexpr uint32_t SPI_CLOCK_POLARITY_LOW                = (0x00000000U);
        static constexpr uint32_t SPI_CLOCK_POLARITY_HIGH               = SPI_CR1_CPOL;
        static constexpr uint32_t SPI_CLOCK_PHASE_LEADING_EDGE          = (0x00000000U);
        static constexpr uint32_t SPI_CLOCK_PHASE_TRAILING_EDGE         = SPI_CR1_CPHA;
        static constexpr uint32_t SPI_CHIP_SELECT_SOFTWARE              = SPI_CR1_SSM;
        static constexpr uint32_t SPI_CHIP_SELECT_HARDWARE_INPUT        = (0x00000000U);
        static constexpr uint32_t SPI_CHIP_SELECT_HARDWARE_OUTPUT       = (SPI_CR2_SSOE << 16U);

        static constexpr uint32_t SPI_BAUD_RATE_PRESCALER_2             = (0x00000000U);
        static constexpr uint32_t SPI_BAUD_RATE_PRESCALER_4             = (SPI_CR1_BR_0);
        static constexpr uint32_t SPI_BAUD_RATE_PRESCALER_8             = (SPI_CR1_BR_1);
        static constexpr uint32_t SPI_BAUD_RATE_PRESCALER_16            = (SPI_CR1_BR_1 | SPI_CR1_BR_0);
        static constexpr uint32_t SPI_BAUD_RATE_PRESCALER_32            = (SPI_CR1_BR_2);
        static constexpr uint32_t SPI_BAUD_RATE_PRESCALER_64            = (SPI_CR1_BR_2 | SPI_CR1_BR_0);
        static constexpr uint32_t SPI_BAUD_RATE_PRESCALER_128           = (SPI_CR1_BR_2 | SPI_CR1_BR_1);
        static constexpr uint32_t SPI_BAUD_RATE_PRESCALER_256           = (SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0);

        static constexpr uint32_t SPI_TI_MODE_DISABLE                           = (0x00000000U);
        static constexpr uint32_t SPI_TI_MODE_ENABLE                            = SPI_CR2_FRF;
        static constexpr uint32_t SPI_TX_BUFFER_EMPTY_INTERRUPT_ENABLE          = SPI_CR2_TXEIE;
        static constexpr uint32_t SPI_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE      = SPI_CR2_RXNEIE;
        static constexpr uint32_t SPI_ERROR_INTERRUPT_ENABLE                    = SPI_CR2_ERRIE;
        static constexpr uint32_t SPI_DEFAULT_TIMEOUT_100_US                    = 100U;
        static constexpr uint32_t SPI_BUSY_FLAG_WORK_AROUND_TIMEOUT_1000_US     = 1000U;
        static constexpr uint32_t SPI_CRC_CALCULATION_DISABLE                   = (0x00000000U);
        static constexpr uint32_t SPI_CRC_CALCULATION_ENABLE                    = SPI_CR1_CRCEN;

        #define HAL_DMA_ENABLE(__HANDLE__)      ((__HANDLE__)->instance->CONFIG_REG |=  DMA_SxCR_EN)
        #define HAL_DMA_DISABLE(__HANDLE__)     ((__HANDLE__)->instance->CONFIG_REG &=  ~DMA_SxCR_EN)
        #define SPI_ENABLE_MODULE(__HANDLE__)  HAL_GENERAL_SET_BIT((__HANDLE__)->instance->CONTROL_REG_1, SPI_CR1_SPE)
        #define SPI_DISABLE_MODULE(__HANDLE__) HAL_GENERAL_CLEAR_BIT((__HANDLE__)->instance->CONTROL_REG_1, SPI_CR1_SPE)
        #define SPI_ENABLE_INTERRUPTS(__HANDLE__, __INTERRUPT__)   HAL_GENERAL_SET_BIT((__HANDLE__)->instance->CONTROL_REG_2, (__INTERRUPT__))
        #define SPI_DISABLE_INTERRUPTS(__HANDLE__, __INTERRUPT__)  HAL_GENERAL_CLEAR_BIT((__HANDLE__)->instance->CONTROL_REG_2, (__INTERRUPT__))
        #define SPI_GET_FLAG(__HANDLE__, __FLAG__) ((((__HANDLE__)->instance->STATUS_REG) & (__FLAG__)) == (__FLAG__))
        #define SPI_CHECK_FLAG(__SR__, __FLAG__) ((((__SR__) & ((__FLAG__) & SPI_FLAG_MASK)) == ((__FLAG__) & SPI_FLAG_MASK)) ? SET : RESET)
        #define SPI_VERIFY_DIRECTION_2_LINE(__MODE__) ((__MODE__) == SPI_DIRECTION_2_LINE)
        #define SPI_VERIFY_DIRECTION_2_LINE_RX_ONLY(__MODE__) ((__MODE__) == SPI_DIRECTION_2_LINE_RX_ONLY)
        #define SPI_VERIFY_DIRECTION_1_LINE(__MODE__) ((__MODE__) == SPI_DIRECTION_1_LINE)
        #define SPI_CLEAR_OVERRUN_FLAG(__HANDLE__)                                                  \
            do {                                                                                    \
                __IO uint32_t tmpreg_ovr = 0x00U;                                                   \
                tmpreg_ovr = (__HANDLE__)->instance->DATA_REG;                                      \
                tmpreg_ovr = (__HANDLE__)->instance->STATUS_REG;                                    \
                HAL_GENERAL_UNUSED(tmpreg_ovr);                                                     \
            }   while(0U)

        #define SPI_CLEAR_MODE_FAULT_FLAG(__HANDLE__)                                               \
            do {                                                                                    \
                __IO uint32_t tmpreg_modf = 0x00U;                                                  \
                tmpreg_modf = (__HANDLE__)->instance->STATUS_REG;                                   \
                HAL_GENERAL_CLEAR_BIT((__HANDLE__)->instance->CONTROL_REG_1, SPI_CR1_SPE);          \
                HAL_GENERAL_UNUSED(tmpreg_modf);                                                    \
            }   while(0U)

        #define HAL_SPI_CLEAR_FORMAT_ERROR_FLAG(__HANDLE__)                                         \
            do {                                                                                    \
                __IO uint32_t tmpreg_fre = 0x00U;                                                   \
                tmpreg_fre = (__HANDLE__)->instance->STATUS_REG;                                    \
                HAL_GENERAL_UNUSED(tmpreg_fre);                                                     \
            }   while(0U)

        #define SPI_CHECK_IT_SOURCE(__CR2__, __INTERRUPT__) ((((__CR2__) & (__INTERRUPT__)) ==      \
                                                     (__INTERRUPT__)) ? SET : RESET)

        #define SPI_RESET_CRC_CALCULATION(__HANDLE__)                                               \
            do{HAL_GENERAL_CLEAR_BIT((__HANDLE__)->instance->CONTROL_REG_1, SPI_CR1_CRCEN);         \
            HAL_GENERAL_SET_BIT((__HANDLE__)->instance->CONTROL_REG_1, SPI_CR1_CRCEN);}while(0U)
        /*************************************************** dma ******************************************************/
        typedef struct
        {
            uint32_t channel;                               // DMA_Channel_selection
            uint32_t direction;                             // DMA_Data_transfer_direction
            uint32_t peripheral_address_reg_increment;      // DMA_Peripheral_incremented_mode
            uint32_t memory_address_reg_increment;          // DMA_Memory_incremented_mode
            uint32_t peripheral_data_width;                 // DMA_Peripheral_data_size
            uint32_t memory_data_width;                     // DMA_Memory_data_size
            uint32_t mode;                                  // DMA_mode
            uint32_t priority;                              // DMA_Priority_level
            uint32_t FIFOMode;                              // DMA_FIFO_direct_mode
            uint32_t fifo_threshold_level;                  // DMA_FIFO_threshold_level
            uint32_t memory_burst_transfer_config;          // DMA_Memory_burst
            uint32_t peripheral_burst_transfer_config;      // DMA_Peripheral_burst
        }dma_init_t;

        typedef enum
        {
            DMA_STATE_RESET             = 0x00U,
            DMA_STATE_READY             = 0x01U,
            DMA_STATE_BUSY              = 0x02U,
            DMA_STATE_TIMEOUT           = 0x03U,
            DMA_STATE_ERROR             = 0x04U, 
            DMA_STATE_ABORT             = 0x05U,
        }dma_state_t;

        typedef struct _dma_handle_t
        {
            dma_stream_t                *instance;
            dma_init_t                  init;
            hal_lock_t                  lock;
            volatile dma_state_t        state;
            void                        *parent;
            void                        (* transfer_complete_callback)              ( struct _dma_handle_t * hdma);
            void                        (* transfer_half_complete_callback)         ( struct _dma_handle_t * hdma);
            void                        (* transfer_mem_1_complete_callback)        ( struct _dma_handle_t * hdma);
            void                        (* transfer_mem_1_half_complete_callback)   ( struct _dma_handle_t * hdma);
            void                        (* transfer_error_callback)                 ( struct _dma_handle_t * hdma);
            void                        (* transfer_abort_callback)                 ( struct _dma_handle_t * hdma);
            volatile uint32_t           error_code;
            uint32_t                    stream_base_address;
            uint32_t                    stream_index;
        }dma_handle_t;
        /**************************************************************************************************************/
        typedef enum
        {
            SPI_STATUS_OK       = 0x00U,
            SPI_STATUS_ERROR    = 0x01U,
            SPI_STATUS_BUSY     = 0x02U,
            SPI_STATUS_TIMEOUT  = 0x03U
        } spi_status_t;

        typedef struct
        {
            uint32_t mode;
            uint32_t direction;
            uint32_t data_size;
            uint32_t clock_polarity;
            uint32_t clock_phase;
            uint32_t chip_select_setting;
            uint32_t baud_rate_prescaler;
            uint32_t first_bit_setting;
            uint32_t ti_mode;
            uint32_t crc_calculation;
            uint32_t crc_polynomial;
        } spi_init_t;

        typedef enum
        {
            SPI_STATE_RESET         = 0x00U,
            SPI_STATE_READY         = 0x01U,
            SPI_STATE_BUSY          = 0x02U,
            SPI_STATE_BUSY_TX       = 0x03U,
            SPI_STATE_BUSY_RX       = 0x04U,
            SPI_STATE_BUSY_TX_RX    = 0x05U,
            SPI_STATE_ERROR         = 0x06U,
            SPI_STATE_ABORT         = 0x07U
        } spi_state_t;

        typedef enum
        {
            SPI_TX_COMPLETE_CALLBACK_ID         = 0x00U,
            SPI_RX_COMPLETE_CALLBACK_ID         = 0x01U,
            SPI_TX_RX_COMPLETE_CALLBACK_ID      = 0x02U,
            SPI_TX_HALF_COMPLETE_CALLBACK_ID    = 0x03U,
            SPI_RX_HALF_COMPLETE_CALLBACK_ID    = 0x04U,
            SPI_TX_RX_HALF_COMPLETE_CALLBACK_ID = 0x05U,
            SPI_ERROR_CALLBACK_ID               = 0x06U,
            SPI_ABORT_CALLBACK_ID               = 0x07U,
            SPI_MSP_INIT_CALLBACK_ID            = 0x08U,
            SPI_MSP_DEINIT_CALLBACK_ID          = 0x09U
        } spi_callback_id_t;

        typedef struct _spi_handle_t
        {
            hal_spi_t                   *instance;
            spi_init_t                  init;
            uint8_t                     *tx_buffer_pointer;
            uint16_t                    tx_transfer_size;
            volatile uint16_t           tx_transfer_counter;
            uint8_t                     *rx_buffer_pointer;
            uint16_t                    rx_transfer_size;
            volatile uint16_t           rx_transfer_counter;
            void                        (*rx_isr_pointer)(spi spi_object, struct _spi_handle_t *spi_handle);
            void                        (*tx_isr_pointer)(spi spi_object, struct _spi_handle_t *spi_handle);
            dma_handle_t                *tx_dma_handle;
            dma_handle_t                *rx_dma_handle;
            hal_lock_t                  lock;
            volatile spi_state_t        state;
            volatile uint32_t           error_code;

            #if (SPI_USE_REGISTER_CALLBACKS == 1U)
                void (* TxCpltCallback)(spi::_spi_handle_t *spi_handle);         //  SPI Tx Completed callback
                void (* RxCpltCallback)(struct _spi_handle_t *spi_handle);       // SPI Rx Completed callback
                void (* TxRxCpltCallback)(struct _spi_handle_t *spi_handle);     // SPI TxRx Completed callback
                void (* TxHalfCpltCallback)(struct _spi_handle_t *spi_handle);   // SPI Tx Half Completed callback
                void (* RxHalfCpltCallback)(struct _spi_handle_t *spi_handle);   // SPI Rx Half Completed callback
                void (* TxRxHalfCpltCallback)(struct _spi_handle_t *spi_handle); // SPI TxRx Half Completed callback
                void (* ErrorCallback)(struct _spi_handle_t *spi_handle);        // SPI Error callback
                void (* AbortCpltCallback)(struct _spi_handle_t *spi_handle);    // SPI Abort callback
                void (* MspInitCallback)(struct _spi_handle_t *spi_handle);      // SPI Msp init callback
                void (* MspDeInitCallback)(struct _spi_handle_t *spi_handle);    // SPI Msp de-init callback
            #endif  /* USE_HAL_SPI_REGISTER_CALLBACKS */
        } spi_handle_t;

        spi_handle_t* spi_module_handle;

        void configure_module(spi_handle_t* spi_handle);
        HAL_StatusTypeDef spi_transmit_receive_interrupt(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);
        #if (SPI_USE_REGISTER_CALLBACKS == 1U)
            typedef void (*spi_callback_pointer_t)(spi_handle_t* spi_module_handle);
            HAL_StatusTypeDef spi_register_callback(spi_callback_id_t CallbackID, spi_callback_pointer_t pCallback);
            HAL_StatusTypeDef spi_unregister_callback(HAL_SPI_CallbackIDTypeDef CallbackID);
        #endif /* SPI_USE_REGISTER_CALLBACKS */
        void close_rx_tx_isr();
        void set_rx_and_tx_interrupt_service_routines() const;
        friend void spi_rx_2_line_16_bit_isr(spi spi_object, struct spi::_spi_handle_t *spi_handle);
        friend void spi_tx_2_line_16_bit_isr(spi spi_object, struct spi::_spi_handle_t *spi_handle);
        friend void spi_rx_2_line_8_bit_isr(spi spi_object, struct spi::_spi_handle_t *spi_handle);
        friend void spi_tx_2_line_8_bit_isr(spi spi_object, struct spi::_spi_handle_t *spi_handle);
        friend void SPI_DMAAbortOnError(spi::dma_handle_t *hdma);
        friend spi::spi_status_t dma_abort_interrupt(spi::dma_handle_t *hdma);
        friend void spi_irq_handler(spi* spi_object);
        friend void HAL_SPI_TxCpltCallback(spi::spi_handle_t *spi_handle);
        friend void HAL_SPI_RxCpltCallback(spi::spi_handle_t *spi_handle);
        friend void HAL_SPI_TxRxCpltCallback(spi::spi_handle_t *spi_handle);
        friend void HAL_SPI_TxHalfCpltCallback(spi::spi_handle_t *spi_handle);
        friend void HAL_SPI_RxHalfCpltCallback(spi::spi_handle_t *spi_handle);
        friend void HAL_SPI_TxRxHalfCpltCallback(spi::spi_handle_t *spi_handle);
        friend void HAL_SPI_ErrorCallback(spi::spi_handle_t *spi_handle);
        friend void HAL_SPI_AbortCpltCallback(spi::spi_handle_t *spi_handle);

    private:

        HAL_StatusTypeDef initialize_module();
        spi_status_t lock_module() const;
        void unlock_module() const;
        uint8_t get_module_communication_state() const;
        uint32_t get_module_operating_mode() const;
        void verify_communication_direction(uint32_t intended_direction) const;
        void set_transaction_parameters(uint8_t *tx_data_pointer, uint8_t *rx_data_pointer, uint16_t packet_size) const;
        HAL_StatusTypeDef wait_for_flag_until_timeout(uint32_t flag, FlagStatus flag_status, uint32_t flag_timeout, uint32_t start_time_from_hal_get_tick);
        HAL_StatusTypeDef end_rx_transaction(uint32_t flag_timeout, uint32_t start_time_from_hal_get_tick);
        HAL_StatusTypeDef end_rx_tx_transaction(uint32_t flag_timeout, uint32_t start_time_from_hal_get_tick);
        void reset_enabled_crc();
        void close_rx_isr();
        void close_tx_isr();
        void abort_rx_isr();
        void abort_tx_isr();
};

#endif //MAIN_CONTROLLER_HAL_SPI_H
