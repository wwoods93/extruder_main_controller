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

#include <vector>
#include "stm32f4xx.h"
#include "hal_general.h"

class spi
{
    public:
        #define SPI_USE_REGISTER_CALLBACKS          1U
        #define SPI_USE_CRC                         0U

        #define CHIP_SELECT_1                       0U
        #define CHIP_SELECT_2                       1U
        #define CHIP_SELECT_3                       2U
        #define SPI_BYTES_MAX                       255U
        #define SPI_BUFFER_MAX                      255U

        static constexpr uint8_t    SPI_PROCEDURE_ERROR_NONE                    = 0U;
        static constexpr uint8_t    SPI_PROCEDURE_STATE_BUS_ERROR               = 1U;
        static constexpr uint8_t    SPI_PROCEDURE_STATE_DATA_ERROR              = 2U;

        static constexpr uint32_t   SPI_CR1_MODE_CONTROLLER                     = HAL_SPI_CR1_CONTROLLER;
        static constexpr uint32_t   SPI_CR1_INTERNAL_CHIP_SELECT                = HAL_SPI_CR1_INTERNAL_CHIP_SELECT;
        static constexpr uint32_t   SPI_CR1_RECEIVE_ONLY                        = HAL_SPI_CR1_RECEIVE_ONLY;
        static constexpr uint32_t   SPI_CR1_BIDIRECTIONAL_MODE                  = HAL_SPI_CR1_BIDIRECTIONAL_MODE;
        static constexpr uint32_t   SPI_CR1_DATA_FRAME_FORMAT                   = HAL_SPI_CR1_DATA_FRAME_FORMAT;
        static constexpr uint32_t   SPI_CR1_CLOCK_POLARITY                      = HAL_SPI_CR1_CLOCK_POLARITY;
        static constexpr uint32_t   SPI_CR1_CLOCK_PHASE                         = HAL_SPI_CR1_CLOCK_PHASE;
        static constexpr uint32_t   SPI_CR1_SOFTWARE_CHIP_SELECT                = HAL_SPI_CR1_SOFTWARE_CHIP_SELECT;
        static constexpr uint32_t   SPI_CR1_BAUD_RATE_CONTROL_MASK              = HAL_SPI_CR1_BAUD_RATE;
        static constexpr uint32_t   SPI_CR1_LSB_FIRST                           = HAL_SPI_CR1_LSB_FIRST;
        static constexpr uint32_t   SPI_CR1_CRC_ENABLE                          = HAL_SPI_CR1_CRC_ENABLE;

        static constexpr uint32_t   SPI_CR2_CHIP_SELECT_OUTPUT_ENABLE           = HAL_SPI_CR2_CHIP_SELECT_OUTPUT_ENABLE;
        static constexpr uint32_t   SPI_CR2_FRAME_FORMAT                        = HAL_SPI_CR2_FRAME_FORMAT;
        static constexpr uint32_t   SPI_CR2_RX_BUFFER_DMA_ENABLE                = HAL_SPI_CR2_RX_BUFFER_DMA_ENABLE;
        static constexpr uint32_t   SPI_CR2_TX_BUFFER_DMA_ENABLE                = HAL_SPI_CR2_TX_BUFFER_DMA_ENABLE;

        #if (SPI_USE_REGISTER_CALLBACKS == 1U)
            static constexpr uint8_t    SPI_ERROR_NONE                          = (0x00000000U);
            static constexpr uint8_t    SPI_ERROR_MODE_FAULT                    = (0x00000001U);
            static constexpr uint8_t    SPI_ERROR_DURING_CRC_CALCULATION        = (0x00000002U);
            static constexpr uint8_t    SPI_ERROR_OVERRUN                       = (0x00000004U);
            static constexpr uint8_t    SPI_ERROR_TI_MODE_FRAME_FORMAT          = (0x00000008U);
            static constexpr uint8_t    SPI_ERROR_DMA_TRANSFER                  = (0x00000010U);
            static constexpr uint8_t    SPI_ERROR_WAITING_FOR_FLAG              = (0x00000020U);
            static constexpr uint8_t    SPI_ERROR_DURING_ABORT                  = (0x00000040U);
            static constexpr uint8_t    SPI_ERROR_CALLBACK_INVALID              = (0x00000080U);
        #endif
        static constexpr uint32_t   SPI_FLAG_RX_BUFFER_NOT_EMPTY                = HAL_SPI_SR_RX_BUFFER_NOT_EMPTY;
        static constexpr uint32_t   SPI_FLAG_TX_BUFFER_EMPTY                    = HAL_SPI_SR_TX_BUFFER_EMPTY;
        static constexpr uint32_t   SPI_FLAG_BUSY                               = HAL_SPI_SR_BUSY;
        static constexpr uint32_t   SPI_FLAG_CRC_ERROR                          = HAL_SPI_SR_CRC_ERROR;
        static constexpr uint32_t   SPI_FLAG_MODE_FAULT                         = HAL_SPI_SR_MODE_FAULT;
        static constexpr uint32_t   SPI_FLAG_OVERRUN                            = HAL_SPI_SR_OVERRUN;
        static constexpr uint32_t   SPI_FLAG_TI_MODE_FRAME_FORMAT_ERROR         = HAL_SPI_SR_TI_MODE_FRAME_FORMAT_ERROR;
        static constexpr uint32_t   SPI_FLAG_BIT_MASK                           = (HAL_SPI_SR_RX_BUFFER_NOT_EMPTY
                                                                                    | HAL_SPI_SR_TX_BUFFER_EMPTY
                                                                                    | HAL_SPI_SR_BUSY | HAL_SPI_SR_CRC_ERROR \
                                                                                    | HAL_SPI_SR_MODE_FAULT | HAL_SPI_SR_OVERRUN
                                                                                    | HAL_SPI_SR_TI_MODE_FRAME_FORMAT_ERROR);
        static constexpr uint32_t   SPI_MODE_PERIPHERAL                         = (0x00000000U);
        static constexpr uint32_t   SPI_MODE_CONTROLLER                         = (HAL_SPI_CR1_CONTROLLER | HAL_SPI_CR1_INTERNAL_CHIP_SELECT);
        static constexpr uint32_t   SPI_DIRECTION_2_LINE                        = (0x00000000U);
        static constexpr uint32_t   SPI_DIRECTION_2_LINE_RX_ONLY                = HAL_SPI_CR1_RECEIVE_ONLY;
        static constexpr uint32_t   SPI_DIRECTION_1_LINE                        = HAL_SPI_CR1_BIDIRECTIONAL_MODE;
        static constexpr uint32_t   SPI_DATA_SIZE_8_BIT                         = (0x00000000U);
        static constexpr uint32_t   SPI_DATA_SIZE_16_BIT                        = HAL_SPI_CR1_DATA_FRAME_FORMAT;
        static constexpr uint32_t   SPI_DATA_MSB_FIRST                          = (0x00000000U);
        static constexpr uint32_t   SPI_DATA_LSB_FIRST                          = HAL_SPI_CR1_LSB_FIRST;
        static constexpr uint32_t   SPI_CLOCK_POLARITY_LOW                      = (0x00000000U);
        static constexpr uint32_t   SPI_CLOCK_POLARITY_HIGH                     = HAL_SPI_CR1_CLOCK_POLARITY;
        static constexpr uint32_t   SPI_CLOCK_PHASE_LEADING_EDGE                = (0x00000000U);
        static constexpr uint32_t   SPI_CLOCK_PHASE_TRAILING_EDGE               = HAL_SPI_CR1_CLOCK_PHASE;
        static constexpr uint32_t   SPI_CHIP_SELECT_SOFTWARE                    = HAL_SPI_CR1_SOFTWARE_CHIP_SELECT;
        static constexpr uint32_t   SPI_CHIP_SELECT_HARDWARE_INPUT              = (0x00000000U);
        static constexpr uint32_t   SPI_CHIP_SELECT_HARDWARE_OUTPUT             = (HAL_SPI_CR2_CHIP_SELECT_OUTPUT_ENABLE << 16U);

        static constexpr uint32_t   SPI_BAUD_RATE_PRESCALER_2                   = (0x00000000U);
        static constexpr uint32_t   SPI_BAUD_RATE_PRESCALER_4                   = (HAL_SPI_CR1_BAUD_RATE_0);
        static constexpr uint32_t   SPI_BAUD_RATE_PRESCALER_8                   = (HAL_SPI_CR1_BAUD_RATE_1);
        static constexpr uint32_t   SPI_BAUD_RATE_PRESCALER_16                  = (HAL_SPI_CR1_BAUD_RATE_1 | HAL_SPI_CR1_BAUD_RATE_0);
        static constexpr uint32_t   SPI_BAUD_RATE_PRESCALER_32                  = (HAL_SPI_CR1_BAUD_RATE_2);
        static constexpr uint32_t   SPI_BAUD_RATE_PRESCALER_64                  = (HAL_SPI_CR1_BAUD_RATE_2 | HAL_SPI_CR1_BAUD_RATE_0);
        static constexpr uint32_t   SPI_BAUD_RATE_PRESCALER_128                 = (HAL_SPI_CR1_BAUD_RATE_2 | HAL_SPI_CR1_BAUD_RATE_1);
        static constexpr uint32_t   SPI_BAUD_RATE_PRESCALER_256                 = (HAL_SPI_CR1_BAUD_RATE_2 | HAL_SPI_CR1_BAUD_RATE_1 | HAL_SPI_CR1_BAUD_RATE_0);

        static constexpr uint32_t   SPI_TI_MODE_DISABLE                         = (0x00000000U);
        static constexpr uint32_t   SPI_TI_MODE_ENABLE                          = HAL_SPI_CR2_FRAME_FORMAT;
        static constexpr uint32_t   SPI_TX_BUFFER_EMPTY_INTERRUPT_ENABLE        = HAL_SPI_CR2_TX_BUFFER_EMPTY_INTERRUPT_ENABLE;
        static constexpr uint32_t   SPI_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE    = HAL_SPI_CR2_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE;
        static constexpr uint32_t   SPI_ERROR_INTERRUPT_ENABLE                  = HAL_SPI_CR2_ERROR_INTERRUPT_ENABLE;
        static constexpr uint32_t   SPI_DEFAULT_TIMEOUT_100_US                  = 100U;
        static constexpr uint32_t   SPI_BUSY_FLAG_WORK_AROUND_TIMEOUT_1000_US   = 1000U;
        static constexpr uint32_t   SPI_CRC_CALCULATION_DISABLE                 = (0x00000000U);
        static constexpr uint32_t   SPI_CRC_CALCULATION_ENABLE                  = HAL_SPI_CR1_CRC_ENABLE;
        static constexpr uint32_t   SPI_I2S_MODE_SELECT                         = HAL_SPI_I2S_MODE_SELECT;
        #if (USE_RTOS == 1U)
        #error "USE_RTOS should be 0 in the current HAL release"
        #else
            #define SPI_LOCK_MODULE(__HANDLE__)                                                         \
                do {                                                                                    \
                    if((__HANDLE__)->lock == HAL_MODULE_LOCKED) { return SPI_STATUS_BUSY; }             \
                    else { (__HANDLE__)->lock = HAL_MODULE_LOCKED; }                                    \
                }   while (0U)                                                                          \

            #define SPI_UNLOCK_MODULE(__HANDLE__) do { (__HANDLE__)->lock = HAL_MODULE_UNLOCKED; } while (0U)
        #endif

        #define HAL_DMA_ENABLE(__HANDLE__)                          ((__HANDLE__)->instance->CONFIG_REG |=  HAL_DMA_SxCR_ENABLE)
        #define HAL_DMA_DISABLE(__HANDLE__)                         ((__HANDLE__)->instance->CONFIG_REG &=  ~HAL_DMA_SxCR_ENABLE)
        #define SPI_ENABLE_MODULE(__HANDLE__)                       HAL_GENERAL_SET_BIT((__HANDLE__)->instance->CONTROL_REG_1, HAL_SPI_CR1_SPI_ENABLE)
        #define SPI_DISABLE_MODULE(__HANDLE__)                      HAL_GENERAL_CLEAR_BIT((__HANDLE__)->instance->CONTROL_REG_1, HAL_SPI_CR1_SPI_ENABLE)
        #define SPI_ENABLE_INTERRUPTS(__HANDLE__, __INTERRUPT__)    HAL_GENERAL_SET_BIT((__HANDLE__)->instance->CONTROL_REG_2, (__INTERRUPT__))
        #define SPI_DISABLE_INTERRUPTS(__HANDLE__, __INTERRUPT__)   HAL_GENERAL_CLEAR_BIT((__HANDLE__)->instance->CONTROL_REG_2, (__INTERRUPT__))
        #define SPI_GET_FLAG_STATUS(__HANDLE__, __FLAG__)           ((((__HANDLE__)->instance->STATUS_REG) & (__FLAG__)) == (__FLAG__))
        #define SPI_CHECK_FLAG_STATUS(__SR__, __FLAG__)             ((((__SR__) & ((__FLAG__) & SPI_FLAG_MASK)) == ((__FLAG__) & SPI_FLAG_MASK)) ? FLAG_SET : FLAG_RESET)
        #define SPI_VERIFY_DIRECTION_2_LINE(__MODE__)               ((__MODE__) == SPI_DIRECTION_2_LINE)
        #define SPI_VERIFY_DIRECTION_2_LINE_RX_ONLY(__MODE__)       ((__MODE__) == SPI_DIRECTION_2_LINE_RX_ONLY)
        #define SPI_VERIFY_DIRECTION_1_LINE(__MODE__)               ((__MODE__) == SPI_DIRECTION_1_LINE)
        #define SPI_VERIFY_VALID_INSTANCE(INSTANCE)                 (((INSTANCE) == SPI_1) || ((INSTANCE) == SPI_2) || \
                                                                     ((INSTANCE) == SPI_3) || ((INSTANCE) == SPI_4))
        #define SPI_VERIFY_MODE(__MODE__)                           (((__MODE__) == SPI_MODE_PERIPHERAL)   || \
                                                                     ((__MODE__) == SPI_MODE_CONTROLLER))

        #define SPI_CHECK_INTERRUPT_SOURCE(__CR2__, __INTERRUPT__)  ((((__CR2__) & (__INTERRUPT__)) ==  \
                                                                    (__INTERRUPT__)) ? FLAG_SET : FLAG_RESET)
        #define SPI_VERIFY_DIRECTION(__MODE__)                      (((__MODE__) == SPI_DIRECTION_2_LINE)        || \
                                                                     ((__MODE__) == SPI_DIRECTION_2_LINE_RX_ONLY) || \
                                                                     ((__MODE__) == SPI_DIRECTION_1_LINE))
        #define SPI_VERIFY_DATA_SIZE(__DATASIZE__)                  (((__DATASIZE__) == SPI_DATA_SIZE_16_BIT) || \
                                                                     ((__DATASIZE__) == SPI_DATA_SIZE_8_BIT))
        #define SPI_VERIFY_CHIP_SELECT_MODE(__NSS__)                (((__NSS__) == SPI_CHIP_SELECT_SOFTWARE)       || \
                                                                     ((__NSS__) == SPI_CHIP_SELECT_HARDWARE_INPUT) || \
                                                                     ((__NSS__) == SPI_CHIP_SELECT_HARDWARE_OUTPUT))
        #define SPI_VERIFY_BAUD_RATE_PRESCALER(__PRESCALER__)       (((__PRESCALER__) == SPI_BAUD_RATE_PRESCALER_2)   || \
                                                                     ((__PRESCALER__) == SPI_BAUD_RATE_PRESCALER_4)   || \
                                                                     ((__PRESCALER__) == SPI_BAUD_RATE_PRESCALER_8)   || \
                                                                     ((__PRESCALER__) == SPI_BAUD_RATE_PRESCALER_16)  || \
                                                                     ((__PRESCALER__) == SPI_BAUD_RATE_PRESCALER_32)  || \
                                                                     ((__PRESCALER__) == SPI_BAUD_RATE_PRESCALER_64)  || \
                                                                     ((__PRESCALER__) == SPI_BAUD_RATE_PRESCALER_128) || \
                                                                     ((__PRESCALER__) == SPI_BAUD_RATE_PRESCALER_256))
        #define SPI_VERIFY_FIRST_BIT_SETTING(__BIT__)               (((__BIT__) == SPI_DATA_MSB_FIRST) || \
                                                                     ((__BIT__) == SPI_DATA_LSB_FIRST))
        #define SPI_VERIFY_TI_MODE(__MODE__)                        (((__MODE__) == SPI_TI_MODE_DISABLE) || \
                                                                     ((__MODE__) == SPI_TI_MODE_ENABLE))
        #define SPI_VERIFY_CLOCK_POLARITY(__CPOL__)                 (((__CPOL__) == SPI_POLARITY_LOW) || \
                                                                     ((__CPOL__) == SPI_POLARITY_HIGH))
        #define SPI_VERIFY_CLOCK_PHASE(__CPHA__)                    (((__CPHA__) == SPI_PHASE_1EDGE) || \
                                                                     ((__CPHA__) == SPI_PHASE_2EDGE))


        #define SPI_CLEAR_OVERRUN_FLAG(__HANDLE__)                                                      \
            do {                                                                                        \
                __IO uint32_t tmpreg_ovr = 0x00U;                                                       \
                tmpreg_ovr = (__HANDLE__)->instance->DATA_REG;                                          \
                tmpreg_ovr = (__HANDLE__)->instance->STATUS_REG;                                        \
                HAL_GENERAL_UNUSED(tmpreg_ovr);                                                         \
            }   while(0U)
        #define SPI_CLEAR_MODE_FAULT_FLAG(__HANDLE__)                                                   \
            do {                                                                                        \
                __IO uint32_t tmpreg_modf = 0x00U;                                                      \
                tmpreg_modf = (__HANDLE__)->instance->STATUS_REG;                                       \
                HAL_GENERAL_CLEAR_BIT((__HANDLE__)->instance->CONTROL_REG_1, HAL_SPI_CR1_SPI_ENABLE);   \
                HAL_GENERAL_UNUSED(tmpreg_modf);                                                        \
            }   while(0U)
        #define HAL_SPI_CLEAR_FORMAT_ERROR_FLAG(__HANDLE__)                                             \
            do {                                                                                        \
                __IO uint32_t tmpreg_fre = 0x00U;                                                       \
                tmpreg_fre = (__HANDLE__)->instance->STATUS_REG;                                        \
                HAL_GENERAL_UNUSED(tmpreg_fre);                                                         \
            }   while(0U)
        #define SPI_RESET_CRC_CALCULATION(__HANDLE__)                                                   \
            do { HAL_GENERAL_CLEAR_BIT((__HANDLE__)->instance->CONTROL_REG_1, HAL_SPI_CR1_CRC_ENABLE);  \
            HAL_GENERAL_SET_BIT((__HANDLE__)->instance->CONTROL_REG_1, HAL_SPI_CR1_CRC_ENABLE); } while(0U)

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
                void (* TxCpltCallback)         (struct _spi_handle_t *spi_handle);
                void (* RxCpltCallback)         (struct _spi_handle_t *spi_handle);
                void (* TxRxCpltCallback)       (struct _spi_handle_t *spi_handle);
                void (* TxHalfCpltCallback)     (struct _spi_handle_t *spi_handle);
                void (* RxHalfCpltCallback)     (struct _spi_handle_t *spi_handle);
                void (* TxRxHalfCpltCallback)   (struct _spi_handle_t *spi_handle);
                void (* ErrorCallback)          (struct _spi_handle_t *spi_handle);
                void (* AbortCpltCallback)      (struct _spi_handle_t *spi_handle);
                void (* MspInitCallback)        (struct _spi_handle_t *spi_handle);
                void (* MspDeInitCallback)      (struct _spi_handle_t *spi_handle);
            #endif
        } spi_handle_t;

        typedef struct
        {
            uint8_t packet_id;
            uint8_t chip_select;
            uint8_t tx_size;
            std::vector<uint8_t> tx_bytes;
            std::vector<uint8_t> rx_bytes;
        } spi_packet_t;

        spi_handle_t* spi_module_handle;
        /******************************************* public member functions ******************************************/
        void configure_module(spi_handle_t* spi_handle);
        spi_status_t spi_transmit_receive_interrupt(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);
        spi_status_t add_packet_to_buffer(uint8_t packet_chip_select, uint8_t packet_tx_size, std::vector<uint8_t>* tx_bytes);
        void shift_up_spi_buffer();
        void process_spi_buffer();
        #if (SPI_USE_REGISTER_CALLBACKS == 1U)
            typedef void (*spi_callback_pointer_t)(spi_handle_t* spi_module_handle);
            spi_status_t spi_register_callback(spi_callback_id_t CallbackID, spi_callback_pointer_t pCallback);
            spi_status_t spi_unregister_callback(spi_callback_id_t CallbackID);
        #endif
        /****************************************** interrupt service routines ****************************************/
        friend void spi_rx_2_line_8_bit_isr(spi spi_object, struct spi::_spi_handle_t *spi_handle);
        friend void spi_tx_2_line_8_bit_isr(spi spi_object, struct spi::_spi_handle_t *spi_handle);
        friend void spi_rx_2_line_16_bit_isr(spi spi_object, struct spi::_spi_handle_t *spi_handle);
        friend void spi_tx_2_line_16_bit_isr(spi spi_object, struct spi::_spi_handle_t *spi_handle);
        /*********************************************** friend functions *********************************************/
        friend void spi_irq_handler(spi* spi_object);
        friend void SPI_DMAAbortOnError(dma_handle_t *dma_handle);
        friend spi::spi_status_t dma_abort_interrupt(dma_handle_t *dma_handle);
        /******************************************** spi callback prototypes *****************************************/
        friend void HAL_SPI_TxCpltCallback(spi::spi_handle_t *spi_handle);
        friend void HAL_SPI_RxCpltCallback(spi::spi_handle_t *spi_handle);
        friend void HAL_SPI_TxRxCpltCallback(spi::spi_handle_t *spi_handle);
        friend void HAL_SPI_TxHalfCpltCallback(spi::spi_handle_t *spi_handle);
        friend void HAL_SPI_RxHalfCpltCallback(spi::spi_handle_t *spi_handle);
        friend void HAL_SPI_TxRxHalfCpltCallback(spi::spi_handle_t *spi_handle);
        friend void HAL_SPI_ErrorCallback(spi::spi_handle_t *spi_handle);
        friend void HAL_SPI_AbortCpltCallback(spi::spi_handle_t *spi_handle);
        /**************************************************************************************************************/
    private:
        /****************************************** private member functions ******************************************/
        uint8_t packet_id_counter = 0;
        std::vector<spi_packet_t*> spi_buffer;
        uint8_t head = 0;
        uint8_t tail = 0;
        spi_status_t initialize_module();
        void initialize_spi_buffer();
        void set_rx_and_tx_interrupt_service_routines() const;
        spi_status_t lock_module() const;
        void unlock_module() const;
        uint8_t get_module_communication_state() const;
        uint32_t get_module_operating_mode() const;
        void verify_communication_direction(uint32_t intended_direction) const;
        void set_transaction_parameters(uint8_t *tx_data_pointer, uint8_t *rx_data_pointer, uint16_t packet_size) const;
        spi_status_t wait_for_flag_until_timeout(uint32_t flag, flag_status_t flag_status, uint32_t flag_timeout, uint32_t start_time);
        spi_status_t end_rx_transaction(uint32_t flag_timeout, uint32_t start_time);
        spi_status_t end_rx_tx_transaction(uint32_t flag_timeout, uint32_t start_time);
        void close_rx_tx_isr();
        void close_rx_isr();
        void close_tx_isr();
        void abort_rx_isr();
        void abort_tx_isr();
        void reset_enabled_crc();
        /**************************************************************************************************************/
};

#endif //MAIN_CONTROLLER_HAL_SPI_Hs
