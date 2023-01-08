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

        #define SPI_USE_REGISTER_CALLBACKS      1U
        #define SPI_USE_CRC                     0U
        #define PERIPHERAL_DEVICE_COUNT_MAX     3U
        #define PERIPHERAL_DEVICE_COUNT         PERIPHERAL_DEVICE_COUNT_MAX

        static constexpr uint8_t DEVICE_0 = 0U;
        static constexpr uint8_t DEVICE_1 = 1U;
        static constexpr uint8_t DEVICE_2 = 2U;

        #define SPI_BYTES_MAX                   255U
        #define SPI_BUFFER_MAX                  255U

        /* type definitions */
        typedef enum
        {
            SPI_STATUS_OK                       = 0x00U,
            SPI_STATUS_ERROR                    = 0x01U,
            SPI_STATUS_BUSY                     = 0x02U,
            SPI_STATUS_TIMEOUT                  = 0x03U
        } status_t;

        typedef enum
        {
            SPI_STATE_RESET                     = 0x00U,
            SPI_STATE_READY                     = 0x01U,
            SPI_STATE_BUSY                      = 0x02U,
            SPI_STATE_BUSY_TX                   = 0x03U,
            SPI_STATE_BUSY_RX                   = 0x04U,
            SPI_STATE_BUSY_TX_RX                = 0x05U,
            SPI_STATE_ERROR                     = 0x06U,
            SPI_STATE_ABORT                     = 0x07U
        } state_t;

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
        } callback_id_t;

        /* constants */
        static constexpr uint8_t    SPI_PROCEDURE_ERROR_NONE                    = 0U;
        static constexpr uint8_t    SPI_PROCEDURE_STATE_BUS_ERROR               = 1U;
        static constexpr uint8_t    SPI_PROCEDURE_STATE_DATA_ERROR              = 2U;

        static constexpr uint32_t   SPI_CR1_MODE_CONTROLLER                     = STM_HAL_SPI_CR1_CONTROLLER;
        static constexpr uint32_t   SPI_CR1_INTERNAL_CHIP_SELECT                = STM_HAL_SPI_CR1_INTERNAL_CHIP_SELECT;
        static constexpr uint32_t   SPI_CR1_RECEIVE_ONLY                        = STM_HAL_SPI_CR1_RECEIVE_ONLY;
        static constexpr uint32_t   SPI_CR1_BIDIRECTIONAL_MODE                  = STM_HAL_SPI_CR1_BIDIRECTIONAL_MODE;
        static constexpr uint32_t   SPI_CR1_DATA_FRAME_FORMAT                   = STM_HAL_SPI_CR1_DATA_FRAME_FORMAT;
        static constexpr uint32_t   SPI_CR1_CLOCK_POLARITY                      = STM_HAL_SPI_CR1_CLOCK_POLARITY;
        static constexpr uint32_t   SPI_CR1_CLOCK_PHASE                         = STM_HAL_SPI_CR1_CLOCK_PHASE;
        static constexpr uint32_t   SPI_CR1_SOFTWARE_CHIP_SELECT                = STM_HAL_SPI_CR1_SOFTWARE_CHIP_SELECT;
        static constexpr uint32_t   SPI_CR1_BAUD_RATE_CONTROL_MASK              = STM_HAL_SPI_CR1_BAUD_RATE;
        static constexpr uint32_t   SPI_CR1_LSB_FIRST                           = STM_HAL_SPI_CR1_LSB_FIRST;
        static constexpr uint32_t   SPI_CR1_CRC_ENABLE                          = STM_HAL_SPI_CR1_CRC_ENABLE;

        static constexpr uint32_t   SPI_CR2_CHIP_SELECT_OUTPUT_ENABLE           = STM_HAL_SPI_CR2_CHIP_SELECT_OUTPUT_ENABLE;
        static constexpr uint32_t   SPI_CR2_FRAME_FORMAT                        = STM_HAL_SPI_CR2_FRAME_FORMAT;
        static constexpr uint32_t   SPI_CR2_RX_BUFFER_DMA_ENABLE                = STM_HAL_SPI_CR2_RX_BUFFER_DMA_ENABLE;
        static constexpr uint32_t   SPI_CR2_TX_BUFFER_DMA_ENABLE                = STM_HAL_SPI_CR2_TX_BUFFER_DMA_ENABLE;

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
        static constexpr uint32_t   SPI_FLAG_RX_BUFFER_NOT_EMPTY                = STM_HAL_SPI_SR_RX_BUFFER_NOT_EMPTY;
        static constexpr uint32_t   SPI_FLAG_TX_BUFFER_EMPTY                    = STM_HAL_SPI_SR_TX_BUFFER_EMPTY;
        static constexpr uint32_t   SPI_FLAG_BUSY                               = STM_HAL_SPI_SR_BUSY;
        static constexpr uint32_t   SPI_FLAG_CRC_ERROR                          = STM_HAL_SPI_SR_CRC_ERROR;
        static constexpr uint32_t   SPI_FLAG_MODE_FAULT                         = STM_HAL_SPI_SR_MODE_FAULT;
        static constexpr uint32_t   SPI_FLAG_OVERRUN                            = STM_HAL_SPI_SR_OVERRUN;
        static constexpr uint32_t   SPI_FLAG_TI_MODE_FRAME_FORMAT_ERROR         = STM_HAL_SPI_SR_TI_MODE_FRAME_FORMAT_ERROR;
        static constexpr uint32_t   SPI_FLAG_BIT_MASK                           = (STM_HAL_SPI_SR_RX_BUFFER_NOT_EMPTY
                                                                                    | STM_HAL_SPI_SR_TX_BUFFER_EMPTY
                                                                                    | STM_HAL_SPI_SR_BUSY | STM_HAL_SPI_SR_CRC_ERROR \
                                                                                    | STM_HAL_SPI_SR_MODE_FAULT | STM_HAL_SPI_SR_OVERRUN
                                                                                    | STM_HAL_SPI_SR_TI_MODE_FRAME_FORMAT_ERROR);
        static constexpr uint32_t   SPI_MODE_PERIPHERAL                         = (0x00000000U);
        static constexpr uint32_t   SPI_MODE_CONTROLLER                         = (STM_HAL_SPI_CR1_CONTROLLER | STM_HAL_SPI_CR1_INTERNAL_CHIP_SELECT);
        static constexpr uint32_t   SPI_DIRECTION_2_LINE                        = (0x00000000U);
        static constexpr uint32_t   SPI_DIRECTION_2_LINE_RX_ONLY                = STM_HAL_SPI_CR1_RECEIVE_ONLY;
        static constexpr uint32_t   SPI_DIRECTION_1_LINE                        = STM_HAL_SPI_CR1_BIDIRECTIONAL_MODE;
        static constexpr uint32_t   SPI_DATA_SIZE_8_BIT                         = (0x00000000U);
        static constexpr uint32_t   SPI_DATA_SIZE_16_BIT                        = STM_HAL_SPI_CR1_DATA_FRAME_FORMAT;
        static constexpr uint32_t   SPI_DATA_MSB_FIRST                          = (0x00000000U);
        static constexpr uint32_t   SPI_DATA_LSB_FIRST                          = STM_HAL_SPI_CR1_LSB_FIRST;
        static constexpr uint32_t   SPI_CLOCK_POLARITY_LOW                      = (0x00000000U);
        static constexpr uint32_t   SPI_CLOCK_POLARITY_HIGH                     = STM_HAL_SPI_CR1_CLOCK_POLARITY;
        static constexpr uint32_t   SPI_CLOCK_PHASE_LEADING_EDGE                = (0x00000000U);
        static constexpr uint32_t   SPI_CLOCK_PHASE_TRAILING_EDGE               = STM_HAL_SPI_CR1_CLOCK_PHASE;
        static constexpr uint32_t   SPI_CHIP_SELECT_SOFTWARE                    = STM_HAL_SPI_CR1_SOFTWARE_CHIP_SELECT;
        static constexpr uint32_t   SPI_CHIP_SELECT_HARDWARE_INPUT              = (0x00000000U);
        static constexpr uint32_t   SPI_CHIP_SELECT_HARDWARE_OUTPUT             = (STM_HAL_SPI_CR2_CHIP_SELECT_OUTPUT_ENABLE << 16U);

        static constexpr uint32_t   SPI_BAUD_RATE_PRESCALER_2                   = (0x00000000U);
        static constexpr uint32_t   SPI_BAUD_RATE_PRESCALER_4                   = (STM_HAL_SPI_CR1_BAUD_RATE_0);
        static constexpr uint32_t   SPI_BAUD_RATE_PRESCALER_8                   = (STM_HAL_SPI_CR1_BAUD_RATE_1);
        static constexpr uint32_t   SPI_BAUD_RATE_PRESCALER_16                  = (STM_HAL_SPI_CR1_BAUD_RATE_1 | STM_HAL_SPI_CR1_BAUD_RATE_0);
        static constexpr uint32_t   SPI_BAUD_RATE_PRESCALER_32                  = (STM_HAL_SPI_CR1_BAUD_RATE_2);
        static constexpr uint32_t   SPI_BAUD_RATE_PRESCALER_64                  = (STM_HAL_SPI_CR1_BAUD_RATE_2 | STM_HAL_SPI_CR1_BAUD_RATE_0);
        static constexpr uint32_t   SPI_BAUD_RATE_PRESCALER_128                 = (STM_HAL_SPI_CR1_BAUD_RATE_2 | STM_HAL_SPI_CR1_BAUD_RATE_1);
        static constexpr uint32_t   SPI_BAUD_RATE_PRESCALER_256                 = (STM_HAL_SPI_CR1_BAUD_RATE_2 | STM_HAL_SPI_CR1_BAUD_RATE_1 | STM_HAL_SPI_CR1_BAUD_RATE_0);

        static constexpr uint32_t   SPI_TI_MODE_DISABLE                         = (0x00000000U);
        static constexpr uint32_t   SPI_TI_MODE_ENABLE                          = STM_HAL_SPI_CR2_FRAME_FORMAT;
        static constexpr uint32_t   SPI_TX_BUFFER_EMPTY_INTERRUPT_ENABLE        = STM_HAL_SPI_CR2_TX_BUFFER_EMPTY_INTERRUPT_ENABLE;
        static constexpr uint32_t   SPI_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE    = STM_HAL_SPI_CR2_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE;
        static constexpr uint32_t   SPI_ERROR_INTERRUPT_ENABLE                  = STM_HAL_SPI_CR2_ERROR_INTERRUPT_ENABLE;
        static constexpr uint32_t   SPI_DEFAULT_TIMEOUT_100_US                  = 100U;
        static constexpr uint32_t   SPI_BUSY_FLAG_WORK_AROUND_TIMEOUT_1000_US   = 1000U;
        static constexpr uint32_t   SPI_CRC_CALCULATION_DISABLE                 = (0x00000000U);
        static constexpr uint32_t   SPI_CRC_CALCULATION_ENABLE                  = STM_HAL_SPI_CR1_CRC_ENABLE;
        static constexpr uint32_t   SPI_I2S_MODE_SELECT                         = STM_HAL_SPI_I2S_MODE_SELECT;

        /* macros */
        #if (USE_RTOS == 1U)
        #error "USE_RTOS should be 0 in the current HAL release"
        #else
            #define SPI_LOCK_MODULE(__HANDLE__)                                                                             \
                do {                                                                                                        \
                    if((__HANDLE__)->lock == HAL_MODULE_LOCKED) { return SPI_STATUS_BUSY; }                                 \
                    else { (__HANDLE__)->lock = HAL_MODULE_LOCKED; }                                                        \
                }   while (0U)
            #define SPI_UNLOCK_MODULE(__HANDLE__) do { (__HANDLE__)->lock = HAL_MODULE_UNLOCKED; } while (0U)
        #endif

        #define STM_HAL_DMA_ENABLE(__HANDLE__)                      ((__HANDLE__)->instance->CONFIG_REG |=  STM_HAL_DMA_SxCR_ENABLE)
        #define STM_HAL_DMA_DISABLE(__HANDLE__)                     ((__HANDLE__)->instance->CONFIG_REG &=  ~STM_HAL_DMA_SxCR_ENABLE)
        #define SPI_ENABLE_MODULE(__HANDLE__)                       STM_HAL_SET_BIT((__HANDLE__)->instance->CONTROL_REG_1, STM_HAL_SPI_CR1_SPI_ENABLE)
        #define SPI_DISABLE_MODULE(__HANDLE__)                      STM_HAL_CLEAR_BIT((__HANDLE__)->instance->CONTROL_REG_1, STM_HAL_SPI_CR1_SPI_ENABLE)
        #define SPI_ENABLE_INTERRUPTS(__HANDLE__, __INTERRUPT__)    STM_HAL_SET_BIT((__HANDLE__)->instance->CONTROL_REG_2, (__INTERRUPT__))
        #define SPI_DISABLE_INTERRUPTS(__HANDLE__, __INTERRUPT__)   STM_HAL_CLEAR_BIT((__HANDLE__)->instance->CONTROL_REG_2, (__INTERRUPT__))
        #define SPI_GET_FLAG_STATUS(__HANDLE__, __FLAG__)          ((((__HANDLE__)->instance->STATUS_REG) & (__FLAG__)) == (__FLAG__))
        #define SPI_CHECK_FLAG_STATUS(__SR__, __FLAG__)            ((((__SR__) & ((__FLAG__) & SPI_FLAG_MASK)) == ((__FLAG__) & SPI_FLAG_MASK)) ? FLAG_SET : FLAG_RESET)
        #define SPI_VERIFY_DIRECTION_2_LINE(__MODE__)                ((__MODE__) == SPI_DIRECTION_2_LINE)
        #define SPI_VERIFY_DIRECTION_2_LINE_RX_ONLY(__MODE__)        ((__MODE__) == SPI_DIRECTION_2_LINE_RX_ONLY)
        #define SPI_VERIFY_DIRECTION_1_LINE(__MODE__)                ((__MODE__) == SPI_DIRECTION_1_LINE)
        #define SPI_VERIFY_VALID_INSTANCE(INSTANCE)                 (((INSTANCE) == SPI_1) || ((INSTANCE) == SPI_2)   ||    \
                                                                     ((INSTANCE) == SPI_3) || ((INSTANCE) == SPI_4))
        #define SPI_VERIFY_MODE(__MODE__)                           (((__MODE__) == SPI_MODE_PERIPHERAL)              ||    \
                                                                     ((__MODE__) == SPI_MODE_CONTROLLER))

        #define SPI_CHECK_INTERRUPT_SOURCE(__CR2__, __INTERRUPT__) ((((__CR2__) & (__INTERRUPT__)) ==                       \
                                                                      (__INTERRUPT__)) ? FLAG_SET : FLAG_RESET)
        #define SPI_VERIFY_DIRECTION(__MODE__)                      (((__MODE__) == SPI_DIRECTION_2_LINE)             ||    \
                                                                     ((__MODE__) == SPI_DIRECTION_2_LINE_RX_ONLY)     ||    \
                                                                     ((__MODE__) == SPI_DIRECTION_1_LINE))
        #define SPI_VERIFY_DATA_SIZE(__DATASIZE__)                  (((__DATASIZE__) == SPI_DATA_SIZE_16_BIT)         ||    \
                                                                     ((__DATASIZE__) == SPI_DATA_SIZE_8_BIT))
        #define SPI_VERIFY_CHIP_SELECT_MODE(__NSS__)                (((__NSS__) == SPI_CHIP_SELECT_SOFTWARE)          ||    \
                                                                     ((__NSS__) == SPI_CHIP_SELECT_HARDWARE_INPUT)    ||    \
                                                                     ((__NSS__) == SPI_CHIP_SELECT_HARDWARE_OUTPUT))
        #define SPI_VERIFY_BAUD_RATE_PRESCALER(__PRESCALER__)       (((__PRESCALER__) == SPI_BAUD_RATE_PRESCALER_2)   ||    \
                                                                     ((__PRESCALER__) == SPI_BAUD_RATE_PRESCALER_4)   ||    \
                                                                     ((__PRESCALER__) == SPI_BAUD_RATE_PRESCALER_8)   ||    \
                                                                     ((__PRESCALER__) == SPI_BAUD_RATE_PRESCALER_16)  ||    \
                                                                     ((__PRESCALER__) == SPI_BAUD_RATE_PRESCALER_32)  ||    \
                                                                     ((__PRESCALER__) == SPI_BAUD_RATE_PRESCALER_64)  ||    \
                                                                     ((__PRESCALER__) == SPI_BAUD_RATE_PRESCALER_128) ||    \
                                                                     ((__PRESCALER__) == SPI_BAUD_RATE_PRESCALER_256))
        #define SPI_VERIFY_FIRST_BIT_SETTING(__BIT__)               (((__BIT__) == SPI_DATA_MSB_FIRST)   || \
                                                                     ((__BIT__) == SPI_DATA_LSB_FIRST))
        #define SPI_VERIFY_TI_MODE(__MODE__)                        (((__MODE__) == SPI_TI_MODE_DISABLE) || \
                                                                     ((__MODE__) == SPI_TI_MODE_ENABLE))
        #define SPI_VERIFY_CLOCK_POLARITY(__CPOL__)                 (((__CPOL__) == SPI_POLARITY_LOW)    || \
                                                                     ((__CPOL__) == SPI_POLARITY_HIGH))
        #define SPI_VERIFY_CLOCK_PHASE(__CPHA__)                    (((__CPHA__) == SPI_PHASE_1EDGE)     || \
                                                                     ((__CPHA__) == SPI_PHASE_2EDGE))
        #define SPI_CLEAR_OVERRUN_FLAG(__HANDLE__)                                                      \
            do {                                                                                        \
                __IO uint32_t tmpreg_ovr = 0x00U;                                                       \
                tmpreg_ovr = (__HANDLE__)->instance->DATA_REG;                                          \
                tmpreg_ovr = (__HANDLE__)->instance->STATUS_REG;                                        \
                STM_HAL_UNUSED(tmpreg_ovr);                                                             \
            }   while (0U)
        #define SPI_CLEAR_MODE_FAULT_FLAG(__HANDLE__)                                                   \
            do {                                                                                        \
                __IO uint32_t tmpreg_modf = 0x00U;                                                      \
                tmpreg_modf = (__HANDLE__)->instance->STATUS_REG;                                       \
                STM_HAL_CLEAR_BIT((__HANDLE__)->instance->CONTROL_REG_1, STM_HAL_SPI_CR1_SPI_ENABLE);   \
                STM_HAL_UNUSED(tmpreg_modf);                                                            \
            }   while (0U)
        #define HAL_SPI_CLEAR_FORMAT_ERROR_FLAG(__HANDLE__)                                             \
            do {                                                                                        \
                __IO uint32_t tmpreg_fre = 0x00U;                                                       \
                tmpreg_fre = (__HANDLE__)->instance->STATUS_REG;                                        \
                STM_HAL_UNUSED(tmpreg_fre);                                                             \
            }   while (0U)
        #define SPI_RESET_CRC_CALCULATION(__HANDLE__)                                                   \
            do { STM_HAL_CLEAR_BIT((__HANDLE__)->instance->CONTROL_REG_1, STM_HAL_SPI_CR1_CRC_ENABLE);  \
            STM_HAL_SET_BIT((__HANDLE__)->instance->CONTROL_REG_1, STM_HAL_SPI_CR1_CRC_ENABLE); } while(0U)

        /* structures */
        typedef struct
        {
             uint8_t packet_id;
             uint8_t chip_select;
             uint8_t tx_size;
             std::vector<uint8_t> tx_bytes;
             std::vector<uint8_t> rx_bytes;
        } packet_t;

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
        } init_t;

        typedef struct
        {
            GPIO_TypeDef* port;
            uint16_t pin;
        } chip_select_t;

        GPIO_TypeDef* port_0 = GPIOB;
        GPIO_TypeDef* port_1 = GPIOC;
        GPIO_TypeDef* port_2 = GPIOC;
        uint16_t pin_0 = GPIO_PIN_14;
        uint16_t pin_1 = GPIO_PIN_7;
        uint16_t pin_2 = GPIO_PIN_8;
        chip_select_t chip_select_0 = { port_0, pin_0 };
        chip_select_t chip_select_1 = { port_1, pin_1 };
        chip_select_t chip_select_2 = { port_2, pin_2 };
        chip_select_t* chip_select[PERIPHERAL_DEVICE_COUNT] = { &chip_select_0, &chip_select_1, &chip_select_2 };

        typedef struct
        {
            GPIO_TypeDef* chip_select_port;
            uint16_t chip_select_pin;
            // clock setting
            // anything else

        } peripheral_device_t;

        typedef struct _handle_t
        {
            hal_spi_t                   *instance;
            init_t                      init;
            uint8_t                     *tx_buffer_ptr;
            uint16_t                    tx_transfer_size;
            volatile uint16_t           tx_transfer_counter;
            uint8_t                     *rx_buffer_ptr;
            uint16_t                    rx_transfer_size;
            volatile uint16_t           rx_transfer_counter;
            void                        (*rx_isr_ptr)(spi spi_object, struct _handle_t *spi_handle);
            void                        (*tx_isr_ptr)(spi spi_object, struct _handle_t *spi_handle);
            dma_handle_t                *tx_dma_handle;
            dma_handle_t                *rx_dma_handle;
            hal_lock_t                  lock;
            volatile state_t            state;
            volatile uint32_t           error_code;
            GPIO_TypeDef* chip_select_port;
            uint16_t chip_select_pin;

            #if (SPI_USE_REGISTER_CALLBACKS == 1U)
                void (* TxCpltCallback)             (struct _handle_t *spi_handle);
                void (* RxCpltCallback)             (struct _handle_t *spi_handle);
                void (* TxRxCpltCallback)           (struct _handle_t *spi_handle);
                void (* TxHalfCpltCallback)         (struct _handle_t *spi_handle);
                void (* RxHalfCpltCallback)         (struct _handle_t *spi_handle);
                void (* TxRxHalfCpltCallback)       (struct _handle_t *spi_handle);
                void (* ErrorCallback)              (struct _handle_t *spi_handle);
                void (* AbortCpltCallback)          (struct _handle_t *spi_handle);
                void (* MspInitCallback)            (struct _handle_t *spi_handle);
                void (* MspDeInitCallback)          (struct _handle_t *spi_handle);
            #endif
        } handle_t;

        /* public member variables */
        #if (SPI_USE_REGISTER_CALLBACKS == 1U)
                typedef void (*spi_callback_ptr_t)(handle_t* spi_module_handle);
                status_t spi_register_callback(callback_id_t callback_id, spi_callback_ptr_t pCallback);
                status_t spi_unregister_callback(callback_id_t callback_id);
        #endif
        handle_t* spi_module_handle;
        /* public member functions */
        void configure_spi_protocol(handle_t* spi_handle);
        void initialize_spi_object(handle_t* spi_handle, callback_id_t complete_callback_id, spi_callback_ptr_t complete_callback_ptr, callback_id_t error_callback_id, spi_callback_ptr_t error_callback_ptr);
        void initialize_spi_buffer();
        status_t spi_transmit_receive_interrupt(uint8_t *tx_data_pointer, uint8_t *rx_data_pointer, uint16_t packet_size, uint8_t device_id);
        status_t add_packet_to_buffer(uint8_t _chip_select, uint8_t _tx_size, uint8_t* _tx_bytes);
        void shift_buffer_contents_to_front();
        void process_spi_buffer();

        /* interrupt service routines */
        friend void spi_tx_2_line_8_bit_isr(spi spi_object, struct spi::_handle_t *spi_handle);
        friend void spi_rx_2_line_8_bit_isr(spi spi_object, struct spi::_handle_t *spi_handle);
        friend void spi_tx_2_line_16_bit_isr(spi spi_object, struct spi::_handle_t *spi_handle);
        friend void spi_rx_2_line_16_bit_isr(spi spi_object, struct spi::_handle_t *spi_handle);
        /* friend functions */
        friend spi::handle_t* get_spi_handle(spi* spi_object);
        friend spi::status_t dma_abort_interrupt(dma_handle_t *dma_handle);
        friend void dma_abort_on_error(dma_handle_t *dma_handle);
        friend void spi_irq_handler(spi* spi_object);
        /* spi callback prototypes */
        friend void HAL_SPI_TxCpltCallback(spi::handle_t *spi_handle);
        friend void HAL_SPI_RxCpltCallback(spi::handle_t *spi_handle);
        friend void HAL_SPI_TxRxCpltCallback(spi::handle_t *spi_handle);
        friend void HAL_SPI_TxHalfCpltCallback(spi::handle_t *spi_handle);
        friend void HAL_SPI_RxHalfCpltCallback(spi::handle_t *spi_handle);
        friend void HAL_SPI_TxRxHalfCpltCallback(spi::handle_t *spi_handle);
        friend void HAL_SPI_ErrorCallback(spi::handle_t *spi_handle);
        friend void HAL_SPI_AbortCpltCallback(spi::handle_t *spi_handle);

    private:
        /* private objects */
        uint8_t packet_id_counter = 0;
        std::vector<packet_t*> spi_buffer;
        uint8_t head = 0;
        uint8_t tail = 0;
        /* private member functions */
        status_t initialize_spi_protocol();
        void set_rx_and_tx_interrupt_service_routines() const;
        status_t lock_module() const;
        void unlock_module() const;
        state_t get_module_communication_state() const;
        uint32_t get_module_operating_mode() const;
        void verify_communication_direction(uint32_t intended_direction) const;
        void set_transaction_parameters(uint8_t *tx_data_ptr, uint8_t *rx_data_ptr, uint16_t packet_size) const;
        status_t wait_for_flag_until_timeout(uint32_t flag, flag_status_t flag_status, uint32_t flag_timeout, uint32_t start_time) const;
        status_t end_rx_transaction(uint32_t flag_timeout, uint32_t start_time);
        status_t end_rx_tx_transaction(uint32_t flag_timeout, uint32_t start_time);
        void close_rx_tx_isr();
        void close_rx_isr();
        void close_tx_isr();
        void abort_rx_isr();
        void abort_tx_isr();
        void reset_enabled_crc();
};

#endif //MAIN_CONTROLLER_HAL_SPI_H
