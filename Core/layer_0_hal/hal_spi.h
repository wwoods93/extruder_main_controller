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

/* c/c++ includes */
#include <vector>
#include <queue>
/* stm32 includes */
#include "stm32f4xx.h"
/* third-party includes */
#include "../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.h"
/* hal includes */
#include "hal_general.h"
/* driver includes */

/* rtos abstraction includes */
#include "../layer_2_rtosal/rtosal_spi_shared_resources.h"
/* sys op includes */

/* meta structure includes */
#include "../meta_structure/meta_structure_system_manager.h"
#include "../meta_structure/meta_structure_resource.h"

    #define SPI_USE_REGISTER_CALLBACKS          1U
    #define SPI_USE_CRC                         0U

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
    static constexpr uint32_t   SPI_CR1_CRC_DISABLE                         = STM_HAL_SPI_CR1_CRC_DISABLE;
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

    static constexpr uint16_t   SPI_CRC_POLYNOMIAL_MIN                      = (0x0001U);
    static constexpr uint32_t   SPI_CRC_POLYNOMIAL_MAX                      = (0xFFFFU);

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
    #define SPI_LOCK_MODULE(__HANDLE__)                                                                 \
                    do {                                                                                \
                        if((__HANDLE__)->lock == HAL_MODULE_LOCKED) { return PROCEDURE_STATUS_BUSY; }         \
                        else { (__HANDLE__)->lock = HAL_MODULE_LOCKED; }                                \
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


    #define SPI_CHECK_INTERRUPT_SOURCE(__CR2__, __INTERRUPT__) ((((__CR2__) & (__INTERRUPT__)) == (__INTERRUPT__)) ? FLAG_SET : FLAG_RESET)
    #define SPI_CLEAR_OVERRUN_FLAG(__HANDLE__)                                                              \
                do {                                                                                        \
                    __IO uint32_t tmpreg_ovr = 0x00U;                                                       \
                    tmpreg_ovr = (__HANDLE__)->instance->DATA_REG;                                          \
                    tmpreg_ovr = (__HANDLE__)->instance->STATUS_REG;                                        \
                    STM_HAL_UNUSED(tmpreg_ovr);                                                             \
                }   while (0U)
    #define SPI_CLEAR_MODE_FAULT_FLAG(__HANDLE__)                                                           \
                do {                                                                                        \
                    __IO uint32_t tmpreg_modf = 0x00U;                                                      \
                    tmpreg_modf = (__HANDLE__)->instance->STATUS_REG;                                       \
                    STM_HAL_CLEAR_BIT((__HANDLE__)->instance->CONTROL_REG_1, STM_HAL_SPI_CR1_SPI_ENABLE);   \
                    STM_HAL_UNUSED(tmpreg_modf);                                                            \
                }   while (0U)
    #define HAL_SPI_CLEAR_FORMAT_ERROR_FLAG(__HANDLE__)                                                     \
                do {                                                                                        \
                    __IO uint32_t tmpreg_fre = 0x00U;                                                       \
                    tmpreg_fre = (__HANDLE__)->instance->STATUS_REG;                                        \
                    STM_HAL_UNUSED(tmpreg_fre);                                                             \
                }   while (0U)
    #define SPI_RESET_CRC_CALCULATION(__HANDLE__)                                                           \
                do { STM_HAL_CLEAR_BIT((__HANDLE__)->instance->CONTROL_REG_1, STM_HAL_SPI_CR1_CRC_ENABLE);  \
                STM_HAL_SET_BIT((__HANDLE__)->instance->CONTROL_REG_1, STM_HAL_SPI_CR1_CRC_ENABLE); } while(0U)


class spi : public resource
{
    public:

        static constexpr uint8_t SPI_TRANSACTION_NOT_IN_PROGRESS            = 0U;
        static constexpr uint8_t SPI_TRANSACTION_IN_PROGRESS                = 1U;
        static constexpr uint8_t SPI_TRANSACTION_COMPLETE                   = 2U;

        static constexpr uint8_t SPI_USER_CHANNELS_MAX                      = 8U;
        static constexpr uint8_t SPI_BYTE_COUNT_MAX                         = 8U;

        static constexpr uint8_t ACTIVE_LOW                                 = 0U;
        static constexpr uint8_t CHIP_SELECT_LOGIC_LEVEL                    = ACTIVE_LOW;
        static constexpr uint8_t CHIP_SELECT_SET                            = CHIP_SELECT_LOGIC_LEVEL;
        static constexpr uint8_t CHIP_SELECT_RESET                          = !CHIP_SELECT_LOGIC_LEVEL;

        static constexpr uint8_t CHANNEL_0                                  = 0U;
        static constexpr uint8_t CHANNEL_1                                  = 1U;
        static constexpr uint8_t CHANNEL_2                                  = 2U;
        static constexpr uint8_t CHANNEL_3                                  = 3U;
        static constexpr uint8_t CHANNEL_4                                  = 4U;
        static constexpr uint8_t CHANNEL_5                                  = 5U;
        static constexpr uint8_t CHANNEL_6                                  = 6U;
        static constexpr uint8_t CHANNEL_7                                  = 7U;

        static constexpr uint8_t SPI_INIT_PROTOCOL_ERROR                    = 0U;
        static constexpr uint8_t SPI_INIT_REGISTER_CALLBACKS_ERROR          = 1U;
        static constexpr uint8_t SPI_INIT_DATA_STRUCTURES_ERROR             = 2U;
        static constexpr uint8_t SPI_INIT_RESET_CHIP_SELECTS_ERROR          = 3U;


        /* type definitions */
        typedef enum
        {
            PROCEDURE_STATUS_OK                 = 0x00U,
            PROCEDURE_STATUS_ERROR              = 0x01U,
            PROCEDURE_STATUS_BUSY               = 0x02U,
            PROCEDURE_STATUS_TIMEOUT            = 0x03U,
        } procedure_status_t;

        typedef enum
        {
            SPI_STATE_RESET                     = 0x00U,
            SPI_STATE_READY                     = 0x01U,
            SPI_STATE_BUSY                      = 0x02U,
            SPI_STATE_BUSY_TX                   = 0x03U,
            SPI_STATE_BUSY_RX                   = 0x04U,
            SPI_STATE_BUSY_TX_RX                = 0x05U,
            SPI_STATE_ERROR                     = 0x06U,
            SPI_STATE_ABORT                     = 0x07U,
        } comms_state_t;

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

        typedef struct
        {
            GPIO_TypeDef* port;
            uint16_t pin;
        } chip_select_t;

        typedef struct
        {
            id_number_t channel_id;
            uint8_t tx_bytes[SPI_BYTE_COUNT_MAX];
            uint8_t rx_bytes[SPI_BYTE_COUNT_MAX];
        } packet_t;

        typedef struct
        {
            id_number_t channel_id;
            uint8_t packet_size;
            uint8_t tx_size;
            chip_select_t chip_select;
        } channel_t;

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

        typedef struct _handle_t
        {
            hal_spi_t           *instance;
            chip_select_t       chip_select;
            init_t              init;
            uint8_t             *tx_buffer_ptr;
            uint16_t            tx_transfer_size;
            volatile uint16_t   tx_transfer_counter;
            uint8_t             *rx_buffer_ptr;
            uint8_t             rx_array[SPI_BYTE_COUNT_MAX];
            uint16_t            rx_transfer_size;
            volatile uint16_t   rx_transfer_counter;
            void                (*rx_isr_ptr)(spi spi_object, struct _handle_t *spi_handle);
            void                (*tx_isr_ptr)(spi spi_object, struct _handle_t *spi_handle);
            dma_handle_t        *tx_dma_handle;
            dma_handle_t        *rx_dma_handle;
            hal_lock_t          lock;
            volatile comms_state_t    state;
            volatile uint32_t   error_code;


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

        typedef void (*spi_callback_ptr_t)(handle_t* spi_module_handle);

        struct
        {
            channel_t channel_0;
            channel_t channel_1;
            channel_t channel_2;
            channel_t channel_3;
            channel_t channel_4;
            channel_t channel_5;
            channel_t channel_6;
            channel_t channel_7;
        } user_list;

        static constexpr uint8_t SIZE_OF_CHIP_SELECT_T = sizeof(chip_select_t);
        static constexpr uint8_t SIZE_OF_PACKET_T = sizeof(packet_t);
        static constexpr uint8_t SIZE_OF_CHANNEL_T = sizeof(channel_t);
        static constexpr uint8_t SIZE_OF_INIT_T = sizeof(init_t);
        static constexpr uint8_t SIZE_OF_HANDLE_T = sizeof(handle_t);

        static constexpr uint8_t SIZE_OF_USER_LIST = sizeof(user_list);

        /* public objects */
        handle_t*       module;
        id_number_t     next_available_user_channel_id = 0;
        uint8_t         send_state = 0;
        channel_t       active_channel;
        packet_t        active_packet;

        std::queue<packet_t> send_buffer;
        std::queue<packet_t> return_buffer_0;
        std::queue<packet_t> return_buffer_1;
        std::queue<packet_t> return_buffer_2;
        std::queue<packet_t> return_buffer_3;
        std::queue<packet_t> return_buffer_4;
        std::queue<packet_t> return_buffer_5;
        std::queue<packet_t> return_buffer_6;
        std::queue<packet_t> return_buffer_7;

        /* public methods */
        spi();
        void initialize(handle_t* _spi_handle, hal_spi_t* _spi_instance, callback_id_t _complete_callback_id, spi_callback_ptr_t _complete_callback_ptr, callback_id_t _error_callback_id, spi_callback_ptr_t _error_callback_ptr);
        spi::procedure_status_t create_channel(id_number_t& _channel_id, uint8_t _packet_size, uint8_t _tx_size, port_name_t _chip_select_port, uint16_t _chip_select_pin);
        procedure_status_t transmit(id_number_t _channel_id, uint8_t _total_byte_count, uint8_t _tx_size, const uint8_t* _tx_bytes);
        void process_send_buffer();
        uint8_t process_return_buffer(id_number_t _channel, uint8_t (&_rx_array)[SPI_BYTE_COUNT_MAX]);
        void get_channel_by_channel_id(channel_t& channel, id_number_t channel_id);
        friend void hal_callbacks_assert_spi_chip_select(spi::handle_t *_module);
        friend void hal_callbacks_deassert_spi_chip_select(spi::handle_t *_module);
        friend spi::handle_t* get_spi_handle(spi* spi_object);
        friend spi::procedure_status_t dma_abort_interrupt(dma_handle_t *dma_handle);
        friend void dma_abort_on_error(dma_handle_t *dma_handle);
        friend void spi_irq_handler(spi* spi_object);

        friend void spi_tx_2_line_8_bit_isr(spi spi_object, struct spi::_handle_t *spi_handle);
        friend void spi_rx_2_line_8_bit_isr(spi spi_object, struct spi::_handle_t *spi_handle);
        friend void spi_tx_2_line_16_bit_isr(spi spi_object, struct spi::_handle_t *spi_handle);
        friend void spi_rx_2_line_16_bit_isr(spi spi_object, struct spi::_handle_t *spi_handle);
        void close_rx_tx_isr();
        void close_rx_isr();
        void close_tx_isr();
        void abort_rx_isr();
        void abort_tx_isr();

    #if (SPI_USE_REGISTER_CALLBACKS == 1U)
        procedure_status_t spi_register_callback(callback_id_t callback_id, spi_callback_ptr_t pCallback) const;
        procedure_status_t spi_unregister_callback(callback_id_t callback_id) const;
    #endif

    private:

        procedure_status_t initialize_protocol(handle_t* _spi_handle, hal_spi_t* _spi_instance);
        procedure_status_t initialize_active_packet();
        procedure_status_t initialize_send_buffer();
        void initialize_return_buffer();
        procedure_status_t spi_transmit_receive_interrupt(uint8_t *rx_data_pointer, uint8_t _tx_index);
        void assert_chip_select() const;
        void deassert_chip_select() const;
        id_number_t assign_next_available_channel_id();

        void send_buffer_push(packet_t& packet);
        void send_buffer_pop();
        void send_buffer_get_front(packet_t& packet);
        void set_active_packet_from_send_buffer();
        void push_active_packet_to_return_buffer();
        procedure_status_t reset_active_packet();
        void reset_active_channel();
        uint8_t calculate_number_of_transmissions_for_active_packet() const;
        procedure_status_t lock_module() const;
        void unlock_module() const;
        void set_transaction_parameters(uint8_t *tx_data_ptr, uint8_t *rx_data_ptr, uint16_t packet_size) const;
        void set_rx_and_tx_interrupt_service_routines() const;
        comms_state_t get_module_communication_state() const;
        uint32_t get_module_operating_mode() const;
        void verify_communication_direction(uint32_t intended_direction) const;
        procedure_status_t wait_for_flag_until_timeout(uint32_t flag, flag_status_t flag_status, uint32_t flag_timeout, uint32_t start_time) const;
        procedure_status_t end_rx_transaction(uint32_t flag_timeout, uint32_t start_time);
        procedure_status_t end_rx_tx_transaction(uint32_t flag_timeout, uint32_t start_time);
        void reset_enabled_crc();
};

#endif //MAIN_CONTROLLER_HAL_SPI_H
