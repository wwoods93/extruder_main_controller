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
#include "hal_spi_definitions.h"
#include "hal_general.h"
/* driver includes */

/* rtos abstraction includes */
#include "../layer_2_rtosal/rtosal_spi_shared_resources.h"
/* sys op includes */

/* meta structure includes */
#include "../meta_structure/meta_structure_system_manager.h"
#include "../meta_structure/meta_structure_resource.h"



    /* macros */

    #define STM_HAL_DMA_ENABLE(__HANDLE__)                      ((__HANDLE__)->instance->CONFIG_REG |=  STM_HAL_DMA_SxCR_ENABLE)
    #define STM_HAL_DMA_DISABLE(__HANDLE__)                     ((__HANDLE__)->instance->CONFIG_REG &=  ~STM_HAL_DMA_SxCR_ENABLE)
    #define SPI_ENABLE_MODULE(__HANDLE__)                       REGISTER_SET_BIT((__HANDLE__)->instance->CONTROL_REG_1, SPI_CR1_BIT_SPI_ENABLE)
    #define SPI_DISABLE_MODULE(__HANDLE__)                      REGISTER_CLEAR_BIT((__HANDLE__)->instance->CONTROL_REG_1, SPI_CR1_BIT_SPI_ENABLE)
    #define SPI_ENABLE_INTERRUPTS(__HANDLE__, __INTERRUPT__)    REGISTER_SET_BIT((__HANDLE__)->instance->CONTROL_REG_2, (__INTERRUPT__))
    #define SPI_DISABLE_INTERRUPTS(__HANDLE__, __INTERRUPT__)   REGISTER_CLEAR_BIT((__HANDLE__)->instance->CONTROL_REG_2, (__INTERRUPT__))
    #define SPI_GET_STATUS_REG_BIT(__HANDLE__, __BIT__)         ((((__HANDLE__)->instance->STATUS_REG) & (__BIT__)) == (__BIT__))
    #define SPI_CHECK_FLAG_STATUS(__SR__, __FLAG__)             ((((__SR__) & ((__FLAG__) & SPI_FLAG_MASK)) == ((__FLAG__) & SPI_FLAG_MASK)) ? FLAG_SET : FLAG_RESET)
    #define SPI_VERIFY_DIRECTION_2_LINE(__MODE__)               ((__MODE__) == SPI_DIRECTION_2_LINE)
    #define SPI_VERIFY_DIRECTION_2_LINE_RX_ONLY(__MODE__)       ((__MODE__) == SPI_DIRECTION_2_LINE_RX_ONLY)
    #define SPI_VERIFY_DIRECTION_1_LINE(__MODE__)               ((__MODE__) == SPI_DIRECTION_1_LINE)
    #define SPI_CHECK_INTERRUPT_SOURCE(__CR2__, __INTERRUPT__)  ((((__CR2__) & (__INTERRUPT__)) == (__INTERRUPT__)) ? FLAG_SET : FLAG_RESET)
    #define SPI_CLEAR_CRC_ERROR(__HANDLE__)                     ((__HANDLE__)->instance->DATA_REG = (uint16_t)(~SPI_SR_BIT_CRC_ERROR))

class spi : public resource
{
    public:

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
            GPIO_TypeDef*   port;
            uint16_t        pin;
        } chip_select_t;

        typedef struct
        {
            id_number_t     channel_id;
            uint8_t         tx_bytes[SPI_BYTE_COUNT_MAX];
            uint8_t         rx_bytes[SPI_BYTE_COUNT_MAX];
        } packet_t;

        typedef struct
        {
            id_number_t     channel_id;
            uint8_t         packet_size;
            uint8_t         tx_size;
            chip_select_t   chip_select;
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
            void (* callbacks[SPI_REGISTER_CALLBACK_COUNT]) (struct _handle_t *spi_handle);

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

        handle_t*       module;
        id_number_t     next_available_user_channel_id = 0;
        uint8_t         send_state = 0;
        channel_t       active_channel;
        packet_t        active_packet;
        uint8_t         use_crc = 0;

        std::queue<packet_t> send_buffer;
        std::queue<packet_t> return_buffer_0;
        std::queue<packet_t> return_buffer_1;
        std::queue<packet_t> return_buffer_2;
        std::queue<packet_t> return_buffer_3;
        std::queue<packet_t> return_buffer_4;
        std::queue<packet_t> return_buffer_5;
        std::queue<packet_t> return_buffer_6;
        std::queue<packet_t> return_buffer_7;

        spi();
        void initialize(handle_t* arg_module, hal_spi_t* arg_instance, uint8_t arg_use_crc, callback_id_t arg_complete_callback_id, spi_callback_ptr_t arg_complete_callback_ptr, callback_id_t arg_error_callback_id, spi_callback_ptr_t arg_error_callback_ptr);
        spi::procedure_status_t create_channel(id_number_t& arg_channel_id, uint8_t arg_packet_size, uint8_t arg_tx_size, port_name_t arg_chip_select_port, uint16_t arg_chip_select_pin);
        void get_channel_by_channel_id(channel_t& arg_channel, id_number_t arg_channel_id);
        procedure_status_t transmit(id_number_t arg_channel_id, uint8_t arg_total_byte_count, uint8_t arg_tx_size, const uint8_t* arg_tx_bytes);
        void process_send_buffer();
        uint8_t process_return_buffer(id_number_t arg_channel, uint8_t (&arg_rx_array)[SPI_BYTE_COUNT_MAX]);
        friend void hal_callbacks_assert_spi_chip_select(spi::handle_t *arg_module);
        friend void hal_callbacks_deassert_spi_chip_select(spi::handle_t *arg_module);
        friend void spi_rx_2_line_8_bit_isr_with_crc(spi arg_object, struct spi::_handle_t *arg_module);
        friend void spi_rx_2_line_16_bit_isr_with_crc(spi arg_object, struct spi::_handle_t *arg_module);
        friend void spi_rx_1_line_8_bit_isr_with_crc(spi arg_object, struct spi::_handle_t *arg_module);
        friend void spi_rx_1_line_16_bit_isr_with_crc(spi arg_object, struct spi::_handle_t *arg_module);
        friend void spi_tx_2_line_8_bit_isr(spi arg_object, struct spi::_handle_t *arg_module);
        friend void spi_rx_2_line_8_bit_isr(spi arg_object, struct spi::_handle_t *arg_module);
        friend void spi_tx_2_line_16_bit_isr(spi arg_object, struct spi::_handle_t *arg_module);
        friend void spi_rx_2_line_16_bit_isr(spi arg_object, struct spi::_handle_t *arg_module);
        friend spi::procedure_status_t dma_abort_interrupt(dma_handle_t *arg_dma_handle);
        friend void dma_abort_on_error(dma_handle_t *arg_dma_handle);
        friend void spi_irq_handler(spi* arg_active_object);

    private:

        procedure_status_t initialize_protocol(handle_t* arg_module, hal_spi_t* arg_resource_instance);
        procedure_status_t initialize_active_packet();
        procedure_status_t initialize_send_buffer();
        void initialize_return_buffer();
        procedure_status_t spi_transmit_receive_interrupt(uint8_t *arg_rx_data_ptr, uint8_t arg_tx_index);
        void assert_chip_select() const;
        void deassert_chip_select() const;
        id_number_t assign_next_available_channel_id();
        void send_buffer_push(packet_t& arg_packet);
        void send_buffer_pop();
        void send_buffer_get_front(packet_t& arg_packet);
        void set_active_packet_from_send_buffer();
        void push_active_packet_to_return_buffer();
        procedure_status_t reset_active_packet();
        void reset_active_channel();
        uint8_t calculate_number_of_transmissions_for_active_packet() const;
        procedure_status_t spi_register_callback(callback_id_t arg_callback_id, spi_callback_ptr_t arg_callback_ptr) const;
        procedure_status_t spi_unregister_callback(callback_id_t arg_callback_id) const;
        void close_tx_rx_isr();
        void close_tx_isr();
        void close_rx_isr();
        procedure_status_t end_rx_transaction(uint32_t arg_timeout);
        procedure_status_t end_tx_rx_transaction(uint32_t arg_timeout);
        void abort_tx_isr();
        void abort_rx_isr();
        procedure_status_t lock_module() const;
        void unlock_module() const;
        void set_transaction_parameters(uint8_t *arg_tx_data_ptr, uint8_t *arg_rx_data_ptr, uint16_t arg_packet_size) const;
        void set_rx_and_tx_interrupt_service_routines() const;
        comms_state_t get_module_communication_state() const;
        uint32_t get_module_operating_mode() const;
        void verify_communication_direction(uint32_t arg_intended_direction) const;
        procedure_status_t wait_for_status_register_bit(uint32_t arg_bit, bit_status_t arg_status, uint32_t arg_timeout) const;

        void reset_enabled_crc() const;
        void clear_mode_fault_flag() const;
        void clear_overrun_flag() const;
        void clear_ti_frame_format_error_flag() const;
};


inline void spi::clear_mode_fault_flag() const
{
    UNUSED_CAST_VOID(REGISTER_READ(module->instance->STATUS_REG));
    REGISTER_CLEAR_BIT(module->instance->CONTROL_REG_1, SPI_CR1_BIT_SPI_ENABLE);
}

inline void spi::clear_overrun_flag() const
{
    UNUSED_CAST_VOID(REGISTER_READ(module->instance->DATA_REG));
    UNUSED_CAST_VOID(REGISTER_READ(module->instance->STATUS_REG));
}

inline void spi::clear_ti_frame_format_error_flag() const
{
    UNUSED_CAST_VOID(REGISTER_READ(module->instance->STATUS_REG));
}

#endif //MAIN_CONTROLLER_HAL_SPI_H
