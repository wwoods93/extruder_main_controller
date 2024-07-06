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


class spi : public resource
{
    public:

        typedef enum
        {
            PROCEDURE_STATUS_OK                 = 0x00U,
            PROCEDURE_STATUS_ERROR              = 0x01U,
            PROCEDURE_STATUS_BUSY               = 0x02U,
            PROCEDURE_STATUS_TIMEOUT            = 0x03U,
            PROCEDURE_STATUS_DATA_ERROR         = 0x04U,
            PROCEDURE_STATUS_BUS_ERROR          = 0x05U,
        } procedure_status_t;

        typedef enum
        {
            MODULE_STATUS_RESET                     = 0x00U,
            MODULE_STATUS_READY                     = 0x01U,
            MODULE_STATUS_BUSY                      = 0x02U,
            MODULE_STATUS_BUSY_TX                   = 0x03U,
            MODULE_STATUS_BUSY_RX                   = 0x04U,
            MODULE_STATUS_BUSY_TX_RX                = 0x05U,
            MODULE_STATUS_ERROR                     = 0x06U,
            MODULE_STATUS_ABORT                     = 0x07U,
        } module_status_t;

        typedef enum
        {
            DATA_REG_ID                         = 0x00U,
            STATUS_REG_ID                       = 0x01U,
            CONTROL_REG_1_ID                    = 0x02U,
            CONTROL_REG_2_ID                    = 0x03U,
            ERROR_CODE_BIT_FIELD_ID             = 0x04U,
        } register_id_t;

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
            uint8_t         tx_size;
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
            volatile module_status_t    state;
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
        uint8_t         chip_select_set = 0;

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
        procedure_status_t create_channel(id_number_t& arg_channel_id, uint8_t arg_packet_size, uint8_t arg_tx_size, port_name_t arg_chip_select_port, uint16_t arg_chip_select_pin);
        void get_channel_by_channel_id(channel_t& arg_channel, id_number_t arg_channel_id);
        procedure_status_t create_packet_and_add_to_send_buffer(id_number_t arg_channel_id, uint8_t arg_total_byte_count, uint8_t arg_tx_size, const uint8_t* arg_tx_bytes);
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
        friend procedure_status_t dma_abort_interrupt(spi* arg_object, dma_handle_t *arg_dma_handle);
        friend void dma_abort_on_error(dma_handle_t *arg_dma_handle);
        void spi_irq_handler();

    private:

        procedure_status_t initialize_protocol(handle_t* arg_module, hal_spi_t* arg_resource_instance);
        procedure_status_t initialize_active_packet();
        procedure_status_t initialize_send_buffer();
        void initialize_return_buffer();
        procedure_status_t spi_transmit_receive_interrupt(uint8_t *arg_rx_data_ptr, uint8_t arg_tx_index);
        void assert_chip_select();
        void deassert_chip_select();
        id_number_t assign_next_available_channel_id();
        void send_buffer_push(packet_t& arg_packet);
        void send_buffer_pop();
        void send_buffer_get_front(packet_t& arg_packet);
        void set_active_packet_from_send_buffer();
        void push_active_packet_to_return_buffer();
        procedure_status_t reset_active_packet();
        void reset_active_channel();
        [[nodiscard]] uint8_t calculate_number_of_transmissions_for_active_packet() const;
        procedure_status_t register_callback(callback_id_t arg_callback_id, spi_callback_ptr_t arg_callback_ptr) const;
        [[nodiscard]] procedure_status_t unregister_callback(callback_id_t arg_callback_id) const;
        void close_tx_rx_isr();
        void close_tx_isr();
        void close_rx_isr();
        procedure_status_t end_rx_transaction(uint32_t arg_timeout);
        procedure_status_t end_tx_rx_transaction(uint32_t arg_timeout);
        void abort_tx_isr();
        void abort_rx_isr();
        [[nodiscard]] procedure_status_t lock_module() const;
        void unlock_module() const;
        void set_transaction_parameters_for_interrupt_tx_rx(uint8_t *arg_tx_data_ptr, uint8_t *arg_rx_data_ptr, uint16_t arg_packet_size) const;
        void set_rx_and_tx_interrupt_service_routines() const;
        [[nodiscard]] module_status_t get_module_communication_state() const;
        [[nodiscard]] uint32_t get_module_operating_mode() const;
        void verify_communication_direction(uint32_t arg_intended_direction) const;
        [[nodiscard]] procedure_status_t wait_for_status_register_bit(uint32_t arg_bit, bit_status_t arg_bit_status, uint32_t arg_timeout) const;

        void set_bit_spi_register_32(register_id_t arg_register, uint32_t arg_bit) const;
        void clear_bit_spi_register_32(register_id_t arg_register, uint32_t arg_bit) const;
        [[nodiscard]] bit_status_t get_status_register_bit(uint32_t arg_bit) const;
        void enable_module() const;
        void disable_module() const;
        void set_error_bit(uint32_t arg_bit) const;
        void enable_interrupts(uint32_t arg_interrupts) const;
        [[nodiscard]] bit_status_t check_interrupt_source(uint32_t arg_interrupt) const;
        void disable_interrupts(uint32_t arg_interrupts) const;
        void reset_enabled_crc() const;
        void clear_mode_fault_flag() const;
        void clear_overrun_flag() const;
        void clear_ti_frame_format_error_flag() const;
        void clear_crc_error() const;
        static void enable_dma(dma_handle_t* arg_dma_module);
        static void disable_dma(dma_handle_t* arg_dma_module);

};


inline void spi::set_bit_spi_register_32(register_id_t arg_register, uint32_t arg_bit) const
{
    switch(arg_register)
    {
        case CONTROL_REG_1_ID:
        {
            module->instance->CONTROL_REG_1 |= arg_bit;
            break;
        }
        case CONTROL_REG_2_ID:
        {
            module->instance->CONTROL_REG_2 |= arg_bit;
            break;
        }
        case ERROR_CODE_BIT_FIELD_ID:
        {

        }
        default:
        {
            break;
        }
    }
}

inline void spi::clear_bit_spi_register_32(register_id_t arg_register, uint32_t arg_bit) const
{
    switch(arg_register)
    {
        case STATUS_REG_ID:
        {
            module->instance->STATUS_REG &= (~arg_bit);
            break;
        }
        case CONTROL_REG_1_ID:
        {
            module->instance->CONTROL_REG_1 &= (~arg_bit);
            break;
        }
        case CONTROL_REG_2_ID:
        {
            module->instance->CONTROL_REG_2 &= (~arg_bit);
            break;
        }
        default:
        {
            break;
        }
    }
}

inline bit_status_t spi::get_status_register_bit(uint32_t arg_bit) const
{
    bit_status_t bit_status = BIT_CLEAR;
    if ((module->instance->STATUS_REG & arg_bit & SPI_SR_BITS_MASK) == (arg_bit & SPI_SR_BITS_MASK))
    {
        bit_status = BIT_SET;
    }
    return bit_status;
}

inline void spi::enable_module() const
{
    module->instance->CONTROL_REG_1 |= SPI_CR1_BIT_SPI_ENABLE;
}

inline void spi::disable_module() const
{
    clear_bit_spi_register_32(CONTROL_REG_1_ID, SPI_CR1_BIT_SPI_ENABLE);
}

inline void spi::set_error_bit(uint32_t arg_bit) const
{
    module->error_code |= arg_bit;
}

inline void spi::enable_interrupts(uint32_t arg_interrupts) const
{
    module->instance->CONTROL_REG_2 |= arg_interrupts;
}

inline bit_status_t spi::check_interrupt_source(uint32_t arg_interrupt) const
{
    bit_status_t bit_status = BIT_CLEAR;
    if ((module->instance->CONTROL_REG_2 & arg_interrupt) == arg_interrupt)
    {
        bit_status = BIT_SET;
    }

    return bit_status;
}

inline void spi::disable_interrupts(uint32_t arg_interrupts) const
{
    module->instance->CONTROL_REG_2 &= (~arg_interrupts);
}

inline void spi::clear_mode_fault_flag() const
{
    uint32_t register_contents = module->instance->STATUS_REG;
    UNUSED_CAST_VOID(register_contents);
    clear_bit_spi_register_32(CONTROL_REG_1_ID, SPI_CR1_BIT_SPI_ENABLE);
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

inline void spi::clear_crc_error() const
{
    module->instance->STATUS_REG = (uint16_t)(SPI_SR_BIT_CRC_ERROR);
}

inline void spi::enable_dma(dma_handle_t* arg_dma_module)
{
    arg_dma_module->instance->STREAM_CONFIG_REG |= SPI_DMA_CONFIG_REG_BIT_ENABLE;
}

inline void spi::disable_dma(dma_handle_t* arg_dma_module)
{
    arg_dma_module->instance->STREAM_CONFIG_REG &= (~SPI_DMA_CONFIG_REG_BIT_ENABLE);
}

#endif //MAIN_CONTROLLER_HAL_SPI_H
