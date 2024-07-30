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
#include <queue>
#include "stm32f4xx.h"
#include "hal_general.h"
#include "hal_spi_definitions.h"
#include "../meta_structure/meta_structure_system_manager.h"

class spi
{
    public:

        #define TX_SIZE_MAX                     8U


        /* type definitions */

        typedef enum
        {
            DATA_REG_ID                         = 0x00U,
            STATUS_REG_ID                       = 0x01U,
            CONTROL_REG_1_ID                    = 0x02U,
            CONTROL_REG_2_ID                    = 0x03U,
        } register_id_t;

        typedef enum
        {
            PROCEDURE_STATUS_OK                       = 0x00U,
            PROCEDURE_STATUS_ERROR                    = 0x01U,
            PROCEDURE_STATUS_BUSY                     = 0x02U,
            PROCEDURE_STATUS_TIMEOUT                  = 0x03U
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
            MODULE_STATUS_ABORT                     = 0x07U
        } module_status_t;

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

        typedef struct
        {
            GPIO_TypeDef*   port;
            uint16_t        pin;
        } chip_select_t;

        typedef struct
        {
            id_number_t     channel_id;
            id_number_t     packet_id;
            uint8_t         total_byte_count;
            uint8_t         tx_byte_count;
            uint8_t         bytes_per_tx[TX_SIZE_MAX];
            uint8_t         tx_bytes[TX_SIZE_MAX];
            uint8_t         rx_bytes[TX_SIZE_MAX];
            chip_select_t   chip_select;
        } packet_t;

        typedef struct
        {
            id_number_t     channel_id;
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
        } settings_t;

        typedef struct _handle_t
        {
            hal_spi_t                   *instance;
            chip_select_t               chip_select;
            settings_t                  settings;
            uint8_t                     *tx_buffer_ptr;
            uint8_t                     *rx_buffer_ptr;
            volatile uint16_t           tx_transfer_counter;
            volatile uint16_t           rx_transfer_counter;
            void                        (*rx_isr_ptr)(spi spi_object, struct _handle_t *spi_handle);
            void                        (*tx_isr_ptr)(spi spi_object, struct _handle_t *spi_handle);
            hal_lock_t                  lock;
            volatile module_status_t    status;
            volatile uint32_t           error_code;
            GPIO_TypeDef*               chip_select_port;
            uint16_t                    chip_select_pin;

            void (* callbacks[SPI_REGISTER_CALLBACK_COUNT]) (struct _handle_t *spi_handle);
        } module_t;

        /* public member variables */
        module_t* module;
        uint8_t rx_result[TX_SIZE_MAX] = {0, 0, 0, 0, 0, 0, 0, 0 };
        id_number_t     next_available_channel_id = 0;
        packet_t        active_packet;
        uint8_t         send_state = 0;
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
        } channel_list;

        typedef void (*spi_callback_ptr_t)(module_t* spi_module_handle);

        /* public member functions */
        procedure_status_t initialize(module_t* arg_module, hal_spi_t* arg_instance);
        procedure_status_t spi_register_callback(callback_id_t _callback_id, spi_callback_ptr_t _callback_ptr) const;
        [[nodiscard]] procedure_status_t spi_unregister_callback(callback_id_t _callback_id) const;
//        void initialize_spi_buffer();
        procedure_status_t create_channel(id_number_t& arg_channel_id, port_name_t arg_chip_select_port, uint16_t arg_chip_select_pin);
        void get_channel_by_channel_id(channel_t& arg_channel, id_number_t arg_channel_id);
        id_number_t assign_next_available_channel_id();
        void process_send_buffer();
        spi::procedure_status_t create_packet_and_add_to_send_buffer(id_number_t arg_channel_id, uint8_t arg_total_byte_count, uint8_t arg_tx_byte_count, uint8_t (&arg_tx_bytes)[8], uint8_t (&arg_bytes_per_tx)[8]);
        void send_buffer_push(packet_t& arg_packet);
        void send_buffer_pop();
        void send_buffer_get_front(packet_t& arg_packet);
        void set_active_packet_from_send_buffer();
        void push_active_packet_to_return_buffer();
        void transmit_and_get_result(uint8_t packet_size, uint8_t* tx_data);
        procedure_status_t spi_transmit_receive_interrupt(uint8_t *tx_data_pointer, uint8_t *rx_data_pointer, uint16_t packet_size, GPIO_TypeDef* chip_select_port, uint16_t chip_select_pin);
        uint8_t process_return_buffer(packet_t& packet, id_number_t arg_channel, uint8_t (&arg_rx_array)[TX_SIZE_MAX]);
        procedure_status_t reset_active_packet();

        friend void spi_tx_2_line_8_bit_isr(spi spi_object, struct spi::_handle_t *spi_handle);
        friend void spi_rx_2_line_8_bit_isr(spi spi_object, struct spi::_handle_t *spi_handle);
        friend void spi_tx_2_line_16_bit_isr(spi spi_object, struct spi::_handle_t *spi_handle);
        friend void spi_rx_2_line_16_bit_isr(spi spi_object, struct spi::_handle_t *spi_handle);

        friend spi::module_t* get_spi_handle(spi* spi_object);
        friend void spi_irq_handler(spi* spi_object);

        friend void HAL_SPI_TxCpltCallback(spi::module_t *spi_handle);
        friend void HAL_SPI_RxCpltCallback(spi::module_t *spi_handle);
        friend void HAL_SPI_TxRxCpltCallback(spi::module_t *spi_handle);
        friend void HAL_SPI_TxHalfCpltCallback(spi::module_t *spi_handle);
        friend void HAL_SPI_RxHalfCpltCallback(spi::module_t *spi_handle);
        friend void HAL_SPI_TxRxHalfCpltCallback(spi::module_t *spi_handle);
        friend void HAL_SPI_ErrorCallback(spi::module_t *spi_handle);
        friend void HAL_SPI_AbortCpltCallback(spi::module_t *spi_handle);

    private:

        void set_rx_and_tx_interrupt_service_routines() const;
//        [[nodiscard]] module_status_t get_module_communication_state() const;
//        [[nodiscard]] uint32_t get_module_operating_mode() const;
        [[nodiscard]] procedure_status_t verify_communication_direction(uint32_t intended_direction) const;
        void set_transaction_parameters(uint8_t *tx_data_ptr, uint8_t *rx_data_ptr, uint16_t packet_size) const;
        [[nodiscard]] procedure_status_t wait_for_flag_until_timeout(uint32_t arg_status_reg_bit, bit_status_t arg_bit_status, uint32_t arg_timeout, uint32_t arg_start_time) const;
        procedure_status_t end_rx_transaction(uint32_t arg_timeout, uint32_t arg_start_time);
        procedure_status_t end_rx_tx_transaction(uint32_t arg_timeout, uint32_t arg_start_time);
        void close_rx_tx_isr();
        void close_rx_isr();
        void close_tx_isr();
        void abort_rx_isr();
        void abort_tx_isr();
        [[nodiscard]] procedure_status_t lock_module() const;
        [[nodiscard]] procedure_status_t unlock_module() const;
        void enable_module() const;
        void disable_module() const;
        void enable_interrupts(uint32_t arg_interrupts) const;
        void disable_interrupts(uint32_t arg_interrupts) const;

        void set_bit_spi_register_32(register_id_t arg_register, uint32_t arg_bit) const;
        void clear_bit_spi_register_32(register_id_t arg_register, uint32_t arg_bit) const;
        [[nodiscard]] bit_status_t get_status_register_bit(uint32_t arg_bit) const;

        void set_error_bit(uint32_t arg_bit) const;

        [[nodiscard]] bit_status_t check_interrupt_source(uint32_t arg_interrupt) const;

        void clear_mode_fault_flag() const;
        void clear_overrun_flag() const;

};

inline spi::procedure_status_t spi::lock_module() const
{
    if (module->lock == HAL_MODULE_LOCKED) { return PROCEDURE_STATUS_BUSY; }
    module->lock = HAL_MODULE_LOCKED;
    return PROCEDURE_STATUS_OK;
}

inline spi::procedure_status_t spi::unlock_module() const
{
    module->lock = HAL_MODULE_UNLOCKED;
    return PROCEDURE_STATUS_OK;
}

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

inline void spi::disable_interrupts(uint32_t arg_interrupts) const
{
    module->instance->CONTROL_REG_2 &= (~arg_interrupts);
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





inline void spi::clear_mode_fault_flag() const
{
    uint32_t register_contents = module->instance->STATUS_REG;
    UNUSED_CAST_VOID(register_contents);
    clear_bit_spi_register_32(CONTROL_REG_1_ID, SPI_CR1_BIT_SPI_ENABLE);
}

inline void spi::clear_overrun_flag() const
{
    uint32_t data_reg_contents = module->instance->DATA_REG;
    uint32_t status_reg_contents = module->instance->STATUS_REG;
    UNUSED_CAST_VOID(data_reg_contents);
    UNUSED_CAST_VOID(status_reg_contents);
}

#endif //MAIN_CONTROLLER_HAL_SPI_H
