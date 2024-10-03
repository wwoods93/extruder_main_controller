/***********************************************************************************************************************
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
#include <memory>
/* stm32 includes */

/* third-party includes */

/* layer_0_hal includes */
#include "hal_wrapper.h"
#include "hal_spi_definitions.h"
/* layer_1_rtosal includes */
#include "../layer_1_rtosal/rtosal.h"
#include "../layer_1_rtosal/rtosal_wrapper.h"
/* layer_2_device includes */

/* layer_3_control includes */

/* layer_4_sys_op includes */

/* layer_n_meta_structure includes */


class spi
{
    public:

        #define TX_SIZE_MAX                                         8U
        static constexpr uint8_t MESSAGE_QUEUE_COUNT_MAX = 8U;

        static constexpr uint32_t   FLAG_TIMEOUT                    = 50U;
        static constexpr uint32_t   TRANSACTION_TIMEOUT             = 100U;
        static constexpr uint16_t   FALLBACK_COUNTDOWN              = 1000U;
        static constexpr uint8_t    SPI_PROCEDURE_ERROR_NONE        = 0U;
        static constexpr uint8_t    SPI_PROCEDURE_STATE_BUS_ERROR   = 1U;
        static constexpr uint8_t    SPI_PROCEDURE_STATE_DATA_ERROR  = 2U;

        static constexpr uint8_t CHANNEL_0 = 0x00U;
        static constexpr uint8_t CHANNEL_1 = 0x01U;
        static constexpr uint8_t CHANNEL_2 = 0x02U;
        static constexpr uint8_t CHANNEL_3 = 0x03U;
        static constexpr uint8_t CHANNEL_4 = 0x04U;
        static constexpr uint8_t CHANNEL_5 = 0x05U;
        static constexpr uint8_t CHANNEL_6 = 0x06U;
        static constexpr uint8_t CHANNEL_7 = 0x07U;



        typedef enum
        {
            MODULE_UNLOCKED                 = 0x00U,
            MODULE_LOCKED                   = 0x01U
        } lock_t;

        typedef enum
        {
            TX_RX                           = 0x00,
            TX_ONLY                         = 0x01,
            RX_ONLY                         = 0x02,
        }transaction_t;

        typedef enum
        {
            DATA_REG_ID                     = 0x00U,
            STATUS_REG_ID                   = 0x01U,
            CONTROL_REG_1_ID                = 0x02U,
            CONTROL_REG_2_ID                = 0x03U,
        } register_id_t;

        typedef enum
        {
            PROCEDURE_STATUS_OK             = 0x00U,
            PROCEDURE_STATUS_ERROR          = 0x01U,
            PROCEDURE_STATUS_BUSY           = 0x02U,
            PROCEDURE_STATUS_TIMEOUT        = 0x03U
        } procedure_status_t;

        typedef enum
        {
            MODULE_STATUS_RESET             = 0x00U,
            MODULE_STATUS_READY             = 0x01U,
            MODULE_STATUS_BUSY              = 0x02U,
            MODULE_STATUS_BUSY_TX           = 0x03U,
            MODULE_STATUS_BUSY_RX           = 0x04U,
            MODULE_STATUS_BUSY_TX_RX        = 0x05U,
            MODULE_STATUS_ERROR             = 0x06U,
            MODULE_STATUS_ABORT             = 0x07U
        } module_status_t;

        typedef enum
        {
            TX_COMPLETE_CALLBACK_ID         = 0x00U,
            RX_COMPLETE_CALLBACK_ID         = 0x01U,
            TX_RX_COMPLETE_CALLBACK_ID      = 0x02U,
            TX_HALF_COMPLETE_CALLBACK_ID    = 0x03U,
            RX_HALF_COMPLETE_CALLBACK_ID    = 0x04U,
            TX_RX_HALF_COMPLETE_CALLBACK_ID = 0x05U,
            ERROR_CALLBACK_ID               = 0x06U,
            ABORT_CALLBACK_ID               = 0x07U,
        } callback_id_t;

        typedef struct
        {
            hal::gpio_t *   port;
            uint16_t        pin;
        } chip_select_t;

        typedef struct
        {
            int16_t         channel_id;
            int16_t         packet_id;
            uint8_t         bytes_per_transaction[TX_SIZE_MAX];
            uint8_t         tx_bytes[TX_SIZE_MAX];
            uint8_t         rx_bytes[TX_SIZE_MAX];
            chip_select_t   chip_select;
        } packet_t;

        typedef struct
        {
            int16_t         channel_id;
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
            settings_t                  settings;
            volatile module_status_t    status;
            volatile uint32_t           error_code;
            chip_select_t               chip_select;
            uint8_t                     *tx_buffer_ptr;
            uint8_t                     *rx_buffer_ptr;
            volatile uint16_t           tx_transfer_counter;
            volatile uint16_t           rx_transfer_counter;
            void                        (*rx_isr_ptr)(spi arg_object, struct _handle_t *arg_module);
            void                        (*tx_isr_ptr)(spi arg_object, struct _handle_t *arg_module);
            uint8_t                     rx_data_ready_flag;
            lock_t                      lock;
            hal::gpio_t*                chip_select_port;
            uint16_t                    chip_select_pin;
            void (* callbacks[SPI_REGISTER_CALLBACK_COUNT]) (spi *arg_object);
        } module_t;

        typedef void (*spi_callback_ptr_t)(spi *arg_object);

        module_t*                   module;
        packet_t                    active_packet;
        packet_t                    pending_packet;
        TIM_HandleTypeDef*          timeout_time_base;
        uint32_t                    timeout_time_base_frequency = 0U;
        uint8_t                     rx_result[TX_SIZE_MAX] = {0, 0, 0, 0, 0, 0, 0, 0 };
        int16_t                     next_available_channel_id = 0U;
        int16_t                     next_available_packet_id = 0U;
        int16_t                     message_queue_count = 0U;
        uint32_t                    packets_requested_count = 0U;
        uint32_t                    packets_received_count = 0U;

        rtosal::message_queue_id_t message_queue_ids[MESSAGE_QUEUE_COUNT_MAX];
        std::queue<packet_t>        send_buffer;
        std::queue<packet_t>        pending_buffer;
        std::queue<packet_t>        return_buffer_0;
        std::queue<packet_t>        return_buffer_1;
        std::queue<packet_t>        return_buffer_2;
        std::queue<packet_t>        return_buffer_3;
        std::queue<packet_t>        return_buffer_4;
        std::queue<packet_t>        return_buffer_5;
        std::queue<packet_t>        return_buffer_6;
        std::queue<packet_t>        return_buffer_7;

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

        void register_intertask_message_queue(rtosal::message_queue_id_t arg_message_queue_id);
        void send_inter_task_transaction_result(rtosal::message_queue_id_t arg_message_queue_id, packet_t& arg_packet);
        void receive_inter_task_transaction_request(rtosal::message_queue_id_t arg_message_queue_id, common_packet_t& arg_tx_common_packet);



        procedure_status_t initialize(module_t* arg_module, uint8_t arg_instance_t, TIM_HandleTypeDef* arg_timeout_time_base, uint32_t arg_timeout_time_base_frequency);
        procedure_status_t register_callback(callback_id_t arg_callback_id, spi_callback_ptr_t arg_callback_ptr) const;
        [[nodiscard]] procedure_status_t unregister_callback(callback_id_t arg_callback_id) const;

        procedure_status_t create_channel(int16_t& arg_channel_id, hal::gpio_t* arg_chip_select_port, uint16_t arg_chip_select_pin);
        void get_channel_by_channel_id(channel_t& arg_channel, int16_t arg_channel_id);
        int16_t assign_next_available_channel_id();
        void process_send_buffer();
        spi::procedure_status_t create_packet_and_add_to_send_buffer(int16_t arg_channel_id, uint8_t arg_total_byte_count, uint8_t (&arg_tx_bytes)[8], uint8_t (&arg_bytes_per_transaction)[8]);
        void send_buffer_push(packet_t& arg_packet);
        void send_buffer_pop();
        void send_buffer_get_front(packet_t& arg_packet);
        void set_active_packet_from_send_buffer();
        void push_active_packet_to_return_buffer();
        void pending_buffer_push(packet_t& arg_packet);
        void pending_buffer_pop();
        void pending_buffer_get_front(spi::packet_t& arg_packet);
        void push_active_packet_to_pending_buffer();
        void push_pending_packet_to_return_buffer();
        void transmit_and_get_result(uint8_t arg_current_transaction_size, uint8_t* arg_tx_data);
        procedure_status_t spi_transmit_receive_interrupt(uint8_t *arg_tx_data_ptr, uint8_t *arg_rx_data_ptr, uint16_t arg_packet_size, hal::gpio_t* arg_chip_select_port, uint16_t arg_chip_select_pin);
        uint8_t process_return_buffer(packet_t& arg_packet, int16_t arg_channel, uint8_t (&arg_rx_array)[TX_SIZE_MAX]);
        procedure_status_t reset_active_packet();
        void chip_select_set_active(uint8_t arg_channel_id);
        void chip_select_set_inactive(uint8_t arg_channel_id);
        [[nodiscard]] uint32_t get_packets_requested_count() const;
        [[nodiscard]] uint32_t get_packets_received_count() const;
        void complete_transaction_tx_rx_success();

        friend void tx_2_line_8_bit_isr(spi arg_object, struct spi::_handle_t *arg_module);
        friend void rx_2_line_8_bit_isr(spi arg_object, struct spi::_handle_t *arg_module);
        friend void tx_2_line_16_bit_isr(spi arg_object, struct spi::_handle_t *arg_module);
        friend void rx_2_line_16_bit_isr(spi arg_object, struct spi::_handle_t *arg_module);
        friend void spi_irq_handler(spi* arg_object);

    private:

        void set_tx_and_rx_interrupt_service_routines() const;
        [[nodiscard]] procedure_status_t verify_communication_direction(uint32_t arg_intended_direction) const;
        void set_transaction_parameters(uint8_t *arg_tx_data_ptr, uint8_t *arg_rx_data_ptr, uint16_t arg_packet_size) const;
        procedure_status_t wait_for_pending_flags_and_end_transaction(transaction_t arg_transaction_type);
        [[nodiscard]] procedure_status_t flag_timeout(uint32_t arg_status_reg_bit, bit_status_t arg_bit_status) const;
        void close_isr(transaction_t arg_transaction_type);

        [[nodiscard]] procedure_status_t lock_module() const;
        [[nodiscard]] procedure_status_t unlock_module() const;
        void enable_module() const;
        void disable_module() const;
        void enable_interrupts(uint32_t arg_interrupts) const;
        void disable_interrupts(uint32_t arg_interrupts) const;
        [[nodiscard]] bit_status_t check_interrupt_source(uint32_t arg_interrupt) const;
        void set_bit_spi_register_32(register_id_t arg_register, uint32_t arg_bit) const;
        void clear_bit_spi_register_32(register_id_t arg_register, uint32_t arg_bit) const;
        [[nodiscard]] bit_status_t get_status_register_bit(uint32_t arg_bit) const;
        void set_error_bit(uint32_t arg_bit) const;
        void clear_mode_fault_flag() const;
        void clear_overrun_flag() const;
};

inline spi::procedure_status_t spi::lock_module() const
{
    if (module->lock == MODULE_LOCKED) { return PROCEDURE_STATUS_BUSY; }
    module->lock = MODULE_LOCKED;
    return PROCEDURE_STATUS_OK;
}

inline spi::procedure_status_t spi::unlock_module() const
{
    module->lock = MODULE_UNLOCKED;
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
