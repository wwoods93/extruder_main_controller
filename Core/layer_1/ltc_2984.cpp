/***********************************************************************************************************************
 * Main_Controller
 * ltc_2984.cpp
 *
 * wilson
 * 2/13/25
 * 8:30 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

/* c/c++ includes */
#include <cstdint>
/* stm32 includes */
#include "../layer_0/hal.h"
#include "../layer_0/rtosal.h"
/* third-party includes */

/* layer_0_hal includes */

/* layer_1 includes */

/* layer_2 includes */

/* layer_3 includes */

/* application includes */

/* ltc_2984 header */
#include "ltc_2984.h"


/*
 * 1. start up
 *  a. apply power (2.85V < Vdd <  5.25V, low threshold = 2.6V)
 *  b. wait for 200ms wakeup period
 *  c. check INTERRUPT pin state == 1
 *  d. check command status reg == 0x40
 *      (start bit = 0; done bit = 1)
 *  e. re-enter start up state any time during normal operation by setting RESET pin LOW
 *  f. notes:
 *      - command status register inaccessible to user during first phase (1st 100ms) of start up, after this
 *        command status register will be accessible and will return 0x80 until LTC2984 is fully initialized
 *      - once fully initialized command status register will return 0x40
 *
 * 2. channel assignment
 *
 *  a. write channel assignment word into corresponding channel assignment data RAM location
 *  b. notes:
 *      - unused channels should have their channel assignment data set to all zeros (this is default at start up)
 *
 * 3. initiate conversion
 *
 *  a. write measurement command into RAM memory location 0x000
 *      i.      conversion initiated by writing 1 to the start bit ([7] = 1) and 0 to the done bit ([6] = 0) of the command status
 *              register, followed by the desired input channel ([4:0] = input channel # (0-20) in binary), [5] always = 0
 *  b. notes:
 *      - sleep initiated by writing 0x97 to the command status register
 *      - it is possible to initiate measurement on multiple channels by setting command status register [4:0] = 0b00000
 *
 * 4. conversion
 *
 *  a. new conversion begins automatically following initiate conversion command
 *      i.      user locked out of RAM during this time, except for location 0x000
 *      ii.     input sensor and sense resistor/cold junction temp read simultaneously
 *  b. conversion takes either 2 or 3 cycles depending on configuration
 *      i. cycle time = 82ms
 *      ii.     2 cycle conversion corresponds to conversion rate = 167ms
 *      iii.    3 cycle conversion corresponds to conversion rate = 251ms
 *  c. conversion complete indicated by either
 *      i.      INTERRUPT pin  state == 1
 *      ii.     status reg START bit == 0 AND status reg DONE  bit == 1
 *
 * 5. read results
 *
 *  a. read result [23:0] and fault status bits [31:24]
 *  b. modify/add channel assignment data if needed
 *
 *
 *
 *
 */

ltc_2984::ltc_2984()
{
    spi_instance = nullptr;
    spi_channel = ID_INVALID;
    chip_select.port = nullptr;
    chip_select.pin = 0xFFFFU;
    timer_handle = nullptr;

    for (uint8_t i = 0U; i < CHANNEL_COUNT; ++i)
    {
        channel_assignment_words[i] = (uint32_t)0x00000000U;
    }
}

void ltc_2984::initialize(spi& arg_spi_instance, int16_t arg_spi_channel_id, hal::gpio_t* arg_chip_select_port, uint16_t arg_chip_select_pin, hal::timer_handle_t* arg_timer_handle)
{
    spi_instance = &arg_spi_instance;
    spi_channel = arg_spi_channel_id;
    chip_select.port = arg_chip_select_port;
    chip_select.pin = arg_chip_select_pin;
    timer_handle = arg_timer_handle;
}

void ltc_2984::rtd_channel_assign(channel_id_e arg_ch_id, uint32_t arg_rtd_type, uint32_t arg_resistor_ptr,
                                  uint32_t arg_num_wires, uint32_t arg_exc_mode, uint32_t arg_exc_current,
                                  uint32_t arg_rtd_curve, uint32_t arg_custom_rtd, uint32_t arg_custom_rtd_len)
{

     uint32_t channel_assignment_word = arg_rtd_type | arg_resistor_ptr | arg_num_wires |
                                          arg_exc_mode | arg_exc_current | arg_rtd_curve |
                                          arg_custom_rtd | arg_custom_rtd_len;

    channel_assignment_words[arg_ch_id] = channel_assignment_word;
    channel_assignment_words[arg_ch_id] = (uint32_t)0x00000000U;

    uint16_t channel_config_data_addr = get_channel_config_data_address(arg_ch_id);

    spi::packet_t packet;

    for (uint8_t i = 0; i < 8U; ++i)
    {
        packet.tx_bytes[i] = 0x00U;
        packet.rx_bytes[i] = 0x00U;
        packet.bytes_per_transaction[i] = 0x00U;
    }

    packet.packet_id = ID_INVALID;
    packet.channel_id = spi_channel;
    packet.chip_select.port = chip_select.port;
    packet.chip_select.pin = chip_select.pin;
    packet.bytes_per_transaction[0] = SPI_PACKET_NUM_BYTES;

    packet.tx_bytes[0] = SPI_INSTRUCTION_WRITE_RAM;
    packet.tx_bytes[1] = (uint8_t)((channel_config_data_addr >> 8U) & 0xFFU);
    packet.tx_bytes[2] = (uint8_t)(channel_config_data_addr & 0xFFU);
    packet.tx_bytes[3] = (uint8_t)((channel_assignment_word >> 24U) & 0xFFU);
    packet.tx_bytes[4] = (uint8_t)((channel_assignment_word >> 16U) & 0xFFU);
    packet.tx_bytes[5] = (uint8_t)((channel_assignment_word >> 8U) & 0xFFU);
    packet.tx_bytes[6] = (uint8_t)(channel_assignment_word & 0xFFU);
    spi_instance->transmit_receive(packet);
}

void ltc_2984::sense_resistor_channel_assign(channel_id_e arg_ch_id) const
{
    uint16_t channel_config_data_addr = get_channel_config_data_address(arg_ch_id);
    uint32_t resistance_value = SENSE_RESISTOR_VALUE_10K;

    spi::packet_t packet;

    for (uint8_t i = 0; i < 8U; ++i)
    {
        packet.tx_bytes[i] = 0x00U;
        packet.rx_bytes[i] = 0x00U;
        packet.bytes_per_transaction[i] = 0x00U;
    }

    packet.packet_id = ID_INVALID;
    packet.channel_id = spi_channel;
    packet.chip_select.port = chip_select.port;
    packet.chip_select.pin = chip_select.pin;
    packet.bytes_per_transaction[0] = SPI_PACKET_NUM_BYTES;

    packet.tx_bytes[0] = SPI_INSTRUCTION_WRITE_RAM;
    packet.tx_bytes[1] = (uint8_t)((channel_config_data_addr >> 8U) & 0xFFU);
    packet.tx_bytes[2] = (uint8_t)(channel_config_data_addr & 0xFFU);
    packet.tx_bytes[3] = (uint8_t)((resistance_value >> 24U) & 0xFFU);
    packet.tx_bytes[4] = (uint8_t)((resistance_value >> 16U) & 0xFFU);
    packet.tx_bytes[5] = (uint8_t)((resistance_value >> 8U) & 0xFFU);
    packet.tx_bytes[6] = (uint8_t)(resistance_value & 0xFFU);
    spi_instance->transmit_receive(packet);
}

uint8_t ltc_2984::read_status_reg()
{
    spi::packet_t packet;

    for (uint8_t i = 0; i < 8U; ++i)
    {
        packet.tx_bytes[i] = 0x00U;
        packet.rx_bytes[i] = 0x00U;
        packet.bytes_per_transaction[i] = 0x00U;
    }

    packet.packet_id = ID_INVALID;
    packet.channel_id = spi_channel;
    packet.chip_select.port = chip_select.port;
    packet.chip_select.pin = chip_select.pin;
    packet.bytes_per_transaction[0] = 4U;

    packet.tx_bytes[0] = SPI_INSTRUCTION_READ_RAM;
    packet.tx_bytes[1] = (uint8_t)((COMMAND_STATUS_REG_ADDR_MSB >> 8U) & 0xFFU);
    packet.tx_bytes[2] = (uint8_t)(COMMAND_STATUS_REG_ADDR_LSB & 0xFFU);
    packet.tx_bytes[3] = (uint8_t)0U;

    spi_instance->transmit_receive(packet);
    return packet.rx_bytes[3];
}

void ltc_2984::initiate_conversion(channel_id_e arg_ch_id) const
{
//    uint8_t measurement_command = START_CONVERSION_START_BIT | arg_ch_id;
    uint8_t measurement_command = START_CONVERSION_CH20;
    uint16_t channel_config_data_addr = get_channel_config_data_address(arg_ch_id);

    spi::packet_t packet;

    for (uint8_t i = 0; i < 8U; ++i)
    {
        packet.tx_bytes[i] = 0x00U;
        packet.rx_bytes[i] = 0x00U;
        packet.bytes_per_transaction[i] = 0x00U;
    }

    packet.packet_id = ID_INVALID;
    packet.channel_id = spi_channel;
    packet.chip_select.port = chip_select.port;
    packet.chip_select.pin = chip_select.pin;
    packet.bytes_per_transaction[0] = 4U;

    packet.tx_bytes[0] = SPI_INSTRUCTION_WRITE_RAM;
    packet.tx_bytes[1] = COMMAND_STATUS_REG_ADDR_MSB;
    packet.tx_bytes[2] = COMMAND_STATUS_REG_ADDR_LSB;
    packet.tx_bytes[3] = (uint8_t)measurement_command;
    spi_instance->transmit_receive(packet);
}

uint32_t ltc_2984::read_conversion_result(channel_id_e arg_ch_id)
{
    uint16_t conversion_result_addr = 0U;
    switch (arg_ch_id)
    {
        case CH1:
        {
            conversion_result_addr = CH1_CONVERSION_RESULT_START_ADDR;
            break;
        }
        case CH2:
        {
            conversion_result_addr = CH2_CONVERSION_RESULT_START_ADDR;
            break;
        }
        case CH3:
        {
            conversion_result_addr = CH3_CONVERSION_RESULT_START_ADDR;
            break;
        }
        case CH4:
        {
            conversion_result_addr = CH4_CONVERSION_RESULT_START_ADDR;
            break;
        }
        case CH5:
        {
            conversion_result_addr = CH5_CONVERSION_RESULT_START_ADDR;
            break;
        }
        case CH6:
        {
            conversion_result_addr = CH6_CONVERSION_RESULT_START_ADDR;
            break;
        }
        case CH7:
        {
            conversion_result_addr = CH7_CONVERSION_RESULT_START_ADDR;
            break;
        }
        case CH8:
        {
            conversion_result_addr = CH8_CONVERSION_RESULT_START_ADDR;
            break;
        }
        case CH9:
        {
            conversion_result_addr = CH9_CONVERSION_RESULT_START_ADDR;
            break;
        }
        case CH10:
        {
            conversion_result_addr = CH10_CONVERSION_RESULT_START_ADDR;
            break;
        }
        case CH11:
        {
            conversion_result_addr = CH11_CONVERSION_RESULT_START_ADDR;
            break;
        }
        case CH12:
        {
            conversion_result_addr = CH12_CONVERSION_RESULT_START_ADDR;
            break;
        }
        case CH13:
        {
            conversion_result_addr = CH13_CONVERSION_RESULT_START_ADDR;
            break;
        }
        case CH14:
        {
            conversion_result_addr = CH14_CONVERSION_RESULT_START_ADDR;
            break;
        }
        case CH15:
        {
            conversion_result_addr = CH15_CONVERSION_RESULT_START_ADDR;
            break;
        }
        case CH16:
        {
            conversion_result_addr = CH16_CONVERSION_RESULT_START_ADDR;
            break;
        }
        case CH17:
        {
            conversion_result_addr = CH17_CONVERSION_RESULT_START_ADDR;
            break;
        }
        case CH18:
        {
            conversion_result_addr = CH18_CONVERSION_RESULT_START_ADDR;
            break;
        }
        case CH19:
        {
            conversion_result_addr = CH19_CONVERSION_RESULT_START_ADDR;
            break;
        }
        case CH20:
        {
            conversion_result_addr = CH20_CONVERSION_RESULT_START_ADDR;
            break;
        }
        default:
        {
            break;
        }
    }

    uint32_t conversion_result;
    spi::packet_t packet;

    for (uint8_t i = 0; i < 8U; ++i)
    {
        packet.tx_bytes[i] = 0x00U;
        packet.rx_bytes[i] = 0x00U;
        packet.bytes_per_transaction[i] = 0x00U;
    }

    packet.packet_id = ID_INVALID;
    packet.channel_id = spi_channel;
    packet.chip_select.port = chip_select.port;
    packet.chip_select.pin = chip_select.pin;
    packet.bytes_per_transaction[0] = 7;

    packet.tx_bytes[0] = SPI_INSTRUCTION_READ_RAM;
    packet.tx_bytes[1] = (uint8_t)((conversion_result_addr >> 8U) & 0xFFU);
    packet.tx_bytes[2] = (uint8_t)(conversion_result_addr & 0xFFU);
    packet.tx_bytes[3] = (uint8_t)((0U >> 24U) & 0xFFU);
    packet.tx_bytes[4] = (uint8_t)((0U >> 16U) & 0xFFU);
    packet.tx_bytes[5] = (uint8_t)((0U >> 8U) & 0xFFU);
    packet.tx_bytes[6] = (uint8_t)(0U & 0xFFU);
    spi_instance->transmit_receive(packet);

    conversion_result = (uint32_t) packet.rx_bytes[3] << 24 | (uint32_t) packet.rx_bytes[4] << 16 | (uint32_t) packet.rx_bytes[5] << 8  |
                  (uint32_t) packet.rx_bytes[6];

    return conversion_result;

}


uint32_t ltc_2984::get_channel_config_data_address(channel_id_e arg_ch_id)
{
    uint16_t channel_config_data_addr = 0x0000U;

    switch (arg_ch_id)
    {
        case CH1:
        {
            channel_config_data_addr = CH1_CONFIG_DATA_START_ADDR;
            break;
        }
        case CH2:
        {
            channel_config_data_addr = CH2_CONFIG_DATA_START_ADDR;
            break;
        }
        case CH3:
        {
            channel_config_data_addr = CH3_CONFIG_DATA_START_ADDR;
            break;
        }
        case CH4:
        {
            channel_config_data_addr = CH4_CONFIG_DATA_START_ADDR;
            break;
        }
        case CH5:
        {
            channel_config_data_addr = CH5_CONFIG_DATA_START_ADDR;
            break;
        }
        case CH6:
        {
            channel_config_data_addr = CH6_CONFIG_DATA_START_ADDR;
            break;
        }
        case CH7:
        {
            channel_config_data_addr = CH7_CONFIG_DATA_START_ADDR;
            break;
        }
        case CH8:
        {
            channel_config_data_addr = CH8_CONFIG_DATA_START_ADDR;
            break;
        }
        case CH9:
        {
            channel_config_data_addr = CH9_CONFIG_DATA_START_ADDR;
            break;
        }
        case CH10:
        {
            channel_config_data_addr = CH10_CONFIG_DATA_START_ADDR;
            break;
        }
        case CH11:
        {
            channel_config_data_addr = CH11_CONFIG_DATA_START_ADDR;
            break;
        }
        case CH12:
        {
            channel_config_data_addr = CH12_CONFIG_DATA_START_ADDR;
            break;
        }
        case CH13:
        {
            channel_config_data_addr = CH13_CONFIG_DATA_START_ADDR;
            break;
        }
        case CH14:
        {
            channel_config_data_addr = CH14_CONFIG_DATA_START_ADDR;
            break;
        }
        case CH15:
        {
            channel_config_data_addr = CH15_CONFIG_DATA_START_ADDR;
            break;
        }
        case CH16:
        {
            channel_config_data_addr = CH16_CONFIG_DATA_START_ADDR;
            break;
        }
        case CH17:
        {
            channel_config_data_addr = CH17_CONFIG_DATA_START_ADDR;
            break;
        }
        case CH18:
        {
            channel_config_data_addr = CH18_CONFIG_DATA_START_ADDR;
            break;
        }
        case CH19:
        {
            channel_config_data_addr = CH19_CONFIG_DATA_START_ADDR;
            break;
        }
        case CH20:
        {
            channel_config_data_addr = CH20_CONFIG_DATA_START_ADDR;
            break;
        }
        default:
        {
            break;
        }
    }
    return channel_config_data_addr;
}

#pragma clang diagnostic push
#pragma ide diagnostic ignored "cppcoreguidelines-narrowing-conversions"
float ltc_2984::rtd_calculate_temp(uint32_t arg_raw_conversion)
{
    arg_raw_conversion = arg_raw_conversion & 0xFFFFFFU;
    int32_t signed_raw_conversion = arg_raw_conversion;
    float scaled_conversion = 0.0F;
    if (arg_raw_conversion & 0x800000U)
    {
        signed_raw_conversion = signed_raw_conversion | 0xFF000000U;
    }
    scaled_conversion = (float)signed_raw_conversion / 1024;

    return scaled_conversion;
}
#pragma clang diagnostic pop
