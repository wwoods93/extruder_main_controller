/***********************************************************************************************************************
 * Main_Controller
 * driver_rtd.cpp
 *
 * wilson
 * 10/9/22
 * 11:34 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

#include <cstdlib>
#include <cstring>
#include <cmath>
#include "stm32f4xx.h"
#include "cmsis_os2.h"
#include "../layer_0/hal.h"
#include "../layer_0/hal_spi.h"
#include "rtd.h"
#include "../layer_0/rtosal.h"
#include "../layer_0/hal_callback.h"

rtd::rtd()
{

}

void rtd::initialize(spi& arg_spi_instance, hal::gpio_t* arg_chip_select_port, uint16_t arg_chip_select_pin, int16_t arg_channel_id, rtosal::message_queue_handle_t arg_output_queue_handle, hal::timer_handle_t* arg_reading_timer_handle, float arg_cal_measured, float arg_cal_expected)
{
    spi_instance = arg_spi_instance;
    chip_select.port = arg_chip_select_port;
    chip_select.pin = arg_chip_select_pin;
    output_queue_handle = arg_output_queue_handle;
    reading_timer_handle = arg_reading_timer_handle;
    channel_id = arg_channel_id;

    cal_resistance_constant_offset = arg_cal_expected - arg_cal_measured;
    cal_resistance_linear_scale_factor = 1 + (cal_resistance_constant_offset / arg_cal_expected);
}

float rtd::read()
{
    spi_packet.packet_id = ID_INVALID;
    spi_packet.channel_id = channel_id;
    spi_packet.chip_select.port = chip_select.port;
    spi_packet.chip_select.pin = chip_select.pin;
    for (uint8_t i = 0U; i < 8U; ++i)
    {
        spi_packet.tx_bytes[i] = complete_tx[i];
        spi_packet.bytes_per_transaction[i] = bytes_per_tx[i];
    }

    spi_instance.transmit_receive(spi_packet);

    for (uint8_t i = 0U; i < 8U; ++i)
    {
        result_packet.bytes[i] = spi_packet.rx_bytes[i];
        result_packet.bytes_per_transaction[i] = spi_packet.bytes_per_transaction[i];
    }

    temperature_celsius_current_reading = 0;
    rtd_resistance_scaled_and_rounded = (double) get_msb_and_lsb_register_bytes_and_concatenate(result_packet);
    rtd_resistance_scaled_and_rounded = ceil((double) (rtd_resistance_scaled_and_rounded * RTD_RESISTANCE_RATIO_SCALE_FACTOR * 1000.0)) / 1000.0;
    rtd_resistance_scaled_and_rounded = ceil(rtd_resistance_scaled_and_rounded * 100);
    temperature_celsius_current_reading = convert_rtd_resistance_to_temperature_celsius((uint32_t) rtd_resistance_scaled_and_rounded);

    switch (calibration_method)
    {
        case CAL_METHOD_CONSTANT:
        {
            temperature_celsius_current_reading += cal_resistance_constant_offset;
            break;
        }
        case CAL_METHOD_LINEAR:
        {
            temperature_celsius_current_reading =
                    temperature_celsius_current_reading * cal_resistance_linear_scale_factor;
            break;
        }
        default:
        {
            break;
        }
    }

    if (temperature_celsius_current_reading > 0.0F && temperature_celsius_current_reading < 60000.0F &&
        (temperature_celsius_current_reading - temperature_celsius_moving_average) < 100.0F)
    {
        temperature_celsius_moving_average -= (temperature_celsius_moving_average / (float) moving_average_sample_count);
        temperature_celsius_moving_average += (temperature_celsius_current_reading / (float) moving_average_sample_count);
        output_data.id = channel_id;
        output_data.value = temperature_celsius_moving_average;

        if (rtosal::message_queue_send(output_queue_handle, &output_data, 50U) == rtosal::OS_OK)
        {

        }
    }

    return temperature_celsius_moving_average;
}

uint16_t rtd::get_msb_and_lsb_register_bytes_and_concatenate(common_packet_t& arg_common_packet)
{
    rtd_register_contents = arg_common_packet.bytes[3];
    rtd_register_contents <<= 8U;
    rtd_register_contents |= arg_common_packet.bytes[5];
    return rtd_register_contents >> 1U;
}

uint32_t rtd::search_lookup_table(uint32_t rtd_resistance)
{
    temperature_range_min = 0U;
    temperature_range_max = CELSIUS_MAX;
    temperature_range_midpoint = (temperature_range_min + temperature_range_max) / 2U ;

    while (temperature_range_min < temperature_range_max)
    {
        uint32_t temperature_celsius_from_rtd_resistance = temperature_to_resistance_pt1000_lookup_table[temperature_range_midpoint];

        if (temperature_celsius_from_rtd_resistance == rtd_resistance)
        {
            break;
        }
        else if (temperature_celsius_from_rtd_resistance < rtd_resistance)
        {
            temperature_range_min = temperature_range_midpoint + 1U ;
        }
        else
        {
            temperature_range_max = temperature_range_midpoint;
        }

        temperature_range_midpoint = (temperature_range_min + temperature_range_max) / 2U;
    }
    return temperature_range_midpoint;
}

float rtd::convert_rtd_resistance_to_temperature_celsius(uint32_t rtd_resistance)
{
    temperature_celsius_float_component = 0.0;
    temperature_celsius_integer_component = 0U;
    next_resistance_in_lookup_table = 0U;

    lookup_table_index = search_lookup_table(rtd_resistance) ;
    temperature_celsius_integer_component = lookup_table_index - 1U + CELSIUS_MIN ;
    resistance_lower_bound = temperature_to_resistance_pt1000_lookup_table[lookup_table_index - 1U];
    resistance_upper_bound = temperature_to_resistance_pt1000_lookup_table[lookup_table_index];

    if (rtd_resistance < resistance_upper_bound)
    {
        temperature_celsius_float_component = ((100 * (rtd_resistance - resistance_lower_bound)) / (resistance_upper_bound - resistance_lower_bound));
    }
    else if (rtd_resistance > resistance_upper_bound)
    {
        ++temperature_celsius_integer_component;
        next_resistance_in_lookup_table = temperature_to_resistance_pt1000_lookup_table[lookup_table_index + 1U];
        temperature_celsius_float_component = (100 * (rtd_resistance - resistance_upper_bound)) / (next_resistance_in_lookup_table - resistance_upper_bound);
    }
    else if (rtd_resistance == resistance_upper_bound)
    {
        ++temperature_celsius_integer_component;
        return (float)temperature_celsius_integer_component;
    }

    return (float)(temperature_celsius_integer_component + (float)temperature_celsius_float_component / 100.0);
}
