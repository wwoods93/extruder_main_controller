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

void rtd::initialize(int16_t arg_channel_id, rtosal::message_queue_handle_t arg_request_queue_handle, rtosal::message_queue_handle_t arg_result_queue_handle, rtosal::message_queue_handle_t arg_output_queue_handle, hal::timer_handle_t* arg_reading_timer_handle)
{
    request_queue_handle = arg_request_queue_handle;
    result_queue_handle = arg_result_queue_handle;
    output_queue_handle = arg_output_queue_handle;
    reading_timer_handle = arg_reading_timer_handle;
    channel_id = arg_channel_id;
}

float rtd::read()
{
    request_reading();
    return receive_reading_and_output_moving_average();
}

uint8_t rtd::request_reading()
{
    if (get_timer_count(reading_timer_handle) - reading_request_tick > READING_PERIOD_MS)
    {
        rtosal::build_common_packet(request_packet, channel_id, complete_tx, bytes_per_tx);

        if (rtosal::message_queue_send(request_queue_handle, &request_packet, 0U) == rtosal::OS_OK)
        {
            reading_request_tick = get_timer_count(reading_timer_handle);
        }

    }
    return 0;
}

float rtd::receive_reading_and_output_moving_average()
{

    if (rtosal::message_queue_receive( result_queue_handle, &result_packet, 50U) == rtosal::OS_OK)
    {
        temperature_celsius_current_reading = 0;

        rtd_resistance_scaled_and_rounded = (double)get_msb_and_lsb_register_bytes_and_concatenate(result_packet);
        rtd_resistance_scaled_and_rounded = ceil((double)(rtd_resistance_scaled_and_rounded * RTD_RESISTANCE_RATIO_SCALE_FACTOR * 1000.0)) / 1000.0;
        rtd_resistance_scaled_and_rounded = ceil(rtd_resistance_scaled_and_rounded * 100);
        temperature_celsius_current_reading = convert_rtd_resistance_to_temperature_celsius((uint32_t) rtd_resistance_scaled_and_rounded);

        if (temperature_celsius_current_reading > 0.0 && temperature_celsius_current_reading < 60000.0)
        {
            temperature_celsius_moving_average -= (temperature_celsius_moving_average / (float) moving_average_sample_count);
            temperature_celsius_moving_average += (temperature_celsius_current_reading / (float) moving_average_sample_count);
            output_data.id = channel_id;
            output_data.value = temperature_celsius_moving_average;

            if (rtosal::message_queue_send(output_queue_handle, &output_data, 50U) == rtosal::OS_OK)
            {

            }
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
