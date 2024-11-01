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
#include "../layer_0/hal_timer.h"
#include "../layer_0/hal_spi.h"
#include "rtd.h"
#include "../layer_0/rtosal.h"
#include "../layer_0/hal_callback.h"

// div2 = 8MHz
// div4 = 4MHz
// div8 = 2MHz
// div16 = 1MHz
// div32 = 500KHz
// div64 = 250KHz
// div128 = 125KHz
// div256 = 62.5KHz


//void rtd::read_rtd()
//{
//
//}

void rtd::initialize(read_rate_t _read_rate_hz, int16_t arg_channel_id, rtosal::message_queue_handle_t arg_request_queue_handle, rtosal::message_queue_handle_t arg_result_queue_handle, rtosal::message_queue_handle_t arg_output_queue_handle, hal::timer_handle_t* arg_reading_timer_handle)
{
    request_queue_handle = arg_request_queue_handle;
    result_queue_handle = arg_result_queue_handle;
    output_queue_handle = arg_output_queue_handle;
    reading_timer_handle = arg_reading_timer_handle;
    channel_id = arg_channel_id;
    initialized = 1U;
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
        common_packet_t packet;
        rtosal::build_common_packet(packet, channel_id, complete_tx, bytes_per_tx);

        if (rtosal::message_queue_send(request_queue_handle, &packet, 0U) == rtosal::OS_OK)
        {
            reading_request_tick = get_timer_count(reading_timer_handle);
        }

    }
    return 0;
}

float rtd::receive_reading_and_output_moving_average()
{
    common_packet_t packet;
    if (rtosal::message_queue_receive( result_queue_handle, &packet, 50U) == rtosal::OS_OK)
    {
        temperature_celsius_current_reading = 0;

        rtd_resistance_scaled_and_rounded = (double)get_msb_and_lsb_register_bytes_and_concatenate(packet);
        rtd_resistance_scaled_and_rounded = ceil((double)(rtd_resistance_scaled_and_rounded * RTD_RESISTANCE_RATIO_SCALE_FACTOR * 1000.0)) / 1000.0;
        rtd_resistance_scaled_and_rounded = ceil(rtd_resistance_scaled_and_rounded * 100);
        temperature_celsius_current_reading = rtd_resistance_to_temperature_celsius((uint32_t)rtd_resistance_scaled_and_rounded);

        if (temperature_celsius_current_reading > 0 && temperature_celsius_current_reading < 60000)
        {
            temperature_celsius_moving_average -= (temperature_celsius_moving_average / (float) moving_average_sample_count);
            temperature_celsius_moving_average += (temperature_celsius_current_reading / (float) moving_average_sample_count);
            rtd_reading.id = channel_id;
            rtd_reading.value = temperature_celsius_moving_average;

            if (rtosal::message_queue_send(output_queue_handle, &rtd_reading, 50U) == rtosal::OS_OK)
            {

            }
        }
    }

    return temperature_celsius_moving_average;
}

uint16_t rtd::get_msb_and_lsb_register_bytes_and_concatenate(common_packet_t& arg_common_packet)
{
    uint16_t rtd_reading = 0;
    rtd_reading = arg_common_packet.bytes[3];
    rtd_reading <<= 8;
    rtd_reading |= arg_common_packet.bytes[5];

    return rtd_reading >> 1;
}





float rtd::compute_temperature_moving_average()
{
    if (temperature_celsius_current_reading > 0 && temperature_celsius_current_reading < 60000)
    {
        temperature_celsius_moving_average -= (temperature_celsius_moving_average / (float) moving_average_sample_count);
        temperature_celsius_moving_average += (temperature_celsius_current_reading / (float) moving_average_sample_count);
    }

    return temperature_celsius_moving_average;
}

float rtd::get_device_reading_degrees_celsius() const
{
    return temperature_celsius_moving_average;
}

uint32_t rtd::search_temperature_to_resistance_pt1000_lookup_table(uint32_t rtd_resistance)
{
    uint16_t temperature_range_min = 0;
    uint16_t temperature_range_max = CELSIUS_MAX;
    uint16_t temperature_range_midpoint = (temperature_range_min + temperature_range_max) / 2 ;

    while (temperature_range_min < temperature_range_max)
    {
        uint32_t temperature_celsius_from_rtd_resistance = temperature_to_resistance_pt1000_lookup_table[temperature_range_midpoint] ;
        if (temperature_celsius_from_rtd_resistance == rtd_resistance) { break; }
        else if (temperature_celsius_from_rtd_resistance < rtd_resistance) { temperature_range_min = temperature_range_midpoint + 1 ; }
        else { temperature_range_max = temperature_range_midpoint; }
        temperature_range_midpoint = (temperature_range_min + temperature_range_max) / 2;
    }
    return temperature_range_midpoint;
}

float rtd::rtd_resistance_to_temperature_celsius(uint32_t rtd_resistance)
{
    uint32_t index = 0;
    uint32_t resistance_upper_bound = 0;
    uint32_t resistance_lower_bound = 0;
    uint32_t temperature_celsius_float_component = 0 ;
    uint16_t temperature_celsius_integer_component = 0 ;
    uint16_t next_resistance_in_lookup_table = 0;

    index = search_temperature_to_resistance_pt1000_lookup_table(rtd_resistance) ;
    temperature_celsius_integer_component = index - 1 + CELSIUS_MIN ;
    resistance_lower_bound = temperature_to_resistance_pt1000_lookup_table[index - 1];
    resistance_upper_bound = temperature_to_resistance_pt1000_lookup_table[index];

    if (rtd_resistance < resistance_upper_bound)
        temperature_celsius_float_component = ((100 * (rtd_resistance - resistance_lower_bound)) / (resistance_upper_bound - resistance_lower_bound));
    else if (rtd_resistance > resistance_upper_bound)
    {
        ++temperature_celsius_integer_component;
        next_resistance_in_lookup_table = temperature_to_resistance_pt1000_lookup_table[index + 1];
        temperature_celsius_float_component = (100 * (rtd_resistance - resistance_upper_bound)) / (next_resistance_in_lookup_table - resistance_upper_bound);
    }
    else if (rtd_resistance == resistance_upper_bound)
    {
        ++temperature_celsius_integer_component;
        temperature_celsius_float_component = 0;
    }

    return (float)(temperature_celsius_integer_component + (float)temperature_celsius_float_component / 100.0);
}

rtd::rtd()
{

}
