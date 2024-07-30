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
#include "peripheral_common.h"
#include "mcu_clock_timers.h"
#include "../layer_0_hal//hal_spi.h"
#include "driver_rtd.h"
#include "../layer_2_rtosal/rtosal.h"
#include "../layer_0_hal//hal_callbacks.h"
#include "../meta_structure/meta_structure_system_manager.h"
#include "../meta_structure/meta_structure_user.h"
#include "../layer_2_rtosal/rtosal_spi_shared_resources.h"

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

void rtd::initialize(read_rate_t _read_rate_hz, id_number_t arg_channel_id)
{
    os_kernel_frequency = osKernelGetTickFreq();

    switch (_read_rate_hz)
    {
        case READ_RATE_10_HZ:
        {
            read_rate_os_ticks = 10U;
            break;
        }
        case READ_RATE_5_HZ:
        {
            read_rate_os_ticks = 20U;
            break;
        }
        case READ_RATE_2_HZ:
        {
            read_rate_os_ticks = 50U;
            break;
        }
        case READ_RATE_1_HZ:
        {
            read_rate_os_ticks = 100U;
            break;
        }
        default:
            break;
    }
    channel_id = arg_channel_id;
    initialized = 1U;
//    std::string rtd_name = "RTD ZONE 1";
//    user_id = register_new_user_to_user_manifest(USER_TYPE_RTD, rtd_name);
}

void rtd::start_read_requests()
{
    request_readings = 1U;
}

uint8_t rtd::send_request_if_flag_set(common_packet_t& _packet)
{
    uint8_t new_request = 0U;
    memset(&_packet, '\0', sizeof(_packet));
    if (send_new_request == 1U)
    {
        rtosal::build_common_packet(_packet, channel_id, complete_tx, bytes_per_tx, 1, 1);

        new_request = 1U;
    }
    return new_request;
}

void rtd::clear_send_new_request_flag()
{
    send_new_request = 0U;
}

void rtd::pass_available_sensor_command_to_buffer(common_packet_t& _packet)
{
    if (setup_command_requested)
    {
//        rtosal::build_common_packet(_packet, 0, tx_data);

    }
    else if (read_command_requested)
    {

    }
}

void rtd::handle_sensor_state()
{
    switch (sensor_state)
    {
        case SENSOR_INITIALIZE:
        {
            if (initialized == 1U)
            {
                sensor_state = SENSOR_IDLE;
            }
            else
            {
                // error
            }
            break;
        }
        case SENSOR_IDLE:
        {
            send_new_request = 1U;

            break;
        }
        case SENSOR_SETUP_COMMAND_SENT:
        {

            break;
        }
        case SENSOR_READ_REGISTER_COMMAND_SENT:
        {

            break;
        }
        case SENSOR_DATA_RECEIVED:
        {

            break;
        }
        default:
            break;
    }
}

void rtd::rtd_begin() const
{
    write_register_8(CONFIG_REGISTER_ADDRESS, RTD_CONFIG_REG_BYTE);
}

void rtd::write_register_8(uint8_t register_address, uint8_t data) const
{
    uint8_t rx_1 = 0;
    uint8_t rx_2 = 0;
    register_address |= WRITE_REGISTER_ADDRESS_MASK;
//    rtd_spi_object->spi_transmit_receive_interrupt(&register_address, &rx_1, 1, device_id);
//    rtd_spi_object->spi_transmit_receive_interrupt(&data, &rx_2, 1, device_id);
//    rtd_spi_object->add_packet_to_buffer(device_id, 1, &register_address);
//    rtd_spi_object->add_packet_to_buffer(device_id, 1, &data);
}

uint8_t rtd::read_register_8(uint8_t register_address) const
{
    register_address &= READ_REGISTER_ADDRESS_MASK;
    uint8_t rx_data = 0;
//    rtd_spi_object->spi_transmit_receive_interrupt(&register_address, &rx_data, 1, device_id);
//    rtd_spi_object->add_packet_to_buffer(device_id, 1, &register_address);
//    while (!hal_callbacks_get_spi_rx_data_ready_flag());
//    hal_callbacks_set_spi_rx_data_ready_flag(0);
    return rx_data;
}

//uint16_t rtd::read_msb_and_lsb_registers_and_concatenate() const
//{
//    uint16_t rtd_reading = 0;
//    uint8_t tx_data_1[2] = {MSB_REGISTER_ADDRESS_FOR_READ, DUMMY_BYTE};
//    uint8_t tx_data_2[2] = {LSB_REGISTER_ADDRESS_FOR_READ, DUMMY_BYTE};
//
//    auto *rx_ptr = static_cast<uint8_t *>(malloc(2 * sizeof(uint8_t)));
//    rtd_spi_object->spi_transmit_receive_interrupt(tx_data_1, rx_ptr, 2, device_id);
////    rtd_spi_object->add_packet_to_buffer(device_id, 2, tx_data_1);
//    while (!hal_callbacks_get_spi_rx_data_ready_flag());
//    hal_callbacks_set_spi_rx_data_ready_flag(0);
//    rtd_reading = *(++rx_ptr);
//    rtd_reading <<= 8;
//    *rx_ptr = 0;
//    *(--rx_ptr) = 0;
//    rtd_spi_object->spi_transmit_receive_interrupt(tx_data_2, rx_ptr, 2, device_id);
////    rtd_spi_object->add_packet_to_buffer(device_id, 2, tx_data_2);
//    while (!hal_callbacks_get_spi_rx_data_ready_flag());
//    hal_callbacks_set_spi_rx_data_ready_flag(0);
//    rtd_reading |= *(++rx_ptr);
//    free(--rx_ptr);
//
//    return rtd_reading >> 1;
//}

uint16_t rtd::read_msb_and_lsb_registers_and_concatenate() const
{
    uint16_t rtd_reading = 0;
//    uint8_t tx_data_1[2] = {MSB_REGISTER_ADDRESS_FOR_READ, DUMMY_BYTE};
//    uint8_t tx_data_2[2] = {LSB_REGISTER_ADDRESS_FOR_READ, DUMMY_BYTE};





//    auto *rx_ptr = static_cast<uint8_t *>(malloc(2 * sizeof(uint8_t)));
//    rtd_spi_object->spi_transmit_receive_interrupt(tx_data_1, rx_ptr, 2, device_id);
//    rtd_spi_object->add_packet_to_buffer(device_id, 2, tx_data_1);
//    while (!hal_callbacks_get_spi_rx_data_ready_flag());
//    hal_callbacks_set_spi_rx_data_ready_flag(0);
//spi_byte = *(++rx_ptr);
//    rtd_reading = *(++rx_ptr);
//    rtd_reading <<= 8;
//    *rx_ptr = 0;
//    *(--rx_ptr) = 0;
//    rtd_spi_object->spi_transmit_receive_interrupt(tx_data_2, rx_ptr, 2, device_id);
////    rtd_spi_object->add_packet_to_buffer(device_id, 2, tx_data_2);
//    while (!hal_callbacks_get_spi_rx_data_ready_flag());
//    hal_callbacks_set_spi_rx_data_ready_flag(0);
//    rtd_reading |= *(++rx_ptr);
//    free(--rx_ptr);

    return rtd_reading >> 1;
}

uint16_t rtd::get_msb_and_lsb_register_bytes_and_concatenate(common_packet_t& arg_common_packet)
{
    uint16_t rtd_reading = 0;
    rtd_reading = arg_common_packet.bytes[3];
    rtd_reading <<= 8;
    rtd_reading |= arg_common_packet.bytes[5];
    return rtd_reading >> 1;


}

uint16_t rtd::read_rtd() const
{
    rtd_begin();
    return read_msb_and_lsb_registers_and_concatenate();
}

float rtd::read_rtd_and_calculate_temperature(common_packet_t& arg_common_packet)
{
    double resistance_as_double = 0;
    float calculated_temperture_celsius = 0;

    rtd_resistance_scaled_and_rounded = (double)get_msb_and_lsb_register_bytes_and_concatenate(arg_common_packet);
    rtd_resistance_scaled_and_rounded = ceil((double)(rtd_resistance_scaled_and_rounded * RTD_RESISTANCE_RATIO_SCALE_FACTOR * 1000.0)) / 1000.0;
    rtd_resistance_scaled_and_rounded = ceil(rtd_resistance_scaled_and_rounded * 100);
    calculated_temperture_celsius = rtd_resistance_to_temperature_celsius((uint32_t)rtd_resistance_scaled_and_rounded);
    temperature_celsius = calculated_temperture_celsius;
     return calculated_temperture_celsius;
}

float rtd::get_device_reading_degrees_celsius() const
{
    return temperature_celsius;
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
    user_id = 0;

}

//void rtd::configure_rtd(user_config_t& _user_config)
//{
//
//}
