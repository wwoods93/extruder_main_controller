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
#include <cmath>
#include "stm32f4xx.h"
#include "cmsis_os2.h"
#include "peripheral_common.h"
#include "mcu_clock_timers.h"
#include "../hardware_abstraction_layer/hal_spi.h"
#include "driver_rtd.h"
#include "../hardware_abstraction_layer/hal_callbacks.h"

// div2 = 8MHz
// div4 = 4MHz
// div8 = 2MHz
// div16 = 1MHz
// div32 = 500KHz
// div64 = 250KHz
// div128 = 125KHz
// div256 = 62.5KHz

void rtd::initialize_rtd(spi* spi_object)
{
    rtd_spi_object = spi_object;
    spi_peripheral = rtd_spi_object->spi_module_handle;
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
    rtd_spi_object->spi_transmit_receive_interrupt(&register_address, &rx_1, 1, device_id);
    rtd_spi_object->spi_transmit_receive_interrupt(&data, &rx_2, 1, device_id);
}

uint8_t rtd::read_register_8(uint8_t register_address) const
{
    register_address &= READ_REGISTER_ADDRESS_MASK;
    uint8_t rx_data = 0;
    rtd_spi_object->spi_transmit_receive_interrupt(&register_address, &rx_data, 1, device_id);
    while (!hal_callbacks_get_spi_rx_data_ready_flag());
    hal_callbacks_set_spi_rx_data_ready_flag(0);
    return rx_data;
}

uint16_t rtd::read_msb_and_lsb_registers_and_concatenate() const
{
    uint16_t rtd_reading = 0;
    uint8_t tx_data_1[2] = {MSB_REGISTER_ADDRESS_FOR_READ, DUMMY_BYTE};
    uint8_t tx_data_2[2] = {LSB_REGISTER_ADDRESS_FOR_READ, DUMMY_BYTE};

    auto *rx_ptr = static_cast<uint8_t *>(malloc(2 * sizeof(uint8_t)));
    rtd_spi_object->spi_transmit_receive_interrupt(tx_data_1, rx_ptr, 2, device_id);
    while (!hal_callbacks_get_spi_rx_data_ready_flag());
    hal_callbacks_set_spi_rx_data_ready_flag(0);
    rtd_reading = *(++rx_ptr);
    rtd_reading <<= 8;
    *rx_ptr = 0;
    *(--rx_ptr) = 0;
    rtd_spi_object->spi_transmit_receive_interrupt(tx_data_2, rx_ptr, 2, device_id);
    while (!hal_callbacks_get_spi_rx_data_ready_flag());
    hal_callbacks_set_spi_rx_data_ready_flag(0);
    rtd_reading |= *(++rx_ptr);
    free(--rx_ptr);

    return rtd_reading >> 1;
}

uint16_t rtd::read_rtd() const
{
    rtd_begin();
    return read_msb_and_lsb_registers_and_concatenate();
}

float rtd::read_rtd_and_calculate_temperature(uint8_t _device_id)
{
    device_id = _device_id;
    double resistance_as_double = 0;
    float calculated_temperture_celsius = 0;

    rtd_resistance_scaled_and_rounded = round(read_rtd() * RTD_RESISTANCE_RATIO_SCALE_FACTOR * 1000.0) / 1000.0;
    rtd_resistance_scaled_and_rounded = round(rtd_resistance_scaled_and_rounded * 100.0);
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

GPIO_TypeDef* get_chip_select_port(rtd* rtd_object)
{
    return rtd_object->chip_select_port;
}

uint16_t get_chip_select_pin(rtd* rtd_object)
{
    return rtd_object->chip_select_pin;
}

void reset_chip_select_port_and_pin(rtd* rtd_object)
{
    rtd_object->chip_select_port = (GPIO_TypeDef*) nullptr;
    rtd_object->chip_select_pin = 0;
}
