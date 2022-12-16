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

// spi2 MOSI    PC1
// spi2 MISO    PC2
// GPIO OUT     PC8
// GPIO OUT     PC7
// GPIO OUT     PB15    -   CHIP SELECT
// GPIO OUT     PB14

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

uint8_t rtd::readRegister8(uint8_t addr, GPIO_TypeDef* port, uint16_t pin)
{
    addr &= 0x7F;
    uint8_t rx_data = 0;
    rtd_spi_object->spi_transmit_receive_interrupt(&addr, &rx_data, 1, port, pin);
    return rx_data;
}

uint16_t rtd::readRegister16(uint8_t addr1, uint8_t addr2, GPIO_TypeDef* port, uint16_t pin)
{
    addr1 &= 0x7F;
    addr2 &= 0x7F;
    uint16_t rtd_reading = 0;
    uint8_t tx_data_1[2] = {addr1, 0xFF};
    uint8_t tx_data_2[2] = {addr2, 0xFF};

    uint8_t *rx_ptr = static_cast<uint8_t *>(malloc(2 * sizeof(uint8_t)));
    rtd_spi_object->spi_transmit_receive_interrupt(tx_data_1, rx_ptr, 2, port, pin);
    while (!hal_callbacks_get_spi_rx_data_ready_flag());
    hal_callbacks_set_spi_rx_data_ready_flag(0);
    rtd_reading = *(++rx_ptr);
    rtd_reading <<= 8;
    *rx_ptr = 0;
    *(--rx_ptr) = 0;
    rtd_spi_object->spi_transmit_receive_interrupt(tx_data_2, rx_ptr, 2, port, pin);
    while (!hal_callbacks_get_spi_rx_data_ready_flag());
    hal_callbacks_set_spi_rx_data_ready_flag(0);
    rtd_reading |= *(++rx_ptr);
    free(--rx_ptr);

    return rtd_reading;
}

void rtd::writeRegister8(uint8_t addr, uint8_t data, GPIO_TypeDef* port, uint16_t pin)
{
    uint8_t rx_1 = 0;
    uint8_t rx_2 = 0;
    addr |= 0x80;
    rtd_spi_object->spi_transmit_receive_interrupt(&addr, &rx_1, 1, port, pin);
    rtd_spi_object->spi_transmit_receive_interrupt(&data, &rx_2, 1, port, pin);
}

bool rtd::rtd_begin(max31865_numwires_t wires, GPIO_TypeDef* port, uint16_t pin)
{
    writeRegister8(MAX31865_CONFIG_REG, 0xD3, port, pin);
    return true;
}

uint16_t rtd::read_rtd(GPIO_TypeDef* port, uint16_t pin)
{
    rtd_begin(MAX31865_3WIRE, port, pin);
    uint16_t rtd = readRegister16(MAX31865_RTDMSB_REG, MAX31865_RTDLSB_REG, port, pin);
    rtd >>= 1;
    return rtd;
}

float rtd::read_rtd_and_calculate_temperature(GPIO_TypeDef* port, uint16_t pin)
{
    double resistance_as_double = 0;
    float calculated_temperture_celsius = 0;

    resistance_as_double = floor(read_rtd(port, pin) * RTD_RESISTANCE_RATIO_SCALE_FACTOR * 1000.0) / 1000.0;
    resistance_as_double = ceil(resistance_as_double * 100.0);
    calculated_temperture_celsius = rtd_resistance_to_temperature_celsius((uint32_t)resistance_as_double);
    return calculated_temperture_celsius;
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

uint32_t rtd::search_temperature_to_resistance_pt1000_lookup_table(uint32_t rtd_resistance)
{
    int temperature_range_min = 0 ;
    int temperature_range_max = 509 ;
    int temperature_range_midpoint = (temperature_range_min + temperature_range_max) / 2 ;

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

float rtd::rtd_resistance_to_temperature_celsius (uint32_t rtd_resistance)
{
    uint32_t index = 0;
    uint32_t resistance_upper_bound = 0;
    uint32_t resistance_lower_bound = 0;
    uint32_t temperature_celsius_float_component = 0 ;
    uint32_t temperture_celsius_integer_component = 0 ;
    uint16_t next_resistance_in_lookup_table = 0;

    index = search_temperature_to_resistance_pt1000_lookup_table(rtd_resistance) ;
    temperture_celsius_integer_component = index - 1 + CELSIUS_MIN ;
    resistance_lower_bound = temperature_to_resistance_pt1000_lookup_table[index - 1];
    resistance_upper_bound = temperature_to_resistance_pt1000_lookup_table[index];

    if (rtd_resistance == resistance_upper_bound)
    {
        ++temperture_celsius_integer_component;
        temperature_celsius_float_component = 0;
    }
    else if (rtd_resistance < resistance_upper_bound)
        temperature_celsius_float_component = ((100 * (rtd_resistance - resistance_lower_bound)) / (resistance_upper_bound - resistance_lower_bound));
    else if (rtd_resistance > resistance_upper_bound)
    {
        ++temperture_celsius_integer_component;
        next_resistance_in_lookup_table = temperature_to_resistance_pt1000_lookup_table[index + 1];
        temperature_celsius_float_component = (100 * (rtd_resistance - resistance_upper_bound)) / (next_resistance_in_lookup_table - resistance_upper_bound);
    }
    else
        temperature_celsius_float_component = ((100 * (rtd_resistance - resistance_lower_bound)) / (resistance_upper_bound - resistance_lower_bound));

    return (float)temperture_celsius_integer_component + (float)temperature_celsius_float_component / 100.0;
}
