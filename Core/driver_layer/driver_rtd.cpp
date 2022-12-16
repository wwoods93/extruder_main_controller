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

//rtd::rtd(spi* spi_object)
//{
//    spi_peripheral = spi_object->spi_module_handle;
//}

//rtd::rtd() {}

void rtd::initialize_rtd(spi* spi_object)
{
    rtd_spi_object = spi_object;
    spi_peripheral = rtd_spi_object->spi_module_handle;
//    rtd_spi_object->configure_module(rtd_spi_object->spi_module_handle);
//    rtd_spi_object->spi_register_callback((spi::callback_id_t )spi::SPI_TX_RX_COMPLETE_CALLBACK_ID, HAL_SPI_TxRxCplt_Callback);
//    rtd_spi_object->spi_register_callback((spi::callback_id_t )spi::SPI_TX_RX_COMPLETE_CALLBACK_ID, HAL_SPI_Error_Callback);
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
}

//void rtd::MX_SPI2_Init()
//{
//    /* USER CODE BEGIN SPI2_Init 0 */
//
//    /* USER CODE END SPI2_Init 0 */
//
//    /* USER CODE BEGIN SPI2_Init 1 */
//
//    /* USER CODE END SPI2_Init 1 */
//    /* SPI2 parameter configuration*/
//    spi_peripheral->instance = SPI2;
//    spi_peripheral->Init.Mode = SPI_MODE_MASTER;
//    spi_peripheral->Init.Direction = SPI_DIRECTION_2LINES;
//    spi_peripheral->Init.DataSize = SPI_DATASIZE_8BIT;
//    spi_peripheral->Init.CLKPolarity = SPI_POLARITY_HIGH;
//    spi_peripheral->Init.CLKPhase = SPI_PHASE_2EDGE;
//    spi_peripheral->Init.NSS = SPI_NSS_SOFT;
//    spi_peripheral->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
//    spi_peripheral->Init.FirstBit = SPI_FIRSTBIT_MSB;
//    spi_peripheral->Init.TIMode = SPI_TIMODE_DISABLE;
//    spi_peripheral->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
//    spi_peripheral->Init.CRCPolynomial = 10;
//    if (HAL_SPI_Init(spi_peripheral) != HAL_OK)
//    {
//        Error_Handler();
//    }
//    /* USER CODE BEGIN SPI2_Init 2 */
//
//    /* USER CODE END SPI2_Init 2 */
//
//}

uint8_t rtd::readRegister8(uint8_t addr, GPIO_TypeDef* port, uint16_t pin)
{
    addr &= 0x7F;
    uint8_t rx_data = 0;
    rtd_spi_object->spi_transmit_receive_interrupt(&addr, &rx_data, 1, port, pin);

//    GPIO_PinClear(RTD_CS);
//    SPI_transfer_8(addr);
//    GPIO_PinSet(RTD_CS);
//    ret = SPI1BUF;
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
    double temp_calculation_term_1 = 0;
    double temp_calculation_term_2 = 0;
    double temp_calculation_term_3 = 0;
    double temp_calculation_term_4 = 0;
    double resistance_ratio = 0;
    double calculated_temperature = 0;

    resistance_ratio = read_rtd(port, pin);
    resistance_ratio /= RESISTANCE_RATIO_DIVISOR;
    resistance_ratio *= RTD_RESISTANCE_REFERENCE;
    double resistance_as_double = floor(resistance_ratio * 100.0) / 100.0;
    uint32_t resistance_as_uint32_t = floor(resistance_as_double * 100);
    float temp = ohmsX100_to_celsius(resistance_as_uint32_t);
    return temp;



//    temp_calculation_term_1 = -RTD_FACTOR_1;
//    temp_calculation_term_2 = RTD_FACTOR_1 * RTD_FACTOR_1 - (4 * RTD_FACTOR_2);
//    temp_calculation_term_3 = (4 * RTD_FACTOR_2) / RTD_RESISTANCE_NOMINAL;
//    temp_calculation_term_4 = 2 * RTD_FACTOR_2;
//
//    calculated_temperature = temp_calculation_term_2 + (temp_calculation_term_3 * resistance_ratio);
//    calculated_temperature = ((double)sqrt(calculated_temperature) + temp_calculation_term_1) / temp_calculation_term_4;
//
//    if (calculated_temperature >= 0)
//        return (float)calculated_temperature;
//
//    resistance_ratio /= RTD_RESISTANCE_NOMINAL;
//    resistance_ratio *= 1000;
//
//    double polynomial_input = resistance_ratio;
//
//    calculated_temperature = INITIAL_CALCULATED_TEMPERATURE;
//    calculated_temperature += DEGREE_1_COEFFICIENT * polynomial_input;
//    polynomial_input *= resistance_ratio;
//    calculated_temperature += DEGREE_2_COEFFICIENT * polynomial_input;
//    polynomial_input *= resistance_ratio;
//    calculated_temperature -= DEGREE_3_COEFFICIENT * polynomial_input;
//    polynomial_input *= resistance_ratio;
//    calculated_temperature -= DEGREE_4_COEFFICIENT * polynomial_input;
//    polynomial_input *= resistance_ratio;
//    calculated_temperature += DEGREE_5_COEFFICIENT * polynomial_input;
//
//    return (float)calculated_temperature;
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

//float rtd::get_temp()
//{
//    float temp = 0;
//    temp = read_rtd_and_calculate_temperature();
//    CORETIMER_DelayMs(17);
//    return temp;
//}

uint32_t rtd::search_pt100_list(uint32_t ohmsX100)
{
    int lower = 0 ;
    int upper = 509 ;
    int mid = (lower + upper) / 2 ;

    do
    {
        uint32_t pt100val = temperature_to_resistance_pt1000_lookup_table[mid] ;

        if (pt100val == ohmsX100)
        {
            break;
        }
        else if (pt100val < ohmsX100)
        {
            lower = mid + 1 ;
        }
        else
        {
            upper = mid ;
        }

        mid = (lower + upper) / 2 ;

    } while (lower < upper)	;
    // falls through on last mismatch

    return(mid);
}

float rtd::ohmsX100_to_celsius (uint32_t ohmsX100)
{
    uint32_t R_upper, R_lower ;
    uint32_t hundredths = 0 ;
    uint32_t iTemp = 0 ;
    float celsius ;

    uint32_t index = search_pt100_list(ohmsX100) ;

    // The minimum integral temperature
    iTemp = index - 1 + CELSIUS_MIN ;

    // fetch floor() and ceiling() resistances since
    // key = intermediate value is the most likely case.

    // ACHTUNG!  (index == 0) is forbidden!
    R_lower = temperature_to_resistance_pt1000_lookup_table[index - 1];
    R_upper = temperature_to_resistance_pt1000_lookup_table[index];

    if (ohmsX100 == R_upper)
    {
        iTemp++ ;
        hundredths = 0 ;
    }
    else if (ohmsX100 < R_upper)
    {
        hundredths = ((100 * (ohmsX100 - R_lower)) / (R_upper - R_lower)) ;
    }
    else if (ohmsX100 > R_upper)
    {
        iTemp++ ;
        // risks index+1 out of range
        uint16_t Rnext = temperature_to_resistance_pt1000_lookup_table[index + 1];
        hundredths = (100 * (ohmsX100 - R_upper)) / (Rnext - R_upper) ;
    }
    else
    {
        hundredths = ((100 * (ohmsX100 - R_lower)) / (R_upper - R_lower)) ;
    }

    celsius  = (float)iTemp + (float)hundredths / 100.0 ;

    return(celsius );
}
