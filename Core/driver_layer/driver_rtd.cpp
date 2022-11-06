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
#include "peripheral_common.h"
#include "mcu_clock_timers.h"
#include "driver_rtd.h"

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

rtd::rtd(SPI_HandleTypeDef* spi_object)
{
    spi_peripheral = spi_object;
}

void rtd::MX_SPI2_Init()
{
    /* USER CODE BEGIN SPI2_Init 0 */

    /* USER CODE END SPI2_Init 0 */

    /* USER CODE BEGIN SPI2_Init 1 */

    /* USER CODE END SPI2_Init 1 */
    /* SPI2 parameter configuration*/
    spi_peripheral->Instance = SPI2;
    spi_peripheral->Init.Mode = SPI_MODE_MASTER;
    spi_peripheral->Init.Direction = SPI_DIRECTION_2LINES;
    spi_peripheral->Init.DataSize = SPI_DATASIZE_8BIT;
    spi_peripheral->Init.CLKPolarity = SPI_POLARITY_HIGH;
    spi_peripheral->Init.CLKPhase = SPI_PHASE_2EDGE;
    spi_peripheral->Init.NSS = SPI_NSS_SOFT;
    spi_peripheral->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    spi_peripheral->Init.FirstBit = SPI_FIRSTBIT_MSB;
    spi_peripheral->Init.TIMode = SPI_TIMODE_DISABLE;
    spi_peripheral->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spi_peripheral->Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(spi_peripheral) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI2_Init 2 */

    /* USER CODE END SPI2_Init 2 */

}

uint8_t rtd::readRegister8(uint8_t addr)
{
    addr &= 0x7F;
    uint8_t rx_data = 0;
    HAL_SPI_TransmitReceive_IT(spi_peripheral, &addr, &rx_data, 1);

//    GPIO_PinClear(RTD_CS);
//    SPI_transfer_8(addr);
//    GPIO_PinSet(RTD_CS);
//    ret = SPI1BUF;
    return rx_data;
}

uint16_t rtd::readRegister16(uint8_t addr1, uint8_t addr2)
{
    addr1 &= 0x7F;
    addr2 &= 0x7F;
    uint16_t ret = 0;
    uint8_t tx_msb = MAX31865_RTDMSB_REG;
    uint8_t tx_lsb = MAX31865_RTDLSB_REG;
    uint8_t tx_full_byte = 0xFF;
//    uint8_t tx_data[2] = {tx_msb, tx_full_byte};
//    uint8_t tx_data[4] = {tx_msb, tx_full_byte, tx_lsb, tx_full_byte};
    uint8_t rx_1 = 0;
    uint8_t rx_2 = 0;
    uint8_t rx_3 = 0;
    uint8_t rx_4 = 0;
//    uint8_t rx_data[2] = {rx_1, rx_2};
//    uint8_t rx_data[4] = {rx_1, rx_2, rx_3, rx_4 };
    // set cs low
    HAL_SPI_TransmitReceive_IT(spi_peripheral, &tx_full_byte, &rx_1, 1);
    HAL_SPI_TransmitReceive_IT(spi_peripheral, &tx_msb, &rx_2, 1);
//    HAL_SPI_TransmitReceive_IT(spi_periph`eral, tx_data, rx_data, 4);
    ret = rx_2;
    ret <<= 8;
    // tx_data[1] = tx_lsb;
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_IT(spi_peripheral, &tx_full_byte, &rx_3, 1);
    HAL_SPI_TransmitReceive_IT(spi_peripheral, &tx_lsb, &rx_4, 1);

    ret |= rx_4;
    // us_delay(1);
    // set cs high

    return ret;
}

void rtd::writeRegister8(uint8_t addr, uint8_t data)
{
    uint8_t rx_1 = 0;
    uint8_t rx_2 = 0;
    addr |= 0x80;
    HAL_SPI_Transmit_IT(spi_peripheral, &addr, 1);
    HAL_SPI_Transmit_IT(spi_peripheral, &data, 1);
}

bool rtd::rtd_begin(max31865_numwires_t wires)
{
//    GPIO_PinOutputEnable(RTD_CS);
//    GPIO_PinSet(RTD_CS);
//    SPI_RTD_init_8();
//    CORETIMER_DelayUs(100);
    //us_delay(100);
    writeRegister8(MAX31865_CONFIG_REG, 0xD3);
    return true;
}

uint16_t rtd::read_rtd()
{
    rtd_begin(MAX31865_3WIRE);
    uint16_t rtd = readRegister16(MAX31865_RTDMSB_REG, MAX31865_RTDLSB_REG);
    rtd >>= 1;
    return rtd;
}

float rtd::read_rtd_and_calculate_temperature()
{
    double Z1, Z2, Z3, Z4, Rt, temp;

    Rt = read_rtd();
    Rt /= 32768;
    Rt *= R_REF;

    Z1 = -RTD_A;
    Z2 = RTD_A * RTD_A - (4 * RTD_B);
    Z3 = (4 * RTD_B) / R_NOM;
    Z4 = 2 * RTD_B;

    temp = Z2 + (Z3 * Rt);
    temp = ((double)sqrt(temp) + Z1) / Z4;

    if (temp >= 0)
        return (float)temp;

    Rt /= R_NOM;
    Rt *= 100; // normalize to 100 ohm

    double rpoly = Rt;

    temp = -242.02;
    temp += 2.2228 * rpoly;
    rpoly *= Rt; // square
    temp += 2.5859e-3 * rpoly;
    rpoly *= Rt; // ^3
    temp -= 4.8260e-6 * rpoly;
    rpoly *= Rt; // ^4
    temp -= 2.8183e-8 * rpoly;
    rpoly *= Rt; // ^5
    temp += 1.5243e-10 * rpoly;

    return (float)temp;
}

//float rtd::get_temp()
//{
//    float temp = 0;
//    temp = read_rtd_and_calculate_temperature();
////    CORETIMER_DelayMs(17);
//    return temp;
//}
