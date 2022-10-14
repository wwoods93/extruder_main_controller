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
#include "peripheral_initialization.h"
#include "mcu_clock_timers.h"
#include "driver_rtd.h"

SPI_HandleTypeDef hspi2;

// spi2 MOSI    PC1
// spi2 MISO    PC2
// GPIO OUT     PC8
// GPIO OUT     PC7
// GPIO OUT     PB15    -   CHIP SELECT
// GPIO OUT     PB14

void MX_SPI2_Init()
{
    /* USER CODE BEGIN SPI2_Init 0 */

    /* USER CODE END SPI2_Init 0 */

    /* USER CODE BEGIN SPI2_Init 1 */

    /* USER CODE END SPI2_Init 1 */
    /* SPI2 parameter configuration*/
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI2_Init 2 */

    /* USER CODE END SPI2_Init 2 */

}

rtd::rtd() = default;

//void rtd::SPI_RTD_init_8( void )
//{
//    uint32_t rdata = 0U;
//    IEC1CLR = 0x8;
//    IEC1CLR = 0x10;
//    IEC1CLR = 0x20;
//    SPI1CON = 0;
//    rdata = SPI1BUF;
//    rdata = rdata;
//    IFS1CLR = 0x8;
//    IFS1CLR = 0x10;
//    IFS1CLR = 0x20;
//    SPI1BRG = 23;
//    SPI1STATCLR = _SPI1STAT_SPIROV_MASK;
//    SPI1CON = 0x8220;
//}

//uint8_t rtd::SPI_transfer_8( uint8_t data )
//{
//    SPI_RTD_init_8();
//    SPI1BUF = data;
//    while (SPI1STATbits.SPIRBE);
//    return (uint8_t)SPI1BUF;
//}


uint8_t rtd::readRegister8(uint8_t addr)
{
    addr &= 0x7F;
    uint8_t rx_data = 0;
    HAL_SPI_TransmitReceive_IT(&hspi2, &addr, &rx_data, 1);

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
    uint8_t tx_data[4] = {tx_msb, tx_full_byte, tx_lsb, tx_full_byte};
    uint8_t rx_1 = 0;
    uint8_t rx_2 = 0;
    uint8_t rx_3 = 0;
    uint8_t rx_4 = 0;
    //uint8_t rx_data[2] = {rx_1, rx_2};
    uint8_t rx_data[4] = {rx_1, rx_2, rx_3, rx_4 };
    // set cs low
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
//    HAL_SPI_TransmitReceive_IT(&hspi2, tx_data, rx_data, 2);
    HAL_SPI_TransmitReceive_IT(&hspi2, tx_data, rx_data, 4);
    ret = rx_data[1];
    ret <<= 8;
    // tx_data[1] = tx_lsb;
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
//    HAL_SPI_TransmitReceive_IT(&hspi2, tx_data, rx_data, 2);

    ret |= rx_data[3];
    // us_delay(1);
    // set cs high


    return ret;
}


void rtd::writeRegister8(uint8_t addr, uint8_t data)
{
    uint8_t tx_data[2] = {addr, data};
    uint8_t rx_data[2] = {0, 0};
    addr |= 0x80;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_IT(&hspi2, tx_data, rx_data, 2);
}

bool rtd::rtd_begin(max31865_numwires_t wires)
{
//    GPIO_PinOutputEnable(RTD_CS);
//    GPIO_PinSet(RTD_CS);
//    SPI_RTD_init_8();
//    CORETIMER_DelayUs(100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
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
