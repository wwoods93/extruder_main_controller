/***********************************************************************************************************************
 * Main_Controller
 * driver_rtd.h
 *
 * wilson
 * 10/9/22
 * 11:34 PM
 *
 * Description:
 *
 **********************************************************************************************************************/
#ifndef MAIN_CONTROLLER_DRIVER_RTD_H
#define MAIN_CONTROLLER_DRIVER_RTD_H

#include <cstdint>
#include "stm32f4xx.h"

#include "../hardware_abstraction_layer/hal_spi.h"

class rtd
{
    public:
        spi* rtd_spi_object;
        spi::handle_t* spi_peripheral;

        GPIO_TypeDef* chip_select_port;
        uint16_t chip_select_pin;

        GPIO_TypeDef* chip_select_1_port = GPIOB;
        GPIO_TypeDef* chip_select_2_port = GPIOC;
        GPIO_TypeDef* chip_select_3_port = GPIOC;

        uint16_t chip_select_1_pin = GPIO_PIN_14;
        uint16_t chip_select_2_pin = GPIO_PIN_7;
        uint16_t chip_select_3_pin = GPIO_PIN_8;



        #define RESISTANCE_RATIO_DIVISOR            32768
        #define INITIAL_CALCULATED_TEMPERATURE      -242.02
        #define DEGREE_1_COEFFICIENT                2.2228
        #define DEGREE_2_COEFFICIENT                2.5859e-3
        #define DEGREE_3_COEFFICIENT                4.8260e-6
        #define DEGREE_4_COEFFICIENT                2.8183e-8
        #define DEGREE_5_COEFFICIENT                1.5243e-10

        /* max31865 configuration bytes */
        static constexpr uint8_t MAX31865_CONFIG_REG        = 0x00;
        static constexpr uint8_t MAX31865_CONFIG_BIAS       = 0x80;
        static constexpr uint8_t MAX31865_CONFIG_MODEAUTO   = 0x40;
        static constexpr uint8_t AX31865_CONFIG_MODEOFF     = 0x00;
        static constexpr uint8_t MAX31865_CONFIG_1SHOT      = 0x20;
        static constexpr uint8_t MAX31865_CONFIG_3WIRE      = 0x10;
        static constexpr uint8_t MAX31865_CONFIG_24WIRE     = 0x00;
        static constexpr uint8_t MAX31865_CONFIG_FAULTSTAT  = 0x02;
        static constexpr uint8_t MAX31865_CONFIG_FILT50HZ   = 0x01;
        static constexpr uint8_t MAX31865_CONFIG_FILT60HZ   = 0x00;
        /* max31865 register access bytes */
        static constexpr uint8_t MAX31865_RTDMSB_REG        = 0x01;
        static constexpr uint8_t MAX31865_RTDLSB_REG        = 0x02;
        static constexpr uint8_t MAX31865_HFAULTMSB_REG     = 0x03;
        static constexpr uint8_t MAX31865_HFAULTLSB_REG     = 0x04;
        static constexpr uint8_t MAX31865_LFAULTMSB_REG     = 0x05;
        static constexpr uint8_t MAX31865_LFAULTLSB_REG     = 0x06;
        static constexpr uint8_t MAX31865_FAULTSTAT_REG     = 0x07;
        /* max31865 fault code bytes */
        static constexpr uint8_t MAX31865_FAULT_HIGHTHRESH  = 0x80;
        static constexpr uint8_t MAX31865_FAULT_LOWTHRESH   = 0x40;
        static constexpr uint8_t MAX31865_FAULT_REFINLOW    = 0x20;
        static constexpr uint8_t MAX31865_FAULT_REFINHIGH   = 0x10;
        static constexpr uint8_t MAX31865_FAULT_RTDINLOW    = 0x08;
        static constexpr uint8_t MAX31865_FAULT_OVUV        = 0x04;
        /* max31865 interpolation polynomial coefficients */
        static constexpr double RTD_FACTOR_1 = 3.9083e-3;
        static constexpr double RTD_FACTOR_2 = -5.775e-7;
        /* max31865 nominal and reference resistances */
        static constexpr float  RTD_RESISTANCE_NOMINAL = 1000.0;
        static constexpr float  RTD_RESISTANCE_REFERENCE = 4300.0;

        typedef enum max31865_numwires
        {
            MAX31865_2WIRE = 0,
            MAX31865_3WIRE = 1,
            MAX31865_4WIRE = 0
        } max31865_numwires_t;

//        explicit rtd(spi* spi_object);
//        rtd();

        void initialize_rtd(spi* spi_object);
//        void SPI_RTD_init_8(void);
//        uint8_t SPI_transfer_8(uint8_t data);
//        void MX_SPI2_Init();
        friend GPIO_TypeDef* get_chip_select_port(rtd* rtd_object);
        friend uint16_t get_chip_select_pin(rtd* rtd_object);
        friend void reset_chip_select_port_and_pin(rtd* rtd_object);

        uint8_t readRegister8(uint8_t addr, GPIO_TypeDef* port, uint16_t pin);
        uint16_t readRegister16(uint8_t addr1, uint8_t addr2, GPIO_TypeDef* port, uint16_t pin);
        void writeRegister8(uint8_t addr, uint8_t data, GPIO_TypeDef* port, uint16_t pin);
        bool rtd_begin(max31865_numwires_t wires, GPIO_TypeDef* port, uint16_t pin);
        uint16_t read_rtd(GPIO_TypeDef* port, uint16_t pin);
        float read_rtd_and_calculate_temperature(GPIO_TypeDef* port, uint16_t pin);
//        float get_temp();
    private:
};



#endif //MAIN_CONTROLLER_DRIVER_RTD_H
