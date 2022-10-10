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

class rtd
{
    public:
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
        static constexpr double RTD_A = 3.9083e-3;
        static constexpr double RTD_B = -5.775e-7;
        /* max31865 nominal and reference resistances */
        static constexpr float  R_NOM = 100.0;
        static constexpr float  R_REF = 430.0;

        typedef enum max31865_numwires
        {
            MAX31865_2WIRE = 0,
            MAX31865_3WIRE = 1,
            MAX31865_4WIRE = 0
        } max31865_numwires_t;

        rtd();
//        void SPI_RTD_init_8(void);
//        uint8_t SPI_transfer_8(uint8_t data);
        uint8_t readRegister8(uint8_t addr);
        uint16_t readRegister16(uint8_t addr1, uint8_t addr2);
        void writeRegister8(uint8_t addr, uint8_t data);
        bool rtd_begin(max31865_numwires_t wires);
        uint16_t read_rtd();
        float read_rtd_and_calculate_temperature();
//        float get_temp();
    private:
};

#endif //MAIN_CONTROLLER_DRIVER_RTD_H
