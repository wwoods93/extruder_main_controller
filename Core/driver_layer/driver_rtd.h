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


        #define CELSIUS_MIN     0

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
        static constexpr double RTD_RESISTANCE_RATIO_SCALE_FACTOR = RTD_RESISTANCE_REFERENCE / RESISTANCE_RATIO_DIVISOR;

        uint32_t temperature_to_resistance_pt1000_lookup_table[510] =
        {
                100000, 100390, 100780, 101170, 101560, 101950, 102340, 102730, 103120, 103510,
                103900, 104290, 104680, 105070, 105460, 105850, 106240, 106630, 107020, 107400,
                107790, 108180, 108570, 108960, 109350, 109730, 110120, 110510, 110900, 111290,
                111670, 112060, 112450, 112830, 113220, 113610, 114000, 114380, 114770, 115150,
                115540, 115930, 116310, 116700, 117080, 117470, 117860, 118240, 118630, 119010,
                119400, 119780, 120170, 120550, 120940, 121320, 121710, 122090, 122470, 122860,
                123240, 123630, 124010, 124390, 124780, 125160, 125540, 125930, 126310, 126690,
                127080, 127460, 127840, 128220, 128610, 128990, 129370, 129750, 130130, 130520,
                130900, 131280, 131660, 132040, 132420, 132800, 133180, 133570, 133950, 134330,
                134710, 135090, 135470, 135850, 136230, 136610, 136990, 137370, 137750, 138130,
                138510, 138880, 139260, 139640, 140020, 140400, 140780, 141160, 141540, 141910,
                142290, 142670, 143050, 143430, 143800, 144180, 144560, 144940, 145310, 145690,
                146070, 146440, 146820, 147200, 147570, 147950, 148330, 148700, 149080, 149460,
                149830, 150210, 150580, 150960, 151330, 151710, 152080, 152460, 152830, 153210,
                153580, 153960, 154330, 154710, 155080, 155460, 155830, 156200, 156580, 156950,
                157330, 157700, 158070, 158450, 158820, 159190, 159560, 159940, 160310, 160680,
                161050, 161430, 161800, 162170, 162540, 162910, 163290, 163660, 164030, 164400,
                164770, 165140, 165510, 165890, 166260, 166630, 167000, 167370, 167740, 168110,
                168480, 168850, 169220, 169590, 169960, 170330, 170700, 171070, 171430, 171800,
                172170, 172540, 172910, 173280, 173650, 174020, 174380, 174750, 175120, 175490,
                175860, 176220, 176590, 176960, 177330, 177690, 178060, 178430, 178790, 179160,
                179530, 179890, 180260, 180630, 180990, 181360, 181720, 182090, 182460, 182820,
                183190, 183550, 183920, 184280, 184650, 185010, 185380, 185740, 186110, 186470,
                186840, 187200, 187560, 187930, 188290, 188660, 189020, 189380, 189750, 190110,
                190470, 190840, 191200, 191560, 191920, 192290, 192650, 193010, 193370, 193740,
                194100, 194460, 194820, 195180, 195550, 195910, 196270, 196630, 196990, 197350,
                197710, 198070, 198430, 198790, 199150, 199510, 199870, 200230, 200590, 200950,
                201310, 201670, 202030, 202390, 202750, 203110, 203470, 203830, 204190, 204550,
                204900, 205260, 205620, 205980, 206340, 206700, 207050, 207410, 207770, 208130,
                208480, 208840, 209200, 209560, 209910, 210270, 210630, 210980, 211340, 211700,
                212050, 212410, 212760, 213120, 213480, 213830, 214190, 214540, 214900, 215250,
                215610, 215960, 216320, 216670, 217030, 217380, 217740, 218090, 218440, 218800,
                219150, 219510, 219860, 220210, 220570, 220920, 221270, 221630, 221980, 222330,
                222680, 223040, 223390, 223740, 224090, 224450, 224800, 225150, 225500, 225850,
                226210, 226560, 226910, 227260, 227610, 227960, 228310, 228660, 229020, 229370,
                229720, 230070, 230420, 230770, 231120, 231470, 231820, 232170, 232520, 232870,
                233210, 233560, 233910, 234260, 234610, 234960, 235310, 235660, 236000, 236350,
                236700, 237050, 237400, 237740, 238090, 238440, 238790, 239130, 239480, 239830,
                240180, 240520, 240870, 241220, 241560, 241910, 242260, 242600, 242950, 243290,
                243640, 243990, 244330, 244680, 245020, 245370, 245710, 246060, 246400, 246750,
                247090, 247440, 247780, 248130, 248470, 248810, 249160, 249500, 249850, 250190,
                250530, 250880, 251220, 251560, 251910, 252250, 252590, 252930, 253280, 253620,
                253960, 254300, 254650, 254990, 255330, 255670, 256010, 256350, 256700, 257040,
                257380, 257720, 258060, 258400, 258740, 259080, 259420, 259760, 260100, 260440,
                260780, 261120, 261460, 261800, 262140, 262480, 262820, 263160, 263500, 263840,
                264180, 264520, 264860, 265200, 265530, 265870, 266210, 266550, 266890, 267220,
                267560, 267900, 268240, 268570, 268910, 269250, 269590, 269920, 270260, 270600,
                270930, 271270, 271610, 271940, 272280, 272610, 272950, 273290, 273620, 273960,
                274290, 274630, 274960, 275300, 275630, 275970, 276300, 276640, 276970, 277310,
                277640, 277980, 278310, 278640, 278980, 279310, 279640, 279980, 280310, 280640,
                280980, 281310, 281640, 281980, 282310, 282640, 282970, 283310, 283640, 283970
        };

        typedef enum max31865_numwires
        {
            MAX31865_2WIRE = 0,
            MAX31865_3WIRE = 1,
            MAX31865_4WIRE = 0
        } max31865_numwires_t;

        void initialize_rtd(spi* spi_object);
        friend GPIO_TypeDef* get_chip_select_port(rtd* rtd_object);
        friend uint16_t get_chip_select_pin(rtd* rtd_object);
        friend void reset_chip_select_port_and_pin(rtd* rtd_object);


        uint8_t readRegister8(uint8_t addr, GPIO_TypeDef* port, uint16_t pin);
        uint16_t readRegister16(uint8_t addr1, uint8_t addr2, GPIO_TypeDef* port, uint16_t pin);
        void writeRegister8(uint8_t addr, uint8_t data, GPIO_TypeDef* port, uint16_t pin);
        bool rtd_begin(max31865_numwires_t wires, GPIO_TypeDef* port, uint16_t pin);
        uint16_t read_rtd(GPIO_TypeDef* port, uint16_t pin);
        float read_rtd_and_calculate_temperature(GPIO_TypeDef* port, uint16_t pin);
        uint32_t search_temperature_to_resistance_pt1000_lookup_table(uint32_t rtd_resistance);
        float rtd_resistance_to_temperature_celsius (uint32_t rtd_resistance);

    private:
};



#endif //MAIN_CONTROLLER_DRIVER_RTD_H
