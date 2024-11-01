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
#ifndef MAIN_CONTROLLER_RTD_H
#define MAIN_CONTROLLER_RTD_H

#include <cstdint>
#include "stm32f4xx.h"
#include "../meta_structure/meta_structure_system_manager.h"
#include "../meta_structure/meta_structure_user.h"
#include "../layer_0/hal_spi.h"
#include "../layer_0/rtosal.h"

class rtd
{
    public:


        typedef enum
        {
            READ_RATE_10_HZ = 0x00U,
            READ_RATE_5_HZ = 0x01U,
            READ_RATE_2_HZ = 0x02U,
            READ_RATE_1_HZ = 0x03U
        } read_rate_t;


        typedef struct
        {
            uint8_t bias_setting;
            uint8_t conversion_mode;
            uint8_t rtd_type;
            uint8_t fault_status_clear_mode;
            uint8_t notch_filter_setting;
        } rtd_sensor_t;

        rtosal::message_queue_handle_t request_queue_handle;
        rtosal::message_queue_handle_t result_queue_handle;
        rtosal::message_queue_handle_t output_queue_handle;

        common_float_data_t rtd_reading;

        int16_t channel_id = ID_INVALID;

        #define CELSIUS_MIN                     0
        #define CELSIUS_MAX                     509
        #define RESISTANCE_RATIO_DIVISOR        32768

        #define READ_REGISTER_ADDRESS_MASK      0x7F
        #define WRITE_REGISTER_ADDRESS_MASK     0x80
        #define DUMMY_BYTE                      0xFF

        static constexpr uint32_t READING_PERIOD_MS = 500U;

        /* config register bytes */
        static constexpr uint8_t CONFIG_REGISTER_ADDRESS                        = 0x00;
        static constexpr uint8_t CONFIG_REGISTER_SET_BIAS                       = 0x80;
        static constexpr uint8_t CONFIG_REGISTER_SET_CONVERSION_MODE_AUTO       = 0x40;
        static constexpr uint8_t CONFIG_REGISTER_SET_CONVERSION_MODE_OFF        = 0x00;
        static constexpr uint8_t CONFIG_REGISTER_SET_CONVERSION_MODE_1_SHOT     = 0x20;
        static constexpr uint8_t CONFIG_REGISTER_SET_RTD_TYPE_3_WIRE            = 0x10;
        static constexpr uint8_t CONFIG_REGISTER_SET_RTD_TYPE_2_OR_4_WIRE       = 0x00;
        static constexpr uint8_t CONFIG_REGISTER_FAULT_STATUS_CLEAR             = 0x02;
        static constexpr uint8_t CONFIG_REGISTER_SET_NOTCH_FILTER_50_HZ         = 0x01;
        static constexpr uint8_t CONFIG_REGISTER_SET_NOTCH_FILTER_60_HZ         = 0x00;

        static constexpr uint8_t RTD_CONFIG_REG_BYTE = CONFIG_REGISTER_SET_BIAS
                                                     | CONFIG_REGISTER_SET_CONVERSION_MODE_AUTO
                                                     | CONFIG_REGISTER_SET_RTD_TYPE_3_WIRE
                                                     | CONFIG_REGISTER_FAULT_STATUS_CLEAR
                                                     | CONFIG_REGISTER_SET_NOTCH_FILTER_50_HZ;
        /* register addresses */
        static constexpr uint8_t MSB_REGISTER_ADDRESS                           = 0x01;
        static constexpr uint8_t LSB_REGISTER_ADDRESS                           = 0x02;
        static constexpr uint8_t MSB_REGISTER_ADDRESS_FOR_READ = MSB_REGISTER_ADDRESS & READ_REGISTER_ADDRESS_MASK;
        static constexpr uint8_t LSB_REGISTER_ADDRESS_FOR_READ = LSB_REGISTER_ADDRESS & READ_REGISTER_ADDRESS_MASK;
        static constexpr uint8_t HIGH_FAULT_MSB_REGISTER                        = 0x03;
        static constexpr uint8_t HIGH_FAULT_LSB_REGISTER                        = 0x04;
        static constexpr uint8_t LOW_FAULT_MSB_REGISTER                         = 0x05;
        static constexpr uint8_t LOW_FAULT_LSB_REGISTER                         = 0x06;
        static constexpr uint8_t FAULT_STATUS_REGISTER                          = 0x07;
        /* fault codes */
        static constexpr uint8_t FAULT_HIGH_THRESHOLD                           = 0x80;
        static constexpr uint8_t FAULT_LOW_THRESHOLD                            = 0x40;
        static constexpr uint8_t FAULT_REFIN_LOW                                = 0x20;
        static constexpr uint8_t FAULT_REFIN_HIGH                               = 0x10;
        static constexpr uint8_t FAULT_RTDIN_LOW                                = 0x08;
        static constexpr uint8_t FAULT_OVER_UNDER_VOLTAGE                       = 0x04;
        /* nominal and reference resistances */
        static constexpr float  RTD_RESISTANCE_NOMINAL = 1000.0;
        static constexpr float  RTD_RESISTANCE_REFERENCE = 4300.0;
        static constexpr double RTD_RESISTANCE_RATIO_SCALE_FACTOR = RTD_RESISTANCE_REFERENCE / RESISTANCE_RATIO_DIVISOR;

        uint32_t temperature_to_resistance_pt1000_lookup_table[510] =
        {
                 100000,  100390,  100780,  101170, 101560, 101950,   102340,  102730,  103120, 103510,
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



        hal::timer_handle_t* reading_timer_handle;
        float temperature_celsius_current_reading = 0;
        float temperature_celsius_moving_average = 1;
        uint8_t moving_average_sample_count = 50U;
        uint32_t reading_request_tick = 0U;

        double rtd_resistance_scaled_and_rounded{};

        uint8_t initialized = 0U;

        uint8_t complete_tx[8] = { CONFIG_REGISTER_ADDRESS | WRITE_REGISTER_ADDRESS_MASK, RTD_CONFIG_REG_BYTE, MSB_REGISTER_ADDRESS_FOR_READ & 0x7F, DUMMY_BYTE, LSB_REGISTER_ADDRESS_FOR_READ & 0x7F, DUMMY_BYTE, 0, 0 };
        uint8_t bytes_per_tx[8] = {2, 4, 0, 0, 0, 0, 0, 0 };

        rtd();

        void initialize(read_rate_t _read_rate_hz, int16_t arg_channel_id, rtosal::message_queue_handle_t arg_request_queue_handle, rtosal::message_queue_handle_t arg_result_queue_handle, rtosal::message_queue_handle_t arg_output_queue_handle, hal::timer_handle_t* arg_reading_timer_handle);
        float read();
        uint8_t request_reading();
        float receive_reading_and_output_moving_average();
        float calculate_and_send_moving_average();

        uint16_t get_msb_and_lsb_register_bytes_and_concatenate(common_packet_t& arg_common_packet);

        float compute_temperature_moving_average();
        [[nodiscard]] float get_device_reading_degrees_celsius() const;
        uint32_t search_temperature_to_resistance_pt1000_lookup_table(uint32_t rtd_resistance);
        float rtd_resistance_to_temperature_celsius (uint32_t rtd_resistance);
//        void configure_rtd(user_config_t& _user_config);

    private:
};



#endif //MAIN_CONTROLLER_RTD_H
