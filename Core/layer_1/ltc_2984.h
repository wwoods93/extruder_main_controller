/***********************************************************************************************************************
 * Main_Controller
 * ltc_2984.h
 *
 * wilson
 * 2/13/25
 * 8:30 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

#ifndef MAIN_CONTROLLER_LTC_2984_H
#define MAIN_CONTROLLER_LTC_2984_H

/* c/c++ includes */
#include <cstdint>
/* stm32 includes */

/* third-party includes */

/* layer_0_hal includes */

/* layer_1 includes */

/* layer_2 includes */

/* layer_3 includes */

/* application includes */




class ltc_2984
{
    public:

        // rtd channel assignment word [31:27] rtd type                                 // #### #--- ---- ---- ---- ---- ---- ----
        static constexpr uint32_t RTD_TYPE_MASK                         = 0xF8000000U;  // 1111 1000 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t RTD_TYPE_PT10                         = 0x50000000U;  // 0101 0000 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t RTD_TYPE_PT50                         = 0x58000000U;  // 0101 1000 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t RTD_TYPE_PT100                        = 0x60000000U;  // 0110 0000 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t RTD_TYPE_PT200                        = 0x68000000U;  // 0110 1000 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t RTD_TYPE_PT500                        = 0x70000000U;  // 0111 0000 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t RTD_TYPE_PT1000                       = 0x78000000U;  // 0111 1000 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t RTD_TYPE_RTD1000_375                  = 0x80000000U;  // 1000 0000 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t RTD_TYPE_NI_120                       = 0x88000000U;  // 1000 1000 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t RTD_TYPE_CUSTOM                       = 0x90000000U;  // 1001 0000 0000 0000 0000 0000 0000 0000
        // rtd channel assignment word [26:22] channel ptr                              // ---- -### ##-- ---- ---- ---- ---- ----
        static constexpr uint32_t SENSE_RESISTOR_CHANNEL_PTR_MASK       = 0x07C00000U;  // 0000 0111 1100 0000 0000 0000 0000 0000
        static constexpr uint32_t SENSE_RESISTOR_PTR_CH2_CH1            = 0x00800000U;  // 0000 0000 1000 0000 0000 0000 0000 0000
        static constexpr uint32_t SENSE_RESISTOR_PTR_CH3_CH2            = 0x00C00000U;  // 0000 0000 1100 0000 0000 0000 0000 0000
        static constexpr uint32_t SENSE_RESISTOR_PTR_CH4_CH3            = 0x01000000U;  // 0000 0001 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t SENSE_RESISTOR_PTR_CH5_CH4            = 0x01400000U;  // 0000 0001 0100 0000 0000 0000 0000 0000
        static constexpr uint32_t SENSE_RESISTOR_PTR_CH6_CH5            = 0x01800000U;  // 0000 0001 1000 0000 0000 0000 0000 0000
        static constexpr uint32_t SENSE_RESISTOR_PTR_CH7_CH6            = 0x01C00000U;  // 0000 0001 1100 0000 0000 0000 0000 0000
        static constexpr uint32_t SENSE_RESISTOR_PTR_CH8_CH7            = 0x02000000U;  // 0000 0010 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t SENSE_RESISTOR_PTR_CH9_CH8            = 0x02400000U;  // 0000 0010 0100 0000 0000 0000 0000 0000
        static constexpr uint32_t SENSE_RESISTOR_PTR_CH10_CH9           = 0x02800000U;  // 0000 0010 1000 0000 0000 0000 0000 0000
        static constexpr uint32_t SENSE_RESISTOR_PTR_CH11_CH10          = 0x02C00000U;  // 0000 0010 1100 0000 0000 0000 0000 0000
        static constexpr uint32_t SENSE_RESISTOR_PTR_CH12_CH11          = 0x03000000U;  // 0000 0011 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t SENSE_RESISTOR_PTR_CH13_CH12          = 0x03400000U;  // 0000 0011 0100 0000 0000 0000 0000 0000
        static constexpr uint32_t SENSE_RESISTOR_PTR_CH14_CH13          = 0x03800000U;  // 0000 0011 1000 0000 0000 0000 0000 0000
        static constexpr uint32_t SENSE_RESISTOR_PTR_CH15_CH14          = 0x03C00000U;  // 0000 0011 1100 0000 0000 0000 0000 0000
        static constexpr uint32_t SENSE_RESISTOR_PTR_CH16_CH15          = 0x04000000U;  // 0000 0100 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t SENSE_RESISTOR_PTR_CH17_CH16          = 0x04400000U;  // 0000 0100 0100 0000 0000 0000 0000 0000
        static constexpr uint32_t SENSE_RESISTOR_PTR_CH18_CH17          = 0x04800000U;  // 0000 0100 1000 0000 0000 0000 0000 0000
        static constexpr uint32_t SENSE_RESISTOR_PTR_CH19_CH18          = 0x04C00000U;  // 0000 0100 1100 0000 0000 0000 0000 0000
        static constexpr uint32_t SENSE_RESISTOR_PTR_CH20_CH19          = 0x05000000U;  // 0000 0101 0000 0000 0000 0000 0000 0000
        // rtd channel assignment word [21:20] number of wires                          // ---- ---- --## ---- ---- ---- ---- ----
        static constexpr uint32_t SENSE_CONFIG_NUM_WIRES_MASK           = 0x00300000U;  // 0000 0000 0011 0000 0000 0000 0000 0000
        static constexpr uint32_t SENSE_CONFIG_2_WIRE                   = 0x00000000U;  // 0000 0000 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t SENSE_CONFIG_3_WIRE                   = 0x00100000U;  // 0000 0000 0001 0000 0000 0000 0000 0000
        static constexpr uint32_t SENSE_CONFIG_4_WIRE                   = 0x00200000U;  // 0000 0000 0010 0000 0000 0000 0000 0000
        static constexpr uint32_t SENSE_CONFIG_4_WIRE_K_SENSE           = 0x00300000U;  // 0000 0000 0011 0000 0000 0000 0000 0000
        // rtd channel assignment word [19:18] excitation mode                          // ---- ---- ---- ##-- ---- ---- ---- ----
        static constexpr uint32_t SENSE_CONFIG_EXCITATION_MODE_MASK     = 0x000C0000U;  // 0000 0000 0000 1100 0000 0000 0000 0000
        static constexpr uint32_t SENSE_CONFIG_GND_EXT                  = 0x00000000U;  // 0000 0000 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t SENSE_CONFIG_GND_INT                  = 0x00040000U;  // 0000 0000 0000 0100 0000 0000 0000 0000
        static constexpr uint32_t SENSE_CONFIG_GND_INT_ISRC_ROTATION    = 0x00080000U;  // 0000 0000 0000 1000 0000 0000 0000 0000
        // rtd channel assignment word [17:14] excitation current                       // ---- ---- ---- --## ##-- ---- ---- ----
        static constexpr uint32_t EXCITATION_CURRENT_MASK               = 0x0003C000U;  // 0000 0000 0000 0011 1100 0000 0000 0000
        static constexpr uint32_t EXCITATION_CURRENT_5_UA               = 0x00004000U;  // 0000 0000 0000 0000 0100 0000 0000 0000
        static constexpr uint32_t EXCITATION_CURRENT_10_UA              = 0x00008000U;  // 0000 0000 0000 0000 1000 0000 0000 0000
        static constexpr uint32_t EXCITATION_CURRENT_25_UA              = 0x0000C000U;  // 0000 0000 0000 0000 1100 0000 0000 0000
        static constexpr uint32_t EXCITATION_CURRENT_50_UA              = 0x00010000U;  // 0000 0000 0000 0001 0000 0000 0000 0000
        static constexpr uint32_t EXCITATION_CURRENT_100_UA             = 0x00014000U;  // 0000 0000 0000 0001 0100 0000 0000 0000
        static constexpr uint32_t EXCITATION_CURRENT_250_UA             = 0x00018000U;  // 0000 0000 0000 0001 1000 0000 0000 0000
        static constexpr uint32_t EXCITATION_CURRENT_500_UA             = 0x0001C000U;  // 0000 0000 0000 0001 1100 0000 0000 0000
        static constexpr uint32_t EXCITATION_CURRENT_1_MA               = 0x00020000U;  // 0000 0000 0000 0010 0000 0000 0000 0000
        // rtd channel assignment word [13:12] rtd curve                                // ---- ---- ---- ---- --## ---- ---- ----
        static constexpr uint32_t RTD_CURVE_MASK                        = 0x00003000U;  // 0000 0000 0000 0000 0011 0000 0000 0000
        static constexpr uint32_t RTD_CURVE_EUROPEAN                    = 0x00000000U;  // 0000 0000 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t RTD_CURVE_AMERICAN                    = 0x00001000U;  // 0000 0000 0000 0000 0001 0000 0000 0000
        static constexpr uint32_t RTD_CURVE_JAPANESE                    = 0x00002000U;  // 0000 0000 0000 0000 0010 0000 0000 0000
        static constexpr uint32_t RTD_CURVE_ITS_90                      = 0x00003000U;  // 0000 0000 0000 0000 0011 0000 0000 0000
        static constexpr uint32_t RTD_CURVE_RTD1000_375                 = 0x00000000U;  // 0000 0000 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t RTD_CURVE_NI_120_LUT                  = 0x00000000U;  // 0000 0000 0000 0000 0000 0000 0000 0000
        // rtd channel assignment word [11:0] custom rtd ptr                            // ---- ---- ---- ---- ---- #### #### ####
        static constexpr uint32_t CUSTOM_RTD_PTR_MASK                   = 0x00000FFFU;  // 0000 0000 0000 0000 0000 1111 1111 1111
        static constexpr uint32_t CUSTOM_RTD_PTR_ADDRESS_MASK           = 0x00000FC0U;  // 0000 0000 0000 0000 0000 1111 1100 0000
        static constexpr uint32_t CUSTOM_RTD_PTR_LENGTH_MASK            = 0x0000003FU;  // 0000 0000 0000 0000 0000 0000 0011 1111

        // rtd data output word [31:24] fault reporting                                 // #### #### ---- ---- ---- ---- ---- ----
        static constexpr uint32_t RTD_FAULT_MASK                        = 0xFF000000U;  // 1111 1111 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t RTD_FAULT_SENSOR_HARD_FAULT           = 0x80000000U;  // 1000 0000 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t RTD_FAULT_ADC_OUT_OF_RANGE_HARD       = 0x40000000U;  // 0100 0000 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t RTD_FAULT_TEMP_OVER_RANGE_SOFT        = 0x08000000U;  // 0000 1000 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t RTD_FAULT_TEMP_UNDER_RANGE_SOFT       = 0x04000000U;  // 0000 0100 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t RTD_FAULT_ADC_OUT_OF_RANGE_SOFT       = 0x02000000U;  // 0000 0010 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t RTD_FAULT_MASK_RESULT_VALID           = 0x01000000U;  // 0000 0001 0000 0000 0000 0000 0000 0000

        // sense resistor assignment word [31:27] sensor type                           // #### #--- ---- ---- ---- ---- ---- ----
        static constexpr uint32_t SENSE_RESISTOR_SENSOR_TYPE            = 0xE8000000U;  // 1111 1000 0000 0000 0000 0000 0000 0000
        // sense resistor assignment word resistor value mask [26:0]                    // ---- -### #### #### #### #### #### ####
        static constexpr uint32_t SENSE_RESISTOR_VALUE_MASK             = 0x07FFFFFFU;  // 0000 0111 1111 1111 1111 1111 1111 1111

    private:
};


#endif //MAIN_CONTROLLER_LTC_2984_H
