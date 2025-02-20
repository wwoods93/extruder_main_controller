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
    
        // rtd channel assignment word [31:27] rtd type                                     // #### #--- ---- ---- ---- ---- ---- ----
        static constexpr uint32_t   RTD_TYPE_MASK                           = 0xF8000000U;  // 1111 1000 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t   RTD_TYPE_PT10                           = 0x50000000U;  // 0101 0000 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t   RTD_TYPE_PT50                           = 0x58000000U;  // 0101 1000 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t   RTD_TYPE_PT100                          = 0x60000000U;  // 0110 0000 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t   RTD_TYPE_PT200                          = 0x68000000U;  // 0110 1000 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t   RTD_TYPE_PT500                          = 0x70000000U;  // 0111 0000 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t   RTD_TYPE_PT1000                         = 0x78000000U;  // 0111 1000 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t   RTD_TYPE_RTD1000_375                    = 0x80000000U;  // 1000 0000 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t   RTD_TYPE_NI_120                         = 0x88000000U;  // 1000 1000 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t   RTD_TYPE_CUSTOM                         = 0x90000000U;  // 1001 0000 0000 0000 0000 0000 0000 0000
        // rtd channel assignment word [26:22] channel ptr                                  // ---- -### ##-- ---- ---- ---- ---- ----
        static constexpr uint32_t   SENSE_RESISTOR_CHANNEL_PTR_MASK         = 0x07C00000U;  // 0000 0111 1100 0000 0000 0000 0000 0000
        static constexpr uint32_t   SENSE_RESISTOR_PTR_CH2_CH1              = 0x00800000U;  // 0000 0000 1000 0000 0000 0000 0000 0000
        static constexpr uint32_t   SENSE_RESISTOR_PTR_CH3_CH2              = 0x00C00000U;  // 0000 0000 1100 0000 0000 0000 0000 0000
        static constexpr uint32_t   SENSE_RESISTOR_PTR_CH4_CH3              = 0x01000000U;  // 0000 0001 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t   SENSE_RESISTOR_PTR_CH5_CH4              = 0x01400000U;  // 0000 0001 0100 0000 0000 0000 0000 0000
        static constexpr uint32_t   SENSE_RESISTOR_PTR_CH6_CH5              = 0x01800000U;  // 0000 0001 1000 0000 0000 0000 0000 0000
        static constexpr uint32_t   SENSE_RESISTOR_PTR_CH7_CH6              = 0x01C00000U;  // 0000 0001 1100 0000 0000 0000 0000 0000
        static constexpr uint32_t   SENSE_RESISTOR_PTR_CH8_CH7              = 0x02000000U;  // 0000 0010 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t   SENSE_RESISTOR_PTR_CH9_CH8              = 0x02400000U;  // 0000 0010 0100 0000 0000 0000 0000 0000
        static constexpr uint32_t   SENSE_RESISTOR_PTR_CH10_CH9             = 0x02800000U;  // 0000 0010 1000 0000 0000 0000 0000 0000
        static constexpr uint32_t   SENSE_RESISTOR_PTR_CH11_CH10            = 0x02C00000U;  // 0000 0010 1100 0000 0000 0000 0000 0000
        static constexpr uint32_t   SENSE_RESISTOR_PTR_CH12_CH11            = 0x03000000U;  // 0000 0011 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t   SENSE_RESISTOR_PTR_CH13_CH12            = 0x03400000U;  // 0000 0011 0100 0000 0000 0000 0000 0000
        static constexpr uint32_t   SENSE_RESISTOR_PTR_CH14_CH13            = 0x03800000U;  // 0000 0011 1000 0000 0000 0000 0000 0000
        static constexpr uint32_t   SENSE_RESISTOR_PTR_CH15_CH14            = 0x03C00000U;  // 0000 0011 1100 0000 0000 0000 0000 0000
        static constexpr uint32_t   SENSE_RESISTOR_PTR_CH16_CH15            = 0x04000000U;  // 0000 0100 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t   SENSE_RESISTOR_PTR_CH17_CH16            = 0x04400000U;  // 0000 0100 0100 0000 0000 0000 0000 0000
        static constexpr uint32_t   SENSE_RESISTOR_PTR_CH18_CH17            = 0x04800000U;  // 0000 0100 1000 0000 0000 0000 0000 0000
        static constexpr uint32_t   SENSE_RESISTOR_PTR_CH19_CH18            = 0x04C00000U;  // 0000 0100 1100 0000 0000 0000 0000 0000
        static constexpr uint32_t   SENSE_RESISTOR_PTR_CH20_CH19            = 0x05000000U;  // 0000 0101 0000 0000 0000 0000 0000 0000
        // rtd channel assignment word [21:20] number of wires                              // ---- ---- --## ---- ---- ---- ---- ----
        static constexpr uint32_t   SENSE_CONFIG_NUM_WIRES_MASK             = 0x00300000U;  // 0000 0000 0011 0000 0000 0000 0000 0000
        static constexpr uint32_t   SENSE_CONFIG_2_WIRE                     = 0x00000000U;  // 0000 0000 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t   SENSE_CONFIG_3_WIRE                     = 0x00100000U;  // 0000 0000 0001 0000 0000 0000 0000 0000
        static constexpr uint32_t   SENSE_CONFIG_4_WIRE                     = 0x00200000U;  // 0000 0000 0010 0000 0000 0000 0000 0000
        static constexpr uint32_t   SENSE_CONFIG_4_WIRE_K_SENSE             = 0x00300000U;  // 0000 0000 0011 0000 0000 0000 0000 0000
        // rtd channel assignment word [19:18] excitation mode                              // ---- ---- ---- ##-- ---- ---- ---- ----
        static constexpr uint32_t   SENSE_CONFIG_EXCITATION_MODE_MASK       = 0x000C0000U;  // 0000 0000 0000 1100 0000 0000 0000 0000
        static constexpr uint32_t   SENSE_CONFIG_GND_EXT                    = 0x00000000U;  // 0000 0000 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t   SENSE_CONFIG_GND_INT                    = 0x00040000U;  // 0000 0000 0000 0100 0000 0000 0000 0000
        static constexpr uint32_t   SENSE_CONFIG_GND_INT_ISRC_ROTATION      = 0x00080000U;  // 0000 0000 0000 1000 0000 0000 0000 0000
        // rtd channel assignment word [17:14] excitation current                           // ---- ---- ---- --## ##-- ---- ---- ----
        static constexpr uint32_t   EXCITATION_CURRENT_MASK                 = 0x0003C000U;  // 0000 0000 0000 0011 1100 0000 0000 0000
        static constexpr uint32_t   EXCITATION_CURRENT_5_UA                 = 0x00004000U;  // 0000 0000 0000 0000 0100 0000 0000 0000
        static constexpr uint32_t   EXCITATION_CURRENT_10_UA                = 0x00008000U;  // 0000 0000 0000 0000 1000 0000 0000 0000
        static constexpr uint32_t   EXCITATION_CURRENT_25_UA                = 0x0000C000U;  // 0000 0000 0000 0000 1100 0000 0000 0000
        static constexpr uint32_t   EXCITATION_CURRENT_50_UA                = 0x00010000U;  // 0000 0000 0000 0001 0000 0000 0000 0000
        static constexpr uint32_t   EXCITATION_CURRENT_100_UA               = 0x00014000U;  // 0000 0000 0000 0001 0100 0000 0000 0000
        static constexpr uint32_t   EXCITATION_CURRENT_250_UA               = 0x00018000U;  // 0000 0000 0000 0001 1000 0000 0000 0000
        static constexpr uint32_t   EXCITATION_CURRENT_500_UA               = 0x0001C000U;  // 0000 0000 0000 0001 1100 0000 0000 0000
        static constexpr uint32_t   EXCITATION_CURRENT_1_MA                 = 0x00020000U;  // 0000 0000 0000 0010 0000 0000 0000 0000
        // rtd channel assignment word [13:12] rtd curve                                    // ---- ---- ---- ---- --## ---- ---- ----
        static constexpr uint32_t   RTD_CURVE_MASK                          = 0x00003000U;  // 0000 0000 0000 0000 0011 0000 0000 0000
        static constexpr uint32_t   RTD_CURVE_EUROPEAN                      = 0x00000000U;  // 0000 0000 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t   RTD_CURVE_AMERICAN                      = 0x00001000U;  // 0000 0000 0000 0000 0001 0000 0000 0000
        static constexpr uint32_t   RTD_CURVE_JAPANESE                      = 0x00002000U;  // 0000 0000 0000 0000 0010 0000 0000 0000
        static constexpr uint32_t   RTD_CURVE_ITS_90                        = 0x00003000U;  // 0000 0000 0000 0000 0011 0000 0000 0000
        static constexpr uint32_t   RTD_CURVE_RTD1000_375                   = 0x00000000U;  // 0000 0000 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t   RTD_CURVE_NI_120_LUT                    = 0x00000000U;  // 0000 0000 0000 0000 0000 0000 0000 0000
        // rtd channel assignment word [11:0] custom rtd ptr                                // ---- ---- ---- ---- ---- #### #### ####
        static constexpr uint32_t   CUSTOM_RTD_PTR_MASK                     = 0x00000FFFU;  // 0000 0000 0000 0000 0000 1111 1111 1111
        static constexpr uint32_t   CUSTOM_RTD_PTR_ADDRESS_MASK             = 0x00000FC0U;  // 0000 0000 0000 0000 0000 1111 1100 0000
        static constexpr uint32_t   CUSTOM_RTD_PTR_LENGTH_MASK              = 0x0000003FU;  // 0000 0000 0000 0000 0000 0000 0011 1111

        // rtd data output word [31:24] fault reporting                                     // #### #### ---- ---- ---- ---- ---- ----
        static constexpr uint32_t   RTD_FAULT_MASK                          = 0xFF000000U;  // 1111 1111 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t   RTD_FAULT_SENSOR_HARD_FAULT             = 0x80000000U;  // 1000 0000 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t   RTD_FAULT_ADC_OUT_OF_RANGE_HARD         = 0x40000000U;  // 0100 0000 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t   RTD_FAULT_TEMP_OVER_RANGE_SOFT          = 0x08000000U;  // 0000 1000 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t   RTD_FAULT_TEMP_UNDER_RANGE_SOFT         = 0x04000000U;  // 0000 0100 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t   RTD_FAULT_ADC_OUT_OF_RANGE_SOFT         = 0x02000000U;  // 0000 0010 0000 0000 0000 0000 0000 0000
        static constexpr uint32_t   RTD_FAULT_MASK_RESULT_VALID             = 0x01000000U;  // 0000 0001 0000 0000 0000 0000 0000 0000

        // sense resistor assignment word [31:27] sensor type                               // #### #--- ---- ---- ---- ---- ---- ----
        static constexpr uint32_t   SENSE_RESISTOR_SENSOR_TYPE              = 0xE8000000U;  // 1111 1000 0000 0000 0000 0000 0000 0000
        // sense resistor assignment word resistor value mask [26:0]                        // ---- -### #### #### #### #### #### ####
        static constexpr uint32_t   SENSE_RESISTOR_VALUE_MASK               = 0x07FFFFFFU;  // 0000 0111 1111 1111 1111 1111 1111 1111

        static constexpr uint32_t   SENSE_RESISTOR_VALUE_10K                = 0x009C40CDU;

         static constexpr uint8_t    CH1_ID                                  = 0x01U;
         static constexpr uint8_t    CH2_ID                                  = 0x02U;
         static constexpr uint8_t    CH3_ID                                  = 0x03U;
         static constexpr uint8_t    CH4_ID                                  = 0x04U;
         static constexpr uint8_t    CH5_ID                                  = 0x05U;
         static constexpr uint8_t    CH6_ID                                  = 0x06U;
         static constexpr uint8_t    CH7_ID                                  = 0x07U;
         static constexpr uint8_t    CH8_ID                                  = 0x08U;
         static constexpr uint8_t    CH9_ID                                  = 0x09U;
         static constexpr uint8_t    CH10_ID                                  = 0x0AU;
         static constexpr uint8_t    CH11_ID                                  = 0x0BU;
         static constexpr uint8_t    CH12_ID                                  = 0x0CU;
         static constexpr uint8_t    CH13_ID                                  = 0x0DU;
         static constexpr uint8_t    CH14_ID                                  = 0x0EU;
         static constexpr uint8_t    CH15_ID                                  = 0x0FU;
         static constexpr uint8_t    CH16_ID                                  = 0x10U;
         static constexpr uint8_t    CH17_ID                                  = 0x11U;
         static constexpr uint8_t    CH18_ID                                  = 0x12U;
         static constexpr uint8_t    CH19_ID                                  = 0x13U;
         static constexpr uint8_t    CH20_ID                                  = 0x14U;

        static constexpr uint8_t    START_CONVERSION_START_BIT              = 0x80U;
        static constexpr uint8_t    START_CONVERSION_MULTIPLE               = 0x80U;        // 1000 0000
        static constexpr uint8_t    START_CONVERSION_CH1                    = 0x81U;        // 1000 0001
        static constexpr uint8_t    START_CONVERSION_CH2                    = 0x82U;        // 1000 0010
        static constexpr uint8_t    START_CONVERSION_CH3                    = 0x83U;        // 1000 0011
        static constexpr uint8_t    START_CONVERSION_CH4                    = 0x84U;        // 1000 0100
        static constexpr uint8_t    START_CONVERSION_CH5                    = 0x85U;        // 1000 0101
        static constexpr uint8_t    START_CONVERSION_CH6                    = 0x86U;        // 1000 0110
        static constexpr uint8_t    START_CONVERSION_CH7                    = 0x87U;        // 1000 0111
        static constexpr uint8_t    START_CONVERSION_CH8                    = 0x88U;        // 1000 1000
        static constexpr uint8_t    START_CONVERSION_CH9                    = 0x89U;        // 1000 1001
        static constexpr uint8_t    START_CONVERSION_CH10                   = 0x8AU;        // 1000 1010
        static constexpr uint8_t    START_CONVERSION_CH11                   = 0x8BU;        // 1000 1011
        static constexpr uint8_t    START_CONVERSION_CH12                   = 0x8CU;        // 1000 1100
        static constexpr uint8_t    START_CONVERSION_CH13                   = 0x8DU;        // 1000 1101
        static constexpr uint8_t    START_CONVERSION_CH14                   = 0x8EU;        // 1000 1110
        static constexpr uint8_t    START_CONVERSION_CH15                   = 0x8FU;        // 1000 1111
        static constexpr uint8_t    START_CONVERSION_CH16                   = 0x90U;        // 1001 0000
        static constexpr uint8_t    START_CONVERSION_CH17                   = 0x91U;        // 1001 0001
        static constexpr uint8_t    START_CONVERSION_CH18                   = 0x92U;        // 1001 0010
        static constexpr uint8_t    START_CONVERSION_CH19                   = 0x93U;        // 1001 0011
        static constexpr uint8_t    START_CONVERSION_CH20                   = 0x94U;        // 1001 0100

        static constexpr uint16_t   COMMAND_STATUS_REG_ADDR                 = 0x0000U;
        static constexpr uint8_t    COMMAND_STATUS_REG_ADDR_MSB             = 0x00U;
        static constexpr uint8_t    COMMAND_STATUS_REG_ADDR_LSB             = 0x00U;
        static constexpr uint16_t   CONVERSION_RESULT_BASE_ADDR             = 0x0010U;
        static constexpr uint16_t   CHANNEL_ASSIGNMENT_BASE_ADDR            = 0x0200U;
        static constexpr uint16_t   CUSTOM_SENSOR_DATA_BASE_ADDR            = 0x0250U;

        static constexpr uint16_t   EEPROM_START_ADDRESS                    = 0x00B0U;
        static constexpr uint16_t   EEPROM_REG_ADDR_KEY_3                   = 0x00B0U;
        static constexpr uint16_t   EEPROM_REG_ADDR_KEY_2                   = 0x00B1U;
        static constexpr uint16_t   EEPROM_REG_ADDR_KEY_1                   = 0x00B2U;
        static constexpr uint16_t   EEPROM_REG_ADDR_KEY_0                   = 0x00B3U;
        static constexpr uint16_t   EEPROM_REG_ADDR_STATUS_REG              = 0x00F9U;
        static constexpr uint16_t   EEPROM_REG_ADDR_READ_RESULT             = 0x00D0U;

        static constexpr uint8_t    EEPROM_START_BIT                        = 0x80U;
        static constexpr uint8_t    EEPROM_READ_COMMAND                     = 0x96U;        // EEPROM_START_BIT | 0x16
        static constexpr uint8_t    EEPROM_WRITE_COMMAND                    = 0x95U;        // EEPROM_START_BIT | 0x15
        static constexpr uint8_t    EEPROM_READ_SUCCESS                     = 0x00U;
        static constexpr uint8_t    EEPROM_READ_FAIL                        = 0xFFU;
        static constexpr uint32_t   EEPROM_KEY_VALUE                        = 0xA53C0F5AU;

        static constexpr uint8_t    EEPROM_STATUS_REG_BIT_CHECKSUM_ERROR    = 0x08U;
        static constexpr uint8_t    EEPROM_STATUS_REG_BIT_PROGRAM_FAIL      = 0x04U;
        static constexpr uint8_t    EEPROM_STATUS_REG_BIT_ECC_FAIL          = 0x02U;
        static constexpr uint8_t    EEPROM_STATUS_REG_BIT_ECC_USED          = 0x01U;

        static constexpr uint8_t    CHANNEL_COUNT                           = 21U;
        static constexpr uint8_t    CHANNEL_CONFIG_DATA_SIZE                = 4U;
        static constexpr uint8_t    CONVERSION_RESULT_DATA_SIZE             = 4U;
        static constexpr uint16_t   CH1_CONFIG_DATA_START_ADDR              = 0x0200U;
        static constexpr uint16_t   CH2_CONFIG_DATA_START_ADDR              = 0x0204U;
        static constexpr uint16_t   CH3_CONFIG_DATA_START_ADDR              = 0x0208U;
        static constexpr uint16_t   CH4_CONFIG_DATA_START_ADDR              = 0x020CU;
        static constexpr uint16_t   CH5_CONFIG_DATA_START_ADDR              = 0x0210U;
        static constexpr uint16_t   CH6_CONFIG_DATA_START_ADDR              = 0x0214U;
        static constexpr uint16_t   CH7_CONFIG_DATA_START_ADDR              = 0x0218U;
        static constexpr uint16_t   CH8_CONFIG_DATA_START_ADDR              = 0x021CU;
        static constexpr uint16_t   CH9_CONFIG_DATA_START_ADDR              = 0x0220U;
        static constexpr uint16_t   CH10_CONFIG_DATA_START_ADDR             = 0x0224U;
        static constexpr uint16_t   CH11_CONFIG_DATA_START_ADDR             = 0x0228U;
        static constexpr uint16_t   CH12_CONFIG_DATA_START_ADDR             = 0x022CU;
        static constexpr uint16_t   CH13_CONFIG_DATA_START_ADDR             = 0x0230U;
        static constexpr uint16_t   CH14_CONFIG_DATA_START_ADDR             = 0x0234U;
        static constexpr uint16_t   CH15_CONFIG_DATA_START_ADDR             = 0x0238U;
        static constexpr uint16_t   CH16_CONFIG_DATA_START_ADDR             = 0x023CU;
        static constexpr uint16_t   CH17_CONFIG_DATA_START_ADDR             = 0x0240U;
        static constexpr uint16_t   CH18_CONFIG_DATA_START_ADDR             = 0x0244U;
        static constexpr uint16_t   CH19_CONFIG_DATA_START_ADDR             = 0x0248U;
        static constexpr uint16_t   CH20_CONFIG_DATA_START_ADDR             = 0x024CU;

        static constexpr uint16_t   CH1_CONVERSION_RESULT_START_ADDR        = 0x0010U;
        static constexpr uint16_t   CH2_CONVERSION_RESULT_START_ADDR        = 0x0014U;
        static constexpr uint16_t   CH3_CONVERSION_RESULT_START_ADDR        = 0x0018U;
        static constexpr uint16_t   CH4_CONVERSION_RESULT_START_ADDR        = 0x001CU;
        static constexpr uint16_t   CH5_CONVERSION_RESULT_START_ADDR        = 0x0020U;
        static constexpr uint16_t   CH6_CONVERSION_RESULT_START_ADDR        = 0x0024U;
        static constexpr uint16_t   CH7_CONVERSION_RESULT_START_ADDR        = 0x0028U;
        static constexpr uint16_t   CH8_CONVERSION_RESULT_START_ADDR        = 0x002CU;
        static constexpr uint16_t   CH9_CONVERSION_RESULT_START_ADDR        = 0x0030U;
        static constexpr uint16_t   CH10_CONVERSION_RESULT_START_ADDR       = 0x0034U;
        static constexpr uint16_t   CH11_CONVERSION_RESULT_START_ADDR       = 0x0038U;
        static constexpr uint16_t   CH12_CONVERSION_RESULT_START_ADDR       = 0x003CU;
        static constexpr uint16_t   CH13_CONVERSION_RESULT_START_ADDR       = 0x0040U;
        static constexpr uint16_t   CH14_CONVERSION_RESULT_START_ADDR       = 0x0044U;
        static constexpr uint16_t   CH15_CONVERSION_RESULT_START_ADDR       = 0x0048U;
        static constexpr uint16_t   CH16_CONVERSION_RESULT_START_ADDR       = 0x004CU;
        static constexpr uint16_t   CH17_CONVERSION_RESULT_START_ADDR       = 0x0050U;
        static constexpr uint16_t   CH18_CONVERSION_RESULT_START_ADDR       = 0x0054U;
        static constexpr uint16_t   CH19_CONVERSION_RESULT_START_ADDR       = 0x0058U;
        static constexpr uint16_t   CH20_CONVERSION_RESULT_START_ADDR       = 0x005CU;

        static constexpr uint8_t    SPI_INSTRUCTION_READ_RAM                = 0x03U;
        static constexpr uint8_t    SPI_INSTRUCTION_WRITE_RAM               = 0x02U;
        static constexpr uint8_t    SPI_PACKET_NUM_BYTES                    = 7U;

        typedef enum
        {
            CH_MULTIPLE = (uint8_t)0x00U,
            CH1         = (uint8_t)0x01U,
            CH2         = (uint8_t)0x02U,
            CH3         = (uint8_t)0x03U,
            CH4         = (uint8_t)0x04U,
            CH5         = (uint8_t)0x05U,
            CH6         = (uint8_t)0x06U,
            CH7         = (uint8_t)0x07U,
            CH8         = (uint8_t)0x08U,
            CH9         = (uint8_t)0x09U,
            CH10        = (uint8_t)0x0AU,
            CH11        = (uint8_t)0x0BU,
            CH12        = (uint8_t)0x0CU,
            CH13        = (uint8_t)0x0DU,
            CH14        = (uint8_t)0x0EU,
            CH15        = (uint8_t)0x0FU,
            CH16        = (uint8_t)0x10U,
            CH17        = (uint8_t)0x11U,
            CH18        = (uint8_t)0x12U,
            CH19        = (uint8_t)0x13U,
            CH20        = (uint8_t)0x14U,
        } channel_id_e;

        spi* spi_instance;
        int16_t spi_channel;
        spi::chip_select_t chip_select {};
        hal::timer_handle_t* timer_handle;
        uint32_t channel_assignment_words[CHANNEL_COUNT] {};



    ltc_2984();
    void initialize(spi& arg_spi_instance, int16_t arg_spi_channel_id, hal::gpio_t* arg_chip_select_port, uint16_t arg_chip_select_pin, hal::timer_handle_t* arg_timer_handle);
    void rtd_channel_assign(channel_id_e arg_ch_id, uint32_t arg_rtd_type, uint32_t arg_resistor_ptr, uint32_t arg_num_wires, uint32_t arg_exc_mode, uint32_t arg_exc_current, uint32_t arg_rtd_curve, uint32_t arg_custom_rtd, uint32_t arg_custom_rtd_len);
    void sense_resistor_channel_assign(channel_id_e arg_ch_id) const;
    void initiate_conversion(channel_id_e arg_ch_id) const;
    void read_conversion_result(channel_id_e arg_ch_id);
    static uint32_t get_channel_config_data_address(channel_id_e arg_ch_id);






    private:
};


#endif //MAIN_CONTROLLER_LTC_2984_H
