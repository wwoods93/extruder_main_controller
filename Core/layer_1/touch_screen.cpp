/***********************************************************************************************************************
 * Main_Controller
 * touch_screen.cpp
 *
 * wilson
 * 10/13/24
 * 9:09 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

/* c/c++ includes */
#include <cstdint>
/* stm32 includes */
#include "stm32f4xx.h"
/* third-party includes */

/* layer_0_hal includes */
#include "../layer_0/hal.h"
#include "../layer_0/hal_wrapper.h"
/* layer_1_rtosal includes */
#include "../layer_0/rtosal.h"
#include "../layer_0/rtosal_wrapper.h"
/* layer_2_device includes */

/* layer_3_control includes */

/* layer_4_sys_op includes */

/* layer_n_meta_structure includes */

/* touch_screen header */
#include "touch_screen.h"

#include "../application/global_id.h"
#include "../application/extruder.h"
#include "../utility/utility.h"


void touch_screen::initialize(I2C_HandleTypeDef *arg_i2c_module, rtosal::message_queue_handle_t arg_message_queue_handle)
{
    i2c_module = arg_i2c_module;
    message_queue_handle = arg_message_queue_handle;
    touch_screen_state = STATE_SEND_RTD_READINGS;
    current_zone_rtd = TEMPERATURE_ZONE_1;
}

void touch_screen::get_intertask_output_data()
{
    if (rtosal::message_queue_receive(message_queue_handle, &received_data, 0U) == 0U)
    {
        switch (received_data.id)
        {
            case ZONE_1_TEMP_GLOBAL_ID:
            {
                zone_1_rtd_reading = received_data.value;
                zone_1_rtd_reading_is_fresh = 1U;
                break;
            }
            case ZONE_2_TEMP_GLOBAL_ID:
            {
                zone_2_rtd_reading = received_data.value;
                zone_2_rtd_reading_is_fresh = 1U;
                break;
            }
            case ZONE_3_TEMP_GLOBAL_ID:
            {
                zone_3_rtd_reading = received_data.value;
                zone_3_rtd_reading_is_fresh = 1U;

                break;
            }
            default:
            {
                break;
            }
        }
    }
    else
    {

    }
}

void touch_screen::update_output()
{
    if (get_timer_2_handle()->Instance->CNT - touch_screen_iteration_tick > 100000U)
    {
        switch (touch_screen_state)
        {
            case STATE_SEND_RTD_READINGS:
            {
                switch (current_zone_rtd)
                {
                    case TEMPERATURE_ZONE_1:
                    {
                        if (zone_1_rtd_reading_is_fresh)
                        {
                            utility::convert_float_to_uint8_array(zone_1_rtd_reading, converter_result);
                            hal::i2c_build_packet_array_from_converted_bytes(i2c_data, current_zone_rtd, converter_result);
                            zone_1_rtd_reading_is_fresh = 0U;
                        }

                        current_zone_rtd = TEMPERATURE_ZONE_2;
                        break;
                    }
                    case TEMPERATURE_ZONE_2:
                    {
                        if (zone_2_rtd_reading_is_fresh)
                        {
                            utility::convert_float_to_uint8_array(zone_2_rtd_reading, converter_result);
                            hal::i2c_build_packet_array_from_converted_bytes(i2c_data, current_zone_rtd, converter_result);
                            zone_2_rtd_reading_is_fresh = 0U;
                        }

                        current_zone_rtd = TEMPERATURE_ZONE_3;
                        break;
                    }
                    case TEMPERATURE_ZONE_3:
                    {
                        if (zone_3_rtd_reading_is_fresh)
                        {
                            utility::convert_float_to_uint8_array(zone_3_rtd_reading, converter_result);
                            hal::i2c_build_packet_array_from_converted_bytes(i2c_data, current_zone_rtd, converter_result);
                            zone_3_rtd_reading_is_fresh = 0U;
                        }

                        current_zone_rtd = TEMPERATURE_ZONE_1;
                        touch_screen_state = STATE_SEND_SPI_REQUEST_COUNT;
                        break;
                    }
                    default:
                    {
                        break;
                    }
                }

                hal::i2c_controller_transmit_interrupt(i2c_module, (0x14 << 1), i2c_data, 5U);

                break;
            }
            case STATE_SEND_SPI_REQUEST_COUNT:
            {
                utility::convert_uint32_to_uint8_array(hal::spi_2.get_packets_requested_count(), converter_result);
                hal::i2c_build_packet_array_from_converted_bytes(i2c_data, 0x07, converter_result);
                hal::i2c_controller_transmit_interrupt(i2c_module, (0x14 << 1), i2c_data, 5U);
                touch_screen_state = STATE_SEND_SPI_RECEIVE_COUNT;
                break;
            }
            case STATE_SEND_SPI_RECEIVE_COUNT:
            {
                utility::convert_uint32_to_uint8_array(hal::spi_2.get_packets_received_count(), converter_result);
                hal::i2c_build_packet_array_from_converted_bytes(i2c_data, 0x06, converter_result);
                hal::i2c_controller_transmit_interrupt(i2c_module, (0x14 << 1), i2c_data, 5U);
                touch_screen_state = STATE_SEND_RTD_READINGS;
                break;
            }
            default:
            {
                touch_screen_state = STATE_SEND_RTD_READINGS;
                break;
            }
        }
        touch_screen_iteration_tick = get_timer_2_handle()->Instance->CNT;
    }
}
