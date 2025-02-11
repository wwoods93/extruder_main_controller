/***********************************************************************************************************************
 * Main_Controller
 * touch_screen.h
 *
 * wilson
 * 10/13/24
 * 9:09 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

#ifndef MAIN_CONTROLLER_TOUCH_SCREEN_H
#define MAIN_CONTROLLER_TOUCH_SCREEN_H

/* c/c++ includes */
#include <cstdint>
/* stm32 includes */
#include "stm32f4xx.h"
/* third-party includes */

/* layer_0_hal includes */

/* layer_1_rtosal includes */
#include "../layer_0/rtosal.h"
/* layer_2_device includes */

/* layer_3_control includes */

/* layer_3 includes */

/* layer_n_meta_structure includes */




class touch_screen
{
    public:

        static constexpr uint8_t STATE_SEND_RTD_READINGS = 0U;
        static constexpr uint8_t STATE_SEND_SPI_REQUEST_COUNT = 1U;
        static constexpr uint8_t STATE_SEND_SPI_RECEIVE_COUNT = 2U;

        float zone_1_rtd_reading = 0.0;
        float zone_2_rtd_reading = 0.0;
        float zone_3_rtd_reading = 0.0;

        uint8_t zone_1_rtd_reading_is_fresh = 0U;
        uint8_t zone_2_rtd_reading_is_fresh = 0U;
        uint8_t zone_3_rtd_reading_is_fresh = 0U;


        uint32_t touch_screen_iteration_tick = 0U;
        uint8_t converter_result[4] = { 0, 0, 0, 0 };
        rtosal::common_float_data_t received_data;

        uint8_t current_zone_rtd;
        uint8_t touch_screen_state;

        uint8_t i2c_data[5] = { 0, 0, 0, 0, 0 };
        I2C_HandleTypeDef* i2c_module;
        rtosal::message_queue_handle_t message_queue_handle;

        void initialize(I2C_HandleTypeDef* arg_i2c_module, rtosal::message_queue_handle_t arg_message_queue_handle);
        void get_intertask_output_data();
        void update_output();

    private:


};


#endif //MAIN_CONTROLLER_TOUCH_SCREEN_H
