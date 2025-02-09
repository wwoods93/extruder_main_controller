/***********************************************************************************************************************
 * Main_Controller
 * system_operation_extrusion_process.cpp
 *
 * wilson
 * 11/6/22
 * 3:51 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

#include <cstdint>
#include <cstring>
#include "stm32f4xx_it.h"
#include "../layer_0/hal.h"
#include "../layer_0/hal_general.h"
#include "../layer_0/rtosal_globals.h"
#include "../layer_0/rtosal.h"
#include "../layer_1/device.h"
#include "../layer_1/device_callback.h"

#include "../application/extruder.h"
#include "sys_op_temp_control.h"


static constexpr uint8_t TEMP_CONTROL_INIT     = 0U;
static constexpr uint8_t TEMP_CONTROL_RUN      = 1U;

spi::module_t spi_2_handle;

int16_t rtd_0_channel_id = ID_INVALID;
int16_t rtd_1_channel_id = ID_INVALID;
int16_t rtd_2_channel_id = ID_INVALID;

uint8_t zone_1_demand = 50U;
uint8_t zone_2_demand = 10U;
uint8_t zone_3_demand = 10U;

uint8_t tx_bytes[8] = { rtd::CONFIG_REGISTER_ADDRESS | WRITE_REGISTER_ADDRESS_MASK, rtd::RTD_CONFIG_REG_BYTE, rtd::MSB_REGISTER_ADDRESS_FOR_READ & 0x7F, DUMMY_BYTE, rtd::LSB_REGISTER_ADDRESS_FOR_READ & 0x7F, DUMMY_BYTE, 0, 0 };
uint8_t bytes_per_tx[8] = { 2, 4, 0, 0, 0, 0, 0, 0 };
namespace sys_op::temp_control
{
    rtosal::message_queue_handle_t comms_handler_output_data_queue_handle = nullptr;
    rtosal::event_flag_handle_t initialization_event_flags_handle = nullptr;

    char debug_msg[] = "period is 8000";

    uint8_t rtd_setup[2] = { rtd::CONFIG_REGISTER_ADDRESS | WRITE_REGISTER_ADDRESS_MASK, rtd::RTD_CONFIG_REG_BYTE };
    uint8_t rtd_setup_rx[2] = { 0, 0 };
    uint8_t rtd_read[4] = { rtd::MSB_REGISTER_ADDRESS_FOR_READ & 0x7F, DUMMY_BYTE, rtd::LSB_REGISTER_ADDRESS_FOR_READ & 0x7F, DUMMY_BYTE };
    uint8_t rtd_read_rx[4] = { 0, 0, 0, 0 };
    spi::packet_t spi_packet;

    void task_intitialize()
    {

    }

    void task_state_machine()
    {
        static uint8_t state = TEMP_CONTROL_INIT;

        switch (state)
        {
            case TEMP_CONTROL_INIT:
            {
                initialization_event_flags_handle = get_initialization_event_flags_handle();

                rtosal::event_flag_wait(initialization_event_flags_handle, READY_FOR_USER_INIT_FLAG, rtosal::OS_FLAGS_ANY, rtosal::OS_WAIT_FOREVER);

                comms_handler_output_data_queue_handle = get_comms_handler_output_data_queue_handle();

                hal::timer_register_callback(get_timer_1_handle(), hal::TIMER_INPUT_CAPTURE_CALLBACK_ID, device_callback_tim_1_input_capture_pulse_detected_callback);
                hal::timer_2_initialize();
                hal::timer_6_initialize();
                hal::timer_time_base_start(get_timer_2_handle());
                hal::timer_time_base_start(get_timer_6_handle());

                hal::spi_2.initialize(&spi_2_handle, SPI_2_ID, get_timer_2_handle());
                hal::spi_2.create_channel(rtd_0_channel_id, PORT_B, GPIO_PIN_14);
                hal::spi_2.create_channel(rtd_1_channel_id, PORT_B, GPIO_PIN_15);
                hal::spi_2.create_channel(rtd_2_channel_id, PORT_B, GPIO_PIN_1);

                device::rtd_zone_1.initialize(hal::spi_2, PORT_B, PIN_14, 0U, comms_handler_output_data_queue_handle, get_timer_6_handle(), 21.34, 20.45);
                device::rtd_zone_2.initialize(hal::spi_2, PORT_B, PIN_15, 1U, comms_handler_output_data_queue_handle, get_timer_6_handle(), 23.15, 20.45);
                device::rtd_zone_3.initialize(hal::spi_2, PORT_B, PIN_1,  2U, comms_handler_output_data_queue_handle, get_timer_6_handle(), 22.67, 20.45);

                device::zone_1_band_heater.initialize(TEMPERATURE_ZONE_1, TIMER_10_ID);
                device::zone_2_band_heater.initialize(TEMPERATURE_ZONE_2, TIMER_13_ID);
                device::zone_3_band_heater.initialize(TEMPERATURE_ZONE_3, TIMER_14_ID);

                hal::timer_input_capture_start_interrupt(get_timer_1_handle(), hal::TIMER_CHANNEL_2);

                state = TEMP_CONTROL_RUN;

                break;
            }
            case TEMP_CONTROL_RUN:
            {
                device::rtd_zone_1.read();
                device::rtd_zone_2.read();
                device::rtd_zone_3.read();
                device::zone_1_band_heater.set_demand(zone_1_demand);
                device::zone_2_band_heater.set_demand(zone_2_demand);
                device::zone_3_band_heater.set_demand(zone_3_demand);

                rtosal::thread_yield();

                break;
            }
            default:
                break;
        }
    }
}
