/***********************************************************************************************************************
 * Main_Controller
 * rtos_abstraction_layer.cpp
 *
 * wilson
 * 11/4/22
 * 12:11 AM
 *
 * Description:
 *
 **********************************************************************************************************************/

#include "rtos_task_driver.h"
#include <cstdint>
#include "cmsis_os.h"
#include "stm32f4xx.h"
#include "mcu_clock_timers.h"
#include "gpio.h"
#include "spi.h"
#include "../driver_layer/driver_dc_motor_controller.h"
#include "../hardware_abstraction_layer/hal_general.h"
#include "../hardware_abstraction_layer/hal_spi.h"
#include "../hardware_abstraction_layer/hal_i2c.h"
#include "../driver_layer/driver_rtd.h"

spi::spi_handle_t hspi2;

void HAL_SPI_TxRxCpltCallback(spi::spi_handle_t *hspi)
{
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET)
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
}

void HAL_SPI_ErrorCallback(spi::spi_handle_t *hspi)
{
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET)
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
}

namespace hal
{
    spi spi_2;
}
namespace driver
{
    dc_motor_controller motor_controller_1;
    //rtd rtd_1(&hspi2);
}

void SPI2_IRQHandler()
{
    spi_irq_handler(&hal::spi_2);
}




void run_initialization_task_functions()
{
    for(;;)
    {



        osDelay(1);
    }
}

void run_preparation_process_task_functions()
{
    for(;;)
    {



        osDelay(1);
    }
}

void run_extrusion_process_task_functions()
{
    for(;;)
    {



        osDelay(1);
    }
}

void run_spooling_process_task_functions()
{
    for(;;)
    {



        osDelay(1);
    }
}

void run_comms_updater_task_functions()
{
    hal::spi_2.configure_module(reinterpret_cast<spi::spi_handle_t *>(&hspi2));
    hal::spi_2.spi_register_callback((spi::spi_callback_id_t )spi::SPI_TX_RX_COMPLETE_CALLBACK_ID, HAL_SPI_TxRxCpltCallback);
    hal::spi_2.spi_register_callback((spi::spi_callback_id_t )spi::SPI_TX_RX_COMPLETE_CALLBACK_ID, HAL_SPI_ErrorCallback);

    uint8_t spi_byte = 0xC2;
    uint8_t rx_data = 0;

    uint8_t chip_select = 3;
    uint8_t tx_size = 2;
    uint8_t tx_bytes[2] = {8, 32};

    uint8_t* ptr;

    ptr = &tx_bytes[0];


    float temp_1 = 0;
    timers_initialize();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    uint32_t count = 0;
    uint8_t motor_command_data[3] = { 0x05, 0x07, 0x02 };
    static uint32_t led_timer = 0;

    for(;;)
    {
        //i2c_motor_set_speed(0, 100);
        if (ms_timer() - led_timer > 999)
        {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            hal::spi_2.add_packet_to_buffer(chip_select, tx_size, ptr);
            if (count % 5 == 0)
                hal::spi_2.process_spi_buffer();
            count++;
            led_timer = ms_timer();
        }
        osDelay(1);
    }
}
