/***********************************************************************************************************************
 * Main_Controller
 * hal_wrapper.h
 *
 * wilson
 * 8/26/24
 * 12:28 AM
 *
 * Description:
 *
 **********************************************************************************************************************/

#ifndef MAIN_CONTROLLER_HAL_WRAPPER_H
#define MAIN_CONTROLLER_HAL_WRAPPER_H

/* c/c++ includes */

/* stm32 includes */
#include "stm32f4xx.h"
/* third-party includes */

/* layer_0_hal includes */

/* layer_1_rtosal includes */

/* layer_2_device includes */

/* layer_3_control includes */

/* layer_4_sys_op includes */

/* layer_n_meta_structure includes */


typedef enum
{
    BIT_CLEAR = 0x00,
    BIT_SET = 0x01
} bit_status_t;

typedef enum
{
    PORT_A      = 0x00,
    PORT_B      = 0x01,
    PORT_C      = 0x02,
    PORT_D      = 0x03,
    PORT_E      = 0x04,
    PORT_F      = 0x05,
    PORT_G      = 0x06,
    PORT_H      = 0x07,
    PORT_NULL   = 0xFF,
} port_name_t;

#define GPIO_PORT_A (hal::gpio_t*)GPIOA
#define GPIO_PORT_B (hal::gpio_t*)GPIOB
#define GPIO_PORT_C (hal::gpio_t*)GPIOC
#define GPIO_PORT_D (hal::gpio_t*)GPIOD
#define GPIO_PORT_E (hal::gpio_t*)GPIOE
#define GPIO_PORT_F (hal::gpio_t*)GPIOF
#define GPIO_PORT_G (hal::gpio_t*)GPIOG
#define GPIO_PORT_H (hal::gpio_t*)GPIOH

#define PIN_0 GPIO_PIN_0
#define PIN_1 GPIO_PIN_1
#define PIN_2 GPIO_PIN_2
#define PIN_3 GPIO_PIN_3
#define PIN_4 GPIO_PIN_4
#define PIN_5 GPIO_PIN_5
#define PIN_6 GPIO_PIN_6
#define PIN_7 GPIO_PIN_7
#define PIN_8 GPIO_PIN_8
#define PIN_9 GPIO_PIN_9
#define PIN_10 GPIO_PIN_10
#define PIN_11 GPIO_PIN_11
#define PIN_12 GPIO_PIN_12
#define PIN_13 GPIO_PIN_13
#define PIN_14 GPIO_PIN_14
#define PIN_15 GPIO_PIN_15
#define PIN_ALL GPIO_PIN_All


namespace hal
{
    typedef GPIO_TypeDef gpio_t;
    void gpio_write_pin(gpio_t* arg_port_name, uint16_t arg_gpio_pin, uint8_t arg_pin_state);
    uint8_t gpio_read_pin(gpio_t* arg_port_name, uint16_t arg_gpio_pin);
    void rtc_get_time_stamp(char arg_time_stamp_string[9]);
    void spi_2_msp_initialize();
}


#endif //MAIN_CONTROLLER_HAL_WRAPPER_H
