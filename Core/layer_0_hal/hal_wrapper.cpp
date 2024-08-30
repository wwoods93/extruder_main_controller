/***********************************************************************************************************************
 * Main_Controller
 * hal_wrapper.cpp
 *
 * wilson
 * 8/26/24
 * 12:28 AM
 *
 * Description:
 *
 **********************************************************************************************************************/

/* c/c++ includes */

/* stm32 includes */
#include "stm32f4xx.h"
/* third-party includes */

/* hal includes */

/* rtosal includes */

/* driver includes */



/* sys op includes */

/* meta structure includes */

/* hal_wrapper header */
#include "hal_wrapper.h"

namespace hal
{
    void gpio_write_pin(gpio_t* arg_port_name, uint16_t arg_gpio_pin, uint8_t arg_pin_state)
    {
        HAL_GPIO_WritePin((GPIO_TypeDef *)arg_port_name, arg_gpio_pin, (GPIO_PinState)arg_pin_state);
    }

    uint8_t gpio_read_pin(gpio_t* arg_port_name, uint16_t arg_gpio_pin)
    {
        return (uint8_t)HAL_GPIO_ReadPin(arg_port_name, arg_gpio_pin);
    }

}
