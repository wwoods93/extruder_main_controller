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

/* layer_0 includes */

/* layer_1_rtosal includes */

/* layer_1 includes */

/* layer_3_control includes */

/* layer_3 includes */

/* layer_n_meta_structure includes */


typedef enum
{
    BIT_CLEAR = 0x00,
    BIT_SET = 0x01
} bit_status_t;




#define PORT_A (hal::gpio_t*)GPIOA
#define PORT_B (hal::gpio_t*)GPIOB
#define PORT_C (hal::gpio_t*)GPIOC
#define PORT_D (hal::gpio_t*)GPIOD
#define PORT_E (hal::gpio_t*)GPIOE
#define PORT_F (hal::gpio_t*)GPIOF
#define PORT_G (hal::gpio_t*)GPIOG
#define PORT_H (hal::gpio_t*)GPIOH

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
    typedef enum
    {
        STATUS_OK       = 0x00U,
        STATUS_ERROR    = 0x01U,
        STATUS_BUSY     = 0x02U,
        STATUS_TIMEOUT  = 0x03U
    } status_t;

    typedef enum
    {
        I2C_CONTROLLER_TX_COMPLETE_CALLBACK_ID  = 0x00U,
        I2C_CONTROLLER_RX_COMPLETE_CALLBACK_ID  = 0x01U,
        I2C_PERIPHERAL_TX_COMPLETE_CALLBACK_ID  = 0x02U,
        I2C_PERIPHERAL_RX_COMPLETE_CALLBACK_ID  = 0x03U,
        I2C_LISTEN_COMPLETE_CALLBACK_ID         = 0x04U,
        I2C_MEM_TX_COMPLETE_CALLBACK_ID         = 0x05U,
        I2C_MEM_RX_COMPLETE_CALLBACK_ID         = 0x06U,
        I2C_ERROR_CALLBACK_ID                   = 0x07U,
        I2C_ABORT_CALLBACK_ID                   = 0x08U,
        I2C_MSP_INIT_CALLBACK_ID                = 0x09U,
        I2C_MSP_DE_INIT_CALLBACK_ID             = 0x0AU,
    } i2c_callback_id_t;

    typedef enum
    {
        TIMER_BASE_MSP_INIT_CALLBACK_ID                   = 0x00U,
        TIMER_BASE_MSP_DE_INIT_CALLBACK_ID                = 0x01U,
        TIMER_INPUT_CAPTURE_MSP_INIT_CALLBACK_ID          = 0x02U,
        TIMER_INPUT_CAPTURE_MSP_DE_INIT_CALLBACK_ID       = 0x03U,
        TIMER_OUTPUT_COMPARE_MSP_INIT_CALLBACK_ID         = 0x04U,
        TIMER_OUTPUT_COMPARE_MSP_DE_INIT_CALLBACK_ID      = 0x05U,
        TIMER_PWM_MSP_INIT_CALLBACK_ID                    = 0x06U,
        TIMER_PWM_MSP_DE_INIT_CALLBACK_ID                 = 0x07U,
        TIMER_ONE_PULSE_MSP_INIT_CALLBACK_ID              = 0x08U,
        TIMER_ONE_PULSE_MSP_DE_INIT_CALLBACK_ID           = 0x09U,
        TIMER_ENCODER_MSP_INIT_CALLBACK_ID                = 0x0AU,
        TIMER_ENCODER_MSP_DE_INIT_CALLBACK_ID             = 0x0BU,
        TIMER_HALL_SENSOR_MSP_INIT_CALLBACK_ID            = 0x0CU,
        TIMER_HALL_SENSOR_MSP_DE_INIT_CALLBACK_ID         = 0x0DU,
        TIMER_PERIOD_ELAPSED_CALLBACK_ID                  = 0x0EU,
        TIMER_PERIOD_ELAPSED_HALF_CALLBACK_ID             = 0x0FU,
        TIMER_TRIGGER_CALLBACK_ID                         = 0x10U,
        TIMER_TRIGGER_HALF_CALLBACK_ID                    = 0x11U,

        TIMER_INPUT_CAPTURE_CALLBACK_ID                   = 0x12U,
        TIMER_INPUT_CAPTURE_HALF_CALLBACK_ID              = 0x13U,
        TIMER_OUTPUT_COMPARE_DELAY_ELAPSED_CALLBACK_ID    = 0x14U,
        TIMER_PWM_PULSE_FINISHED_CALLBACK_ID              = 0x15U,
        TIMER_PWM_PULSE_FINISHED_HALF_CALLBACK_ID         = 0x16U,
        TIMER_ERROR_CALLBACK_ID                           = 0x17U,
        TIMER_COMMUTATION_CALLBACK_ID                     = 0x18U,
        TIMER_COMMUTATION_HALF_CALLBACK_ID                = 0x19U,
        TIMER_BREAK_CALLBACK_ID                           = 0x1AU,
    } timer_callback_id_t;


    static constexpr uint32_t TIMER_CHANNEL_1   = 0x00000000U;
    static constexpr uint32_t TIMER_CHANNEL_2   = 0x00000004U;
    static constexpr uint32_t TIMER_CHANNEL_3   = 0x00000008U;
    static constexpr uint32_t TIMER_CHANNEL_4   = 0x0000000CU;
    static constexpr uint32_t TIMER_CHANNEL_ALL = 0x0000003CU;

    typedef TIM_HandleTypeDef timer_handle_t;
    typedef  void (*timer_callback_t)(timer_handle_t *arg_timer_handle);

    status_t timer_register_callback(timer_handle_t* arg_timer_handle, timer_callback_id_t arg_callback_id_t, timer_callback_t arg_callback);
    status_t timer_input_capture_start_interrupt(timer_handle_t* arg_timer_handle, uint32_t arg_channel);
    status_t timer_time_base_start(timer_handle_t* arg_timer_handle);

    typedef I2C_HandleTypeDef i2c_handle_t;
    typedef  void (*i2c_callback_t)(i2c_handle_t *arg_i2c_handle);

    status_t i2c_register_callback(i2c_handle_t* arg_i2c_handle, i2c_callback_id_t arg_callback_id, i2c_callback_t arg_callback);
    status_t i2c_controller_transmit_interrupt(i2c_handle_t* arg_i2c_handle, uint16_t arg_address, uint8_t* arg_data, uint16_t arg_size);

    // gpio
    typedef GPIO_TypeDef gpio_t;
    void gpio_write_pin(gpio_t* arg_port_name, uint16_t arg_gpio_pin, uint8_t arg_pin_state);
    uint8_t gpio_read_pin(gpio_t* arg_port_name, uint16_t arg_gpio_pin);
    void gpio_toggle_pin(gpio_t* arg_port_name, uint16_t arg_gpio_pin);

    // rtc
    void rtc_get_time_stamp(char arg_time_stamp_string[9]);

    // spi
    void spi_1_msp_initialize();
    void spi_2_msp_initialize();
}


#endif //MAIN_CONTROLLER_HAL_WRAPPER_H
