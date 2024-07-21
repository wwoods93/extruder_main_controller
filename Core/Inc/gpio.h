/***********************************************************************************************************************
 * Main_Controller
 * gpio.h
 *
 * wilson
 * 7/12/22
 * 9:35 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

//
// Created by wilson on 7/12/22.
//

#ifndef MAIN_CONTROLLER_GPIO_H
#define MAIN_CONTROLLER_GPIO_H


#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define STEPPER_PULSE_Pin GPIO_PIN_0
#define STEPPER_PULSE_GPIO_Port GPIOA
#define STEPPER_DIR_Pin GPIO_PIN_1
#define STEPPER_DIR_GPIO_Port GPIOA
#define STEPPER_ENABLE_Pin GPIO_PIN_4
#define STEPPER_ENABLE_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define STEPPER_ALARM_Pin GPIO_PIN_0
#define STEPPER_ALARM_GPIO_Port GPIOB
#define SPI2_CS3_Pin GPIO_PIN_1
#define SPI2_CS3_GPIO_Port GPIOB
#define SPI2_CS2_Pin GPIO_PIN_15
#define SPI2_CS2_GPIO_Port GPIOB
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA

#ifdef __cplusplus
extern "C" {
#endif

void MX_GPIO_Init(void);

#ifdef __cplusplus
}
#endif

#endif //MAIN_CONTROLLER_GPIO_H
