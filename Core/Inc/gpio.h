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
#define RELAY_A1_Pin GPIO_PIN_1
#define RELAY_A1_GPIO_Port GPIOA
#define RELAY_A4_Pin GPIO_PIN_4
#define RELAY_A4_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define SPI1_CS2_Pin GPIO_PIN_4
#define SPI1_CS2_GPIO_Port GPIOC
#define RELAY_C5_Pin GPIO_PIN_5
#define RELAY_C5_GPIO_Port GPIOC
#define RELAY_B0_Pin GPIO_PIN_0
#define RELAY_B0_GPIO_Port GPIOB
#define SPI2_CS3_Pin GPIO_PIN_1
#define SPI2_CS3_GPIO_Port GPIOB
#define RELAY_B2_Pin GPIO_PIN_2
#define RELAY_B2_GPIO_Port GPIOB
#define SPI2_CS1_Pin GPIO_PIN_14
#define SPI2_CS1_GPIO_Port GPIOB
#define SPI2_CS2_Pin GPIO_PIN_15
#define SPI2_CS2_GPIO_Port GPIOB
#define RELAY_C6_Pin GPIO_PIN_6
#define RELAY_C6_GPIO_Port GPIOC
#define LTC_2984_INTERRUPT_Pin GPIO_PIN_7
#define LTC_2984_INTERRUPT_GPIO_Port GPIOC
#define RELAY_C8_Pin GPIO_PIN_8
#define RELAY_C8_GPIO_Port GPIOC
#define RELAY_C9_Pin GPIO_PIN_9
#define RELAY_C9_GPIO_Port GPIOC
#define SPI1_CS1_Pin GPIO_PIN_10
#define SPI1_CS1_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define RELAY_B9_Pin GPIO_PIN_9
#define RELAY_B9_GPIO_Port GPIOB

#ifdef __cplusplus
extern "C" {
#endif

void MX_GPIO_Init(void);

#ifdef __cplusplus
}
#endif

#endif //MAIN_CONTROLLER_GPIO_H
