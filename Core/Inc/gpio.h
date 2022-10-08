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
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
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
