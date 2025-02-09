///***********************************************************************************************************************
// * Main_Controller
// * hal_gpio.h
// *
// * wilson
// * 3/3/23
// * 11:55 PM
// *
// * Description:
// *
// **********************************************************************************************************************/
//
//#ifndef MAIN_CONTROLLER_HAL_GPIO_H
//#define MAIN_CONTROLLER_HAL_GPIO_H
//
/* c/c++ includes */
//
///* stm32 includes */
//#include "stm32f4xx_hal_def.h"
///* third-party includes */
//
///* layer_0 includes */
//
///* layer_1_rtosal includes */
//
///* layer_1 includes */
//
///* layer_3_control includes */
//
///* layer_3 includes */
//
///* layer_n_meta_structure includes */
//
//
//
//typedef struct
//{
//    uint32_t pin;       /* Pin !< Specifies the GPIO pins to be configured.
//                           This parameter can be any value of @ref GPIO_pins_define */
//
//    uint32_t pin_mode;      /* Mode!< Specifies the operating mode for the selected pins.
//                           This parameter can be a value of @ref GPIO_mode_define */
//
//    uint32_t pull_resistor;      /* Pull !< Specifies the Pull-up or Pull-Down activation for the selected pins.
//                           This parameter can be a value of @ref GPIO_pull_define */
//
//    uint32_t pin_speed;     /* Speed !< Specifies the speed for the selected pins.
//                           This parameter can be a value of @ref GPIO_speed_define */
//
//    uint32_t alternate_function;  /* Alternate !< Peripheral to be connected to the selected pins.
//                            This parameter can be a value of @ref GPIO_Alternate_function_selection */
//}hal_gpio_init_t;
//
//typedef enum
//{
//    HAL_GPIO_PIN_RESET = 0,
//    HAL_GPIO_PIN_SET
//}hal_gpio_pin_state_t;
//
//
//#define HAL_GPIO_PIN_0                 ((uint16_t)0x0001)  /* Pin 0 selected    */
//#define HAL_GPIO_PIN_1                 ((uint16_t)0x0002)  /* Pin 1 selected    */
//#define HAL_GPIO_PIN_2                 ((uint16_t)0x0004)  /* Pin 2 selected    */
//#define HAL_GPIO_PIN_3                 ((uint16_t)0x0008)  /* Pin 3 selected    */
//#define HAL_GPIO_PIN_4                 ((uint16_t)0x0010)  /* Pin 4 selected    */
//#define HAL_GPIO_PIN_5                 ((uint16_t)0x0020)  /* Pin 5 selected    */
//#define HAL_GPIO_PIN_6                 ((uint16_t)0x0040)  /* Pin 6 selected    */
//#define HAL_GPIO_PIN_7                 ((uint16_t)0x0080)  /* Pin 7 selected    */
//#define HAL_GPIO_PIN_8                 ((uint16_t)0x0100)  /* Pin 8 selected    */
//#define HAL_GPIO_PIN_9                 ((uint16_t)0x0200)  /* Pin 9 selected    */
//#define HAL_GPIO_PIN_10                ((uint16_t)0x0400)  /* Pin 10 selected   */
//#define HAL_GPIO_PIN_11                ((uint16_t)0x0800)  /* Pin 11 selected   */
//#define HAL_GPIO_PIN_12                ((uint16_t)0x1000)  /* Pin 12 selected   */
//#define HAL_GPIO_PIN_13                ((uint16_t)0x2000)  /* Pin 13 selected   */
//#define HAL_GPIO_PIN_14                ((uint16_t)0x4000)  /* Pin 14 selected   */
//#define HAL_GPIO_PIN_15                ((uint16_t)0x8000)  /* Pin 15 selected   */
//#define HAL_GPIO_PIN_All               ((uint16_t)0xFFFF)  /* All pins selected */
//
//#define HAL_GPIO_PIN_MASK              0x0000FFFFU /* PIN mask for assert test */
///**
//  * @}
//  */
//
///** @defgroup GPIO_mode_define GPIO mode define
//  * @brief GPIO Configuration Mode
//  *        Elements values convention: 0x00WX00YZ
//  *           - W  : EXTI trigger detection on 3 bits
//  *           - X  : EXTI mode (IT or Event) on 2 bits
//  *           - Y  : Output type (Push Pull or Open Drain) on 1 bit
//  *           - Z  : GPIO mode (Input, Output, Alternate or Analog) on 2 bits
//  * @{
//  */
//
//#define  GPIO_MODE_INPUT                        MODE_INPUT                                                  /*!< Input Floating Mode                   */
//#define  GPIO_MODE_OUTPUT_PP                    (MODE_OUTPUT | OUTPUT_PP)                                   /*!< Output Push Pull Mode                 */
//#define  GPIO_MODE_OUTPUT_OD                    (MODE_OUTPUT | OUTPUT_OD)                                   /*!< Output Open Drain Mode                */
//#define  GPIO_MODE_AF_PP                        (MODE_AF | OUTPUT_PP)                                       /*!< Alternate Function Push Pull Mode     */
//#define  GPIO_MODE_AF_OD                        (MODE_AF | OUTPUT_OD)                                       /*!< Alternate Function Open Drain Mode    */
//
//#define  GPIO_MODE_ANALOG                       MODE_ANALOG                                                 /*!< Analog Mode  */
//
//#define  GPIO_MODE_IT_RISING                    (MODE_INPUT | EXTI_IT | TRIGGER_RISING)                     /*!< External Interrupt Mode with Rising edge trigger detection          */
//#define  GPIO_MODE_IT_FALLING                   (MODE_INPUT | EXTI_IT | TRIGGER_FALLING)                    /*!< External Interrupt Mode with Falling edge trigger detection         */
//#define  GPIO_MODE_IT_RISING_FALLING            (MODE_INPUT | EXTI_IT | TRIGGER_RISING | TRIGGER_FALLING)   /*!< External Interrupt Mode with Rising/Falling edge trigger detection  */
//
//#define  GPIO_MODE_EVT_RISING                   (MODE_INPUT | EXTI_EVT | TRIGGER_RISING)                     /*!< External Event Mode with Rising edge trigger detection             */
//#define  GPIO_MODE_EVT_FALLING                  (MODE_INPUT | EXTI_EVT | TRIGGER_FALLING)                    /*!< External Event Mode with Falling edge trigger detection            */
//#define  GPIO_MODE_EVT_RISING_FALLING           (MODE_INPUT | EXTI_EVT | TRIGGER_RISING | TRIGGER_FALLING)   /*!< External Event Mode with Rising/Falling edge trigger detection     */
//
//
//#define  HAL_GPIO_SPEED_FREQ_LOW         0x00000000U  /*!< IO works at 2 MHz, please refer to the product datasheet */
//#define  HAL_GPIO_SPEED_FREQ_MEDIUM      0x00000001U  /*!< range 12,5 MHz to 50 MHz, please refer to the product datasheet */
//#define  HAL_GPIO_SPEED_FREQ_HIGH        0x00000002U  /*!< range 25 MHz to 100 MHz, please refer to the product datasheet  */
//#define  HAL_GPIO_SPEED_FREQ_VERY_HIGH   0x00000003U  /*!< range 50 MHz to 200 MHz, please refer to the product datasheet  */
//
//#define  HAL_GPIO_NO_PULL        0x00000000U   /*!< No Pull-up or Pull-down activation  */
//#define  HAL_GPIO_PULL_UP        0x00000001U   /*!< Pull-up activation                  */
//#define  HAL_GPIO_PULL_DOWN      0x00000002U   /*!< Pull-down activation                */
//
///**
//  * @brief  Checks whether the specified EXTI line flag is set or not.
//  * @param  __EXTI_LINE__ specifies the EXTI line flag to check.
//  *         This parameter can be GPIO_PIN_x where x can be(0..15)
//  * @retval The new state of __EXTI_LINE__ (SET or RESET).
//  */
//#define __HAL_GPIO_EXTI_GET_FLAG(__EXTI_LINE__) (EXTI->PR & (__EXTI_LINE__))
//
///**
//  * @brief  Clears the EXTI's line pending flags.
//  * @param  __EXTI_LINE__ specifies the EXTI lines flags to clear.
//  *         This parameter can be any combination of GPIO_PIN_x where x can be (0..15)
//  * @retval None
//  */
//#define __HAL_GPIO_EXTI_CLEAR_FLAG(__EXTI_LINE__) (EXTI->PR = (__EXTI_LINE__))
//
///**
//  * @brief  Checks whether the specified EXTI line is asserted or not.
//  * @param  __EXTI_LINE__ specifies the EXTI line to check.
//  *          This parameter can be GPIO_PIN_x where x can be(0..15)
//  * @retval The new state of __EXTI_LINE__ (SET or RESET).
//  */
//#define __HAL_GPIO_EXTI_GET_IT(__EXTI_LINE__) (EXTI->PR & (__EXTI_LINE__))
//
///**
//  * @brief  Clears the EXTI's line pending bits.
//  * @param  __EXTI_LINE__ specifies the EXTI lines to clear.
//  *          This parameter can be any combination of GPIO_PIN_x where x can be (0..15)
//  * @retval None
//  */
//#define __HAL_GPIO_EXTI_CLEAR_IT(__EXTI_LINE__) (EXTI->PR = (__EXTI_LINE__))
//
///**
//  * @brief  Generates a Software interrupt on selected EXTI line.
//  * @param  __EXTI_LINE__ specifies the EXTI line to check.
//  *          This parameter can be GPIO_PIN_x where x can be(0..15)
//  * @retval None
//  */
//#define __HAL_GPIO_EXTI_GENERATE_SWIT(__EXTI_LINE__) (EXTI->SWIER |= (__EXTI_LINE__))
///**
//  * @}
//  */
//
///* Include GPIO HAL Extension module */
//#include "stm32f4xx_hal_gpio_ex.h"
//
///* Exported functions --------------------------------------------------------*/
///** @addtogroup GPIO_Exported_Functions
//  * @{
//  */
//
///** @addtogroup GPIO_Exported_Functions_Group1
//  * @{
//  */
///* Initialization and de-initialization functions *****************************/
//void  HAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init);
//void  HAL_GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin);
///**
//  * @}
//  */
//
///** @addtogroup GPIO_Exported_Functions_Group2
//  * @{
//  */
///* IO operation functions *****************************************************/
//GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
//void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
//void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
//HAL_StatusTypeDef HAL_GPIO_LockPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
//void HAL_GPIO_EXTI_IRQHandler(uint16_t GPIO_Pin);
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
//
///**
//  * @}
//  */
//
///**
//  * @}
//  */
///* Private types -------------------------------------------------------------*/
///* Private variables ---------------------------------------------------------*/
///* Private constants ---------------------------------------------------------*/
///** @defgroup GPIO_Private_Constants GPIO Private Constants
//  * @{
//  */
//#define GPIO_MODE_Pos                           0U
//#define GPIO_MODE                               (0x3UL << GPIO_MODE_Pos)
//#define MODE_INPUT                              (0x0UL << GPIO_MODE_Pos)
//#define MODE_OUTPUT                             (0x1UL << GPIO_MODE_Pos)
//#define MODE_AF                                 (0x2UL << GPIO_MODE_Pos)
//#define MODE_ANALOG                             (0x3UL << GPIO_MODE_Pos)
//#define OUTPUT_TYPE_Pos                         4U
//#define OUTPUT_TYPE                             (0x1UL << OUTPUT_TYPE_Pos)
//#define OUTPUT_PP                               (0x0UL << OUTPUT_TYPE_Pos)
//#define OUTPUT_OD                               (0x1UL << OUTPUT_TYPE_Pos)
//#define EXTI_MODE_Pos                           16U
//#define EXTI_MODE                               (0x3UL << EXTI_MODE_Pos)
//#define EXTI_IT                                 (0x1UL << EXTI_MODE_Pos)
//#define EXTI_EVT                                (0x2UL << EXTI_MODE_Pos)
//#define TRIGGER_MODE_Pos                         20U
//#define TRIGGER_MODE                            (0x7UL << TRIGGER_MODE_Pos)
//#define TRIGGER_RISING                          (0x1UL << TRIGGER_MODE_Pos)
//#define TRIGGER_FALLING                         (0x2UL << TRIGGER_MODE_Pos)
//
///**
//  * @}
//  */
//
///* Private macros ------------------------------------------------------------*/
///** @defgroup GPIO_Private_Macros GPIO Private Macros
//  * @{
//  */
//#define IS_GPIO_PIN_ACTION(ACTION) (((ACTION) == GPIO_PIN_RESET) || ((ACTION) == GPIO_PIN_SET))
//#define IS_GPIO_PIN(PIN)           (((((uint32_t)PIN) & GPIO_PIN_MASK ) != 0x00U) && ((((uint32_t)PIN) & ~GPIO_PIN_MASK) == 0x00U))
//#define IS_GPIO_MODE(MODE) (((MODE) == GPIO_MODE_INPUT)              ||\
//                            ((MODE) == GPIO_MODE_OUTPUT_PP)          ||\
//                            ((MODE) == GPIO_MODE_OUTPUT_OD)          ||\
//                            ((MODE) == GPIO_MODE_AF_PP)              ||\
//                            ((MODE) == GPIO_MODE_AF_OD)              ||\
//                            ((MODE) == GPIO_MODE_IT_RISING)          ||\
//                            ((MODE) == GPIO_MODE_IT_FALLING)         ||\
//                            ((MODE) == GPIO_MODE_IT_RISING_FALLING)  ||\
//                            ((MODE) == GPIO_MODE_EVT_RISING)         ||\
//                            ((MODE) == GPIO_MODE_EVT_FALLING)        ||\
//                            ((MODE) == GPIO_MODE_EVT_RISING_FALLING) ||\
//                            ((MODE) == GPIO_MODE_ANALOG))
//#define IS_GPIO_SPEED(SPEED) (((SPEED) == GPIO_SPEED_FREQ_LOW)  || ((SPEED) == GPIO_SPEED_FREQ_MEDIUM) || \
//                              ((SPEED) == GPIO_SPEED_FREQ_HIGH) || ((SPEED) == GPIO_SPEED_FREQ_VERY_HIGH))
//#define IS_GPIO_PULL(PULL) (((PULL) == GPIO_NOPULL) || ((PULL) == GPIO_PULLUP) || \
//                            ((PULL) == GPIO_PULLDOWN))
//
//
//
//
//
//#endif //MAIN_CONTROLLER_HAL_GPIO_H
