/*
 * stm32f411xx_gpio_driver.h
 *
 *  Created on: May 25, 2023
 *      Author: Rémi Boulière
 */

#ifndef INC_STM32F411XX_GPIO_DRIVER_H_
#define INC_STM32F411XX_GPIO_DRIVER_H_

#include "stm32f411xx.h"

/* GPIO Pin configuration structure */
typedef struct
{
    uint8_t GPIO_PinNumber;         /* Possible values from GPIO_PIN_NUMBERS                           */
    uint8_t GPIO_PinMode;           /* Possible values from GPIO_PIN_MODES                             */
    uint8_t GPIO_PinSpeed;          /* Possible values from GPIO_PIN_SPEED                             */
    uint8_t GPIO_PinPuPdControl;    /* Possible values from GPIO_PIN_PULL_UP_PULL_DOWN                 */
    uint8_t GPIO_PinOPType;         /* Possible values from GPIO_PIN_OUTPUT_TYPE                       */
    uint8_t GPIO_PinAltFunMode;     /* Has value only when GPIO_PIN_MODES is set to Alt functionality  */
}GPIO_PinConfig_t;

/* Structure for handling a GPIO pin */

typedef struct {
	GPIO_RegDef_t *pGPIOx; /* Hold the base address of the GPIO Port to which the pin belong */
	GPIO_PinConfig_t GPIO_PinCOnfig; /* Gold the GPIO Pin Configuration settings */
}GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0       0
#define GPIO_PIN_NO_1       1
#define GPIO_PIN_NO_2       2
#define GPIO_PIN_NO_3       3
#define GPIO_PIN_NO_4       4
#define GPIO_PIN_NO_5       5
#define GPIO_PIN_NO_6       6
#define GPIO_PIN_NO_7       7
#define GPIO_PIN_NO_8       8
#define GPIO_PIN_NO_9       9
#define GPIO_PIN_NO_10      10
#define GPIO_PIN_NO_11      11
#define GPIO_PIN_NO_12      12
#define GPIO_PIN_NO_13      13
#define GPIO_PIN_NO_14      14
#define GPIO_PIN_NO_15      15


/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN        0 /* GPIO Input mode                          */
#define GPIO_MODE_OUT       1 /* GPIO Output mode                         */
#define GPIO_MODE_ALTFN     2 /* GPIO Alternate functionality             */
#define GPIO_MODE_ANALOG    3 /* GPIO Analog mode                         */
#define GPIO_MODE_IT_FT     4 /* GPIO Input mode falling edge trigger     */
#define GPIO_MODE_IT_RT     5 /* GPIO Input mode rising edge trigger      */
#define GPIO_MODE_IT_FRT    6 /* GPIO Input mode fall-rising edge trigger */


/*
 * @GPIO_PIN_OUTPUT_TYPE
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP     0 /* GPIO Output type push-pull mode  */
#define GPIO_OP_TYPE_OD     1 /* GPIO Output type open-drain mode */


/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW      0
#define GPIO_SPEED_MEDIUM   1
#define GPIO_SPEED_FAST     2
#define GPIO_SPEED_HIGH     3


/*
 * @GPIO_PIN_PULL_UP_PULL_DOWN
 * GPIO pin pull-up and pull-down configuration macros
 */
#define GPIO_PIN_NO_PUPD    0 /* GPIO configuration no pull-up, pull-down */
#define GPIO_PIN_PU         1 /* GPIO configuration pull-up               */
#define GPIO_PIN_PD         2 /* GPIO configuration pull-down             */

/*
 * Peripherical Clock Setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);

/*
 * Init and De-Init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Read/Write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F411XX_GPIO_DRIVER_H_ */
