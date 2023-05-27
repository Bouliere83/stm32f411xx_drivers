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
