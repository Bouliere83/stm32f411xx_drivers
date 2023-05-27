/*
 * stm32f411xx_gpio_driver.c
 *
 *  Created on: May 25, 2023
 *      Author: Rémi Boulière
 */


#include "stm32f411xx_gpio_driver.h"


/*
 * Peripherical Clock Setup
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
{

}

/*
 * Init and De-Init
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{

}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

}

/*
 * Read/Write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{

}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{

}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{

}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

}


/*
 * IRQ Handling
 */

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi)
{

}

void GPIO_IRQHandling(uint8_t PinNumber)
{

}
