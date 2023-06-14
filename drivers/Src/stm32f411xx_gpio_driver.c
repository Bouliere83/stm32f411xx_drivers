/*
 * stm32f411xx_gpio_driver.c
 *
 *  Created on: May 25, 2023
 *      Author: Rémi Boulière
 */


#include "stm32f411xx_gpio_driver.h"



/*****************************************************************
 * @fn          - GPIO_PeriClockControl
 *
 * @brief       - This function enables or disables peripheral
 *                clock for the given GPIO port
 *
 * @param[in]   - Base address of the GPIO peripheral
 * @param[in]   - Macros: Enable or Disable
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		}
	}
	else {
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOH) {
			GPIOH_PCLK_DI();
		}
	}
}


/*****************************************************************
 * @fn          - GPIO_Init
 *
 * @brief       - This function initialize GPIO peripherals
 *
 * @param[in]   - Pointer to GPIO Handle structure
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;

	/* Mode */
	if(pGPIOHandle->GPIO_PinCOnfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = pGPIOHandle->GPIO_PinCOnfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinCOnfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER &= ~(3 << pGPIOHandle->GPIO_PinCOnfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;
		temp = 0;
	}
	else {
		//Interrupt mode
	}

	/* Speed */
	temp = pGPIOHandle->GPIO_PinCOnfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinCOnfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(3 << pGPIOHandle->GPIO_PinCOnfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	/* pupd */
	temp = pGPIOHandle->GPIO_PinCOnfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinCOnfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(3 << pGPIOHandle->GPIO_PinCOnfig.GPIO_PinNumber);

	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	/* otype */
	temp = pGPIOHandle->GPIO_PinCOnfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinCOnfig.GPIO_PinNumber;
	pGPIOHandle->pGPIOx->PUPDR &= ~(1 << pGPIOHandle->GPIO_PinCOnfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	/* Alternate functions */
	if(pGPIOHandle->GPIO_PinCOnfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint32_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinCOnfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinCOnfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << pGPIOHandle->GPIO_PinCOnfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinCOnfig.GPIO_PinAltFunMode << (4 * temp2));
	}
}

/*****************************************************************
 * @fn          - GPIO_DeInit
 *
 * @brief       - This function de-initialize GPIO peripherals
 *
 * @param[in]   - Base address of the GPIO peripheral
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB) {
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC) {
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD) {
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE) {
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOH) {
		GPIOH_REG_RESET();
	}
}

/*****************************************************************
 * @fn          - GPIO_ReadFromInputPin
 *
 * @brief       - This function reads value of input pin, on
 *                a specific port
 *
 * @param[in]   - Base address of the GPIO peripheral
 * @param[in]   - Pin number
 *
 * @return      - Content of the input data
 *
 * @Note        - 0 or 1
 *
 *****************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

}

/*****************************************************************
 * @fn          - GPIO_ReadFromInputPort
 *
 * @brief       - This function reads value of input port
 *
 * @param[in]   - Base address of the GPIO peripheral
 *
 * @return      - Content of the input data
 *
 * @Note        - None
 *
 *****************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{

}


/*****************************************************************
 * @fn          - GPIO_WriteToOutputPin
 *
 * @brief       - This function writes value on a specific
 *                output pin
 *
 * @param[in]   - Base address of the GPIO peripheral
 * @param[in]   - Pin number
 * @param[in]   - Value (Set/Reset Macro)
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{

}

/*****************************************************************
 * @fn          - GPIO_WriteToOutputPort
 *
 * @brief       - This function writes value on a specific
 *                output port
 *
 * @param[in]   - Base address of the GPIO peripheral
 * @param[in]   - Value (Set/Reset Macro)
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{

}

/*****************************************************************
 * @fn          - GPIO_ToggleOutputPin
 *
 * @brief       - This function toggles specific output pin
 *
 * @param[in]   - Base address of the GPIO peripheral
 * @param[in]   - Pin number
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

}


/*****************************************************************
 * @fn          - GPIO_IRQConfig
 *
 * @brief       - This function configures interrupt
 *
 * @param[in]   - IRQ Interrupt number
 * @param[in]   - Macro: Enable/Disable
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi)
{

}

/*****************************************************************
 * @fn          - GPIO_IRQPriorityConfig
 *
 * @brief       - This function configures interrupt priority
 *
 * @param[in]   - IRQ Interrupt number
 * @param[in]   - IRQ interrupt priority
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber)
{

}
