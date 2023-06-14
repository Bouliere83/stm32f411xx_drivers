/*
 * stm32f411xx.h
 *
 *  Created on: May 19, 2023
 *      Author: Rémi Boulière
 *
 *  Memory addresses for STM32F411xx microcontroller
 */

#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

#include "stdint.h"

/******************* General memory mapping ********************/

#define FLASH_BASEADDR			0x08000000U		// Base address for flash memory (512kb)
#define SRAM_BASEADDR  			0x20000000U		// Base address for SRAM Memory (128kb)
#define ROM_BASEADDR			0x1FFF0000U		// Base address for System memory (ROM)

/******************* Peripherical addresses ********************/

#define PERIPHERICAL_BASEADDR	0x40000000U		// Base address for peripherical registers

/* Bus */

#define APB1_BASEADDR 			PERIPHERICAL_BASEADDR
#define APB2_BASEADDR 			0x40007400U
#define AHB1_BASEADDR			0x40020000U
#define AHB2_BASEADDR			0x50000000U

/******************* RCC ********************/

#define RCC_BASEADDR			0x40023800

/* RCC Registers structure */
typedef struct
{
	volatile uint32_t CR;           /* RCC clock control register,                                  Address offset: 0x00    */
	volatile uint32_t PLLCFGR;      /* RCC PLL configuration register,                              Address offset: 0x04    */
	volatile uint32_t CFGR;         /* RCC clock configuration register,                            Address offset: 0x08    */
	volatile uint32_t CIR;          /* RCC clock interrupt register,                                Address offset: 0x0C    */
	volatile uint32_t AHB1RSTR;     /* RCC AHB1 peripheral reset register,                          Address offset: 0x10    */
	volatile uint32_t AHB2RSTR;     /* RCC AHB2 peripheral reset register,                          Address offset: 0x14    */
	volatile uint32_t AHB3RSTR;     /* RCC AHB3 peripheral reset register,                          Address offset: 0x18    */
	uint32_t      RESERVED0;    	/* RCC reserved register,                                       Address offset: 0x1C    */
	volatile uint32_t APB1RSTR;     /* RCC APB1 peripheral reset register,                          Address offset: 0x20 	*/
	volatile uint32_t APB2RSTR;     /* RCC APB2 peripheral reset register,                          Address offset: 0x24    */
	uint32_t      RESERVED1[2];		/* RCC reserved register,                                       Address offset: 0x28-2C */
	volatile uint32_t AHB1ENR;      /* RCC AHB1 peripheral clock enable register,                   Address offset: 0x30    */
	volatile uint32_t AHB2ENR;      /* RCC AHB2 peripheral clock enable register,                   Address offset: 0x34    */
	volatile uint32_t AHB3ENR;      /* RCC AHB3 peripheral clock enable register,                   Address offset: 0x38    */
	uint32_t      RESERVED2;    	/* RCC reserved register,                                       Address offset: 0x3C    */
	volatile uint32_t APB1ENR;      /* RCC APB1 peripheral clock enable register,                   Address offset: 0x40    */
	volatile uint32_t APB2ENR;      /* RCC APB2 peripheral clock enable register,                   Address offset: 0x44    */
	uint32_t      RESERVED3[2];		/* RCC reserved register,                                       Address offset: 0x48-4C */
	volatile uint32_t AHB1LPENR;    /* RCC AHB1 peripheral clock enable in low power mode register,	Address offset: 0x50    */
	volatile uint32_t AHB2LPENR;	/* RCC AHB2 peripheral clock enable in low power mode register,	Address offset: 0x54 	*/
	volatile uint32_t AHB3LPENR;	/* RCC AHB3 peripheral clock enable in low power mode register,	Address offset: 0x58 	*/
	uint32_t      RESERVED4;    	/* RCC reserved register,                                       Address offset: 0x5C    */
	volatile uint32_t APB1LPENR;    /* RCC APB1 peripheral clock enable in low power mode register,	Address offset: 0x60    */
	volatile uint32_t APB2LPENR;    /* RCC APB2 peripheral clock enable in low power mode register,	Address offset: 0x64    */
	uint32_t      RESERVED5[2]; 	/* RCC reserved register,                                       Address offset: 0x68-6C */
	volatile uint32_t BDCR;         /* RCC back up domain control register,                         Address offset: 0x70    */
	volatile uint32_t CSR;          /* RCC clock control & status register,                         Address offset: 0x74    */
	uint32_t      RESERVED6[2]; 	/* RCC reserved register,                                       Address offset: 0x78-7C */
	volatile uint32_t SSCGR;        /* RCC spread spectrum clock generation register,               Address offset: 0x80    */
	volatile uint32_t PLLI2SCFGR;   /* RCC PLLI2S configuration register,                           Address offset: 0x84    */
	volatile uint32_t PLLSAICFGR;   /* RCC PLL configuration register,                              Address offset: 0x88    */
	volatile uint32_t DCKCFGR;      /* RCC dedicated clock configuration register,                  Address offset: 0x8C    */
	volatile uint32_t CKGATENR;     /* RCC clock gated enable register,                             Address offset: 0x90    */
	volatile uint32_t DCKCFGR2;     /* RCC dedicated clock configuration register 2,                Address offset: 0x94    */
}RCC_RegDef_t;

#define RCC ((RCC_RegDef_t*)RCC_BASEADDR)

/* EXTI */

#define EXTI_BASEADDR			(APB2_BASEADDR + 0xC800)

/* System configuration */

#define SYSCFG_BASEADDR			(APB2_BASEADDR + 0xC400)

/******************* GPIO ********************/

/* Base addresses */
#define GPIOA_BASEADDR			(AHB1_BASEADDR + 0x0000)
#define GPIOB_BASEADDR			(AHB1_BASEADDR + 0x0400)
#define GPIOC_BASEADDR			(AHB1_BASEADDR + 0x0800)
#define GPIOD_BASEADDR			(AHB1_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR			(AHB1_BASEADDR + 0x1000)
#define GPIOH_BASEADDR			(AHB1_BASEADDR + 0x1C00)

/* Structure used to access all GPIO Registers */
typedef struct
{
	volatile uint32_t MODER;        /* GPIO port mode register,                         Address offset: 0x00    */
	volatile uint32_t OTYPER;       /* GPIO port output type register,                  Address offset: 0x04    */
	volatile uint32_t OSPEEDR;      /* GPIO port output speed register,                 Address offset: 0x08    */
	volatile uint32_t PUPDR;        /* GPIO port pull-up/down register,                 Address offset: 0x0C    */
	volatile uint32_t IDR;          /* GPIO port input data register,                   Address offset: 0x10    */
	volatile uint32_t ODR;          /* GPIO port output data register,                  Address offset: 0x14    */
	volatile uint32_t BSRR;         /* GPIO port bit set/reset register,                Address offset: 0x18    */
	volatile uint32_t LCKR;         /* GPIO port configuration lock register,           Address offset: 0x1C    */
	volatile uint32_t AFR[2];       /* AFR[0]: GPIO alternate function low register,
                              AFR[1]: GPIO alternate function high register,   Address offset: 0x20-24 */
} GPIO_RegDef_t;


#define GPIOA	((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 	((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 	((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD	((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE	((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH 	((GPIO_RegDef_t*)GPIOH_BASEADDR)

/*
 * Clock enable macros for GPIO peripherals
 */
#define GPIOA_PCLK_EN()	(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()	(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()	(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()	(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()	(RCC->AHB1ENR |= (1 << 4))
#define GPIOH_PCLK_EN()	(RCC->AHB1ENR |= (1 << 7))

/*
 * Clock disable macros for GPIO peripherals
 */
#define GPIOA_PCLK_DI()	(RCC->AHB1RSTR &= ~(1 << 0))
#define GPIOB_PCLK_DI()	(RCC->AHB1RSTR &= ~(1 << 1))
#define GPIOC_PCLK_DI()	(RCC->AHB1RSTR &= ~(1 << 2))
#define GPIOD_PCLK_DI()	(RCC->AHB1RSTR &= ~(1 << 3))
#define GPIOE_PCLK_DI()	(RCC->AHB1RSTR &= ~(1 << 4))
#define GPIOH_PCLK_DI()	(RCC->AHB1RSTR &= ~(1 << 7))

/* Reset GPIO Peripheral */

#define GPIOA_REG_RESET()	do { (RCC->AHB1ENR |= (1 << 0)); (RCC->AHB1ENR &= ~(1 << 0));}while(0)
#define GPIOB_REG_RESET()	do { (RCC->AHB1ENR |= (1 << 1)); (RCC->AHB1ENR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RESET()	do { (RCC->AHB1ENR |= (1 << 2)); (RCC->AHB1ENR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET()	do { (RCC->AHB1ENR |= (1 << 3)); (RCC->AHB1ENR &= ~(1 << 3));}while(0)
#define GPIOE_REG_RESET()	do { (RCC->AHB1ENR |= (1 << 4)); (RCC->AHB1ENR &= ~(1 << 4));}while(0)
#define GPIOH_REG_RESET()	do { (RCC->AHB1ENR |= (1 << 7)); (RCC->AHB1ENR &= ~(1 << 7));}while(0)

/******************* SPI ********************/

#define SPI1_BASEADDR			(APB2_BASEADDR + 0xBC00)
#define SPI2_BASEADDR			(APB1_BASEADDR + 0x3800)
#define SPI3_BASEADDR			(APB1_BASEADDR + 0x3C00)
#define SPI4_BASEADDR			(APB2_BASEADDR + 0xC000)
#define SPI5_BASEADDR			(APB2_BASEADDR + 0xDC00)

/*
 * Peripheral register definition structure for SPI
 */
typedef struct
{
	volatile uint32_t CR1;      /* SPI control register 1,          Address offset: 0x00 */
	volatile uint32_t CR2;      /* SPI control register 2,          Address offset: 0x04 */
	volatile uint32_t SR;       /* SPI status register,             Address offset: 0x08 */
	volatile uint32_t DR;       /* SPI data register,               Address offset: 0x0C */
	volatile uint32_t CRCPR;    /* SPI CRC polynomial register,     Address offset: 0x10 */
	volatile uint32_t RXCRCR;   /* SPI RX CRC register,             Address offset: 0x14 */
	volatile uint32_t TXCRCR;   /* SPI TX CRC register,             Address offset: 0x18 */
	volatile uint32_t I2SCFGR;  /* SPI_I2S configuration register,  Address offset: 0x1C */
	volatile uint32_t I2SPR;    /* SPI_I2S prescaler register,      Address offset: 0x20 */
}SPI_RegDef_t;

#define SPI1	((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2	((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3	((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4	((SPI_RegDef_t*)SPI4_BASEADDR)
#define SPI5	((SPI_RegDef_t*)SPI5_BASEADDR)

/*
 * Clock enable macros for SPI peripherals
 */
#define SPI1_PCLK_EN()	(RCC->APB2ENR 	|= (1 << 12))
#define SPI2_PCLK_EN()	(RCC->APB1ENR 	|= (1 << 14))
#define SPI3_PCLK_EN()	(RCC->APB1ENR 	|= (1 << 15))
#define SPI4_PCLK_EN()	(RCC->APB2ENR 	|= (1 << 13))
#define SPI5_PCLK_EN()	(RCC->APB2ENR  	|= (1 << 20))

/*
 * Clock disable macros for SPI peripherals
 */
#define SPI1_PCLK_DI()	(RCC->APB2RSTR  &= ~(1 << 12))
#define SPI2_PCLK_DI()	(RCC->APB1RSTR 	&= ~(1 << 14))
#define SPI3_PCLK_DI()	(RCC->APB1RSTR 	&= ~(1 << 15))
#define SPI4_PCLK_DI()	(RCC->APB2RSTR  &= ~(1 << 13))
#define SPI5_PCLK_DI()	(RCC->APB2RSTR  &= ~(1 << 20))

/******************* I2C ********************/

#define I2C1_BASEADDR			(APB1_BASEADDR + 0x5400)
#define I2C2_BASEADDR			(APB1_BASEADDR + 0x5800)
#define I2C3_BASEADDR			(APB1_BASEADDR + 0x5C00)

/*
 * Peripheral register definition structure for I2C
 */
typedef struct
{
    volatile uint32_t CR1;      /* I2C control register 1,      Address offset: 0x00 */
    volatile uint32_t CR2;      /* I2C control register 2,      Address offset: 0x04 */
    volatile uint32_t OAR1;     /* I2C own address register 1,  Address offset: 0x08 */
    volatile uint32_t OAR2;     /* I2C own address register 2,  Address offset: 0x0C */
    volatile uint32_t DR;       /* I2C data register,           Address offset: 0x10 */
    volatile uint32_t SR1;      /* I2C status register 1,       Address offset: 0x14 */
    volatile uint32_t SR2;      /* I2C status register 2,       Address offset: 0x18 */
    volatile uint32_t CCR;      /* I2C clock control register,  Address offset: 0x1C */
    volatile uint32_t TRISE;    /* I2C TRISE register,          Address offset: 0x20 */
    volatile uint32_t FLTR;     /* I2C FLTR register,           Address offset: 0x24 */
}I2C_RegDef_t;

/*
 * Clock enable macros for I2C peripherals
 */
#define I2C1_PCLK_EN()	(RCC->APB1ENR 	|= (1 << 21))
#define I2C2_PCLK_EN()	(RCC->APB1ENR 	|= (1 << 22))
#define I2C3_PCLK_EN()	(RCC->APB1ENR 	|= (1 << 23))

/*
 * Clock disable macros for I2C peripherals
 */
#define I2C1_PCLK_DI()	(RCC->APB1RSTR  &= ~(1 << 21))
#define I2C2_PCLK_DI()	(RCC->APB1RSTR 	&= ~(1 << 22))
#define I2C3_PCLK_DI()	(RCC->APB1RSTR 	&= ~(1 << 23))

/******************* USART ********************/

#define USART1_BASEADDR			(APB2_BASEADDR + 0x9C00)
#define USART2_BASEADDR			(APB1_BASEADDR + 0x4400)
#define USART6_BASEADDR			(APB2_BASEADDR + 0xA000)

/*
 * Peripheral register definition structure for USART
 */
typedef struct
{
    volatile uint32_t SR;   /* USART status register,                   Address offset: 0x00 */
    volatile uint32_t DR;   /* USART data register,                     Address offset: 0x04 */
    volatile uint32_t BRR;  /* USART baud rate register,                Address offset: 0x08 */
    volatile uint32_t CR1;  /* USART control register 1,                Address offset: 0x0C */
    volatile uint32_t CR2;  /* USART control register 2,                Address offset: 0x10 */
    volatile uint32_t CR3;  /* USART control register 3,                Address offset: 0x14 */
    volatile uint32_t GTPR; /* USART guard time and prescaler register, Address offset: 0x18 */
}USART_RegDef_t;

/*
 * Clock enable macros for I2C peripherals
 */
#define USART1_PCLK_EN()	(RCC->APB2ENR 	|= (1 << 4))
#define USART2_PCLK_EN()	(RCC->APB1ENR 	|= (1 << 17))
#define USART6_PCLK_EN()	(RCC->APB2ENR 	|= (1 << 5))

/*
 * Clock disable macros for I2C peripherals
 */
#define USART1_PCLK_DI()	(RCC->APB2RSTR  &= ~(1 << 4))
#define USART2_PCLK_DI()	(RCC->APB1RSTR 	&= ~(1 << 17))
#define USART6_PCLK_DI()	(RCC->APB2RSTR 	&= ~(1 << 5))

/*
 * General Macros
 */
#define ENABLE 			1
#define DISABLE			0
#define SET  			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
#endif /* INC_STM32F411XX_H_ */
