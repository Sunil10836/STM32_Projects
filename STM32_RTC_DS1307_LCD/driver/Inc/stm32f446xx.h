/*
 * stm32f446xx.h
 *
 *  Created on: Jul 20, 2025
 *      Author: Sunil Sutar
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stdint.h>

#define __vo 	volatile

/************************ START: Processor Specific Details ****************************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register addresses
 */
#define NVIC_ISER0			 ((__vo uint32_t *) 0xE000E100 )
#define NVIC_ISER1			 ((__vo uint32_t *) 0xE000E104 )
#define NVIC_ISER2			 ((__vo uint32_t *) 0xE000E108 )
#define NVIC_ISER3			 ((__vo uint32_t *) 0xE000E10C )

/*
 * ARM Cortex Mx Processor NVIC ISERx register addresses
 */
#define NVIC_ICER0			 ((__vo uint32_t *) 0XE000E180 )
#define NVIC_ICER1			 ((__vo uint32_t *) 0XE000E184 )
#define NVIC_ICER2			 ((__vo uint32_t *) 0XE000E188 )
#define NVIC_ICER3			 ((__vo uint32_t *) 0XE000E18C )

/*
 * ARM Cortex Mx Processor NVIC Priority register addresses
 */
#define NVIC_PR_BASEADDR			 ((__vo uint32_t *) 0xE000E400 )

#define NO_PR_BITS_IMPLEMENTED		 4

/************************ END : Processor Specific Details ****************************************/

/*
 * Base addresses of SRAM and Flash memories
 */

#define FLASH_BASEADDR		0x08000000U
#define SRAM1_BASEADDR		0x20000000U		//112KB
#define SRAM2_BASEADDR		0x2001C000U
#define ROM_BASEADDR		0x1FFF0000U

#define SRAM 				SRAM1_BASEADDR

/*
 * Base addresses of Bus Domains AHBx and APBx
 */

#define PERIPH_BASEADDR			0x40000000U
#define APB1PERIPH_BASEADDR		PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR		0x40010000U
#define AHB1PERIPH_BASEADDR		0x40020000U
#define AHB2PERIPH_BASEADDR		0x50000000U

/*
 * Base addresses of AHB1 Peripherals : GPIOA to GPIOH
 */
#define GPIOA_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1C00)

/*
 * Base addresses of APB1 Peripheral : I2C1, I2C2, I2C3, SPI2, SPI3, USART2, USART3, UART4, UART5
 *
 */
/* Base addresses of I2Cx Peripherals */
#define I2C1_BASEADDR			(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASEADDR + 0x5C00)

/* Base addresses of SPIx Peripherals */
#define SPI2_BASEADDR			(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASEADDR + 0x3C00)

/* Base addresses of USARTx Peripherals */
#define USART2_BASEADDR			(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR			(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR			(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR			(APB1PERIPH_BASEADDR + 0x5000)

/*
 * Base addresses of APB2 Peripheral : SPI1, SPI4 USART1, USART6, EXTI, SYSCFG
 *
 */
#define SPI1_BASEADDR			(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR			(APB2PERIPH_BASEADDR + 0x3400)

#define USART1_BASEADDR			(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR			(APB2PERIPH_BASEADDR + 0x1400)

#define SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR + 0x3800)
#define EXTI_BASEADDR			(APB2PERIPH_BASEADDR + 0x3C00)

#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3800)


/****************** Peripheral Register Definition Structures *********************/
/*
 * Peripheral Register Definition structure for GPIO
 */
typedef struct
{
	__vo uint32_t MODER;		/*!< GPIO Mode Register, Address Offset = 0x00 */
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSSRL;
	__vo uint32_t BSSRH;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];
}GPIO_RegDef_t;

/*
 * Peripheral Register Definition structure for GPIO
 */
typedef struct
{
	__vo uint32_t CR;			/*!<    Address Offset = 0x00 */
	__vo uint32_t PLLCFGR;		/*!<    Address Offset = 0x04 */
	__vo uint32_t CFGR;			/*!<    Address Offset = 0x08 */
	__vo uint32_t CIR;			/*!<    Address Offset = 0x0C */
	__vo uint32_t AHB1RSTR;		/*!<    Address Offset = 0x10 */
	__vo uint32_t AHB2RSTR;		/*!<    Address Offset = 0x14 */
	__vo uint32_t AHB3RSTR;		/*!<    Address Offset = 0x18 */
	__vo uint32_t RESERVED1;	/*!<    Address Offset = 0x1C */
	__vo uint32_t APB1RSTR;		/*!<    Address Offset = 0x20 */
	__vo uint32_t APB2RSTR;		/*!<    Address Offset = 0x24 */
	__vo uint32_t RESERVED2[2];	/*!<    Address Offset = 0x28 and 0x2C */
	__vo uint32_t AHB1ENR;		/*!<    Address Offset = 0x30 */
	__vo uint32_t AHB2ENR;		/*!<    Address Offset = 0x34 */
	__vo uint32_t AHB3ENR;		/*!<    Address Offset = 0x38 */
	__vo uint32_t RESERVED3;	/*!<    Address Offset = 0x3C */
	__vo uint32_t APB1ENR;		/*!<    Address Offset = 0x40 */
	__vo uint32_t APB2ENR;		/*!<    Address Offset = 0x44 */
	__vo uint32_t RESERVED4[2];	/*!<    Address Offset = 0x48 and 0x4C */
	__vo uint32_t AHB1LPENR;	/*!<    Address Offset = 0x50 */
	__vo uint32_t AHB2LPENR;	/*!<    Address Offset = 0x54 */
	__vo uint32_t AHB3LPENR;	/*!<    Address Offset = 0x58 */
	__vo uint32_t RESERVED5;	/*!<    Address Offset = 0x5C */
	__vo uint32_t APB1LPENR;	/*!<    Address Offset = 0x60 */
	__vo uint32_t APB2LPENR;	/*!<    Address Offset = 0x64 */
	__vo uint32_t RESERVED6[2];	/*!<    Address Offset = 0x68 and 06C */
	__vo uint32_t BDCR;			/*!<    Address Offset = 0x70 */
	__vo uint32_t CSR;			/*!<    Address Offset = 0x74 */
	__vo uint32_t RESERVED7[2];	/*!<    Address Offset = 0x78 and 07C */
	__vo uint32_t SSCGR;		/*!<    Address Offset = 0x80 */
	__vo uint32_t PLLI2SCFGR;	/*!<    Address Offset = 0x84 */
	__vo uint32_t PLLSAISCFGR;	/*!<    Address Offset = 0x88 */
	__vo uint32_t DCKCFGR;		/*!<    Address Offset = 0x8C */
	__vo uint32_t CKGATENR;		/*!<    Address Offset = 0x90 */
	__vo uint32_t DCKCFGR2;		/*!<    Address Offset = 0x94 */

}RCC_RegDef_t;

/*
 * Peripheral Register Definition structure for EXTI
 */
typedef struct
{
	__vo uint32_t IMR;		/*!< Interrupt mask register (EXTI_IMR), 			 Address Offset = 0x00 */
	__vo uint32_t EMR;		/*!< Event mask register (EXTI_EMR), 				 Address Offset = 0x04 */
	__vo uint32_t RTSR;		/*!< Rising trigger selection register (EXTI_RTSR),  Address Offset = 0x08 */
	__vo uint32_t FTSR;		/*!< Falling trigger selection register (EXTI_FTSR), Address Offset = 0x0C */
	__vo uint32_t SWIER;	/*!< Software interrupt event register (EXTI_SWIER), Address Offset = 0x10 */
	__vo uint32_t PR;		/*!< Pending register (EXTI_PR),                     Address Offset = 0x14 */

}EXTI_RegDef_t;

/*
 * Peripheral Register Definition structure for SYSCFG
 */
typedef struct
{
	__vo uint32_t MEMRMP;			/*!<  SYSCFG memory remap register ,			      Address Offset = 0x00 */
	__vo uint32_t PMC;				/*!< SYSCFG peripheral mode configuration register ,  Address Offset = 0x04 */
	__vo uint32_t EXTICR[4];		/*!< SYSCFG external interrupt configuration register1 to 4, Address Offset = 0x08 to 0x14 */
		 uint32_t RESERVED1[2];		/*!< RESERVED				                      	  Address Offset = 0x18 to 0x1C */
	__vo uint32_t CMPCR;			/*!< Compensation cell control register,          	  Address Offset = 0x20 */
		 uint32_t RESERVED2[2];		/*!< RESERVED				                      	  Address Offset = 0x24 to 0x28 */
	__vo uint32_t CFGR;				/*!< SYSCFG configuration register,               	  Address Offset = 0x2C */
}SYSCFG_RegDef_t;

/*
 * Peripheral Register Definition structure for SPI
 */
typedef struct
{
	__vo uint32_t CR1;				/*!< TODO, Address Offset = 0x00 */
	__vo uint32_t CR2;				/*!< TODO, Address Offset = 0x04 */
	__vo uint32_t SR;				/*!< TODO, Address Offset = 0x08 */
	__vo uint32_t DR;				/*!< TODO, Address Offset = 0x0C */
	__vo uint32_t CRCPR;			/*!< TODO, Address Offset = 0x10 */
	__vo uint32_t RXCRCR;			/*!< TODO, Address Offset = 0x14 */
	__vo uint32_t TXCRCR;			/*!< TODO, Address Offset = 0x18 */
	__vo uint32_t I2SCFGR;			/*!< TODO, Address Offset = 0x1C */
	__vo uint32_t I2SPR;			/*!< TODO, Address Offset = 0x20 */
}SPI_RegDef_t;


/*
 * Peripheral Register Definition structure for USART
 */
typedef struct
{
	__vo uint32_t SR;				/*!< TODO, Address Offset = 0x00 */
	__vo uint32_t DR;				/*!< TODO, Address Offset = 0x04 */
	__vo uint32_t BRR;				/*!< TODO, Address Offset = 0x08 */
	__vo uint32_t CR1;				/*!< TODO, Address Offset = 0x0C */
	__vo uint32_t CR2;				/*!< TODO, Address Offset = 0x10 */
	__vo uint32_t CR3;				/*!< TODO, Address Offset = 0x14 */
	__vo uint32_t GTPR;				/*!< TODO, Address Offset = 0x18 */
}USART_RegDef_t;

/*
 * Peripheral Register Definition structure for I2C
 */

typedef struct
{
	__vo uint32_t CR1;				/*!< TODO, Address Offset = 0x00 */
	__vo uint32_t CR2;				/*!< TODO, Address Offset = 0x04 */
	__vo uint32_t OAR1;				/*!< TODO, Address Offset = 0x08 */
	__vo uint32_t OAR2;				/*!< TODO, Address Offset = 0x0C */
	__vo uint32_t DR;				/*!< TODO, Address Offset = 0x10 */
	__vo uint32_t SR1;				/*!< TODO, Address Offset = 0x14 */
	__vo uint32_t SR2;				/*!< TODO, Address Offset = 0x18 */
	__vo uint32_t CCR;				/*!< TODO, Address Offset = 0x1C */
	__vo uint32_t TRISE;			/*!< TODO, Address Offset = 0x20 */
	__vo uint32_t FLTR;				/*!< TODO, Address Offset = 0x24 */

}I2C_RegDef_t;


/*
 * Peripheral Definition (Peripheral Base addresses type-casted to xxx_RegDef_t
 */
#define GPIOA		( (GPIO_RegDef_t*) GPIOA_BASEADDR )
#define GPIOB		( (GPIO_RegDef_t*) GPIOB_BASEADDR )
#define GPIOC		( (GPIO_RegDef_t*) GPIOC_BASEADDR )
#define GPIOD		( (GPIO_RegDef_t*) GPIOD_BASEADDR )
#define GPIOE		( (GPIO_RegDef_t*) GPIOE_BASEADDR )
#define GPIOF		( (GPIO_RegDef_t*) GPIOF_BASEADDR )
#define GPIOG		( (GPIO_RegDef_t*) GPIOG_BASEADDR )
#define GPIOH		( (GPIO_RegDef_t*) GPIOH_BASEADDR )

#define RCC			( (RCC_RegDef_t*) RCC_BASEADDR )

#define EXTI		( (EXTI_RegDef_t*) EXTI_BASEADDR )

#define SYSCFG		( (SYSCFG_RegDef_t*) SYSCFG_BASEADDR )

#define SPI1		( (SPI_RegDef_t*) SPI1_BASEADDR )
#define SPI2		( (SPI_RegDef_t*) SPI2_BASEADDR )
#define SPI3		( (SPI_RegDef_t*) SPI3_BASEADDR )
#define SPI4		( (SPI_RegDef_t*) SPI4_BASEADDR )

#define USART1		( (USART_RegDef_t*) USART1_BASEADDR )
#define USART2		( (USART_RegDef_t*) USART2_BASEADDR )
#define USART3		( (USART_RegDef_t*) USART3_BASEADDR )
#define UART4		( (USART_RegDef_t*) UART4_BASEADDR  )
#define UART5		( (USART_RegDef_t*) UART5_BASEADDR  )
#define USART6		( (USART_RegDef_t*) USART6_BASEADDR )

#define I2C1		( (I2C_RegDef_t*) I2C1_BASEADDR )
#define I2C2		( (I2C_RegDef_t*) I2C2_BASEADDR )
#define I2C3		( (I2C_RegDef_t*) I2C3_BASEADDR )

/************************** Clock Enable Macros ***********************/
/*
 * Clock Enable Macros for GPIOx Peripherals
 */
#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))

/*
 * Clock Enable Macros for I2Cx Peripherals
 */
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1 << 23))

/*
 * Clock Enable Macros for SPIx Peripherals
 */
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()		(RCC->APB2ENR |= (1 << 13))

/*
 * Clock Enable Macros for USARTx Peripherals
 */
#define USART1_PCLK_EN()		(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()		(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()		(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()			(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()			(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()		(RCC->APB2ENR |= (1 << 5))

/*
 * Clock Enable Macros for SYSCFG Peripherals
 */
#define SYSCFG_PCLK_EN()		(RCC->APB2ENR |= (1 << 14))

/************************** Clock Disable Macros ***********************/
/*
 * Clock Disable Macros for GPIOx Peripherals
 */
#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 7))

/*
 * Clock Disable Macros for I2Cx Peripherals
 */
#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 23))

/*
 * Clock Disable Macros for SPIx Peripherals
 */
#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 13))

/*
 * Clock Disable Macros for USARTx Peripherals
 */
#define USART1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 5))

/*
 * Clock Disable Macros for SYSCFG Peripherals
 */
#define SYSCFG_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 14))

/*
 * Macros to Reset GPIOx Peripheral
 */
#define GPIOA_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 0)); (RCC->APB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 1)); (RCC->APB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 2)); (RCC->APB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 3)); (RCC->APB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 4)); (RCC->APB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 5)); (RCC->APB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 6)); (RCC->APB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 7)); (RCC->APB1RSTR &= ~(1 << 7)); }while(0)


/*
 * This macro returns a code (Between 0 to 7) for a given  GPIO base address(x)
 */
#define GPIO_BASEADDR_TO_CODE(x)   ((x == GPIOA)? 0:\
									(x == GPIOB)? 1:\
									(x == GPIOC)? 2:\
									(x == GPIOD)? 3:\
									(x == GPIOE)? 4:\
									(x == GPIOF)? 5:\
									(x == GPIOG)? 6:\
									(x == GPIOH)? 7:0)

/*
 * IRQ Number for EXTI Lines
 */
#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40

/*
 * IRQ Priority levels
 */
#define NVIC_IRQ_PRIO0			0
//TODO
#define NVIC_IRQ_PRIO15			15

//Some Generic Macros
#define ENABLE			1
#define DISABLE			0

#define SET 			ENABLE
#define RESET 			DISABLE

#define GPIO_PIN_SET	ENABLE
#define GPIO_PIN_RESET	DISABLE

#define FLAG_SET		SET
#define FLAG_RESET		RESET

/*****************************************************************************
* Bit Position Definitions of SPI Peripheral
*****************************************************************************/
/*
 *  Bit Position Definitions of SPI_CR1
 */
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

/*
 *  Bit Position Definitions of SPI_CR2
 */
#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7

/*
 *  Bit Position Definitions of SPI_SR
 */
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8

/*****************************************************************************
* Bit Position Definitions of USART Peripheral
*****************************************************************************/
/*
 *  Bit Position Definitions of USART_CR1
 */
#define USART_CR1_SBK		0
#define USART_CR1_RWU		1
#define USART_CR1_RE		2
#define USART_CR1_TE		3
#define USART_CR1_IDLEIE	4
#define USART_CR1_RXNEIE	5
#define USART_CR1_TCIE		6
#define USART_CR1_TXEIE		7
#define USART_CR1_PEIE		8
#define USART_CR1_PS		9
#define USART_CR1_PCE		10
#define USART_CR1_WAKE		11
#define USART_CR1_M			12
#define USART_CR1_UE		13
#define USART_CR1_OVER8		15

/*
 *  Bit Position Definitions of USART_CR2
 */
#define USART_CR2_STOP		12

/*
 *  Bit Position Definitions of USART_CR3
 */
#define USART_CR3_RTSE		8
#define USART_CR3_CTSE		9

/*
 *  Bit Position Definitions of USART_SR
 */
#define USART_SR_PE			0
#define USART_SR_FE			1
#define USART_SR_NF			2
#define USART_SR_ORE		3
#define USART_SR_IDLE		4
#define USART_SR_RXNE		5
#define USART_SR_TC			6
#define USART_SR_TXE		7
#define USART_SR_LBD		8
#define USART_SR_CTS		9

/*****************************************************************************
* Bit Position Definitions of I2C Peripheral
*****************************************************************************/
//CR1, CR2, SR1, SR2 and CCR
/*
 *  Bit Position Definitions of I2C_CR1
 */
#define I2C_CR1_PE			0
#define I2C_CR1_START		8
#define I2C_CR1_STOP		9
#define I2C_CR1_ACK			10

/*
 *  Bit Position Definitions of I2C_CR2
 */
#define I2C_CR2_FREQ		0

/*
 *  Bit Position Definitions of I2C_SR1
 */
#define I2C_SR1_SB			0
#define I2C_SR1_ADDR		1
#define I2C_SR1_BTF			2
#define I2C_SR1_RxNE		6
#define I2C_SR1_TxE			7



#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_usart_driver.h"
#include "stm32f446xx_i2c_driver.h"
#include "stm32f407xx_rcc_driver.h"

#endif /* INC_STM32F446XX_H_ */
