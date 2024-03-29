/*
 * stm32f407xx.h
 *
 *  Created on: Jul 1, 2019
 *      Author: gabriel
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define __vo volatile


/*************************START: Processor Specific Details***********************************
 *
 * ARM Cortex M4 Processor NVIC ISERx Register Addresses (Interrupt Set)
 */

#define NVIC_ISER0				((__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1				((__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2				((__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3				((__vo uint32_t*)0xE000E10C )

/*
 * ARM Cortex M4 Processor NVIC ICERx Register Addresses (Interrupt Clear)
 */

#define NVIC_ICER0				((__vo uint32_t*)0xE000E180 )
#define NVIC_ICER1				((__vo uint32_t*)0xE000E184 )
#define NVIC_ICER2				((__vo uint32_t*)0xE000E188 )
#define NVIC_ICER3				((__vo uint32_t*)0xE000E18C )


/*
 * ARM Cortex M4 Processor NVIC Priority Register Address calculation
 */
#define NVIC_PR_BASE_ADDR		((__vo uint32_t*)0xE000E4000)

// specific to each microcontroller. This value is 3 in some TI MCUs
#define NO_PR_BITS_IMPLEMENTED	4

/*
 * base addresses of flash and SRAM memories
 */
// base addresses are from STM32F407xx reference manual
// by default the values are considered signed, so we add U for unsigned int
#define FLASH_BASEADDR 			0x08000000U // also called main memory
#define SRAM1_BASEADDR 			0x20000000U // 112kB size, SRAM2 starts after 112kB
#define SRAM2_BASEADDR			0x2001C000U // SRAM2 is the auxiliary internal SRAM
#define ROM 					0x1FFF0000U // also called system memory
#define SRAM 					SRAM1_BASEADDR // SRAM1 acts as our main SRAM

#define PERIPH_BASE_ADDR		0x40000000U // base address of all peripherals
#define APB1_PERIPH_BASEADDR	PERIPH_BASE_ADDR // APB1 starts, also is TIM2's CR1 register
#define APB2_PERIPH_BASEADDR	0x40010000U // APBs are lower speed
#define AHB1_PERIPH_BASEADDR	0x40020000U // AHB (bus) peripherals are higher speed
#define AHB2_PERIPH_BASEADDR	0x50000000U // They do GPIOs and camera interfaces for example

// GPIO Peripherals on AHB1 bus
#define GPIOA_BASEADDR			(AHB1_PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR			(AHB1_PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR			(AHB1_PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR			(AHB1_PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR			(AHB1_PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR			(AHB1_PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR			(AHB1_PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR			(AHB1_PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR			(AHB1_PERIPH_BASEADDR + 0x2000)
#define GPIOJ_BASEADDR			(AHB1_PERIPH_BASEADDR + 0x2400)
#define GPIOK_BASEADDR			(AHB1_PERIPH_BASEADDR + 0x2800)

#define RCC_BASEADDR			(AHB1_PERIPH_BASEADDR + 0x3800)

// I2C, SPI, and USART peripherals on APB1 bus
#define I2C1_BASEADDR			(APB1_PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR			(APB1_PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR			(APB1_PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR			(APB1_PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR			(APB1_PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR			(APB1_PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR			(APB1_PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR			(APB1_PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR			(APB1_PERIPH_BASEADDR + 0x5000)
#define UART7_BASEADDR			(APB1_PERIPH_BASEADDR + 0x7800)
#define UART8_BASEADDR			(APB1_PERIPH_BASEADDR + 0x7C00)

// SPI, UART, EXT1 (interrupt) and SYSCFG peripherals on APB2 bus
#define EXTI_BASEADDR			(APB2_PERIPH_BASEADDR + 0x3C00)

#define SPI1_BASEADDR			(APB2_PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR			(APB2_PERIPH_BASEADDR + 0x3400)

#define SYSCFG_BASEADDR			(APB2_PERIPH_BASEADDR + 0x3800)

#define USART1_BASEADDR			(APB2_PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR			(APB2_PERIPH_BASEADDR + 0x1400)


/*********************** PERIPHERAL REGISTER DEFINITION STRUCTURE******************************************/
// Specific to the STM32F407xx

typedef struct
{
	__vo uint32_t MODER;		// Mode Register							Address Offset: 0x00
	__vo uint32_t OTYPER;		// Output Type Register						Address Offset: 0x04
	__vo uint32_t OSPEEDR;		// Output Speed Register					Address Offset: 0x08
	__vo uint32_t PUPDR;		// Pullup Pulldown Register					Address Offset: 0x0C
	__vo uint32_t IDR;			// Input Data Register						Address Offset: 0x10
	__vo uint32_t ODR;			// Output Data Register						Address Offset: 0x14
	__vo uint32_t BSRR;			// Bit Set/Reset Register					Address Offset: 0x18
	__vo uint32_t LCKR;			// Configuration Lock Register				Address Offset: 0x1C
	__vo uint32_t AFR[2]; 		// Alternate Function Low/High Register		Address Offset: 0x20-0x24
	// array of 2 uint32, AFRL register and AFRH register, alternate function low and high
	// AFR[0]: Low Register, AFR[1]: High Register
}GPIO_RegDef_t;

/*
 * Peripheral Register Definition Structure for SPI
 */
typedef struct
{
	__vo uint32_t CR1;			// Control Register	1						Address Offset: 0x00
	__vo uint32_t CR2;			// Control Register	2						Address Offset: 0x04
	__vo uint32_t SR;			// Status Register							Address Offset: 0x08
	__vo uint32_t DR;			// Data Register							Address Offset: 0x0C
	__vo uint32_t CRCPR;		// CRC Polynomial Register					Address Offset: 0x10
	__vo uint32_t RXCRCR;		// RX CRC Register							Address Offset: 0x14
	__vo uint32_t TXCRCR;		// TX CRC Register							Address Offset: 0x18
	__vo uint32_t I2SCFGR;		// I2S Config Register						Address Offset: 0x1C
	__vo uint32_t I2SPR;		// I2S Prescaler Register					Address Offset: 0x20
}SPI_RegDef_t;

/*
 * Peripheral Register Definition Structure for RCC (Reset and Clock Control)
 */
typedef struct
{
	__vo uint32_t CR;			// Clock Control Register					Address Offset: 0x00
	__vo uint32_t PLLCFGR;		// PLL Configuration Register				Address Offset: 0x04
	__vo uint32_t CFGR;			// Clock Configuration Register				Address Offset: 0x08
	__vo uint32_t CIR;			// Clock Interrupt Register					Address Offset: 0x0C
	__vo uint32_t AHB1RSTR;		// AHB1 Peripheral Reset Register			Address Offset: 0x10
	__vo uint32_t AHB2RSTR;		// AHB2 Peripheral Reset Register			Address Offset: 0x14
	__vo uint32_t AHB3RSTR;		// AHB3 Peripheral Reset Register			Address Offset: 0x18
	uint32_t 	  RESERVED0;	// Reserved, 0x1C
	__vo uint32_t APB1RSTR; 	// APB1 Peripheral Reset Register			Address Offset: 0x20
	__vo uint32_t APB2RSTR;		// APB2 Peripheral Reset Register			Address Offset: 0x24
	uint32_t 	  RESERVED1[2];	// Reserved, 0x28-0x2C
	__vo uint32_t AHB1ENR;		// AHB1 Peripheral Clock Enable Register	Address Offset: 0x30
	__vo uint32_t AHB2ENR;		// AHB2 Peripheral Clock Enable Register	Address Offset: 0x34
	__vo uint32_t AHB3ENR;		// AHB3 Peripheral Clock Enable Register	Address Offset: 0x38
	uint32_t 	  RESERVED2;	// Reserved, 0x3C
	__vo uint32_t APB1ENR;		// APB1 Peripheral Clock Enable Register	Address Offset: 0x40
	__vo uint32_t APB2ENR; 		// APB2 Peripheral Clock Enable Register	Address Offset: 0x44
	uint32_t 	  RESERVED3[2]; // Reserved, 0x48-0x4C
	__vo uint32_t AHB1LPENR;	// AHB1 Peripheral Clock Enable Low Power	Address Offset: 0x50
	__vo uint32_t AHB2LPENR;	// AHB2 Peripheral Clock Enable Low Power	Address Offset: 0x54
	__vo uint32_t AHB3LPENR;	// AHB3 Peripheral Clock Enable Low Power	Address Offset: 0x58
	uint32_t 	  RESERVED4;	// Reserved, 0x5C
	__vo uint32_t APB1LPENR;	// APB1 Peripheral Clock Enable Low Power	Address Offset: 0x60
	__vo uint32_t APB2LPENR;	// APB2 Peripheral Clock Enable Low Power	Address Offset: 0x64
	uint32_t 	  RESERVED5[2]; // Reserved, 0x68-0x6C
	__vo uint32_t BDCR;			// Backup Domain Control Register			Address Offset: 0x70
	__vo uint32_t CSR;			// Clock Control & Status Register			Address Offset: 0x74
	uint32_t 	  RESERVED6[2];	// Reserved, 0x78-0x7C
	__vo uint32_t SSCGR;		// Spread Spectrum Clock Generation			Address Offset: 0x80
	__vo uint32_t PLLI2SCFGR;	// PLLI2S Configuration Register			Address Offset: 0x84
	__vo uint32_t PLLSAICFGR;	// PLL Configuration Register				Address Offset: 0x88
	__vo uint32_t DCKCFGR; 		// Dedicated Clock Configuration Register	Address Offset: 0x8C
}RCC_RegDef_t;

/*
 * Peripheral Register Definition Structure for EXTI
 */
typedef struct
{
	__vo uint32_t IMR; 			// Interrupt Mask Register					Address Offset: 0x00
	__vo uint32_t EMR; 			// Event Mask Register						Address Offset: 0x04
	__vo uint32_t RTSR; 		// Rising Trigger Selection Register		Address Offset: 0x08
	__vo uint32_t FTSR; 		// Falling Trigger Selection Register		Address Offset: 0x0C
	__vo uint32_t SWIER; 		// Software Interrupt Event Register		Address Offset: 0x10
	__vo uint32_t PR; 			// Pending  Register						Address Offset: 0x14
}EXTI_RegDef_t;

/*
 * Peripheral Register Definition Structure for SYSCFG
 */
typedef struct
{
	__vo uint32_t MEMRMP; 		// Memory Remap Register					Address Offset: 0x00
	__vo uint32_t PMC; 			// Peripheral Mode Configuration Register	Address Offset: 0x04
	__vo uint32_t EXTICR[4]; 	// External Interrupt Config 1-4 Register	Address Offset: 0x08-0x14
	uint32_t RESERVED[2]; 		// Reserved, 0x18-0x1C
	__vo uint32_t CMPCR; 		// Compensation Cell Control Register		Address Offset: 0x20
}SYSCFG_RegDef_t;


// Peripheral Definitions (Peripheral base addresses typecasted to xxx_RegDef_t)
// We can use these later in our application to create easy pointers to our base address and peripheral register structures

#define GPIOA 					((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB 					((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC 					((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD 					((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE 					((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF 					((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG 					((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH 					((GPIO_RegDef_t*) GPIOH_BASEADDR)
#define GPIOI 					((GPIO_RegDef_t*) GPIOI_BASEADDR)
#define GPIOJ 					((GPIO_RegDef_t*) GPIOJ_BASEADDR)
#define GPIOK 					((GPIO_RegDef_t*) GPIOK_BASEADDR)

#define RCC						((RCC_RegDef_t*) RCC_BASEADDR)
#define EXTI					((EXTI_RegDef_t*) EXTI_BASEADDR)
#define SYSCFG					((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)

#define SPI1 					((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2 					((SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3 					((SPI_RegDef_t*) SPI3_BASEADDR)
#define SPI4 					((SPI_RegDef_t*) SPI4_BASEADDR)

// Clock Enable Macros for GPIOx Peripherals
#define GPIOA_PCLK_EN() 		(RCC->AHB1ENR |= (1 << 0) )
#define GPIOB_PCLK_EN() 		(RCC->AHB1ENR |= (1 << 1) )
#define GPIOC_PCLK_EN() 		(RCC->AHB1ENR |= (1 << 2) )
#define GPIOD_PCLK_EN() 		(RCC->AHB1ENR |= (1 << 3) )
#define GPIOE_PCLK_EN() 		(RCC->AHB1ENR |= (1 << 4) )
#define GPIOF_PCLK_EN() 		(RCC->AHB1ENR |= (1 << 5) )
#define GPIOG_PCLK_EN() 		(RCC->AHB1ENR |= (1 << 6) )
#define GPIOH_PCLK_EN() 		(RCC->AHB1ENR |= (1 << 7) )
#define GPIOI_PCLK_EN() 		(RCC->AHB1ENR |= (1 << 8) )
#define GPIOJ_PCLK_EN() 		(RCC->AHB1ENR |= (1 << 9) )
#define GPIOK_PCLK_EN() 		(RCC->AHB1ENR |= (1 << 10) )

// Clock Enable Macros for I2Cx Peripherals
#define I2C1_PCLK_EN() 			(RCC->APB1ENR |= (1 << 21) )
#define I2C2_PCLK_EN() 			(RCC->APB1ENR |= (1 << 22) )
#define I2C3_PCLK_EN() 			(RCC->APB1ENR |= (1 << 23) )

// Clock Enable Macros for SPIx Peripherals
#define SPI1_PCLK_EN() 			(RCC->APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN() 			(RCC->APB1ENR |= (1 << 14) )
#define SPI3_PCLK_EN() 			(RCC->APB1ENR |= (1 << 15) )
#define SPI4_PCLK_EN() 			(RCC->APB2ENR |= (1 << 13) )

// Clock Enable Macros for USARTx Peripherals
#define USART1_PCLK_EN()		(RCC->APB2ENR |= (1 << 4) )
#define USART2_PCLK_EN()		(RCC->APB1ENR |= (1 << 17) )
#define USART3_PCLK_EN()		(RCC->APB1ENR |= (1 << 18) )
#define UART4_PCLK_EN()			(RCC->APB1ENR |= (1 << 19) )
#define UART5_PCLK_EN()			(RCC->APB1ENR |= (1 << 20) )
#define USART6_PCLK_EN()		(RCC->APB2ENR |= (1 << 5) )
#define UART7_PCLK_EN()			(RCC->APB1ENR |= (1 << 30) )
#define UART8_PCLK_EN()			(RCC->APB1ENR |= (1 << 31) )

// Clock Enable Macros for SYSCFG Peripheral
#define SYSCFG_PCLK_EN()		(RCC->APB2ENR |= (1 << 14) )

// Clock Disable Macros for GPIOx Peripherals
#define GPIOA_PCLK_DI() 		(RCC->AHB1ENR &= ~(1 << 0) )
#define GPIOB_PCLK_DI() 		(RCC->AHB1ENR &= ~(1 << 1) )
#define GPIOC_PCLK_DI() 		(RCC->AHB1ENR &= ~(1 << 2) )
#define GPIOD_PCLK_DI() 		(RCC->AHB1ENR &= ~(1 << 3) )
#define GPIOE_PCLK_DI() 		(RCC->AHB1ENR &= ~(1 << 4) )
#define GPIOF_PCLK_DI() 		(RCC->AHB1ENR &= ~(1 << 5) )
#define GPIOG_PCLK_DI() 		(RCC->AHB1ENR &= ~(1 << 6) )
#define GPIOH_PCLK_DI() 		(RCC->AHB1ENR &= ~(1 << 7) )
#define GPIOI_PCLK_DI() 		(RCC->AHB1ENR &= ~(1 << 8) )
#define GPIOJ_PCLK_DI() 		(RCC->AHB1ENR &= ~(1 << 9) )
#define GPIOK_PCLK_DI() 		(RCC->AHB1ENR &= ~(1 << 10) )

// Clock Disable Macros for I2Cx Peripherals
#define I2C1_PCLK_DI() 			(RCC->APB1ENR &= ~(1 << 21) )
#define I2C2_PCLK_DI() 			(RCC->APB1ENR &= ~(1 << 22) )
#define I2C3_PCLK_DI() 			(RCC->APB1ENR &= ~(1 << 23) )

// Clock Disable Macros for SPIx Peripherals
#define SPI1_PCLK_DI() 			(RCC->APB2ENR &= ~(1 << 12) )
#define SPI2_PCLK_DI() 			(RCC->APB1ENR &= ~(1 << 14) )
#define SPI3_PCLK_DI() 			(RCC->APB1ENR &= ~(1 << 15) )
#define SPI4_PCLK_DI() 			(RCC->APB2ENR &= ~(1 << 13) )

// Clock Disable Macros for USARTx Peripherals
#define USART1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 4) )
#define USART2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 17) )
#define USART3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 18) )
#define UART4_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 19) )
#define UART5_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 20) )
#define USART6_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 5) )
#define UART7_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 30) )
#define UART8_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 31) )

// Clock Disable Macros for SYSCFG Peripheral
#define SYSCFG_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 14) )

/*
 * Macros to reset GPIOx peripherals
 */
// do while loop to include multiple statements in a single C macro, executes only once
// no need to put a semi colon at the very end since when we use GPIOA_REG_RESET(); in the .c file
// we will have a semi colon
// NEED to set the reset pin to 1 and then back to 0 so we're not continuously reseting the peripheral
#define GPIOA_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 0));   (RCC->AHB1RSTR &= ~(1 << 0));} while(0)
#define GPIOB_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 1));   (RCC->AHB1RSTR &= ~(1 << 1));} while(0)
#define GPIOC_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 2));   (RCC->AHB1RSTR &= ~(1 << 2));} while(0)
#define GPIOD_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 3));   (RCC->AHB1RSTR &= ~(1 << 3));} while(0)
#define GPIOE_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 4));   (RCC->AHB1RSTR &= ~(1 << 4));} while(0)
#define GPIOF_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 5));   (RCC->AHB1RSTR &= ~(1 << 5));} while(0)
#define GPIOG_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 6));   (RCC->AHB1RSTR &= ~(1 << 6));} while(0)
#define GPIOH_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 7));   (RCC->AHB1RSTR &= ~(1 << 7));} while(0)
#define GPIOI_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 8));   (RCC->AHB1RSTR &= ~(1 << 8));} while(0)
#define GPIOJ_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 9));   (RCC->AHB1RSTR &= ~(1 << 9));} while(0)
#define GPIOK_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 10));  (RCC->AHB1RSTR &= ~(1 << 10));} while(0)

/*
 * Macros to reset SPIx peripherals
 */
#define SPI1_REG_RESET()		do{(RCC->APB2RSTR |= (1 << 12));   (RCC->APB2RSTR &= ~(1 << 12));} while(0)
#define SPI2_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 14));   (RCC->APB1RSTR &= ~(1 << 14));} while(0)
#define SPI3_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 15));   (RCC->APB1RSTR &= ~(1 << 15));} while(0)
#define SPI4_REG_RESET()		do{(RCC->APB2RSTR |= (1 << 13));   (RCC->APB2RSTR &= ~(1 << 13));} while(0)

/*
 * Returns portcode for a given GPIO port i.e. A is 0000, B is 0001, C is 0010...
 */
// C conditional or ternary operations:
// ? means is 'then'. : means 'else','\' means line break, the braces at start is an if statement
#define GPIO_BASEADDR_TO_CODE(x)  ( (x == GPIOA) ? 0 :\
									(x == GPIOB) ? 1 :\
									(x == GPIOC) ? 2 :\
									(x == GPIOD) ? 3 :\
									(x == GPIOE) ? 4 :\
									(x == GPIOF) ? 5 :\
									(x == GPIOG) ? 6 :\
									(x == GPIOH) ? 7 :\
									(x == GPIOI) ? 8 :\
									(x == GPIOJ) ? 9 :0)

/*
 * IRQ (Interrupt Request) Numbers for STM32F407x
 * NOTE: These are super specific to the MCU, look in the interrupt vector table in the reference manual
 * You can update this list with other peripherals that need interrupts
 */

#define IRQ_NO_EXTI0				6
#define IRQ_NO_EXTI1				7
#define IRQ_NO_EXTI2				8
#define IRQ_NO_EXTI3				9
#define IRQ_NO_EXTI4				10
#define IRQ_NO_EXTI5_9				23
#define IRQ_NO_EXTI10_15			40

#define IRQ_NO_SPI1					35
#define IRQ_NO_SPI2					36
#define IRQ_NO_SPI3					51

/*
 * IRQ PRIORITY LEVEL
 */
#define NVIC_IRQ_PRIO0				0
#define NVIC_IRQ_PRIO1				1
#define NVIC_IRQ_PRIO2				2
#define NVIC_IRQ_PRIO3				3
#define NVIC_IRQ_PRIO4				4
#define NVIC_IRQ_PRIO5				5
#define NVIC_IRQ_PRIO6				6
#define NVIC_IRQ_PRIO7				7
#define NVIC_IRQ_PRIO8				8
#define NVIC_IRQ_PRIO9				9
#define NVIC_IRQ_PRIO10				10
#define NVIC_IRQ_PRIO11				11
#define NVIC_IRQ_PRIO12				12
#define NVIC_IRQ_PRIO13				13
#define NVIC_IRQ_PRIO14				14
#define NVIC_IRQ_PRIO15				15

// Some Generic Macros
#define ENABLE 					1
#define DISABLE 				0
#define SET 					ENABLE
#define RESET 					DISABLE
#define GPIO_PIN_SET			SET
#define GPIO_PIN_RESET			RESET
#define FLAG_RESET				RESET
#define FLAG_SET				SET

/********************************************************************************************
 * Bit Position Definitions of SPI peripheral
 ********************************************************************************************/
/*
 * Bit position definitions for SPI_CR1
 */
#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSBFIRST		7
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_DFF				11
#define SPI_CR1_CRCNEXT			12
#define SPI_CR1_CRCEN			13
#define SPI_CR1_BIDIOE			14
#define SPI_CR1_BIDIMODE		15

/*
 * Bit position definitions for SPI_CR2
 */
#define SPI_CR2_RXDMAEN			0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE			2
#define SPI_CR2_FRF				4
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7

/*
 * Bit position definitions for SPI_SR Status Register
 */
#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRCERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7
#define SPI_SR_FRE				8



#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"

#endif /* INC_STM32F407XX_H_ */
