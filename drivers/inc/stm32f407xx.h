/*
 * stm32f407xx.h
 *
 *  Created on: Jul 1, 2019
 *      Author: gabriel
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

/*
 * base addresses of flash and SRAM memories
 */

// base addresses are from STM32F407xx reference manual
// by default the values are considered signed, so we add U for unsigned int
#define FLASH_BASEADDR 			0x08000000U // also called main memory
#define SRAM1_BASEADDR 			0x20000000U // 112kB size, SRAM2 starts after 112kB
#define SRAM2_BASEADDR			0x2001C000U // SRAM2 is the auxiliary internal SRAM
#define ROM 					0x1FFF0000U // also called system memory

#define PERIPH_BASEADDR			0x40000000U // base address of all peripherals
#define APB1PERIPH_BASEADDR		PERIPH_BASE_ADDR // APB1 starts, also is TIM2's CR1 register
#define APB2PERIPH_BASEADDR		0x40010000U // APBs are lower speed
#define AHB1PERIPH_BASEADDR		0x40020000U // AHB (bus) peripherals are higher speed
#define AHB2PERIPH_BASEADDR		0x50000000U // They do GPIOs and camera interfaces for example

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



#define SRAM 					SRAM1_BASEADDR /* SRAM1 acts as our main SRAM

#endif /* INC_STM32F407XX_H_ */
