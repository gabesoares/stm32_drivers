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
#define SRAM1_BASEADDR 			0x20000000U
#define SRAM2_BASEADDR			0x2001C000U // SRAM2 is the auxiliary internal SRAM
#define ROM 					0x1FFF0000U // also called system memory

#define SRAM 					SRAM1_BASEADDR /* SRAM1 acts as our main SRAM

#endif /* INC_STM32F407XX_H_ */
