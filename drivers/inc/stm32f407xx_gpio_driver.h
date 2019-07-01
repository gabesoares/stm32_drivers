/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Jul 1, 2019
 *      Author: gabriel
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"



typedef struct
{
	uint8_t GPIO_PinNumber; // 1 byte, since variables can be 0 to 15
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltfunMode;
}GPIO_PinConfig_t;


typedef struct
{
	// pointer to hold the base address of the GPIO peripheral
	GPIO_Reg_def_t *pGPIOx = GPIOA; // pointer type: This holds the base address of the GPIO port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;   // This holds GPIO pin configuration settings

}GPIO_Handle_t;

/**************************************************************************************************
 * 									APIs Supported by this Driver
 * 					For more information about the APIs check the function definitions
 **************************************************************************************************/

/*
 * Peripheral Clock Setup
 */
void GPIO_PeriClockControl(void);

/*
 * Init and De-init
 */
void GPIO_Init(void);
void GPIO_DeInit(void);

/*
 * Data Read and Write
 */
void GPIO_ReadFromInputPin(void);
void GPIO_ReadFromInputPort(void);
void GPIO_WriteToOutputPin(void);
void GPIO_WriteToOutputPort(void);
void GPIO_ToggleOutputPin(void);


/*
 * IRQ Configuration and ISR Handling
 */
void GPIO_IRQConfig(void);
void GPIO_IRQhandling(void);










#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
