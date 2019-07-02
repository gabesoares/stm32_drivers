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
	GPIO_RegDef_t *pGPIOx; // pointer type: This holds the base address of the GPIO port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;   // This holds GPIO pin configuration settings

}GPIO_Handle_t;

/**************************************************************************************************
 * 									APIs Supported by this Driver
 * 					For more information about the APIs check the function definitions
 **************************************************************************************************/

/*
 * Peripheral Clock Setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi); // user sends a pointer pointing to base address

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle); // user creates a pointer of this type and sends it
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data Read and Write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber); // returns a 0 or 1
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx); // reads from a whole 16 bit port 0s or 1s
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);


/*
 * IRQ Configuration and ISR Handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQhandling(uint8_t PinNumber);










#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
