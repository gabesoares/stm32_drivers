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
{	// 1 byte, since variables can be 0 to 15
	uint8_t GPIO_PinNumber; 		// See the possible values from @GPIO_PIN_NUMBERS
	uint8_t GPIO_PinMode;			// See the possible values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed; 			// See the possible values from @GPIO_PIN_SPEED
	uint8_t GPIO_PinPuPdControl; 	// See the possible values from @GPIO_PIN_PUPD
	uint8_t GPIO_PinOPType;			// See the possible values from @GPIO_PIN_OUTPUT_TYPES
	uint8_t GPIO_PinAltfunMode;
}GPIO_PinConfig_t;


typedef struct
{
	// pointer to hold the base address of the GPIO peripheral
	GPIO_RegDef_t *pGPIOx; // pointer type: This holds the base address of the GPIO port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;   // This holds GPIO pin configuration settings

}GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0 			0
#define GPIO_PIN_NO_1 			1
#define GPIO_PIN_NO_2 			2
#define GPIO_PIN_NO_3 			3
#define GPIO_PIN_NO_4 			4
#define GPIO_PIN_NO_5 			5
#define GPIO_PIN_NO_6 			6
#define GPIO_PIN_NO_7 			7
#define GPIO_PIN_NO_8 			8
#define GPIO_PIN_NO_9 			9
#define GPIO_PIN_NO_10 			10
#define GPIO_PIN_NO_11			11
#define GPIO_PIN_NO_12			12
#define GPIO_PIN_NO_13			13
#define GPIO_PIN_NO_14			14
#define GPIO_PIN_NO_15			15


/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN 			0
#define GPIO_MODE_OUT			1
#define GPIO_MODE_ALTFN			2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_IT_FT			4 // input falling edge interrupt
#define GPIO_MODE_IT_RT			5 // input interrupt rising edge
#define GPIO_MODE_IT_RFT		6 // input interrupt rising and falling edge

/*
 * @GPIO_PIN_OUTPUT_TYPES
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP 		0 // GPIO output type push pull
#define GPIO_OP_TYPE_OD			1 // GPIO output type open drain

/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW 			0
#define GPIO_SPEED_MEDIUM 		1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_HIGH			3

/*
 * @GPIO_PIN_PUPD
 * GPIO pin pull up pull down combinations
 */
#define GPIO_NO_PUPD			0
#define GPIO_PIN_PU				1
#define GPIO_PIN_PD				2
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
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQhandling(uint8_t PinNumber);










#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
