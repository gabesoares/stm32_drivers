/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Jul 1, 2019
 *      Author: gabriel
 */


#include "stm32f407xx_gpio_driver.h"

/***********************************************************
 * @function 				- GPIO_PeriClockControl
 *
 * @brief					- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]				- Base address of the GPIO port peripheral (GPIOA/GPIOB/GPIOC...)
 * @param[in]				- ENABLE or DISABLE macros
 *
 * @return					- None
 *
 * @Note					- None
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi) // user sends a pointer pointing to base address
{

}

/***********************************************************
 * @function 				- GPIO_Init
 *
 * @brief					- Initializes the given GPIO port to the desired pin config
 *
 * @param[in]				- Handle structure containing GPIO port base address and pin configuration structure
 *
 * @return					- None
 *
 * @Note					- The handle and pin config structure format can be found in stm32f407xx_gpio_driver.h
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) // user creates a pointer of this type and sends it
{

}

/***********************************************************
 * @function 				- GPIO_DeInit
 *
 * @brief					- This function resets the GPIO registers to the default setting
 *
 * @param[in]				- Base address of the GPIO port peripheral (GPIOA/GPIOB/GPIOC...)
 *
 * @return					- None
 *
 * @Note					- None
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

}

/***********************************************************
 * @function 				- GPIO_ReadFromInputPin
 *
 * @brief					- This function reads the desired GPIO pin
 *
 * @param[in]				- Base address of the GPIO port peripheral (GPIOA/GPIOB/GPIOC...)
 * @param[in]				- Desired pin number to be read from
 *
 * @return					- Returns a 0 or a 1
 *
 * @Note					- None
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) // returns a 0 or 1
{

}

/***********************************************************
 * @function 				- GPIO_ReadFromInputPort
 *
 * @brief					- This function reads all the pins of the desired GPIO port
 *
 * @param[in]				- Base address of the GPIO port peripheral (GPIOA/GPIOB/GPIOC...)
 *
 * @return					- A 16-bit int which stores all 16 pin states, 0 or 1
 *
 * @Note					- None
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) // reads from a whole 16 bit port 0s or 1s
{

}

/***********************************************************
 * @function 				- GPIO_WriteToOutputPin
 *
 * @brief					- This function writes a 0 or a 1 to the desired GPIO output pin
 *
 * @param[in]				- Base address of the GPIO port peripheral (GPIOA/GPIOB/GPIOC...)
 * @param[in]				- Desired pin number to write to
 * @param[in]				- Value to be written (0 or 1)
 *
 * @return					- None
 *
 * @Note					- None
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{

}

/***********************************************************
 * @function 				- GPIO_WriteToOutputPort
 *
 * @brief					- This function writes a 0 or a 1 to all pins in the desired GPIO output port
 *
 * @param[in]				- Base address of the GPIO port peripheral (GPIOA/GPIOB/GPIOC...)
 * @param[in]				- Desired values to be written (16 0s or 1s), entered as a 16-bit int
 *
 * @return					- None
 *
 * @Note					- None
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{

}

/***********************************************************
 * @function 				- GPIO_ToggleOutputPin
 *
 * @brief					- This function toggles the output value of a given GPIO pin
 *
 * @param[in]				- Base address of the GPIO port peripheral (GPIOA/GPIOB/GPIOC...)
 * @param[in]				- Desired GPIO pin to be toggled
 *
 * @return					- None
 *
 * @Note					- None
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

}


/*
 * IRQ Configuration and ISR Handling
 */

/***********************************************************
 * @function 				- GPIO_IRQConfig
 *
 * @brief					- This function configures the IRQ registers
 *
 * @param[in]				- IRQ Number
 * @param[in]				- Priority level for the interrupt
 * @param[in]				- ENABLE or DISABLE macros
 *
 * @return					- None
 *
 * @Note					- None
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi)
{

}

/***********************************************************
 * @function 				- GPIO_IRQhandling
 *
 * @brief					- This function does the interrupt handler (ISR) for a given pin
 *
 * @param[in]				- Desired pin to attach an ISR to
 *
 * @return					- None
 *
 * @Note					- None
 */
void GPIO_IRQhandling(uint8_t PinNumber)
{

}
