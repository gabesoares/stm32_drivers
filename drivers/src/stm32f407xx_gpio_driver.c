/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Jul 1, 2019
 *      Author: gabriel
 */


#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx.h"

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
	if (EnorDi == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	else
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
		else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
	}
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
	uint32_t temp = 0; // temporary register
	//1. configure the mode of GPIO pin

	// non-interrupt modes
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// left shift the PinMode to 2 times the pin number. in pinMode register each pin needs 2 bits for pinmode setting
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

		// clear the bits in the register first. Need to do this to make sure our pin setting works as intended
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else // interrupt modes 4,5,6 input falling, rising, and rising/falling
	{
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_FT)
		{
			// 1. Configure the falling trigger selection register (FTSR)
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear the corresponding RTSR bit to remove rising edge just in case
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RT)
		{
			// 1. Configure the rising trigger selection register (RTSR)
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear the corresponding FTSR bit to remove rising edge just in case
			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RFT)
		{
			// 1. Configure the rising and falling trigger selection register
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// 2. Configure the GPIO port selection in SYSCFG_EXTICR
		// Need to do this configuration since different pins/ports are multiplexed through EXTI
		// There are 16 EXTI multiplexers, EXTI0 has PA0, PB0, PC0... PJ0 all multiplexed to one output
		// EXTI15 has PA15, PB15, PC15... PJ15 all multiplexed to one output
		// These 16 EXTI multiplexers have their select lines held in the four EXTICR registers

		// this tells us which of the 4 EXTICR registers to write into
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		// this tells us what offset we need in the register to access our pin4
		uint8_t temp2 = 4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4);

		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

		SYSCFG_PCLK_EN(); // ENABLE  SYSCFG peripheral clock
		// clear bits in registers we plan on changing
		SYSCFG->EXTICR[temp1] &= ~(0xF << temp2);
		// set the bits we want to for the EXTI interrupt
		SYSCFG->EXTICR[temp1] |= portcode << temp2;

		// 3. Enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0;
	//2. configure the speed

	// left shift the PinSpeed to 2 times the pin number. in pinSpeed register each pin needs 2 bits for speed setting
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

	// clear the bits in the register first. Need to do this to make sure our pin setting works as intended
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;
	//3. configure the PuPd settings

	// left shift the pinPuPd to 2 times the pin number. in pinPuPd register each pin needs 2 bits for PuPd setting
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

	// clear the bits in the register first. Need to do this to make sure our pin setting works as intended
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;
	//4. configure the output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType <<  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	// clear the bits in the register first. Need to do this to make sure our pin setting works as intended
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;


	temp = 0;
	//5. configure the alternate functionality
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1, temp2;
		// do an integer divide to figure out if we're in AFRH (pins 8-15) or AFRL (pins 0-7)
		// we have an array AFR[2], AFR[0] is AFRL (low register) and AFR[1] is AFRH (high register)

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;

		// do a mod operation to figure out where to start the 4-bit positions for each pin
		// i.e. in AFRH, pin 8 starts with 4 bits at bit 0, pin 9 at bit 4, pin 10 at bit 8
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

		// clear the desired bits in the register first
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << ( 4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltfunMode << ( 4 * temp2));

	}

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
	// can use the RCC AHB1 Peripheral Reset Register RCC_AHB1RSTR
	// need to write a 1 and after a few clock cycles back to 0

	if (pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
	else if (pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}
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
	uint8_t value;
	// shift the corresponding bit position all the way to the right so we can read it
	// read by ANDing with 1 to get rid of everything else
	value = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x00000001 );

	return value;
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
	uint16_t value;
	// need entire Input Data Readport returned here
	value = (uint16_t) pGPIOx->IDR;

	return value;
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
	if (Value == GPIO_PIN_SET)
	{
		// write 1
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		// write 0
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
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
	pGPIOx->ODR = Value;
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
	// bitwise XOR operation
	pGPIOx->ODR ^= (1 << PinNumber);
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
