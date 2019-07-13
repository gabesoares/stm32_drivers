/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Jul 8, 2019
 *      Author: gabriel
 */

#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx.h"


/***********************************************************
 * @function 				- SPI_PeriClockControl
 *
 * @brief					- This function enables or disables peripheral clock for the given SPI port
 *
 * @param[in]				- Base address of the SPI port peripheral (SPI1/SPI2/SPI3...)
 * @param[in]				- ENABLE or DISABLE macros
 *
 * @return					- None
 *
 * @Note					- None
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) // user sends a pointer pointing to base address
{
	if (EnorDi == ENABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if (pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
	}
	else
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
		else if (pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}
	}
}

/***********************************************************
 * @function 				- SPI_Init
 *
 * @brief					- Initializes the given SPI port to the desired pin config
 *
 * @param[in]				- Handle structure containing SPI port base address and SPI configuration structure
 *
 * @return					- None
 *
 * @Note					- The handle and pin config structure format can be found in stm32f407xx_spi_driver.h
 */
void SPI_Init(SPI_Handle_t *pSPIHandle) // user creates a pointer of this type and sends it
{
	uint32_t tempreg = 0;

	// enable peripheral clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// 1. Configure the device mode
	// Make the MCU as a master to be able to produce a serial clock
	// MSTR bit in CR1 register needs to be set to 1 (device mode)

	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	// 2. Bus Configuration

	// BIDI MODE (BiDirectional Mode) bit in CR1 register, which is for full/half duplex
	// set to 0 for full duplex, set to 1 for half duplex

	// BIDI OE (BiDirectional Output Enable)
	// if bit is 0, then output is disabled (device acts as receive-only mode)
	// if bit is 1, then output is enabled (device acts as transmit-only mode)
	// if the BIDI MODE bit is 0, then this bit is a don't care, since full duplex

	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) // if full duplex selected
	{
		// BIDI Mode pin should be 0
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) // if half duplex selected
	{
		// BIDI Mode pin should be 1
		tempreg |= 1 << SPI_CR1_BIDIMODE;
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		// RXONLY bit only valid when BIDI MODE is 2-line unidirectional , set to 0
		// In this case you don't connect MOSI, only MISO
		// but problem is that the clock is produced only when data is produced on MOSI line
		// so SCK won't be produced.. so we need to force it to produce clock even if MOSI line is unconfigured
		// we do this by forcing the RXONLY bit to 1, receive-only mode

		// BIDI Mode pin should be 0 (2-line unidirectional)
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		// RXONLY bit must be set
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	// 3. Configure the SPI Serial Clock Speed (Baudrate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	// 4. Configure the Data Frame Format (DFF)
	// DFF bit, in CR1. The data frame (shift register) format
	// 0 for 8-bit data frame. 1 for 16-bit data frame
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	// 5. Configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;
	// 6. Configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	// SSM is bit 9 in CR1
	// when 0 software slave management is disabled (HW chip select). when 1 SSM is enabled (SW)
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = tempreg; // freshly using CR1 so can use the assignment operator
}

/***********************************************************
 * @function 				- SPI_DeInit
 *
 * @brief					- This function resets the SPI registers to the default setting
 *
 * @param[in]				- Base address of the SPI port peripheral (SPI1/SPI2/SPI3...)
 *
 * @return					- None
 *
 * @Note					- None
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	// can use the RCC AHB1 Peripheral Reset Register RCC_AHB1RSTR
	// need to write a 1 and after a few clock cycles back to 0

	if (pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if (pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if (pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
	else if (pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	}
}

/***********************************************************
 * @function 				- SPI_GetFlagStatus
 *
 * @brief					- This function returns the status of a desired flag in the SPI Status Register
 *
 * @param[in]				- Base address of the SPI port peripheral (SPI1/SPI2/SPI3...)
 * @param[in]				- Flag Name
 *
 * @return					- Whether the flag is set or not: FLAG_SET or FLAG_RESET
 *
 * @Note					- None
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName) // testing if the bit position of the flag if 0 or 1
	{
		// flags are specified in the SPI header file, from the status register
		return FLAG_SET;
	}
	else
	{
		return FLAG_RESET;
	}

}
/***********************************************************
 * @function 				- SPI_SendData
 *
 * @brief					- This function sends the desired data over SPI
 *
 * @param[in]				- Base address of the SPI port peripheral (SPI1/SPI2/SPI3...)
 * @param[in]				- Pointer to an 8-bit buffer for the data
 * @param[in]				- Length of data to be sent
 *
 * @return					- None
 *
 * @Note					- This is a blocking/polling function
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	// blocking API (function call will wait until all the bytes are transmitted)
	// Waits until Tx buffer is empty (check status register: TXE flag goes high when Tx buffer is empty)
	// can also do a non-blocking one that uses interrupts

	// Then check DFF, if it's 8-bit load data register with 1 byte
	// if it's 16 bits need to load data register with 2 byts
	// then decrement Len and keep loop going till Len = 0

	// SPI data register: if you write to it, the value will go into the Tx buffer
	// (before you write to it, you need to make sure it's empty. i.e. the TXE flag is high)
	// if you read from it, you are reading the values from the Rx buffer
	// (before you read from it, you need to make sure the receive buffer is FULL, RXNE flag is high).

	while (Len > 0)
	{
		// 1. Wait until TXE is set (=1)
		while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET );

		// 2. Check the DFF bit in CR1
		if ( (pSPIx->CR1 & (1 << SPI_CR1_DFF) ) ) // if the DFF bit is set to 1
		{
			// DFF is 16 bit

			// 1. load the data into the Data Register (DR)
			// (need to typecast to 16-bit pointer first, then dereference
			pSPIx->DR = *((uint16_t *) pTxBuffer);
			// Just sent out 2 bits of data
			Len--;
			Len--;
			// now increment the pointer, typecast to 16 bit first
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			// DFF is 8 bit
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}

/***********************************************************
 * @function 				- SPI_ReceiveData
 *
 * @brief					- This function reads  data over SPI
 *
 * @param[in]				- Base address of the SPI port peripheral (SPI1/SPI2/SPI3...)
 * @param[in]				- Pointer to an 8-bit buffer for the data to be received
 * @param[in]				- Length of data to be received
 *
 * @return					- None
 *
 * @Note					- None
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while (Len > 0)
	{
		// 1. Wait until RXNE is set (=1)
		while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET );

		// 2. Check the DFF bit in CR1
		if ( (pSPIx->CR1 & (1 << SPI_CR1_DFF) ) ) // if the DFF bit is set to 1
		{
			// DFF is 16 bit

			// 1. Read the data from the Data Register (DR) into pRxBuffer address
			// (need to typecast to 16-bit pointer first, then dereference
			*((uint16_t *) pRxBuffer) =pSPIx->DR;
			// Just read 2 bits of data
			Len--;
			Len--;
			// now increment the pointer, typecast to 16 bit first
			(uint16_t*)pRxBuffer++;
		}
		else
		{
			// DFF is 8 bit
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}

/***********************************************************
 * @function 				- SPI_PeripheralControl
 *
 * @brief					- This function enables or disables the SPIx peripheral
 *
 * @param[in]				- Base address of the SPI port peripheral (SPI1/SPI2/SPI3...)
 * @param[in]				- Enable or Disable macro
 *
 * @return					- None
 *
 * @Note					- Enables the SPE bit of the SPI_CR1 register
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/***********************************************************
 * @function 				- SPI_SSIConfig
 *
 * @brief					- This function sets or resets the SSI pin for software slave management (SSM)
 *
 * @param[in]				- Base address of the SPI port peripheral (SPI1/SPI2/SPI3...)
 * @param[in]				- Enable or Disable macro
 *
 * @return					- None
 *
 * @Note					- Enables the SSI bit of the SPI_CR1 register
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/***********************************************************
 * @function 				- SPI_SSOEConfig
 *
 * @brief					- This function sets or resets the SSOE pin for slave select output enable
 *
 * @param[in]				- Base address of the SPI port peripheral (SPI1/SPI2/SPI3...)
 * @param[in]				- Enable or Disable macro
 *
 * @return					- None
 *
 * @Note					- Enables the SSOE bit of the SPI_CR2 register
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

/*
 * IRQ Configuration and ISR Handling
 */

/***********************************************************
 * @function 				- SPI_IRQInterruptConfig
 *
 * @brief					- This function configures the IRQ registers for interrupts
 *
 * @param[in]				- IRQ Number
 * @param[in]				- ENABLE or DISABLE macros
 *
 * @return					- None
 *
 * @Note					- None
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	// processor specific! dealing with NVIC. EXTI is on the microcontroller peripheral side
	// need to look at the ARM Cortex-M4 User Guide document and the NVIC section
	// enabling/disabling interrupts on a given interrupt

	if (EnorDi == ENABLE)
	{
		if (IRQNumber <= 31)
		{
			// program ISER0 register in NVIC (interrupt enable)
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64)
		{
			// program ISER1 register in NVIC (interrupt enable)
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			// program ISER2 register in NVIC (interrupt enable)
			// only 81 interrupts in our STM32F407
			*NVIC_ISER2 |= (1 << (IRQNumber % 32));
		}
	}
	else
	{
		if (IRQNumber <= 31)
		{
			// program ICER0 register in NVIC (interrupt clear)
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64)
		{
			// program ICER1 register in NVIC (interrupt clear)
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			// program ICER2 register in NVIC (interrupt clear)
			// only 81 interrupts in our STM32F407
			*NVIC_ICER2 |= (1 << (IRQNumber % 32));
		}
	}
}


/***********************************************************
 * @function 				- SPI_IRQPriorityConfig
 *
 * @brief					- This function configures the priorities for interrupts in IRQ register
 *
 * @param[in]				- IRQ Number
 * @param[in]				- Priority level for the interrupt
 *
 * @return					- None
 *
 * @Note					- Registers are processor-specific, so are found in the Cortex-M4 User Guide
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// First find out the IPR register (Interrupt Priority Register)


	// each IPR register in the NVIC handles 4 different IRQs to set priorities
	// i.e. in IPR0: IRQ3_PRI IRQ2_PRI IRQ1_PRI IRQ0_PRI, each with 8 bits
	// i.e. in IPR59 IRQ239_PRI IRQ238_PRI IRQ237_PRI IRQ236_PRI each with 8 bits

	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	// only upper four bits in each 8 bit section of the register is used to set priority
	// lower four bits are not used, specific to the microcontroller
	// in ST case it is 4
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	// adding to base address to get the right register
	// the next address location is one uint32_t away, since NVIC_PR_BASE_ADDR is a uint32_t pointer
	// so incrementing the pointer will go to the next address 32 bits away
	*(NVIC_PR_BASE_ADDR + iprx) |= ( IRQPriority << shift_amount );
}

/***********************************************************
 * @function 				- SPI_IRQhandling
 *
 * @brief					- This function does the interrupt handler (ISR) for a given pin
 *
 * @param[in]				- Desired pin to attach an ISR to
 *
 * @return					- None
 *
 * @Note					- None
 */
void SPI_IRQhandling(SPI_Handle_t *pHandle)
{
	// EXTI has a Pending Register PR
	// NVIC also has a Pending Register PR

	// Implement ISR - application-specific

	// clear the EXTI PR register corresponding to the pin number
	/*
	if (EXTI->PR & (1 << PinNumber))
	{
		// clear
		EXTI->PR |= (1 << PinNumber);
	}
	*/

	// store the address of the ISR at the vector address location
	// corresponding to the IRQ Number for which you have written the ISR
}
