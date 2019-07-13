/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Jul 8, 2019
 *      Author: gabriel
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

/*
 * Configuration structure for SPIx peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode; // See the possible values from @SPI_DEVICE_MODE
	uint8_t SPI_BusConfig;	// See the possible values from @SPI_BusConfig
	uint8_t SPI_SclkSpeed;  // See the possible values from @SPI_SclkSpeed
	uint8_t SPI_DFF;		// See the possible values from @SPI_DFF
	uint8_t SPI_CPOL; 		// See the possible values from @SPI_CPOL (clock polarity (stay high or low when idle?)
	uint8_t SPI_CPHA; 		// See the possible values from @SPI_CPHA (clock phase, which edge to capture data on rising/falling)
	uint8_t SPI_SSM; 		// See the possible values from @SPI_SSM (slave select mode, could be internal)
}SPI_Config_t;

/*
 * Handle structure for SPIx peripheral
 */
typedef struct
{
	SPI_RegDef_t *pSPIx; // Pointer to hold base address of SPIx(x:0,1,2,) peripheral
	SPI_Config_t SPIConfig;

}SPI_Handle_t;

/*
 * @SPI_DeviceMode
 * Either master mode or slave mode
 */
#define SPI_DEVICE_MODE_MASTER 			1
#define SPI_DEVICE_MODE_SLAVE 			0

/*
 * @SPI_BusConfig
 * Define whether the bus is full duplex, half duplex, or simplex (tx or rx not both)
 */
#define SPI_BUS_CONFIG_FD 				1			// full duplex
#define SPI_BUS_CONFIG_HD 				2			// half duplex
//#define SPI_bUS_CONFIG_SIMPLEX_TXONLY 3			// simplex tx only, this is technically same as full duplex
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY 	3			// simplex rx only

/*
 * @SPI_SclkSpeed
 * Configure the baud rate, 3 bits to pick the divider to divide the APB bus clock
 */
#define SPI_SCLK_SPEED_DIV2				0			// F_clock (APB clock) divided by 2
#define SPI_SCLK_SPEED_DIV4				1			// F_clock (APB clock) divided by 4
#define SPI_SCLK_SPEED_DIV8				2			// F_clock (APB clock) divided by 8
#define SPI_SCLK_SPEED_DIV16			3			// F_clock (APB clock) divided by 16
#define SPI_SCLK_SPEED_DIV32			4			// F_clock (APB clock) divided by 32
#define SPI_SCLK_SPEED_DIV64			5			// F_clock (APB clock) divided by 64
#define SPI_SCLK_SPEED_DIV128			6			// F_clock (APB clock) divided by 128
#define SPI_SCLK_SPEED_DIV256			7			// F_clock (APB clock) divided by 256

/*
 * @SPI_DFF
 * Data frame format, 8-bit or 16-bit
 */
#define SPI_DFF_8BITS					0
#define SPI_DFF_16BITS					1

/*
 * @SPI_CPOL
 * SCK Clock Polarity (hold high/low when idle)
 */
#define SPI_CPOL_LOW					0
#define SPI_CPOL_HIGH					1

/*
 * @SPI_CPHA
 * SCK Clock Phase (Capture data on the rising edge or falling edge)
 */
#define SPI_CPHA_LOW					0
#define SPI_CPHA_HIGH					1

/*
 * @SPI_SSM
 * Slave select management
 */
#define SPI_SSM_DI						0 // default is hardware external chip select
#define SPI_SSM_EN						1 // software internal chip select management

/*
 * SPI Related Status Register Flag Definitions
 */
#define SPI_RXNE_FLAG					(1 << SPI_SR_RXNE)
#define SPI_TXE_FLAG					(1 << SPI_SR_TXE)
#define SPI_CHSIDE_FLAG					(1 << SPI_SR_CHSIDE)
#define SPI_UDR_FLAG					(1 << SPI_SR_UDR)
#define SPI_CRCERR_FLAG					(1 << SPI_SR_CRCERR)
#define SPI_MODF_FLAG					(1 << SPI_SR_MODF)
#define SPI_OVR_FLAG					(1 << SPI_SR_OVR)
#define SPI_BSY_FLAG					(1 << SPI_SR_BSY)
#define SPI_FRE_FLAG					(1 << SPI_SR_FRE)

/**************************************************************************************************
 * 									APIs Supported by this Driver
 * 					For more information about the APIs check the function definitions
 **************************************************************************************************/
/*
 * Peripheral Clock Setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi); // user sends a pointer pointing to base address

/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle); // user creates a pointer of this type and sends it
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len); // standard practice to define Length as uint32_T
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQhandling(SPI_Handle_t *pHandle);

/*
 * Other Peripheral Control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
