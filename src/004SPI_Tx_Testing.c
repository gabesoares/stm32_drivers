/*
 * 004SPI_Tx_Testing.c
 *
 *  Created on: Jul 8, 2019
 *      Author: gabriel
 */

#include <string.h>
#include "stm32f407xx.h"

void SPI2_GPIOInits()
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltfunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// SCK on PB13
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	// peripheral clock enabled in GPIO_Init function
	GPIO_Init(&SPIPins);

	// MOSI on PB15
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	// peripheral clock enabled in GPIO_Init function
	GPIO_Init(&SPIPins);

	// MISO on PB14
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	// peripheral clock enabled in GPIO_Init function
	GPIO_Init(&SPIPins);

	// NSS on PB12
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	// peripheral clock enabled in GPIO_Init function
	GPIO_Init(&SPIPins);

}
void SPI2_Inits(void)
{
	SPI_Handle_t SPI_Master;

	SPI_Master.pSPIx = SPI2;
	SPI_Master.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI_Master.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD; // full duplex
	SPI_Master.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2; // 8MHz baudrate, since APB bus running at 16MHz from HSI
	SPI_Master.SPIConfig.SPI_SSM = SPI_SSM_EN; // software slave management enabled
	SPI_Master.SPIConfig.SPI_DFF = SPI_DFF_8BITS; // 8-bit shift register for data transfer
	SPI_Master.SPIConfig.SPI_CPOL = SPI_CPOL_HIGH; // keep clock low when idle
	SPI_Master.SPIConfig.SPI_CPHA = SPI_CPHA_LOW; // capture data on first rising edge

	SPI_Init(&SPI_Master);
}

int main(void)
{
	char user_data[] = "Hello world";
	// Need to configure GPIO pins as alternate functionality for SPI2 first!
	// PB15 --> SPI2_MOSI
	// PB14 --> SPI2_MISO
	// PB13 --> SPI2_SCK
	// PB12 --> SPI2_NSS
	// ALT Functionality Mode 5 for all

	// this function is used to initialize GPIO pins as SPI2 pins
	SPI2_GPIOInits();

	// initialize all SPI peripheral settings
	SPI2_Inits();

	// If in SSM (software slave management) mode, the SSI bit controls the value of NSS
	// if SSI is 0, the NSS pin will be internally pulled low, this indicates that
	// the device is acting as a slave not a master, i.e. in like the multi-master mode config
	// this engages the Master Mode Fault, and sets the MODF bit to 1
	// this then resets the SPE and MSTR bits to 0
	// So to avoid this, we want to set SSI to 1 so we can maintain our status as a master
	SPI_SSIConfig(SPI2, ENABLE);

	// Now enable the SPI2 peripheral (SPE bit in SPI_CR1 register)
	SPI_PeripheralControl(SPI2, ENABLE);

	// Send Data
	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data)); // user_data is of char type pointer, typecast to uint8_t pointer

	// wait until the last bit is transmitted successfully before abruptly closing the peripheral
	// check to make sure BUSY (BSY) flag is low, i.e. SPI is not busy
	while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));

	SPI_PeripheralControl(SPI2, DISABLE);

	while(1);


	return 0;
}
