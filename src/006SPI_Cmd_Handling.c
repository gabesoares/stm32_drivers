/*
 * 006SPI_Cmd_Handling.c
 *
 *  Created on: Jul 12, 2019
 *      Author: gabriel
 */


#include <string.h>
#include "stm32f407xx.h"

extern void initialise_monitor_handles();

// Command codes that the Arduino understands
#define COMMAND_LED_CTRL			0x50
#define COMMAND_SENSOR_READ			0x51
#define COMMAND_LED_READ			0x52
#define COMMAND_PRINT				0x53
#define COMMAND_ID_READ				0x54

#define LED_ON						1
#define LED_OFF						0

// Arduino Analog Pins
#define ANALOG_PIN0					0
#define ANALOG_PIN1					1
#define ANALOG_PIN2					2
#define ANALOG_PIN3					3
#define ANALOG_PIN4					4
#define ANALOG_PIN5					5

// Arduino LED
#define LED_PIN						9

void delay(void)
{
	for (uint32_t i = 0; i < 500000/2; i++);
}
void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn;

	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);
}

uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{
	if (ackbyte == 0xF5)
	{
		// ack
		return 1;
	}
	else
	{
		// nack (not acknowledge)
		return 0;
	}
}
void SPI2_GPIOInits()
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltfunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
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
	SPI_Master.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; // 2MHz baudrate, since APB bus running at 16MHz from HSI
	SPI_Master.SPIConfig.SPI_SSM = SPI_SSM_DI; // software slave management disabled (hardware CS)
	SPI_Master.SPIConfig.SPI_DFF = SPI_DFF_8BITS; // 8-bit shift register for data transfer
	SPI_Master.SPIConfig.SPI_CPOL = SPI_CPOL_LOW; // keep clock low when idle
	SPI_Master.SPIConfig.SPI_CPHA = SPI_CPHA_LOW; // capture data on first rising edge

	SPI_Init(&SPI_Master);
}

int main(void)
{
	uint8_t dummywrite = 0xFF;
	uint8_t dummyread;

	initialise_monitor_handles(); // semihosting enable
	printf("Application is running\n");
	// Need to configure GPIO pins as alternate functionality for SPI2 first!
	// PB15 --> SPI2_MOSI
	// PB14 --> SPI2_MISO
	// PB13 --> SPI2_SCK
	// PB12 --> SPI2_NSS
	// ALT Functionality Mode 5 for all

	// WAIT TILL BUTTON IS PRESSED
	// this function is used to initialize GPIO pins as SPI2 pins
	SPI2_GPIOInits();
	printf("SPI Initialized\n");

	GPIO_ButtonInit();

	while(1)
	{
		while( ! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		// avoid debounce
		delay();

		// initialize all SPI peripheral settings
		SPI2_Inits();

		// If in SSM (software slave management) mode, the SSI bit controls the value of NSS
		// if SSI is 0, the NSS pin will be internally pulled low, this indicates that
		// the device is acting as a slave not a master, i.e. in like the multi-master mode config
		// this engages the Master Mode Fault, and sets the MODF bit to 1
		// this then resets the SPE and MSTR bits to 0
		// So to avoid this, we want to set SSI to 1 so we can maintain our status as a master
	//	SPI_SSIConfig(SPI2, ENABLE);

		// Enable the SSOE for the hardware to bring Chip Select (NSS) down as soon as peripheral is enabled (SPE)
		SPI_SSOEConfig(SPI2, ENABLE);

		// Now enable the SPI2 peripheral (SPE bit in SPI_CR1 register)
		SPI_PeripheralControl(SPI2, ENABLE); // when the SPI peripheral is enabled, the NSS output is automatically pulled to 0
		// i.e. when SPE bit = 1, NSS output will be 0
		// when SPE bit goes back to 0, NSS goes up to 1 (disabled)
		// SSOE (Slave Select Output Enable) bit allows this to happen (automatic NSS going down with SPE going up)
		// so need to set SSOE bit

		// 1. CMD_LED_CTRL   <pin no(1)>   <value(1)>
		uint8_t commandcode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];

		SPI_SendData(SPI2, &commandcode, 1); // 1 byte --> if the slave supports this command, it will send back an ACK
		// ^^ this transmission has resulted in 1 byte of data received as well (FULL DUPLEX MISO MOSI)
		// so need to read the buffer so that we can make sure it's cleared for when we receive the ACK/NACK in the next transmission

		// do a dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummyread, 1);

		// Now send some dummy bits (1 byte) to fetch the response from the slave.
		SPI_SendData(SPI2, &dummywrite, 1); // we will be receiving during this time as well
		SPI_ReceiveData(SPI2, &ackbyte, 1); // let's read what was in the receive buffer

		if (SPI_VerifyResponse(ackbyte))
		{
			// slave verified, now we can send arguments
			args[0] = LED_PIN;
			args[1] = LED_ON;
			// Now send this data to the slave
			SPI_SendData(SPI2, args, 2);
		}

		// Send Data

		// 2. CMD_SENSOR_READ 		<pin no(1)>

		while( ! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		// avoid debounce
		delay();

		commandcode = COMMAND_SENSOR_READ;
		SPI_SendData(SPI2, &commandcode, 1);

		// do a dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummyread, 1);

		// Now send some dummy bits (1 byte) to fetch the response from the slave.
		SPI_SendData(SPI2, &dummywrite, 1); // we will be receiving during this time as well
		SPI_ReceiveData(SPI2, &ackbyte, 1); // let's read what was in the receive buffer

		if (SPI_VerifyResponse(ackbyte))
		{
			// slave verified, now we can send arguments
			args[0] = ANALOG_PIN0;
			// Now send this data to the slave
			SPI_SendData(SPI2, &args[0], 1);
			// do a dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2, &dummyread, 1);

			// insert some delay so the arduino has time to do ADC conversion, otherwise we can't read properly
			delay();

			uint8_t analog_read;
			// Now send some dummy bits (1 byte) to fetch the response from the slave.
			SPI_SendData(SPI2, &dummywrite, 1); // we will be receiving during this time as well
			SPI_ReceiveData(SPI2, &analog_read, 1); // let's read what was in the receive buffer
			printf("COMMAND_SENSOR_READ %d\n", analog_read);
		}

		// 3. 	COMMAND_LED_READ 		<pin no(1)>

		while( ! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		// avoid debounce
		delay();

		commandcode = 	COMMAND_LED_READ;
		SPI_SendData(SPI2, &commandcode, 1);

		// do a dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummyread, 1);

		// Now send some dummy bits (1 byte) to fetch the response from the slave.
		SPI_SendData(SPI2, &dummywrite, 1); // we will be receiving during this time as well
		SPI_ReceiveData(SPI2, &ackbyte, 1); // let's read what was in the receive buffer

		if (SPI_VerifyResponse(ackbyte))
		{
			// slave verified, now we can send arguments
			args[0] = LED_PIN;
			// Now send this data to the slave
			SPI_SendData(SPI2, &args[0], 1);
			// do a dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2, &dummyread, 1);

			// insert some delay so the arduino has time to do ADC conversion, otherwise we can't read properly
			delay();

			uint8_t led_read;
			// Now send some dummy bits (1 byte) to fetch the response from the slave.
			SPI_SendData(SPI2, &dummywrite, 1); // we will be receiving during this time as well
			SPI_ReceiveData(SPI2, &led_read, 1); // let's read what was in the receive buffer
			printf("COMMAND_LED_READ %d\n", led_read);
		}

		// 4. 	COMMAND_PRINT 		<data_to_print>

		while( ! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		// avoid debounce
		delay();

		commandcode = 	COMMAND_PRINT;
		SPI_SendData(SPI2, &commandcode, 1);

		// do a dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummyread, 1);


		// Now send some dummy bits (1 byte) to fetch the response from the slave.
		SPI_SendData(SPI2, &dummywrite, 1); // we will be receiving during this time as well
		SPI_ReceiveData(SPI2, &ackbyte, 1); // let's read what was in the receive buffer

		if (SPI_VerifyResponse(ackbyte))
		{
			// slave verified, now we can send arguments
			char data[] = "Please print me!";

			// send length of data first!
			uint8_t datalength = strlen(data);
			SPI_SendData(SPI2, &datalength, 1); // 1 byte of data

			// Now send this data to the slave
			SPI_SendData(SPI2, (uint8_t*)data, strlen(data));
			// do a dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2, &dummyread, 1);
		}

		// 4. 	COMMAND_ID_READ

		while( ! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		// avoid debounce
		delay();

		commandcode = 	COMMAND_ID_READ;
		SPI_SendData(SPI2, &commandcode, 1);

		// do a dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummyread, 1);

		// Now send some dummy bits (1 byte) to fetch the response from the slave.
		SPI_SendData(SPI2, &dummywrite, 1); // we will be receiving during this time as well
		SPI_ReceiveData(SPI2, &ackbyte, 1); // let's read what was in the receive buffer

		if (SPI_VerifyResponse(ackbyte))
		{
			uint8_t arduino_id[10];
			// Dummy write to get the ID back from the arduino
			for (int i = 0; i < 10; i++)
			{
				SPI_SendData(SPI2, &dummywrite, 1);
				// read the ID
				SPI_ReceiveData(SPI2, &arduino_id[i], 1);
			}


			printf("COMMAND_ID : %s \n", arduino_id);


		}


		// wait until the last bit is transmitted successfully before abruptly closing the peripheral
		// check to make sure BUSY (BSY) flag is low, i.e. SPI is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));

		SPI_PeripheralControl(SPI2, DISABLE);
		printf("SPI Communication Closed\n");
	}


	return 0;
}
