/* --COPYRIGHT--,BSD
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

/*
 * This class is based on the ADS124S08.c provided by TI and adapted for use with
 * Particle Photon and other Arduino-like MCU's.
 *
 * Note that the confusingly named "selectDeviceCSLow"
 * actually selects the device and that "releaseChipSelect" deselects it. Not obvious
 * from the naming, but according to the datasheet CS is active LOW.
 */

#include "ADS124S08.h"

/*
 * Writes the nCS pin low and waits a while for the device to finish working before
 * handing control back to the caller for a SPI transfer.
 */
void ADS124S08::selectDeviceCSLow(void){
	digitalWrite( CS_PIN, LOW );
}

/*
 * Pulls the nCS pin high. Performs no waiting.
 */
void ADS124S08::releaseChipSelect(void){
	digitalWrite( CS_PIN, HIGH );
}

/*
 * Initializes device for use in the ADS124S08 EVM.
 *
 * \return True if device is in correct hardware defaults and is connected
 *
 */
ADS124S08::ADS124S08(void)
{
	pinMode( CS_PIN, OUTPUT );
	pinMode( START_PIN, OUTPUT );
	pinMode( RESET_PIN, OUTPUT );
	pinMode( CKEN_PIN, OUTPUT );
	pinMode( DRDY_PIN, INPUT );

	digitalWrite( START_PIN, LOW );
	digitalWrite( RESET_PIN, HIGH );
	digitalWrite( CKEN_PIN, LOW );

	/* Default register settings */
	registers[ID_ADDR_MASK] 			= 0x08;
	registers[STATUS_ADDR_MASK] 	= 0x80;
	registers[INPMUX_ADDR_MASK]		= 0x01;
	registers[PGA_ADDR_MASK] 			= 0x00;
	registers[DATARATE_ADDR_MASK] = 0x14;
	registers[REF_ADDR_MASK] 			= 0x10;
	registers[IDACMAG_ADDR_MASK] 	= 0x00;
	registers[IDACMUX_ADDR_MASK] 	= 0xFF;
	registers[VBIAS_ADDR_MASK] 		= 0x00;
	registers[SYS_ADDR_MASK]			= 0x10;
	registers[OFCAL0_ADDR_MASK] 	= 0x00;
	registers[OFCAL1_ADDR_MASK] 	= 0x00;
	registers[OFCAL2_ADDR_MASK] 	= 0x00;
	registers[FSCAL0_ADDR_MASK] 	= 0x00;
	registers[FSCAL1_ADDR_MASK] 	= 0x00;
	registers[FSCAL2_ADDR_MASK] 	= 0x40;
	registers[GPIODAT_ADDR_MASK] 	= 0x00;
	registers[GPIOCON_ADDR_MASK]	= 0x00;
	fStart = false;
	releaseChipSelect();
	deassertStart();
}

void ADS124S08::begin()
{
	SPI.begin( CS_PIN );
	SPI.setBitOrder(MSBFIRST);
	SPI.setDataMode( SPI_MODE1 );
	SPI.setClockSpeed( 1000000 );
	#if defined (SPI_HAS_TRANSACTION)
		mySPISettings = SPISettings(1000000, MSBFIRST, SPI_MODE1);
	#endif
}

/*
 * Reads a single register contents from the specified address
 *
 * \param regnum identifies which address to read
 *
 */
char ADS124S08::regRead(unsigned int regnum)
{
	int i;
  uint8_t ulDataTx[3];
  uint8_t ulDataRx[3];
	ulDataTx[0] = REGRD_OPCODE_MASK + (regnum & 0x1f);
	ulDataTx[1] = 0x00;
	ulDataTx[2] = 0x00;
	selectDeviceCSLow();

	#if defined (SPI_HAS_TRANSACTION)
		SPI.beginTransaction(mySPISettings);
	#endif

	for(i = 0; i < 3; i++)
		ulDataRx[i] = SPI.transfer(ulDataTx[i]);
	if(regnum < NUM_REGISTERS)
			registers[regnum] = ulDataRx[2];

	#if defined (SPI_HAS_TRANSACTION)
		SPI.endTransaction();
	#endif

	releaseChipSelect();
	//Serial.printlnf("regRead tx: %02x %02x %02x",ulDataTx[0],ulDataTx[1],ulDataTx[2]);
	//Serial.printlnf("regRead rx: %02x %02x %02x",ulDataRx[0],ulDataRx[1],ulDataRx[2]);
	return ulDataRx[2];
}

/*
 * Reads a group of registers starting at the specified address
 *
 * \param regnum is addr_mask 8-bit mask of the register from which we start reading
 * \param count The number of registers we wish to read
 * \param *location pointer to the location in memory to write the data
 *
 */
void ADS124S08::readRegs(unsigned int regnum, unsigned int count, uint8_t *data)
{
	int i;
  uint8_t ulDataTx[2];
	ulDataTx[0] = REGRD_OPCODE_MASK + (regnum & 0x1f);
	ulDataTx[1] = count-1;
	selectDeviceCSLow();
	SPI.transfer(ulDataTx[0]);
	SPI.transfer(ulDataTx[1]);
	for(i = 0; i < count; i++)
	{
		data[i] = SPI.transfer(0);
		if(regnum+i < NUM_REGISTERS)
			registers[regnum+i] = data[i];
	}
	releaseChipSelect();
}

/*
 * Writes a single of register with the specified data
 *
 * \param regnum addr_mask 8-bit mask of the register to which we start writing
 * \param data to be written
 *
 */
void ADS124S08::regWrite(unsigned int regnum, unsigned char data)
{
	uint8_t ulDataTx[3];
	ulDataTx[0] = REGWR_OPCODE_MASK + (regnum & 0x1f);
	ulDataTx[1] = 0x00;
	ulDataTx[2] = data;
	selectDeviceCSLow();
	SPI.transfer(ulDataTx[0]);
	SPI.transfer(ulDataTx[1]);
	SPI.transfer(ulDataTx[2]);
	releaseChipSelect();
	//Serial.printlnf("regWrite tx: %02x %02x %02x",ulDataTx[0],ulDataTx[1],ulDataTx[2]);
	return;
}

/*
 * Writes a group of registers starting at the specified address
 *
 * \param regnum is addr_mask 8-bit mask of the register from which we start writing
 * \param count The number of registers we wish to write
 * \param *location pointer to the location in memory to read the data
 *
 */
void ADS124S08::writeRegs(unsigned int regnum, unsigned int howmuch, unsigned char *data)
{
	unsigned int i;
  uint8_t ulDataTx[2];
	ulDataTx[0] = REGWR_OPCODE_MASK + (regnum & 0x1f);
	ulDataTx[1] = howmuch-1;
	selectDeviceCSLow();
	SPI.transfer(ulDataTx[0]);
	SPI.transfer(ulDataTx[1]);
	for(i=0; i < howmuch; i++)
	{
		SPI.transfer( data[i] );
		if(regnum+i < NUM_REGISTERS)
			registers[regnum+i] = data[i];
	}
	releaseChipSelect();
	return;
}

/*
 * Sends a command to the ADS124S08
 *
 * \param op_code is the command being issued
 *
 */
void ADS124S08::sendCommand(uint8_t op_code)
{
	selectDeviceCSLow();
	SPI.transfer(op_code);
	releaseChipSelect();
	return;
}

/*
 * Sends a STOP/START command sequence to the ADS124S08 to restart conversions (SYNC)
 *
 */
void ADS124S08::reStart(void)
{
	sendCommand(STOP_OPCODE_MASK);
	sendCommand(START_OPCODE_MASK);
	return;
}

/*
 * Sets the GPIO hardware START pin high (red LED)
 *
 */
void ADS124S08::assertStart()
{
	fStart = true;
	digitalWrite(START_PIN ,HIGH);
}

/*
 * Sets the GPIO hardware START pin low
 *
 */
void ADS124S08::deassertStart()
{
	fStart = false;
	digitalWrite(START_PIN, LOW);
}

/*
 * Sets the GPIO hardware external oscillator enable pin high
 *
 */
void ADS124S08::assertClock()
{
	digitalWrite(CKEN_PIN, 1);
}

/*
 * Sets the GPIO hardware external oscillator enable pin low
 *
 */
void ADS124S08::deassertClock()
{
	digitalWrite(CKEN_PIN, LOW);
}

int ADS124S08::rData(uint8_t *dStatus, uint8_t *dData, uint8_t *dCRC)
{
	int result = -1;
	selectDeviceCSLow();

	// according to datasheet chapter 9.5.4.2 Read Data by RDATA Command
	sendCommand(RDATA_OPCODE_MASK);

	// if the Status byte is set - grab it
	uint8_t shouldWeReceiveTheStatusByte = (registers[SYS_ADDR_MASK] & 0x01) == DATA_MODE_STATUS;
	if( shouldWeReceiveTheStatusByte )
	{
		dStatus[0] = SPI.transfer(0x00);
		//Serial.print("status: ");
		//Serial.print(dStatus[0]);
	}

	// get the conversion data (3 bytes)
	uint8_t data[3];
	data[0] = SPI.transfer(0x00);
	data[1] = SPI.transfer(0x00);
	data[2] = SPI.transfer(0x00);
	result = data[0];
	result = (result<<8) + data[1];
	result = (result<<8) + data[2];
	//Serial.printlnf(" 1: %02x 2: %02x, 3: %02x = %d", data[0], data[1], data[2], result);

	// is CRC enabled?
	uint8_t isCrcEnabled = (registers[SYS_ADDR_MASK] & 0x02) == DATA_MODE_CRC;
	if( isCrcEnabled )
	{
		dCRC[0] = SPI.transfer(0x00);
	}

	releaseChipSelect();
	return result;
}

/*
 *
 * Read the last conversion result
 *
 */
int ADS124S08::dataRead(uint8_t *dStatus, uint8_t *dData, uint8_t *dCRC)
{
	uint8_t xcrc;
	uint8_t xstatus;
	int iData;
	selectDeviceCSLow();
	if((registers[SYS_ADDR_MASK] & 0x01) == DATA_MODE_STATUS)
	{
		xstatus = SPI.transfer(0x00);
		//Serial.print("0:");
		//Serial.print(xstatus);
		dStatus[0] = (uint8_t)xstatus;
	}

	// get the conversion data (3 bytes)
	uint8_t data[3];
	data[0] = SPI.transfer(0x00);
	data[1] = SPI.transfer(0x00);
	data[2] = SPI.transfer(0x00);

	/*
	Serial.print(" 1:");
	Serial.print(data[0]);
	Serial.print(" 2:");
	Serial.print(data[1]);
	Serial.print(" 3:");
	Serial.println(data[2]);
	*/

	iData = data[0];
	iData = (iData<<8) + data[1];
	iData = (iData<<8) + data[2];
	if((registers[SYS_ADDR_MASK] & 0x02) == DATA_MODE_CRC)
	{
		xcrc = SPI.transfer(0x00);
		dCRC[0] = (uint8_t)xcrc;
	}
	releaseChipSelect();
	return iData ;
}
