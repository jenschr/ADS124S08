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
 * ADS124S08.h
 *
 */
#ifndef ADS124S08_H_
#define ADS124S08_H_

#include <SPI.h>

/* Start definitions */
#define NUM_REGISTERS 18
/*
 * Address masks used for register addressing with
 * either a REGRD of REGWR mask
 *
 */
#define ID_ADDR_MASK 0x00
#define STATUS_ADDR_MASK 0x01
#define INPMUX_ADDR_MASK 0x02
#define PGA_ADDR_MASK 0x03
#define DATARATE_ADDR_MASK 0x04
#define REF_ADDR_MASK 0x05
#define IDACMAG_ADDR_MASK 0x06
#define IDACMUX_ADDR_MASK 0x07
#define VBIAS_ADDR_MASK 0x08
#define SYS_ADDR_MASK 0x09
#define OFCAL0_ADDR_MASK 0x0A
#define OFCAL1_ADDR_MASK 0x0B
#define OFCAL2_ADDR_MASK 0x0C
#define FSCAL0_ADDR_MASK 0x0D
#define FSCAL1_ADDR_MASK 0x0E
#define FSCAL2_ADDR_MASK 0x0F
#define GPIODAT_ADDR_MASK 0x10
#define GPIOCON_ADDR_MASK 0x11

/* Opcode masks (or "Commands" if you will...) */
#define NOP_OPCODE_MASK 0x00
#define WAKE_OPCODE_MASK 0x02
#define SLEEP_OPCODE_MASK 0x04
#define RESET_OPCODE_MASK 0x06
#define START_OPCODE_MASK 0x08
#define STOP_OPCODE_MASK 0x0A
#define SFOCAL_OPCODE_MASK 0x19
#define SYOCAL_OPCODE_MASK 0x16
#define SYGCAL_OPCODE_MASK 0x17
#define RDATA_OPCODE_MASK 0x12
#define REGRD_OPCODE_MASK 0x20
#define REGWR_OPCODE_MASK 0x40

/* Register sub masks */
/* ADS124S08 Register 0 (ID) Definition
 *   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
 *--------------------------------------------------------------------------------------------
 *        			RESERVED[4:0]      			 			 |  	      DEV_ID[2:0]
 *
 */
/* Define ID (revision) */
#define ADS_ID_A 0x00
#define ADS_ID_B 0x80
/* Define VER (device version) */
#define ADS_124S08 0x00
#define ADS_124S06 0x01
#define ADS_114S08 0x04
#define ADS_114S06 0x05
/* ADS124S08 Register 1 (STATUS) Definition */
/*   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
 *--------------------------------------------------------------------------------------------
 *   FL_POR  |    nRDY   | FL_P_RAILP| FL_P_RAILN| FL_N_RAILP| FL_N_RAILN| FL_REF_L1 | FL_REF_L0
 *
 */
#define ADS_FL_POR 0x80
#define ADS_RDY 0x40
#define ADS_FL_P_RAILP 0x20
#define ADS_FL_P_RAILN 0x10
#define ADS_FL_N_RAILP 0x08
#define ADS_FL_N_RAILN 0x04
#define ADS_FL_REF_L1 0x02
#define ADS_FL_REF_L0 0x10
/* ADS124S08 Register 2 (INPMUX) Definition */
/*   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
 *--------------------------------------------------------------------------------------------
 *         			MUXP[3:0]   				 |       			MUXN[3:0]
 *
 */
/* Define the ADC positive input channels (MUXP) */
#define ADS_P_AIN0 0x00
#define ADS_P_AIN1 0x10
#define ADS_P_AIN2 0x20
#define ADS_P_AIN3 0x30
#define ADS_P_AIN4 0x40
#define ADS_P_AIN5 0x50
#define ADS_P_AIN6 0x60
#define ADS_P_AIN7 0x70
#define ADS_P_AIN8 0x80
#define ADS_P_AIN9 0x90
#define ADS_P_AIN10 0xA0
#define ADS_P_AIN11 0xB0
#define ADS_P_AINCOM 0xC0
/* Define the ADC negative input channels (MUXN)*/
#define ADS_N_AIN0 0x00
#define ADS_N_AIN1 0x01
#define ADS_N_AIN2 0x02
#define ADS_N_AIN3 0x03
#define ADS_N_AIN4 0x04
#define ADS_N_AIN5 0x05
#define ADS_N_AIN6 0x06
#define ADS_N_AIN7 0x07
#define ADS_N_AIN8 0x08
#define ADS_N_AIN9 0x09
#define ADS_N_AIN10 0x0A
#define ADS_N_AIN11 0x0B
#define ADS_N_AINCOM 0x0C
/* ADS124S08 Register 3 (PGA) Definition */
/*   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
 *--------------------------------------------------------------------------------------------
 *   		DELAY[2:0]		     	 |      PGA_EN[1:0]      |              GAIN[2:0]
 *
 */
/* Define conversion delay in tmod clock periods */
#define ADS_DELAY_14 0x00
#define ADS_DELAY_25 0x20
#define ADS_DELAY_64 0x40
#define ADS_DELAY_256 0x60
#define ADS_DELAY_1024 0x80
#define ADS_DELAY_2048 0xA0
#define ADS_DELAY_4096 0xC0
#define ADS_DELAY_1 0xE0
/* Define PGA control */
#define ADS_PGA_BYPASS 0x00
#define ADS_PGA_ENABLED 0x08
/* Define Gain */
#define ADS_GAIN_1 0x00
#define ADS_GAIN_2 0x01
#define ADS_GAIN_4 0x02
#define ADS_GAIN_8 0x03
#define ADS_GAIN_16 0x04
#define ADS_GAIN_32 0x05
#define ADS_GAIN_64 0x06
#define ADS_GAIN_128 0x07
/* ADS124S08 Register 4 (DATARATE) Definition */
/*   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
 *--------------------------------------------------------------------------------------------
 *   G_CHOP  |    CLK    |    MODE   |   FILTER  | 				  DR[3:0]
 *
 */
#define ADS_GLOBALCHOP 0x80
#define ADS_CLKSEL_EXT 0x40
#define ADS_CONVMODE_SS 0x20
#define ADS_FILTERTYPE_LL 0x10
/* Define the data rate */
#define ADS_DR_2_5 0x00
#define ADS_DR_5 0x01
#define ADS_DR_10 0x02
#define ADS_DR_16 0x03
#define ADS_DR_20 0x04
#define ADS_DR_50 0x05
#define ADS_DR_60 0x06
#define ADS_DR_100 0x07
#define ADS_DR_200 0x08
#define ADS_DR_400 0x09
#define ADS_DR_800 0x0A
#define ADS_DR_1000 0x0B
#define ADS_DR_2000 0x0C
#define ADS_DR_4000 0x0D
/* ADS124S08 Register 5 (REF) Definition */
/*   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
 *--------------------------------------------------------------------------------------------
 *	  FL_REF_EN[1:0]	 | nREFP_BUF | nREFN_BUF | 		REFSEL[1:0]		 | 		REFCON[1:0]
 *
 */
#define ADS_FLAG_REF_DISABLE 0x00
#define ADS_FLAG_REF_EN_L0 0x40
#define ADS_FLAG_REF_EN_BOTH 0x80
#define ADS_FLAG_REF_EN_10M 0xC0
#define ADS_REFP_BYP_DISABLE 0x20
#define ADS_REFP_BYP_ENABLE 0x00
#define ADS_REFN_BYP_DISABLE 0x10
#define ADS_REFN_BYP_ENABLE 0x00
#define ADS_REFSEL_P0 0x00
#define ADS_REFSEL_P1 0x04
#define ADS_REFSEL_INT 0x08
#define ADS_REFINT_OFF 0x00
#define ADS_REFINT_ON_PDWN 0x01
#define ADS_REFINT_ON_ALWAYS 0x02
/* ADS124S08 Register 6 (IDACMAG) Definition */
/*   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
 *--------------------------------------------------------------------------------------------
 * FL_RAIL_EN|	  PSW	 |     0     | 		0	 | 			    	IMAG[3:0]
 *
 */
#define ADS_FLAG_RAIL_ENABLE 0x80
#define ADS_FLAG_RAIL_DISABLE 0x00
#define ADS_PSW_OPEN 0x00
#define ADS_PSW_CLOSED 0x40
#define ADS_IDACMAG_OFF 0x00
#define ADS_IDACMAG_10 0x01
#define ADS_IDACMAG_50 0x02
#define ADS_IDACMAG_100 0x03
#define ADS_IDACMAG_250 0x04
#define ADS_IDACMAG_500 0x05
#define ADS_IDACMAG_750 0x06
#define ADS_IDACMAG_1000 0x07
#define ADS_IDACMAG_1500 0x08
#define ADS_IDACMAG_2000 0x09
/* ADS124S08 Register 7 (IDACMUX) Definition */
/*   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
 *--------------------------------------------------------------------------------------------
 *                    I2MUX[3:0]                 |                   I1MUX[3:0]
 *
 */
/* Define IDAC2 Output */
#define ADS_IDAC2_A0 0x00
#define ADS_IDAC2_A1 0x10
#define ADS_IDAC2_A2 0x20
#define ADS_IDAC2_A3 0x30
#define ADS_IDAC2_A4 0x40
#define ADS_IDAC2_A5 0x50
#define ADS_IDAC2_A6 0x60
#define ADS_IDAC2_A7 0x70
#define ADS_IDAC2_A8 0x80
#define ADS_IDAC2_A9 0x90
#define ADS_IDAC2_A10 0xA0
#define ADS_IDAC2_A11 0xB0
#define ADS_IDAC2_AINCOM 0xC0
#define ADS_IDAC2_OFF 0xF0
/* Define IDAC1 Output */
#define ADS_IDAC1_A0 0x00
#define ADS_IDAC1_A1 0x01
#define ADS_IDAC1_A2 0x02
#define ADS_IDAC1_A3 0x03
#define ADS_IDAC1_A4 0x04
#define ADS_IDAC1_A5 0x05
#define ADS_IDAC1_A6 0x06
#define ADS_IDAC1_A7 0x07
#define ADS_IDAC1_A8 0x08
#define ADS_IDAC1_A9 0x09
#define ADS_IDAC1_A10 0x0A
#define ADS_IDAC1_A11 0x0B
#define ADS_IDAC1_AINCOM 0x0C
#define ADS_IDAC1_OFF 0x0F
/* ADS124S08 Register 8 (VBIAS) Definition */
/*   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
 *--------------------------------------------------------------------------------------------
 *  VB_LEVEL | 	VB_AINC  |  VB_AIN5  |  VB_AIN4  |  VB_AIN3  |  VB_AIN2  |  VB_AIN1  |  VB_AIN0
 *
 */
#define ADS_VBIAS_LVL_DIV2 0x00
#define ADS_VBIAS_LVL_DIV12 0x80
/* Define VBIAS here */
#define ADS_VB_AINC 0x40
#define ADS_VB_AIN5 0x20
#define ADS_VB_AIN4 0x10
#define ADS_VB_AIN3 0x08
#define ADS_VB_AIN2 0x04
#define ADS_VB_AIN1 0x02
#define ADS_VB_AIN0 0x01
/* ADS124S08 Register 9 (SYS) Definition */
/*   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
 *--------------------------------------------------------------------------------------------
 *			   SYS_MON[2:0]			 |	   CAL_SAMP[1:0]     |  TIMEOUT  | 	  CRC	 | SENDSTAT
 *
 */
#define ADS_SYS_MON_OFF 0x00
#define ADS_SYS_MON_SHORT 0x20
#define ADS_SYS_MON_TEMP 0x40
#define ADS_SYS_MON_ADIV4 0x60
#define ADS_SYS_MON_DDIV4 0x80
#define ADS_SYS_MON_BCS_2 0xA0
#define ADS_SYS_MON_BCS_1 0xC0
#define ADS_SYS_MON_BCS_10 0xE0
#define ADS_CALSAMPLE_1 0x00
#define ADS_CALSAMPLE_4 0x08
#define ADS_CALSAMPLE_8 0x10
#define ADS_CALSAMPLE_16 0x18
#define ADS_TIMEOUT_DISABLE 0x00
#define ADS_TIMEOUT_ENABLE 0x04
#define ADS_CRC_DISABLE 0x00
#define ADS_CRC_ENABLE 0x02
#define ADS_SENDSTATUS_DISABLE 0x00
#define ADS_SENDSTATUS_ENABLE 0x01
/* ADS124S08 Register A (OFCAL0) Definition */
/*   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
 *--------------------------------------------------------------------------------------------
 *                                         OFC[7:0]
 *
 */
/* ADS124S08 Register B (OFCAL1) Definition */
/*   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
 *--------------------------------------------------------------------------------------------
 *                                         OFC[15:8]
 *
 */
/* ADS124S08 Register C (OFCAL2) Definition */
/*   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
 *--------------------------------------------------------------------------------------------
 *                                         OFC[23:16]
 *
 */
/* ADS124S08 Register D (FSCAL0) Definition */
/*   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
 *--------------------------------------------------------------------------------------------
 *                                         FSC[7:0]
 *
 */
/* ADS124S08 Register E (FSCAL1) Definition */
/*   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
 *--------------------------------------------------------------------------------------------
 *                                         FSC[15:8]
 *
 */
/* ADS124S08 Register F (FSCAL2) Definition */
/*   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
 *--------------------------------------------------------------------------------------------
 *                                         FSC[23:16]
 *
 */
/* ADS124S08 Register 10 (GPIODAT) Definition */
/*   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
 *--------------------------------------------------------------------------------------------
 *                      DIR[3:0]    			 | 					DAT[3:0]
 *
 */
/* Define GPIO direction (0-Output; 1-Input) here */
#define ADS_GPIO0_DIR_INPUT 0x10
#define ADS_GPIO1_DIR_INPUT 0x20
#define ADS_GPIO2_DIR_INPUT 0x40
#define ADS_GPIO3_DIR_INPUT 0x80
/*
 *
 */
/* Define GPIO data here */
/*
 *
 */
/* ADS124S08 Register 11 (GPIOCON) Definition */
/*   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
 *--------------------------------------------------------------------------------------------
 *     0	 |	   0	 |	   0	 |	   0     |                    CON[3:0]
 *
 */
/* Define GPIO configuration (0-Analog Input; 1-GPIO) here */
#define ADS_GPIO0_DIR_INPUT 0x10
#define ADS_GPIO1_DIR_INPUT 0x20
#define ADS_GPIO2_DIR_INPUT 0x40
#define ADS_GPIO3_DIR_INPUT 0x80
/*
 *
 */
/* Lengths of conversion data components */
#define DATA_LENGTH 3
#define STATUS_LENGTH 1
#define CRC_LENGTH 1
/*
 * The time we have to wait for the CS GPIO to actually
 * pull down before we start sending SCLKs
 *
 */
#define CHIP_SELECT_WAIT_TIME 0

/* Flag to signal that we are in the process of collecting data */
#define DATA_MODE_NORMAL 0x00
#define DATA_MODE_STATUS 0x01
#define DATA_MODE_CRC 0x02

/* The clock rate of the internal XTAL... */
#define DEVICE_ICLK 16384000

/* Set the SPI SCLK speed */
#define SPI_SPEED 5000000
/*
 * The TIVA has a 4 to 16 bit word size, to handle  32 bit words the TIVA needs to assemble
 * two groups of words together.  32 bits comprises of 2x16 words (or 4x8 words).
*/
#define SPI_WORD_SIZE 8

/* Chip select feedthrough GPIO - we hand feed chip select so overages may be pumped out. */
#define CS_PIN PB12
#define DRDY_PIN PD13
/* GPIO definitions. */
#define START_PIN PD14
#define RESET_PIN PD15
#define CKEN_PIN -1

#define CAN_LED1 PC4

#define SPI_HAS_TRANSACTION 1

#if defined(SPI_HAS_TRANSACTION)
static SPISettings mySPISettings;
#endif

class ADS124S08
{
	// Device command prototypes
public:
	ADS124S08(void);
	void begin();
	void SPI_Init(uint32_t SPI_speed, uint8_t chipSelect_Pin);
	void SPI_Write_16(uint16_t Address, uint16_t Data);
	void SPI_Write_32(uint16_t Address, uint32_t Data);
	uint16_t SPI_Read_16(uint16_t Address);
	uint32_t SPI_Read_32(uint16_t Address);
	char regRead(unsigned int regnum);
	void readRegs(unsigned int regnum, unsigned int count, uint8_t *data);
	void regWrite(unsigned int regnum, unsigned char data);
	void writeRegs(unsigned int regnum, unsigned int howmuch, unsigned char *data);
	void reStart(void);
	void sendCommand(uint8_t op_code);
	int rData(uint8_t *dStatus, uint8_t *dData, uint8_t *dCRC);
	int dataRead(uint8_t *dStatus, uint8_t *dData, uint8_t *dCRC);
	void selectDeviceCSLow(void);
	void releaseChipSelect(void);
	void assertStart(void);
	void deassertStart(void);
	void assertClock(void);
	void deassertClock(void);
	bool converting;
	uint8_t registers[NUM_REGISTERS];

 

private:
	uint8_t _chipSelect_Pin;
	bool fStart;
	void DRDY_int(void);
	uint8_t _drdy_pin;
	uint8_t _start_pin;
	uint8_t _reset_pin;
};

#endif /* ADS124S08_H_ */
