/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


/*  Standard C Included Files */
#include <stdio.h>
#include <string.h>

/*  SDK Included Files */
#include "main.h"
#include "rgb_lcd.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
// FlexIO for LCD
#define BOARD_FLEXIO_BASE FLEXIO0               // Defines the FlexIO instance to be used
#define FLEXIO_I2C_SDA_PIN (17U)                // Defines the FlexIO pin to be used for the SDA pin
#define FLEXIO_I2C_SCL_PIN (16U)                // Defines the FlexIO pin to be used for the SCL pin
#define FLEXIO_CLOCK_FREQUENCY (12000000U)      // Defines the clock frequency used in the application

#define I2C_BAUDRATE (100000) /* 100K */        // Defines the I2C baud rate


#define LCD_ADDRESS     (0x7c>>1)
#define RGB_ADDRESS     (0xc4>>1)

#define CLK_SPEED 150U //Put your board's clock speed here in MHz, FRDM-K82f = 150

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static bool I2C_write_lcd(FLEXIO_I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t value);

void delay(int time);
void rgb_lcd_init();
void rgb_lcd_write(char *data);
void rgb_led_init();
void rgb_lcd_clear();
void rgb_lcd_set_cursor(uint8_t col, uint8_t row);

/*******************************************************************************
 * Variables
 ******************************************************************************/

uint8_t readBuff[2];
uint8_t g_master_buff[3];
flexio_i2c_master_handle_t g_m_handle;
FLEXIO_I2C_Type i2cDev;
uint32_t gStateBufferPos[8] = {0};
uint8_t status0_value = 0;

volatile bool completionFlag = false;
volatile bool nakFlag = false;

extern volatile uint8_t fxioState;

uint8_t command_reg = 0;
uint8_t data_reg = 0;
uint8_t command = 0;

/*******************************************************************************
 * Code
 ******************************************************************************/

static void flexio_i2c_master_callback(FLEXIO_I2C_Type *base,
                                       flexio_i2c_master_handle_t *handle,
                                       status_t status,
                                       void *userData)
{
    /* Signal transfer success when received success status. */
    if (status == kStatus_Success)
    {
        completionFlag = true;
    }
    /* Signal transfer success when received success status. */
    if (status == kStatus_FLEXIO_I2C_Nak)
    {
        nakFlag = true;
    }
}


static bool I2C_write_lcd(FLEXIO_I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t value)
{
    flexio_i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress = device_addr;
    masterXfer.direction = kFLEXIO_I2C_Write;
    masterXfer.subaddress = reg_addr;
    masterXfer.subaddressSize = 1;
    masterXfer.data = &value;
    masterXfer.dataSize = 1;

    /*  direction=write : start+device_write;cmdbuff;xBuff; */
    /*  direction=recive : start+device_write;cmdbuff;repeatStart+device_read;xBuff; */

    FLEXIO_I2C_MasterTransferNonBlocking(&i2cDev, &g_m_handle, &masterXfer);

    /*  Wait for transfer completed. */
    while ((!nakFlag) && (!completionFlag))
    {
    }

    nakFlag = false;

    if (completionFlag == true)
    {
        completionFlag = false;
        return true;
    }
    else
    {
        return false;
    }
}


void delay(int time)
{
  uint32_t delayNum = (CLK_SPEED / 15) * time; //as clock speed in MHz, delay is in us
  volatile uint32_t cnt = 0U;
  for(cnt = 0U; cnt < delayNum; ++cnt){
	  __asm("NOP");
  }
}


void rgb_lcd_init()
{
	// SEE PAGE 45/46 of LCD Datasheet FOR INITIALIZATION SPECIFICATION!
	// according to datasheet, we need at least 40ms after power rises above 2.7V
	// before sending commands. Freedom board can turn on way befer 4.5V so we'll wait 50
	delay(50000);  //50ms

	// this is according to the hitachi HD44780 datasheet
	// page 45 figure 23

	// Send function set command sequence
	command_reg =0x80;
	command = 0x28;
	I2C_write_lcd(&i2cDev, LCD_ADDRESS, command_reg, command);
	delay(4500);  //4.5 ms

	// second try
	I2C_write_lcd(&i2cDev, LCD_ADDRESS, command_reg, command);
	delay(150);

	// third go
	I2C_write_lcd(&i2cDev, LCD_ADDRESS, command_reg, command);

	command = 0x0C;  //LCD_DISPLAYON , LCD_CURSOROFF , LCD_BLINKOFF;
	I2C_write_lcd(&i2cDev, LCD_ADDRESS, command_reg, command);

	command = 0x01;  // clear it off
	I2C_write_lcd(&i2cDev, LCD_ADDRESS, command_reg, command);
	delay(2000);    // this command takes long time

	command = 0x0C;  // Initialize to default text direction
	I2C_write_lcd(&i2cDev, LCD_ADDRESS, command_reg, command);

}


void rgb_lcd_clear()
{
	command_reg =0x80;
	command = 0x01;  // clear it off
	I2C_write_lcd(&i2cDev, LCD_ADDRESS, command_reg, command);
	delay(2000);    // this command takes long time
}


void rgb_lcd_set_cursor(uint8_t col, uint8_t row)
{

    command = (row == 0 ? col|0x80 : col|0xc0);
    command_reg =0x80;
    I2C_write_lcd(&i2cDev, LCD_ADDRESS, command_reg, command);

}


void rgb_lcd_write(char *data)
{
	 uint32_t i = 0;
	 data_reg = 0x40;
	 for(i=0;i<strlen(data);i++)
	 {
	 	I2C_write_lcd(&i2cDev, LCD_ADDRESS, data_reg, data[i]);
	 }

}


void rgb_led_init()
{
	//Reset
	 command_reg = REG_MODE1;
	 command = 0x00;
	 I2C_write_lcd(&i2cDev, RGB_ADDRESS, command_reg, command);

	 /*  Function set */
	 command_reg = REG_OUTPUT;
	 command = 0xFF;
	 I2C_write_lcd(&i2cDev, RGB_ADDRESS, command_reg, command);

	 command_reg = REG_MODE2;
	 command = 0x20;
	 I2C_write_lcd(&i2cDev, RGB_ADDRESS, command_reg, command);

	 /* Color Set */
	 command_reg = REG_GREEN;
	 command = 0xFF;          //amount of green 0 - 255
	 I2C_write_lcd(&i2cDev, RGB_ADDRESS, command_reg, command);

	 command_reg = REG_RED;
	 command = 0xFF;          //amount of red 0 - 255
	 I2C_write_lcd(&i2cDev, RGB_ADDRESS, command_reg, command);

	 command_reg = REG_BLUE;
	 command = 0xFF;          //amount of blue 0 - 255
	 I2C_write_lcd(&i2cDev, RGB_ADDRESS, command_reg, command);
}



/*!
 * @brief Main function
 */


int main(void)
{

	BOARD_InitPins();
	BOARD_BootClockRUN();
	BOARD_InitDebugConsole();


    flexio_i2c_master_config_t masterConfig;
    CLOCK_SetFlexio0Clock(2U);


    /*do hardware configuration*/
    i2cDev.flexioBase = BOARD_FLEXIO_BASE;
    i2cDev.SDAPinIndex = FLEXIO_I2C_SDA_PIN;
    i2cDev.SCLPinIndex = FLEXIO_I2C_SCL_PIN;
    i2cDev.shifterIndex[0] = 0U;
    i2cDev.shifterIndex[1] = 1U;
    i2cDev.timerIndex[0] = 0U;
    i2cDev.timerIndex[1] = 1U;

    /*
     * masterConfig.enableMaster = true;
     * masterConfig.enableInDoze = false;
     * masterConfig.enableInDebug = true;
     * masterConfig.enableFastAccess = false;
     * masterConfig.baudRate_Bps = 100000U;
     */
    FLEXIO_I2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Bps = I2C_BAUDRATE;

    /* Initialize FlexIO I2C master and handle for interrupt */
    FLEXIO_I2C_MasterInit(&i2cDev, &masterConfig, FLEXIO_CLOCK_FREQUENCY);
    FLEXIO_I2C_MasterTransferCreateHandle(&i2cDev, &g_m_handle, flexio_i2c_master_callback, NULL);

    // lcd initialization
    rgb_lcd_init();
    rgb_led_init();

	rgb_lcd_write("Hello World!");
	delay(10000000);
	rgb_lcd_set_cursor(5,1);
	rgb_lcd_write("Allahu");
	delay(1000);
    rgb_lcd_clear();

    for(;;) { /* Infinite loop to avoid leaving the main function */
        __asm("NOP"); /* something to use as a breakpoint stop while looping */
      }
}

