/*
* Copyright (c) 2015, Freescale Semiconductor, Inc.
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
#include <math.h>
#include "main.h"
/*  SDK Included Files */
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_flexio_i2c_master.h"
#include "clock_config.h"
#include "pin_mux.h"
#include "rgb_lcd.h"
#include "fsl_flexio.h"
#include "fsl_adc16.h"
#include "fsl_gpio.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define BOARD_FLEXIO_BASE FLEXIO0   //flexio0
//accelerometer SDA pin is connected to PTA1 which is FLEXIO_0 D11 and Arduino SDA
#define FLEXIO_I2C_SDA_PIN (11U)
//accelerometer SCL pin is connected to PTA2 which is FLEXIO_0 D12 and Arduno SCL
#define FLEXIO_I2C_SCL_PIN (12U)
//FLEXIO clock frequency 12MHz
#define FLEXIO_CLOCK_FREQUENCY 12000000U
#define I2C_BAUDRATE (100000) /* 100K */
//built-in accelerometer FOXS8700 I2C address
#define ACCEL_STATUS (0x00U)
#define ACCEL_XYZ_DATA_CFG (0x0EU)
#define ACCEL_CTRL_REG1 (0x2AU)
#define ACCEL_READ_TIMES (10U)
//Grove lcd I2C address
#define LCD_ADDRESS     (0x7c>>1)
#define RGB_ADDRESS     (0xc4>>1)
//adc base & user channel
#define DEMO_ADC16_BASE ADC0
#define DEMO_ADC16_CHANNEL_GROUP 0U
#define DEMO_ADC16_USER_CHANNEL1 8U // | PTB0 | Arduino ADC pin 0
#define DEMO_ADC16_AIR_QUALITY_CHANNEL 9U //  |  PTB1 | Arduino ADC pin 1

#define MAXTIMINGS 85  //for DHT11 reading
#define SENSOR GPIOC
#define TEMP_HUMID_PIN 10U  // D5 in base shield
#define BUZZER_PIN 0U  // Arduino D3 in base shield  PTD0
#define BUTTON_PIN 11U  // Arduino D4 in base shield PTC11

#define RED_LED_PIN 12U  //D2 in base shield header
#define GREEN_LED_PIN 5U   //d9
#define VIBRATION_MOTOR_PIN 3U   //d9

#define FLEXIO_RED_LED_PIN 16U   //Arduino D6 header in base shield
#define FLEXIO_GREEN_LED_PIN 17U   //Arduino D7 header

#define CLK_SPEED 150U //Put your board's clock speed here in MHz, FRDM-K82f = 150


/*******************************************************************************
 * Function prototypes
 ******************************************************************************/

static bool I2C_write_to_device(FLEXIO_I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t value);
static bool I2C_read_from_device(
    FLEXIO_I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize);

void delay(int time);
void FLEXIO_I2C_init();
void I2C_lcd_init();
void rgb_lcd_init();
void rgb_lcd_write(char *data);
void rgb_led_init();
void rgb_lcd_clear();
void rgb_lcd_set_cursor(uint8_t col, uint8_t row);
bool read_temp_humidity();
int adc_read();
void tostring(char str[], int num);

/*******************************************************************************
 * Variables
 ******************************************************************************/

//  FOXS8700 and MMA8451 device address
const uint8_t g_accel_address[] = {0x1CU, 0x1DU, 0x1EU, 0x1FU};
//  i2c communication
flexio_i2c_master_handle_t g_m_handle;
FLEXIO_I2C_Type i2cDev;
uint8_t g_accel_addr_found = 0x1CU;
volatile bool completionFlag = false;
volatile bool nakFlag = false;
// variables for lcd
uint8_t readBuff[2];
uint8_t g_master_buff[3];
uint32_t gStateBufferPos[8] = {0};
uint8_t status0_value = 0;

extern volatile uint8_t fxioState;
// variables for lcd
uint8_t command_reg = 0;
uint8_t data_reg = 0;
uint8_t command = 0;
// variables for accelerometer data
int16_t x_avg, y_avg, z_avg;
int16_t x, y, z;
// variables for ADC
volatile uint32_t g_Adc16ConversionValue;
adc16_config_t adc16ConfigStruct;
adc16_channel_config_t adc16ChannelConfigStruct;
// variable for string conversion
uint8_t data[6];
char str_for_am[10];
// variable for PWM
uint16_t duty_cycle;
// variable for sonar
int distance = 0;
// variables for fall detection
bool fall = false; //stores if a fall has occurred
bool trigger1 = false; //stores if first trigger (lower threshold) has occurred
bool trigger2 = false; //stores if second trigger (upper threshold) has occurred
bool trigger3 = false; //stores if third trigger (orientation change) has occurred

uint8_t trigger1count = 0; //stores the counts past since trigger 1 was set true
uint8_t trigger2count = 0; //stores the counts past since trigger 2 was set true
uint8_t trigger3count = 0; //stores the counts past since trigger 3 was set true

/*******************************************************************************
 * Function definition
 ******************************************************************************/

///////////////  callback function for i2c  /////////////////////////////////////
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
//.............................................................................//


//////////////////  i2c write function  /////////////////////////////////////////
static bool I2C_write_to_device(FLEXIO_I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t value)
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
//......................................................................................//


////////////////////////// i2c read function  //////////////////////////////////////////
static bool I2C_read_from_device(
    FLEXIO_I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize)
{
    flexio_i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress = device_addr;
    masterXfer.direction = kFLEXIO_I2C_Read;
    masterXfer.subaddress = reg_addr;
    masterXfer.subaddressSize = 1;
    masterXfer.data = rxBuff;
    masterXfer.dataSize = rxSize;

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
//...................................................................................//


//////////////////////  lcd i2c bus initialization function   /////////////////////////
void FLEXIO_I2C_init()
{
	    flexio_i2c_master_config_t masterConfig;

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
}
//...................................................................................................//


///////////////      delay function in microsecond  /////////////////////////////////////////////////
void delay(int time)
{
  uint32_t delayNum = (CLK_SPEED / 15) * time; //as clock speed in MHz, delay is in us
  volatile uint32_t cnt = 0U;
  for(cnt = 0U; cnt < delayNum; ++cnt){
	  __asm("NOP");
  }
}
//..................................................................................................//

/////////////////   lcd initialization function  ////////////////////////////////////////////////////
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
	I2C_write_to_device(&i2cDev, LCD_ADDRESS, command_reg, command);
	delay(4500);  //4.5 ms

	// second try
	I2C_write_to_device(&i2cDev, LCD_ADDRESS, command_reg, command);
	delay(150);

	// third go
	I2C_write_to_device(&i2cDev, LCD_ADDRESS, command_reg, command);

	command = 0x0C;  //LCD_DISPLAYON , LCD_CURSOROFF , LCD_BLINKOFF;
	I2C_write_to_device(&i2cDev, LCD_ADDRESS, command_reg, command);

	command = 0x01;  // clear it off
	I2C_write_to_device(&i2cDev, LCD_ADDRESS, command_reg, command);
	delay(2000);    // this command takes long time

	command = 0x0C;  // Initialize to default text direction
	I2C_write_to_device(&i2cDev, LCD_ADDRESS, command_reg, command);

}
//...........................................................................................//

////////////////////// Function for clearing lcd //////////////////////////////////////////////
void rgb_lcd_clear()
{
	command_reg =0x80;
	command = 0x01;  // clear it off
	I2C_write_to_device(&i2cDev, LCD_ADDRESS, command_reg, command);
	delay(2000);    // this command takes long time
}
//...........................................................................................//

///////////////////// Function for cursor set /////////////////////////////////////////////////

void rgb_lcd_set_cursor(uint8_t col, uint8_t row)
{

    command = (row == 0 ? col|0x80 : col|0xc0);
    command_reg =0x80;
    I2C_write_to_device(&i2cDev, LCD_ADDRESS, command_reg, command);

}
//.............................................................................................

///////////////////  Function for writing into lcd ////////////////////////////////////////////

void rgb_lcd_write(char *data)
{
	 uint32_t i = 0;
	 data_reg = 0x40;
	 for(i=0;i<strlen(data);i++)
	 {
		 I2C_write_to_device(&i2cDev, LCD_ADDRESS, data_reg, data[i]);
	 }

}
//............................................................................................//

////////////////////  Function for backlight of lcd  ///////////////////////////////////////////

void rgb_led_init()
{
	//Reset
	 command_reg = REG_MODE1;
	 command = 0x00;
	 I2C_write_to_device(&i2cDev, RGB_ADDRESS, command_reg, command);

	 //  Function set
	 command_reg = REG_OUTPUT;
	 command = 0xFF;
	 I2C_write_to_device(&i2cDev, RGB_ADDRESS, command_reg, command);

	 command_reg = REG_MODE2;
	 command = 0x20;
	 I2C_write_to_device(&i2cDev, RGB_ADDRESS, command_reg, command);

	 // Color Set
	 command_reg = REG_GREEN;
	 command = 0xFF;          //amount of green 0 - 255
	 I2C_write_to_device(&i2cDev, RGB_ADDRESS, command_reg, command);

	 command_reg = REG_RED;
	 command = 0xFF;          //amount of red 0 - 255
	 I2C_write_to_device(&i2cDev, RGB_ADDRESS, command_reg, command);

	 command_reg = REG_BLUE;
	 command = 0xFF;          //amount of blue 0 - 255
	 I2C_write_to_device(&i2cDev, RGB_ADDRESS, command_reg, command);
}
//.............................................................................................//

/////////////////// accelerometer read function /////////////////////////////////////////////////
void read_accelerometer_value(){

	    /*  read the accel xyz value if there is accel device on board */
	    if (true)
	    {
	        uint8_t databyte = 0;
	        uint8_t write_reg = 0;
	        uint8_t readBuffAxcel[7];
	        uint8_t status0_value = 0;

	        /*  please refer to the "example FXOS8700CQ Driver Code" in FXOS8700 datasheet. */
	        /*  write 0000 0000 = 0x00 to accelerometer control register 1 */
	        /*  standby */
	        /*  [7-1] = 0000 000 */
	        /*  [0]: active=0 */
	        write_reg = ACCEL_CTRL_REG1;
	        databyte = 0;
	        I2C_write_to_device(&i2cDev, g_accel_addr_found, write_reg, databyte);

	        /*  write 0000 0001= 0x01 to XYZ_DATA_CFG register */
	        /*  [7]: reserved */
	        /*  [6]: reserved */
	        /*  [5]: reserved */
	        /*  [4]: hpf_out=0 */
	        /*  [3]: reserved */
	        /*  [2]: reserved */
	        /*  [1-0]: fs=01 for accelerometer range of +/-4g range with 0.488mg/LSB */
	        /*  databyte = 0x01; */
	        write_reg = ACCEL_XYZ_DATA_CFG;
	        databyte = 0x01;
	        I2C_write_to_device(&i2cDev, g_accel_addr_found, write_reg, databyte);

	        /*  write 0000 1101 = 0x0D to accelerometer control register 1 */
	        /*  [7-6]: aslp_rate=00 */
	        /*  [5-3]: dr=001 for 200Hz data rate (when in hybrid mode) */
	        /*  [2]: lnoise=1 for low noise mode */
	        /*  [1]: f_read=0 for normal 16 bit reads */
	        /*  [0]: active=1 to take the part out of standby and enable sampling */
	        /*   databyte = 0x0D; */
	        write_reg = ACCEL_CTRL_REG1;
	        databyte = 0x0d;
	        I2C_write_to_device(&i2cDev, g_accel_addr_found, write_reg, databyte);
	        PRINTF("The accel values:\r\n");

	        status0_value = 0;
	        /*  wait for new data are ready. */
	        while (status0_value != 0xff)
	           {
	            I2C_read_from_device(&i2cDev, g_accel_addr_found, ACCEL_STATUS, &status0_value, 1);
	           }

	            /*  Multiple-byte Read from STATUS (0x00) register */
	        I2C_read_from_device(&i2cDev, g_accel_addr_found, ACCEL_STATUS, readBuffAxcel, 7);

	        status0_value = readBuffAxcel[0];
	        x = ((int16_t)(((readBuffAxcel[1] * 256U) | readBuffAxcel[2]))) / 4U;
	        y = ((int16_t)(((readBuffAxcel[3] * 256U) | readBuffAxcel[4]))) / 4U;
	        z = ((int16_t)(((readBuffAxcel[5] * 256U) | readBuffAxcel[6]))) / 4U;

	    }

	 PRINTF("\r\nEnd of I2C example .\r\n");
}
//................................................................................................//

///////////////////// function to convert int to string ////////////////////////////////////////////
void tostring(char str[], int num)
{
	int i, rem, len = 0, n;
	if(num<0){
		num *= -1;
	}
    n = num;
    while (n != 0)
    {
        len++;
        n /= 10;
    }
    for (i = 0; i < len; i++)
    {
        rem = num % 10;
        num = num / 10;
        str[len - (i + 1)] = rem + '0';
    }

        str[len] = '\0';
}
//..................................................................................................//

////////////////////   printing accelerometer data to lcd ////////////////////////////////////////////

void convert_to_string(int16_t x_value, int16_t y_value, int16_t z_value){
	    char str_for_x[10];
	    char str_for_y[10];
	    char str_for_z[10];

	    tostring(str_for_x, x_value);
	    tostring(str_for_y, y_value);
	    tostring(str_for_z, z_value);

	    rgb_lcd_clear();
	    if(x_value<0){
	    	rgb_lcd_write("X=");
	    	rgb_lcd_write("-");
	        rgb_lcd_write(str_for_x);}
	    else{
	    	rgb_lcd_write("X=");
	    	rgb_lcd_write(str_for_x);}

	    rgb_lcd_set_cursor(8, 0);
	    if(y_value<0){
	    	rgb_lcd_write("Y=");
	        rgb_lcd_write("-");
	        rgb_lcd_write(str_for_y);}
	    else{
	    	rgb_lcd_write("Y=");
	    	rgb_lcd_write(str_for_y);}

	    rgb_lcd_set_cursor(0, 1);
	    if(z_value<0){
	    	rgb_lcd_write("Z=");
	        rgb_lcd_write("-");
	        rgb_lcd_write(str_for_z);}
	    else{
	    	rgb_lcd_write("Z=");
	        rgb_lcd_write(str_for_z);}

}
//..............................................................................................//

/////////////////// ADC initialization  ///////////////////////////////////////////////
void ADC_Init()
{
	/*
	     * adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceVref;
	     * adc16ConfigStruct.clockSource = kADC16_ClockSourceAsynchronousClock;
	     * adc16ConfigStruct.enableAsynchronousClock = true;
	     * adc16ConfigStruct.clockDivider = kADC16_ClockDivider8;
	     * adc16ConfigStruct.resolution = kADC16_ResolutionSE12Bit;
	     * adc16ConfigStruct.longSampleMode = kADC16_LongSampleDisabled;
	     * adc16ConfigStruct.enableHighSpeed = false;
	     * adc16ConfigStruct.enableLowPower = false;
	     * adc16ConfigStruct.enableContinuousConversion = false;
	     */
	    ADC16_GetDefaultConfig(&adc16ConfigStruct);
	    ADC16_Init(DEMO_ADC16_BASE, &adc16ConfigStruct);
	    ADC16_EnableHardwareTrigger(DEMO_ADC16_BASE, false); /* Make sure the software trigger is used. */
	#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
	    if (kStatus_Success == ADC16_DoAutoCalibration(DEMO_ADC16_BASE))
	    {
	        PRINTF("ADC16_DoAutoCalibration() Done.\r\n");
	    }
	    else
	    {
	        PRINTF("ADC16_DoAutoCalibration() Failed.\r\n");
	    }
	#endif /* FSL_FEATURE_ADC16_HAS_CALIBRATION */

	    adc16ChannelConfigStruct.channelNumber = DEMO_ADC16_AIR_QUALITY_CHANNEL;
	    adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false; /* Enable the interrupt. */
	#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
	    adc16ChannelConfigStruct.enableDifferentialConversion = false;
	#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */
}
//...............................................................................................//

/////////////////////// ADC read function //////////////////////////////////////////////////////////////

int adc_read()
{
	adc16ChannelConfigStruct.channelNumber = DEMO_ADC16_AIR_QUALITY_CHANNEL;
	ADC16_SetChannelConfig(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);

	while (kADC16_ChannelConversionDoneFlag != ADC16_GetChannelStatusFlags(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP))
	 {
	 }
    g_Adc16ConversionValue = ADC16_GetChannelConversionValue(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP);

	PRINTF("ADC Value: %d\r\n", g_Adc16ConversionValue);
	return g_Adc16ConversionValue;
}
//...................................................................................................//

//////////////////////////  function to calculate air pollution //////////////////////////////////////////

void calculate_pollution(){
	rgb_lcd_set_cursor(0, 0);
	rgb_lcd_write("Air Qty= ");
	int value = adc_read();
	tostring(str_for_am, value);
	rgb_lcd_set_cursor(9, 0);
	rgb_lcd_write(str_for_am);

	if(value < 100){
	    PRINTF("Low Pollution");
	    rgb_lcd_set_cursor(0, 1);
	    rgb_lcd_write("Low Pollution");
	    	}

	else if(value > 100 && value < 400){
	    PRINTF("High Pollution");
	    rgb_lcd_set_cursor(0, 1);
	    rgb_lcd_write("High Pollution");
	    	}

	else if(value > 400){
		PRINTF("Very High Pollution");
		rgb_lcd_set_cursor(0, 1);
		rgb_lcd_write("Very High Pollution");
	        }

}
//...............................................................................................//

///////////////// function for reading humidity and temperature///////////////////////////////////

bool read_temp_humidity(){

	int i, j = 0;
	int counter = 0, count = 6;
	bool laststate = 1;


	gpio_pin_config_t pin_config_output =
		 {
				 kGPIO_DigitalOutput,1, // pin as output
		 };
	gpio_pin_config_t pin_config_input =
	 	 {
	 			 kGPIO_DigitalInput,0,  // pin as input
	 	 };

	GPIO_PinInit(SENSOR, TEMP_HUMID_PIN, &pin_config_output);

	GPIO_WritePinOutput(SENSOR, TEMP_HUMID_PIN, 1);  // output high
	delay(250000);                       // wait for 250ms
    // 20ms low pulse from the master to initiate communication
	// check DHT11 datasheet for details information
	GPIO_WritePinOutput(SENSOR, TEMP_HUMID_PIN, 0);  // output low
	delay(20000);                        // wait for 20ms

	GPIO_WritePinOutput(SENSOR, TEMP_HUMID_PIN, 1);  // output high
	delay(40);                           // wait 40us
    // configure the pin as input to receive data from dht sensor
	GPIO_PinInit(SENSOR, TEMP_HUMID_PIN, &pin_config_input);

	// dht sensor transmit 40 bits data
	data[0] = data[1] = data[2] = data[3] = data[4] = 0;

	counter = 0; j = 0; laststate = 1;
	  //
	for ( i=0; i< MAXTIMINGS; i++) {
	    counter = 0;
	    while (GPIO_ReadPinInput(SENSOR, TEMP_HUMID_PIN) == laststate) {
	        counter++;
	        delay(10);
	        if (counter == 255) {
	          break;
	        }
	      }
	    laststate = GPIO_ReadPinInput(SENSOR, TEMP_HUMID_PIN);

	    if (counter == 255) break;

	    // ignore first 3 transitions
	    if ((i >= 4) && (i%2 == 0)) {
	        // save each bit into the storage bytes
	       data[j/8] <<= 1;
	       if (counter > count)
	          data[j/8] |= 1;
	       j++;
	      }
      }
    // check 8bit parity to confirm the receive data is correct
	if ((j >= 40) &&
	      (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) ) {
	    return true;
	  }

	return false;
}
//..............................................................................................//

////////////////////////////  function to play a tone in buzzer ///////////////////////////////////

void play_tone(int tone, int duration) {
	 for (int i = 0; i < duration * 1000L; i += tone * 2) {
	      GPIO_SetPinsOutput(GPIOD, 1U << BUZZER_PIN);
	      delay(tone);
	      GPIO_ClearPinsOutput(GPIOD, 1U << BUZZER_PIN);
	      delay(tone);
     }
}
//..................................................................................................//

//////////////////////////  Function to produce musical note//////////////////////////////////////////
void play_note(char note, int duration) {
	 char names[] = { 'c', 'd', 'e', 'f', 'g', 'a', 'b', 'C' };
	 int tones[] = { 191, 170, 151, 143, 127, 113, 101, 95 };
	 // play the tone corresponding to the note name
	 for (int i = 0; i < 8; i++) {
	 if (names[i] == note) {
	       play_tone(tones[i], duration);

	    }
    }
}
//...................................................................................................//

//////////////////  Function to play alert ////////////////////////////////////////////////////////////

void play_alart()
{
	play_note('a', 150);
	delay(20000);
	play_note('a', 200);

	delay(300000);
}
//....................................................................................................//

///////////////////////// Function to initialized for GPIO as output & input///////////////////////////

void GPIO_Init()
{
	gpio_pin_config_t pin_config_output =
	  {
	    kGPIO_DigitalOutput,0, // pin as output  GPIO_DigitalOutput
	  };
    GPIO_PinInit(GPIOD, BUZZER_PIN, &pin_config_output);
    GPIO_PinInit(GPIOC, RED_LED_PIN, &pin_config_output);
    GPIO_PinInit(GPIOC, GREEN_LED_PIN, &pin_config_output);
    GPIO_PinInit(GPIOC, VIBRATION_MOTOR_PIN, &pin_config_output);

    gpio_pin_config_t pin_config_input =
	 {
	   kGPIO_DigitalInput, // pin as output  GPIO_DigitalOutput
	 };
	GPIO_PinInit(GPIOC, BUTTON_PIN, &pin_config_input);
}
//...............................................................................................//

///////////// Function using FlexIO module to read pulse width of Sonar////////////////////////////

void FLEXIO_SONAR_Init(uint8_t ECHO_PIN) {

	flexio_timer_config_t timerConfigSonar;
	flexio_shifter_config_t shifterConfigSonar;

	memset(&timerConfigSonar, 0, sizeof(timerConfigSonar));
	memset(&shifterConfigSonar, 0, sizeof(shifterConfigSonar));

	/* Timer for capture (128bits)*/
	timerConfigSonar.triggerSelect = FLEXIO_TIMER_TRIGGER_SEL_TIMn(4);
	timerConfigSonar.triggerPolarity = kFLEXIO_TimerTriggerPolarityActiveHigh;
	timerConfigSonar.triggerSource = kFLEXIO_TimerTriggerSourceInternal;
	timerConfigSonar.timerMode = kFLEXIO_TimerModeSingle16Bit;
	timerConfigSonar.timerOutput = kFLEXIO_TimerOutputZeroNotAffectedByReset;
	timerConfigSonar.timerDecrement = kFLEXIO_TimerDecSrcOnTriggerInputShiftTriggerInput;
	timerConfigSonar.timerReset = kFLEXIO_TimerResetNever;
	timerConfigSonar.timerDisable = kFLEXIO_TimerDisableOnTimerCompareTriggerLow;
	timerConfigSonar.timerEnable = kFLEXIO_TimerEnableOnTriggerRisingEdge;
	timerConfigSonar.timerStop = kFLEXIO_TimerStopBitDisabled;
	timerConfigSonar.timerStart = kFLEXIO_TimerStartBitDisabled;
	timerConfigSonar.timerCompare = 256U;   //256
	FLEXIO_SetTimerConfig(FLEXIO0, 3, &timerConfigSonar);

	/* Timer shifter (1 tick = 2cm)*/
	timerConfigSonar.triggerSelect = FLEXIO_TIMER_TRIGGER_SEL_PININPUT(ECHO_PIN);
	timerConfigSonar.triggerPolarity = kFLEXIO_TimerTriggerPolarityActiveHigh;
	timerConfigSonar.triggerSource = kFLEXIO_TimerTriggerSourceInternal;
	timerConfigSonar.timerMode = kFLEXIO_TimerModeSingle16Bit;
	timerConfigSonar.timerOutput = kFLEXIO_TimerOutputZeroNotAffectedByReset;
	timerConfigSonar.timerDecrement = kFLEXIO_TimerDecSrcOnFlexIOClockShiftTimerOutput;
	timerConfigSonar.timerReset = kFLEXIO_TimerResetOnTimerTriggerRisingEdge;
	timerConfigSonar.timerDisable = kFLEXIO_TimerDisableOnPreTimerDisable;
	timerConfigSonar.timerEnable = kFLEXIO_TimerEnableOnTriggerRisingEdge;
	timerConfigSonar.timerStop = kFLEXIO_TimerStopBitDisabled;
	timerConfigSonar.timerStart = kFLEXIO_TimerStartBitDisabled;
	timerConfigSonar.timerCompare = 330U;  // value depends on FlexIO clock & resolution
	/* adjusting this value you can change the resolution, more the
	 * resolution, more the shifter resister is required. I used 5 shifter (2-6) as shifter
	 * zero and one is used in i2c module. One shifter can store 32 bits means one shifter
	 * can measure up to 32cm. So, five shifter can measure 5*32 = 160 cm. If I decrease
	 * the resolution and count 2cm for every clock then it increase to 320cm. For that just
	 * use time compare value 660 instead of 330 and multiply shifter value by 2.
	 */
	FLEXIO_SetTimerConfig(FLEXIO0, 4, &timerConfigSonar);


	// Shifter bit 128-159 for 129-160cm
	shifterConfigSonar.timerSelect = 3;
	shifterConfigSonar.timerPolarity = kFLEXIO_ShifterTimerPolarityOnPositive;
	shifterConfigSonar.pinConfig = kFLEXIO_PinConfigOutputDisabled;
	shifterConfigSonar.shifterMode = kFLEXIO_ShifterModeReceive;
	shifterConfigSonar.inputSource = kFLEXIO_ShifterInputFromNextShifterOutput;
	shifterConfigSonar.shifterStop = kFLEXIO_ShifterStopBitDisable;
	shifterConfigSonar.shifterStart = kFLEXIO_ShifterStartBitDisabledLoadDataOnEnable;
	FLEXIO_SetShifterConfig(FLEXIO0, 2, &shifterConfigSonar);

	// Shifter bit 96-127 for 97-128cm
	shifterConfigSonar.timerSelect = 3;
	shifterConfigSonar.timerPolarity = kFLEXIO_ShifterTimerPolarityOnPositive;
	shifterConfigSonar.pinConfig = kFLEXIO_PinConfigOutputDisabled;
	shifterConfigSonar.shifterMode = kFLEXIO_ShifterModeReceive;
	shifterConfigSonar.inputSource = kFLEXIO_ShifterInputFromNextShifterOutput;
	shifterConfigSonar.shifterStop = kFLEXIO_ShifterStopBitDisable;
	shifterConfigSonar.shifterStart = kFLEXIO_ShifterStartBitDisabledLoadDataOnEnable;
	FLEXIO_SetShifterConfig(FLEXIO0, 3, &shifterConfigSonar);

	// Shifter bit 64-95 for 65-96cm
	shifterConfigSonar.timerSelect = 3;
	shifterConfigSonar.timerPolarity = kFLEXIO_ShifterTimerPolarityOnPositive;
	shifterConfigSonar.pinConfig = kFLEXIO_PinConfigOutputDisabled;
	shifterConfigSonar.shifterMode = kFLEXIO_ShifterModeReceive;
	shifterConfigSonar.inputSource = kFLEXIO_ShifterInputFromNextShifterOutput;
	shifterConfigSonar.shifterStop = kFLEXIO_ShifterStopBitDisable;
	shifterConfigSonar.shifterStart = kFLEXIO_ShifterStartBitDisabledLoadDataOnEnable;
	FLEXIO_SetShifterConfig(FLEXIO0, 4, &shifterConfigSonar);

	// Shifter bit 32-63 for 33 - 64cm
	shifterConfigSonar.timerSelect = 3;
	shifterConfigSonar.timerPolarity = kFLEXIO_ShifterTimerPolarityOnPositive;
	shifterConfigSonar.pinConfig = kFLEXIO_PinConfigOutputDisabled;
	shifterConfigSonar.shifterMode = kFLEXIO_ShifterModeReceive;
	shifterConfigSonar.inputSource = kFLEXIO_ShifterInputFromNextShifterOutput;
	shifterConfigSonar.shifterStop = kFLEXIO_ShifterStopBitDisable;
	shifterConfigSonar.shifterStart = kFLEXIO_ShifterStartBitDisabledLoadDataOnEnable;
	FLEXIO_SetShifterConfig(FLEXIO0, 5, &shifterConfigSonar);

	// Shifter bit 0-31 for 1-32cm
	shifterConfigSonar.timerSelect = 3;
	shifterConfigSonar.timerPolarity = kFLEXIO_ShifterTimerPolarityOnPositive;
	shifterConfigSonar.pinConfig = kFLEXIO_PinConfigOutputDisabled;
	shifterConfigSonar.pinSelect = ECHO_PIN;
	shifterConfigSonar.pinPolarity = kFLEXIO_PinActiveHigh;
	shifterConfigSonar.shifterMode = kFLEXIO_ShifterModeReceive;
	shifterConfigSonar.inputSource = kFLEXIO_ShifterInputFromPin;
	shifterConfigSonar.shifterStop = kFLEXIO_ShifterStopBitDisable;
	shifterConfigSonar.shifterStart = kFLEXIO_ShifterStartBitDisabledLoadDataOnEnable;
	FLEXIO_SetShifterConfig(FLEXIO0, 6, &shifterConfigSonar);
}
//.................................................................................................//

/////////////////// Function to find distance of object from sonar///////////////////////////////////

uint16_t FLEXIO_Sonar_read_distance(GPIO_Type *base, uint32_t flexio_pin) {

	PORT_SetPinMux(PORTC, 1U, kPORT_MuxAsGpio);

	gpio_pin_config_t config =
		 {
		   kGPIO_DigitalOutput, 0,
		 };
	GPIO_PinInit(GPIOC, 1U, &config);

	/* generating trigger signal */
	GPIO_WritePinOutput(base, flexio_pin, 1U);
	delay(5);
	GPIO_WritePinOutput(base, flexio_pin, 0U);

	PORT_SetPinMux(PORTC, 1U, kPORT_MuxAlt7);

	FLEXIO_SONAR_Init(flexio_pin+12);
	/* Wait for result checking the status bit for shifter 2-6 (e.g.   1111100) */
	while((FLEXIO_GetShifterStatusFlags(FLEXIO0) & 124) != 124) {
		;
	}

	/* Count bits to find the distance in cm */
	return (__builtin_popcount(FLEXIO0->SHIFTBUF[2]) + __builtin_popcount(FLEXIO0->SHIFTBUF[3]) +
			__builtin_popcount(FLEXIO0->SHIFTBUF[4]) + __builtin_popcount(FLEXIO0->SHIFTBUF[5]) +
			__builtin_popcount(FLEXIO0->SHIFTBUF[6]));
}
//................................................................................................//

///////////////////////////// Function to show distance into LCD////////////////////////////////////
void show_distance(){

	PRINTF("%d\r\n", distance);
	tostring(str_for_am, distance);
	rgb_lcd_write("Distance= ");
	rgb_lcd_set_cursor(10, 0);
    rgb_lcd_write(str_for_am);
    rgb_lcd_set_cursor(14, 0);
    rgb_lcd_write("CM");
}
//.................................................................................................//

////////////////////////////////// Function to generate PWM using FlexIO module/////////////////////
void FLEXIO_PWM_for_RED_LED_Init(uint16_t duty)
{
	flexio_timer_config_t timerConfig;
	memset(&timerConfig, 0, sizeof(timerConfig));

    timerConfig.triggerSelect = FLEXIO_TIMER_TRIGGER_SEL_SHIFTnSTAT(0);  //not mdt
	timerConfig.triggerPolarity = kFLEXIO_TimerTriggerPolarityActiveLow;  //not mdt
	timerConfig.triggerSource = kFLEXIO_TimerTriggerSourceInternal;  //internal
	timerConfig.pinConfig = kFLEXIO_PinConfigOutput;  //output
	timerConfig.pinSelect = FLEXIO_RED_LED_PIN;   //pwm pin & this pin must be a FlexIO pin
	timerConfig.pinPolarity = kFLEXIO_PinActiveHigh;  //active high
	timerConfig.timerMode = kFLEXIO_TimerModeDual8BitPWM; //pwm mode
	timerConfig.timerOutput = kFLEXIO_TimerOutputOneNotAffectedByReset; //initial output high
	timerConfig.timerDecrement = kFLEXIO_TimerDecSrcOnFlexIOClockShiftTimerOutput;  //flexio clock
	timerConfig.timerReset = kFLEXIO_TimerResetNever;        //never reset
	timerConfig.timerDisable = kFLEXIO_TimerDisableNever;    //never disable
	timerConfig.timerEnable = kFLEXIO_TimerEnabledAlways;    //always enable
	timerConfig.timerStop = kFLEXIO_TimerStopBitDisabled;    //stop bit disable
	timerConfig.timerStart = kFLEXIO_TimerStartBitDisabled;  //start bit disable

	// Set TIMCMP[15:8] = Low pulse width, TIMCMP[7:0] = high pulse width //
	timerConfig.timerCompare = duty; //this duty control the brightness

	FLEXIO_SetTimerConfig(FLEXIO0, 5, &timerConfig);
}

void FLEXIO_PWM_for_GREEN_LED_Init(uint16_t duty)
{
	flexio_timer_config_t timerConfig;
	memset(&timerConfig, 0, sizeof(timerConfig));

	timerConfig.triggerSelect = FLEXIO_TIMER_TRIGGER_SEL_SHIFTnSTAT(0);  //not mdt
	timerConfig.triggerPolarity = kFLEXIO_TimerTriggerPolarityActiveLow;  //not mdt
	timerConfig.triggerSource = kFLEXIO_TimerTriggerSourceInternal;  //internal
	timerConfig.pinConfig = kFLEXIO_PinConfigOutput;  //output
	timerConfig.pinSelect = FLEXIO_GREEN_LED_PIN;   //pwm pin & this pin must be a FlexIO pin
	timerConfig.pinPolarity = kFLEXIO_PinActiveHigh;  //active high
	timerConfig.timerMode = kFLEXIO_TimerModeDual8BitPWM; //pwm mode
	timerConfig.timerOutput = kFLEXIO_TimerOutputOneNotAffectedByReset; //initial output high
	timerConfig.timerDecrement = kFLEXIO_TimerDecSrcOnFlexIOClockShiftTimerOutput;  //flexio clock
	timerConfig.timerReset = kFLEXIO_TimerResetNever;        //never reset
	timerConfig.timerDisable = kFLEXIO_TimerDisableNever;    //never disable
	timerConfig.timerEnable = kFLEXIO_TimerEnabledAlways;    //always enable
	timerConfig.timerStop = kFLEXIO_TimerStopBitDisabled;    //stop bit disable
	timerConfig.timerStart = kFLEXIO_TimerStartBitDisabled;  //start bit disable

	// Set TIMCMP[15:8] = Low pulse width, TIMCMP[7:0] = high pulse width
    timerConfig.timerCompare = duty;

	FLEXIO_SetTimerConfig(FLEXIO0, 6, &timerConfig);
}
//..............................................................................................//


//////////////////////// Calculate orientation using accelerometer data///////////////////////////
int calculate_angle_change()
{
	read_accelerometer_value();
	int xAngle = (int16_t)floor((double)x * 0.011);
    int yAngle = (int16_t)floor((double)y * 0.011);
	int zAngle = (int16_t)floor((double)z * 0.011);

	int angleChange = pow(pow(xAngle,2)+pow(yAngle,2)+pow(zAngle,2),0.5);

	return angleChange;
}
//...............................................................................................//

//////////////////Function to calculate acceleration magnitude in any direction////////////////////

int calculate_acceleration_magnitude()
{
	read_accelerometer_value();
	float Raw_AM = pow(pow(x,2)+pow(y,2)+pow(z,2),0.5);
	int AM = (int)Raw_AM / 100;

	return AM;
}
//................................................................................................//

///////////////// function to detect fall///////////////////////////////////////////////////////////
void check_for_fall()
{
	if (trigger3==true){
	     trigger3count++;
	     if (trigger3count>=10){ //*****100******//allow 10 s for user to regain normal orientation

	        if ((calculate_angle_change()>=0) && (calculate_angle_change()<=35)){ //if orientation changes remains between 80-100 degrees
	            fall=true; trigger3=false; trigger3count=0;
	              }
	        else{
	           trigger3=false; trigger3count=0;
	           PRINTF("TRIGGER 3 DEACTIVATED\r\n");
	        }
	      }
	   }
	  if (fall==true){ //in event of a fall detection
	    PRINTF("FALL DETECTED\r\n");
	    while(1){
	    	play_alart();
	    	if(GPIO_ReadPinInput(GPIOC, BUTTON_PIN))
	    		break;
	    }
	    delay(20);
	    fall=false;
	    }
	  if (trigger2count>=6){ //allow 0.5s for orientation change
	    trigger2=false; trigger2count=0;
	    PRINTF("TRIGGER 2 DECACTIVATED\r\n");
	    }
	  if (trigger1count>=6){ //allow 0.5s for AM to break upper threshold
	    trigger1=false; trigger1count=0;
	    PRINTF("TRIGGER 1 DECACTIVATED\r\n");
	    }
	  if (trigger2==true){
	    trigger2count++;

	    if (calculate_angle_change()>=0 && calculate_angle_change()<=40){ //if orientation changes by between 80-100 degrees
	      trigger3=true; trigger2=false; trigger2count=0;
	      PRINTF("TRIGGER 3 ACTIVATED\r\n");
	       }
	    }
	  if (trigger1==true){
	    trigger1count++;
	    if (calculate_acceleration_magnitude()>=25){ //if AM breaks upper threshold (3g)
	      trigger2=true;
	      PRINTF("TRIGGER 2 ACTIVATED\r\n");
	      trigger1=false; trigger1count=0;
	      }
	    }
	  if (calculate_acceleration_magnitude()<=5 && trigger2==false){ //if AM breaks lower threshold (0.4g)
	    trigger1=true;
	    PRINTF("TRIGGER 1 ACTIVATED\r\n");
	    }
	  delay(100000);
}
//...................................................................................................//

////////////// Function to display temperature & humidity in lcd///////////////////////////////////////
void display_temp_humidity(){
	read_temp_humidity();
	rgb_lcd_set_cursor(0, 0);
	tostring(str_for_am, data[0]);
    rgb_lcd_write("RH = ");
	rgb_lcd_write(str_for_am);
	rgb_lcd_write("%");

	tostring(str_for_am, data[2]);
	rgb_lcd_set_cursor(0, 1);
	rgb_lcd_write("Temp = ");
	rgb_lcd_write(str_for_am);
	rgb_lcd_write(" C");

}
//...................................................................................................//

/*!
 * @brief Main function
 */
int main(void){
    // board & clock initialization
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    // gpio initialization
    GPIO_Init();
    // set clock for FlexIO module
    CLOCK_SetFlexio0Clock(2U);
    CLOCK_EnableClock(kCLOCK_Flexio0);
    // enable FlexIO module
    FLEXIO_Enable(FLEXIO0, true);
    // initialized i2c Bus
    FLEXIO_I2C_init();
    // initialized LCD
    rgb_lcd_init();
    rgb_led_init();
    rgb_lcd_clear();
    // initialized adc
    ADC_Init();
    /************************ Starting of main code *************************/
    int menu = 0;
    while (1)
    {
    	int buttonState = GPIO_ReadPinInput(GPIOC, BUTTON_PIN);
    	delay(70000); //70ms for removing switch debounce
        // check wither any fall is detected or not
    	check_for_fall();
        // implementing button press menu
    	if(buttonState){
    		menu++;
    		rgb_lcd_clear();
    		if(menu==3)
    			menu = 0;
    	}
        // display temperature & humidity by default
    	if(menu == 0){
    		display_temp_humidity();
    	}
        // display air quality for first button press
    	if(menu == 1){
    		calculate_pollution();
    	}
        // display distance of obstacle from the back side as sonar is set in back side
    	if(menu == 2){
    	    show_distance();
        }
        // check air quality
    	int air_quality = adc_read();
    	PRINTF("%d\r\n",air_quality);
        // turn green led for low pollution, brightness can be adjust by changing duty cycle
    	if(air_quality <= 100){
    		FLEXIO_PWM_for_GREEN_LED_Init(0x05ff);
    		FLEXIO_PWM_for_RED_LED_Init(0x0f00);
    	}
    	// turn red led on for high pollution, brightness can be adjusted
    	else if(air_quality >= 100){
    	    FLEXIO_PWM_for_GREEN_LED_Init(0x0f00);
    	    FLEXIO_PWM_for_RED_LED_Init(0x0fff);
    	}
    	// play alert for very high pollution
    	else if(air_quality >= 300){
    		play_alart();
    	}
    	// take 10 reading and calculate the distance, 10 reading for more accuracy
        for(int k=0; k<10; k++){
    	distance += FLEXIO_Sonar_read_distance(GPIOC, 1);
        }
        distance /=10;
    	PRINTF("Distance = %d\r\n", distance);
    	// if something within 10CM then vibrate the motor
        if(distance<=10){
        	GPIO_WritePinOutput(GPIOC, VIBRATION_MOTOR_PIN, 1);  // output high
        	delay(200000);
        	GPIO_WritePinOutput(GPIOC, VIBRATION_MOTOR_PIN, 0);  // output high
        	delay(200000);
        }
        // give some rest for the controller!!!
    	delay(100000);

    	//GPIO_WritePinOutput(GPIOD, BUZZER_PIN, 1);  // output high
    	//play_alart();

    }
}

