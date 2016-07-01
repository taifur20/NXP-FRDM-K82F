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


#include <stdio.h>
#include "main.h"

#define CLK_SPEED 150U
// how many timing transitions we need to keep track of. 2 * number bits + extra
#define MAXTIMINGS 85
#define SENSOR GPIOC
#define TEMP_HUMID_PIN 10U  // D5 in base shild

uint8_t data[6];
int i, j = 0;
int counter = 0, count = 6;
bool laststate = 1;
float f;

bool read_temp_humidity();

void delay(int time)
{
  uint32_t delayNum = (CLK_SPEED / 15) * time; //as clock speed in MHz, delay is in us
  volatile uint32_t cnt = 0U;
  for(cnt = 0U; cnt < delayNum; ++cnt){
	  __asm("NOP");
  }
}

bool read_temp_humidity(){

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


int main(void) {
  


  BOARD_InitPins(); 
  BOARD_BootClockRUN();
  BOARD_InitDebugConsole();




  for(;;) { /* Infinite loop to avoid leaving the main function */

	 		read_temp_humidity();
	 		//if(read_temp_humidity())
	 		  PRINTF("%d%%RH\t%d C\r\n",data[0],data[2]);
	 		delay(1000000);
    /* something to use as a breakpoint stop while looping */
  }
}
