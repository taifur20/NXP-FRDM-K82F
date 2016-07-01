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
#include <fsl_flexio.h>

#define FLEXIO_PWM_PIN 22U  //arduno d3

int main(void) {

  flexio_timer_config_t timerConfig;
  memset(&timerConfig, 0, sizeof(timerConfig));

  
  BOARD_InitPins(); 
  BOARD_BootClockRUN();
  BOARD_InitDebugConsole();


  CLOCK_EnableClock(kCLOCK_Flexio0);  //mdt
  FLEXIO_Enable(FLEXIO0, true);  //must

  /*3. Configure the timer 0 for generating bit clock. */
  timerConfig.triggerSelect = FLEXIO_TIMER_TRIGGER_SEL_SHIFTnSTAT(0);  //not mdt
  timerConfig.triggerPolarity = kFLEXIO_TimerTriggerPolarityActiveLow;  //not mdt
  timerConfig.triggerSource = kFLEXIO_TimerTriggerSourceInternal;  //internal
  timerConfig.pinConfig = kFLEXIO_PinConfigOutput;  //output
  timerConfig.pinSelect = FLEXIO_PWM_PIN;   //pwm pin
  timerConfig.pinPolarity = kFLEXIO_PinActiveHigh;  //active high
  timerConfig.timerMode = kFLEXIO_TimerModeDual8BitPWM; //pwm mode
  timerConfig.timerOutput = kFLEXIO_TimerOutputOneNotAffectedByReset; //initial output high
  timerConfig.timerDecrement = kFLEXIO_TimerDecSrcOnFlexIOClockShiftTimerOutput;  //flexio clock
  timerConfig.timerReset = kFLEXIO_TimerResetNever;        //never reset
  timerConfig.timerDisable = kFLEXIO_TimerDisableNever;    //never disable
  timerConfig.timerEnable = kFLEXIO_TimerEnabledAlways;    //always enable
  timerConfig.timerStop = kFLEXIO_TimerStopBitDisabled;    //stop bit disable
  timerConfig.timerStart = kFLEXIO_TimerStartBitDisabled;  //start bit disable

  /* Set TIMCMP[15:8] = Low pulse width, TIMCMP[7:0] = high pulse width */
  timerConfig.timerCompare = 0x0FFF;

  FLEXIO_SetTimerConfig(FLEXIO0, 6, &timerConfig);
    

  for(;;) { /* Infinite loop to avoid leaving the main function */
    __asm("NOP"); /* something to use as a breakpoint stop while looping */
  }
}
