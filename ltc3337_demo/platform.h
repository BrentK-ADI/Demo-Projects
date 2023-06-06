/********************************************************************************
 * Copyright (C) 2023 Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
#ifndef __PLATFORM_H__
#define __PLATFORM_H__

#include <stdint.h>
#include "i2c.h"

#define DEMO_I2C    MXC_I2C2
#define NUM_LEDS    3

//Define the i2c_device_t to allow it to be generic among Platform_ users
typedef mxc_i2c_regs_t i2c_device_t;

//General callback prototype for ISR callbacks
typedef void (*isr_callback_function)(void);

/**
 * Initializes the platform support. Should be called first before any other
 * Platform calls
 */
void Platform_init(void);

/**
 * Performs an I2C read operation.
 * @param dev - I2C Device to read from
 * @param addr - Address of the device to read
 * @param readBuf - Buffer to read data into
 * @param count - Number of bytes to read from the bus
 * @returns - 0 on success, Non-0 on error
 */
int32_t Platform_i2cRead(i2c_device_t* dev, uint8_t addr, 
                         uint8_t* readBuf,  uint8_t count);

/**
 * Performs an I2C write operation.
 * @param dev - I2C Device to write to
 * @param addr - Address of the device to write
 * @param readBuf - Buffer of data to write
 * @param count - Number of bytes to write
 * @returns - 0 on success, Non-0 on error
 */
int32_t Platform_i2cWrite(i2c_device_t* dev, uint8_t addr, 
                          uint8_t* writeBuf, uint8_t count);

/**
 * Performs an I2C write operation, followed by a read operation. If repeat
 * start is set, a stop condition will not be issued following the write. If
 * repeat start is not set, a stop condition will be set after the write, prior 
 * to the start condition for the read.
 * @param dev - I2C Device to utilize
 * @param addr - Address of the device
 * @param writeBuf - Buffer of data to write
 * @param writeCount - Number of bytes to write
 * @param readBuf - Buffer to read data into
 * @param readCount - Number of bytes to read from the bus
 * @param repeatStart - If 1, do not issue a stop condition following the write
 * @returns - 0 on success, Non-0 on error
 */
int32_t Platform_i2cWriteRead(i2c_device_t* dev, uint8_t addr, 
                              uint8_t* writeBuf, uint8_t writeCount,
                              uint8_t* readBuf, uint8_t readCount,
                              uint8_t repeatStart );

/**
 * Assigns the callback function for handling LTC3337 interrupt signals.
 * @param func - Function to be called (or NULL) when the LTC3337 issues an
 *               interrupt
 */                              
void Platform_setLTC_ISR_Handler(isr_callback_function func);

/**
 * Assigns the callback function for handling button presses.
 * @param func - Function to be called (or NULL) when the configuration menu
 *               button is pressed
 */                              
void Platform_setButtonISR_Handler(isr_callback_function func);

/**
 * Sets the state of the Load control signal (active high)
 * @param en - On / Off state of the control signal
 */                              
void Platform_setLoad1(uint8_t en);

/**
 * Sets the state of the Load2 control signal (active high)
 * @param en - On / Off state of the control signal
 */                              
void Platform_setLoad2(uint8_t en);

/**
 * Starts/Restarts the high resolution timer for measuring execution time
 */                             
void Platform_timerStart(void);

/**
 * Stops the high resolution timer, and returns the elapsed time in microseconds
 * @returns Timer elapsed time in microseconds
 */                             
uint32_t Platform_timerStop_uS(void);

/**
 * Performs a blocking delay
 * @param ms - Delay time in milliseconds
 */                             
void Platform_delay(uint32_t ms);

/**
 * Writes a single character out the console UART
 * @param ch - Character to write
 */                             
void Platform_writeChar(char ch);

/**
 * Reads a single character from the console UART. This is a blocking function
 * until a character is received
 * @returns Character read
 */                             
char Platform_readChar(void);

/**
 * Writes a Null-terminated string out the console UART
 * @param str - String to write
 */                             
void Platform_writeString(char* str);

#endif
