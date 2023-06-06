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
#ifndef __LTC3337_H__
#define __LTC3337_H__

#include <stdint.h>
#include "platform.h"


#define LTC3337_MAX_TEMP_C  159 //Maximum temperature for alarms
#define LTC3337_MIN_TEMP_C  -41 //Minimum temperature for alarms
#define LTC3337_MAX_PRESCALE 15 //Maximum prescaler value


//Bitfield definitions for reading interrupt status
#define LTC3337_INT_OVERFLOW    (1U << 0)
#define LTC3337_INT_COUNT_ALARM (1U << 1)
#define LTC3337_INT_LOW_TEMP    (1U << 2)
#define LTC3337_INT_HIGH_TEMP   (1U << 3)
#define LTC3337_INT_ADC_READY   (1U << 4)


/**
 * Instance structure for the LTC3337 instance. Storage should be
 * allocated in user's code. Fields should be used by the driver only,
 * and not read or manipulated by the user
 */
typedef struct
{
    i2c_device_t* i2cHandle;    //I2C device
    uint8_t iPeakLatched;       //iPeak value read at init
    uint16_t latchedRegA;       //Latched Register A value
} ltc3337_inst_t;


/**
 * Structure for holding the accumulated charge calculation
 */
typedef struct
{
    uint32_t A_hr;  //A-hrs
    uint32_t nA_hr; //nA-hrs
} charge_count_t;

/**
 * Enumeration of sources for reading voltage
 */
typedef enum
{
    BAT_IN_IPEAK_ON = 0,
    BAT_IN_IPEAK_OFF,
    BAT_OUT_IPEAK_ON,
    BAT_OUT_IPEAK_OFF
} ltc3337_voltage_src_t;


/**
 * Initializes the driver instance. Assigns the I2C bus and reads the 
 * IPK pins.  The prescaler is set to the user value, and the accumulated
 * charge alarm is defaulted to maximum 0xFF
 * @param dev - Pointer to the device instance
 * @param i2cHandle - Pointer to the I2C bus device
 * @param prescale - Prescaler value
 * @returns 0 on success, Non-0 on error
 */
int32_t LTC3337_Init(ltc3337_inst_t* dev, i2c_device_t* i2cHandle, 
                     uint8_t prescale);

/**
 * Sets the prescaler value of the device.
 * IMPORTANT: Changing the prescaler does not affect the value of the accumulated
 * charge register. If the prescaler is set sometime following the initial
 * runtime setup, after charge has been accumulated, the acccumulated charge
 * register should be set according to the current value shifted by the 
 * difference in original and current prescaler. This driver does not
 * automatically perform that action.
 * IMPORTANT: Changing the prescaler does not alter the configuration of alarm. 
 * That may need to be changed if configured prior to the prescaler
 * @param dev - Pointer to the device instance
 * @param prescale - Prescaler value
 * @returns 0 on success, Non-0 on error
 */
int32_t LTC3337_SetPrescaler(ltc3337_inst_t* dev, uint8_t prescale);

/**
 * Set the alarm thresholds for hot and cold temperatures
 * @param dev - Device to configure
 * @param hotAlarm - Hot alarm threshold, in Degrees C
 * @param coldAlarm - Cold alarm threshold, in Degrees C
 * @returns 0 on success, Non-0 on error
 */
int32_t LTC3337_SetTemperatureAlarmsC(ltc3337_inst_t* dev, int16_t hotAlarm, 
                                      int16_t coldAlarm );

/**
 * Sets the state of the coulomb counter shutdown function
 * @param dev - Device to configure
 * @param shutdownEn - 1 - Enable shutdown, 0 - Coulomb counter running
  * @returns 0 on success, Non-0 on error
 */
int32_t LTC3337_SetCounterShutdown(ltc3337_inst_t* dev, uint8_t shutdownEn );

/**
 * Sets the alarm level for the coulomb counter alarm. The input value is the
 * raw register value, and can be calculated using LTC3337_CalculateChargeRegister
 * or manually by hand.
 * IMPORTANT: Only the upper 8-bits of the value are utilized. Use the roundUp
 * flag to have the driver optionally round up to the next valid bit
 * @param dev - Device to configure
 * @param regValue - Register value, as if all 16-bits were utilized
 * @param roundUp - 1 -Have the driver round up to next usable alarm bit
 * @returns 0 on success, Non-0 on error
 */
int32_t LTC3337_SetCounterAlarm(ltc3337_inst_t* dev, uint16_t regValue, 
                                uint8_t roundUp );


/**
 * Sets the level for the coulomb counter accumulated charge register. 
 * The input value is the raw register value, and can be calculated using 
 * LTC3337_CalculateChargeRegister or manually by hand.
 * IMPORTANT: Only the upper 8-bits of the value are utilized. Use the roundUp
 * flag to have the driver optionally round up to the next valid bit
 * @param dev - Device to configure
 * @param regValue - Register value, as if all 16-bits were utilized
 * @param roundUp - 1 -Have the driver round up to next usable alarm bit
 * @returns 0 on success, Non-0 on error
 */
int32_t LTC3337_SetAccumulatedCharge(ltc3337_inst_t* dev, uint16_t regValue, 
                                     uint8_t roundUp );

/**
 * Gets the current value of the accumulated charge register.  This function
 * can return the raw value and/or the calculated value based on the prescaler
 * and IPK settings of dev.
 * @param dev - Device to read from
 * @param value - Pointer to calculated storage location, or NULL
 * @param rawValue - Pointer to raw storage location, or NULL
 * @returns 0 on success, Non-0 on error
 */
int32_t LTC3337_GetAccumulatedCharge(ltc3337_inst_t* dev, charge_count_t* value, 
                                     uint16_t* rawValue );

/**
 * Gets the current value of the requested voltage monitor in mV.  
 * @param dev - Device to read from
 * @param source - Which voltage source to read
 * @param value - Location to store calcualted value, in mV
 * @returns 0 on success, Non-0 on error
 */
int32_t LTC3337_GetVoltage_mV(ltc3337_inst_t* dev, ltc3337_voltage_src_t source, 
                              uint32_t* value );

/**
 * Gets the current die temperature
 * @param dev - Device to read from
 * @param value - Location to store calculated value, in Deg C
 * @returns 0 on success, Non-0 on error
 */
int32_t LTC3337_GetTemperatureC(ltc3337_inst_t* dev, int16_t* value );


/**
 * Reads and clears the interrupt status of the chip.  Since the die temp is 
 * also provided in the interrupt register, temperature is optionally provided
 * as well. This allows a user to poll interrupt status and not accidentally
 * clear interrupts by reading temperature inbetween interrupt polls
 * @param dev - Device to read from
 * @param intField - Interrupt status bits
 * @param tempC - Location to store temperature (or NULL)
 * @returns 0 on success, Non-0 on error
 */
int32_t LTC3337_GetAndClearInterrupts(ltc3337_inst_t* dev, uint8_t* intField,
                                      int16_t* tempC );

/**
 * Gets a register value equivalent of the provided charge in A/nA-hrs
 * to be used in configuring the charge alarms, and resetting the charge
 * register. This function does not access the device over the bus, but
 * utilizes the latched prescaled and IPK values to perform the calculation
 * @param dev - Device to utilize configuration from
 * @param chargeA - Pointer to the charge values in A/nA-hrs
 * @param regValue - Pointer to store the output register value
 * @returns 0 on success, Non-0 on error
 */
int32_t LTC3337_CalculateChargeRegister(ltc3337_inst_t* dev, 
                                        charge_count_t* chargeA, 
                                        uint16_t* regValue );

#endif
