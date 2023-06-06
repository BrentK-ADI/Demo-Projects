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
#include "CppUTest/TestHarness_c.h"
#include "CppUtestExt/MockSupport_c.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/**
 * This unit test is focused on checking all the integer math, not I2C commands.
 * In order to simplify the dependencies of the testing, #define__PLATFORM_H__
 * to trick LTC3337.c/.h to not include the contents of Platform.h, then just
 * mock up the i2c_device_t and functions to get everything to compile and
 * link.
 */
//Bypass including Platform.h, and mock up what is needed
#define __PLATFORM_H__
typedef uint32_t i2c_device_t;
int32_t Platform_i2cRead(i2c_device_t* dev, uint8_t addr, uint8_t* readBuf, uint8_t count){return 0;}
int32_t Platform_i2cWrite(i2c_device_t* dev, uint8_t addr, uint8_t* writeBuf, uint8_t count){return 0;}
int32_t Platform_i2cWriteRead(i2c_device_t* dev, uint8_t addr,
                              uint8_t* writeBuf, uint8_t writeCount,
                              uint8_t* readBuf, uint8_t readCount,
                              uint8_t repeatStart ){ return 0; }



//Shortcut to test static functions in the LTC3337.c file
#include "../LTC3337.c"

/**
 * Helper function to independently calculate the counter value using
 * double precision math. This will be our "expected value" against the
 * integer math used in the LTC3337 driver.
 * @param counter - Counter register value
 * @param prescale - Prescale value (M)
 * @param ipk - IPK pin values (0-7)
 * @returns Charge in A-Hr
 */
static double CalcA_hr_double(uint16_t counter, uint8_t prescale, uint8_t ipk )
{
    double qlsb_A;
    double qlsb_M;

    //Independently look up what the qLSB is for each IPK
    switch(ipk)
    {
        case 0: //5ma
            qlsb_A = 0.0007457;
            break;
        case 1: //10ma
            qlsb_A = 0.001491;
            break;
        case 2: //15ma
            qlsb_A = 0.002237;
            break;
        case 3: //20ma
            qlsb_A = 0.002983;
            break;
        case 4: //25ma
            qlsb_A = 0.003728;
            break;
        case 5: //50ma
            qlsb_A = 0.007457;
            break;
        case 6: //75ma
            qlsb_A = 0.01118;
            break;
        case 7: //100ma
            qlsb_A = 0.01491;
            break;
        default:
            return 0.0;
    }

    //qLSBM = qLSB / 2^M
    qlsb_M = qlsb_A / pow(2, prescale);

    //Accumulated value is counter * qlsb_M
    return (double)counter * qlsb_M;
}

/**
 * This is our actual test for the counter integer math.
 * Since the number of test cases is small enough, run through
 * all permutations of counter, prescale and IPK to check the
 * integer math against our independent double math
 */
TEST_C(ltc3337, counter_math)
{
    charge_count_t intCalc;
    double doubleCalc;
    double intToDouble;

    for( int ipk = 0; ipk < LTC3337_NUM_IPEAKS; ipk++ )
    {
        for( int ps = 0; ps <= LTC3337_RA_PRESCALE_MSK; ps++ )
        {
            for(int count = 0; count <= 0xFFFF; count++)
            {
                //Call our driver's function, using the lookup table to get qLSB
                LTC3337_ChargeCountToA(count, ps, iPeakLookupTable[ipk].qLsb_nA_hr, &intCalc );

                //Get the independently calculated value
                doubleCalc = CalcA_hr_double(count, ps, ipk);

                //Convert the integer values to a double
                intToDouble = (double)intCalc.A_hr + ((double)intCalc.nA_hr / 1000000000.0);

                //Check our math to +/- 1nA-hr
                //Note, as a sanity check. Try commenting out the following lines in the driver
                //     fractCount = qlsb_m_remainder * chargeCount;
                //     value->nA_hr += fractCount >> prescale;
                //to show we fail when not including that last little bit of remainder
                CHECK_EQUAL_C_REAL(doubleCalc, intToDouble, 1.0/1000000000.0);
            }
        }
    }
}


/**
 * Helper function to independently calculate the counter value using
 * double precision math. This will be our "expected value" against the
 * integer math used in the LTC3337 driver.
 * @param A_hr - Value in A-Hr
 * @param prescale - Prescale value (M)
 * @param ipk - IPK pin values (0-7)
 * @returns Charge counter register
 */
static double CalcRegFromA_hr_double(double A_hr, uint8_t prescale, uint8_t ipk )
{
    double qlsb_A;
    double qlsb_M;

    //Independently look up what the qLSB is for each IPK
    switch(ipk)
    {
        case 0: //5ma
            qlsb_A = 0.0007457;
            break;
        case 1: //10ma
            qlsb_A = 0.001491;
            break;
        case 2: //15ma
            qlsb_A = 0.002237;
            break;
        case 3: //20ma
            qlsb_A = 0.002983;
            break;
        case 4: //25ma
            qlsb_A = 0.003728;
            break;
        case 5: //50ma
            qlsb_A = 0.007457;
            break;
        case 6: //75ma
            qlsb_A = 0.01118;
            break;
        case 7: //100ma
            qlsb_A = 0.01491;
            break;
        default:
            return 0.0;
    }

    //qLSBM = qLSB / 2^M
    qlsb_M = qlsb_A / pow(2, prescale);
    return (double)A_hr / qlsb_M;
}

/**
 * This is our actual test for the counter integer math.
 * Since the number of test cases is small enough, run through
 * all permutations of counter, prescale and IPK to check the
 * integer math against our independent double math
 */
TEST_C(ltc3337, counter_math_rev)
{
    double A_hr;
    charge_count_t intCalc;
    uint16_t regValue;
    double intPart;
    double fractPart;
    double doubleCalc;
    double intToDouble;
    int32_t delta;

    for( int ipk = 0; ipk < LTC3337_NUM_IPEAKS; ipk++ )
    {
        for( int ps = 0; ps <= LTC3337_RA_PRESCALE_MSK; ps++ )
        {
            for(int count = 0; count <= 0xFFFF; count++)
            {
                //Calculate the value for testing purposes
                A_hr = CalcA_hr_double(count, ps, ipk);
                
                //Separate A-hr and fractional A-hr
                fractPart = modf(A_hr, &intPart);
                        
                //Create the integer struct
                intCalc.A_hr = (uint32_t)(intPart);
                intCalc.nA_hr = (uint32_t)(fractPart * 1000000000.0);

                //Recreate the value for internal use. This should be the same as A_hr before,
                //but clipped to resolution of 1nA-hr
                A_hr = (double)(intCalc.A_hr) + ((double)intCalc.nA_hr / 1000000000.0);

                //Our internal "expected" calculation
                doubleCalc = CalcRegFromA_hr_double(A_hr, ps, ipk);     

                //Calculation from the driver
                regValue = LTC3337_A_ToChargeCount(ps, iPeakLookupTable[ipk].qLsb_nA_hr, &intCalc);
                
                //Check our math to +/- 1bit
                //Note: for sanity check, comment out the inclusion of the remainder in the driver
                //code workingCount += (remainderCount / qlsb_m_nA);
                delta = (int32_t)doubleCalc - (int32_t)regValue;                
                CHECK_C(abs(delta) <= 1);                        
            }
        }
    }
}


/**
 * Helper function to independently calculate the temperature value using
 * double precision math. This will be our "expected value" against the
 * integer math used in the LTC3337 driver.
 * @param reg - Temperature register value
 * @returns Temperature in Deg C
 */
static double CalcTemp_double(uint8_t reg )
{
    return (0.784 * (double)reg) - 41.0;
}

/**
 * This is our actual test for the temperature integer math.
 * Since the number of test cases is small enough, run through
 * all permutations of temps sensor, to check the
 * integer math against our independent double math
 */
TEST_C(ltc3337, temp_math)
{
    int16_t intCalc;
    double doubleCalc;
    double intToDouble;

    for( int reg = 0; reg <= 0xFF; reg++ )
    {
        intCalc = LTC3337_TempRegToC(reg);
        doubleCalc = CalcTemp_double(reg);

        //Check we correctly rounded
        //Note: For a sanity check, remove the +500 in the temperature
        //calculation in LTC3337.c
        CHECK_EQUAL_C_INT((int16_t)round(doubleCalc), intCalc);
    }
}

/**
 * Helper function to independently calculate the register value from
 * a temperature in C using double precision math. This will be our 
 * "expected value" against the integer math used in the LTC3337 driver.
 * @param temp - Temperature value in Deg C
 * @returns Equivalent register value
 */
static double CalcRegFromTemp_double(int16_t temp )
{
    return ((double)temp + 41) / 0.784;
}

/**
 * This is our actual test for the temperature to register
 * integer math. Since the number of test cases is small enough, 
 * run through all permutations of temperature values, to check the
 * integer math against our independent double math
 */
TEST_C(ltc3337, temp_math2)
{
    uint8_t regCalc;
    double doubleReg;

    for( int temp = LTC3337_MIN_TEMP_C; temp <= LTC3337_MAX_TEMP_C; temp++ )
    {
        regCalc = LTC3337_TempC_ToReg(temp);
        doubleReg = CalcRegFromTemp_double(temp);

        //Check we correctly rounded
        //Note: For a sanity check, remove the (LTC3337_TLSB_mC / 2)
        //in the temperature calculation in LTC3337.c
        CHECK_EQUAL_C_INT((uint8_t)round(doubleReg), regCalc);
    }
}

/**
 * Helper function to independently calculate the battery value using
 * double precision math. This will be our "expected value" against the
 * integer math used in the LTC3337 driver.
 * @param reg - Battery register value
 * @returns Battery in mV
 */
static double CalcBatt_double(uint16_t reg )
{
    return (1.465 * (double)reg);
}

/**
 * This is our actual test for the battery integer math.
 * Since the number of test cases is small enough, run through
 * all permutations of battery regs, to check the
 * integer math against our independent double math
 */
TEST_C(ltc3337, batt_math)
{
    uint16_t intCalc;
    double doubleCalc;
    double intToDouble;

    for( int reg = 0; reg <= 0xFFF; reg++ )
    {
        intCalc = LTC3337_VBatTo_mV(reg);
        doubleCalc = CalcBatt_double(reg);

        //Check we correctly rounded
        //Note: For a sanity check, remove the +500 in the battery
        //calculation in LTC3337.c
        CHECK_EQUAL_C_INT((int16_t)round(doubleCalc), intCalc);
    }
}