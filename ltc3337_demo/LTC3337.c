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
#include <assert.h>
#include <stdio.h>
#include "LTC3337.h"

#define LTC3337_I2C_ADDR    0x64 // b1100100[r/w] 0xC8, 0xC9

#define LTC3337_REG_A   0x01 //Prescaler Select, IRQ Clear, Shutdown, Alarm Thresh
#define LTC3337_RA_PRESCALE_POS     0
#define LTC3337_RA_PRESCALE_MSK     0x0F
#define LTC3337_RA_CLEAR_INT_POS    4
#define LTC3337_RA_CLEAR_INT        ((uint16_t)1 << LTC3337_RA_CLEAR_INT_POS)
#define LTC3337_RA_CNT_CHK_POS      5
#define LTC3337_RA_CNT_CHK          ((uint16_t)1 << LTC3337_RA_CNT_CHK_POS)
#define LTC3337_RA_SHTDN_POS        6
#define LTC3337_RA_SHTDN            ((uint16_t)1 << LTC3337_RA_SHTDN_POS)
#define LTC3337_RA_ADC_CONV_POS     7
#define LTC3337_RA_ADC_CONV         ((uint16_t)1 << LTC3337_RA_ADC_CONV_POS)
#define LTC3337_RA_ALARM_LVL_MSK    0xFF
#define LTC3337_RA_ALARM_LVL_POS    8

#define LTC3337_REG_B   0x02 //Accumulated Charge

#define LTC3337_REG_C   0x03 //Status, Temperature
#define LTC3337_RC_OVERFLOW_POS     0
#define LTC3337_RC_OVERFLOW         ((uint16_t)1 << LTC3337_RC_OVERFLOW_POS)
#define LTC3337_RC_ALARM_TRIP_POS   1
#define LTC3337_RC_ALARM_TRIP       ((uint16_t)1 << LTC3337_RC_ALARM_TRIP_POS)
#define LTC3337_RC_ALARM_MIN_POS    2
#define LTC3337_RC_ALARM_MIN        ((uint16_t)1 << LTC3337_RC_ALARM_MIN_POS)
#define LTC3337_RC_ALARM_MAX_POS    3
#define LTC3337_RC_ALARM_MAX        ((uint16_t)1 << LTC3337_RC_ALARM_MAX_POS)
#define LTC3337_RC_ADC_READY_POS    4
#define LTC3337_RC_ADC_READY        ((uint16_t)1 << LTC3337_RC_ADC_READY_POS)
#define LTC3337_RC_IPK_PIN_MSK      0x7
#define LTC3337_RC_IPK_PIN_POS      5
#define LTC3337_RC_DIE_TEMP_MSK     0xFF
#define LTC3337_RC_DIE_TEMP_POS     8

#define LTC3337_REG_D   0x04 //BAT_IN V, Ipeak On
#define LTC3337_REG_E   0x05 //BAT_IN V, Ipeak Off
#define LTC3337_REG_F   0x06 //BAT_OUT V, Ipeak On
#define LTC3337_REG_G   0x07 //BAT_OUT V, Ipeak Off
#define LTC3337_BATV_MSK            0xFFF

#define LTC3337_REG_H   0x08 //Temp Alarm Config
#define LTC3337_RH_TEMP_MSK         0xFF
#define LTC3337_RH_HOT_ALARM_POS    8
#define LTC3337_RH_COLD_ALARM_POS   0


#define LTC3337_VLSB_uV     1465 //1.465 mV = 1465 uV
#define LTC3337_TLSB_mC     784  //0.784 Deg C = 784 mC
#define LTC3337_TEMP_MIN_C  -41

#define LTC3337_NUM_IPEAKS  8 //3-bit iPeak Input

//Scale values used in integer match to calulate A/nA-hrs from counters
#define LTC3337_NANO_AMP        1000000000
#define LTC3337_CALC_SCALE      10000
#define LTC3337_CALC_TO_WHOLE   (LTC3337_NANO_AMP / LTC3337_CALC_SCALE)

/**
 * Structure supporting the iPeak lookup table
 */
typedef struct 
{
    uint8_t  iPeak_mA;       //iPeak, in mA
    uint32_t qLsb_nA_hr;     //qLsb, nA-hr component
} ipeak_entry_t;

/**
 * Lookup table for the iPeak values. This converts the 3-bit
 * iPeak input configuration signals, to a iPeak current and qLsb 
 * values for Coulomb counter calculations
 */
static const ipeak_entry_t iPeakLookupTable[LTC3337_NUM_IPEAKS] = 
{
    { .iPeak_mA = 5,   .qLsb_nA_hr = 745700 },   //b000 - 5ma, 745.7 uA-hr
    { .iPeak_mA = 10,  .qLsb_nA_hr = 1491000 },  //b001 - 10ma, 1.491 mA-hr
    { .iPeak_mA = 15,  .qLsb_nA_hr = 2237000 },  //b010 - 15ma, 2.237 mA-hr
    { .iPeak_mA = 20,  .qLsb_nA_hr = 2983000 },  //b011 - 20ma, 2.983 mA-hr
    { .iPeak_mA = 25,  .qLsb_nA_hr = 3728000 },  //b100 - 25ma, 3.728 mA-hr
    { .iPeak_mA = 50,  .qLsb_nA_hr = 7457000 },  //b101 - 50ma, 7.457 mA-hr
    { .iPeak_mA = 75,  .qLsb_nA_hr = 11180000 }, //b110 - 75ma, 11.18 mA-hr
    { .iPeak_mA = 100, .qLsb_nA_hr = 14910000 }, //b111 - 100ma, 14.91 mA-hr
};

/** Local Prototypes */
static int32_t  LTC3337_ReadReg(ltc3337_inst_t* dev, uint8_t reg, uint16_t* data);
static int32_t  LTC3337_WriteReg(ltc3337_inst_t* dev, uint8_t reg, uint16_t data);
static uint32_t LTC3337_VBatTo_mV(uint16_t rawValue);
static int16_t  LTC3337_TempRegToC(uint8_t rawValue);
static uint8_t  LTC3337_TempC_ToReg(int16_t tempC);
static void     LTC3337_ChargeCountToA(uint16_t chargeCount, uint8_t prescale, 
                                       uint32_t lsbnA,       charge_count_t* value);
static uint16_t LTC3337_A_ToChargeCount(uint8_t prescale, uint32_t lsbnA,       
                                        charge_count_t* chargeA);                                       



int32_t LTC3337_Init(ltc3337_inst_t* dev, i2c_device_t* i2cHandle, uint8_t prescale)
{   
    int32_t ret = 0;
    uint16_t regVal;

    assert(dev);
    assert(i2cHandle);

    //Assign the I2C instance
    dev->i2cHandle = i2cHandle;
    
    ret = LTC3337_ReadReg(dev, LTC3337_REG_C, &regVal);
    if( ret == 0 ) //Success
    {
        dev->iPeakLatched = (regVal >> LTC3337_RC_IPK_PIN_POS) & LTC3337_RC_IPK_PIN_MSK;

        regVal = (prescale & LTC3337_RA_PRESCALE_MSK) << LTC3337_RA_PRESCALE_POS;
        regVal |= (0xFF & LTC3337_RA_ALARM_LVL_MSK) << LTC3337_RA_ALARM_LVL_POS;
        dev->latchedRegA = regVal;
        ret = LTC3337_WriteReg(dev, LTC3337_REG_A, regVal);
    }

    return ret;
}


int32_t LTC3337_GetVoltage_mV(ltc3337_inst_t* dev, ltc3337_voltage_src_t source, 
                              uint32_t* value )
{
    uint8_t reg;
    uint16_t regValue;
    int32_t ret = 0;

    assert(dev);
    assert(value);

    switch(source)
    {
        case BAT_IN_IPEAK_ON:
            reg = LTC3337_REG_D;
            break;
        case BAT_IN_IPEAK_OFF:
            reg = LTC3337_REG_E;
            break;
        case BAT_OUT_IPEAK_ON:
            reg = LTC3337_REG_F;
            break;            
        case BAT_OUT_IPEAK_OFF:
            reg = LTC3337_REG_G;
            break;            
        default:
            ret = -1;
            break;
    }

    if( ret == 0 )
    {
        ret = LTC3337_ReadReg(dev, reg, &regValue);
        if( ret == 0 )
        {
            *value = LTC3337_VBatTo_mV(regValue);
        }
    }

    return ret;
}


int32_t LTC3337_GetTemperatureC(ltc3337_inst_t* dev, int16_t* value )
{
    uint16_t regValue;
    int32_t ret = 0;

    assert(dev);
    assert(value);

    ret = LTC3337_ReadReg(dev, LTC3337_REG_C, &regValue);
    if( ret == 0 )
    {
        *value = LTC3337_TempRegToC((regValue >> LTC3337_RC_DIE_TEMP_POS) & LTC3337_RC_DIE_TEMP_MSK);
    }

    return ret;
}


int32_t LTC3337_GetAccumulatedCharge(ltc3337_inst_t* dev, charge_count_t* value, 
                                     uint16_t* rawValue )
{
    uint16_t regValue;
    uint8_t prescale;

    int32_t ret;
    assert(dev);

    ret = LTC3337_ReadReg(dev, LTC3337_REG_B, &regValue);

    if( ret == 0 )
    {
        if( rawValue != NULL )
        {
            *rawValue = regValue;
        }

        if( value != NULL )
        {
            prescale = (dev->latchedRegA >> LTC3337_RA_PRESCALE_POS) & LTC3337_RA_PRESCALE_MSK;
            LTC3337_ChargeCountToA(regValue, prescale, 
                                   iPeakLookupTable[dev->iPeakLatched].qLsb_nA_hr, 
                                   value);
        }        
    }

    return ret;
}


int32_t LTC3337_SetPrescaler(ltc3337_inst_t* dev, uint8_t prescale)
{
    uint16_t regValue;
    int32_t ret;
    assert(dev);

    regValue = dev->latchedRegA;
    regValue &= ~(LTC3337_RA_PRESCALE_MSK << LTC3337_RA_PRESCALE_POS);
    regValue |= ((prescale & LTC3337_RA_PRESCALE_MSK) << LTC3337_RA_PRESCALE_POS);
    ret = LTC3337_WriteReg(dev, LTC3337_REG_A, regValue);
    if( ret == 0 )
    {
        dev->latchedRegA = regValue;
    }

    return ret;
}


int32_t LTC3337_SetTemperatureAlarmsC(ltc3337_inst_t* dev, int16_t hotAlarm, 
                                      int16_t coldAlarm )
{
    uint16_t regVal;
    uint8_t tempVal;

    assert(dev);

    if((hotAlarm < LTC3337_MIN_TEMP_C) || (hotAlarm > LTC3337_MAX_TEMP_C) ||
       (coldAlarm < LTC3337_MIN_TEMP_C) || (coldAlarm > LTC3337_MAX_TEMP_C))
    {
        return -1;
    }

    tempVal = LTC3337_TempC_ToReg(hotAlarm);
    regVal = (tempVal & LTC3337_RH_TEMP_MSK) << LTC3337_RH_HOT_ALARM_POS;
    tempVal = LTC3337_TempC_ToReg(coldAlarm);
    regVal |= (tempVal & LTC3337_RH_TEMP_MSK) << LTC3337_RH_COLD_ALARM_POS;

    return LTC3337_WriteReg(dev, LTC3337_REG_H, regVal);
}


int32_t LTC3337_SetCounterShutdown(ltc3337_inst_t* dev, uint8_t shutdownEn )
{
    uint16_t regValue;
    int32_t ret;
    
    assert(dev);

    regValue = dev->latchedRegA;

    if( shutdownEn )
    {
        regValue |= LTC3337_RA_SHTDN;
    }
    else
    {
        regValue &= ~(LTC3337_RA_SHTDN);
    }

    ret = LTC3337_WriteReg(dev, LTC3337_REG_A, regValue);
    if( ret == 0 )
    {
        dev->latchedRegA = regValue;
    }

    return ret;
}


int32_t LTC3337_SetCounterAlarm(ltc3337_inst_t* dev, uint16_t counterRegVal, 
                                uint8_t roundUp )
{
    uint8_t dataToSet;
    uint16_t regValue;
    int32_t ret;
    
    assert(dev);

    regValue = dev->latchedRegA;

    //Only the upper 8 bits are utilized
    dataToSet = (counterRegVal >> 8);

    if(( roundUp ) && (counterRegVal & 0xFF) && (dataToSet != 0xFF))
    {
        dataToSet++;
    }
    
    regValue &= ~(LTC3337_RA_ALARM_LVL_MSK << LTC3337_RA_ALARM_LVL_POS);
    regValue |= ((dataToSet & LTC3337_RA_ALARM_LVL_MSK) << LTC3337_RA_ALARM_LVL_POS);
    
    ret = LTC3337_WriteReg(dev, LTC3337_REG_A, regValue);
    if( ret == 0 )
    {
        dev->latchedRegA = regValue;
    }

    return ret;
}


int32_t LTC3337_SetAccumulatedCharge(ltc3337_inst_t* dev, uint16_t regValue, 
                                     uint8_t roundUp )
{
    uint16_t dataToSet;
    
    assert(dev);

    //Only upper 8-bits are writable
    dataToSet = regValue & 0xFF00;

    if(( roundUp ) && (regValue & 0xFF) && (dataToSet != 0xFF00))
    {
        dataToSet += 0x100;
    }

    return LTC3337_WriteReg(dev, LTC3337_REG_B, dataToSet);
}


int32_t LTC3337_GetAndClearInterrupts(ltc3337_inst_t* dev, uint8_t* intField,
                                      int16_t* tempC )
{
    uint16_t intRegValue;
    uint16_t writeRegValue;
    int32_t ret = 0;

    assert(dev);
    assert(intField);

    ret = LTC3337_ReadReg(dev, LTC3337_REG_C, &intRegValue);
    if( ret == 0 )
    {
        writeRegValue = dev->latchedRegA;
        writeRegValue |= (LTC3337_RA_CLEAR_INT);
        ret = LTC3337_WriteReg(dev, LTC3337_REG_A, writeRegValue);

        *intField = 0;
        *intField |= (intRegValue & LTC3337_RC_ADC_READY) ? LTC3337_INT_ADC_READY : 0;
        *intField |= (intRegValue & LTC3337_RC_ALARM_MAX) ? LTC3337_INT_HIGH_TEMP : 0;
        *intField |= (intRegValue & LTC3337_RC_ALARM_MIN) ? LTC3337_INT_LOW_TEMP : 0;
        *intField |= (intRegValue & LTC3337_RC_ALARM_TRIP) ? LTC3337_INT_COUNT_ALARM : 0;
        *intField |= (intRegValue & LTC3337_RC_OVERFLOW) ? LTC3337_INT_OVERFLOW : 0;
        if( tempC != NULL )
        {
            *tempC = LTC3337_TempRegToC((intRegValue >> LTC3337_RC_DIE_TEMP_POS) & LTC3337_RC_DIE_TEMP_MSK);
        }
    }

    return ret;    
}


int32_t LTC3337_CalculateChargeRegister(ltc3337_inst_t* dev, 
                                        charge_count_t* chargeA, 
                                        uint16_t* regValue )
{
    uint8_t prescale;
    
    assert(dev);
    assert(chargeA);
    assert(regValue);
    prescale = (dev->latchedRegA >> LTC3337_RA_PRESCALE_POS) & LTC3337_RA_PRESCALE_MSK;
    *regValue = LTC3337_A_ToChargeCount(prescale, 
                                        iPeakLookupTable[dev->iPeakLatched].qLsb_nA_hr, 
                                        chargeA);

    return 0;
}

/**
 * Reads a register from the device.
 * @param dev - Device instance
 * @param reg - Register to read
 * @param data - Data storage location
 * @returns 0 on success, else on error
 */
static int32_t LTC3337_ReadReg(ltc3337_inst_t* dev, uint8_t reg, uint16_t* data)
{
    uint8_t regBuf = reg;
    uint8_t dataBuf[2];
    int32_t ret;

    assert(dev);
    assert(data);    
    
    ret = Platform_i2cWriteRead(dev->i2cHandle, LTC3337_I2C_ADDR, &regBuf, 1, dataBuf, 2, 0 );

    //Data comes Little Endian
    *data = ((uint16_t)dataBuf[0]) | (((uint16_t)dataBuf[1]) << 8);

    return ret;
}

/**
 * Writes a register ont the device.
 * @param dev - Device instance
 * @param reg - Register to write
 * @param data - Data to write
 * @returns 0 on success, else on error
 */
static int32_t LTC3337_WriteReg(ltc3337_inst_t* dev, uint8_t reg, uint16_t data)
{
    uint8_t dataBuf[3];

    assert(dev);
    
    //Register followed by Little Endian 16-bit word
    dataBuf[0] = reg;
    dataBuf[1] = (uint8_t)(data & 0xFF);
    dataBuf[2] = (uint8_t)((data >> 8) & 0xFF);

    return Platform_i2cWrite(dev->i2cHandle, LTC3337_I2C_ADDR, dataBuf, 3 );
}


/**
 * Converts a raw register value for battery voltage to
 * millivolts
 * @param rawValue - Value to convert
 * @returns Converted value in millivolts
 */
static uint32_t LTC3337_VBatTo_mV(uint16_t rawValue)
{
    //uV to mV, rounded to nearest mV
    return (((uint32_t)rawValue * LTC3337_VLSB_uV) + 500) / 1000; 
}

/**
 * Converts a raw register value for temperature to
 * Degrees C
 * @param rawValue - Value to convert
 * @returns Converted value in Degrees C
 */
static int16_t LTC3337_TempRegToC(uint8_t rawValue)
{
    //Rounded to the nearest C
    return (int16_t)(((rawValue * LTC3337_TLSB_mC) + 500) / 1000) + LTC3337_TEMP_MIN_C;
}

/**
 * Converts a temperature in Deg C to the parts register
 * equivalent
 * @param tempC - Temperature in C
 * @returns Converted register value
*/
static uint8_t LTC3337_TempC_ToReg(int16_t tempC)
{
    int32_t working;

    if( tempC > LTC3337_MAX_TEMP_C )
    {
        return 0xFF;
    }
    else if( tempC < LTC3337_MIN_TEMP_C )
    {
        return 0x00;
    }
    else
    {
        working = (tempC - LTC3337_TEMP_MIN_C) * 1000;
        //Round to the nearest
        working = (working + (LTC3337_TLSB_mC / 2)) / LTC3337_TLSB_mC;
        return (uint8_t)working;
    }
}

/**
 * Converts a charge count register to accumulated charge in
 * A/nA based on the prescaler qLSB values
 * @param chargeCount - Charge count register value
 * @param prescale - Prescale value
 * @param lsbnA - LSB value in Nano-amps
 * @param retValue - Pointer to return structure
  */
static void LTC3337_ChargeCountToA(uint16_t chargeCount, uint8_t prescale, 
                                   uint32_t lsbnA,       charge_count_t* retValue)
{
    uint32_t qlsb_m_nA;
    uint32_t qlsb_m_remainder;
    uint32_t wholeCount, fractCount;

    assert(retValue);

    //Per the data sheet qLSB_M = (qLSB / 2*M)
    //We can get qLSB_M by doing (qLSB >> M);
    qlsb_m_nA = (lsbnA >> prescale);

    //Get the fractional bits we shifted out
    qlsb_m_remainder = lsbnA & ((1 << prescale) - 1);

    //In an effort to keep all the math 32-bit integer based, avoiding floats, 
    //to be portable across differnt systems:
    //Let's convert our qlsb_m_nA to whole and fractional parts
    //This is to prevent an overflow during the math.
    //Worst case is 14910000 nA with a prescaler of 0. If we had a full counter
    //(0xFFFF), multiplication works out to 0xE3_814C_7DD0, 32-bit overflow.
    //Using base-2 components is easier programming wise, but using base-10 
    //helps track the units better:
    //Divisor of 10,000 (10uA per whole value, 1nA per fract Value)
    //  - 14910000 / 10000 = 1491 * 0xFFFF (full counter) = 0x5D2_FA2D
    //  - 9,999 (Wost case remainder) * 0xFFFF = 0x270E_D8F1

    wholeCount = (uint32_t)chargeCount * (qlsb_m_nA / LTC3337_CALC_SCALE);
    fractCount = (uint32_t)chargeCount * (qlsb_m_nA % LTC3337_CALC_SCALE);

    retValue->A_hr = wholeCount / LTC3337_CALC_TO_WHOLE; //10uA per bit,  / 100,000 to get A
    retValue->nA_hr = (wholeCount % LTC3337_CALC_TO_WHOLE) * LTC3337_CALC_SCALE; //*10,000 nA per bit
    retValue->nA_hr += fractCount; //Add in the other nA's

    //Add back in the remainder after shifting out the prescaler
    //Each LSB is 1/2^M nA
    fractCount = qlsb_m_remainder * chargeCount;
    retValue->nA_hr += fractCount >> prescale;

    //If the nA-hr component overflowed 1A, adjust accordingly
    while( retValue->nA_hr > LTC3337_NANO_AMP)
    {
        retValue->A_hr++;
        retValue->nA_hr -= LTC3337_NANO_AMP;
    }
}
 

/**
 * Converts a charge in A/nA-hr to an equivalent count register 
 * value to be used for setting alarms and clearing the register
 * @param prescale - Prescale value
 * @param lsbnA - LSB value in Nano-amps
 * @param chargeA - Pointer to the charge value in A/nA-hrs
 * @returns Equivalent register value based on parameters
 */
static uint16_t LTC3337_A_ToChargeCount(uint8_t prescale, uint32_t lsbnA,       
                                        charge_count_t* chargeA)
{
    uint32_t qlsb_m_nA;
    uint32_t qlsb_m_remainder;
    uint32_t bitsPerA = 0;
    uint32_t extraPerA = 0;   
    uint32_t workingCount = 0;
    uint32_t remainderCount = 0;
    uint32_t fractCount = 0;

    assert(chargeA);

    //Per the data sheet qLSB_M = (qLSB / 2*M)
    //We can get qLSB_M by doing (qLSB >> M);
    qlsb_m_nA = (lsbnA >> prescale);

    //Get the fractional bits we shifted out
    qlsb_m_remainder = lsbnA & ((1 << prescale) - 1);

    //Calculate the number of bits for each A-hr
    bitsPerA = LTC3337_NANO_AMP / qlsb_m_nA;
    extraPerA = LTC3337_NANO_AMP % qlsb_m_nA;

    //Exceeds the bounds of the configuration, and could overflow
    //the math.  Drop out now.
    if( chargeA->A_hr > (0xFFFF / bitsPerA))
    {
        return 0xFFFF;
    }
    
    //Get the baseline count
    workingCount = (chargeA->A_hr * bitsPerA);
    workingCount += (chargeA->nA_hr / qlsb_m_nA);

    //Accumulate any remainders from the math, and add extra bits
    //if we overflow qlsb_m_nA
    remainderCount = (chargeA->A_hr * extraPerA);
    remainderCount += (chargeA->nA_hr % qlsb_m_nA);
    workingCount += (remainderCount / qlsb_m_nA);

    //Incorporate the fractional piece from the prescaler
    //Since these fractional pieces would be more divising components
    //We'll subtract the result
    fractCount = workingCount * qlsb_m_remainder;
    workingCount -= ((fractCount / lsbnA)); //divide by lsbna to negate the prescaler
   
    if( workingCount > 0xFFFF ) //Clamp to register max
    {
        return 0xFFFF;
    }
    else
    {
        return workingCount;
    }
}
 