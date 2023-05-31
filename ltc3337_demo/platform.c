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
#include <stddef.h>
#include <string.h>
#include "platform.h"
#include "max32655.h"
#include "mxc_errors.h"
#include "gpio.h"
#include "mxc_delay.h"
#include "mxc_sys.h"
#include "tmr.h"

#include "uart.h"

#define PLATFORM_ELAPSED_TIMER  MXC_TMR1

static const mxc_gpio_cfg_t gpio_cfg_i2c2_vddioh = { 
            MXC_GPIO0, (MXC_GPIO_PIN_30 | MXC_GPIO_PIN_31),
            MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE,
            MXC_GPIO_VSSEL_VDDIOH };

static const mxc_gpio_cfg_t gpio_cfg_ltcirq = { 
            MXC_GPIO0, (MXC_GPIO_PIN_25),
            MXC_GPIO_FUNC_IN, MXC_GPIO_PAD_NONE,
            MXC_GPIO_VSSEL_VDDIOH };

static const mxc_gpio_cfg_t gpio_cfg_btnirq = { 
            MXC_GPIO0, (MXC_GPIO_PIN_3),
            MXC_GPIO_FUNC_IN, MXC_GPIO_PAD_PULL_UP,
            MXC_GPIO_VSSEL_VDDIO };

static const mxc_gpio_cfg_t gpio_cfg_load1 = { 
            MXC_GPIO2, (MXC_GPIO_PIN_4),
            MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE,
            MXC_GPIO_VSSEL_VDDIOH };

static const mxc_gpio_cfg_t gpio_cfg_load2 = { 
            MXC_GPIO2, (MXC_GPIO_PIN_3),
            MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE,
            MXC_GPIO_VSSEL_VDDIOH };


static isr_callback_function ltcISR_Handler = NULL;
static isr_callback_function btnISR_Handler = NULL;

static void LTC_ISR(void* userData);
static void BTN_ISR(void* userData);
static void ConsoleInit(void);

void GPIO2_Handler( )
{
    MXC_GPIO_Handler(2);
}

void GPIO0_Handler( )
{
    MXC_GPIO_Handler(0);
}

void Platform_init( )
{
    mxc_tmr_cfg_t tmrFree;

    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO0);
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO1);
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO2);

    ConsoleInit();

    MXC_I2C_Init(DEMO_I2C, 1, 0);
    MXC_I2C_SetFrequency(DEMO_I2C, 100000);
    MXC_GPIO_Config(&gpio_cfg_i2c2_vddioh); //3.3V logic for I2C    

    MXC_GPIO_Config(&gpio_cfg_ltcirq);
    MXC_GPIO_Config(&gpio_cfg_btnirq);
    
    MXC_GPIO_RegisterCallback(&gpio_cfg_ltcirq, LTC_ISR, NULL);
    MXC_GPIO_IntConfig(&gpio_cfg_ltcirq, MXC_GPIO_INT_FALLING);
    MXC_GPIO_EnableInt(gpio_cfg_ltcirq.port, gpio_cfg_ltcirq.mask);    
    NVIC_EnableIRQ(GPIO2_IRQn);
    MXC_NVIC_SetVector(GPIO2_IRQn, GPIO2_Handler);    
    
    MXC_GPIO_RegisterCallback(&gpio_cfg_btnirq, BTN_ISR, NULL);
    MXC_GPIO_IntConfig(&gpio_cfg_btnirq, MXC_GPIO_INT_FALLING);
    MXC_GPIO_EnableInt(gpio_cfg_btnirq.port, gpio_cfg_btnirq.mask);    
    NVIC_EnableIRQ(GPIO0_IRQn);
    MXC_NVIC_SetVector(GPIO0_IRQn, GPIO0_Handler);        

    Platform_setLoad1( 0 );
    Platform_setLoad2( 0 );

    MXC_GPIO_Config(&gpio_cfg_load1);
    MXC_GPIO_Config(&gpio_cfg_load2);

    uint32_t periodTicks = MXC_TMR_GetPeriod(PLATFORM_ELAPSED_TIMER, MXC_TMR_APB_CLK, 2, 1000000);
    tmrFree.pres = TMR_PRES_2;
    tmrFree.mode = TMR_MODE_CONTINUOUS;
    tmrFree.bitMode = TMR_BIT_MODE_32;
    tmrFree.clock = MXC_TMR_APB_CLK;
    tmrFree.cmp_cnt = periodTicks;
    tmrFree.pol = 0;
    MXC_TMR_Init(PLATFORM_ELAPSED_TIMER, &tmrFree, false);
}

int32_t Platform_i2cRead(i2c_device_t* dev, uint8_t addr, uint8_t* readBuf, uint8_t count)
{
    return Platform_i2cWriteRead(dev, addr, NULL, 0, readBuf, count, 0);
}

int32_t Platform_i2cWrite(i2c_device_t* dev, uint8_t addr, uint8_t* writeBuf, uint8_t count)
{
    return Platform_i2cWriteRead(dev, addr, writeBuf, count, NULL, 0, 0 );
}

int32_t Platform_i2cWriteRead(i2c_device_t* dev, uint8_t addr, 
                              uint8_t* writeBuf, uint8_t writeCount,
                              uint8_t* readBuf, uint8_t readCount,
                              uint8_t repeatStart )
{
    mxc_i2c_req_t reqMaster;
    reqMaster.i2c = dev;
    reqMaster.addr = addr;
    reqMaster.tx_buf = writeBuf;
    reqMaster.tx_len = writeCount;
    reqMaster.rx_buf = readBuf;
    reqMaster.rx_len = readCount;
    reqMaster.restart = repeatStart;   

    if( MXC_I2C_MasterTransaction(&reqMaster) == 0 )
    {
        return 0;
    }
    else
    {
        return -1;
    }
}

void Platform_delay(uint32_t ms)
{
    MXC_Delay(MXC_DELAY_MSEC(ms));
}


void Platform_setLoad1(uint8_t en)
{
    if( en )
    {
        MXC_GPIO_OutSet(gpio_cfg_load1.port, gpio_cfg_load1.mask);
    }
    else
    {
        MXC_GPIO_OutClr(gpio_cfg_load1.port, gpio_cfg_load1.mask);
    }
}

void Platform_setLoad2(uint8_t en)
{
    if( en )
    {
        MXC_GPIO_OutSet(gpio_cfg_load2.port, gpio_cfg_load2.mask);
    }
    else
    {
        MXC_GPIO_OutClr(gpio_cfg_load2.port, gpio_cfg_load2.mask);
    }
}

void Platform_setLTC_ISR_Handler(isr_callback_function func)
{
    ltcISR_Handler = func;
}

void LTC_ISR(void* userData)
{
    if( ltcISR_Handler != NULL )
    {
        ltcISR_Handler();
    }
}

void Platform_setButtonISR_Handler(isr_callback_function func)
{  
    btnISR_Handler = func;
}

void BTN_ISR(void* userData)
{
    if( btnISR_Handler != NULL )
    {
        btnISR_Handler();
    }
}

void ConsoleInit( )
{
    mxc_uart_regs_t *ConsoleUart = MXC_UART_GET_UART(CONSOLE_UART);
    MXC_UART_Init(ConsoleUart, CONSOLE_BAUD, MXC_UART_IBRO_CLK);
}

void Platform_writeString(char* str)
{
    while(*str != '\0')
    {
        Platform_writeChar(*str);
        str++;
    }
}

void Platform_writeChar(char ch)
{
    while(MXC_UART_WriteCharacter(MXC_UART_GET_UART(CONSOLE_UART), ch) == E_OVERFLOW){}
}

char Platform_readChar(void)
{
    return (char)MXC_UART_ReadCharacter(MXC_UART_GET_UART(CONSOLE_UART));
}

void Platform_timerStart( )
{
    MXC_TMR_SW_Start(PLATFORM_ELAPSED_TIMER);
}

uint32_t Platform_timerStop_uS( )
{
    return MXC_TMR_SW_Stop(PLATFORM_ELAPSED_TIMER);
}