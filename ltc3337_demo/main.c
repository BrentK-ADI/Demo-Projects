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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "platform.h"
#include "LTC3337.h"
#include "ConfigMenu.h"
#include "crc.h"
#include "flc.h"


//Define the length of our temporary strings
#define SCRATCH_STR_LEN 128 //Adjust if necessary

//Fixed value to signify the start of our config
#define CFG_START_WORD  0xA55A5AA5

//Polynomial for CRC calculation, CRC-32
#define CFG_CRC_POLY 0xEDB88320

/**
 * Structure for holding the configuration parameters for the demo
 */
typedef struct 
{
    uint32_t startWord;      //Start word to signify our block
    uint32_t load1onTimeMs;  //On time for Load #1
    uint32_t load1IdleMs;    //Idle time between Load 1 and 2
    uint32_t load2onTimeMs;  //On time for Load #2
    uint32_t load2IdleMs;    //Idle time between Load 2 and 1&2
    uint32_t load12onTimeMs; //Om time for load #1&2
    uint32_t endIdleMs;      //Idle time before running loads again
    uint8_t  devicePrescale; //Prescaler for the LTC3337
    uint8_t  pad[3];         //Padding for 4-byte alignment
    uint32_t cfgCrc;         //Checksum to validate a config block
} ltc3337_demo_config_t;


/**
 * State machine states for executing the load loop
 */
typedef enum 
{ 
    STATE_LOAD1_SETUP,
    STATE_LOAD1, 
    STATE_LOAD1_IDLE,
    STATE_LOAD2_SETUP, 
    STATE_LOAD2, 
    STATE_LOAD2_IDLE,
    STATE_LOAD1_2_SETUP,
    STATE_LOAD1_2,
    STATE_END_IDLE 
} ltc3337_demo_state_t;


/**
 * Structure for maintaining stats during a load on cycle
 */
typedef struct
{
    charge_count_t startCharge; //Count at the start of cycle
    charge_count_t endCharge;   //Count at the end of cycle
    uint32_t       timeuS;      //Cycle time in microseconds
}load_stats_t;


/**
 * Defines the default values if there is not existing parameters in flash 
 */
static const ltc3337_demo_config_t defaultConfig = {
    .devicePrescale = 10,
    .endIdleMs = 60000,
    .load1onTimeMs = 30000,
    .load1IdleMs = 1000,
    .load2onTimeMs = 3000,
    .load12onTimeMs = 0,
    .load2IdleMs = 0,
};


//Active running configuration
static ltc3337_demo_config_t activeConfig;

//Our actual instance of the LTC3337 chip
static ltc3337_inst_t ltc3337_inst;

//String buffer, keeps it off the stack
static char scratchStr[SCRATCH_STR_LEN];

//Define our Flash area for storing the configuration information
extern int __cfg_nvm_start__;
#define CFG_FLASH_ADDR ((uint32_t)(&__cfg_nvm_start__))

//ISR Flags
static volatile int gotBtn = 0;
static volatile int gotIsr = 0;

/** Local Prototypes */
static void LTC_ISR_Handler( void );
static void BTN_ISR_Handler( void );
static void PrintDetails(load_stats_t* loadStats );
static void RunConfigMenu( void );
static void RunDemoLoop(void);
static void LoadConfiguration( ltc3337_demo_config_t* destCfg);
static void SaveConfiguration( ltc3337_demo_config_t* destCfg);

/**
 * Main entry point into our application. Runs the state machine
 */
int main( )
{
    //Initialize the platform
    Platform_init();

    //Register our handlers for button and LTC3337 interrupts
    Platform_setLTC_ISR_Handler(LTC_ISR_Handler);
    Platform_setButtonISR_Handler(BTN_ISR_Handler);

    while( 1 )
    {
        RunDemoLoop();
    }

    return 0;
}


static void RunDemoLoop( )
{
    uint8_t ints;
    uint8_t runLoop = 1;
    uint8_t printState = 0;
    uint32_t stateTicks;
    ltc3337_demo_state_t currentState;
    load_stats_t loadStats;

    //Load the configuration from flash
    LoadConfiguration(&activeConfig);

    //Initialize the LTC3337 and clear any interruptws
    LTC3337_Init(&ltc3337_inst, DEMO_I2C, activeConfig.devicePrescale);    
    LTC3337_GetAndClearInterrupts(&ltc3337_inst, &ints, NULL);

    //Initialize the state machine
    currentState = STATE_LOAD1_SETUP;
    stateTicks = 0;

    while( runLoop ) //Loop forever
    {
        printState = 0;

        switch(currentState)
        {
            case STATE_LOAD1_SETUP:
                //The setup state allows us to bypass the load so its not 
                //switced on for 1 cycle when executing the state machine
                if( activeConfig.load1onTimeMs )
                {
                    Platform_writeString("Starting Load 1...\n");
                    LTC3337_GetAccumulatedCharge(&ltc3337_inst, &loadStats.startCharge, NULL);
                    Platform_timerStart();
                    Platform_setLoad1( 1 );
                    stateTicks = activeConfig.load1onTimeMs;
                    currentState = STATE_LOAD1;
                }
                else
                {
                    stateTicks = activeConfig.load1IdleMs;
                    currentState = STATE_LOAD1_IDLE;
                }
                break;
            case STATE_LOAD1:
                if( stateTicks == 0 )
                {
                    Platform_setLoad1( 0 );
                    loadStats.timeuS = Platform_timerStop_uS();
                    LTC3337_GetAccumulatedCharge(&ltc3337_inst, &loadStats.endCharge, NULL);
                    stateTicks = activeConfig.load1IdleMs;
                    currentState = STATE_LOAD1_IDLE;
                    Platform_writeString("Load 1 Complete...\n");
                    printState = 1;
                }                                
                break;
            case STATE_LOAD1_IDLE:
                if( stateTicks == 0 )
                {
                    currentState = STATE_LOAD2_SETUP;
                }
                break;
            case STATE_LOAD2_SETUP:
                //The setup state allows us to bypass the load so its not 
                //switced on for 1 cycle when executing the state machine
                if( activeConfig.load2onTimeMs )
                {
                    Platform_writeString("Starting Load 2...\n");
                    LTC3337_GetAccumulatedCharge(&ltc3337_inst, &loadStats.startCharge, NULL);
                    Platform_timerStart();
                    Platform_setLoad2( 1 );
                    stateTicks = activeConfig.load2onTimeMs;
                    currentState = STATE_LOAD2;
                }
                else
                {
                    stateTicks = activeConfig.load2IdleMs;
                    currentState = STATE_LOAD2_IDLE;
                }
                break;
            case STATE_LOAD2:
                if( stateTicks == 0 )
                {
                    Platform_setLoad2( 0 );
                    loadStats.timeuS = Platform_timerStop_uS();
                    LTC3337_GetAccumulatedCharge(&ltc3337_inst, &loadStats.endCharge, NULL);
                    stateTicks = activeConfig.load2IdleMs;
                    currentState = STATE_LOAD2_IDLE;
                    Platform_writeString("Load 2 Complete...\n");
                    printState = 1;
                }                     
                break;
            case STATE_LOAD2_IDLE:
                if( stateTicks == 0 )
                {
                    currentState = STATE_LOAD1_2_SETUP;
                }            
                break;
            case STATE_LOAD1_2_SETUP:
                //The setup state allows us to bypass the load so its not 
                //switced on for 1 cycle when executing the state machine
                if( activeConfig.load12onTimeMs )
                {
                    Platform_writeString("Starting Loads 1&2...\n");
                    LTC3337_GetAccumulatedCharge(&ltc3337_inst, &loadStats.startCharge, NULL);
                    Platform_timerStart();                    
                    Platform_setLoad1( 1 );
                    Platform_setLoad2( 1 );
                    stateTicks = activeConfig.load12onTimeMs;
                    currentState = STATE_LOAD1_2;
                }
                else
                {
                    stateTicks = activeConfig.endIdleMs;
                    currentState = STATE_END_IDLE;
                }            
                break;
            case STATE_LOAD1_2:
                if( stateTicks == 0 )
                {
                    Platform_setLoad1( 0 );
                    Platform_setLoad2( 0 );
                    loadStats.timeuS = Platform_timerStop_uS();
                    LTC3337_GetAccumulatedCharge(&ltc3337_inst, &loadStats.endCharge, NULL);                    
                    stateTicks = activeConfig.endIdleMs;
                    currentState = STATE_END_IDLE;
                    Platform_writeString("Loads 1&2 Complete...\n");
                    printState = 1;
                }                                 
                break;
            case STATE_END_IDLE:
                if( stateTicks == 0 )   
                {
                    currentState = STATE_LOAD1_SETUP;
                }            
                break;
        }

        if( stateTicks )
        {
            stateTicks--;
        }


        if( printState )
        {
            //The print mechanism will take some time, probably longer than 1ms
            //But timing is not super critical here for demo purposes. A more 
            //time critical implementation would do things async
            PrintDetails(&loadStats);
        }
        else
        {
            Platform_delay(1); //Delay 1 ms between loop iterations
        }


        if( gotIsr )
        {
            gotIsr = 0;
            LTC3337_GetAndClearInterrupts(&ltc3337_inst, &ints, NULL);
            Platform_writeString("Got ISR!!!\n");
            /*******
             * This demo does not utilize the interrupt capabilities, however
             * is fully supported by the LTC3337 driver and this hardware. Add
             * Code here as necessary to read the interrupts on a LTC3337 
             * interrupt and process accordingly.
             ******/
        }

        if( gotBtn )
        {
            gotBtn = 0;
            //Disable the loads before entering the console
            Platform_setLoad1( 0 );
            Platform_setLoad2( 0 );
            Platform_writeString("Entering console...\n");
            RunConfigMenu();
            runLoop = 0; //Exit the loop so we restart with new values
        }
    }
}




static void PrintDetails(load_stats_t* loadStats )
{
    charge_count_t accCharge;
    uint32_t tempV;
    uint16_t rawCharge;
    int16_t tempC;
    uint32_t deltaCharge;
    float estLoad;
    float runTimeHours;

    LTC3337_GetTemperatureC(&ltc3337_inst, &tempC);
    snprintf(scratchStr, SCRATCH_STR_LEN, "\nTemp: %d C\n", tempC);
    Platform_writeString(scratchStr);
    LTC3337_GetVoltage_mV(&ltc3337_inst, BAT_IN_IPEAK_ON, &tempV);
    snprintf(scratchStr, SCRATCH_STR_LEN, "Vin_On: %d, ", tempV);
    Platform_writeString(scratchStr);
    LTC3337_GetVoltage_mV(&ltc3337_inst, BAT_IN_IPEAK_OFF, &tempV);
    snprintf(scratchStr, SCRATCH_STR_LEN, "Vin_Off: %d, ", tempV);        
    Platform_writeString(scratchStr);
    LTC3337_GetVoltage_mV(&ltc3337_inst, BAT_OUT_IPEAK_ON, &tempV);
    snprintf(scratchStr, SCRATCH_STR_LEN, "VOut_On: %d, ", tempV);
    Platform_writeString(scratchStr);
    LTC3337_GetVoltage_mV(&ltc3337_inst, BAT_OUT_IPEAK_OFF, &tempV);
    snprintf(scratchStr, SCRATCH_STR_LEN, "VOut_Off: %d\n", tempV);                
    Platform_writeString(scratchStr);
    LTC3337_GetAccumulatedCharge(&ltc3337_inst, &accCharge, &rawCharge);
    snprintf(scratchStr, SCRATCH_STR_LEN, "Acc %d.%09d A-hr (0x%X)\n\n", accCharge.A_hr, accCharge.nA_hr, rawCharge);
    Platform_writeString(scratchStr);

    deltaCharge = (loadStats->endCharge.A_hr - loadStats->startCharge.A_hr) * 1000000000;
    if( loadStats->endCharge.nA_hr < loadStats->startCharge.nA_hr )
    {
        deltaCharge -= (loadStats->startCharge.nA_hr - loadStats->endCharge.nA_hr);
    }
    else
    {
        deltaCharge += (loadStats->endCharge.nA_hr - loadStats->startCharge.nA_hr);
    }

    snprintf(scratchStr, SCRATCH_STR_LEN, "Run Time %f mS\n", ((float)loadStats->timeuS) / 1000.0f);
    Platform_writeString(scratchStr);


    snprintf(scratchStr, SCRATCH_STR_LEN, "Diff %d nA-hr\n", deltaCharge);
    Platform_writeString(scratchStr);

    runTimeHours = ((float)loadStats->timeuS) / (60.0f * 60.0f * 1000000.0f);
    estLoad = (((float)deltaCharge) / runTimeHours) / 1000000.0f;
    snprintf(scratchStr, SCRATCH_STR_LEN, "Avg Load: %f mA\n",estLoad);
    Platform_writeString(scratchStr);
    Platform_writeString("--------------------\n\n");
}

static void LTC_ISR_Handler( )
{
    gotIsr = 1;
}

static void BTN_ISR_Handler( )
{
    gotBtn = 1;
}



static void LoadConfiguration( ltc3337_demo_config_t* destCfg)
{
    mxc_crc_req_t crcReq;
    uint8_t flashValid = 0;
    ltc3337_demo_config_t* flashCfgPtr = (ltc3337_demo_config_t*)CFG_FLASH_ADDR;
    
    MXC_CRC_Init();
    MXC_CRC_SetPoly(CFG_CRC_POLY);

    flashValid = 0; //Assume invalid
    if( flashCfgPtr->startWord == CFG_START_WORD )
    {
        crcReq.dataBuffer = (uint32_t*)flashCfgPtr;
        crcReq.dataLen = (sizeof(ltc3337_demo_config_t) - sizeof(uint32_t)) / sizeof(uint32_t);
        
        MXC_CRC_Compute(&crcReq);

        if( crcReq.resultCRC == flashCfgPtr->cfgCrc )
        {
            flashValid = 1;
        }
    }

    if( flashValid )
    {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"        
#pragma GCC diagnostic ignored "-Wstringop-overflow"        
//Disable the array bounds checks here. Since we're getting the location of 
//the memory section from the linker by referencing it as an int, GCC warns us
//we're copying data outside the 4-byte int bounday. Thats OK.
        memcpy(destCfg, flashCfgPtr, sizeof(ltc3337_demo_config_t));
#pragma GCC diagnostic pop
    }
    else
    {
        memcpy(destCfg, &defaultConfig, sizeof(ltc3337_demo_config_t));
    }
}


static void SaveConfiguration( ltc3337_demo_config_t* destCfg)
{
    mxc_crc_req_t crcReq;
    destCfg->startWord = CFG_START_WORD;
    crcReq.dataBuffer = (uint32_t*)destCfg;
    crcReq.dataLen = (sizeof(ltc3337_demo_config_t) - sizeof(uint32_t)) / sizeof(uint32_t);

    MXC_CRC_Init();
    MXC_CRC_SetPoly(CFG_CRC_POLY);
    MXC_CRC_Compute(&crcReq);
    destCfg->cfgCrc = crcReq.resultCRC;
    
    MXC_FLC_PageErase((uint32_t)CFG_FLASH_ADDR);
    MXC_FLC_Write((uint32_t)CFG_FLASH_ADDR, sizeof(ltc3337_demo_config_t), (uint32_t*)destCfg);
}

/*******************************************************************************
 * MENU RELATED CODE
 ******************************************************************************/
#define MENU_HEADING    "LTC3337 Demo"

//Configuration used for menu operations
static ltc3337_demo_config_t menuConfig;

static void MenuSetDefaultsCallback(void* cfgData);
static void MenuResetAccumulatorCallback(void* cfgData);

//Definition of character ops for the menu
static config_menu_ops_t menuOps = {.readChar  = Platform_readChar,
                                    .writeChar = Platform_writeChar };

//Define the actual user manu entries
static config_menu_entry_t menuEntries[] = {
    { .itemText = "LTC3337 Prescaler", .dataType = MENU_ITEM_UINT8,
      .data.uint8_t_data = { .dataPtr = &menuConfig.devicePrescale, 
                             .minVal = 0, .maxVal = LTC3337_MAX_PRESCALE},
      .readOnly = 0, .showCurrent = 1, .showRange = 1},

    { .itemText = "Load 1 On time, ms", .dataType = MENU_ITEM_UINT32,
      .data.uint32_t_data = { .dataPtr = &menuConfig.load1onTimeMs, 
                              .minVal = 0, .maxVal = 0xFFFFFFFF},
      .readOnly = 0, .showCurrent = 1, .showRange = 0 },

    { .itemText = "Load 1 Idle time, ms", .dataType = MENU_ITEM_UINT32,
      .data.uint32_t_data = { .dataPtr = &menuConfig.load1IdleMs, 
                              .minVal = 0, .maxVal = 0xFFFFFFFF},
      .readOnly = 0, .showCurrent = 1, .showRange = 0 },   

    { .itemText = "Load 2 On time, ms", .dataType = MENU_ITEM_UINT32,
      .data.uint32_t_data = { .dataPtr = &menuConfig.load2onTimeMs, 
                              .minVal = 0, .maxVal = 0xFFFFFFFF},
      .readOnly = 0, .showCurrent = 1, .showRange = 0 },      

    { .itemText = "Load 2-Idle time, ms", .dataType = MENU_ITEM_UINT32,
      .data.uint32_t_data = { .dataPtr = &menuConfig.load2IdleMs, 
                              .minVal = 0, .maxVal = 0xFFFFFFFF},
      .readOnly = 0, .showCurrent = 1, .showRange = 0 },      

    { .itemText = "Load 1&2 On time, ms", .dataType = MENU_ITEM_UINT32,
      .data.uint32_t_data = { .dataPtr = &menuConfig.load12onTimeMs, 
                              .minVal = 0, .maxVal = 0xFFFFFFFF},
      .readOnly = 0, .showCurrent = 1, .showRange = 0 },      

    { .itemText = "End Idle time, ms", .dataType = MENU_ITEM_UINT32,
      .data.uint32_t_data = { .dataPtr = &menuConfig.endIdleMs, 
                              .minVal = 0, .maxVal = 0xFFFFFFFF},
      .readOnly = 0, .showCurrent = 1, .showRange = 0 }, 

    { .itemText = "Restore Defaults", .dataType = MENU_ITEM_ACTION,
      .data.action_t_data = { .callback = MenuSetDefaultsCallback,
                              .actionData = &menuConfig },
      .readOnly = 0, .showCurrent = 1, .showRange = 0 },

    { .itemText = "Reset Charge Accumulator", .dataType = MENU_ITEM_ACTION,
      .data.action_t_data = { .callback = MenuResetAccumulatorCallback,
                              .actionData = NULL },
      .readOnly = 0, .showCurrent = 1, .showRange = 0 },
};

static uint8_t numMenuEntries = sizeof(menuEntries) / sizeof(config_menu_entry_t);


static void RunConfigMenu( )
{
    memcpy(&menuConfig, &activeConfig, sizeof(ltc3337_demo_config_t));
    ConfigMenuInit(&menuOps, LINE_END_LF, 1);
    ConfigMenuRun(MENU_HEADING, menuEntries, numMenuEntries);
    SaveConfiguration(&menuConfig);
}

static void MenuSetDefaultsCallback( void* cfgData )
{
    memcpy(cfgData, &defaultConfig, sizeof(ltc3337_demo_config_t));
}

static void MenuResetAccumulatorCallback( void* cfgData )
{
    LTC3337_SetAccumulatedCharge(&ltc3337_inst, 0, 0);
}
