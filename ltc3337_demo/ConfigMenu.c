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
#include <string.h>
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include "ConfigMenu.h"

//Length for any shared string buffs
#define SCRATCH_STR_LEN 128

//Text to show the the exit menu item
#define EXIT_ENTRY  " x) Exit the menu"
#define ENTRY_PROMPT "Select Entry: "
#define HEADING_DIV  "----------------------------------------"

//Saved off char ops from init
static config_menu_ops_t charOps;

//Saved off line ending from init
static config_menu_line_end_t lineEnd;

//Saved off echo from init
static uint8_t doEcho;

//Scratch buffer to keep string data off the stack
static char scratchStr[SCRATCH_STR_LEN];

/** Local Prototypes **/
static void PrintEntry(config_menu_entry_t* entry, uint8_t number);
static void GetNewValue(config_menu_entry_t* entry );
static void ConfigMenuPrintString(const char* line, uint8_t numLineEnds);
static uint32_t ConfigMenuReadLine(char* buf, uint32_t len);


void ConfigMenuInit(config_menu_ops_t* ops, config_menu_line_end_t le, uint8_t echo)
{
    assert(ops);
    assert(ops->readChar);
    assert(ops->writeChar);

    memcpy(&charOps, ops, sizeof(config_menu_ops_t));
    lineEnd = le;
    doEcho = echo;
}

void ConfigMenuRun(const char* headingText, config_menu_entry_t* entries, 
                   uint8_t numEntries)
{
    uint8_t runloop = 1;
    uint8_t checkIdx = 0;
    uint8_t entry;
    uint32_t len;
    char* chPtr;
    uint8_t isNum;

    assert(entries);

    while( runloop ) //Loop until an exit condition
    {
        //Print some new lines to clear the screen
        ConfigMenuPrintString("", 5);
        ConfigMenuPrintString(headingText, 1);
        ConfigMenuPrintString(HEADING_DIV, 1);

        //Draw all the entries
        for( int i = 0; i < numEntries; i++ )
        {
            PrintEntry(&entries[i], i);
        }
        ConfigMenuPrintString(EXIT_ENTRY, 1); //And exit
        ConfigMenuPrintString(ENTRY_PROMPT, 0); //And prompt

        //Ask for a line from the user
        len = ConfigMenuReadLine(scratchStr, SCRATCH_STR_LEN);

        checkIdx = 1; //Assume we'll check for a valid index
        if( len == 0 ) //Empty user entry.
        {
            checkIdx = 0; //Do nothing
        }
        else if( len == 1 )  //Single character
        {
            //Exit condition
            if(( scratchStr[0] == 'X') || (scratchStr[0] == 'x'))
            {
                runloop = 0;
                checkIdx = 0;
            }
        }

        //No special conditions, check for a menu item index
        if( checkIdx )
        {
            isNum = 1;
            chPtr = scratchStr;
            while( *chPtr != '\0' )
            {
                if(isdigit((int)*chPtr) == 0)
                {
                    isNum = 0;
                    break;
                }
                chPtr++;
            }

            if(isNum)
            {
                entry = strtoul(scratchStr, NULL, 10);
                if(entry < numEntries)
                {
                    if( entries[entry].readOnly)
                    {
                        ConfigMenuPrintString("Read only entry!", 1);
                    }
                    else
                    {
                        GetNewValue(&entries[entry]);
                    }
                }
            }
        }
    }
}

/**
 * Prints the provided menu entry
 * @param entry = Menu entry to print
 * @param number - Representative menu number
 */
void PrintEntry(config_menu_entry_t* entry, uint8_t number)
{
    uint32_t strIdx = 0;
    config_data_t *data = &entry->data;

    if( entry->readOnly )
    {
         //For read only, dont display the entry number, thats a hint to the user
        strIdx += snprintf(&scratchStr[strIdx], SCRATCH_STR_LEN - strIdx, "    ");
    }
    else
    {
        strIdx += snprintf(&scratchStr[strIdx], SCRATCH_STR_LEN - strIdx, "%2d) ", number);
    }

    //Show the item's text
    strIdx += snprintf(&scratchStr[strIdx], SCRATCH_STR_LEN - strIdx, "%s", entry->itemText);

    if( entry->showCurrent ) //Show the current flag is set
    {
        //Note: This switch statement implementation seems bulky, but the verbose
        //nature of the datatypes the user needs to set is probably a nicer
        //implementation than some clever casting 
        switch(entry->dataType)
        {
            case MENU_ITEM_UINT8:
                strIdx += snprintf(&scratchStr[strIdx], SCRATCH_STR_LEN - strIdx,
                    ": %u", *data->uint8_t_data.dataPtr);
                break;
            case MENU_ITEM_UINT16:
                strIdx += snprintf(&scratchStr[strIdx], SCRATCH_STR_LEN - strIdx, 
                    ": %u", *data->uint16_t_data.dataPtr);
                break;
            case MENU_ITEM_UINT32:
                strIdx += snprintf(&scratchStr[strIdx], SCRATCH_STR_LEN - strIdx, 
                    ": %u", *data->uint32_t_data.dataPtr);
                break;
            case MENU_ITEM_INT8:
                strIdx += snprintf(&scratchStr[strIdx], SCRATCH_STR_LEN - strIdx, 
                    ": %d", *data->int8_t_data.dataPtr);
                break;
            case MENU_ITEM_INT16:
                strIdx += snprintf(&scratchStr[strIdx], SCRATCH_STR_LEN - strIdx, 
                    ": %d", *data->int16_t_data.dataPtr);
                break;
            case MENU_ITEM_INT32:
                strIdx += snprintf(&scratchStr[strIdx], SCRATCH_STR_LEN - strIdx, 
                    ": %d", *data->int32_t_data.dataPtr);
                break;
#ifdef CONFIG_MENU_FLOAT                
            case MENU_ITEM_FLOAT:
                strIdx += snprintf(&scratchStr[strIdx], SCRATCH_STR_LEN - strIdx, 
                    ": %f", *data->float_t_data.dataPtr);
                break;
#endif
            case MENU_ITEM_ON_OFF:
                strIdx += snprintf(&scratchStr[strIdx], SCRATCH_STR_LEN - strIdx, 
                    ": %s", (*data->on_off_t_data.dataPtr) == 0 ? "OFF" : "ON");
                break;
            case MENU_ITEM_ACTION:
                //Do nothing for actions
                break;
        }
    }

    if( entry->showRange )
    {
        //Same note as above. 
        switch(entry->dataType)
        {
            case MENU_ITEM_UINT8:
                strIdx += snprintf(&scratchStr[strIdx], SCRATCH_STR_LEN - strIdx, 
                    " (%u-%u)", data->uint8_t_data.minVal, data->uint8_t_data.maxVal );
                break;
            case MENU_ITEM_UINT16:
                strIdx += snprintf(&scratchStr[strIdx], SCRATCH_STR_LEN - strIdx, 
                    " (%u-%u)", data->uint16_t_data.minVal, data->uint16_t_data.maxVal);
                break;
            case MENU_ITEM_UINT32:
                strIdx += snprintf(&scratchStr[strIdx], SCRATCH_STR_LEN - strIdx, 
                    " (%u-%u)", data->uint32_t_data.minVal, data->uint32_t_data.maxVal);
                break;
            case MENU_ITEM_INT8:
                strIdx += snprintf(&scratchStr[strIdx], SCRATCH_STR_LEN - strIdx, 
                    " (%d-%d)", data->int8_t_data.minVal, data->int8_t_data.maxVal);
                break;
            case MENU_ITEM_INT16:
                strIdx += snprintf(&scratchStr[strIdx], SCRATCH_STR_LEN - strIdx, 
                    " (%d-%d)", data->int16_t_data.minVal, data->int16_t_data.maxVal);
                break;
            case MENU_ITEM_INT32:
                strIdx += snprintf(&scratchStr[strIdx], SCRATCH_STR_LEN - strIdx, 
                    " (%d-%d)", data->int32_t_data.minVal, data->int32_t_data.maxVal);
                break;
#ifdef CONFIG_MENU_FLOAT
            case MENU_ITEM_FLOAT:
                strIdx += snprintf(&scratchStr[strIdx], SCRATCH_STR_LEN - strIdx, 
                    " (%f-%f)", data->float_t_data.minVal, data->float_t_data.maxVal);
                break;
#endif
            case MENU_ITEM_ON_OFF:
                //No min/max for OnOff
                break;
            case MENU_ITEM_ACTION:
                //Do nothing for actions
                break;
        }
    }

    ConfigMenuPrintString(scratchStr, 1);
}


/**
 * Requests a new value from the user, performs range checking, and updates
 * the data pointer if valid
 * @param entry - Entry to get a new value fro
 */
void GetNewValue(config_menu_entry_t* entry)
{
    uint32_t len;
    uint32_t tempU = 0;
    int32_t tempS = 0;
    uint8_t valid = 0;
    config_data_t* data = &entry->data;
#ifdef CONFIG_MENU_FLOAT
    float tempF = 0.0f;
#endif

    //On/Off data is a special type that just toggles. No extra user entry
    //required
    if( entry->dataType == MENU_ITEM_ON_OFF )
    {
        if(*data->on_off_t_data.dataPtr)
        {
            *data->on_off_t_data.dataPtr = 0;
        }
        else
        {
            *data->on_off_t_data.dataPtr = 1;
        }
        return;
    }

    //Actions are a special type that just calls the users callback function. No
    //extra user entry required.
    if( entry->dataType == MENU_ITEM_ACTION )
    {
        if( data->action_t_data.callback != NULL )
        {
            data->action_t_data.callback(data->action_t_data.actionData);
        }
        return;
    }
    
    ConfigMenuPrintString("Enter new value: ", 0);
    len = ConfigMenuReadLine(scratchStr, SCRATCH_STR_LEN);

    if( len == 0 )
    {
        ConfigMenuPrintString("Invalid value", 1);
        return;
    }

    //Convert the text to the appropriate type
    switch(entry->dataType)
    {
            case MENU_ITEM_UINT8:
            case MENU_ITEM_UINT16:
            case MENU_ITEM_UINT32:
                tempU = strtoul(scratchStr, NULL, 10);
                break;
            case MENU_ITEM_INT8:                
            case MENU_ITEM_INT16:
            case MENU_ITEM_INT32:
                tempS = strtol(scratchStr, NULL, 10);
                break;
#ifdef CONFIG_MENU_FLOAT      
            case MENU_ITEM_FLOAT:
                tempF = strtof(scratchStr, NULL);
                break;
#endif
            default:
                //Should never get here
                assert(0);
                break;
    }

    valid = 0;
    switch(entry->dataType)
    {
        case MENU_ITEM_UINT8:
            if((tempU >= data->uint8_t_data.minVal) && 
               (tempU <= data->uint8_t_data.maxVal))
            {
                *data->uint8_t_data.dataPtr = (uint8_t)tempU;
                valid = 1;
            }
            break;
        case MENU_ITEM_UINT16:
            if((tempU >= data->uint16_t_data.minVal) && 
               (tempU <= data->uint16_t_data.maxVal))
            {
                *data->uint16_t_data.dataPtr = (uint16_t)tempU;
                valid = 1;
            }
            break;            
        case MENU_ITEM_UINT32:
            if((tempU >= data->uint32_t_data.minVal) && 
               (tempU <= data->uint32_t_data.maxVal))
            {
                *data->uint32_t_data.dataPtr = (uint32_t)tempU;
                valid = 1;
            }
            break;            
        case MENU_ITEM_INT8:                
            if((tempS >= data->int8_t_data.minVal) && 
               (tempS <= data->int8_t_data.maxVal))
            {
                *data->int8_t_data.dataPtr = (int8_t)tempS;
                valid = 1;
            }
            break;
        case MENU_ITEM_INT16:
            if((tempS >= data->int16_t_data.minVal) && 
               (tempS <= data->int16_t_data.maxVal))
            {
                *data->int16_t_data.dataPtr = (int16_t)tempS;
                valid = 1;
            }
            break;            
        case MENU_ITEM_INT32:
            if((tempS >= data->int32_t_data.minVal) && 
               (tempS <= data->int32_t_data.maxVal))
            {
                *data->int32_t_data.dataPtr = (int32_t)tempS;
                valid = 1;
            }
            break;
#ifdef CONFIG_MENU_FLOAT
        case MENU_ITEM_FLOAT:
            if((tempF >= data->float_t_data.minVal) && 
               (tempF <= data->float_t_data.maxVal))
            {
                *data->float_t_data.dataPtr = tempF;
                valid = 1;
            }
            break;
#endif
        default:
            //Should never get here
            assert(0);
            break;
    }

    if( valid == 0 )
    {
        ConfigMenuPrintString("Value out of range", 1);
        ConfigMenuPrintString("", 1);
    }
}

/**
 * Prints the provided string and line endings to the terminal
 * @param line - Line to print (null terminated)
 * @param numLineEnds - Number of line endings to print after
 */
static void ConfigMenuPrintString(const char* line, uint8_t numLineEnds)
{
    if( line != NULL )
    {
        while(*line != '\0')
        {
            charOps.writeChar(*line);
            line++;
        }
    }

    while(numLineEnds > 0)
    {
        switch( lineEnd )
        {
            case LINE_END_CR:
                charOps.writeChar('\r');
                break;
            case LINE_END_CR_LF:
                charOps.writeChar('\r');
                charOps.writeChar('\n');
                break;
            case LINE_END_LF:
                charOps.writeChar('\n');
                break;
        }
        numLineEnds--;
    }
}

/**
 * Reads a line from the terminal, identified by the line ending type
 * @param buf- Buffer to read in to. Will be null terminated when finished
 * @param len - Length of the buffer
 * @returns Number of characters in the line, excluding line endings and null
 *          termination
 */
static uint32_t ConfigMenuReadLine(char* buf, uint32_t len)
{
    uint32_t count = 0;
    char* ptr = buf;
    uint8_t runLoop = 1;
    uint8_t leState = 0;

    //Loop leaving enough room for a null if we run out of room
    while((count < (len-1)) && (runLoop))
    {
        //Read the character
        *ptr = charOps.readChar();

        //Perform the echo if enableed
        if( doEcho )
        {
            charOps.writeChar(*ptr);
        }
        
        //Never save off the CR or LF chars. This supports the 2 character line
        //endings, by not incrementing the counter and pointer as well
        if((*ptr != '\n') && (*ptr != '\r'))
        {            
            count++;
            ptr++;
        }
        else
        {
            //Handle the line ending depending on type
            switch( lineEnd)
            {
                case LINE_END_CR:
                    if(*ptr == '\r')
                    {
                        runLoop = 0;
                    }
                    break;
                case LINE_END_CR_LF:
                    if((leState == 0 ) && (*ptr == '\r'))
                    {
                        leState = 1;
                    }
                    else if((leState == 1 ) && (*ptr == '\n'))
                    {
                        runLoop = 0;
                    }
                    else
                    {
                        leState = 0;
                    }
                    break;
                case LINE_END_LF:
                    if(*ptr == '\n')
                    {
                        runLoop = 0;
                    }
                    break;
            }
        }
    }

    //Always set the last character to null
    *ptr = '\0';

    return count;
}
