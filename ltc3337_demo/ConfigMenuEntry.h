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
#ifndef __CONFIG_MENU_ENTRY_H__
#define __CONFIG_MENU_ENTRY_H__

#include <stdint.h>
#include <stddef.h>

//Comment out/undefine CONFIG_MENU_FLOAT to disable support for floating point
//menu items.  This is important on memory constrained embedded systems, the
//string conversion and formatting associated with floats is a lot of overhead,
//if you're not using them, don't inlude this to prevent unnecessary stuff from
//getting linked in
//#define CONFIG_MENU_FLOAT 

/**
 * Enumeration of possible data types.
 */
typedef enum 
{
    MENU_ITEM_UINT8,    //Unsigned 8-bit
    MENU_ITEM_UINT16,   //Unsigned 16-bit
    MENU_ITEM_UINT32,   //Unsigned 32-bit
    MENU_ITEM_INT8,     //Signed 8-bit
    MENU_ITEM_INT16,    //Signed 16-bit
    MENU_ITEM_INT32,    //Signed 32-bit
#ifdef CONFIG_MENU_FLOAT
    MENU_ITEM_FLOAT,    //32-bit Float
#endif
    MENU_ITEM_ON_OFF,   //Boolean, displayed as On/Off
    MENU_ITEM_ACTION    //Action only, no value changes
} config_menu_type_t;

/**
 * Data structure for Uint8's
 */
typedef struct 
{
    uint8_t* dataPtr;
    uint8_t  minVal;
    uint8_t  maxVal;
} config_uint8_t;

/**
 * Data structure for uint16's
*/
typedef struct 
{
    uint16_t* dataPtr;
    uint16_t  minVal;
    uint16_t  maxVal;
} config_uint16_t;

/**
 * Data structure for uint32's
 */
typedef struct 
{
    uint32_t* dataPtr;
    uint32_t  minVal;
    uint32_t  maxVal;
} config_uint32_t;

/**
 * Data structure for int8's
 */
typedef struct 
{
    int8_t* dataPtr;
    int8_t  minVal;
    int8_t  maxVal;
} config_int8_t;

/**
 * Data structure for int16's
 */
typedef struct 
{
    int16_t* dataPtr;
    int16_t  minVal;
    int16_t  maxVal;
} config_int16_t;

/**
 * Data structure for int32's
 */
typedef struct 
{
    int32_t* dataPtr;
    int32_t  minVal;
    int32_t  maxVal;
} config_int32_t;

#ifdef CONFIG_MENU_FLOAT
/**
 * Data structure for floats
 */
typedef struct 
{
    float   *dataPtr;
    float   minVal;
    float   maxVal;
} config_float_t;
#endif

/**
 * Data structure for on/off booleans
 */
typedef struct
{
    uint8_t *dataPtr;
} config_on_off_t;


/** Prototype for action item callbacks */
typedef void (*action_t_callback)(void* );

/**
 * Data structure for action items
 */
typedef struct
{
    action_t_callback callback; //Actual callback function
    void* actionData;           //Data to provide to the callback
} config_action_t;


/**
 * Union everything all together for easy storage in our entry structure
*/
typedef union 
{
    config_uint8_t  uint8_t_data;
    config_uint16_t uint16_t_data;
    config_uint32_t uint32_t_data;
    config_int8_t   int8_t_data;
    config_int16_t  int16_t_data;
    config_int32_t  int32_t_data;
#ifdef CONFIG_MENU_FLOAT
    config_float_t  float_t_data;
#endif
    config_on_off_t on_off_t_data;
    config_action_t action_t_data;
} config_data_t;

/**
 * Structure that defines the actual menu entry
 */
typedef struct 
{
    const char*        itemText;    //Display Text
    config_menu_type_t dataType;    //Data type in use
    config_data_t      data;        //Detailed data information
    uint8_t            readOnly;    //Entry is read only
    uint8_t            showCurrent; //Flag to show current value in menu 
    uint8_t            showRange;   //Flag to show the range in the menu    
} config_menu_entry_t;

#endif
