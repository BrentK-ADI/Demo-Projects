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
#ifndef __CONFIG_MENU_H__
#define __CONFIG_MENU_H__
#include <stddef.h>
#include <stdint.h>

#include "ConfigMenuEntry.h"

/*******************************************************************************
 * This implementation is a general purposes configuration menu.  The system
 * takes in an array of menu entries, each of which has a display string,
 * pointer to data to view/set, ranges, and other optional display parameters
 * 
 * The system will automatically list the entires, in order provided, starting
 * at 0 for menu items.  When in the menu, the operator can select an entry, 
 * then input a new value to update.  The system will automatically update the
 * data pointer after performing range checks. 
 * 
 * See ConfigMenuEntry.h for information on the entries and their data types
 */

/**
 * Structure defining the char read and write ops for the config menu. This
 * allows things to be platform independent
 */
typedef struct 
{
    char (*readChar)(void);
    void (*writeChar)(char);
} config_menu_ops_t;

/**
 * Defines the different line ending styles
 * Used for both writing data to the console and reading in lines
 */
typedef enum 
{ 
    LINE_END_CR,   // '\r'
    LINE_END_LF,   // '\n'
    LINE_END_CR_LF // '\r\n'
} config_menu_line_end_t;

/**
 * Initializes the configuration menu instance.  
 * @param ops - Character operations
 * @param le - Line endings to use
 * @param echo - 1 will echo back characters read.
*/
void ConfigMenuInit(config_menu_ops_t* ops, 
                    config_menu_line_end_t le, 
                    uint8_t echo);

/**
 * Runs the configuration menu loop on the provided entries. This is a blocking
 * call and will run the menu until the user exits it. The menu will 
 * automatically add the 'x' option to exit on all menus
 * @param entries - Menu entries to show
 * @param numEntries - Number of menu entries 
 */
void ConfigMenuRun(const char* headingString,
                   config_menu_entry_t* entries, 
                   uint8_t numEntries);

#endif
