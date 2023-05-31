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

/* Reserve 1 page of flash for configuration data */
CFG_MEM_SIZE = 0x2000;
FLASH_MEM_SIZE = _ARM_FLASH_SIZE - _PAL_NVM_SIZE - CFG_MEM_SIZE;
PAL_NVM_ORIGIN = FLASH_MEM_SIZE + _ARM_FLASH_ORIGIN;
CFG_MEM_ORIGIN = PAL_NVM_ORIGIN + _PAL_NVM_SIZE;

/* Use leftover memory for RISCV */
RISCV_FLASH_ORIGIN = PAL_NVM_ORIGIN + _PAL_NVM_SIZE;
RISCV_FLASH_SIZE = _FLASH_SIZE - _ARM_FLASH_SIZE;

MAILBOX_ORIGIN = _ARM_SRAM_ORIGIN + _ARM_SRAM_SIZE;

RISCV_SRAM_ORIGIN = MAILBOX_ORIGIN + 2*_MAILBOX_SIZE;
RISCV_SRAM_SIZE = _SRAM_SIZE - _ARM_SRAM_SIZE - 2*_MAILBOX_SIZE;


MEMORY {
    ROM        (rx) : ORIGIN = 0x00000000,          LENGTH = 0x00010000 /* 64kB ROM */
    FLASH      (rx) : ORIGIN = _ARM_FLASH_ORIGIN,   LENGTH = FLASH_MEM_SIZE
    PAL_NVM_DB (r)  : ORIGIN = PAL_NVM_ORIGIN,      LENGTH = _PAL_NVM_SIZE
    CFG        (rw) : ORIGIN = CFG_MEM_ORIGIN,      LENGTH = CFG_MEM_SIZE
    RISCV_FLASH(rx) : ORIGIN = RISCV_FLASH_ORIGIN,  LENGTH = RISCV_FLASH_SIZE
    SRAM      (rwx) : ORIGIN = _ARM_SRAM_ORIGIN,    LENGTH = _ARM_SRAM_SIZE
    MAILBOX_0  (rw) : ORIGIN = MAILBOX_ORIGIN,      LENGTH = _MAILBOX_SIZE
    MAILBOX_1  (rw) : ORIGIN = MAILBOX_ORIGIN + _MAILBOX_SIZE, LENGTH = _MAILBOX_SIZE
    RISCV_SRAM(rwx) : ORIGIN = RISCV_SRAM_ORIGIN,   LENGTH = RISCV_SRAM_SIZE
}

SECTIONS {
    .rom :
    {
        KEEP(*(.rom_vector))
        *(.rom_handlers*)
    } > ROM

    .text :
    {
        _text = .;
        KEEP(*(.isr_vector))
        EXCLUDE_FILE (*riscv.o) *(.text*)    /* program code, exclude RISCV code */
        *(.rodata*)  /* read-only data: "const" */

        KEEP(*(.init))
        KEEP(*(.fini))

        /* C++ Exception handling */
        KEEP(*(.eh_frame*))
        _etext = .;
    } > FLASH

    /* Binary import */
    .bin_storage :
    {
       FILL(0xFF)
      _bin_start_ = .;
      KEEP(*(.bin_storage_img))
      _bin_end_ = .;
      . = ALIGN(4);
    } > FLASH
    
    .rom_code :
    {
        . = ALIGN(16);
        _sran_code = .;
        *(.rom_code_section)
        _esran_code = .;
    } > ROM

    .flash_code :
    {
        . = ALIGN(16);
        _sran_code = .;
        *(.flash_code_section)
        _esran_code = .;
    } > FLASH

    .sram_code :
    {
        . = ALIGN(16);
        _sran_code = .;
        *(.sram_code_section)
        _esran_code = .;
    } > SRAM

    /* it's used for C++ exception handling      */
    /* we need to keep this to avoid overlapping */
    .ARM.exidx :
    {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } > FLASH

    .data :
    {
        _data = ALIGN(., 4);
        *(.data*)           /*read-write initialized data: initialized global variable*/
        *(.flashprog*)      /* Flash program */
        
        /* These array sections are used by __libc_init_array to call static C++ constructors */
        . = ALIGN(4);
        /* preinit data */
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP(*(.preinit_array))
        PROVIDE_HIDDEN (__preinit_array_end = .);

        . = ALIGN(4);
        /* init data */
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE_HIDDEN (__init_array_end = .);

        . = ALIGN(4);
        /* finit data */
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array))
        PROVIDE_HIDDEN (__fini_array_end = .);

        _edata = ALIGN(., 4);
    } > SRAM AT>FLASH
    __load_data = LOADADDR(.data);

    .bss :
    {
        . = ALIGN(4);
        _bss = .;
        *(.bss*)     /*read-write zero initialized data: uninitialzed global variable*/
        *(COMMON)
        _ebss = ALIGN(., 4);
    } > SRAM

    .pal_nvm_db :
    {
    /* Align to the sector size */
    . = ALIGN(0x2000);
    __pal_nvm_db_start__ = .;
    . = . + _PAL_NVM_SIZE;
    __pal_nvm_db_end__ = .;
    } > PAL_NVM_DB

    .cfg_nvm :
    {
        /* Align to page */
        . = ALIGN(0x2000);
        __cfg_nvm_start__ = .;
        . = . + CFG_MEM_SIZE;
        __cfg_nvm_end__ = .;
    } > CFG
    
    .riscv_flash :
    {
        KEEP(*riscv.o) /* RISCV program code */
    } > RISCV_FLASH

    .shared :
    {
        . = ALIGN(4);
        _shared = .;
        *(.mailbox*)
        . = ALIGN(4);
        *(.shared*)     /*read-write zero initialized data: uninitialzed global variable*/
        _eshared = ALIGN(., 4);
    } > SRAM
    __shared_data = LOADADDR(.shared);

    /* Set stack top to end of RAM, and stack limit move down by
     * size of stack_dummy section */
    __StackTop = ORIGIN(SRAM) + LENGTH(SRAM);
    __StackLimit = __StackTop - SIZEOF(.stack_dummy);

    /* .stack_dummy section doesn't contains any symbols. It is only
     * used for linker to calculate size of stack sections, and assign
     * values to stack symbols later */
    .stack_dummy (COPY):
    {
        *(.stack*)
    } > SRAM

    .heap (COPY):
    {
        . = ALIGN(4);
        *(.heap*)
        __HeapLimit = ABSOLUTE(__StackLimit);
    } > SRAM

    .mailbox_0 (NOLOAD) :
    {
        _mailbox_0 = ALIGN(., 4); 
        *(.mailbox_0*)           /*Shared memory section*/
        . = . + _MAILBOX_SIZE;
        _emailbox_0 = ALIGN(., 4);
    } > MAILBOX_0

    .mailbox_1 (NOLOAD) :
    {
        _mailbox_1 = ALIGN(., 4);
        *(.mailbox_1*)           /*Shared memory section*/
        . = . + _MAILBOX_SIZE;
        _emailbox_1 = ALIGN(., 4);
    } > MAILBOX_1

    PROVIDE(__stack = __StackTop);

    /* Check if data + heap + stack exceeds RAM limit */
    ASSERT(__StackLimit >= _ebss, "region RAM overflowed with stack")
}
