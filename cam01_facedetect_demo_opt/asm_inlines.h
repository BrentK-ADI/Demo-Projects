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
#ifndef __ASM_INLINES_H__
#define __ASM_INLINES_H__
#include <stdint.h>

/*******************************************************************************
 * The GCC built in intrinsics for assembly calls is not complete, specifically
 * with instructions that have optional extra features built into the encodings
 * such as UXTAH {<Rd>,} <Rn>, <Rm> {, <rotation>} where the rotation value is
 * 8,16, or 24 bits, and is part of the actual instruction.
 * 
 * This collection of inline functions provides shortcuts to including assembly
 * calls into code
 */


/**
 * UXTAB16{<c>}{<q>} {<Rd>,} <Rn>, <Rm> {, <rotation>}
 * rotated = ROR(R[m], rotation); 
 * R[d]<15:0> = R[n]<15:0> + ZeroExtend(rotated<7:0>, 16);
 * R[d]<31:16> = R[n]<31:16> + ZeroExtend(rotated<23:16>, 16);
 * 
 * Rn = Fixed16, 2x 16-bit
 * Rm = Rotated8, 2x 8-bits zero extended
 */
inline uint32_t __UXTAB16_ROR8(uint32_t fixed16, uint32_t rotated8 )
{
    uint32_t result;
    asm("uxtab16 %0, %1, %2, ROR #8" : "=r" (result) : "r" (fixed16), "r" (rotated8) );
    return(result);
}

/**
 * UXTAB16{<c>}{<q>} {<Rd>,} <Rn>, <Rm> {, <rotation>}
 * rotated = ROR(R[m], rotation); 
 * R[d]<15:0> = R[n]<15:0> + ZeroExtend(rotated<7:0>, 16);
 * R[d]<31:16> = R[n]<31:16> + ZeroExtend(rotated<23:16>, 16);
 * 
 * Rn = Fixed16, 2x 16-bit
 * Rm = Fixed8, 2x 8-bits zero extended
 */
inline uint32_t __UXTAB16(uint32_t fixed16, uint32_t fixed8 )
{
    uint32_t result;

    asm("uxtab16 %0, %1, %2" : "=r" (result) : "r" (fixed16), "r" (fixed8) );
    return(result);
}

/**
 * UXTB16{<c>}{<q>} {<Rd>,} <Rm> {, <rotation>}
 * rotated = ROR(R[m], rotation);
 * R[d]<15:0> = ZeroExtend(rotated<7:0>, 16);
 * R[d]<31:16> = ZeroExtend(rotated<23:16>, 16);
 * 
 * Rm = Rotated8, 2x 8-bits zero extended
 */
inline uint32_t __UXTB16_ROR8(uint32_t rotated8 )
{
    uint32_t result;

    asm("uxtb16 %0, %1, ROR #8" : "=r" (result) : "r" (rotated8) );
    return(result);
}

/**
 * UXTB16{<c>}{<q>} {<Rd>,} <Rm> {, <rotation>}
 * rotated = ROR(R[m], rotation);
 * R[d]<15:0> = ZeroExtend(rotated<7:0>, 16);
 * R[d]<31:16> = ZeroExtend(rotated<23:16>, 16);
 * 
 * Rm = Fixed8, 2x 8-bits zero extended
 */
inline uint32_t __UXTB16(uint32_t fixed8 )
{
    uint32_t result;

    asm("uxtb16 %0, %1" : "=r" (result) : "r" (fixed8) );
    return(result);
}

/**
 * UXTAH{<c>}{<q>} {<Rd>,} <Rn>, <Rm> {, <rotation>}
 * rotated = ROR(R[m], rotation);
 * R[d] = R[n] + ZeroExtend(rotated<15:0>, 32);
 * 
 * Rn = Fixed32, 1x 32-bit
 * Rm = Rotated16, 1x 16-bits zero extended
 */
inline uint32_t __UXTAH_ROR16(uint32_t fixed32, uint32_t rotated16 )
{
    uint32_t result;

    asm("uxtah %0, %1, %2, ROR #16" : "=r" (result) : "r" (fixed32), "r" (rotated16) );
    return(result);
}

/**
 * UXTAH{<c>}{<q>} {<Rd>,} <Rn>, <Rm> {, <rotation>}
 * rotated = ROR(R[m], rotation);
 * R[d] = R[n] + ZeroExtend(rotated<15:0>, 32);
 * 
 * Rn = Fixed32, 1x 32-bit
 * Rm = Fixed16, 1x 16-bits zero extended
 */
inline uint32_t __UXTAH(uint32_t fixed32, uint32_t rotated16 )
{
    uint32_t result;

    asm("uxtah %0, %1, %2" : "=r" (result) : "r" (fixed32), "r" (rotated16) );
    return(result);
}

/**
 * SMULW<y>{<c>}{<q>} {<Rd>,} <Rn>, <Rm>
 * operand2 = if m_high then R[m]<31:16> else R[m]<15:0>;
 * product = SInt(R[n]) * SInt(operand2);
 * R[d] = product<47:16>;
 * // Signed overflow cannot occur
 * 
 * Rn = Fixed32, 1x 32-bit
 * Rm = src16top, 1x 16-bits, signed extended, upper word of register
 */
inline uint32_t __SMULWT(uint32_t fixed32, uint32_t src16top )
{
    uint32_t result;
    asm("smulwt %0, %1, %2" : "=r" (result) : "r" (fixed32), "r" (src16top) );
    return(result);
}

/**
 * SMULW<y>{<c>}{<q>} {<Rd>,} <Rn>, <Rm>
 * operand2 = if m_high then R[m]<31:16> else R[m]<15:0>;
 * product = SInt(R[n]) * SInt(operand2);
 * R[d] = product<47:16>;
 * // Signed overflow cannot occur
 * 
 * Rn = Fixed32, 1x 32-bit
 * Rm = src16bottom, 1x 16-bits, signed extended, lower word of register
 */
inline uint32_t __SMULWB(uint32_t fixed32, uint32_t src16bottom )
{
    uint32_t result;
    asm("smulwb %0, %1, %2" : "=r" (result) : "r" (fixed32), "r" (src16bottom) );
    return(result);
}

/**
 * SMUL<x><y>{<c>}{<q>} {<Rd>,} <Rn>, <Rm>
 * operand1 = if n_high then R[n]<31:16> else R[n]<15:0>;
 * operand2 = if m_high then R[m]<31:16> else R[m]<15:0>;
 * result = SInt(operand1) * SInt(operand2);
 * R[d] = result<31:0>;
 * // Signed overflow cannot occur
 * 
 * Rn = src1bottom16, 1x 16-bit, lower word of register
 * Rm = src2bottom16, 1x 16-bit, lower word of register
 */
inline uint32_t __SMULBB(uint32_t src1bottom16, uint32_t src2bottom16 )
{
    uint32_t result;
    asm("smulbb %0, %1, %2" : "=r" (result) : "r" (src1bottom16), "r" (src2bottom16) );
    return(result);
}


/**
 * SMUL<x><y>{<c>}{<q>} {<Rd>,} <Rn>, <Rm>
 * operand1 = if n_high then R[n]<31:16> else R[n]<15:0>;
 * operand2 = if m_high then R[m]<31:16> else R[m]<15:0>;
 * result = SInt(operand1) * SInt(operand2);
 * R[d] = result<31:0>;
 * // Signed overflow cannot occur
 * 
 * Rn = src1bottom16, 1x 16-bit, lower word of register
 * Rm = src2top16, 1x 16-bit, upper word of register
 */
inline uint32_t __SMULBT(uint32_t src1bottom16, uint32_t src2top16 )
{
    uint32_t result;
    asm("smulbt %0, %1, %2" : "=r" (result) : "r" (src1bottom16), "r" (src2top16) );
    return(result);
}


/**
 * USAT{<c>}{<q>} <Rd>, #<imm>, <Rn> {, <shift>}
 * operand = Shift(R[n], shift_t, shift_n, APSR.C); // APSR.C ignored
 * (result, sat) = UnsignedSatQ(SInt(operand), saturate_to);
 * R[d] = ZeroExtend(result, 32);
 * if sat then
 *     APSR.Q = '1';
 * 
 * Rn = src1
 * For unsigned n-bit saturation using USAT, this means that:
 *   -If the value to be saturated is less than 0, the result returned is 0.
 *   -If the value to be saturated is greater than 2n−1, the result returned is 2n−1.
 *   -Otherwise, the result returned is the same as the value to be saturated.* 
 */
inline uint32_t __USAT_8BITS_ROR8(uint32_t src )
{
    uint32_t result;
    asm("usat %0, #8, %1, ASR #8" : "=r" (result) : "r" (src));
    return(result);
}

/**
 * PKHBT{<c>}{<q>} {<Rd>,} <Rn>, <Rm> {, LSL #<imm>} tbform == FALSE
 * operand2 = Shift(R[m], shift_t, shift_n, APSR.C); // APSR.C ignored
 * R[d]<15:0> = R[n]<15:0>;
 * R[d]<31:16> = operand2<31:16>;
 * 
 * Rn = lowersrc16fixed - Lower 16 bits to keep, stored in lower 16
 * Rm = uppersrc16lsl - Upper 16 bits to keep, after shift
 */
inline uint32_t __PKHBT_LSL16(uint32_t lowersrc16fixed, uint32_t uppersrc16lsl )
{
    uint32_t result;
    asm("pkhbt %0, %1, %2, LSL #16 " : "=r" (result) : "r" (lowersrc16fixed), "r" (uppersrc16lsl));
    return(result);
}


/**
 * PKHBT{<c>}{<q>} {<Rd>,} <Rn>, <Rm> {, LSL #<imm>} tbform == FALSE
 * operand2 = Shift(R[m], shift_t, shift_n, APSR.C); // APSR.C ignored
 * R[d]<15:0> = R[n]<15:0>;
 * R[d]<31:16> = operand2<31:16>;
 * 
 * Rn = lowersrc16fixed - Lower 16 bits to keep, stored in lower 16
 * Rm = uppersrc16fixed - Upper 16 bits to keep, stored in upper 16
 */
inline uint32_t __PKHBT(uint32_t lowersrc16fixed, uint32_t uppersrc16fixed )
{
    uint32_t result;
    asm("pkhbt %0, %1, %2 " : "=r" (result) : "r" (lowersrc16fixed), "r" (uppersrc16fixed));
    return(result);
}

/**
 * PKHTB{<c>}{<q>} {<Rd>,} <Rn>, <Rm> {, ASR #<imm>} tbform == TRUE
 * operand2 = Shift(R[m], shift_t, shift_n, APSR.C); // APSR.C ignored
 * R[d]<15:0> = operand2<15:0>
 * R[d]<31:16> = R[n]<31:16>
 * 
 * Rn = uppersrc16fixed - Upper 16 bits to keep, stored in upper 16
 * Rm = lowersrc16asr - Lower 16 bits to keep, after shift
 */
inline uint32_t __PKHTB_ASR16(uint32_t uppersrc16fixed, uint32_t lowersrc16asr )
{
    uint32_t result;
    asm("pkhtb %0, %1, %2, ASR #16 " : "=r" (result) : "r" (uppersrc16fixed), "r" (lowersrc16asr));
    return(result);
}

/**
 * AND{S}{<c>}{<q>} {<Rd>,} <Rn>, #<const>
 * result = R[n] AND imm32;
 * 
 * Rn = src
 * Immediate is 0xFF00FF00 keeping the MSBs of 2 16-bit words
 */
inline uint32_t __AND_KEEPMSBS(uint32_t src)
{
    uint32_t result;
    asm("and %0, %1, #0xFF00FF00 " : "=r " (result) : "r" (src));
    return(result);
}

/**
 * AND{S}{<c>}{<q>} {<Rd>,} <Rn>, #<const>
 * result = R[n] AND imm32;
 * 
 * Rn = src
 * Immediate is 0x00FF00FF keeping the LSBs of 2 16-bit words
 */
inline uint32_t __AND_KEEPLSBS(uint32_t src)
{
    uint32_t result;
    asm("and %0, %1, #0x00FF00FF " : "=r " (result) : "r" (src));
    return(result);
}



/**
 * ORR{S}{<c>}{<q>} {<Rd>,} <Rn>, <Rm> {, <shift>}
 * (shifted, carry) = Shift_C(R[m], shift_t, shift_n, APSR.C);
 * result = R[n] OR shifted;
 */
inline uint32_t __ORR(uint32_t src1, uint32_t src2)
{
    uint32_t result;
    asm("orr %0, %1, %2 " : "=r " (result) : "r" (src1) , "r" (src2));
    return(result);
}

/**
 * ORR{S}{<c>}{<q>} {<Rd>,} <Rn>, <Rm> {, <shift>}
 * (shifted, carry) = Shift_C(R[m], shift_t, shift_n, APSR.C);
 * result = R[n] OR shifted;
 * 
 * Rn = src1 Kept as is
 * Rm = src2lsl - LSL 8 prior to ORR;ing
 */
inline uint32_t __ORR_LSL8(uint32_t src1, uint32_t src2lsl)
{
    uint32_t result;
    asm("orr %0, %1, %2, LSL #8 " : "=r " (result) : "r" (src1) , "r" (src2lsl));
    return(result);
}

/**
 * ROR{S}{<c>}{<q>} {<Rd>,} <Rm>, #<imm>
 */
inline uint32_t __ROR_16(uint32_t src)
{
    uint32_t result;
    asm("ror %0, %1, #16 " : "=r " (result) : "r" (src));
    return(result);
}

/**
 * REV16{<c>}{<q>} <Rd>, <Rm>
 *  bits(32) result;
 *  result<31:24> = R[m]<23:16>;
 *  result<23:16> = R[m]<31:24>;
 *  result<15:8> = R[m]<7:0>;
 *  result<7:0> = R[m]<15:8>;
 *  R[d] = result;
 */
inline uint32_t __REV16(uint32_t src)
{
    uint32_t result;
    asm("rev16 %0, %1 " : "=r " (result) : "r" (src));
    return(result);
}

/**
 * REV{<c>}{<q>} <Rd>, <Rm>
 *  bits(32) result;
 * result<31:24> = R[m]<7:0>;
 * result<23:16> = R[m]<15:8>;
 * result<15:8> = R[m]<23:16>;
 * result<7:0> = R[m]<31:24>;
 * R[d] = result;
 */
inline uint32_t __REV(uint32_t src)
{
    uint32_t result;
    asm("rev %0, %1 " : "=r " (result) : "r" (src));
    return(result);
}

/**
 * UHADD8{<c>}{<q>} {<Rd>,} <Rn>, <Rm>
 *  sum1 = UInt(R[n]<7:0>) + UInt(R[m]<7:0>);
 *  sum2 = UInt(R[n]<15:8>) + UInt(R[m]<15:8>);
 *  sum3 = UInt(R[n]<23:16>) + UInt(R[m]<23:16>);
 *  sum4 = UInt(R[n]<31:24>) + UInt(R[m]<31:24>);
 *  R[d]<7:0> = sum1<8:1>;
 *  R[d]<15:8> = sum2<8:1>;
 *  R[d]<23:16> = sum3<8:1>;
 *  R[d]<31:24> = sum4<8:1>;
 */
inline uint32_t __UHADD8(uint32_t src1, uint32_t src2)
{
    uint32_t result;
    asm("uhadd8 %0, %1, %2 " : "=r " (result) : "r" (src1), "r" (src2));
    return(result);
}

/**
 * USUB8{<c>}{<q>} {<Rd>,} <Rn>, <Rm>
 * diff1 = UInt(R[n]<7:0>) - UInt(R[m]<7:0>);
 * diff2 = UInt(R[n]<15:8>) - UInt(R[m]<15:8>);
 * diff3 = UInt(R[n]<23:16>) - UInt(R[m]<23:16>);
 * diff4 = UInt(R[n]<31:24>) - UInt(R[m]<31:24>);
 * R[d]<7:0> = diff1<7:0>;
 * R[d]<15:8> = diff2<7:0>;
 * R[d]<23:16> = diff3<7:0>;
 * R[d]<31:24> = diff4<7:0>;
 * 
 * Does 4 8-bit subtractions of src2 from src1
 */
inline uint32_t __USUB8(uint32_t src1, uint32_t src2)
{
    uint32_t result;
    asm("usub8 %0, %1, %2 " : "=r " (result) : "r" (src1), "r" (src2));
    return(result);
}
#endif
