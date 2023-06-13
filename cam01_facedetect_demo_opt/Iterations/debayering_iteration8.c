/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 ******************************************************************************/

#include <stdio.h>
#include <math.h>
#include <assert.h>
#include "debayering.h"
#include "asm_inlines.h"
#ifdef DEBUG
#define PRINT(...) printf(__VA_ARGS__)
#else
#define PRINT(...)
#endif

typedef struct 
{
    uint8_t p0;
    uint8_t p1;
    uint8_t p2;
    uint8_t p3;
} __attribute__((packed)) pixel_data_4_t;

typedef union 
{
    pixel_data_4_t bytes;
    uint32_t       word;
}pixel_data_t;


// Calculate color correction coefficients using gray world assumption.
// This simple method uses the green channel to determine the coefficients.
/*
 E. Y. Lam, “Combining gray world and retinex theory for automatic
white balance in digital photography,” in Proc. 9th IEEE Intl. Symposium
on Comsumer Electronics, 2005, pp. 134–139.
*/
void calc_correction_simple(uint8_t *bayer_pattern, unsigned int w, unsigned int h,
                            uint16_t *out_coeff_r, uint16_t *out_coeff_b)
{
    unsigned int r_avg = 0, g_avg = 0, b_avg = 0;
    unsigned int count = 0;
    uint32_t* pixelPtr = (uint32_t*)bayer_pattern;
    uint32_t  dataVal;
    uint32_t  counter1;
    uint32_t  counter2;

    assert((w & 0x3) == 0); //This assumes columns divisible by 4
    assert((w <= 1024));    //More than 1024 will overflow
    assert((h & 0x1) == 0); //This assumes even rows

    for( int y = 0; y < (h / 2); y++)
    {
        //Even Row Loop
        counter1 = 0;
        counter2 = 0;
        for( int x = 0; x < (w / 4); x++)
        {  
            dataVal = *pixelPtr++;
            //Accumuate bytes at 0 & 2 locations
            counter1 = __UXTAB16(counter1, dataVal);
            //Accumuate bytes at 1 & 3 locations
            counter2 = __UXTAB16_ROR8(counter2, dataVal);
        }

        //Blue-Green Row
        b_avg = __UXTAH(b_avg, counter1); //Get and add lower 16-bits
        b_avg = __UXTAH_ROR16(b_avg, counter1); //Get and add upper 16-bits
        g_avg = __UXTAH(g_avg, counter2); //Get and add lower 16-bits
        g_avg = __UXTAH_ROR16(g_avg, counter2); //Get and add upper 16-bits

        counter1 = 0;
        counter2 = 0;
        for( int x = 0; x < (w / 4); x++)
        {  
            dataVal = *pixelPtr++;
            //Accumuate bytes at 0 & 2 locations
            counter1 = __UXTAB16(counter1, dataVal);
            //Accumuate bytes at 1 & 3 locations
            counter2 = __UXTAB16_ROR8(counter2, dataVal);
        }
        //Green-Red Row
        g_avg = __UXTAH(g_avg, counter1); //Get and add lower 16-bits
        g_avg = __UXTAH_ROR16(g_avg, counter1); //Get and add upper 16-bits 
        r_avg = __UXTAH(r_avg, counter2); //Get and add lower 16-bits
        r_avg = __UXTAH_ROR16(r_avg, counter2); //Get and add upper 16-bits

    }

    //Get half count for green
    count = (w * h) >> 1;
    g_avg = g_avg / count;

    //Halve again for blue and red
    count = count >> 1;
    r_avg = r_avg / count;
    b_avg = b_avg / count;

    *out_coeff_r = (uint16_t)(g_avg << 8) / r_avg;
    *out_coeff_b = (uint16_t)(g_avg << 8) / b_avg;
}

void color_correct(uint8_t *srcimg, unsigned int w, unsigned int h)
{
    uint16_t coeff_r = 0, coeff_b = 0;
    uint32_t scaledVal;
    pixel_data_4_t* pixelDataPtr = (pixel_data_4_t*)srcimg;
    pixel_data_4_t tempPixels;

    assert((w & 0x3) == 0); //This assumes columns divisible by 4
    assert((h & 0x1) == 0); //This assumes even rows
    
    calc_correction_simple(srcimg, w, h, &coeff_r, &coeff_b);

    // Apply color correction before debayering, helps slightly to reduce artifacts
    for( int y = 0; y < (h / 2); y++)
    {
        for (int x = 0; x < (w / 4); x++) 
        {
            tempPixels = *pixelDataPtr;
            scaledVal = coeff_b * tempPixels.p0;
            tempPixels.p0 = (scaledVal & 0xFFFF0000) ? 255 : 
                                (uint8_t)(scaledVal >> 8);
            scaledVal = coeff_b * tempPixels.p2;
            tempPixels.p2 = (scaledVal & 0xFFFF0000) ? 255 : 
                                (uint8_t)(scaledVal >> 8);            
            *pixelDataPtr++ = tempPixels;                    
        }

        for (int x = 0; x < (w / 4); x++) 
        {
            tempPixels = *pixelDataPtr;
            scaledVal = coeff_r * tempPixels.p1;
            tempPixels.p1 = (scaledVal & 0xFFFF0000) ? 255 : 
                                (uint8_t)(scaledVal >> 8);
            scaledVal = coeff_r * tempPixels.p3;
            tempPixels.p3 = (scaledVal & 0xFFFF0000) ? 255 : 
                                (uint8_t)(scaledVal >> 8);            
            *pixelDataPtr++ = tempPixels;       
        }
    }
}


static inline pixel_data_t FetchBayerValues(uint32_t* srcPtr, uint32_t split)
{
    pixel_data_t retVal;
    uint32_t row2;
    retVal.word = *srcPtr;//Get the primary word
    if(split)
    {
        retVal.word = retVal.word >> 16;
        row2 = *(srcPtr+1);
        retVal.word |= (row2 << 16);
    }

    return retVal;
}


void bayer_bilinear_demosaicing_col_to_row(uint8_t *srcimg, uint32_t src_width, uint32_t src_height,
                                           uint32_t col, uint32_t h_offset, rgb888_pixel_t *dstimg0,
                                           rgb888_pixel_t *dstimg1, uint32_t dst_count)
{
    unsigned int r, g, b = 0;
    pixel_data_t row0, row1, row2;
    uint32_t isSplit;
    uint32_t* srcPtr32 = (uint32_t*)srcimg;
    uint32_t rowStride4 = (src_width / 4);

    rgb888_pixel_t tempPixel;

    assert((col & 1) == 0);      //Needs to be a even column
    assert((h_offset & 1) == 0); //Needs to be an even row
    assert((dst_count & 1) == 0); //Even number of dest pixels

    if( col & 0x2 )
    {
        isSplit = 1;
    }
    else
    {
        isSplit = 0;
    }
    
    //Preload Rows 2 and 1
    srcPtr32 += ((src_width * (h_offset + dst_count +1)) + col) / 4;
    row2 = FetchBayerValues(srcPtr32, col);
    srcPtr32 -= rowStride4;
    row1 = FetchBayerValues(srcPtr32, col);
    srcPtr32 -= rowStride4;

    for( int i = 0; i < dst_count / 2; i++)
    {
        
        row0 = FetchBayerValues(srcPtr32, isSplit);
        srcPtr32 -= rowStride4;

        //Even row Even column first
        r = row0.bytes.p1 + row2.bytes.p1;
        r = r >> 1; // Divide by 2
        g = row1.bytes.p1; // We're at green pixel
        b = row1.bytes.p0 + row1.bytes.p2;
        b = b >> 1; // Divide by 2
        tempPixel.color_data.r = (uint8_t)r;
        tempPixel.color_data.g = (uint8_t)g;
        tempPixel.color_data.b = (uint8_t)b;
        *dstimg0++ = tempPixel;            

        //Even Row, Odd Column
        r = row0.bytes.p1 + row0.bytes.p3 + row2.bytes.p1 + row2.bytes.p3;
        r = r >> 2; // Divide by 4
        g = row1.bytes.p1 + row1.bytes.p3 + row0.bytes.p2 + row2.bytes.p2;
        g = g >> 2; // Divide by 4
        b = row1.bytes.p2; // We're at blue pixel
        tempPixel.color_data.r = (uint8_t)r;
        tempPixel.color_data.g = (uint8_t)g;
        tempPixel.color_data.b = (uint8_t)b;
        *dstimg1++ = tempPixel;   


        //Reuse rows 0 and 1, moving upwards
        row2 = row1;
        row1 = row0;
        row0 = FetchBayerValues(srcPtr32, isSplit);
        srcPtr32 -= rowStride4;


        //Odd Row, Even Column
        r = row1.bytes.p1; // We're at red pixel
        g = row1.bytes.p0 + row1.bytes.p2 + row0.bytes.p1 + row2.bytes.p1;
        g = g >> 2; // Divide by 4
        b = row0.bytes.p0 + row0.bytes.p2 + row2.bytes.p0 + row2.bytes.p2;
        b = b >> 2; // Divide by 4
        tempPixel.color_data.r = (uint8_t)r;
        tempPixel.color_data.g = (uint8_t)g;
        tempPixel.color_data.b = (uint8_t)b;
        *dstimg0++ = tempPixel;            
    
        //Odd Row, Odd Column
        r = row1.bytes.p1 + row1.bytes.p3;
        r = r >> 1; // Divide by 2
        g = row1.bytes.p2; // We're at green pixel
        b = row0.bytes.p2 + row2.bytes.p2;
        b = b >> 1; // Divide by 2
        tempPixel.color_data.r = (uint8_t)r;
        tempPixel.color_data.g = (uint8_t)g;
        tempPixel.color_data.b = (uint8_t)b;
        *dstimg1++ = tempPixel;    

        //Reuse rows 0 and 1, moving upwards
        row2 = row1;
        row1 = row0;
    }
}