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

typedef struct 
{
        uint8_t red;
        uint8_t green;
        uint8_t blue;
        uint8_t unused;
} rgb888_data_t;

typedef union 
{
    uint32_t wordData;
    pixel_data_4_t byteData;
} pixel_union_t;



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


static inline uint32_t FetchBayerValues(uint32_t* srcPtr, uint32_t split)
{
    uint32_t row;
    row = *srcPtr;//Get the primary word
    if( split )
    {
        //Get the next word, and join its lower half with the upper half of the
        //first word
        row = __PKHBT(*(srcPtr+1), row); 

        //Need to rotate 16 bits to align everything in order
        row = __ROR_16(row);
    }
    return row;
}

void bayer_bilinear_demosaic_cols_to_rows_rgb888(uint8_t *srcimg, uint32_t src_width,
                                                 uint32_t src_height, uint32_t col0,
                                                 uint32_t row_offset, uint32_t* destRow0,
                                                 uint32_t* destRow1, uint32_t destCount)
{
    uint32_t* srcPtr;
    uint32_t  rowWidth;
    uint32_t  isSplit;

    pixel_union_t row0, row1, row2;
    pixel_union_t vertAvg, tempRow, tempRow2, horzAvg;
    rgb888_data_t pixel0, pixel1;
    
    assert((src_width & 0x3) == 0); //This assumes columns divisible by 4
    assert((col0 & 0x1) == 0);
    assert((row_offset & 0x1) == 0); //Need to start on an even row
    

    //Long way about to get the starting pointer location
    srcPtr = (uint32_t*)srcimg;        //Base address
    rowWidth = (src_width / 4);        //Determine number of words per row
    srcPtr += (rowWidth * row_offset); //Add in the row offset
    srcPtr += (col0 / 4);              //Add in the column offset
    
    //Since we're rotating the clockwise, the last row read is actual the first
    //column of the output pixel. So start at the bottom and work upwards
    srcPtr += (rowWidth * destCount); 

    //This algorithm generates 2 output rows are once from the 2 pixels that can
    //be interpolated from 3 rows of 4-bayer pixels.
    //Determine if the 2 output rows are generated from word aligned memory, or
    //split across 2 words in memory
    if( col0 & 0x2 )
    {
        isSplit = 1;
    }
    else
    {
        isSplit = 0;
    }

    //Pre-load rows 1 and 2 before the loop
    row2.wordData = FetchBayerValues(srcPtr + (rowWidth * 2),isSplit);
    row1.wordData = FetchBayerValues(srcPtr + rowWidth, isSplit);

    //Loop through the image. 2 rows will be worked on per iteration
    for(int i = 0; i < destCount; i+=2 )
    {
        //Fill in row0
        row0.wordData = FetchBayerValues(srcPtr, isSplit);
        srcPtr -= rowWidth;

        // Even row (B G B G) - Row 0
        //          (G R G R) - Row 1
        //          (B G B G) - Row 2
        //Do some parallel math
        vertAvg.wordData = __UHADD8(row0.wordData, row2.wordData);
        tempRow.wordData = __PKHBT_LSL16(row1.wordData, vertAvg.wordData);
        tempRow2.wordData = __PKHTB_ASR16(vertAvg.wordData, row1.wordData);
        horzAvg.wordData = __UHADD8(tempRow.wordData, tempRow2.wordData);

        //vertAvg = [ Bs[0] Gs[1] Bs[2] Gs[3] ]
        //tempRow is [G[0] R[1] Bs[0] Gs[1]]
        //tempRow2 is [G[2] R[3] Bs[2] Gs[3]];
        //horzAvg is [ G[0&2] R[1&3] B_Corners[0&2] G Corners[1&3]]
        //Now we have all the calculations needed to do our operations

        //NOTE: All of these assignments can be done with some assembly to reduce
        //over head. To be continued....
        pixel0.red = row1.byteData.p1;
        pixel0.blue = horzAvg.byteData.p2;
        //The following line can be done with some tricky assembly code. For now
        //Just do it with C for read ability.
        //Note casting is needed to ensure no overflow
        pixel0.green = (uint8_t)(((uint16_t)horzAvg.byteData.p0 + (uint16_t)vertAvg.byteData.p1) >> 1);
        *destRow0++ = *(uint32_t*)&pixel0;

        pixel1.green = row1.byteData.p2;
        pixel1.blue = vertAvg.byteData.p2;
        pixel1.red = horzAvg.byteData.p1;
        *destRow1++ = *(uint32_t*)&pixel1;               

        //Shift the 3 row window up a row, fetching a new row 0
        row2 = row1;
        row1 = row0;
        row0.wordData = FetchBayerValues(srcPtr, isSplit);
        srcPtr -= rowWidth;

        // Odd row (G R G R) - Row 0
        //         (B G B G) - Row 1
        //         (G R G R) - Row 2
        vertAvg.wordData = __UHADD8(row0.wordData, row2.wordData);
        tempRow.wordData = __PKHBT_LSL16(row1.wordData, vertAvg.wordData);
        tempRow2.wordData = __PKHTB_ASR16(vertAvg.wordData, row1.wordData);
        horzAvg.wordData = __UHADD8(tempRow.wordData, tempRow2.wordData);

        //vertAvg is the average of the columns between row0 and row 2
        //tempRow is [B[0] G[1] G[0] R[1]]
        //tempRow2 is [B[2] G[3] G[2] R[3]];
        //horzAvg is [ B[0&2] G[1&3] G_Corners[0&2] R Corners[1&3]]

        //Now we have all the calculations needed to do our operations
        pixel0.green = row1.byteData.p1;
        pixel0.blue = horzAvg.byteData.p0;
        pixel0.red = vertAvg.byteData.p1;
        *destRow0++ = *(uint32_t*)&pixel0;

        //Same note as above with regard to tricky assembly math
        pixel1.green = (uint8_t)(((uint16_t)horzAvg.byteData.p1 + (uint16_t)vertAvg.byteData.p2) >> 1);
        pixel1.blue = row1.byteData.p2;
        pixel1.red = horzAvg.byteData.p3;
        *destRow1++ = *(uint32_t*)&pixel1;    
        
        //Prep the next iteration
        row2 = row1;
        row1 = row0;
    }
}