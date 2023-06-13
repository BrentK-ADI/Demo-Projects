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


// Return an array index given an x and y coordinate and an x resolution
// This function also enforces array boundaries.
unsigned int _i(unsigned int x, unsigned int y, unsigned int xres, unsigned int yres)
{
    if (x < 0)
        x = 0;
    else if (x > xres)
        x = xres - 1;
    if (y < 0)
        y = 0;
    else if (y > yres)
        y = yres - 1;
    return y * xres + x;
}


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

void bayer_bilinear_demosaicing_col_to_row(uint8_t *srcimg, uint32_t src_width, uint32_t src_height,
                                           uint32_t col, uint32_t h_offset, rgb888_pixel_t *dstimg,
                                           uint32_t dst_count)
{
    unsigned int r, g, b = 0;
    rgb888_pixel_t tempPixel;

    //X is fixed at the column value
    int x = col;

    if( x & 1 ) //Odd column
    {
        //Just need to loop vertically
        int y = (h_offset + dst_count);
        while( y > (int)h_offset)
        {
            r = (srcimg[_i(x, y + 1, src_width, src_height)] + // Up
                    srcimg[_i(x, y - 1, src_width, src_height)]); // Down
            r = r >> 1; // Divide by 2
            g = srcimg[_i(x, y, src_width, src_height)]; // We're at green pixel
            b = (srcimg[_i(x - 1, y, src_width, src_height)] + // Left
                    srcimg[_i(x + 1, y, src_width, src_height)]); // Right
            b = b >> 1; // Divide by 2
            tempPixel.color_data.r = (uint8_t)r;
            tempPixel.color_data.g = (uint8_t)g;
            tempPixel.color_data.b = (uint8_t)b;
            *dstimg++ = tempPixel;            
            y--;

            r = srcimg[_i(x, y, src_width, src_height)]; // We're at red pixel
            g = (srcimg[_i(x - 1, y, src_width, src_height)] + // Left
                    srcimg[_i(x + 1, y, src_width, src_height)] + // Right
                    srcimg[_i(x, y + 1, src_width, src_height)] + // Up
                    srcimg[_i(x, y - 1, src_width, src_height)]); // Down
            g = g >> 2; // Divide by 4
            b = (srcimg[_i(x - 1, y + 1, src_width, src_height)] + // Top left
                    srcimg[_i(x + 1, y + 1, src_width, src_height)] + // Top right
                    srcimg[_i(x - 1, y - 1, src_width, src_height)] + // Bottom left
                    srcimg[_i(x + 1, y - 1, src_width, src_height)]); // Bottom right
            b = b >> 2; // Divide by 4

            tempPixel.color_data.r = (uint8_t)r;
            tempPixel.color_data.g = (uint8_t)g;
            tempPixel.color_data.b = (uint8_t)b;
            *dstimg++ = tempPixel;            
            y--;
        }
    } 
    else //Even Column
    {
        //Just need to loop vertically
        int y = (h_offset + dst_count);
        while( y > (int)h_offset)
        {
            r = (srcimg[_i(x - 1, y + 1, src_width, src_height)] + // Top left
                    srcimg[_i(x + 1, y + 1, src_width, src_height)] + // Top right
                    srcimg[_i(x - 1, y - 1, src_width, src_height)] + // Bottom left
                    srcimg[_i(x + 1, y - 1, src_width, src_height)]); // Bottom right
            r = r >> 2; // Divide by 4
            g = (srcimg[_i(x - 1, y, src_width, src_height)] + // Left
                    srcimg[_i(x + 1, y, src_width, src_height)] + // Right
                    srcimg[_i(x, y + 1, src_width, src_height)] + // Up
                    srcimg[_i(x, y - 1, src_width, src_height)]); // Down
            g = g >> 2; // Divide by 4
            b = srcimg[_i(x, y, src_width, src_height)]; // We're at blue pixel

            //Store the RGB 8-bit data directly
            tempPixel.color_data.r = (uint8_t)r;
            tempPixel.color_data.g = (uint8_t)g;
            tempPixel.color_data.b = (uint8_t)b;
            *dstimg++ = tempPixel;            
            y--;
            
            r = (srcimg[_i(x - 1, y, src_width, src_height)] + // Left
                    srcimg[_i(x + 1, y, src_width, src_height)]); // Right
            r = r >> 1; // Divide by 2
            g = srcimg[_i(x, y, src_width, src_height)]; // We're at green pixel
            b = (srcimg[_i(x, y + 1, src_width, src_height)] + // Up
                    srcimg[_i(x, y - 1, src_width, src_height)]); // Down
            b = b >> 1; // Divide by 2

            tempPixel.color_data.r = (uint8_t)r;
            tempPixel.color_data.g = (uint8_t)g;
            tempPixel.color_data.b = (uint8_t)b;
            *dstimg++ = tempPixel;            
            y--;
        }
    }
}