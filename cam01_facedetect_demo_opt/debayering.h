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

#ifndef LIBRARIES_MISCDRIVERS_CAMERA_DEBAYERING_H_
#define LIBRARIES_MISCDRIVERS_CAMERA_DEBAYERING_H_

#include <stdint.h>

typedef union
{
    struct 
    {
        uint8_t r;
        uint8_t g;
        uint8_t b;
        uint8_t unused;
    } __attribute__((packed)) color_data;

    uint32_t pixel_data;
} rgb888_pixel_t;

/**
* @brief Apply color correction to a RAW8 bayer pattern image using the "gray world" assumption. (E. Y. Lam,
* “Combining gray world and retinex theory for automatic
* white balance in digital photography,” in Proc. 9th IEEE Intl. Symposium
* on Comsumer Electronics, 2005, pp. 134–139.).  Call this function before debayering to improve color accuracy.
* @param srcimg Pointer to the RAW8 bayer pattern.  The image will be modified in-place.
* @param[in] w Width of the bayer pattern (in pixels)
* @param[in] h Height of the bayer pattern (in pixels)
****************************************************************************/
void color_correct(uint8_t *srcimg, unsigned int w, unsigned int h);

/**
 * @brief Color-correct and debayer two source columns of the input image to 
 *        generate two output rows of the destination image
 *
 * @param[in] srcimg     Pointer to the raw bayer pattern.
 * @param[in] src_width  Width of the entire source image (in pixels).
 * @param[in] src_height Height of the entire source image (in pixels).
 * @param[in] col        Column to convert
 * @param[in] h_offset   Vertical offset of the sub-section within the source image (in pixels).
 * @param[out] dstimg0     Output pointer for column 0 of the converted RGB888 image.
 * @param[out] dstimg1     Output pointer for column 1 of the converted RGB888 image.
 * @param[in] dst_count   Total input rows (output columns) to process
 ****************************************************************************/
void bayer_bilinear_demosaicing_col_to_row(uint8_t *srcimg, uint32_t src_width, uint32_t src_height,
                                           uint32_t col, uint32_t h_offset, rgb888_pixel_t *dstimg0,
                                           rgb888_pixel_t *dstimg1, uint32_t dst_count);

#endif // LIBRARIES_MISCDRIVERS_CAMERA_DEBAYERING_H_
