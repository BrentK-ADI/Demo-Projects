/*******************************************************************************
* Copyright (C) 2019-2023 Maxim Integrated Products, Inc., All rights Reserved.
*
* This software is protected by copyright laws of the United States and
* of foreign countries. This material may also be protected by patent laws
* and technology transfer regulations of the United States and of foreign
* countries. This software is furnished under a license agreement and/or a
* nondisclosure agreement and may only be used or reproduced in accordance
* with the terms of those agreements. Dissemination of this information to
* any party or parties not specified in the license agreement and/or
* nondisclosure agreement is expressly prohibited.
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
*******************************************************************************/

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "debayering.h"
#include "mxc.h"
#include "cnn.h"
#include "mxc_errors.h"

#include "camera.h"
#include "post_process.h"


#define CAMERA_FREQ 10000000
#define CROP_X 224
#define CROP_Y 168


volatile uint32_t cnn_time; // Stopwatch
extern volatile uint8_t face_detected;


void fail(void)
{
    printf("\n*** FAIL ***\n\n");
    while (1)
        ;
}

// Data input: HWC 3x224x168 (112896 bytes total / 37632 bytes per channel):
void load_input(void)
{
    uint8_t *raw;
    uint32_t imglen, w, h;
    volatile uint32_t execTime;
    
    camera_sleep(0); // Wake up camera
    camera_start_capture_image();
    while (!camera_is_image_rcv()) {}
    camera_sleep(1); // Sleep camera to preserve power

    camera_get_image(&raw, &imglen, &w, &h);

    MXC_TMR_SW_Start(MXC_TMR1);     
    color_correct(raw, w, h);

    // The model needs 168x224.
    // The HM0360 give us 320x240.
    // We will achieve this by debayering "on the fly" to crop to
    // 224x168 while feeding the CNN column-wise instead of row-wise.
    unsigned int crop_x = 224, crop_y = 168;
    unsigned int start_x = (320 - crop_x) >> 1;
    unsigned int start_y = (240 - crop_y) >> 1;

    

    rgb888_pixel_t rgb888_buffer0[crop_y];
    rgb888_pixel_t rgb888_buffer1[crop_y];
    rgb888_pixel_t rgb888;

    for (unsigned int x = start_x; x < start_x + crop_x; x+=2) {
           
        bayer_bilinear_demosaicing_col_to_row(raw, w, h, x, start_y, rgb888_buffer0, rgb888_buffer1, crop_y);
 
        for (unsigned int y = 0; y < crop_y; y++) { // ... here.
            rgb888 = rgb888_buffer0[y];
            // Normalize from [0, 255] -> [-128, 127]
            rgb888.pixel_data = __USUB8(rgb888.pixel_data, 0x00808080);

            // Loading data into the CNN fifo
            while (((*((volatile uint32_t *)0x50000004) & 1)) != 0)
                ; // Wait for FIFO 0
            *((volatile uint32_t *)0x50000008) = rgb888.pixel_data; // Write FIFO 0
        }

        for (unsigned int y = 0; y < crop_y; y++) { // ... here.
            rgb888 = rgb888_buffer1[y];
            // Normalize from [0, 255] -> [-128, 127]
            rgb888.pixel_data = __USUB8(rgb888.pixel_data, 0x00808080);

            // Loading data into the CNN fifo
            while (((*((volatile uint32_t *)0x50000004) & 1)) != 0)
                ; // Wait for FIFO 0
            *((volatile uint32_t *)0x50000008) = rgb888.pixel_data; // Write FIFO 0
        }

    }
    execTime = MXC_TMR_SW_Stop(MXC_TMR1);  
    return;
}

int main(void)
{
    int ret = 0, id = 0;
    MXC_ICC_Enable(MXC_ICC0); // Enable cache

    // Switch to 100 MHz clock
    MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
    SystemCoreClockUpdate();

    printf("Waiting...\n");

    // DO NOT DELETE THIS LINE:
    MXC_Delay(SEC(2)); // Let debugger interrupt if needed

    // Initialize DMA and acquire a channel for the camera interface to use
    printf("Initializing DMA\n");
    MXC_DMA_Init();
    int dma_channel = MXC_DMA_AcquireChannel();

    // Initialize the camera driver.
    printf("Initializing camera\n");
    camera_init(CAMERA_FREQ);

    int slaveAddress = camera_get_slave_address();
    printf("Camera I2C slave address: %02x\n", slaveAddress);

    ret = camera_get_manufacture_id(&id);
    if (ret != STATUS_OK) {
        printf("Error returned from reading camera id. Error %d\n", ret);
        return -1;
    }
    printf("Camera ID detected: %04x\n", id);

    camera_set_hmirror(0);
    camera_set_vflip(0);
    camera_write_reg(0x3024, 0x0); // Set context A (320x240)

    ret = camera_setup(320, // width
                       240, // height
                       PIXFORMAT_BAYER, // pixel format
                       FIFO_FOUR_BYTE, // FIFO mode (four bytes is suitable for most cases)
                       USE_DMA, // DMA (enabling DMA will drastically decrease capture time)
                       dma_channel); // Allocate the DMA channel retrieved in initialization

    if (ret != E_NO_ERROR) {
        printf("Failed to setup camera!\n");
        return ret;
    }

    printf("\n*** CNN Inference Test facedet_tinierssd ***\n");

    while (1) {
        face_detected = 0;
        LED_On(1);

        // Switch to high power fast IPO clock
        MXC_SYS_ClockEnable(MXC_SYS_CLOCK_IPO);
        MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);

        // Enable peripheral, enable CNN interrupt, turn on CNN clock
        // CNN clock: APB (50 MHz) div 1
        cnn_enable(MXC_S_GCR_PCLKDIV_CNNCLKSEL_PCLK, MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV1);
        cnn_init(); // Bring state machine into consistent state
        cnn_load_weights(); // Load kernels
        cnn_load_bias();
        cnn_configure(); // Configure state machine
        cnn_start(); // Start CNN processing
        load_input(); // Load data input via FIFO

        //Temporarily turn off Sleep mode during testing.
        while (cnn_time == 0) {} // MXC_LP_EnterSleepMode(); // Wait for CNN

        // Run Non-Maximal Suppression (NMS) on bounding boxes
        get_priors();
        localize_objects();
        LED_Off(1);

        if (!face_detected) {
            LED_Off(0);
        } else {
            LED_On(0);
        }

        cnn_disable(); // Shut down CNN clock, disable peripheral

        // Switch to low power IBRO clock
        MXC_SYS_ClockEnable(MXC_SYS_CLOCK_IBRO);
        MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IBRO);
        MXC_SYS_ClockDisable(MXC_SYS_CLOCK_IPO);

        MXC_Delay(MXC_DELAY_MSEC(150)); // Slight delay to allow LED to be seen

#ifdef CNN_INFERENCE_TIMER
        printf("Approximate data loading and inference time: %u us\n\n", cnn_time);
#endif
    }

    return 0;
}

/*
  SUMMARY OF OPS
  Hardware: 589,595,888 ops (588,006,720 macc; 1,589,168 comp; 0 add; 0 mul; 0 bitwise)
    Layer 0: 4,327,680 ops (4,064,256 macc; 263,424 comp; 0 add; 0 mul; 0 bitwise)
    Layer 1: 11,063,808 ops (10,838,016 macc; 225,792 comp; 0 add; 0 mul; 0 bitwise)
    Layer 2: 43,502,592 ops (43,352,064 macc; 150,528 comp; 0 add; 0 mul; 0 bitwise)
    Layer 3: 86,854,656 ops (86,704,128 macc; 150,528 comp; 0 add; 0 mul; 0 bitwise)
    Layer 4: 86,854,656 ops (86,704,128 macc; 150,528 comp; 0 add; 0 mul; 0 bitwise)
    Layer 5: 86,854,656 ops (86,704,128 macc; 150,528 comp; 0 add; 0 mul; 0 bitwise)
    Layer 6: 173,709,312 ops (173,408,256 macc; 301,056 comp; 0 add; 0 mul; 0 bitwise)
    Layer 7 (backbone_conv8): 86,779,392 ops (86,704,128 macc; 75,264 comp; 0 add; 0 mul; 0 bitwise)
    Layer 8 (backbone_conv9): 5,513,088 ops (5,419,008 macc; 94,080 comp; 0 add; 0 mul; 0 bitwise)
    Layer 9 (backbone_conv10): 1,312,640 ops (1,290,240 macc; 22,400 comp; 0 add; 0 mul; 0 bitwise)
    Layer 10 (conv12_1): 647,360 ops (645,120 macc; 2,240 comp; 0 add; 0 mul; 0 bitwise)
    Layer 11 (conv12_2): 83,440 ops (80,640 macc; 2,800 comp; 0 add; 0 mul; 0 bitwise)
    Layer 12: 1,354,752 ops (1,354,752 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 13: 40,320 ops (40,320 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 14: 677,376 ops (677,376 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 15: 20,160 ops (20,160 macc; 0 comp; 0 add; 0 mul; 0 bitwise)

  RESOURCE USAGE
  Weight memory: 275,184 bytes out of 442,368 bytes total (62.2%)
  Bias memory:   536 bytes out of 2,048 bytes total (26.2%)
*/
