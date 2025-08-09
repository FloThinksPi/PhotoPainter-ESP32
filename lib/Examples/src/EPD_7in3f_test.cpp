/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2025 Florian Braun (FloThinksPi)
 */

/*****************************************************************************
* | File      	:   EPD_7in3f_test.c
* | Author      :   Waveshare team
* | Function    :   7.3inch e-Paper (F) Demo
* | Info        :
*----------------
* |	This version:   V1.0
* | Date        :   2023-03-13
* | Info        :
* -----------------------------------------------------------------------------
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documnetation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to  whom the Software is
# furished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
******************************************************************************/
#include "EPD_7in3f_test.h"
#include <ImageData.h>
#include <DEV_Config.h>
#include <EPD_7in3f.h>
#include <GUI_Paint.h>
#include <GUI_BMPfile.h>

#include <stdlib.h> // malloc() free()
#include <string.h>

int EPD_7in3f_display(float vol)
{
    printf("e-Paper Init and Clear...\r\n");
    EPD_7IN3F_Init();

    //Create a new image cache
    UBYTE *Image;
    UDOUBLE Imagesize = ((EPD_7IN3F_WIDTH % 2 == 0)? (EPD_7IN3F_WIDTH / 2 ): (EPD_7IN3F_WIDTH / 2 + 1)) * EPD_7IN3F_HEIGHT;
    if((Image = (UBYTE *)malloc(Imagesize)) == NULL) {
        printf("Failed to apply for black memory...\r\n");
        return -1;
    }
    printf("Paint_NewImage\r\n");
    Paint_NewImage(Image, EPD_7IN3F_WIDTH, EPD_7IN3F_HEIGHT, 0, EPD_7IN3F_WHITE);
    Paint_SetScale(7);

    printf("Display BMP\r\n");
    // Paint_SelectImage(Image);
    // Paint_Clear(EPD_7IN3F_WHITE);
    // EPD_7IN3F_Display(Image);
    
    // while(true) {
    //     DEV_ESP_SPI_Read_nByte(rx, 16);
    //     // printf every byte in rx_buf as hex

    //     for(int i = 0; i < 15; i++) {
    //         printf("%02x", rx[i]);
    //     }
    //     printf(" - ");
    //     for(int i = 0; i < 15; i++) {
    //         printf("%s", rx[i]);
    //     }
    //     printf("\n");
    //     sleep_ms(100);
    //     watchdog_update();
    // }
    // DEV_ESP_SPI_Read_nByte(Image7color, 8);
    printf("Drawing Bitmap\r\n");
    uint32_t y, x, c;

    printf("Reading From ESP32 via I2C - using static test data\r\n");
    uint8_t BUFFERSIZE = 32;
    uint8_t rx[32];
    uint8_t tx[32];

    for (y = 0; y < (Paint.WidthByte * Paint.HeightByte)/BUFFERSIZE; y++) {
        watchdog_update();
        uint32_t Adress = y*BUFFERSIZE;
        if (y+1 >= (Paint.WidthByte * Paint.HeightByte)/BUFFERSIZE) {
            tx[0] = 0xFF;
            tx[1] = 0xFF;
            tx[2] = 0xFF;
            tx[3] = 0xFF;
        } else {
            tx[0] = 0xAA;
            tx[1] = 0xAA;
            tx[2] = 0xAA;
            tx[3] = 0xAA;
            tx[31] = 0xBB;
            tx[30] = 0xBB;
            tx[29] = 0xBB;
            tx[28] = 0xBB;
        }

        // Use static test pattern instead of SPI communication
        for(int i = 0; i < BUFFERSIZE; i++) {
            rx[i] = (i + y) % 256; // Simple test pattern
        }
        
        memcpy(&Paint.Image[Adress], &rx[0], BUFFERSIZE);
    }

    Paint_SetRotate(270);
    char strvol[21] = {0};
    sprintf(strvol, "%f V", vol);
    if(vol < 4.3) {
        Paint_DrawString_EN(10, 10, "Low voltage, please charge in time.", &Font16, EPD_7IN3F_BLACK, EPD_7IN3F_WHITE);
        Paint_DrawString_EN(10, 26, strvol, &Font16, EPD_7IN3F_BLACK, EPD_7IN3F_WHITE);
    }

    printf("EPD_Display\r\n");
    EPD_7IN3F_Display(Image);

    printf("Goto Sleep...\r\n\r\n");
    EPD_7IN3F_Sleep();
    free(Image);
    Image = NULL;

    return 0;
}

int EPD_7in3f_display_with_data(uint8_t* image_data, uint32_t data_size, float vol)
{
    printf("e-Paper Init and Clear...\r\n");
    EPD_7IN3F_Init();

    // Use the provided image_data buffer directly instead of allocating new memory
    UBYTE *Image = image_data;
    UDOUBLE Imagesize = ((EPD_7IN3F_WIDTH % 2 == 0)? (EPD_7IN3F_WIDTH / 2 ): (EPD_7IN3F_WIDTH / 2 + 1)) * EPD_7IN3F_HEIGHT;
    printf("Using provided image buffer (%d bytes, expected %d bytes)\r\n", data_size, Imagesize);
    
    if (data_size != Imagesize) {
        printf("ERROR: Image size mismatch - received %d bytes, expected %d bytes\r\n", data_size, Imagesize);
        return -1;
    }
    
    printf("✓ Using provided image buffer directly (no malloc needed)\r\n");
    
    // Display the image data directly without any Paint operations that might modify it
    printf("Displaying image data directly (%d bytes)\r\n", data_size);
    
    printf("EPD_Display\r\n");
    EPD_7IN3F_Display(Image);

    printf("Goto Sleep...\r\n\r\n");
    EPD_7IN3F_Sleep();
    
    // Show voltage warning after display (optional)
    if(vol < 3.3) {
        printf("WARNING: Low voltage detected: %f V - please charge\r\n", vol);
    }

    return 0;
}