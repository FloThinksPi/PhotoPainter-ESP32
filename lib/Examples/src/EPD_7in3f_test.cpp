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

int EPD_7in3f_display_with_data(uint8_t* image_data, uint32_t data_size, float vol, bool show_battery_info, float battery_voltage, uint32_t display_cycles)
{
    printf("📺 INIT DEBUG: Starting e-Paper Init and Clear...\r\n");
    EPD_7IN3F_Init();
    printf("📺 INIT DEBUG: EPD_7IN3F_Init() completed successfully\r\n");

    // Use the provided image_data buffer directly instead of allocating new memory
    UBYTE *Image = image_data;
    UDOUBLE Imagesize = ((EPD_7IN3F_WIDTH % 2 == 0)? (EPD_7IN3F_WIDTH / 2 ): (EPD_7IN3F_WIDTH / 2 + 1)) * EPD_7IN3F_HEIGHT;
    printf("📺 INIT DEBUG: Using provided image buffer (%d bytes, expected %d bytes)\r\n", data_size, Imagesize);
    
    if (data_size != Imagesize) {
        printf("ERROR: Image size mismatch - received %d bytes, expected %d bytes\r\n", data_size, Imagesize);
        return -1;
    }
    
    printf("✓ Using provided image buffer directly (no malloc needed)\r\n");
    
    // If battery overlay is requested, add it to the image buffer BEFORE displaying
    if (show_battery_info || vol < 3.5f) {
        printf("Adding battery overlay to image buffer (debug mode or low voltage)...\r\n");
        
        // CRITICAL: Set up Paint system FIRST before any drawing operations
        printf("Setting up Paint system for text overlay...\r\n");
        Paint_NewImage(Image, EPD_7IN3F_WIDTH, EPD_7IN3F_HEIGHT, 0, EPD_7IN3F_WHITE);
        Paint_SetScale(7);  // Important: Set the color scale
        Paint_SetRotate(ROTATE_90);  // Rotate 90 degrees instead of 270 to match image orientation
        
        // Create battery status text - centered at top of image
        char battery_text[128];
        
        // Non-linear Li-ion battery percentage calculation based on discharge curve
        // Values above 4.0V = 100%, below 3.3V = 0%, with curve-matched interpolation
        int battery_percent;
        printf("BATTERY DEBUG: Calculating percentage for %.3fV\r\n", battery_voltage);
        
        if (battery_voltage >= 4.0f) {
            battery_percent = 100; // Everything above 4.0V is 100% (full charge)
            printf("BATTERY DEBUG: High voltage %.3fV -> 100%%\r\n", battery_voltage);
        } else if (battery_voltage <= 3.3f) {
            battery_percent = 0;   // Everything below 3.3V is 0%
            printf("BATTERY DEBUG: Low voltage %.3fV -> 0%%\r\n", battery_voltage);
        } else {
            // Non-linear interpolation matching Li-ion discharge curve
            // Using piecewise linear approximation of the curve
            if (battery_voltage >= 3.8f) {
                // 4.0V-3.8V: 100% to 75% (steep drop at high voltage)
                battery_percent = 75 + (int)((battery_voltage - 3.8f) / (4.0f - 3.8f) * 25.0f);
                printf("BATTERY DEBUG: Range 3.8-4.0V: %.3fV -> %d%%\r\n", battery_voltage, battery_percent);
            } else if (battery_voltage >= 3.7f) {
                // 3.8V-3.7V: 75% to 50% (moderate slope)
                battery_percent = 50 + (int)((battery_voltage - 3.7f) / (3.8f - 3.7f) * 25.0f);
                printf("BATTERY DEBUG: Range 3.7-3.8V: %.3fV -> %d%%\r\n", battery_voltage, battery_percent);
            } else if (battery_voltage >= 3.6f) {
                // 3.7V-3.6V: 50% to 25% (moderate slope)
                battery_percent = 25 + (int)((battery_voltage - 3.6f) / (3.7f - 3.6f) * 25.0f);
                printf("BATTERY DEBUG: Range 3.6-3.7V: %.3fV -> %d%%\r\n", battery_voltage, battery_percent);
            } else if (battery_voltage >= 3.5f) {
                // 3.6V-3.5V: 25% to 10% (getting steeper)
                battery_percent = 10 + (int)((battery_voltage - 3.5f) / (3.6f - 3.5f) * 15.0f);
                printf("BATTERY DEBUG: Range 3.5-3.6V: %.3fV -> %d%%\r\n", battery_voltage, battery_percent);
            } else if (battery_voltage >= 3.4f) {
                // 3.5V-3.4V: 10% to 3% (steep drop)
                battery_percent = 3 + (int)((battery_voltage - 3.4f) / (3.5f - 3.4f) * 7.0f);
                printf("BATTERY DEBUG: Range 3.4-3.5V: %.3fV -> %d%%\r\n", battery_voltage, battery_percent);
            } else {
                // 3.4V-3.3V: 3% to 0% (very steep drop near cutoff)
                battery_percent = (int)((battery_voltage - 3.3f) / (3.4f - 3.3f) * 3.0f);
                printf("BATTERY DEBUG: Range 3.3-3.4V: %.3fV -> %d%%\r\n", battery_voltage, battery_percent);
            }
        }
        
        // Ensure bounds
        if (battery_percent < 0) battery_percent = 0;
        if (battery_percent > 100) battery_percent = 100;
        
        if (battery_voltage < 3.2f) {
            snprintf(battery_text, sizeof(battery_text), "Critical battery %d%% - please recharge immediately", battery_percent);
        } else if (battery_voltage < 3.5f) {
            snprintf(battery_text, sizeof(battery_text), "Low battery %d%% - please recharge", battery_percent);
        } else {
            snprintf(battery_text, sizeof(battery_text), "Battery %d%% (%.2fV) - %lu cycles since last charge", battery_percent, battery_voltage, display_cycles);
        }
        
        printf("Drawing battery text: '%s' at position (10,10)\r\n", battery_text);
        
        // Draw a test rectangle to verify drawing is working (larger and more visible)
        printf("Drawing test rectangle at (5,5) size 600x60\r\n");
        Paint_DrawRectangle(5, 5, 600, 60, EPD_7IN3F_BLACK, DOT_PIXEL_3X3, DRAW_FILL_FULL);
        
        // Display main battery text at top with high contrast (white text on black background)
        printf("Drawing white text on black background\r\n");
        Paint_DrawString_EN(10, 15, battery_text, &Font16, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);
        
        // Display cycle count on second line if low battery
        if (battery_voltage < 3.2f) {
            char cycle_text[64];
            snprintf(cycle_text, sizeof(cycle_text), "Cycles since last charge: %lu", display_cycles);
            printf("Drawing cycle text: '%s' at position (10,35)\r\n", cycle_text);
            Paint_DrawString_EN(10, 35, cycle_text, &Font16, EPD_7IN3F_WHITE, EPD_7IN3F_BLACK);
        }
        
        printf("✓ Battery overlay added: %.2fV, %lu cycles\r\n", battery_voltage, display_cycles);
    }
    
    // Display the image data with any overlay
    printf("📺 DISPLAY DEBUG: About to display image data (%d bytes)\r\n", data_size);
    
    printf("📺 STEP 1: Calling EPD_7IN3F_Display() function...\r\n");
    EPD_7IN3F_Display(Image);
    printf("📺 STEP 2: EPD_7IN3F_Display() function completed!\r\n");

    printf("📺 STEP 3: Going to sleep...\r\n");
    EPD_7IN3F_Sleep();
    
    // Show voltage warning after display (optional)
    if(vol < 3.3) {
        printf("WARNING: Low voltage detected: %f V - please charge\r\n", vol);
    }

    return 0;
}