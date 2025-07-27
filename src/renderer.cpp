/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2025 Florian Braun (FloThinksPi)
 */

#include "Adafruit_TinyUSB.h"
#include <Arduino.h>
#include <Wire.h>
#include "EPD_7in3f_test.h"  
#include "led.h"
#include "waveshare_PCF85063.h" // RTC
#include "DEV_Config.h"

#include <time.h>
#include <cstring>
#include <cstdint>
#include <cstdarg>

// I2C command definitions for slave mode
#define CMD_WRITE_CHUNK       0x01
#define CMD_RENDER_IMAGE      0x02
#define CMD_GET_STATUS        0x03
#define CMD_WRITE_CHUNK_ADDR  0x04  // Write chunk to specific address offset
#define CMD_RENDER_BMP_ORDER  0x05  // Render image data that's in BMP order (needs reordering)

// I2C Pin Configuration for Pi Pico (SLAVE)
#define I2C_SDA_PIN 4    // Pi Pico GPIO 4 (I2C0 SDA) -> ESP32 GPIO 21 (SDA)  
#define I2C_SCL_PIN 5    // Pi Pico GPIO 5 (I2C0 SCL) -> ESP32 GPIO 22 (SCL)
#define I2C_SLAVE_ADDRESS 0x42  // Pi Pico I2C slave address
#define I2C_PORT i2c0   // Use I2C0 with correct hardware pins

#define CHUNK_SIZE 123  // Match actual I2C data capacity
static uint8_t image_buffer[192000]; // Buffer to store received image
static uint32_t received_bytes = 0;
static bool render_requested = false;
static bool bmp_reorder_needed = false; // Flag to track if BMP reordering is needed
static uint8_t status_response = 0; // 0=ready, 1=receiving, 2=rendering, 3=error

extern const char *fileList;
extern char pathName[];

// I2C callback functions
void onI2CReceive(int bytes) {
    if (bytes > 0) {
        uint8_t command = Wire.read();
        bytes--;
        
        switch (command) {
            case CMD_WRITE_CHUNK: {
                if (bytes >= 5) { // At least chunk_id (4 bytes) + some data
                    uint32_t chunk_id = 0;
                    chunk_id |= ((uint32_t)Wire.read()) << 24;
                    chunk_id |= ((uint32_t)Wire.read()) << 16;
                    chunk_id |= ((uint32_t)Wire.read()) << 8;
                    chunk_id |= Wire.read();
                    bytes -= 4;
                    
                    // Calculate offset in image buffer  
                    uint32_t offset = chunk_id * 123; // Each chunk is 123 bytes
                    
                    // Read remaining data into buffer
                    int bytes_written = 0;
                    for (int i = 0; i < bytes && (offset + i) < sizeof(image_buffer); i++) {
                        if (Wire.available()) {
                            image_buffer[offset + i] = Wire.read();
                            bytes_written++;
                            received_bytes = max(received_bytes, offset + i + 1);
                        } else {
                            Serial.printf("⚠ Wire not available at byte %d of chunk %d\n", i, chunk_id);
                            break;
                        }
                    }
                    if (bytes_written != 123) {
                        Serial.printf("⚠ Chunk %d: expected 123 bytes, wrote %d bytes to offset %d\n", 
                                      chunk_id, bytes_written, offset);
                    } 
                }
                break;
            }
            
            case CMD_WRITE_CHUNK_ADDR: {
                if (bytes >= 9) { // At least 8 bytes header + some data
                    // Read 4-byte address offset
                    uint32_t address_offset = 0;
                    address_offset |= ((uint32_t)Wire.read()) << 24;
                    address_offset |= ((uint32_t)Wire.read()) << 16;
                    address_offset |= ((uint32_t)Wire.read()) << 8;
                    address_offset |= Wire.read();
                    
                    // Read 4-byte data offset within address
                    uint32_t data_offset = 0;
                    data_offset |= ((uint32_t)Wire.read()) << 24;
                    data_offset |= ((uint32_t)Wire.read()) << 16;
                    data_offset |= ((uint32_t)Wire.read()) << 8;
                    data_offset |= Wire.read();
                    bytes -= 8;
                    
                    uint32_t final_offset = address_offset + data_offset;
                    
                    // Read remaining data into buffer
                    int bytes_written = 0;
                    for (int i = 0; i < bytes && (final_offset + i) < sizeof(image_buffer); i++) {
                        if (Wire.available()) {
                            image_buffer[final_offset + i] = Wire.read();
                            bytes_written++;
                            received_bytes = max(received_bytes, final_offset + i + 1);
                        } else {
                            Serial.printf("⚠ Wire not available at byte %d of addressed chunk\n", i);
                            break;
                        }
                    }
                    Serial.printf("Addressed write: addr_offset=%u, data_offset=%u, final=%u, wrote=%d bytes\n", 
                                 address_offset, data_offset, final_offset, bytes_written);
                } else {
                    // Flush remaining bytes for malformed command
                    while (Wire.available()) Wire.read();
                }
                break;
            }
            
            case CMD_RENDER_IMAGE:
                render_requested = true;
                bmp_reorder_needed = false; // Regular render, no reordering needed
                Serial.printf("Render command received for %d bytes\n", received_bytes);
                break;
                
            case CMD_RENDER_BMP_ORDER:
                render_requested = true;
                bmp_reorder_needed = true; // BMP reordering is needed
                Serial.printf("BMP reorder and render command received for %d bytes\n", received_bytes);
                status_response = 2; // Set to rendering status
                break;
                
            case CMD_GET_STATUS:
                // Status will be sent on next request
                break;
                
            default:
                Serial.printf("Unknown command: 0x%02X\n", command);
                // Flush remaining bytes
                while (Wire.available()) Wire.read();
                break;
        }
    }
}

void onI2CRequest() {
    // Send status response
    Wire.write(status_response);
}

// Initialize I2C as slave using Arduino Wire library
void init_i2c() {
    Serial.printf("Initializing I2C slave on pins %d (SDA), %d (SCL) at address 0x%02X\n", 
                  I2C_SDA_PIN, I2C_SCL_PIN, I2C_SLAVE_ADDRESS);
    
    // Initialize I2C as slave
    Wire.begin(I2C_SLAVE_ADDRESS); // Start as I2C slave
    Wire.setClock(600000); // 600kHz to match master
    
    // Set up receive and request handlers
    Wire.onReceive(onI2CReceive);
    Wire.onRequest(onI2CRequest);
    
    Serial.printf("I2C slave initialized successfully\n");
}

// I2C slave handler functions
void handle_i2c_write_chunk(uint8_t* data, size_t len) {
    if (len < 5) return; // Need at least command + 4-byte address
    
    uint32_t address = data[1] | (data[2] << 8) | (data[3] << 16) | (data[4] << 24);
    uint8_t chunk_len = len - 5;
    
    if (address + chunk_len <= sizeof(image_buffer)) {
        memcpy(&image_buffer[address], &data[5], chunk_len);
        if (address + chunk_len > received_bytes) {
            received_bytes = address + chunk_len;
        }
        Serial.printf("Received chunk: addr=%d, len=%d, total=%d bytes\n", address, chunk_len, received_bytes);
        status_response = 1; // receiving
    }
}

void handle_i2c_render_command() {
    render_requested = true;
    status_response = 2; // rendering
    Serial.printf("Render command received for %d bytes\n", received_bytes);
}

uint8_t handle_i2c_get_status() {
    return status_response;
}

// Global flag to track render completion
static bool render_was_completed = false;

// Reorder BMP data from sequential (BMP order) to display order
void reorderBmpToDisplay() {
    Serial.println("Reordering BMP data to display format...");
    
    const uint32_t width = 800;
    const uint32_t height = 480;
    const uint32_t bytes_per_row = width / 2; // 400 bytes per row (2 pixels per byte)
    
    // Allocate temporary buffer for reordering
    uint8_t* temp_buffer = (uint8_t*)malloc(192000);
    if (!temp_buffer) {
        Serial.println("✗ Failed to allocate temp buffer for BMP reordering");
        return;
    }
    
    Serial.println("Applying BMP to display transformations...");
    
    // The image_buffer currently contains BMP data in this order:
    // - Row 0 (bottom of image) followed by Row 1, Row 2, ... Row 479 (top of image)
    // - Each row has pixels left-to-right as they appear in BMP
    //
    // We need to transform to display order:
    // - Row 0 should be top of display, Row 479 should be bottom
    // - Apply X-coordinate flip: display_x = (width-1) - bmp_x
    
    for (uint32_t bmp_row = 0; bmp_row < height; bmp_row++) {
        // Convert BMP row to display row (flip Y coordinate)
        uint32_t display_row = (height - 1) - bmp_row;
        
        // Source: current row in BMP order
        uint32_t src_row_start = bmp_row * bytes_per_row;
        
        // Destination: target row in display order
        uint32_t dst_row_start = display_row * bytes_per_row;
        
        // Process each pair of pixels in the row (with X-flip)
        for (uint32_t pixel_pair = 0; pixel_pair < bytes_per_row; pixel_pair++) {
            uint32_t src_byte_pos = src_row_start + pixel_pair;
            
            // Extract the two pixels from source byte
            uint8_t src_byte = image_buffer[src_byte_pos];
            uint8_t left_pixel = (src_byte >> 4) & 0x0F;  // High nibble
            uint8_t right_pixel = src_byte & 0x0F;        // Low nibble
            
            // Apply X-coordinate flip: the rightmost pixel pair becomes leftmost
            uint32_t flipped_pixel_pair = (bytes_per_row - 1) - pixel_pair;
            uint32_t dst_byte_pos = dst_row_start + flipped_pixel_pair;
            
            // Swap left and right pixels when flipping X coordinate
            uint8_t flipped_byte = (right_pixel << 4) | left_pixel;
            
            temp_buffer[dst_byte_pos] = flipped_byte;
        }
        
        // Progress reporting
        if (bmp_row % 100 == 0) {
            float percent = ((float)(bmp_row + 1) / height) * 100.0;
            Serial.printf("Reordering progress: %d/%d rows (%.1f%%)\n", 
                         bmp_row + 1, height, percent);
        }
    }
    
    // Copy reordered data back to image buffer
    memcpy(image_buffer, temp_buffer, 192000);
    free(temp_buffer);
    
    Serial.println("✓ BMP reordering completed - image is now in display format");
}

// Check if render is requested and process it
bool check_and_process_render() {
    if (render_requested && received_bytes > 0) {
        Serial.printf("Processing render request for %d bytes\n", received_bytes);
        status_response = 2; // rendering
        
        // Apply BMP reordering if needed
        if (bmp_reorder_needed) {
            Serial.println("BMP reordering requested - converting from BMP order to display order");
            reorderBmpToDisplay();
            bmp_reorder_needed = false; // Reset flag
        }
        
        // Expected size for e-paper display: 800x480 pixels, 4 bits per pixel = 192,000 bytes
        const uint32_t expected_size = 192000;
        
        // Try to call the actual display function with received data
        Serial.printf("Calling EPD_7in3f_display_with_data with %d bytes of received image data...\n", received_bytes);
        Serial.printf("Expected size: %d bytes, received: %d bytes, difference: %d bytes\n", 
                     expected_size, received_bytes, (int)(expected_size - received_bytes));
        
        // If we're close to the expected size (within 10 bytes), pad with zeros
        if (received_bytes >= expected_size - 10 && received_bytes <= expected_size) {
            if (received_bytes < expected_size) {
                Serial.printf("Padding %d missing bytes with zeros\n", expected_size - received_bytes);
                memset(&image_buffer[received_bytes], 0, expected_size - received_bytes);
            }
            
            // Call the display function with exactly 192,000 bytes
            int display_result = EPD_7in3f_display_with_data(image_buffer, expected_size, 3.3f);
            
            if (display_result == 0) {
                Serial.printf("✓ Image successfully displayed on e-paper display! (%d bytes)\n", expected_size);
            } else {
                Serial.printf("⚠ Display function returned error %d, but continuing\n", display_result);
            }
        } else {
            Serial.printf("⚠ Image size too different from expected (%d vs %d), skipping display\n", 
                         received_bytes, expected_size);
        }
        
        Serial.printf("✓ Image rendering completed! (%d bytes processed)\n", received_bytes);
        
        // Keep the data and status for a while so fetcher knows we're done
        status_response = 0; // ready for next image  
        render_requested = false;
        render_was_completed = true; // Set the completion flag
        
        // Don't reset received_bytes immediately - let it stay for status reporting
        return true;
    }
    return false;
}

#define enChargingRtc 0

/*
Mode 0: Automatically get pic folder names and sort them
Mode 1: Automatically get pic folder names but not sorted
Mode 2: pic folder name is not automatically obtained, users need to create fileList.txt file and write the picture name in TF card by themselves
*/
#define Mode 2


float measureVBAT(void)
{
    float Voltage=0.0;
    const float conversion_factor = 3.3f / (1 << 12);
    uint16_t result = adc_read();
    Voltage = result * conversion_factor * 3;
    Serial.printf("Raw value: 0x%03x, voltage: %f V\n", result, Voltage);
    return Voltage;
}

void run_display(Time_data Time, Time_data alarmTime)
{
    // For now, use the original display function - we'll modify it to use received data later
    EPD_7in3f_display(measureVBAT());

    PCF85063_clear_alarm_flag();    // clear RTC alarm flag
    rtcRunAlarm(Time, alarmTime);  // RTC run alarm
}


void chargeState_callback(uint gpio, uint32_t events) 
{
    if(DEV_Digital_Read(VBUS)) {
        if(!DEV_Digital_Read(CHARGE_STATE)) { // battery is charging
            ledCharging();
        }
        else {  // charge complete
            ledCharged();
        }
    }
}

// HID report descriptor using TinyUSB's template
// Single Report (no ID) descriptor
uint8_t const desc_hid_report[] = {
    TUD_HID_REPORT_DESC_GAMEPAD()
};


Adafruit_USBD_HID usb_hid;

void setup()
{
    // Manual begin() is required on core without built-in support e.g. mbed rp2040
    if (!TinyUSBDevice.isInitialized()) {
        TinyUSBDevice.begin(0);
    }

    Serial.begin(115200);

    // Setup HID
    usb_hid.setPollInterval(2);
    usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
    usb_hid.begin();

    // If already enumerated, additional class driverr begin() e.g msc, hid, midi won't take effect until re-enumeration
    if (TinyUSBDevice.mounted()) {
        TinyUSBDevice.detach();
        delay(10);
        TinyUSBDevice.attach();
    }
    sleep_ms(3000);  // Give USB more time to enumerate
    
    Serial.printf("=== RENDERER STARTING ===\r\n");
    
    
    // Don't wait for serial connection - continue regardless
    Serial.printf("USB Serial initialized!\r\n");
    
    // Initialize basic LED functionality first for debugging
    Serial.printf("Initializing basic GPIO for LED test...\r\n");
    
    // Try to initialize the development module
    Serial.printf("Attempting DEV_Module_Init...\r\n");
    
    
    int init_result = DEV_Module_Init();
    if(init_result != 0) {
        Serial.printf("ERROR: DEV_Module_Init failed with code %d!\r\n", init_result);
        
        
        
        // Try to blink LED manually even if init failed
        Serial.printf("Attempting manual LED blink...\r\n");
        // Don't return - try to continue
    } else {
        Serial.printf("DEV_Module_Init successful\r\n");
        
        
        // Test LED functionality
        Serial.printf("Testing LED functionality...\r\n");
        
        ledPowerOn();  // This should blink LED_ACT 3 times
    }
    
    // Disable watchdog initially to prevent resets during debugging
    Serial.printf("Skipping watchdog for now - debugging mode\r\n");
    
    // watchdog_enable(8*1000, 1);    // 8s - DISABLED for debugging
    
    
    // Original initialization continues here...
    Time_data Time = {2024-2000, 3, 31, 0, 0, 0};
    Time_data alarmTime = Time;
    alarmTime.hours +=1;
    
    Serial.printf("Setting up remaining hardware...\r\n");
    
    
    // Initialize I2C communication
    init_i2c();
    
    Serial.printf("=== I2C SLAVE READY ===\r\n");
    Serial.printf("Waiting for commands from ESP32 master...\r\n");
    Serial.printf("Ready to receive image data and render commands\r\n");
    
    // Main loop - just wait for I2C commands and process them
    while(1) {
        // Check if we received a render command
        if (check_and_process_render()) {
            Serial.printf("✓ Image rendered successfully!\n");
            
            // Celebrate with LED pattern
            for(int i = 0; i < 5; i++) {
                DEV_Digital_Write(LED_ACT, 1);
                DEV_Delay_ms(100);
                DEV_Digital_Write(LED_ACT, 0);
                DEV_Delay_ms(100);
            }
        }
        
        // Simple heartbeat every 5 seconds
        static unsigned long last_heartbeat = 0;
        static unsigned long render_complete_time = 0;
        
        if (millis() - last_heartbeat > 5000) {
            Serial.printf("I2C Slave Status: %d bytes received, render_requested=%s\n", 
                         received_bytes, render_requested ? "true" : "false");
            last_heartbeat = millis();
            
            // Single heartbeat blink
            DEV_Digital_Write(LED_ACT, 1);
            DEV_Delay_ms(50);
            DEV_Digital_Write(LED_ACT, 0);
        }
        
        // Reset received_bytes after render is complete and some time has passed
        if (received_bytes > 0 && !render_requested && status_response == 0) {
            // Only start the timer if we actually completed a render
            if (render_complete_time == 0 && render_was_completed) {
                render_complete_time = millis(); // Mark when render completed
                render_was_completed = false; // Reset the flag
                Serial.printf("Starting 10-second delay before clearing data\n");
            } else if (render_complete_time > 0 && millis() - render_complete_time > 10000) { // Wait 10 seconds after render
                Serial.printf("Clearing received data after successful render (waited 10s)\n");
                received_bytes = 0;
                render_complete_time = 0;
            }
        }
        
        delay(100); // Small delay to prevent busy loop
    }
    
}

void loop()
{
     #ifdef TINYUSB_NEED_POLLING_TASK
    // Manual call tud_task since it isn't called by Core's background
    TinyUSBDevice.task();
    #endif

    // not enumerated()/mounted() yet: nothing to do
    if (!TinyUSBDevice.mounted()) {
        return;
    }

    if (!usb_hid.ready()) return;
    
    DEV_Digital_Write(LED_ACT, 1);
    DEV_Delay_ms(500);
    DEV_Digital_Write(LED_ACT, 0);
    DEV_Delay_ms(500);
}