/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2025 Florian Braun (FloThinksPi)
 */

#include <Arduino.h>
#include <Wire.h>
#include "EPD_7in3f_test.h"  
#include "led.h"
#include "waveshare_PCF85063.h" // RTC
#include "DEV_Config.h"
#include "hardware/watchdog.h"

#include <time.h>
#include <cstring>
#include <cstdint>
#include <cstdarg>

// Power management variables (simplified) - MAXIMUM POWER EFFICIENCY
static unsigned long button_press_time = 0;
static unsigned long last_button_state = 1; // 1 = not pressed (pull-up)
static const unsigned long TRANSFER_TIMEOUT_MS = 60000; // 60 seconds
static const unsigned long BUTTON_DEBOUNCE_MS = 100;    // Reduced for faster response
static const unsigned long POST_RENDER_DELAY_MS = 200; // ULTRA-FAST: 200ms (was 2000ms) - minimum for I2C completion

// Power-aware watchdog management
static bool watchdog_enabled = false;
static bool last_power_state = false; // Track previous power state (false = battery, true = external)
static unsigned long last_power_check = 0;
static const unsigned long POWER_CHECK_INTERVAL_MS = 2000; // Check power every 2 seconds

// RTC wake-up configuration - EASILY CONFIGURABLE
// Common wake-up intervals (uncomment the one you want):
// static const unsigned long RTC_WAKEUP_INTERVAL_MINUTES = 15;  // Every 15 minutes
// static const unsigned long RTC_WAKEUP_INTERVAL_MINUTES = 30;  // Every 30 minutes  
// static const unsigned long RTC_WAKEUP_INTERVAL_MINUTES = 60;  // Every 1 hour
// static const unsigned long RTC_WAKEUP_INTERVAL_MINUTES = 120; // Every 2 hours
// static const unsigned long RTC_WAKEUP_INTERVAL_MINUTES = 360; // Every 6 hours
static const unsigned long RTC_WAKEUP_INTERVAL_MINUTES = 30; // Wake up every 30 minutes (change this value as needed)
static const unsigned long RTC_WAKEUP_INTERVAL_MS = RTC_WAKEUP_INTERVAL_MINUTES * 60 * 1000; // Convert to milliseconds

// I2C command definitions for slave mode
#define CMD_WRITE_CHUNK       0x01
#define CMD_RENDER_IMAGE      0x02
#define CMD_GET_STATUS        0x03
#define CMD_WRITE_CHUNK_ADDR  0x04  // Write chunk to specific address offset
#define CMD_RENDER_BMP_ORDER  0x05  // Render image data that's in BMP order (needs reordering)

// I2C Pin Configuration for Pi Pico (SLAVE) - ULTRA-OPTIMIZED
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

// Forward declarations for RTC and power management functions
void setupRTCWakeup();
void updateRTCWakeup();
bool handleRTCInterrupt();
void enablePowerAwareWatchdog();
void disablePowerAwareWatchdog();
bool isExternalPowerConnected();
void updatePowerAwareWatchdog();

// I2C callback functions
void onI2CReceive(int bytes) {
    if (bytes > 0) {
        uint8_t command = Wire.read();
        bytes--;
        
        // ULTRA-OPTIMIZED: Minimal feedback for maximum speed
        static unsigned long last_feedback = 0;
        if (millis() - last_feedback > 10000) { // Feedback every 10 seconds (was 5s)
            Serial.printf("I2C: cmd=0x%02X, bytes=%d\n", command, bytes);
            last_feedback = millis();
        }
        
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
                    
                    // Read 4-byte data size (NOT data offset!)
                    uint32_t data_size = 0;
                    data_size |= ((uint32_t)Wire.read()) << 24;
                    data_size |= ((uint32_t)Wire.read()) << 16;
                    data_size |= ((uint32_t)Wire.read()) << 8;
                    data_size |= Wire.read();
                    bytes -= 8;
                    
                    // Final offset is just the address_offset (no addition!)
                    uint32_t final_offset = address_offset;
                    
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
                    Serial.printf("Addressed write: addr=%u, expected_size=%u, final=%u, wrote=%d bytes\n", 
                                 address_offset, data_size, final_offset, bytes_written);
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

// Initialize I2C as slave using Arduino Wire library - ULTRA-OPTIMIZED
void init_i2c() {
    Serial.printf("Initializing ULTRA-FAST I2C slave on pins %d (SDA), %d (SCL) at address 0x%02X\n", 
                  I2C_SDA_PIN, I2C_SCL_PIN, I2C_SLAVE_ADDRESS);
    
    // Initialize I2C as slave with maximum speed settings
    Wire.begin(I2C_SLAVE_ADDRESS); // Start as I2C slave
    Wire.setClock(800000); // Increased to 800kHz to match master (was 600kHz)
    
    // Set up receive and request handlers
    Wire.onReceive(onI2CReceive);
    Wire.onRequest(onI2CRequest);
    
    Serial.printf("ULTRA-FAST I2C slave initialized at 800kHz\n");
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

// Button state detection for short press with watchdog safety
bool detectButtonPress() {
    static unsigned long last_check = 0;
    static bool button_processed = false;
    
    unsigned long now = millis();
    if (now - last_check < BUTTON_DEBOUNCE_MS) {
        return false; // Too soon to check again
    }
    last_check = now;
    
    unsigned long current_button_state = DEV_Digital_Read(NEXT_BUTTON_PIN);
    
    // Detect falling edge (button press) - only trigger once per press
    if (last_button_state == 1 && current_button_state == 0 && !button_processed) {
        button_processed = true;
        last_button_state = current_button_state;
        if (watchdog_enabled) {
            watchdog_update(); // Critical: Update watchdog on button press (only if enabled)
        }
        return true; // Button was just pressed
    }
    
    // Reset processed flag on button release
    if (current_button_state == 1) {
        button_processed = false;
    }
    
    last_button_state = current_button_state;
    return false;
}

// Power-aware watchdog management functions
void enablePowerAwareWatchdog() {
    if (!watchdog_enabled) {
        watchdog_enable(1000, 1); // 1 second timeout with reset enabled
        watchdog_enabled = true;
        Serial.printf("🔋 Watchdog ENABLED - battery mode timeout protection active\r\n");
    }
}

void disablePowerAwareWatchdog() {
    if (watchdog_enabled) {
        // Note: Pi Pico SDK doesn't have watchdog_disable(), but we can track state
        // The watchdog will continue running but we won't update it, so system will reset
        // To actually disable, we need to let it timeout or reset the chip
        // For now, we'll just mark it as disabled and provide unlimited updates
        watchdog_enabled = false;
        Serial.printf("🔌 Watchdog DISABLED - external power unlimited operation\r\n");
    }
}

bool isExternalPowerConnected() {
    return DEV_Digital_Read(VBUS) != 0; // VBUS high = external power connected
}

void updatePowerAwareWatchdog() {
    unsigned long current_time = millis();
    
    // Only check power state periodically to avoid excessive GPIO reads
    if (current_time - last_power_check < POWER_CHECK_INTERVAL_MS) {
        // Still update watchdog if it's enabled
        if (watchdog_enabled) {
            watchdog_update();
        }
        return;
    }
    
    bool current_power_state = isExternalPowerConnected();
    
    // Check if power state changed
    if (current_power_state != last_power_state) {
        Serial.printf("⚡ POWER STATE CHANGE: %s → %s\r\n",
                     last_power_state ? "External" : "Battery",
                     current_power_state ? "External" : "Battery");
        
        if (current_power_state) {
            // External power connected - disable watchdog timeout
            disablePowerAwareWatchdog();
            Serial.printf("📱 Unlimited operation time available for WiFi configuration\r\n");
        } else {
            // External power disconnected - enable watchdog protection
            enablePowerAwareWatchdog();
            Serial.printf("⏱️ Battery mode activated - watchdog timeout protection enabled\r\n");
        }
        
        last_power_state = current_power_state;
    }
    
    // Update watchdog if enabled (battery mode)
    if (watchdog_enabled) {
        watchdog_update();
    }
    
    last_power_check = current_time;
}

// Reorder BMP data from sequential (BMP order) to display order - OPTIMIZED
void reorderBmpToDisplay() {
    Serial.println("ULTRA-FAST reordering BMP data to display format...");
    
    const uint32_t width = 800;
    const uint32_t height = 480;
    const uint32_t bytes_per_row = width / 2; // 400 bytes per row (2 pixels per byte)
    
    // Allocate temporary buffer for reordering
    uint8_t* temp_buffer = (uint8_t*)malloc(192000);
    if (!temp_buffer) {
        Serial.println("✗ Failed to allocate temp buffer for BMP reordering");
        return;
    }
    
    Serial.println("Applying ULTRA-FAST BMP to display transformations...");
    
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
            
            // CRITICAL: When flipping X, we also need to swap left/right pixels within the byte
            // This is because we're essentially mirroring the entire row
            uint8_t flipped_byte = (right_pixel << 4) | left_pixel;
            
            temp_buffer[dst_byte_pos] = flipped_byte;
        }
        
        // ULTRA-OPTIMIZED: Progress reporting every 200 rows for speed
        if (bmp_row % 200 == 0) {
            float percent = ((float)(bmp_row + 1) / height) * 100.0;
            Serial.printf("Reordering: %d/%d rows (%.1f%%)\n", 
                         bmp_row + 1, height, percent);
        }
    }
    
    // Copy reordered data back to image buffer
    memcpy(image_buffer, temp_buffer, 192000);
    free(temp_buffer);
    
    Serial.println("✓ ULTRA-FAST BMP reordering completed");
}

// Flip the image horizontally in place - ULTRA-OPTIMIZED
void flipImageHorizontally() {
    Serial.println("ULTRA-FAST horizontal flip to correct mirroring...");
    
    const uint32_t width = 800;
    const uint32_t height = 480;
    const uint32_t bytes_per_row = width / 2; // 400 bytes per row (2 pixels per byte)
    
    for (uint32_t y = 0; y < height; y++) {
        uint32_t row_start = y * bytes_per_row;
        for (uint32_t x = 0; x < bytes_per_row / 2; x++) {
            uint32_t left_pos = row_start + x;
            uint32_t right_pos = row_start + (bytes_per_row - 1 - x);
            
            // Swap the bytes at opposite ends of the row
            uint8_t temp = image_buffer[left_pos];
            image_buffer[left_pos] = image_buffer[right_pos];
            image_buffer[right_pos] = temp;
        }
    }

    // After swapping bytes, the nibbles (pixels) within each byte are now in the wrong order.
    // We need to iterate through the entire buffer and swap the nibbles in every byte.
    for (uint32_t i = 0; i < received_bytes; i++) {
        uint8_t current_byte = image_buffer[i];
        image_buffer[i] = ((current_byte & 0x0F) << 4) | ((current_byte & 0xF0) >> 4);
    }

    Serial.println("✓ ULTRA-FAST image flipped horizontally");
}

// Render the image from the buffer - ULTRA-OPTIMIZED
void renderImage() {
    // If BMP reordering is needed, do it first
    if (bmp_reorder_needed) {
        reorderBmpToDisplay();
    }

    // Flip the image horizontally before displaying to correct mirroring
    flipImageHorizontally();
    
    Serial.printf("ULTRA-FAST rendering image from buffer (%d bytes)...\n", received_bytes);
    
    // Expected size for e-paper display: 800x480 pixels, 4 bits per pixel = 192,000 bytes
    const uint32_t expected_size = 192000;
    
    // Try to call the actual display function with received data
    Serial.printf("Calling EPD_7in3f_display_with_data with %d bytes...\n", received_bytes);
    Serial.printf("Expected: %d bytes, received: %d bytes, diff: %d bytes\n", 
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
            Serial.printf("✓ ULTRA-FAST image displayed successfully! (%d bytes)\n", expected_size);
        } else {
            Serial.printf("⚠ Display function returned error %d, but continuing\n", display_result);
        }
    } else {
        Serial.printf("⚠ Image size too different from expected (%d vs %d), skipping display\n", 
                     received_bytes, expected_size);
    }
    
    Serial.printf("✓ ULTRA-FAST image rendering completed! (%d bytes)\n", received_bytes);
    
    // Keep the data and status for a while so fetcher knows we're done
    status_response = 0; // ready for next image  
    render_requested = false;
    render_was_completed = true; // Set the completion flag
    
    // ULTRA-FAST power-off in battery mode after MINIMAL delay
    if (!DEV_Digital_Read(VBUS)) {
        Serial.printf("✓ Battery mode render complete - setting up next RTC wake-up\n");
        
        // Set up the next RTC wake-up alarm before powering off
        updateRTCWakeup();
        
        Serial.printf("✓ Next RTC wake-up configured - powering off in %d ms\n", POST_RENDER_DELAY_MS);
        DEV_Delay_ms(POST_RENDER_DELAY_MS); // MINIMAL delay (200ms) to ensure I2C completion
        Serial.printf("ULTRA-FAST powering off after successful render...\n");
        powerOff();
    }
    
    // Don't reset received_bytes immediately - let it stay for status reporting
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

// Configure RTC wake-up alarm using global interval
// This function sets up the PCF85063 RTC to generate an interrupt after the specified interval
// The RTC_INT pin (GPIO 6) will go LOW when the alarm triggers, waking the Pi Pico from deep sleep
void setupRTCWakeup() {
    Serial.printf("Setting up RTC wake-up every %lu minutes (%lu ms)\n", 
                  RTC_WAKEUP_INTERVAL_MINUTES, RTC_WAKEUP_INTERVAL_MS);
    
    // Clear any existing alarm flags
    PCF85063_clear_alarm_flag();
    
    // Get current time from RTC
    Time_data current_time = PCF85063_GetTime();
    Serial.printf("Current RTC time: %d-%d-%d %d:%d:%d\n", 
                 current_time.years, current_time.months, current_time.days,
                 current_time.hours, current_time.minutes, current_time.seconds);
    
    // Calculate alarm time by adding the wake-up interval
    Time_data alarm_time = current_time;
    alarm_time.minutes += RTC_WAKEUP_INTERVAL_MINUTES;
    
    // Handle minute overflow
    if (alarm_time.minutes >= 60) {
        alarm_time.hours += alarm_time.minutes / 60;
        alarm_time.minutes = alarm_time.minutes % 60;
    }
    
    // Handle hour overflow  
    if (alarm_time.hours >= 24) {
        alarm_time.days += alarm_time.hours / 24;
        alarm_time.hours = alarm_time.hours % 24;
        
        // For simplicity, if day overflows, just wrap to day 1
        // More complex month/year handling could be added if needed
        if (alarm_time.days > 31) {
            alarm_time.days = 1;
            alarm_time.months++;
            if (alarm_time.months > 12) {
                alarm_time.months = 1;
                alarm_time.years++;
            }
        }
    }
    
    Serial.printf("Setting RTC alarm for: %d-%d-%d %d:%d:%d\n", 
                 alarm_time.years, alarm_time.months, alarm_time.days,
                 alarm_time.hours, alarm_time.minutes, alarm_time.seconds);
    
    // Set the RTC alarm using the PCF85063 alarm function
    PCF85063_alarm_Time_Enabled(alarm_time);
    
    Serial.printf("✓ RTC wake-up alarm configured for %lu minute intervals\n", RTC_WAKEUP_INTERVAL_MINUTES);
    Serial.printf("✓ Pi Pico will wake up when RTC_INT (GPIO %d) goes LOW\n", RTC_INT);
}

// Update RTC alarm for next wake-up (call this after each wake-up)
void updateRTCWakeup() {
    Serial.printf("Updating RTC alarm for next wake-up in %lu minutes\n", RTC_WAKEUP_INTERVAL_MINUTES);
    
    // Clear the alarm flag that triggered this wake-up
    PCF85063_clear_alarm_flag();
    
    // Set up the next alarm
    setupRTCWakeup();
}

// Handle RTC interrupt (wake-up from sleep)
bool handleRTCInterrupt() {
    if (!DEV_Digital_Read(RTC_INT)) {
        Serial.printf("🕒 RTC WAKE-UP INTERRUPT detected!\n");
        
        // Clear the alarm flag
        PCF85063_clear_alarm_flag();
        
        // Set up the next wake-up alarm
        updateRTCWakeup();
        
        Serial.printf("✓ RTC interrupt handled - next alarm set\n");
        return true;
    }
    return false;
}

// Display current power management configuration
void printPowerConfig() {
    Serial.printf("=== POWER MANAGEMENT CONFIGURATION ===\r\n");
    Serial.printf("RTC Wake-up Interval: %lu minutes\r\n", RTC_WAKEUP_INTERVAL_MINUTES);
    Serial.printf("Transfer Timeout: %lu seconds\r\n", TRANSFER_TIMEOUT_MS / 1000);
    Serial.printf("Post-Render Delay: %lu ms (ULTRA-FAST)\r\n", POST_RENDER_DELAY_MS);
    Serial.printf("Button Debounce: %lu ms\r\n", BUTTON_DEBOUNCE_MS);
    Serial.printf("Power-Aware Watchdog: ENABLED\r\n");
    Serial.printf("  🔋 Battery Mode: 1000ms timeout active\r\n");
    Serial.printf("  🔌 External Power: Watchdog disabled (unlimited time)\r\n");
    Serial.printf("  ⚡ Dynamic: Adapts to power changes in real-time\r\n");
    Serial.printf("=====================================\r\n");
}

void run_display(Time_data Time, Time_data alarmTime)
{
    // For now, use the original display function - we'll modify it to use received data later
    EPD_7in3f_display(measureVBAT());

    PCF85063_clear_alarm_flag();    // clear RTC alarm flag
    rtcRunAlarm(Time, alarmTime);  // RTC run alarm
}

void setup()
{
    Serial.begin(115200);
    DEV_Delay_ms(20); // ULTRA-FAST startup - reduced from 50ms
    
    Serial.printf("=== ULTRA-FAST RENDERER STARTING ===\r\n");
    Serial.printf("MAXIMUM POWER EFFICIENCY MODE\r\n");
    
    // ULTRA-FAST hardware initialization (moved early for RTC access)
    Serial.printf("ULTRA-FAST hardware init...\r\n");
    if(DEV_Module_Init() != 0) {
        Serial.printf("ERROR: DEV_Module_Init failed!\r\n");
        return;
    }
    
    // ULTRA-FAST RTC initialization
    PCF85063_init();
    Serial.printf("RTC initialized\r\n");
    
    // Check if device was woken up by RTC interrupt
    if (handleRTCInterrupt()) {
        Serial.printf("🕒 Device woken up by RTC alarm - this is a scheduled wake-up\r\n");
    } else {
        Serial.printf("🔄 Device powered on normally - not an RTC wake-up\r\n");
    }
    
    // Show power management configuration
    printPowerConfig();
    
    // Initialize power-aware watchdog system
    Serial.printf("Initializing power-aware watchdog system...\r\n");
    
    // Check initial power state and configure watchdog accordingly
    bool initial_power_state = isExternalPowerConnected();
    last_power_state = initial_power_state;
    
    if (initial_power_state) {
        Serial.printf("🔌 External power detected at startup - watchdog disabled\r\n");
        // Start with watchdog disabled for unlimited operation time
        watchdog_enabled = false;
    } else {
        Serial.printf("🔋 Battery power detected at startup - watchdog enabled\r\n");
        // Enable watchdog for battery protection
        watchdog_enable(1000, 1);
        watchdog_enabled = true;
    }
    
    last_power_check = millis();
    
    // Configure RTC wake-up alarm with global interval
    setupRTCWakeup();
    
    // Initialize I2C communication ULTRA-EARLY (before battery check for max responsiveness)
    init_i2c();
    Serial.printf("ULTRA-FAST I2C slave ready at 800kHz\r\n");
    
    // Check battery voltage last (can be slow)
    float voltage = measureVBAT();
    if (voltage < 3.1) {
        Serial.printf("Low battery voltage: %.2fV - powering off\r\n", voltage);
        ledLowPower();
        powerOff();
        return;
    } else {
        Serial.printf("Battery OK: %.2fV - ULTRA-FAST operation ready\r\n", voltage);
        ledPowerOn(); // Like original PhotoPainter
    }
    
    Serial.printf("=== ULTRA-FAST READY FOR BUTTON/I2C ===\r\n");
    Serial.printf("Total startup: POWER-AWARE - watchdog adapts to power state\r\n");
}

void loop()
{
    // Power-aware watchdog management - automatically handles external power changes
    updatePowerAwareWatchdog();
    
    // Follow the EXACT pattern from original PhotoPainter main()
    if (!DEV_Digital_Read(VBUS)) {
        // Battery mode - optimized for fast wake/sleep cycles
        static unsigned long last_activity = millis();
        static bool activity_detected = false;
        
        // Check for any activity (I2C data or button press)
        bool current_activity = false;
        
        // Check for I2C data
        if (received_bytes > 0 || render_requested) {
            current_activity = true;
            if (!activity_detected) {
                Serial.printf("I2C activity detected - staying awake\n");
                activity_detected = true;
            }
        }
        
        // Check for short button press to wake/keep alive
        if (detectButtonPress()) {
            current_activity = true;
            Serial.printf("Button pressed (short) - staying awake\n");
            // ULTRA-FAST LED acknowledgment
            DEV_Digital_Write(LED_ACT, 1);
            DEV_Delay_ms(20); // Reduced from 50ms
            DEV_Digital_Write(LED_ACT, 0);
            // Additional watchdog update after button press (only if watchdog enabled)
            if (watchdog_enabled) {
                watchdog_update();
            }
        }
        
        // Update activity timer if we detected activity
        if (current_activity) {
            last_activity = millis();
        }
        
        // Handle rendering if requested (this will auto-power-off after completion)
        if (render_requested) {
            if (watchdog_enabled) {
                watchdog_update(); // Update before potentially long rendering
            }
            renderImage();
            // renderImage() calls powerOff() automatically in battery mode
            return; // Exit loop since we're powering off
        }
        
        // Only power off if no activity for extended period
        if (millis() - last_activity > TRANSFER_TIMEOUT_MS) {
            Serial.printf("No activity for %d seconds - setting up RTC wake-up and powering off\n", TRANSFER_TIMEOUT_MS/1000);
            // Set up RTC wake-up before powering off due to timeout
            updateRTCWakeup();
            powerOff();
        }
        
        // Check for RTC interrupt even in battery mode (periodic wake-ups)
        if (handleRTCInterrupt()) {
            Serial.printf("🕒 RTC wake-up in battery mode - scheduled image update\n");
            current_activity = true; // Keep device awake for potential image update
            last_activity = millis(); // Reset timeout
        }
        
        // ULTRA-OPTIMIZED: Show periodic status in battery mode (less frequent)
        static unsigned long last_status = 0;
        if (millis() - last_status > 20000) { // Every 20 seconds (was 10s)
            unsigned long time_remaining = TRANSFER_TIMEOUT_MS - (millis() - last_activity);
            Serial.printf("Battery mode: %d bytes, %lu seconds remaining\n", 
                         received_bytes, time_remaining/1000);
            last_status = millis();
        }
        
    } else {
        // USB mode - like original PhotoPainter: stay in loop
        // Handle I2C data and rendering
        if (render_requested) {
            if (watchdog_enabled) {
                watchdog_update(); // Update before rendering in USB mode
            }
            renderImage();
            Serial.printf("✓ USB mode render complete\n");
        }
        
        // Check for short button press like original
        if (detectButtonPress()) {
            Serial.printf("Button pressed (short) in USB mode\n");
            // ULTRA-FAST LED feedback
            DEV_Digital_Write(LED_ACT, 1);
            DEV_Delay_ms(20); // Reduced from 100ms
            DEV_Digital_Write(LED_ACT, 0);
            // Additional watchdog update after button press (only if watchdog enabled)
            if (watchdog_enabled) {
                watchdog_update();
            }
        }
        
        // Check RTC interrupt
        if (handleRTCInterrupt()) {
            Serial.printf("RTC interrupt in USB mode - scheduled wake-up occurred\n");
        }
        
        // ULTRA-FAST heartbeat (less verbose for power efficiency)
        static unsigned long last_heartbeat = 0;
        if (millis() - last_heartbeat > 60000) { // Every 60 seconds (was 30s)
            Serial.printf("USB mode - %d bytes received\n", received_bytes);
            last_heartbeat = millis();
        }
    }
    
    // ULTRA-FAST polling for maximum button responsiveness and power efficiency
    DEV_Delay_ms(2); // Reduced from 5ms for even faster response while maintaining power efficiency
}