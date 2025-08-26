/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2025 Florian Braun (FloThinksPi)
 */

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>  // For persistent storage of settings
#include "EPD_7in3f_test.h"  
#include "led.h"
#include "waveshare_PCF85063.h" // RTC
#include "DEV_Config.h"
#include "hardware/watchdog.h"
#include "hardware/adc.h"  // For ADC functions

#include <time.h>
#include <cstring>
#include <cstdint>
#include <cstdarg>

// Forward declarations
float measureVBAT(void);
void initBatterySampling(void);
void updateBatterySampling(void);
uint8_t getChargingStatus();
void saveWakeupInterval(uint32_t minutes);
uint32_t loadWakeupInterval();
void initPersistentStorage();
uint32_t getCurrentSectorAddress();
void saveWakeupIntervalWearLeveled(uint32_t minutes);
uint32_t loadWakeupIntervalWearLeveled();

// EEPROM addresses for persistent settings with WEAR LEVELING
#define EEPROM_SIZE 512          // Total EEPROM size to initialize
#define WEAR_LEVELING_SECTORS 8  // Use 8 different sectors to distribute writes
#define SECTOR_SIZE 32           // Each wear-leveling sector is 32 bytes
#define MAGIC_OFFSET 0           // Magic number offset within sector
#define WAKEUP_OFFSET 4          // Wakeup interval offset within sector
#define CYCLE_OFFSET 8           // Write cycle counter offset within sector

#define EEPROM_MAGIC_VALUE 0x12345678  // Magic number for validation

// Default settings
#define DEFAULT_WAKEUP_MINUTES 30

// Wear leveling state
static uint32_t eeprom_write_cycle = 0;

// Power management variables (simplified) - MAXIMUM POWER EFFICIENCY
static unsigned long button_press_time = 0;
static unsigned long last_button_state = 1; // 1 = not pressed (pull-up)
static const unsigned long TRANSFER_TIMEOUT_MS = 60000; // 60 seconds
static const unsigned long BUTTON_DEBOUNCE_MS = 100;    // Reduced for faster response
static const unsigned long POST_RENDER_DELAY_MS = 50; // 50ms - ULTRA-FAST completion for speed optimization

// Power-aware watchdog management
static bool watchdog_enabled = false;
static bool last_power_state = false; // Track previous power state (false = battery, true = external)
static unsigned long last_power_check = 0;
static const unsigned long POWER_CHECK_INTERVAL_MS = 2000; // Check power every 2 seconds

// RTC wake-up configuration - NOW PERSISTENT!
// The wakeup interval is loaded from EEPROM on startup and saved when changed
// Common wake-up intervals for reference:
// 15 minutes, 30 minutes, 60 minutes (1 hour), 120 minutes (2 hours), 360 minutes (6 hours)
static unsigned long RTC_WAKEUP_INTERVAL_MINUTES; // Will be loaded from EEPROM on startup
static unsigned long RTC_WAKEUP_INTERVAL_MS; // Calculated from minutes

// I2C command definitions for slave mode
#define CMD_WRITE_CHUNK       0x01
#define CMD_RENDER_IMAGE      0x02
#define CMD_GET_STATUS        0x03
#define CMD_WRITE_CHUNK_ADDR  0x04  // Write chunk to specific address offset
#define CMD_RENDER_BMP_ORDER  0x05  // Render image data that's in BMP order (needs reordering)
#define CMD_GET_BATTERY       0x06  // Get battery voltage
#define CMD_GET_WAKEUP        0x07  // Get RTC wakeup interval in minutes
#define CMD_SET_WAKEUP        0x08  // Set RTC wakeup interval in minutes
#define CMD_GET_CHARGING      0x09  // Get charging status (0=not charging, 1=charging, 2=charge complete)
#define CMD_SLEEP_NOW         0x0A  // Put renderer to sleep immediately
#define CMD_SET_BATTERY_INFO  0x0B  // Set battery info for overlay display (voltage, cycles)
#define CMD_GET_NEXT_IMAGE_REQUEST  0x0C  // Check if button was pressed to request next image
#define CMD_GET_DEBUG_STATUS  0x0D  // Get debug mode status (0=off, 1=on)
#define CMD_SET_DEBUG_MODE    0x0E  // Set debug mode (0=off, 1=on)

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

// Battery overlay information
static bool show_battery_overlay = false;
static float battery_voltage = 0.0f;
static uint32_t display_cycles = 0;

// Continuous battery voltage sampling
static float cached_battery_voltage = 3.7f;  // Cached voltage for instant I2C response
static unsigned long last_battery_sample = 0;
static const unsigned long BATTERY_SAMPLE_INTERVAL = 100; // Sample every 100ms
static uint32_t battery_sample_buffer[64];  // Rolling buffer for 64 samples
static uint8_t battery_sample_index = 0;
static bool battery_sampling_initialized = false;

// Next image request tracking
static bool next_image_requested = false;

// Debug mode control
static bool debug_mode_enabled = false;

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
        
        // ULTRA-OPTIMIZED: Minimal feedback for maximum speed - only essential commands
        static unsigned long last_feedback = 0;
        if (command != CMD_WRITE_CHUNK_ADDR && millis() - last_feedback > 10000) {
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
                    
                    // Read remaining data into buffer with maximum speed
                    int bytes_written = 0;
                    for (int i = 0; i < bytes && (final_offset + i) < sizeof(image_buffer); i++) {
                        if (Wire.available()) {
                            image_buffer[final_offset + i] = Wire.read();
                            bytes_written++;
                            received_bytes = max(received_bytes, final_offset + i + 1);
                        } else {
                            break; // ULTRA-OPTIMIZED: No debug output during high-speed transfers
                        }
                    }
                    // OPTIMIZED: Only log addressed writes periodically to avoid I/O overhead
                    static unsigned long last_addr_log = 0;
                    if (millis() - last_addr_log > 5000) { // Every 5 seconds
                        Serial.printf("Addressed write: addr=%u, wrote=%d bytes, total=%u\n", 
                                     address_offset, bytes_written, received_bytes);
                        last_addr_log = millis();
                    }
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
                
            case CMD_GET_BATTERY: {
                // Get battery voltage command - response will be sent via onI2CRequest
                status_response = 0x80; // Special status to indicate battery voltage response
                break;
            }
                
            case CMD_GET_WAKEUP: {
                // Get RTC wakeup interval - response will be sent via onI2CRequest  
                status_response = 0x81; // Special status to indicate wakeup interval response
                break;
            }
                
            case CMD_SET_WAKEUP: {
                if (bytes >= 4) { // Need 4 bytes for new wakeup interval in minutes
                    uint32_t new_interval = 0;
                    new_interval |= ((uint32_t)Wire.read()) << 24;
                    new_interval |= ((uint32_t)Wire.read()) << 16;
                    new_interval |= ((uint32_t)Wire.read()) << 8;
                    new_interval |= Wire.read();
                    
                    // Validate interval (1 minute to 30.42 days)
                    if (new_interval >= 1 && new_interval <= 43800) {
                        RTC_WAKEUP_INTERVAL_MINUTES = new_interval;
                        RTC_WAKEUP_INTERVAL_MS = RTC_WAKEUP_INTERVAL_MINUTES * 60 * 1000;
                        
                        // SAVE TO PERSISTENT STORAGE
                        saveWakeupInterval(new_interval);
                        
                        Serial.printf("✓ RTC wakeup interval updated to %lu minutes and SAVED to EEPROM\n", RTC_WAKEUP_INTERVAL_MINUTES);
                        status_response = 0x82; // Success response for set wakeup
                    } else {
                        Serial.printf("⚠ Invalid wakeup interval: %lu minutes (must be 1-43800)\n", new_interval);
                        status_response = 0x83; // Error response for set wakeup
                    }
                } else {
                    // Flush remaining bytes for malformed command
                    while (Wire.available()) Wire.read();
                    status_response = 0x83; // Error response
                }
                break;
            }
                
            case CMD_GET_CHARGING: {
                // Get charging status - response will be sent via onI2CRequest
                status_response = 0x84; // Special status to indicate charging status response
                break;
            }
                
            case CMD_SLEEP_NOW: {
                // Put renderer to sleep immediately after brief delay
                Serial.printf("💤 SLEEP command received from fetcher - powering off immediately\n");
                status_response = 0x85; // Sleep command acknowledged
                
                // Set up RTC wake-up before sleeping
                updateRTCWakeup();
                
                // Brief delay to send I2C response, then sleep
                DEV_Delay_ms(100);
                powerOff();
                break;
            }
                
            case CMD_SET_BATTERY_INFO: {
                if (bytes >= 8) { // Need 8 bytes: 4 for voltage (float), 4 for cycles (uint32_t)
                    // Read voltage as float (4 bytes)
                    uint8_t voltage_bytes[4];
                    voltage_bytes[0] = Wire.read();
                    voltage_bytes[1] = Wire.read(); 
                    voltage_bytes[2] = Wire.read();
                    voltage_bytes[3] = Wire.read();
                    memcpy(&battery_voltage, voltage_bytes, 4);
                    
                    // Read cycles as uint32_t (4 bytes) - little-endian to match ESP32 sender
                    display_cycles = 0;
                    display_cycles |= Wire.read();                    // LSB first
                    display_cycles |= ((uint32_t)Wire.read()) << 8;
                    display_cycles |= ((uint32_t)Wire.read()) << 16;
                    display_cycles |= ((uint32_t)Wire.read()) << 24;  // MSB last
                    
                    show_battery_overlay = true; // Always set when battery info is received
                    Serial.printf("✓ Battery overlay info set: %.2fV, %lu cycles (will show if debug enabled)\n", 
                                 battery_voltage, display_cycles);
                    status_response = 0x86; // Battery info set successfully
                } else {
                    // Flush remaining bytes for malformed command
                    while (Wire.available()) Wire.read();
                    status_response = 0x87; // Error response
                    show_battery_overlay = false;
                }
                break;
            }
                
            case CMD_GET_NEXT_IMAGE_REQUEST: {
                // No additional data needed for this command
                while (Wire.available()) Wire.read(); // Flush any remaining bytes
                
                if (next_image_requested) {
                    status_response = 0x01; // Next image requested
                    next_image_requested = false; // Clear the flag after reporting
                    Serial.printf("✓ Next image request flag cleared and reported to ESP32\n");
                } else {
                    status_response = 0x00; // No next image request
                }
                break;
            }
            
            case CMD_GET_DEBUG_STATUS: {
                // No additional data needed for this command
                while (Wire.available()) Wire.read(); // Flush any remaining bytes
                
                // Response will be sent via onI2CRequest
                status_response = 0x88; // Special status to indicate debug status response
                Serial.printf("Debug status requested - current: %s\n", debug_mode_enabled ? "ON" : "OFF");
                break;
            }
                
            case CMD_SET_DEBUG_MODE: {
                if (bytes >= 1) { // Need 1 byte for debug mode (0=off, 1=on)
                    uint8_t new_debug_mode = Wire.read();
                    debug_mode_enabled = (new_debug_mode != 0);
                    Serial.printf("✓ Debug mode %s\n", debug_mode_enabled ? "enabled" : "disabled");
                    status_response = 0x89; // Debug mode set successfully
                } else {
                    // Flush remaining bytes for malformed command
                    while (Wire.available()) Wire.read();
                    status_response = 0x8A; // Error response
                }
                break;
            }
                
            default:
                Serial.printf("Unknown command: 0x%02X\n", command);
                // Flush remaining bytes
                while (Wire.available()) Wire.read();
                break;
        }
    }
}

void onI2CRequest() {
    if (status_response == 0x80) {
        // Send battery voltage as 4-byte float
        float voltage = measureVBAT();
        uint8_t* voltage_bytes = (uint8_t*)&voltage;
        Wire.write(voltage_bytes, 4);
        status_response = 0; // Reset to ready status
    } else if (status_response == 0x81) {
        // Send RTC wakeup interval as 4-byte uint32_t (in minutes)
        uint32_t interval = RTC_WAKEUP_INTERVAL_MINUTES;
        Wire.write((uint8_t*)&interval, 4);
        status_response = 0; // Reset to ready status
    } else if (status_response == 0x84) {
        // Send charging status as single byte
        uint8_t charging_status = getChargingStatus();
        Wire.write(charging_status);
        status_response = 0; // Reset to ready status
    } else if (status_response == 0x85) {
        // Sleep command acknowledged
        Wire.write(0x85);
        // Don't reset status - device will power off shortly
    } else if (status_response == 0x88) {
        // Send debug mode status as single byte
        uint8_t debug_status = debug_mode_enabled ? 1 : 0;
        Wire.write(debug_status);
        status_response = 0; // Reset to ready status
    } else {
        // Send regular status response
        Wire.write(status_response);
    }
}

// Initialize I2C as slave using Arduino Wire library - ULTRA-OPTIMIZED
void init_i2c() {
    Serial.printf("Initializing I2C slave on pins %d (SDA), %d (SCL) at address 0x%02X\n", 
                  I2C_SDA_PIN, I2C_SCL_PIN, I2C_SLAVE_ADDRESS);
    
    // Initialize I2C as slave with maximum speed settings
    Wire.begin(I2C_SLAVE_ADDRESS); // Start as I2C slave
    Wire.setClock(5000000); // OPTIMIZED: 1.5MHz proven optimal for maximum throughput
    
    // Set up receive and request handlers
    Wire.onReceive(onI2CReceive);
    Wire.onRequest(onI2CRequest);
    
    Serial.printf("I2C slave initialized at 1.5MHz (proven optimal)\n");
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

// Get charging status: 0=not charging, 1=charging, 2=charge complete, 3=no external power
uint8_t getChargingStatus() {
    if (!DEV_Digital_Read(VBUS)) {
        return 3; // No external power connected
    }
    
    if (!DEV_Digital_Read(CHARGE_STATE)) {
        return 1; // Battery is charging
    } else {
        return 2; // Charge complete (external power connected but not charging)
    }
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
    Serial.println("reordering BMP data to display format...");
    
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
    
    Serial.println("✓ BMP reordering completed");
}

// Flip the image horizontally in place - ULTRA-OPTIMIZED
void flipImageHorizontally() {
    Serial.println("horizontal flip to correct mirroring...");
    
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

    Serial.println("✓ image flipped horizontally");
}

// Rotate the image 180 degrees in place - corrects upside down orientation
void rotateImage180() {
    Serial.println("180-degree rotation to correct orientation...");
    
    const uint32_t width = 800;
    const uint32_t height = 480;
    const uint32_t bytes_per_row = width / 2; // 400 bytes per row (2 pixels per byte)
    const uint32_t total_bytes = bytes_per_row * height;
    
    // Swap bytes from opposite ends of the entire image buffer
    for (uint32_t i = 0; i < total_bytes / 2; i++) {
        uint32_t opposite = total_bytes - 1 - i;
        uint8_t temp = image_buffer[i];
        image_buffer[i] = image_buffer[opposite];
        image_buffer[opposite] = temp;
    }
    
    // After swapping bytes, we need to swap the nibbles (pixels) within each byte
    // because the pixel order within bytes is also reversed
    for (uint32_t i = 0; i < total_bytes; i++) {
        uint8_t current_byte = image_buffer[i];
        image_buffer[i] = ((current_byte & 0x0F) << 4) | ((current_byte & 0xF0) >> 4);
    }

    Serial.println("✓ image rotated 180 degrees");
}

// Render the image from the buffer - ULTRA-OPTIMIZED
void renderImage() {
    // If BMP reordering is needed, do it first
    if (bmp_reorder_needed) {
        reorderBmpToDisplay();
    }

    // Flip the image horizontally before displaying to correct mirroring
    flipImageHorizontally();
    
    // OPTIONAL: Uncomment the next line if image is still upside down relative to text
    // rotateImage180();
    
    Serial.printf("rendering image from buffer (%d bytes)...\n", received_bytes);
    
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
        
        // Call the display function with exactly 192,000 bytes and battery overlay info
        Serial.printf("🚀 RENDERER: Starting ultra-optimized display rendering...\n");
        unsigned long render_start_ms = millis();
        int display_result = EPD_7in3f_display_with_data(image_buffer, expected_size, 3.3f, 
                                                         show_battery_overlay && debug_mode_enabled, 
                                                         battery_voltage, display_cycles);
        unsigned long render_time_ms = millis() - render_start_ms;
        
        Serial.printf("⚡ RENDERER PERFORMANCE: Display completed in %lu ms (%.2f seconds)\n", 
                     render_time_ms, render_time_ms / 1000.0f);
        
        if (display_result == 0) {
            Serial.printf("✅ RENDERER SUCCESS: Ultra-fast image displayed! (%d bytes in %lu ms)\n", expected_size, render_time_ms);
        } else {
            Serial.printf("⚠ Display function returned error %d, but completed in %lu ms\n", display_result, render_time_ms);
        }
    } else {
        Serial.printf("⚠ Image size too different from expected (%d vs %d), skipping display\n", 
                     received_bytes, expected_size);
    }
    
    Serial.printf("✓ image rendering completed! (%d bytes)\n", received_bytes);
    
    // Keep the data and status for a while so fetcher knows we're done
    status_response = 0; // ready for next image  
    render_requested = false;
    render_was_completed = true; // Set the completion flag
    
    // power-off in battery mode after MINIMAL delay
    if (!DEV_Digital_Read(VBUS)) {
        Serial.printf("✓ Battery mode render complete - setting up next RTC wake-up\n");
        
        // Set up the next RTC wake-up alarm before powering off
        updateRTCWakeup();
        
        Serial.printf("✓ Next RTC wake-up configured - powering off in %d ms\n", POST_RENDER_DELAY_MS);
        DEV_Delay_ms(POST_RENDER_DELAY_MS); // MINIMAL delay (200ms) to ensure I2C completion
        Serial.printf("powering off after successful render...\n");
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
    // Return cached value for instant I2C response
    return cached_battery_voltage;
}

// Initialize continuous battery sampling
void initBatterySampling() {
    // Initialize ADC for VSYS monitoring (channel 3 on Pico W)
    adc_init();
    adc_gpio_init(29);  // ADC3 - VSYS/3 input (battery voltage divider)
    adc_select_input(3); // Select ADC3
    
    Serial.println("ADC initialized for battery voltage measurement (VSYS/3 via ADC3)");
    
    // Initialize rolling buffer with current ADC reading
    uint16_t initial_reading = adc_read();  // Use SDK function directly
    for (int i = 0; i < 64; i++) {
        battery_sample_buffer[i] = initial_reading;
    }
    battery_sample_index = 0;
    battery_sampling_initialized = true;
    last_battery_sample = millis();
    
    Serial.printf("Battery sampling initialized with 64-sample rolling buffer (initial: 0x%03x)\n", initial_reading);
}

// Update battery voltage sampling (call regularly from main loop)
void updateBatterySampling() {
    if (!battery_sampling_initialized) {
        return;
    }
    
    unsigned long now = millis();
    if (now - last_battery_sample >= BATTERY_SAMPLE_INTERVAL) {
        // Take a new ADC sample
        uint16_t new_sample = adc_read();
        
        // Add to rolling buffer
        battery_sample_buffer[battery_sample_index] = new_sample;
        battery_sample_index = (battery_sample_index + 1) % 64;
        
        // Calculate average from rolling buffer
        uint32_t total = 0;
        for (int i = 0; i < 64; i++) {
            total += battery_sample_buffer[i];
        }
        uint16_t avg_result = total / 64;
        
        // Update cached voltage
        // Pi Pico W: VSYS is divided by 3 before ADC3, ADC is 12-bit (4096 levels), 3.3V ref
        const float conversion_factor = 3.3f / 4096.0f; // ADC to voltage
        const float vsys_multiplier = 3.0f;             // Account for 3:1 voltage divider
        cached_battery_voltage = avg_result * conversion_factor * vsys_multiplier;
        
        last_battery_sample = now;
        
        // Debug output every 10 seconds
        static unsigned long last_debug = 0;
        if (now - last_debug >= 10000) {
            Serial.printf("Battery sampling: raw=0x%03x, voltage=%.3fV (cached)\n", 
                         avg_result, cached_battery_voltage);
            last_debug = now;
        }
    }
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
    Serial.printf("Post-Render Delay: %lu ms  \r\n", POST_RENDER_DELAY_MS);
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

// ========================================
// PERSISTENT STORAGE FUNCTIONS
// ========================================

// Initialize EEPROM and load settings from persistent storage
void initPersistentStorage() {
    Serial.println("Initializing wear-leveled persistent storage...");
    EEPROM.begin(EEPROM_SIZE);
    
    // Load settings using wear-leveled approach
    RTC_WAKEUP_INTERVAL_MINUTES = loadWakeupIntervalWearLeveled();
    Serial.printf("Loaded RTC wakeup interval with wear leveling: %lu minutes\n", RTC_WAKEUP_INTERVAL_MINUTES);
    
    // Calculate milliseconds from minutes
    RTC_WAKEUP_INTERVAL_MS = RTC_WAKEUP_INTERVAL_MINUTES * 60 * 1000;
    
    Serial.printf("✓ Wear-leveled persistent storage initialized - wakeup interval: %lu minutes\n", RTC_WAKEUP_INTERVAL_MINUTES);
}

// Get current sector address for wear leveling
uint32_t getCurrentSectorAddress() {
    return (eeprom_write_cycle % WEAR_LEVELING_SECTORS) * SECTOR_SIZE;
}

// Save wakeup interval with wear leveling
void saveWakeupIntervalWearLeveled(uint32_t minutes) {
    eeprom_write_cycle++; // Increment wear leveling counter
    uint32_t sector_addr = getCurrentSectorAddress();
    
    Serial.printf("Saving wakeup interval with wear leveling: %lu minutes to sector %lu (cycle %lu)\n", 
                  minutes, sector_addr / SECTOR_SIZE, eeprom_write_cycle);
    
    // Write to wear-leveled sector
    EEPROM.put(sector_addr + MAGIC_OFFSET, EEPROM_MAGIC_VALUE);
    EEPROM.put(sector_addr + WAKEUP_OFFSET, minutes);
    EEPROM.put(sector_addr + CYCLE_OFFSET, eeprom_write_cycle);
    EEPROM.commit();
    
    Serial.println("✓ Wakeup interval saved with wear leveling");
}

// Load wakeup interval with wear leveling
uint32_t loadWakeupIntervalWearLeveled() {
    uint32_t highest_cycle = 0;
    uint32_t best_minutes = DEFAULT_WAKEUP_MINUTES;
    bool found_valid = false;
    
    // Check all wear-leveling sectors to find the most recent valid data
    for (int sector = 0; sector < WEAR_LEVELING_SECTORS; sector++) {
        uint32_t sector_addr = sector * SECTOR_SIZE;
        
        uint32_t magic, minutes, cycle;
        EEPROM.get(sector_addr + MAGIC_OFFSET, magic);
        EEPROM.get(sector_addr + WAKEUP_OFFSET, minutes);
        EEPROM.get(sector_addr + CYCLE_OFFSET, cycle);
        
        // Check if this sector has valid data and is newer
        if (magic == EEPROM_MAGIC_VALUE && cycle > highest_cycle) {
            // Validate wakeup interval
            if (minutes >= 1 && minutes <= 43800) {
                highest_cycle = cycle;
                best_minutes = minutes;
                found_valid = true;
                eeprom_write_cycle = cycle; // Update current cycle counter
                
                Serial.printf("Found valid wear-leveled data in sector %d: %lu minutes (cycle %lu)\n", 
                              sector, minutes, cycle);
            }
        }
    }
    
    if (!found_valid) {
        Serial.println("No valid wear-leveled data found, using default");
        // Initialize with wear leveling
        saveWakeupIntervalWearLeveled(DEFAULT_WAKEUP_MINUTES);
    }
    
    return best_minutes;
}

// Legacy save function (now uses wear leveling)
void saveWakeupInterval(uint32_t minutes) {
    saveWakeupIntervalWearLeveled(minutes);
}

// Legacy load function (now uses wear leveling)  
uint32_t loadWakeupInterval() {
    return loadWakeupIntervalWearLeveled();
}

void setup()
{
    Serial.begin(115200);
    DEV_Delay_ms(20); // startup - reduced from 50ms
    
    Serial.printf("=== RENDERER STARTING ===\r\n");
    Serial.printf("MAXIMUM POWER EFFICIENCY MODE\r\n");
    
    // Initialize persistent storage first (loads RTC wakeup interval)
    initPersistentStorage();
    
    // hardware initialization (moved early for RTC access)
    Serial.printf("hardware init...\r\n");
    if(DEV_Module_Init() != 0) {
        Serial.printf("ERROR: DEV_Module_Init failed!\r\n");
        return;
    }
    
    // RTC initialization
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
    Serial.printf("I2C slave ready at 800kHz\r\n");
    
    // Initialize continuous battery sampling for instant I2C responses
    initBatterySampling();
    Serial.printf("Battery sampling system initialized\r\n");
    
    // Check battery voltage last (now uses cached value)
    float voltage = measureVBAT();
    if (voltage < 3.1) {
        Serial.printf("Low battery voltage: %.2fV - powering off\r\n", voltage);
        ledLowPower();
        powerOff();
        return;
    } else {
        Serial.printf("Battery OK: %.2fV - operation ready\r\n", voltage);
        ledPowerOn(); // Like original PhotoPainter
    }
    
    Serial.printf("=== READY FOR BUTTON/I2C ===\r\n");
    Serial.printf("Total startup: POWER-AWARE - watchdog adapts to power state\r\n");
}

void loop()
{
    // Update battery sampling for accurate cached values
    updateBatterySampling();
    
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
            // LED acknowledgment
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
            Serial.printf("Button pressed (short) in USB mode - requesting next image\n");
            
            // Set flag to request next image
            next_image_requested = true;
            
            // LED feedback
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
        
        // heartbeat (less verbose for power efficiency)
        static unsigned long last_heartbeat = 0;
        if (millis() - last_heartbeat > 60000) { // Every 60 seconds (was 30s)
            Serial.printf("USB mode - %d bytes received\n", received_bytes);
            last_heartbeat = millis();
        }
    }
    
    // polling for maximum button responsiveness and power efficiency
    DEV_Delay_ms(2); // Reduced from 5ms for even faster response while maintaining power efficiency
}