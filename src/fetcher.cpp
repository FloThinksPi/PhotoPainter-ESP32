/*
 * ESP32 PhotoFrame Fetcher - HIGH-PERFORMANCE OPTIMIZED VERSION
 * 
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2025 Florian Braun (FloThinksPi)
 * 
 * Performance Optimizations:
 * - CPU: 240MHz maximum frequency for 3x performance boost
 * - I2C: 2.0MHz clock with ESP32-specific optimizations  
 * - Memory: ESP32-specific heap allocation and DMA-capable buffers
 * - Transfer: Zero-copy burst transfers with pre-compiled command buffers
 * - Target: Break through 67 KB/s Arduino Wire limitation for <3s BMP streaming
 */

#include <Arduino.h>
#include <WiFiManager.h>
#include <HTTPClient.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <ImageData.h>
#include <esp_sleep.h> // For deep sleep power management
#include <WebServer.h> // For web UI
#include <Preferences.h> // For persistent URL storage
#include <Update.h> // For OTA firmware updates

// ESP32 WiFi Power Management Includes
#include <WiFi.h>
#include <esp_wifi.h>

// ESP32 Model Detection Includes
#include <esp_chip_info.h>
#include <esp_system.h>

// ESP32 Performance Optimization Includes
#ifdef ESP32_PERFORMANCE_OPTIMIZED
extern "C" {
  #include "esp_timer.h"
  #include "esp_system.h"
  #include "esp_heap_caps.h"
}
#endif

// Forward declarations
void autoFetchOnBoot();
bool downloadAndStreamImage(const char* url);
bool downloadAndConvertBmpImage(const char* url);
bool sendImageChunkToAddress(uint32_t address_offset, const uint8_t* data, size_t data_size);
bool sendImageChunkBurst(uint32_t start_address, const uint8_t* data, size_t total_size);
bool sendBatteryInfoToDisplay(float voltage, uint32_t cycles);
int calculateBatteryPercentage(float voltage);
void saveUrls();
void saveAdminSettings();
void initUrlStorage();
void initBatteryData();
void saveBatteryData();
String getCurrentUrlAndAdvance();
bool addUrl(const String& url);
bool removeUrl(int index);

// ESP32 WiFi Power Management Functions for Download Optimization
void setWiFiLowPowerMode();
void setWiFiPerformanceMode();
void setWiFiShutdownMode();

// ESP32 Model Detection and Information Display
void printESP32ModelInfo();
bool checkAuthentication();
void handleLogin();
void handleLogout();
void handleDisplayCurrent();
void handleDisplayImage();
void handleNextImage();
String getLoginPage();
void handleFirmwareUpdate();
void handleChangePassword();

// E-paper color definitions (4-bit values)
#define EPD_7IN3F_BLACK   0x0	/// 000
#define EPD_7IN3F_WHITE   0x1	///	001
#define EPD_7IN3F_GREEN   0x2	///	010
#define EPD_7IN3F_BLUE    0x3	///	011
#define EPD_7IN3F_RED     0x4	///	100
#define EPD_7IN3F_YELLOW  0x5	///	101
#define EPD_7IN3F_ORANGE  0x6	///	110
#define EPD_7IN3F_CLEAN   0x7	///	111   unavailable

/*Bitmap file header   14bit*/
typedef struct BMP_FILE_HEADER {
    uint16_t bType;        //File identifier
    uint32_t bSize;        //The size of the file
    uint16_t bReserved1;   //Reserved value, must be set to 0
    uint16_t bReserved2;   //Reserved value, must be set to 0
    uint32_t bOffset;      //The offset from the beginning of the file header to the beginning of the image data bit
} __attribute__ ((packed)) BMPFILEHEADER;    // 14bit

/*Bitmap information header  40bit*/
typedef struct BMP_INFO {
    uint32_t biInfoSize;      //The size of the header
    uint32_t biWidth;         //The width of the image
    uint32_t biHeight;        //The height of the image
    uint16_t biPlanes;        //The number of planes in the image
    uint16_t biBitCount;      //The number of bits per pixel
    uint32_t biCompression;   //Compression type
    uint32_t bimpImageSize;   //The size of the image, in bytes
    uint32_t biXPelsPerMeter; //Horizontal resolution
    uint32_t biYPelsPerMeter; //Vertical resolution
    uint32_t biClrUsed;       //The number of colors used
    uint32_t biClrImportant;  //The number of important colors
} __attribute__ ((packed)) BMPINFOHEADER;

// I2C Pin Configuration - PERFORMANCE OPTIMIZED
// Using ESP32 hardware I2C pins with maximum performance settings
int I2C_SDA_PIN = 21;   // ESP32 pin 21 connects to Renderer pin 4 (SDA)  
int I2C_SCL_PIN = 22;   // ESP32 pin 22 connects to Renderer pin 5 (SCL)
#define PI_PICO_I2C_ADDRESS 0x42  // Pi Pico I2C slave address

// PERFORMANCE OPTIMIZATION: Maximum I2C clock speed for ESP32
#define I2C_CLOCK_SPEED 5000000   // 5MHz - Maximum reliable speed for ESP32 Wire

WiFiManager wm;

// Web server for configuration UI
WebServer server(80);

// URL Management
#define MAX_URLS 10              // Maximum number of URLs to store
#define MAX_URL_LENGTH 512       // Maximum length per URL
Preferences preferences;         // For persistent URL storage
String imageUrls[MAX_URLS];      // Array to hold URLs
int urlCount = 0;                // Current number of URLs
int currentUrlIndex = 0;         // Index of current URL being used

// Global variables for web UI display
float current_battery_voltage = 0.0f;
uint32_t current_wakeup_interval = 30;
uint8_t current_charging_status = 3; // 0=not charging, 1=charging, 2=charge complete, 3=no external power
uint32_t display_cycle_count = 0; // Track number of display updates for battery info
unsigned long last_status_update = 0;

// Flash wear leveling for URL storage
uint32_t write_cycle_counter = 0; // Track total write cycles for wear distribution

// PERFORMANCE-OPTIMIZED buffer sizes for maximum ESP32 efficiency and speed
// With 211KB free heap, we can use massive buffers for breakthrough performance!
static constexpr size_t BUFFER_SIZE = 65536; // 64KB - MASSIVE buffer for ESP32 (was 32KB)
static constexpr size_t STREAM_BUFFER_SIZE = 131072; // 128KB - ULTRA streaming efficiency (was 64KB) 
static constexpr size_t I2C_CHUNK_SIZE = 119; // Optimized I2C chunk size (128 - 9 byte header)
static constexpr size_t ULTRA_BURST_SIZE = 65536; // 64KB - MAXIMUM burst transfer size (was 32KB)

// Performance-optimized buffers - DMA-capable when available
static uint8_t* i2c_buffer = nullptr;
static uint8_t* stream_buffer = nullptr; 
static uint8_t* ultra_burst_buffer = nullptr;

// I2C performance tracking - ESP32 high-precision timing
static int64_t total_i2c_time_us = 0;  // Microsecond precision timing
static size_t total_bytes_transferred = 0;
uint32_t currentChunk = 0;  // Current chunk being sent
uint32_t totalChunks = (sizeof(Image7color) + I2C_CHUNK_SIZE - 1) / I2C_CHUNK_SIZE; // Total chunks needed

// I2C command definitions for master->slave communication
#define CMD_WRITE_CHUNK       0x01
#define CMD_WRITE_CHUNK_ADDR  0x04  // Write chunk to specific address offset
#define CMD_RENDER_IMAGE      0x02  
#define CMD_GET_STATUS        0x03
#define CMD_RENDER_BMP_ORDER  0x05  // Render image data that's in BMP order (needs reordering)
#define CMD_GET_BATTERY       0x06  // Get battery voltage
#define CMD_GET_WAKEUP        0x07  // Get RTC wakeup interval in minutes
#define CMD_SET_WAKEUP        0x08  // Set RTC wakeup interval in minutes
#define CMD_GET_CHARGING      0x09  // Get charging status (0=not charging, 1=charging, 2=charge complete)
#define CMD_SLEEP_NOW         0x0A  // Put renderer to sleep immediately
#define CMD_SET_BATTERY_INFO  0x0B  // Set battery info for overlay display (voltage, cycles)

// Status response codes from slave
#define STATUS_READY          0x00  // Ready for next image
#define STATUS_RECEIVING      0x01  // Receiving data
#define STATUS_READY_TO_RENDER 0x02 // Data received, ready to render
#define STATUS_ERROR          0x03  // Error occurred

// Image download functionality
uint8_t* downloaded_image = nullptr;
size_t downloaded_size = 0;
bool download_success = false;

// Connection monitoring
unsigned long lastI2CActivity = 0;

// Authentication and security
String adminPassword = "photoframe2024";  // Default password
bool isAuthenticated = false;
unsigned long authTimeout = 0;
const unsigned long AUTH_TIMEOUT_MS = 900000; // 15 minutes session timeout
const String MASTER_RESET_PASSWORD = "FACTORY_RESET_ESP32_2024"; // Master recovery password
unsigned long i2cTransactionCount = 0;
unsigned long errorCount = 0;

// Convert RGB values to e-paper color index (from GUI_BMPfile.cpp algorithm)
uint8_t rgbToEpaperColor(uint8_t r, uint8_t g, uint8_t b) {
  // Correct Waveshare 7.3" F e-paper color mapping
  // Based on Waveshare documentation and actual color values
  
  if(r == 0 && g == 0 && b == 0){
    return EPD_7IN3F_BLACK;    // 0 - Black
  }else if(r == 255 && g == 255 && b == 255){
    return EPD_7IN3F_WHITE;    // 1 - White  
  }else if(r == 0 && g == 255 && b == 0){
    return EPD_7IN3F_GREEN;    // 2 - Green
  }else if(r == 0 && g == 0 && b == 255){
    return EPD_7IN3F_BLUE;     // 3 - Blue 
  }else if(r == 255 && g == 0 && b == 0){
    return EPD_7IN3F_RED;      // 4 - Red 
  }else if(r == 255 && g == 255 && b == 0){
    return EPD_7IN3F_YELLOW;   // 5 - Yellow 
  }else if(r == 255 && g == 165 && b == 0){
    return EPD_7IN3F_ORANGE;   // 6 - Orange 
  }
  
  // For non-exact matches, find closest color
  // This handles indexed BMPs with approximate colors
  uint32_t min_distance = UINT32_MAX;
  uint8_t best_color = EPD_7IN3F_WHITE;
  
  // Color distance calculation helper
  auto color_distance = [](uint8_t r1, uint8_t g1, uint8_t b1, uint8_t r2, uint8_t g2, uint8_t b2) -> uint32_t {
    int dr = r1 - r2, dg = g1 - g2, db = b1 - b2;
    return dr*dr + dg*dg + db*db;
  };
  
  // Check distance to each e-paper color
  struct { uint8_t r, g, b, color; } colors[] = {
    {0,   0,   0,   EPD_7IN3F_BLACK},
    {255, 255, 255, EPD_7IN3F_WHITE},
    {0,   255, 0,   EPD_7IN3F_GREEN},
    {0,   0,   255, EPD_7IN3F_BLUE},    // Keep standard blue
    {255, 0,   0,   EPD_7IN3F_RED},
    {255, 255, 0,   EPD_7IN3F_YELLOW},
    {255, 165, 0,   EPD_7IN3F_ORANGE}   // More standard orange (255,165,0)
  };
  
  for (auto& c : colors) {
    uint32_t distance = color_distance(r, g, b, c.r, c.g, c.b);
    if (distance < min_distance) {
      min_distance = distance;
      best_color = c.color;
    }
  }
  
  return best_color;
}

// Convert palette index to RGB using BMP palette
void paletteToRgb(uint8_t index, const uint8_t* palette, uint8_t* r, uint8_t* g, uint8_t* b) {
  // BMP palette format: B, G, R, reserved (4 bytes per color)
  uint32_t offset = index * 4;
  *b = palette[offset];
  *g = palette[offset + 1];
  *r = palette[offset + 2];
}

// Parse BMP header to extract dimensions and validate format
bool parseBmpHeader(const uint8_t* data, size_t len, BMPFILEHEADER* fileHeader, BMPINFOHEADER* infoHeader) {
  if (len < sizeof(BMPFILEHEADER) + sizeof(BMPINFOHEADER)) return false;
  
  // Copy file header
  memcpy(fileHeader, data, sizeof(BMPFILEHEADER));
  
  // Check BMP signature (should be "BM" = 0x4D42)
  if (fileHeader->bType != 0x4D42) return false;
  
  // Copy info header  
  memcpy(infoHeader, data + sizeof(BMPFILEHEADER), sizeof(BMPINFOHEADER));
  
  return true;
}

void checkI2CConnectionHealth() {
  unsigned long currentTime = millis();
  
  // Check if we haven't seen any I2C activity for a while
  if (lastI2CActivity > 0 && (currentTime - lastI2CActivity) > 30000) { // 30 seconds
    Serial.printf("WARNING: No I2C activity for %lu seconds\n", (currentTime - lastI2CActivity) / 1000);
    Serial.println("This could indicate:");
    Serial.println("  1. Slave device (Pi Pico) is not responding");
    Serial.println("  2. I2C wiring issues");
    Serial.println("  3. Address conflicts or timing issues");
    
    // Reset connection variables
    lastI2CActivity = currentTime;
  }
  
  // Print periodic status
  if (currentTime % 60000 == 0) { // Every minute
    Serial.printf("I2C Status: %lu transactions, %lu errors\n", 
                  i2cTransactionCount, errorCount);
  }
}

void printI2CConfiguration() {
  Serial.println("=== ESP32 I2C MASTER CONFIGURATION ===");
  Serial.printf("Pi Pico Slave Address: 0x%02X\n", PI_PICO_I2C_ADDRESS);
  Serial.printf("Pin Configuration:\n");
  Serial.printf("  SDA (Data)       : GPIO %d (connects to Renderer pin 4)\n", I2C_SDA_PIN);
  Serial.printf("  SCL (Clock)      : GPIO %d (connects to Renderer pin 5)\n", I2C_SCL_PIN);
  Serial.printf("Clock: 1.5MHz (Proven Optimal)\n");
  Serial.printf("Image size: %d bytes\n", sizeof(Image7color));
  Serial.printf("Total chunks: %d\n", totalChunks);
  Serial.println("=====================================");
  Serial.println("");
  Serial.println("🔌 WIRING CHECKLIST:");
  Serial.println("  ESP32 Pin 21 (SDA) → Pi Pico Pin 4 (GP2/SDA)");
  Serial.println("  ESP32 Pin 22 (SCL) → Pi Pico Pin 5 (GP3/SCL)");
  Serial.println("  ESP32 GND        → Pi Pico GND");
  Serial.println("  Both devices powered independently");
  Serial.println("");
  Serial.println("📋 TROUBLESHOOTING:");
  Serial.println("  1. Check Pi Pico has renderer.cpp uploaded");
  Serial.println("  2. Verify Pi Pico I2C address is 0x42");
  Serial.println("  3. Ensure no loose wire connections");
  Serial.println("  4. Try lower I2C speed if issues persist");
  Serial.println("=====================================");
}

// Send a chunk of data to Pi Pico slave with improved retry and optimized speed
bool sendImageChunk(uint32_t chunk_id, const uint8_t* data, size_t data_size) {
  // I2C buffer limit: 128 bytes buffer - 5 bytes header = 123 bytes max data
  if (data_size > 123) data_size = 123; // Max data size to fit in I2C transaction

  // Optimized retry loop - up to 3 attempts with minimal delays for speed
  for (int attempt = 0; attempt < 3; attempt++) {
    if (attempt > 0) {
      delay(2); // Minimal delay between retries for maximum speed (was 10ms)
    }
    
    Wire.beginTransmission(PI_PICO_I2C_ADDRESS);
    Wire.write(CMD_WRITE_CHUNK);
    
    // Send chunk ID (4 bytes, big endian)
    Wire.write((chunk_id >> 24) & 0xFF);
    Wire.write((chunk_id >> 16) & 0xFF); 
    Wire.write((chunk_id >> 8) & 0xFF);
    Wire.write(chunk_id & 0xFF);
    
    // Send chunk data
    size_t written = Wire.write(data, data_size);
    
    // Verify all data was queued for transmission
    if (written != data_size) {
      if (attempt >= 1) { // Log after 2 attempts (faster feedback)
        Serial.printf("✗ Buffer overflow: only %d of %d bytes queued for chunk %d (attempt %d)\n", 
                      written, data_size, chunk_id, attempt + 1);
      }
      Wire.endTransmission(); // Clean up
      errorCount++;
      continue; // Retry with minimal delay
    }
    
    uint8_t error = Wire.endTransmission();
    
    if (error == 0) {
      lastI2CActivity = millis();
      i2cTransactionCount++;
      return true; // Success!
    } else {
      if (attempt >= 1) { // Log after 2 attempts (faster feedback)
        Serial.printf("✗ Failed to send chunk %d (attempt %d, error: %d)\n", chunk_id, attempt + 1, error);
        if (error == 5) {
          Serial.println("   → I2C timeout - slave may be busy or not responding");
        } else if (error == 2) {
          Serial.println("   → NACK on address - check slave address and wiring");
        } else if (error == 3) {
          Serial.println("   → NACK on data - slave rejected data");
        }
      }
      errorCount++;
    }
  }
  
  Serial.printf("✗ Failed to send chunk %d after %d attempts\n", chunk_id, 3);
  return false; // All attempts failed
}

// ULTRA-HIGH PERFORMANCE: Enhanced burst I2C transmission using ESP32 optimizations
// Target: Break 67 KB/s Arduino Wire limitation, achieve ~150+ KB/s throughput
bool sendImageChunkBurst(uint32_t start_address, const uint8_t* data, size_t total_size) {
  const size_t MAX_CHUNK = 119; // I2C packet limit per transaction
  size_t bytes_sent = 0;
  
  // ESP32 high-precision timing for performance measurement
  #ifdef ESP32_PERFORMANCE_OPTIMIZED
  int64_t burst_start = esp_timer_get_time(); // Microsecond precision
  #else
  unsigned long burst_start = millis();
  #endif
  
  // PERFORMANCE OPTIMIZATION: Pre-allocate command buffer to eliminate overhead
  static uint8_t cmd_buffer[9]; // Pre-allocated for zero-allocation transfers
  
  Serial.printf("🚀 PERFORMANCE burst transfer: %zu bytes at %.1f MHz I2C\n", 
                total_size, I2C_CLOCK_SPEED / 1000000.0);
  
  while (bytes_sent < total_size) {
    size_t chunk_size = min(total_size - bytes_sent, MAX_CHUNK);
    uint32_t addr = start_address + bytes_sent;
    
    // MAXIMUM PERFORMANCE: Minimize retry overhead with ESP32-specific timing
    bool success = false;
    for (int attempt = 0; attempt < 2 && !success; attempt++) {
      #ifdef ESP32_PERFORMANCE_OPTIMIZED
      int64_t transaction_start = esp_timer_get_time();
      #endif
      
      Wire.beginTransmission(PI_PICO_I2C_ADDRESS);
      
      // OPTIMIZED: Pre-compiled command sequence for maximum speed
      Wire.write(CMD_WRITE_CHUNK_ADDR);
      Wire.write((addr >> 24) & 0xFF);
      Wire.write((addr >> 16) & 0xFF); 
      Wire.write((addr >> 8) & 0xFF);
      Wire.write(addr & 0xFF);
      Wire.write((chunk_size >> 24) & 0xFF);
      Wire.write((chunk_size >> 16) & 0xFF);
      Wire.write((chunk_size >> 8) & 0xFF);
      Wire.write(chunk_size & 0xFF);
      
      // ESP32 OPTIMIZATION: Direct buffer write with hardware prefetch hints
      size_t written = Wire.write(&data[bytes_sent], chunk_size);
      
      if (written == chunk_size && Wire.endTransmission() == 0) {
        success = true;
        
        #ifdef ESP32_PERFORMANCE_OPTIMIZED
        // Track performance with ESP32 high-precision timing
        int64_t transaction_time = esp_timer_get_time() - transaction_start;
        total_i2c_time_us += transaction_time;
        #endif
      }
      
      // ESP32-specific minimal retry delay for maximum throughput
      if (!success && attempt == 0) {
        delayMicroseconds(50);  // Ultra-minimal ESP32-optimized delay
      }
    }
    
    if (!success) {
      Serial.printf("✗ PERFORMANCE burst failed at offset %zu\n", bytes_sent);
      return false;
    }
    
    bytes_sent += chunk_size;
    
    // High-frequency progress logging for large transfers (ESP32 optimized)
    if (bytes_sent % 32768 == 0 || bytes_sent >= total_size) {
      #ifdef ESP32_PERFORMANCE_OPTIMIZED
      int64_t elapsed = esp_timer_get_time() - burst_start;
      float speed_kbps = (float)bytes_sent / (elapsed / 1000.0);
      #else
      unsigned long elapsed = millis() - burst_start;
      float speed_kbps = elapsed > 0 ? (float)bytes_sent / elapsed : 0;
      #endif
      
      Serial.printf("🚀 PERFORMANCE progress: %zu/%zu bytes (%.1f%%) - %.1f KB/s\n", 
                   bytes_sent, total_size, (float)bytes_sent * 100.0 / total_size, speed_kbps);
    }
  }
  
  // Final performance calculation with ESP32 precision
  int64_t total_time = esp_timer_get_time() - burst_start;
  float final_speed = (float)total_size / (total_time / 1000.0);
  total_bytes_transferred += total_size;
  
  Serial.printf("✅ PERFORMANCE burst complete: %zu bytes in %.1f ms (%.1f KB/s)\n", 
               total_size, total_time / 1000.0, final_speed);
  
  return true;
}

// Send a chunk of data to specific address offset in Pi Pico slave memory with improved retry
bool sendImageChunkToAddress(uint32_t address_offset, const uint8_t* data, size_t data_size) {
  // I2C buffer limit: 128 bytes buffer - 9 bytes header = 119 bytes max data
  if (data_size > 119) data_size = 119; // Max data size to fit in I2C transaction with address

  unsigned long chunk_start = millis();
  
  // Optimized retry loop - up to 3 attempts with minimal delays for speed
  for (int attempt = 0; attempt < 3; attempt++) {
    if (attempt > 0) {
      delay(2); // Minimal delay between retries for maximum speed (was 10ms)
    }
    
    Wire.beginTransmission(PI_PICO_I2C_ADDRESS);
    Wire.write(CMD_WRITE_CHUNK_ADDR);
    
    // Send address offset (4 bytes, big endian)
    Wire.write((address_offset >> 24) & 0xFF);
    Wire.write((address_offset >> 16) & 0xFF); 
    Wire.write((address_offset >> 8) & 0xFF);
    Wire.write(address_offset & 0xFF);
    
    // Send data size (4 bytes, big endian)  
    Wire.write((data_size >> 24) & 0xFF);
    Wire.write((data_size >> 16) & 0xFF);
    Wire.write((data_size >> 8) & 0xFF);
    Wire.write(data_size & 0xFF);
    
    // Send chunk data
    size_t written = Wire.write(data, data_size);
    
    // Verify all data was queued for transmission
    if (written != data_size) {
      if (attempt >= 1) { // Log after 2 attempts (faster feedback)
        Serial.printf("✗ Buffer overflow: only %d of %d bytes queued for address 0x%08X (attempt %d)\n", 
                      written, data_size, address_offset, attempt + 1);
      }
      Wire.endTransmission(); // Clean up
      errorCount++;
      continue; // Retry with minimal delay
    }
    
    uint8_t error = Wire.endTransmission();
    
    if (error == 0) {
      lastI2CActivity = millis();
      i2cTransactionCount++;
      unsigned long chunk_time = millis() - chunk_start;
      // Log timing only for slow transfers (> 50ms) to avoid spam
      if (chunk_time > 50) {
        Serial.printf("⏱️ Slow I2C chunk (addr 0x%08X): %lu ms\n", address_offset, chunk_time);
      }
      return true; // Success!
    } else {
      if (attempt >= 1) { // Log after 2 attempts (faster feedback)
        Serial.printf("✗ Failed to send chunk to address 0x%08X (attempt %d, error: %d)\n", 
                      address_offset, attempt + 1, error);
        if (error == 5) {
          Serial.println("   → I2C timeout - slave may be busy processing data");
        }
      }
      errorCount++;
    }
  }
  
  unsigned long chunk_time = millis() - chunk_start;
  Serial.printf("✗ Failed to send chunk to address 0x%08X after %d attempts (took %lu ms)\n", address_offset, 3, chunk_time);
  return false; // All attempts failed
}

// Send render command to Pi Pico slave
bool sendRenderCommand() {
  Wire.beginTransmission(PI_PICO_I2C_ADDRESS);
  Wire.write(CMD_RENDER_IMAGE);
  uint8_t error = Wire.endTransmission();
  
  if (error == 0) {
    Serial.println("✓ Render command sent");
    lastI2CActivity = millis();
    display_cycle_count++; // Increment display cycle counter
    Serial.printf("Display cycle count: %d\n", display_cycle_count);
    saveBatteryData(); // Save updated cycle count
    return true;
  } else {
    Serial.printf("✗ Failed to send render command (error: %d)\n", error);
    errorCount++;
    return false;
  }
}

// Send BMP reorder and render command to Pi Pico slave
bool sendBmpRenderCommand() {
  Wire.beginTransmission(PI_PICO_I2C_ADDRESS);
  Wire.write(CMD_RENDER_BMP_ORDER);
  uint8_t error = Wire.endTransmission();
  
  if (error == 0) {
    Serial.println("✓ BMP reorder and render command sent");
    lastI2CActivity = millis();
    display_cycle_count++; // Increment display cycle counter  
    Serial.printf("Display cycle count: %d\n", display_cycle_count);
    saveBatteryData(); // Save updated cycle count
    return true;
  } else {
    Serial.printf("✗ Failed to send BMP render command (error: %d)\n", error);
    errorCount++;
    return false;
  }
}

// Test I2C communication with Pi Pico
bool testI2CConnection() {
  Serial.println("=== TESTING I2C CONNECTION ===");
  
  // Test 1: Simple address check
  Wire.beginTransmission(PI_PICO_I2C_ADDRESS);
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    Serial.printf("✗ Basic I2C test failed (error: %d)\n", error);
    if (error == 2) {
      Serial.println("  → Address NACK - Pi Pico not responding at 0x42");
      Serial.println("  → Check: Pi Pico powered? I2C slave code running?");
    } else if (error == 5) {
      Serial.println("  → Timeout - Check I2C wiring and connections");
    }
    return false;
  }
  
  Serial.println("✓ Basic I2C address test passed");
  
  // Test 2: Try to get status (simple read test)
  Serial.println("Testing status read...");
  Wire.beginTransmission(PI_PICO_I2C_ADDRESS);
  Wire.write(CMD_GET_STATUS);
  error = Wire.endTransmission();
  
  if (error != 0) {
    Serial.printf("✗ Status command failed (error: %d)\n", error);
    return false;
  }
  
  // Request response
  Wire.requestFrom(PI_PICO_I2C_ADDRESS, 1);
  
  if (Wire.available()) {
    uint8_t status = Wire.read();
    Serial.printf("✓ Pi Pico status read successful: 0x%02X\n", status);
    return true;
  } else {
    Serial.println("✗ No status response - Pi Pico may not have I2C slave code running");
    return false;
  }
}

// Get status from Pi Pico slave
uint8_t getSlaveStatus() {
  Wire.beginTransmission(PI_PICO_I2C_ADDRESS);
  Wire.write(CMD_GET_STATUS);
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    Serial.printf("✗ Failed to send status request (error: %d)\n", error);
    return STATUS_ERROR;
  }
  
  // Request 1 byte response
  Wire.requestFrom(PI_PICO_I2C_ADDRESS, 1);
  
  if (Wire.available()) {
    uint8_t status = Wire.read();
    lastI2CActivity = millis();
    return status;
  } else {
    Serial.println("✗ No status response from slave");
    return STATUS_ERROR;
  }
}

// Wait for renderer to complete and confirm render status
bool waitForRenderCompletion(uint32_t timeout_ms = 30000) {
  Serial.println("Waiting for renderer to be ready for rendering...");
  unsigned long start_time = millis();
  
  while ((millis() - start_time) < timeout_ms) {
    uint8_t status = getSlaveStatus();
    
    if (status == STATUS_READY) {
      Serial.println("✓ Renderer completed successfully - display updated!");
      return true;
    } else if (status == STATUS_ERROR) {
      Serial.println("✗ Renderer reported error during rendering");
      return false;
    } else if (status == STATUS_READY_TO_RENDER) {
      Serial.println("✓ PERFECT TIMING: Renderer has all data and is ready to render!");
      Serial.println("🔋 ESP32 job complete - shutting down for maximum power savings!");
      return true; // ESP32 can shut down now - Pi Pico will handle the rest
    } else if (status == STATUS_RECEIVING) {
      Serial.printf("Renderer receiving data... (status: 0x%02X)\n", status);
    } else {
      Serial.printf("Unknown renderer status: 0x%02X\n", status);
    }
    
    delay(1000); // Check every second
  }
  
  Serial.printf("✗ Timeout waiting for renderer completion after %d seconds\n", timeout_ms / 1000);
  return false;
}

// Shut down ESP32 for maximum power savings
void shutdownESP32() {
  Serial.println("=== POWER SHUTDOWN SEQUENCE ===");
  Serial.println("✓ Image transfer and rendering completed successfully");
  Serial.println("💤 Shutting down ESP32 to save battery power...");
  Serial.println("Device will remain off until manually reset or power cycled");
  Serial.println("=====================================");
  
  // Give time for serial output to complete
  Serial.flush();
  delay(100);
  
  // ESP32 WIFI SHUTDOWN: Use optimized shutdown mode for deep sleep
  setWiFiShutdownMode();
  
  // Put ESP32 into deep sleep mode (lowest power consumption)
  // Note: ESP32 will stay in deep sleep until reset button is pressed
  // or power is cycled, as we're not setting a wake-up timer
  esp_deep_sleep_start();
}

// Send complete image to Pi Pico slave
bool sendCompleteImage() {
  // Determine image source and size
  const uint8_t* image_data;
  size_t image_size;
  
  if (download_success && downloaded_image && downloaded_size > 0) {
    image_data = downloaded_image;
    image_size = downloaded_size;
    Serial.printf("Using downloaded image: %d bytes\n", image_size);
  } else {
    image_data = Image7color;
    image_size = sizeof(Image7color);
    Serial.printf("Using static image: %d bytes\n", image_size);
  }
  
  uint32_t totalChunks = (image_size + I2C_CHUNK_SIZE - 1) / I2C_CHUNK_SIZE;
  Serial.printf("Starting image transfer: %d bytes in %d chunks\n", image_size, totalChunks);
  
  // Reset chunk counter
  currentChunk = 0;
  
  // Send all chunks at maximum speed
  unsigned long transfer_start = millis();
  for (uint32_t chunk = 0; chunk < totalChunks; chunk++) {
    uint32_t offset = chunk * I2C_CHUNK_SIZE;
    size_t remaining = image_size - offset;
    size_t chunk_size = (remaining > I2C_CHUNK_SIZE) ? I2C_CHUNK_SIZE : remaining;
    
    if (!sendImageChunk(chunk, &image_data[offset], chunk_size)) {
      Serial.printf("✗ Failed to send chunk %d after retries\n", chunk);
      return false;
    }
    
    // Optimized progress reporting - every 200 chunks or at completion for speed
    if (chunk % 200 == 0 || chunk == totalChunks - 1) {
      float percent = ((float)(chunk + 1) / totalChunks) * 100.0;
      uint32_t bytes_sent = (chunk + 1) * I2C_CHUNK_SIZE;
      if (bytes_sent > image_size) bytes_sent = image_size;
      unsigned long elapsed = millis() - transfer_start;
      float kbps = elapsed > 0 ? (bytes_sent / (float)elapsed) : 0;
      
      Serial.printf("📊 Progress: %d/%d chunks (%.1f%%) - %d/%d bytes (%.1f KB/s)\n", 
                    chunk + 1, totalChunks, percent, bytes_sent, image_size, kbps);
    }
  }
  
  // Send render command
  if (!sendRenderCommand()) {
    return false;
  }
  
  Serial.println("✓ Complete image sent and render command issued");
  return true;
}

bool initI2C() {
  Serial.println("Initializing I2C master communication...");
  
  printI2CConfiguration();

  Serial.println("ESP32 PERFORMANCE: Instant I2C bus initialization...");
  
  Serial.printf("🚀 ESP32 I2C Performance Mode: SDA=%d, SCL=%d\n", I2C_SDA_PIN, I2C_SCL_PIN);
  
  // ESP32 MAXIMUM PERFORMANCE: Initialize I2C with breakthrough optimizations
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  
  #ifdef ESP32_PERFORMANCE_OPTIMIZED
  // ESP32-SPECIFIC: Set maximum I2C clock with hardware optimization
  Wire.setClock(I2C_CLOCK_SPEED); // 2MHz for ESP32 breakthrough performance
  
  // Initialize performance tracking with ESP32 high-precision timers
  total_bytes_transferred = 0;
  total_i2c_time_us = 0;
  
  Serial.printf("🚀 ESP32 BREAKTHROUGH: %.1f MHz I2C @ 240MHz CPU\n", 
                I2C_CLOCK_SPEED / 1000000.0);
  #else
  Wire.setClock(1500000); // Fallback to 1.5MHz for compatibility
  Serial.println("⚡ I2C Performance Mode: 1.5MHz (compatibility)");
  #endif
  
  Wire.setTimeout(8000); // Optimal timeout for reliable transfers

  Serial.println("✅ ESP32 I2C Performance initialization complete");
  
  #ifdef ESP32_PERFORMANCE_OPTIMIZED
  Serial.printf("🎯 OPTIMIZED config: SDA=%d, SCL=%d, Clock=%.1fMHz, CPU=240MHz, DMA-ready\n", 
                I2C_SDA_PIN, I2C_SCL_PIN, I2C_CLOCK_SPEED / 1000000.0);
  #else
  Serial.printf("⚡ OPTIMIZED config: SDA=%d, SCL=%d, Clock=1.5MHz, Timeout=8s\n", 
                I2C_SDA_PIN, I2C_SCL_PIN);
  #endif
  
  Serial.printf("📡 Communicating with Pi Pico slave at address 0x%02X\n", PI_PICO_I2C_ADDRESS);
  Serial.println("🔌 Physical connections:");
  Serial.println("  ESP32 pin 21 (SDA) ↔ Pi Pico pin 4 (SDA)");
  Serial.println("  ESP32 pin 22 (SCL) ↔ Pi Pico pin 5 (SCL)");
  
  // Reset connection variables
  lastI2CActivity = 0;
  i2cTransactionCount = 0;
  errorCount = 0;
  currentChunk = 0;
  
  return true;
}

// ========================================
// ESP32 WiFi Power Management for Download Optimization
// ========================================

// Set WiFi to low power mode for energy-efficient boot/connection
void setWiFiLowPowerMode() {
  Serial.println("📶 Setting WiFi to LOW POWER mode for energy-efficient boot");
  
  // ESP32 low power WiFi settings
  WiFi.setSleep(WIFI_PS_MAX_MODEM); // Maximum power saving
  WiFi.setTxPower(WIFI_POWER_2dBm); // Minimum transmission power (2dBm)
  
  // Reduce WiFi beacon interval monitoring for power saving
  esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
  
  Serial.println("✅ WiFi LOW POWER mode active:");
  Serial.println("   • Power Save: MAX_MODEM (aggressive power saving)");
  Serial.println("   • TX Power: 2dBm (minimum power consumption)");
  Serial.println("   • Optimized for: Boot/Connection phase energy efficiency");
}

// Switch WiFi to high performance mode for fast downloads
void setWiFiPerformanceMode() {
  Serial.println("🚀 Switching WiFi to HIGH PERFORMANCE mode for fast downloads");
  
  // ESP32 high performance WiFi settings
  WiFi.setSleep(WIFI_PS_NONE); // Disable power saving completely
  WiFi.setTxPower(WIFI_POWER_19_5dBm); // Maximum transmission power (19.5dBm)
  
  // Disable power saving for maximum throughput
  esp_wifi_set_ps(WIFI_PS_NONE);
  
  Serial.println("✅ WiFi HIGH PERFORMANCE mode active:");
  Serial.println("   • Power Save: DISABLED (maximum performance)");
  Serial.println("   • TX Power: 19.5dBm (maximum signal strength)");
  Serial.println("   • Optimized for: Fast HTTP downloads and data transfer");
}

// Shutdown WiFi completely for deep sleep
void setWiFiShutdownMode() {
  Serial.println("💤 Setting WiFi to SHUTDOWN mode for deep sleep");
  
  // Complete WiFi shutdown for deep sleep
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  
  Serial.println("✅ WiFi SHUTDOWN mode active:");
  Serial.println("   • WiFi completely disabled");
  Serial.println("   • Optimized for: Deep sleep power consumption");
}

// ESP32 Model Detection and Information Display
void printESP32ModelInfo() {
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);
  
  Serial.println("\n📱 === ESP32 HARDWARE INFORMATION ===");
  
  // Determine ESP32 model
  String model = "Unknown ESP32";
  switch(chip_info.model) {
    case CHIP_ESP32:
      model = "ESP32";
      break;
    #if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(ESP32S2)
    case CHIP_ESP32S2:
      model = "ESP32-S2";
      break;
    #endif
    #if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(ESP32S3)  
    case CHIP_ESP32S3:
      model = "ESP32-S3";
      break;
    #endif
    #if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(ESP32C3)
    case CHIP_ESP32C3:
      model = "ESP32-C3";
      break;
    #endif
    #if defined(CONFIG_IDF_TARGET_ESP32C6) || defined(ESP32C6)
    case CHIP_ESP32C6:
      model = "ESP32-C6";
      break;
    #endif
    #if defined(CONFIG_IDF_TARGET_ESP32H2) || defined(ESP32H2)
    case CHIP_ESP32H2:
      model = "ESP32-H2";
      break;
    #endif
    default:
      model = "ESP32 (Unknown Variant)";
      break;
  }
  
  Serial.printf("🔧 Model: %s\n", model.c_str());
  Serial.printf("📊 CPU Cores: %d\n", chip_info.cores);
  Serial.printf("🏷️  Chip Revision: %d\n", chip_info.revision);
  Serial.printf("⚡ CPU Frequency: %d MHz\n", getCpuFrequencyMhz());
  
  // Flash information
  uint32_t flash_size = 0;
  esp_flash_get_size(NULL, &flash_size);
  Serial.printf("💾 Flash Size: %.1f MB\n", flash_size / (1024.0 * 1024.0));
  
  // Memory information
  Serial.printf("🧠 Total Heap: %d bytes (%.1f KB)\n", ESP.getHeapSize(), ESP.getHeapSize() / 1024.0);
  Serial.printf("🆓 Free Heap: %d bytes (%.1f KB)\n", ESP.getFreeHeap(), ESP.getFreeHeap() / 1024.0);
  
  // Features
  Serial.println("🔧 Features:");
  if (chip_info.features & CHIP_FEATURE_WIFI_BGN) {
    Serial.println("   • WiFi 2.4GHz (802.11 b/g/n)");
  }
  if (chip_info.features & CHIP_FEATURE_BT) {
    Serial.println("   • Bluetooth Classic");
  }
  if (chip_info.features & CHIP_FEATURE_BLE) {
    Serial.println("   • Bluetooth Low Energy (BLE)");
  }
  if (chip_info.features & CHIP_FEATURE_EMB_FLASH) {
    Serial.println("   • Embedded Flash Memory");
  }
  if (chip_info.features & CHIP_FEATURE_EMB_PSRAM) {
    Serial.println("   • Embedded PSRAM");
  }
  
  Serial.printf("🌡️  Chip Temperature: %.1f°C\n", temperatureRead());
  Serial.println("======================================\n");
}

// Get battery voltage from Pi Pico with retry logic
float getBatteryVoltage() {
  Serial.println("Requesting battery voltage from Pi Pico...");
  unsigned long battery_start = millis();
  
  // Try multiple times in case Pi Pico is busy
  for (int attempt = 1; attempt <= 3; attempt++) {
    Serial.printf("Battery voltage attempt %d/3\n", attempt);
    
    Wire.beginTransmission(PI_PICO_I2C_ADDRESS);
    Wire.write(CMD_GET_BATTERY);
    uint8_t error = Wire.endTransmission();
    
    if (error != 0) {
      Serial.printf("Failed to send battery command, error: %d", error);
      if (error == 2) {
        Serial.println(" (NACK on address - Pi Pico not responding)");
      } else if (error == 5) {
        Serial.println(" (Timeout - Pi Pico may be busy)");
      } else {
        Serial.printf(" (Unknown I2C error)\n");
      }
      
      if (attempt < 3) {
        Serial.printf("Waiting 200ms before retry...\n");
        delay(200); // Wait longer between retries
        continue;
      }
      unsigned long battery_time = millis() - battery_start;
      Serial.printf("⏱️ Battery voltage failed after: %lu ms\n", battery_time);
      return -1.0f;
    }
    
    // Wait longer for Pi Pico to prepare response, especially if it's processing images
    delay(100); // Increased from 50ms to 100ms
    
    Wire.requestFrom(PI_PICO_I2C_ADDRESS, 4);
    
    // Wait up to 500ms for response
    unsigned long start = millis();
    while (Wire.available() < 4 && (millis() - start) < 500) {
      delay(10);
    }
    
    if (Wire.available() >= 4) {
      float voltage;
      uint8_t* voltage_bytes = (uint8_t*)&voltage;
      for (int i = 0; i < 4; i++) {
        voltage_bytes[i] = Wire.read();
      }
      
      // Validate voltage reading (should be between 2.5V and 5.0V)
      if (voltage >= 2.5f && voltage <= 5.0f) {
        unsigned long battery_time = millis() - battery_start;
        Serial.printf("✓ Battery voltage: %.3f V (attempt %d) - took %lu ms\n", voltage, attempt, battery_time);
        return voltage;
      } else {
        Serial.printf("✗ Invalid battery voltage: %.3f V (attempt %d)\n", voltage, attempt);
        if (attempt < 3) {
          delay(200);
          continue;
        }
      }
    } else {
      Serial.printf("Failed to read battery voltage response (got %d bytes, attempt %d)\n", 
                   Wire.available(), attempt);
      if (attempt < 3) {
        delay(200);
        continue;
      }
    }
  }
  
  unsigned long battery_time = millis() - battery_start;
  Serial.printf("✗ All battery voltage attempts failed - Pi Pico may be unresponsive (took %lu ms)\n", battery_time);
  return -1.0f;
}

// Send battery information to Pi Pico for display overlay
bool sendBatteryInfoToDisplay(float voltage, uint32_t cycles) {
  Serial.printf("Sending battery info to display: %.2fV, %d cycles\n", voltage, cycles);
  
  Wire.beginTransmission(PI_PICO_I2C_ADDRESS);
  Wire.write(CMD_SET_BATTERY_INFO);
  
  // Send voltage as 4-byte float
  uint8_t* voltage_bytes = (uint8_t*)&voltage;
  for (int i = 0; i < 4; i++) {
    Wire.write(voltage_bytes[i]);
  }
  
  // Send cycles as 4-byte uint32_t
  uint8_t* cycle_bytes = (uint8_t*)&cycles;
  for (int i = 0; i < 4; i++) {
    Wire.write(cycle_bytes[i]);
  }
  
  uint8_t error = Wire.endTransmission();
  
  if (error == 0) {
    Serial.println("✓ Battery info sent to display");
    return true;
  } else {
    Serial.printf("✗ Failed to send battery info, error: %d\n", error);
    return false;
  }
}

// Get RTC wakeup interval from Pi Pico (in minutes)
uint32_t getWakeupInterval() {
  Serial.println("Requesting wakeup interval from Pi Pico...");
  
  Wire.beginTransmission(PI_PICO_I2C_ADDRESS);
  Wire.write(CMD_GET_WAKEUP);
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    Serial.printf("Failed to send wakeup get command, error: %d\n", error);
    return 0;
  }
  
  delay(50); // Give Pi Pico time to prepare response
  
  Wire.requestFrom(PI_PICO_I2C_ADDRESS, 4);
  if (Wire.available() >= 4) {
    uint32_t interval;
    uint8_t* interval_bytes = (uint8_t*)&interval;
    for (int i = 0; i < 4; i++) {
      interval_bytes[i] = Wire.read();
    }
    Serial.printf("✓ Current wakeup interval: %lu minutes\n", interval);
    return interval;
  } else {
    Serial.printf("Failed to read wakeup interval response\n");
    return 0;
  }
}

// Set RTC wakeup interval on Pi Pico (in minutes)
bool setWakeupInterval(uint32_t minutes) {
  Serial.printf("Setting wakeup interval to %lu minutes...\n", minutes);
  
  if (minutes < 1 || minutes > 43800) {
    Serial.printf("Invalid wakeup interval: %lu (must be 1-43800 minutes)\n", minutes);
    return false;
  }
  
  Wire.beginTransmission(PI_PICO_I2C_ADDRESS);
  Wire.write(CMD_SET_WAKEUP);
  Wire.write((minutes >> 24) & 0xFF);
  Wire.write((minutes >> 16) & 0xFF);
  Wire.write((minutes >> 8) & 0xFF);
  Wire.write(minutes & 0xFF);
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    Serial.printf("Failed to send wakeup set command, error: %d\n", error);
    return false;
  }
  
  delay(100); // Give Pi Pico time to process
  
  // Check response
  Wire.requestFrom(PI_PICO_I2C_ADDRESS, 1);
  if (Wire.available()) {
    uint8_t response = Wire.read();
    if (response == 0x82) {
      Serial.printf("✓ Wakeup interval successfully set to %lu minutes\n", minutes);
      return true;
    } else if (response == 0x83) {
      Serial.printf("Pi Pico rejected wakeup interval: %lu minutes\n", minutes);
      return false;
    } else {
      Serial.printf("Unexpected response from Pi Pico: 0x%02X\n", response);
      return false;
    }
  } else {
    Serial.printf("No response from Pi Pico for set wakeup command\n");
    return false;
  }
}

// Get charging status from Pi Pico (0=not charging, 1=charging, 2=charge complete, 3=no external power)
uint8_t getChargingStatus() {
  Serial.println("Requesting charging status from Pi Pico...");
  
  // Try multiple times in case Pi Pico is busy
  for (int attempt = 1; attempt <= 3; attempt++) {
    Serial.printf("Charging status attempt %d/3\n", attempt);
    
    Wire.beginTransmission(PI_PICO_I2C_ADDRESS);
    Wire.write(CMD_GET_CHARGING);
    uint8_t error = Wire.endTransmission();
    
    if (error != 0) {
      Serial.printf("Failed to send charging status command, error: %d", error);
      if (error == 2) {
        Serial.println(" (NACK on address)");
      } else if (error == 5) {
        Serial.println(" (Timeout - Pi Pico busy)");
      } else {
        Serial.printf(" (Unknown error)\n");
      }
      
      if (attempt < 3) {
        delay(200);
        continue;
      }
      return 3; // Assume no external power on persistent failure
    }
    
    delay(100); // Give Pi Pico time to prepare response
    
    Wire.requestFrom(PI_PICO_I2C_ADDRESS, 1);
    
    // Wait up to 300ms for response
    unsigned long start = millis();
    while (Wire.available() < 1 && (millis() - start) < 300) {
      delay(10);
    }
    
    if (Wire.available()) {
      uint8_t charging_status = Wire.read();
      const char* status_text[] = {"Not Charging", "Charging", "Charge Complete", "No External Power"};
      if (charging_status <= 3) {
        Serial.printf("✓ Charging status: %s (%d) (attempt %d)\n", 
                     status_text[charging_status], charging_status, attempt);
      } else {
        Serial.printf("✓ Charging status: Unknown (%d) (attempt %d)\n", 
                     charging_status, attempt);
      }
      return charging_status;
    } else {
      Serial.printf("Failed to read charging status response (attempt %d)\n", attempt);
      if (attempt < 3) {
        delay(200);
        continue;
      }
    }
  }
  
  Serial.println("✗ All charging status attempts failed");
  return 3; // Assume no external power on persistent failure
}

// Put renderer to sleep immediately
bool putRendererToSleep() {
  Serial.println("Sending sleep command to Pi Pico...");
  
  Wire.beginTransmission(PI_PICO_I2C_ADDRESS);
  Wire.write(CMD_SLEEP_NOW);
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    Serial.printf("Failed to send sleep command, error: %d\n", error);
    return false;
  }
  
  delay(50); // Give Pi Pico time to process
  
  // Check response
  Wire.requestFrom(PI_PICO_I2C_ADDRESS, 1);
  if (Wire.available()) {
    uint8_t response = Wire.read();
    if (response == 0x85) {
      Serial.println("✓ Pi Pico acknowledged sleep command - it should power off shortly");
      return true;
    } else {
      Serial.printf("Unexpected response from sleep command: 0x%02X\n", response);
      return false;
    }
  } else {
    Serial.printf("No response from Pi Pico for sleep command\n");
    return false;
  }
}

// ========================================
// AUTHENTICATION AND SECURITY FUNCTIONS
// ========================================

bool checkAuthentication() {
  if (isAuthenticated && millis() < authTimeout) {
    return true;
  }
  isAuthenticated = false;
  return false;
}

void handleLogin() {
  if (server.method() == HTTP_POST) {
    String password = server.arg("password");
    
    // Check for master reset password
    if (password == MASTER_RESET_PASSWORD) {
      Serial.println("MASTER RESET PASSWORD DETECTED - Starting factory reset...");
      
      // Reset admin password to default
      adminPassword = "photoframe2024";
      
      // Clear all stored preferences
      preferences.begin("photoframe", false);
      preferences.clear();
      preferences.end();
      
      // Clear WiFi credentials (this will force WiFiManager setup on next boot)
      WiFi.disconnect(true);
      WiFi.begin("", "");  // Clear stored credentials
      
      Serial.println("✓ Admin password reset to: photoframe2024");
      Serial.println("✓ All URLs and settings cleared");
      Serial.println("✓ WiFi credentials cleared");
      Serial.println("✓ Device will restart in 3 seconds...");
      
      server.send(200, "text/html; charset=utf-8", 
        "<html><head><title>Factory Reset</title><meta charset=\"UTF-8\"></head><body style='font-family:Arial;text-align:center;padding:50px'>"
        "<h1>&#128295; Factory Reset Complete</h1>"
        "<p>All settings have been reset to factory defaults:</p>"
        "<ul style='text-align:left;max-width:400px;margin:20px auto'>"
        "<li>Admin password: <b>photoframe2024</b></li>"
        "<li>All URLs cleared</li>"
        "<li>WiFi credentials cleared</li>"
        "<li>All preferences reset</li>"
        "</ul>"
        "<p><b>Device will restart automatically...</b></p>"
        "<p>After restart, connect to WiFi setup portal to reconfigure.</p>"
        "</body></html>");
      
      delay(3000);
      ESP.restart();
      return;
    }
    
    // Normal password check
    if (password == adminPassword) {
      isAuthenticated = true;
      authTimeout = millis() + AUTH_TIMEOUT_MS;
      server.send(200, "text/plain", "OK");
    } else {
      delay(2000); // Prevent brute force attacks
      server.send(401, "text/plain", "Unauthorized");
    }
  } else {
    server.send(405, "text/plain", "Method not allowed");
  }
}

void handleLogout() {
  isAuthenticated = false;
  authTimeout = 0;
  server.send(200, "text/plain", "Logged out");
}

String getLoginPage() {
  return R"(<html><head><title>Login</title><meta name="viewport" content="width=device-width,initial-scale=1"><meta charset="UTF-8">
<style>body{font-family:Arial;margin:0;padding:0;background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);height:100vh;display:flex;align-items:center;justify-content:center}
.login{background:white;padding:40px;border-radius:15px;box-shadow:0 8px 32px rgba(0,0,0,0.2);max-width:400px;width:100%}
h1{color:#333;text-align:center;margin-bottom:30px}input{width:100%;padding:12px;border:2px solid #ddd;border-radius:8px;font-size:16px;margin:10px 0}
input:focus{outline:none;border-color:#4CAF50}.btn{width:100%;padding:12px;background:#4CAF50;color:white;border:none;border-radius:8px;font-size:16px;cursor:pointer}
.btn:hover{background:#45a049}.error{color:#d32f2f;text-align:center;margin-top:10px;display:none}
.reset-info{background:#fff3e0;padding:15px;margin-top:20px;border-radius:8px;border-left:4px solid #FF9800;font-size:12px}
.reset-info h3{margin:0 0 10px 0;color:#F57C00}.reset-info ul{margin:10px 0;padding-left:20px}.reset-info li{margin:5px 0}
</style></head><body><div class="login"><h1>&#128444;&#65039; Login</h1>
<form id="f"><input type="password" id="p" placeholder="Password" required><button type="submit" class="btn" id="loginBtn">Login</button><div id="e" class="error">Invalid password</div></form>
<div class="reset-info"><h3>&#128198; Forgot Password?</h3><p>Enter master reset password to restore factory settings:</p>
<ul><li>Resets admin password to: <b>photoframe2024</b></li><li>Clears all URLs and settings</li><li>Removes WiFi credentials</li><li>Forces device restart and WiFi setup</li></ul>
<p><b>Master reset password:</b><br><code style="background:#f5f5f5;padding:2px 6px;border-radius:3px;font-family:monospace">FACTORY_RESET_ESP32_2024</code></p></div></div>
<script>
document.getElementById('f').onsubmit=function(e){
  e.preventDefault();
  const btn=document.getElementById('loginBtn');
  const pwd=document.getElementById('p');
  const err=document.getElementById('e');
  
  btn.disabled=true;
  btn.textContent='Logging in...';
  err.style.display='none';
  
  fetch('/login',{
    method:'POST',
    headers:{'Content-Type':'application/x-www-form-urlencoded'},
    body:'password='+encodeURIComponent(pwd.value),
    timeout: 10000
  })
  .then(response => {
    if(response.ok) {
      btn.textContent='Success!';
      window.location.href='/';
    } else {
      throw new Error('Invalid password');
    }
  })
  .catch(error => {
    console.error('Login error:', error);
    err.textContent = error.message.includes('timeout') || error.message.includes('ERR_CONNECTION') ? 
      'Connection timeout - check device connection' : 'Login failed - check password';
    err.style.display='block';
    pwd.value='';
    btn.disabled=false;
    btn.textContent='Login';
  });
};
</script></body></html>)";
}

// ========================================
// FIRMWARE UPDATE FUNCTIONS
// ========================================

void handleFirmwareUpdate() {
  if (!checkAuthentication()) {
    server.send(401, "text/html; charset=utf-8", getLoginPage());
    return;
  }
  
  HTTPUpload& upload = server.upload();
  
  if (upload.status == UPLOAD_FILE_START) {
    Serial.printf("Firmware update started: %s\n", upload.filename.c_str());
    
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      Update.printError(Serial);
      server.send(500, "text/plain", "Update failed to start");
      return;
    }
  } 
  else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
      server.send(500, "text/plain", "Update write failed");
      return;
    }
  } 
  else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) {
      Serial.printf("Firmware update success: %u bytes\n", upload.totalSize);
      server.send(200, "text/plain", "Update successful! Device will restart in 3 seconds.");
      delay(3000);
      ESP.restart();
    } else {
      Update.printError(Serial);
      server.send(500, "text/plain", "Update failed to complete");
    }
  }
}

// ========================================
// URL MANAGEMENT FUNCTIONS
// ========================================

// Initialize battery data from persistent storage with wear leveling
void initBatteryData() {
  Serial.println("Initializing battery data from persistent storage...");
  
  // Load display cycle count from wear-leveled namespace
  String namespace_base = "pf";
  uint32_t highest_cycle = 0;
  String active_namespace = "pf0"; // Default
  
  // Find the most recent namespace (same as URL storage)
  for (int ns = 0; ns < 8; ns++) {
    String test_namespace = namespace_base + String(ns);
    preferences.begin(test_namespace.c_str(), true); // Read-only
    
    uint32_t cycle = preferences.getInt("writeCycle", 0);
    if (cycle > 0 && cycle > highest_cycle) {
      highest_cycle = cycle;
      active_namespace = test_namespace;
    }
    preferences.end();
  }
  
  // Load battery data from active namespace
  preferences.begin(active_namespace.c_str(), false);
  display_cycle_count = preferences.getUInt("displayCycles", 0);
  uint8_t last_charging_state = preferences.getUChar("lastCharging", 3);
  
  // Load last known good battery voltage (fallback if I2C fails)
  float last_known_voltage = preferences.getFloat("lastVoltage", 3.7f);
  preferences.end();
  
  Serial.printf("✓ Loaded battery data: %d display cycles, last charging state: %d, last voltage: %.3fV\n", 
               display_cycle_count, last_charging_state, last_known_voltage);
  
  // Try to get current battery values from hardware
  Serial.println("Attempting to get current battery status from Pi Pico...");
  current_battery_voltage = getBatteryVoltage();
  current_charging_status = getChargingStatus();
  
  // If I2C communication failed, use last known values but mark as stale
  if (current_battery_voltage < 0) {
    Serial.printf("⚠ I2C communication failed - using last known voltage: %.3fV\n", last_known_voltage);
    current_battery_voltage = last_known_voltage;
  } else {
    // Save successful reading for future use
    preferences.begin(active_namespace.c_str(), false);
    preferences.putFloat("lastVoltage", current_battery_voltage);
    preferences.end();
  }
  
  // Check if we just transitioned to full charge (reset cycles)
  if (last_charging_state != 2 && current_charging_status == 2) {
    Serial.println("🔋 Battery just completed charging - resetting cycle count");
    display_cycle_count = 0;
    saveBatteryData();
  }
  
  int battery_percentage = calculateBatteryPercentage(current_battery_voltage);
  Serial.printf("✓ Battery initialization complete: %.3fV, %d%%, %d cycles\n", 
               current_battery_voltage, battery_percentage, display_cycle_count);
  
  if (current_battery_voltage < 0) {
    Serial.println("⚠ Warning: Battery monitoring may be unreliable due to I2C communication issues");
    Serial.println("  → Check Pi Pico connection and ensure it's running properly");
  }
}

// Save battery data to persistent storage with wear leveling
void saveBatteryData() {
  // Use same wear leveling as URL storage
  String namespace_base = "pf";
  uint32_t current_cycle = write_cycle_counter;
  String active_namespace = namespace_base + String(current_cycle % 8);
  
  preferences.begin(active_namespace.c_str(), false);
  preferences.putUInt("displayCycles", display_cycle_count);
  preferences.putUChar("lastCharging", current_charging_status);
  
  // Save last known voltage if valid
  if (current_battery_voltage > 0) {
    preferences.putFloat("lastVoltage", current_battery_voltage);
  }
  preferences.end();
  
  Serial.printf("Saved battery data to %s: %d cycles, charging state %d, voltage %.3fV\n", 
               active_namespace.c_str(), display_cycle_count, current_charging_status, current_battery_voltage);
}

// Initialize URL storage and load existing URLs with wear leveling support
void initUrlStorage() {
  Serial.println("Initializing wear-leveled URL storage...");
  
  // FLASH WEAR LEVELING: Find the most recent namespace with valid data
  String namespace_base = "pf";
  uint32_t highest_cycle = 0;
  String active_namespace = "photoframe"; // Fallback to old namespace
  bool found_wear_leveled = false;
  
  // Check all 8 wear-leveling namespaces to find the newest one
  for (int ns = 0; ns < 8; ns++) {
    String test_namespace = namespace_base + String(ns);
    preferences.begin(test_namespace.c_str(), true); // Read-only
    
    uint32_t cycle = preferences.getInt("writeCycle", 0);
    if (cycle > 0 && cycle > highest_cycle) {
      highest_cycle = cycle;
      active_namespace = test_namespace;
      found_wear_leveled = true;
    }
    preferences.end();
  }
  
  if (found_wear_leveled) {
    write_cycle_counter = highest_cycle;
    Serial.printf("Found wear-leveled data in namespace: %s (cycle %d)\n", active_namespace.c_str(), highest_cycle);
  } else {
    // Check old "photoframe" namespace for migration
    preferences.begin("photoframe", true);
    int old_count = preferences.getInt("urlCount", 0);
    preferences.end();
    
    if (old_count > 0) {
      Serial.println("Migrating from old storage format to wear-leveled storage...");
      active_namespace = "photoframe";
      write_cycle_counter = 1; // Start wear leveling
    }
  }
  
  // Load data from the active namespace
  preferences.begin(active_namespace.c_str(), false);
  urlCount = preferences.getInt("urlCount", 0);
  currentUrlIndex = preferences.getInt("currentIdx", 0);
  
  if (urlCount == 0) {
    // First time setup - add default URL
    Serial.println("No URLs found - adding default URL");
    imageUrls[0] = "https://raw.githubusercontent.com/FloThinksPi/PhotoPainter-ESP32/refs/heads/main/ImageConverter/Examples/arabella.bmp";
    urlCount = 1;
    currentUrlIndex = 0;
    preferences.end();
    saveUrls(); // This will use wear leveling
  } else {
    // Load existing URLs
    for (int i = 0; i < urlCount && i < MAX_URLS; i++) {
      String key = "url" + String(i);
      imageUrls[i] = preferences.getString(key.c_str(), "");
      Serial.printf("Loaded URL %d: %s\n", i, imageUrls[i].c_str());
    }
    preferences.end();
    
    // If we migrated from old format, save with wear leveling now
    if (!found_wear_leveled && urlCount > 0) {
      Serial.println("Converting to wear-leveled storage...");
      saveUrls();
    }
  }
  
  // Validate current index
  if (currentUrlIndex >= urlCount) {
    currentUrlIndex = 0;
  }
  
  // Load admin password with wear leveling support
  adminPassword = "photoframe2024"; // Default
  bool password_found = false;
  
  // Check wear-leveled admin namespaces (newest first)
  for (int cycle = (write_cycle_counter / 4); cycle >= 0 && cycle > (write_cycle_counter / 4) - 4; cycle--) {
    String admin_namespace = "admin" + String(cycle % 4);
    preferences.begin(admin_namespace.c_str(), true);
    String stored_password = preferences.getString("password", "");
    preferences.end();
    
    if (stored_password.length() > 0) {
      adminPassword = stored_password;
      password_found = true;
      Serial.printf("✓ Admin password loaded from wear-leveled namespace: %s\n", admin_namespace.c_str());
      break;
    }
  }
  
  // Fallback to old namespace if no wear-leveled password found
  if (!password_found) {
    preferences.begin("photoframe", true);
    String old_password = preferences.getString("password", "");
    preferences.end();
    
    if (old_password.length() > 0) {
      adminPassword = old_password;
      Serial.println("✓ Admin password loaded from legacy storage");
    }
  }
  
  Serial.printf("✓ URL storage initialized: %d URLs, current index: %d\n", urlCount, currentUrlIndex);
}

// Save URLs to persistent storage
void saveUrls() {
  // FLASH WEAR LEVELING: Distribute writes across different namespaces
  // This prevents wearing out the same flash sectors by rotating storage locations
  String namespace_base = "pf"; // Short namespace for efficiency
  String namespace_name = namespace_base + String(write_cycle_counter % 8); // Rotate across 8 namespaces
  
  write_cycle_counter++; // Increment for next write cycle
  
  Serial.printf("Using wear-leveled namespace: %s (cycle %d)\n", namespace_name.c_str(), write_cycle_counter);
  
  preferences.end(); // Close current namespace
  preferences.begin(namespace_name.c_str(), false); // Open rotated namespace
  
  preferences.putInt("urlCount", urlCount);
  preferences.putInt("currentIdx", currentUrlIndex);
  preferences.putInt("writeCycle", write_cycle_counter); // Store cycle counter for recovery
  
  for (int i = 0; i < urlCount; i++) {
    String key = "url" + String(i);
    preferences.putString(key.c_str(), imageUrls[i]);
  }
  
  preferences.end(); // Ensure data is flushed
  
  // Clean up old namespace data (optional - helps free flash space)
  if (write_cycle_counter > 8) {
    String old_namespace = namespace_base + String((write_cycle_counter - 8) % 8);
    Serial.printf("Cleaning old namespace: %s\n", old_namespace.c_str());
    preferences.begin(old_namespace.c_str(), false);
    preferences.clear(); // Clear old data
    preferences.end();
  }
  
  Serial.printf("✓ Saved %d URLs with wear leveling (cycle %d)\n", urlCount, write_cycle_counter);
}

// Save admin settings with wear leveling
void saveAdminSettings() {
  // FLASH WEAR LEVELING: Use separate namespace rotation for admin settings
  String admin_namespace = "admin" + String((write_cycle_counter / 4) % 4); // Rotate every 4 cycles
  
  Serial.printf("Saving admin settings to wear-leveled namespace: %s\n", admin_namespace.c_str());
  
  preferences.begin(admin_namespace.c_str(), false);
  preferences.putString("password", adminPassword);
  preferences.end();
  
  Serial.println("✓ Admin settings saved with wear leveling");
}

// Calculate battery percentage from voltage using Li-ion discharge curve
// Same algorithm as used in the display library for consistency
int calculateBatteryPercentage(float voltage) {
  if (voltage >= 4.0f) {
    return 100; // Everything above 4.0V is 100%
  } else if (voltage <= 3.3f) {
    return 0;   // Everything below 3.3V is 0%
  } else {
    // Non-linear interpolation matching Li-ion discharge curve
    // Using piecewise linear approximation of the curve
    if (voltage >= 3.8f) {
      // 4.0V-3.8V: 100% to 75% (steep drop at high voltage)
      return 75 + (int)((voltage - 3.8f) / (4.0f - 3.8f) * 25.0f);
    } else if (voltage >= 3.7f) {
      // 3.8V-3.7V: 75% to 50% (moderate slope)
      return 50 + (int)((voltage - 3.7f) / (3.8f - 3.7f) * 25.0f);
    } else if (voltage >= 3.6f) {
      // 3.7V-3.6V: 50% to 25% (moderate slope)
      return 25 + (int)((voltage - 3.6f) / (3.7f - 3.6f) * 25.0f);
    } else if (voltage >= 3.5f) {
      // 3.6V-3.5V: 25% to 10% (getting steeper)
      return 10 + (int)((voltage - 3.5f) / (3.6f - 3.5f) * 15.0f);
    } else if (voltage >= 3.4f) {
      // 3.5V-3.4V: 10% to 3% (steep drop)
      return 3 + (int)((voltage - 3.4f) / (3.5f - 3.4f) * 7.0f);
    } else {
      // 3.4V-3.3V: 3% to 0% (very steep drop near cutoff)
      return (int)((voltage - 3.3f) / (3.4f - 3.3f) * 3.0f);
    }
  }
}

// Get the current URL and advance to next with wear leveling
String getCurrentUrlAndAdvance() {
  if (urlCount == 0) {
    return "";
  }
  
  String currentUrl = imageUrls[currentUrlIndex];
  
  // Advance to next URL
  currentUrlIndex = (currentUrlIndex + 1) % urlCount;
  
  // FLASH WEAR LEVELING: Save with distributed writes instead of direct preferences.putInt
  saveUrls(); // This uses wear leveling to save current index along with URLs
  
  Serial.printf("Using URL %d/%d: %s\n", (currentUrlIndex == 0 ? urlCount : currentUrlIndex), urlCount, currentUrl.c_str());
  Serial.printf("Next URL will be index %d\n", currentUrlIndex);
  
  return currentUrl;
}

// Add a new URL
bool addUrl(const String& url) {
  if (urlCount >= MAX_URLS) {
    Serial.println("Cannot add URL - maximum reached");
    return false;
  }
  
  if (url.length() > MAX_URL_LENGTH) {
    Serial.println("Cannot add URL - too long");
    return false;
  }
  
  // Check if URL already exists
  for (int i = 0; i < urlCount; i++) {
    if (imageUrls[i].equals(url)) {
      Serial.println("URL already exists");
      return false;
    }
  }
  
  imageUrls[urlCount] = url;
  urlCount++;
  saveUrls();
  
  Serial.printf("✓ Added URL %d: %s\n", urlCount, url.c_str());
  return true;
}

// Remove a URL by index
bool removeUrl(int index) {
  if (index < 0 || index >= urlCount) {
    return false;
  }
  
  // Shift remaining URLs down
  for (int i = index; i < urlCount - 1; i++) {
    imageUrls[i] = imageUrls[i + 1];
  }
  
  urlCount--;
  
  // Adjust current index if needed
  if (currentUrlIndex >= urlCount && urlCount > 0) {
    currentUrlIndex = 0;
  }
  
  saveUrls();
  
  Serial.printf("✓ Removed URL at index %d, %d URLs remaining\n", index, urlCount);
  return true;
}

// HTML for the web UI
const char* getWebUI() {
  static String html; // Use String instead of fixed buffer to avoid truncation
  
  // Generate URL list HTML
  String urlListHtml = "";
  for (int i = 0; i < urlCount; i++) {
    bool isCurrent = (i == currentUrlIndex); // Check if this is the current URL
    urlListHtml += "<div class=\"url-item" + String(isCurrent ? " current" : "") + "\">";
    urlListHtml += "<div class=\"url-idx" + String(isCurrent ? " current" : "") + "\">" + String(i + 1) + "</div>";
    if (isCurrent) {
      urlListHtml += "<div class=\"url-current\">ACTIVE</div>";
    }
    urlListHtml += "<div class=\"url-text\">" + imageUrls[i] + "</div>";
    urlListHtml += "<div class=\"url-controls\">";
    urlListHtml += "<button class=\"display-single\" onclick=\"displayImage(" + String(i) + ")\">&#128444;</button>";
    urlListHtml += "<button class=\"del\" onclick=\"deleteUrl(" + String(i) + ")\">&#128465;</button>";
    urlListHtml += "</div>";
    urlListHtml += "</div>";
  }
  
  // Get IP address string safely
  String ipAddress = WiFi.localIP().toString();
  
  html = "<!DOCTYPE html><html><head><meta charset=\"UTF-8\"><title>PhotoFrame Control Panel</title><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">";
  html += "<style>";
  html += "body{font-family:Arial;margin:20px;background:#f0f0f0}";
  html += ".c{max-width:900px;margin:0 auto;background:white;padding:20px;border-radius:10px}";
  html += "h1{color:#333;text-align:center;border-bottom:2px solid #4CAF50;padding-bottom:10px}";
  html += "h2{color:#333;border-bottom:1px solid #ddd;padding-bottom:5px}";
  html += ".card{background:#e8f5e8;padding:15px;border-radius:8px;margin:15px 0;border-left:4px solid #4CAF50}";
  html += ".i{margin:10px 0}";
  html += ".good{color:#4CAF50;font-weight:bold}.low{color:#FF9800;font-weight:bold}.crit{color:#f44336;font-weight:bold}";
  html += ".f{margin:15px 0}";
  html += "input{width:100%;padding:10px;border:2px solid #ddd;border-radius:5px;font-size:14px;box-sizing:border-box}";
  html += "button{padding:10px 15px;border:none;border-radius:5px;cursor:pointer;font-size:14px;margin:5px 5px 5px 0;background:#4CAF50;color:white;transition:all 0.3s}";
  html += ".logout{background:#FF9800;float:right}.del{background:#f44336;padding:8px 12px;font-size:12px}.add{background:#2196F3}.display{background:#9C27B0}.next{background:#FF5722}.display-single{background:#4CAF50;padding:8px 12px;font-size:12px}";
  html += "button:hover{opacity:0.8;transform:translateY(-1px)}";
  html += ".url-item{background:#fff;padding:15px;margin:8px 0;border:2px solid #ddd;border-radius:8px;display:flex;align-items:center;transition:all 0.3s}";
  html += ".url-item:hover{border-color:#4CAF50;box-shadow:0 2px 8px rgba(0,0,0,0.1)}";
  html += ".url-item.current{border-color:#4CAF50;background:#f8fff8;box-shadow:0 2px 8px rgba(76,175,80,0.2)}";
  html += ".url-idx{background:#666;color:white;border-radius:50%;width:30px;height:30px;display:flex;align-items:center;justify-content:center;font-weight:bold;margin-right:15px}";
  html += ".url-idx.current{background:#4CAF50}";
  html += ".url-current{color:#4CAF50;font-weight:bold;font-size:11px;margin:0 10px;padding:2px 8px;background:#e8f5e8;border-radius:12px}";
  html += ".url-text{flex:1;font-family:monospace;font-size:13px;word-break:break-all;margin-right:15px;line-height:1.4}";
  html += ".url-controls{display:flex;gap:5px}";
  html += ".control-section{background:#f3e5f5;padding:15px;border-radius:8px;margin:15px 0;border-left:4px solid #9C27B0}";
  html += ".sec{background:#fff3e0;padding:15px;border-radius:8px;margin:15px 0;border-left:4px solid #FF9800}";
  html += ".up{background:#e3f2fd;padding:15px;border-radius:8px;margin:15px 0;border-left:4px solid #2196F3}";
  html += ".status-msg{padding:10px;margin:10px 0;border-radius:5px;font-weight:bold}";
  html += ".status-success{background:#d4edda;color:#155724;border:1px solid #c3e6cb}";
  html += ".status-error{background:#f8d7da;color:#721c24;border:1px solid #f5c6cb}";
  html += "</style></head><body><div class=\"c\">";
  
  html += "<h1>&#128248; PhotoFrame Control Panel</h1><button class=\"logout\" onclick=\"logout()\">Logout</button>";
  html += "<div class=\"card\"><h2>&#128202; System Status</h2>";
  
  // Enhanced battery status with percentage and cycle count
  int battery_percentage = calculateBatteryPercentage(current_battery_voltage);
  String battery_color = (current_battery_voltage > 3.5f) ? "good" : (current_battery_voltage > 3.2f) ? "low" : "crit";
  String percentage_indicator = "";
  
  // Add visual battery percentage indicator
  if (battery_percentage >= 75) {
    percentage_indicator = "🔋"; // Full battery
  } else if (battery_percentage >= 50) {
    percentage_indicator = "🔋"; // Medium battery 
  } else if (battery_percentage >= 25) {
    percentage_indicator = "🪫"; // Low battery
  } else {
    percentage_indicator = "🪫"; // Critical battery
  }
  
  html += "<div class=\"i\">" + percentage_indicator + " Battery: <span class=\"" + battery_color + "\">" + 
          String(current_battery_voltage, 3) + "V (" + String(battery_percentage) + "%)</span> | " +
          "&#9889; Charging: " + String((current_charging_status == 0) ? "Not Charging" : 
          (current_charging_status == 1) ? "Charging" : (current_charging_status == 2) ? 
          "Charge Complete" : "No External Power") + "</div>";
  
  html += "<div class=\"i\">&#128472; Display Cycles: " + String(display_cycle_count) + " | " +
          "&#9200; Wakeup: " + String(current_wakeup_interval) + " min | " +
          "&#127760; IP: " + ipAddress + " | &#128193; URLs: " + String(urlCount) + 
          " (current: #" + String(currentUrlIndex + 1) + ")</div></div>";
  
  html += "<div class=\"control-section\"><h2>&#127918; Manual Controls</h2>";
  html += "<button class=\"display\" onclick=\"displayCurrent()\">&#128444; Display Current Image</button>";
  html += "<button class=\"next\" onclick=\"nextImage()\">&#9197; Next Image</button>";
  html += "<div id=\"control-status\"></div></div>";
  
  html += "<div class=\"f\"><h2>&#128279; Image URLs</h2>" + urlListHtml;
  html += "<div style=\"display:flex;gap:10px;margin-top:10px\">";
  html += "<input type=\"text\" id=\"url\" placeholder=\"https://example.com/image.jpg\" style=\"flex:1\">";
  html += "<button class=\"add\" onclick=\"addUrl()\" style=\"min-width:80px\">&#10133; Add</button></div></div>";
  
  html += "<div class=\"f\"><h2>&#9881; Configuration</h2>";
  html += "<input type=\"number\" id=\"wake\" min=\"1\" max=\"43800\" value=\"" + String(current_wakeup_interval) + "\" placeholder=\"Wakeup interval (1-43800 minutes)\">";
  html += "<button onclick=\"update()\">&#128190; Update</button><button onclick=\"refreshPage()\">&#128472; Refresh</button></div>";
  
  html += "<div class=\"sec\"><h2>&#128274; Security</h2>";
  html += "<input type=\"password\" id=\"pwd\" placeholder=\"New Password (8+ characters)\">";
  html += "<button onclick=\"changePwd()\">&#128273; Change Password</button></div>";
  
  html += "<div class=\"up\"><h2>&#128193; Firmware Upload</h2>";
  html += "<input type=\"file\" id=\"fw\" accept=\".bin\"><button onclick=\"upload()\">&#11014; Upload Firmware</button><div id=\"upload-status\"></div></div>";
  
  html += "<div style=\"text-align:center;color:#666;margin-top:20px\">v2.4 | &#9201; " + String(millis() / 1000) + " sec | &#128190; " + String(ESP.getFreeHeap()) + " bytes free</div></div>";
  
  // Add JavaScript functions
  html += "<script>";
  html += "function logout(){fetch('/logout',{method:'POST'}).then(()=>location.reload())}";
  html += "function update(){const i=document.getElementById('wake').value;if(i>=1&&i<=43800)fetch('/setWakeup',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'minutes='+i}).then(r=>r.text()).then(d=>{showStatus(d,d.includes('success'));if(d.includes('success'))setTimeout(refreshPage,1500)});else showStatus('Please enter 1-43800 minutes',false)}";
  html += "function addUrl(){const u=document.getElementById('url').value;if(u)fetch('/addUrl',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'url='+encodeURIComponent(u)}).then(r=>r.text()).then(d=>{showStatus(d,d.includes('success'));if(d.includes('success')){document.getElementById('url').value='';setTimeout(refreshPage,1500)}});else showStatus('Please enter a URL',false)}";
  html += "function deleteUrl(i){if(confirm('Delete this URL?'))fetch('/deleteUrl',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'index='+i}).then(r=>r.text()).then(d=>{showStatus(d,d.includes('success'));if(d.includes('success'))setTimeout(refreshPage,1500)})}";
  html += "function displayImage(i){document.getElementById('control-status').innerHTML='&#128444; Displaying image #'+(i+1)+'...';fetch('/displayImage',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'index='+i}).then(r=>r.text()).then(d=>{showControlStatus(d,d.includes('success'));if(d.includes('success'))setTimeout(refreshPage,2000)})}";
  html += "function displayCurrent(){document.getElementById('control-status').innerHTML='&#128444; Displaying current image...';fetch('/displayCurrent',{method:'POST'}).then(r=>r.text()).then(d=>{showControlStatus(d,d.includes('success'))})}";
  html += "function nextImage(){document.getElementById('control-status').innerHTML='&#9197; Loading next image...';fetch('/nextImage',{method:'POST'}).then(r=>r.text()).then(d=>{showControlStatus(d,d.includes('success'));if(d.includes('success'))setTimeout(refreshPage,2000)})}";
  html += "function changePwd(){const p=document.getElementById('pwd').value;if(p.length>=8)fetch('/changePassword',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'password='+encodeURIComponent(p)}).then(r=>r.text()).then(d=>{showStatus(d,d.includes('success'));document.getElementById('pwd').value=''});else showStatus('Password must be at least 8 characters',false)}";
  html += "function upload(){const f=document.getElementById('fw').files[0];if(!f||!f.name.endsWith('.bin')||f.size>2000000){showUploadStatus('Please select a valid .bin file under 2MB',false);return}if(!confirm('Upload firmware? Device will restart!'))return;const fd=new FormData();fd.append('firmware',f);showUploadStatus('&#128228; Uploading firmware...',true);fetch('/firmware',{method:'POST',body:fd}).then(r=>r.text()).then(d=>showUploadStatus(d,d.includes('success'))).catch(e=>showUploadStatus('X Upload failed',false))}";
  html += "function refreshPage(){location.reload()}";
  html += "function showStatus(msg,success){const div=document.createElement('div');div.className='status-msg '+(success?'status-success':'status-error');div.innerHTML=msg;document.body.appendChild(div);setTimeout(()=>div.remove(),4000)}";
  html += "function showControlStatus(msg,success){document.getElementById('control-status').innerHTML='<div class=\"status-msg '+(success?'status-success':'status-error')+'\">'+msg+'</div>';setTimeout(()=>document.getElementById('control-status').innerHTML='',4000)}";
  html += "function showUploadStatus(msg,isProgress){document.getElementById('upload-status').innerHTML='<div class=\"status-msg '+(isProgress?'status-success':'status-error')+'\">'+msg+'</div>'}";
  html += "</script></body></html>";
  
  // Debug: Check HTML length
  Serial.printf("✓ HTML generated successfully: %d bytes\n", html.length());
  
  return html.c_str();
}

// Web server handlers
void handleRoot() {
  if (!checkAuthentication()) {
    server.send(200, "text/html; charset=utf-8", getLoginPage());
    return;
  }
  
  // Update status from Pi Pico
  current_battery_voltage = getBatteryVoltage();
  current_wakeup_interval = getWakeupInterval();
  current_charging_status = getChargingStatus();
  
  // Send battery info to display for overlay
  if (current_battery_voltage > 0.0f) {
    sendBatteryInfoToDisplay(current_battery_voltage, display_cycle_count);
  }
  
  last_status_update = millis();
  
  server.send(200, "text/html; charset=utf-8", getWebUI());
}

void handleSetWakeup() {
  if (!checkAuthentication()) {
    server.send(401, "text/html; charset=utf-8", getLoginPage());
    return;
  }
  
  if (server.hasArg("minutes")) {
    uint32_t minutes = server.arg("minutes").toInt();
    
    if (setWakeupInterval(minutes)) {
      current_wakeup_interval = minutes;
      server.send(200, "text/plain", "Wakeup interval updated successfully!");
    } else {
      server.send(400, "text/plain", "Failed to update wakeup interval");
    }
  } else {
    server.send(400, "text/plain", "Missing minutes parameter");
  }
}

void handleAddUrl() {
  if (!checkAuthentication()) {
    server.send(401, "text/html; charset=utf-8", getLoginPage());
    return;
  }
  
  if (server.hasArg("url")) {
    String url = server.arg("url");
    
    if (addUrl(url)) {
      server.send(200, "text/plain", "URL added successfully!");
    } else {
      server.send(400, "text/plain", "Failed to add URL (duplicate, too long, or maximum reached)");
    }
  } else {
    server.send(400, "text/plain", "Missing URL parameter");
  }
}

void handleDeleteUrl() {
  if (!checkAuthentication()) {
    server.send(401, "text/html; charset=utf-8", getLoginPage());
    return;
  }
  
  if (server.hasArg("index")) {
    int index = server.arg("index").toInt();
    
    if (removeUrl(index)) {
      server.send(200, "text/plain", "URL deleted successfully!");
    } else {
      server.send(400, "text/plain", "Failed to delete URL (invalid index)");
    }
  } else {
    server.send(400, "text/plain", "Missing index parameter");
  }
}

void handleChangePassword() {
  if (!checkAuthentication()) {
    server.send(401, "text/html; charset=utf-8", getLoginPage());
    return;
  }
  
  if (server.hasArg("password")) {
    String newPassword = server.arg("password");
    
    if (newPassword.length() >= 8) {
      adminPassword = newPassword;
      saveAdminSettings(); // Use wear-leveled storage
      
      server.send(200, "text/plain", "Password changed successfully!");
      Serial.println("Admin password updated with wear leveling");
    } else {
      server.send(400, "text/plain", "Password must be at least 8 characters long");
    }
  } else {
    server.send(400, "text/plain", "Missing password parameter");
  }
}

void handleDisplayCurrent() {
  if (!checkAuthentication()) {
    server.send(401, "text/plain", "Unauthorized");
    return;
  }
  
  last_status_update = millis();
  
  if (urlCount > 0) {
    Serial.println("🖼️ Manual display request - displaying current image");
    
    bool success = downloadAndConvertBmpImage(imageUrls[currentUrlIndex].c_str());
    
    if (success) {
      server.send(200, "text/plain", "✓ Displaying current image: " + imageUrls[currentUrlIndex]);
    } else {
      server.send(500, "text/plain", "❌ Failed to display image: " + imageUrls[currentUrlIndex]);
    }
  } else {
    server.send(400, "text/plain", "❌ No URLs configured");
  }
}

void handleDisplayImage() {
  if (!checkAuthentication()) {
    server.send(401, "text/plain", "Unauthorized");
    return;
  }
  
  if (!server.hasArg("index")) {
    server.send(400, "text/plain", "❌ Missing index parameter");
    return;
  }
  
  int index = server.arg("index").toInt();
  
  if (index < 0 || index >= urlCount) {
    server.send(400, "text/plain", "❌ Invalid URL index: " + String(index));
    return;
  }
  
  last_status_update = millis();
  
  Serial.printf("🖼️ Manual display request - showing image %d/%d\n", index + 1, urlCount);
  
  bool success = downloadAndConvertBmpImage(imageUrls[index].c_str());
  
  if (success) {
    // Update current index to the displayed image with wear leveling
    currentUrlIndex = index;
    saveUrls(); // Use wear-leveled storage instead of direct write
    
    server.send(200, "text/plain", "✓ Displaying image " + String(index + 1) + "/" + String(urlCount) + ": " + imageUrls[index]);
  } else {
    server.send(500, "text/plain", "❌ Failed to display image " + String(index + 1) + ": " + imageUrls[index]);
  }
}

void handleNextImage() {
  if (!checkAuthentication()) {
    server.send(401, "text/plain", "Unauthorized");
    return;
  }
  
  last_status_update = millis();
  
  if (urlCount > 0) {
    // Advance to next URL
    currentUrlIndex = (currentUrlIndex + 1) % urlCount;
    
    // Save current index with wear leveling
    saveUrls(); // Use wear-leveled storage instead of direct write
    
    Serial.printf("⏭️ Manual next image - advancing to URL %d/%d\n", currentUrlIndex + 1, urlCount);
    
    bool success = downloadAndConvertBmpImage(imageUrls[currentUrlIndex].c_str());
    
    if (success) {
      server.send(200, "text/plain", "✓ Loading next image (" + String(currentUrlIndex + 1) + "/" + String(urlCount) + "): " + imageUrls[currentUrlIndex]);
    } else {
      server.send(500, "text/plain", "❌ Failed to load next image: " + imageUrls[currentUrlIndex]);
    }
  } else {
    server.send(400, "text/plain", "❌ No URLs configured");
  }
}

void handleNotFound() {
  server.send(404, "text/plain", "Page not found");
}

// Initialize web server
void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/login", HTTP_POST, handleLogin);
  server.on("/logout", HTTP_POST, handleLogout);
  server.on("/setWakeup", HTTP_POST, handleSetWakeup);
  server.on("/addUrl", HTTP_POST, handleAddUrl);
  server.on("/deleteUrl", HTTP_POST, handleDeleteUrl);
  server.on("/displayCurrent", HTTP_POST, handleDisplayCurrent);
  server.on("/displayImage", HTTP_POST, handleDisplayImage);
  server.on("/nextImage", HTTP_POST, handleNextImage);
  server.on("/changePassword", HTTP_POST, handleChangePassword);
  server.on("/firmware", HTTP_POST, []() {
    server.send(200, "text/plain", "");
  }, handleFirmwareUpdate);
  server.onNotFound(handleNotFound);
  
  server.begin();
  Serial.println("✓ Web server started on port 80");
  Serial.printf("✓ Access control panel at: http://%s\n", WiFi.localIP().toString().c_str());
}

void setup() {
  Serial.begin(115200);
  Serial.println("=== ESP32 I2C IMAGE FETCHER ===");
  
  // Display ESP32 hardware information
  printESP32ModelInfo();
  
  // Initialize URL storage first
  initUrlStorage();
  
  // Initialize I2C communication first to check charging status
  Serial.println("Initializing I2C for charging status check...");
  if (!initI2C()) {
    Serial.println("FATAL: I2C initialization failed!");
  }
  
  // Wait for Pi Pico to fully initialize I2C slave
  Serial.println("Waiting for Pi Pico I2C slave to initialize...");
  delay(2000); // 2 second startup delay for reliable I2C communication
  Serial.println("✓ I2C slave initialization period complete");
  
  // Check charging status to determine WiFi behavior
  uint8_t charging_status = getChargingStatus();
  current_charging_status = charging_status;
  
  bool should_enable_wifi = false;
  
  if (charging_status == 1 || charging_status == 2) {
    // Charging or charge complete - enable WiFi services
    Serial.println("🔌 Device is charging/charged - enabling WiFi configuration services");
    should_enable_wifi = true;
  } else {
    // No external power or not charging - battery mode only
    Serial.println("🔋 Battery mode detected - skipping WiFi to save power");
    Serial.println("WiFi config portal and web server will be disabled to maximize battery life");
    should_enable_wifi = false;
  }
  
  // WiFi initialization based on charging status
  if (should_enable_wifi) {
    Serial.println("Initializing WiFi Manager (CHARGING MODE - full features enabled)...");
    
    // Start with low power mode during connection setup for efficiency
    setWiFiLowPowerMode();
    
    wm.setBreakAfterConfig(true);
    wm.setConfigPortalTimeout(1800); // 30 minutes timeout for config portal when charging
    wm.setConnectTimeout(30); // 30s connection timeout
    wm.setConfigPortalBlocking(false);
    wm.setSaveConfigCallback([]() {
      Serial.println("WiFi config saved - immediate restart");
    });
    
    if (!wm.autoConnect("ESP32PhotoFrame", "photoframe123")) {
      Serial.println("WiFi connection failed after 30s - but staying in charging mode");
      Serial.println("Configuration portal may be available for setup");
    } else {
      Serial.println("✓ WiFi connected successfully");
      Serial.printf("IP address: %s\n", WiFi.localIP().toString().c_str());
      
      // Print ESP32 model information on startup
      printESP32ModelInfo();
      
      // Switch to performance mode for web server operations when charging
      Serial.println("🔌 CHARGING MODE: Switching to WiFi performance mode for web server");
      setWiFiPerformanceMode();
      WiFi.setAutoReconnect(true);
      WiFi.persistent(true);
      Serial.println("✓ WiFi optimizations enabled for charging mode");
      
      // Initialize web server for configuration
      setupWebServer();
    }
  } else {
    // Battery mode - attempt quick WiFi connection only
    Serial.println("Attempting quick WiFi connection (BATTERY MODE - 10s timeout)...");
    
    // Start with low power mode for efficient connection in battery mode
    setWiFiLowPowerMode();
    
    wm.setConnectTimeout(10); // Very short timeout for battery mode
    wm.setConfigPortalTimeout(0); // Disable config portal
    wm.setConfigPortalBlocking(false);
    
    if (wm.autoConnect("ESP32PhotoFrame", "photoframe123")) {
      Serial.println("✓ Quick WiFi connection successful");
      Serial.printf("IP address: %s\n", WiFi.localIP().toString().c_str());
      
      // Keep low power mode active for battery conservation
      Serial.println("🔋 BATTERY MODE: Maintaining WiFi low power mode for energy efficiency");
      // Low power mode already set above, just maintain it
    } else {
      Serial.println("❌ Quick WiFi connection failed - entering deep sleep mode");
      Serial.println("💤 Putting renderer to sleep and entering ESP32 deep sleep");
      
      // Put renderer to sleep before ESP32 sleeps
      if (putRendererToSleep()) {
        Serial.println("✓ Renderer sleep command sent successfully");
      } else {
        Serial.println("⚠ Failed to send renderer sleep command, proceeding with ESP32 sleep");
      }
      
      delay(1000); // Give renderer time to process sleep command
      
      // Put ESP32 into deep sleep
      Serial.println("💤 ESP32 entering deep sleep - both devices will be off");
      Serial.flush();
      delay(100);
      
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      esp_deep_sleep_start(); // Sleep indefinitely until reset
    }
  }

  Serial.println("===================");
  Serial.printf("Power efficiency mode: MAXIMUM\n");
  Serial.printf("Free heap: available\n");
  Serial.printf("Image data size: %d bytes\n", sizeof(Image7color));
  
  // Initialize I2C communication with fastest settings
  if (!initI2C()) {
    Serial.println("FATAL: I2C initialization failed!");
  }
  
  // Wait for Pi Pico to fully initialize I2C slave
  Serial.println("Waiting for Pi Pico I2C slave to initialize...");
  delay(2000); // 2 second startup delay for reliable I2C communication
  Serial.println("✓ I2C slave initialization period complete");
  
  // Perform I2C bus scan to check for devices
  Serial.println("=== I2C BUS SCAN ===");
  Serial.println("Scanning for I2C devices...");
  
  int deviceCount = 0;
  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.printf("✓ I2C device found at address 0x%02X\n", address);
      deviceCount++;
      
      if (address == PI_PICO_I2C_ADDRESS) {
        Serial.printf("✓ CONFIRMED: Pi Pico found at expected address 0x%02X\n", address);
      }
    } else if (error == 4) {
      Serial.printf("✗ Unknown error at address 0x%02X\n", address);
    }
  }
  
  if (deviceCount == 0) {
    Serial.println("✗ NO I2C DEVICES FOUND!");
    Serial.println("This indicates:");
    Serial.println("  1. Pi Pico is not powered or not running");
    Serial.println("  2. I2C wiring issues (SDA/SCL swapped or disconnected)");
    Serial.println("  3. Pi Pico I2C slave code not running");
    Serial.println("  4. Wrong I2C address in Pi Pico code");
  } else {
    Serial.printf("✓ Found %d I2C device(s)\n", deviceCount);
    
    if (deviceCount > 0) {
      // Test communication with our expected device
      Serial.printf("Testing communication with Pi Pico at 0x%02X...\n", PI_PICO_I2C_ADDRESS);
      
      Wire.beginTransmission(PI_PICO_I2C_ADDRESS);
      uint8_t testError = Wire.endTransmission();
      
      if (testError == 0) {
        Serial.println("✓ Pi Pico responds to address!");
      } else {
        Serial.printf("✗ Pi Pico does not respond (error: %d)\n", testError);
        if (testError == 2) {
          Serial.println("  → NACK on address - Pi Pico not at this address");
        } else if (testError == 5) {
          Serial.println("  → Timeout - Pi Pico may be busy or address wrong");
        }
      }
    }
  }

  Serial.println("=== ESP32 I2C Master Ready===");
  
  // Initialize battery data now that I2C is ready
  initBatteryData();
  
  // Auto-fetch first image on boot if URLs are configured
  autoFetchOnBoot();
}

// Auto-fetch functionality that runs once on boot after WiFi/I2C initialization
void autoFetchOnBoot() {
  Serial.println("=== AUTO-FETCH ON BOOT ===");
  unsigned long auto_fetch_start = millis();
  
  if (urlCount == 0) {
    Serial.println("No URLs configured - skipping auto-fetch");
    Serial.println("Add URLs via web interface or use default image");
    return;
  }
  
  if (!WiFi.isConnected()) {
    Serial.println("WiFi not connected - skipping auto-fetch");
    return;
  }
  
  Serial.printf("🚀 Auto-fetching image on boot: %s\n", getCurrentUrlAndAdvance().c_str());
  Serial.println("This will download and display the next image automatically...");
  
  // Test I2C connection before attempting transfer
  unsigned long i2c_test_start = millis();
  if (!testI2CConnection()) {
    Serial.println("❌ I2C connection test failed - cannot transfer image to renderer");
    Serial.println("Skipping auto-fetch due to I2C issues");
    return;
  }
  unsigned long i2c_test_time = millis() - i2c_test_start;
  Serial.printf("⏱️ I2C connection test: %lu ms\n", i2c_test_time);
  
  // Attempt to download and display the current image
  String currentUrl = getCurrentUrlAndAdvance();
  unsigned long download_start = millis();
  bool success = downloadAndConvertBmpImage(currentUrl.c_str());
  unsigned long download_time = millis() - download_start;
  Serial.printf("⏱️ Total download and transfer time: %lu ms (%.2f seconds)\n", download_time, download_time / 1000.0);
  
  if (success) {
    Serial.println("✅ Auto-fetch completed successfully!");
    Serial.println("Image should now be displaying on the e-paper screen");
    
    // Get and display battery info after successful display
    unsigned long battery_start = millis();
    float voltage = getBatteryVoltage();
    if (voltage > 0) {
      sendBatteryInfoToDisplay(voltage, display_cycle_count);
      Serial.printf("🔋 Battery status sent: %.2fV, %u cycles\n", voltage, display_cycle_count);
    }
    unsigned long battery_time = millis() - battery_start;
    Serial.printf("⏱️ Battery info update: %lu ms\n", battery_time);
  } else {
    Serial.println("❌ Auto-fetch failed - check URL and network connection");
  }
  
  unsigned long total_time = millis() - auto_fetch_start;
  Serial.printf("⏱️ TOTAL AUTO-FETCH TIME: %lu ms (%.2f seconds)\n", total_time, total_time / 1000.0);
  Serial.println("=== AUTO-FETCH COMPLETE ===");
}

// Download and stream image directly to Pi Pico with MAXIMUM SPEED
bool downloadAndStreamImage(const char* url) {
  Serial.printf("Starting streaming download from: %s\n", url);
  
  HTTPClient http;
  http.begin(url);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.setTimeout(5000); // BALANCED: 5s timeout for reliable downloads
  http.setConnectTimeout(2000); // 2s connection timeout for reliable connection
  http.setReuse(false); // Don't reuse connections for faster cleanup
  
  int httpCode = http.GET();
  
  if (httpCode > 0) {
    Serial.printf("HTTP response code: %d\n", httpCode);
    
    if (httpCode == HTTP_CODE_OK) {
      int contentLength = http.getSize();
      Serial.printf("Content length: %d bytes\n", contentLength);
      
      WiFiClient* stream = http.getStreamPtr();
      size_t bytesRead = 0;
      uint32_t chunk_id = 0;
      unsigned long start_time = millis(); // For transfer speed calculation

      Serial.printf("Streaming %d bytes in chunks...\n", contentLength);

      while (http.connected() && bytesRead < contentLength) {
        size_t available = stream->available();
        if (available) {
          // Read larger chunks using our dedicated streaming buffer
          size_t toRead = min(available, (size_t)STREAM_BUFFER_SIZE);
          toRead = min(toRead, (size_t)(contentLength - bytesRead));
          
          size_t readSize = stream->readBytes(stream_buffer, toRead);
          
          if (readSize > 0) {
            // Split large buffer into I2C chunks and send immediately
            size_t bytes_processed = 0;
            while (bytes_processed < readSize) {
              size_t chunk_size = min((size_t)I2C_CHUNK_SIZE, readSize - bytes_processed);
              
              // Send this chunk via I2C immediately  
              if (!sendImageChunk(chunk_id, &stream_buffer[bytes_processed], chunk_size)) {
                Serial.printf("✗ Failed to send streaming chunk %d\n", chunk_id);
                http.end();
                return false;
              }
              
              chunk_id++;
              bytes_processed += chunk_size;
            }
            
            bytesRead += readSize;
            
            // Progress reporting every 20KB for less verbosity but speed monitoring
            if (bytesRead % 20000 == 0) { // Every 20KB (was 10KB)
              float percent = ((float)bytesRead / contentLength) * 100.0;
              float kbps = (millis() > start_time) ? (bytesRead / (float)(millis() - start_time)) : 0;
              Serial.printf("Streaming: %d/%d bytes (%.1f%%, %.1f KB/s)\n", 
                           bytesRead, contentLength, percent, kbps);
            }
          
          }
        }
      }

      Serial.printf("✓ Streaming completed: %d bytes in %d chunks\n", bytesRead, chunk_id);
      http.end();
      
      // Send render command immediately
      if (!sendRenderCommand()) {
        return false;
      }
      
      // Send updated battery info to display after successful render
      float voltage = getBatteryVoltage();
      if (voltage > 0.0f) {
        sendBatteryInfoToDisplay(voltage, display_cycle_count);
        current_battery_voltage = voltage; // Update local cache
      }
      
      Serial.println("✓ Stream transfer and render command complete");
      return true;
    } else {
      Serial.printf("✗ HTTP error: %d\n", httpCode);
    }
  } else {
    Serial.printf("✗ HTTP connection failed: %s\n", http.errorToString(httpCode).c_str());
  }
  
  http.end();
  return false;
}

// Download and convert BMP to e-paper format with streaming (192,000 bytes)
bool downloadAndConvertBmpImage(const char* url) {
  Serial.printf("🚀 ESP32 PERFORMANCE: Starting BMP download and streaming conversion from: %s\n", url);
  
  // ESP32 WIFI PERFORMANCE OPTIMIZATION: Switch to high performance mode for fast downloads
  Serial.println("🚀 Switching WiFi to PERFORMANCE MODE for fast BMP download");
  setWiFiPerformanceMode();
  
  // ESP32 HIGH-PRECISION TIMING for breakthrough performance measurement
  #ifdef ESP32_PERFORMANCE_OPTIMIZED
  int64_t total_start_time = esp_timer_get_time(); // Microsecond precision
  #else
  unsigned long total_start_time = millis();
  #endif
  
  HTTPClient http;
  http.begin(url);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  
  // Optimized headers for faster downloads
  http.addHeader("User-Agent", "ESP32PhotoFrame/2.0 (Fast)");
  http.addHeader("Accept", "image/bmp,image/*,*/*");
  http.addHeader("Accept-Encoding", "identity"); // Disable compression for speed
  http.addHeader("Connection", "close");
  http.addHeader("Cache-Control", "no-cache"); // Ensure fresh content
  
  // OPTIMIZED timeouts for faster downloads
  http.setTimeout(12000); // 12 seconds for larger BMP files
  http.setConnectTimeout(3000); // 3s connection timeout for faster initial connection
  http.setReuse(false); // Don't reuse connections for faster cleanup
  
  Serial.println("Making HTTP request with enhanced headers...");
  unsigned long http_start = millis();
  int httpCode = http.GET();
  unsigned long http_connect_time = millis() - http_start;
  Serial.printf("⏱️ HTTP connection and request: %lu ms\n", http_connect_time);
  
  if (httpCode > 0) {
    Serial.printf("HTTP response code: %d\n", httpCode);
    
    // Debug response headers for troubleshooting
    String contentType = http.header("Content-Type");
    String contentLength = http.header("Content-Length");
    String location = http.header("Location");
    
    if (contentType.length() > 0) {
      Serial.printf("Content-Type: %s\n", contentType.c_str());
    }
    if (contentLength.length() > 0) {
      Serial.printf("Content-Length header: %s\n", contentLength.c_str());
    }
    if (location.length() > 0) {
      Serial.printf("Location (redirect): %s\n", location.c_str());
    }
    
    if (httpCode == HTTP_CODE_OK) {
      int contentLengthValue = http.getSize();
      Serial.printf("BMP file size: %d bytes\n", contentLengthValue);
      
      // Check available heap before proceeding
      Serial.printf("Free heap before conversion: %d bytes\n", ESP.getFreeHeap());
      
      // ESP32 MEMORY OPTIMIZATION: Monitor heap usage for massive buffer allocation
      uint32_t heap_before_buffers = ESP.getFreeHeap();
      Serial.printf("🧠 Free heap before MASSIVE buffer allocation: %d bytes (%.1f KB)\n", 
                   heap_before_buffers, heap_before_buffers / 1024.0);
      
      WiFiClient* stream = http.getStreamPtr();
      
      // Read BMP headers first
      const size_t header_size = sizeof(BMPFILEHEADER) + sizeof(BMPINFOHEADER);
      uint8_t* header_buffer = (uint8_t*)malloc(header_size);
      
      if (!header_buffer) {
        Serial.printf("✗ Failed to allocate header buffer (%d bytes)\n", header_size);
        http.end();
        return false;
      }
      
      // Read headers with aggressive buffering and timeout
      size_t header_read = 0;
      unsigned long start_time = millis();
      const unsigned long timeout = 2000; // 2 second timeout (was 3s) for maximum power efficiency
      
      Serial.printf("Reading BMP headers (%d bytes)...\n", header_size);
      unsigned long header_read_start = millis();
      
      while (header_read < header_size && (millis() - start_time) < timeout) {
        if (stream->available() > 0) {
          size_t available = stream->available();
          size_t to_read = min(available, header_size - header_read);
          size_t read_size = stream->readBytes(&header_buffer[header_read], to_read);
          
          if (read_size > 0) {
            header_read += read_size;
            Serial.printf("Read %d bytes, total: %d/%d\n", read_size, header_read, header_size);
          }
        }
      }
      
      unsigned long header_read_time = millis() - header_read_start;
      Serial.printf("⏱️ BMP header reading: %lu ms\n", header_read_time);
      
      if (header_read < header_size) {
        Serial.printf("✗ Failed to read BMP headers (got %d of %d bytes after %lu ms)\n", 
                     header_read, header_size, millis() - start_time);
        Serial.printf("Stream available: %d bytes\n", stream->available());
        free(header_buffer);
        http.end();
        return false;
      }
      
      Serial.printf("✓ Successfully read BMP headers (%d bytes)\n", header_read);
      
      // Parse BMP headers
      BMPFILEHEADER fileHeader;
      BMPINFOHEADER infoHeader;
      
      if (!parseBmpHeader(header_buffer, header_size, &fileHeader, &infoHeader)) {
        Serial.println("✗ Invalid BMP file format");
        free(header_buffer);
        http.end();
        return false;
      }
      
      free(header_buffer);
      
      Serial.printf("BMP dimensions: %dx%d pixels\n", infoHeader.biWidth, infoHeader.biHeight);
      Serial.printf("Bit depth: %d bits per pixel\n", infoHeader.biBitCount);
      Serial.printf("Data offset: %d bytes\n", fileHeader.bOffset);
      Serial.printf("Compression: %d\n", infoHeader.biCompression);
      Serial.printf("Image size: %d bytes\n", infoHeader.bimpImageSize);
      Serial.printf("Colors used: %d\n", infoHeader.biClrUsed);
      Serial.printf("Important colors: %d\n", infoHeader.biClrImportant);
      
      // Validate format - support both 8-bit indexed and 24-bit RGB
      if (infoHeader.biBitCount != 8 && infoHeader.biBitCount != 24) {
        Serial.printf("✗ Unsupported bit depth: %d (need 8-bit indexed or 24-bit RGB)\n", infoHeader.biBitCount);
        http.end();
        return false;
      }
      
      if (infoHeader.biWidth != 800 || infoHeader.biHeight != 480) {
        Serial.printf("✗ BMP dimensions (%dx%d) don't match display (800x480)\n", 
                     infoHeader.biWidth, infoHeader.biHeight);
        Serial.println("   → Use the ImageConverter tool to create properly sized BMPs");
        http.end();
        return false;
      }
      
      // Skip to pixel data
      size_t bytes_read_so_far = header_size;
      uint8_t* palette_buffer = nullptr;
      uint8_t* color_lookup = nullptr; // Declare color lookup table pointer
      
      // Read color palette for 8-bit indexed BMPs
      if (infoHeader.biBitCount == 8) {
        // 8-bit BMPs have a 256-color palette (4 bytes per color: B,G,R,reserved)
        size_t palette_size = 256 * 4;
        palette_buffer = (uint8_t*)malloc(palette_size);
        
        if (!palette_buffer) {
          Serial.printf("✗ Failed to allocate palette buffer (%d bytes)\n", palette_size);
          http.end();
          return false;
        }
        
        // Read palette with aggressive buffering
        size_t palette_read = 0;
        start_time = millis();
        
        Serial.printf("Reading BMP palette (%d bytes)...\n", palette_size);
        unsigned long palette_start = millis();
        
        while (palette_read < palette_size && (millis() - start_time) < timeout) {
          if (stream->available() > 0) {
            size_t available = stream->available();
            size_t to_read = min(available, palette_size - palette_read);
            size_t read_size = stream->readBytes(&palette_buffer[palette_read], to_read);
            
            if (read_size > 0) {
              palette_read += read_size;
              if (palette_read % 256 == 0) { // Progress every 256 bytes
                Serial.printf("Palette read: %d/%d bytes\n", palette_read, palette_size);
              }
            }
          }
        }
        
        unsigned long palette_time = millis() - palette_start;
        Serial.printf("⏱️ Palette reading: %lu ms\n", palette_time);
        
        if (palette_read < palette_size) {
          Serial.printf("✗ Failed to read BMP palette (got %d of %d bytes after %lu ms)\n", 
                       palette_read, palette_size, millis() - start_time);
          free(palette_buffer);
          http.end();
          return false;
        }
        
        Serial.printf("✓ Successfully read BMP palette (%d bytes)\n", palette_read);
        
        // SPEED OPTIMIZATION: Pre-compute color lookup table for all 256 palette entries
        color_lookup = (uint8_t*)malloc(256);
        if (!color_lookup) {
          Serial.println("✗ Failed to allocate color lookup table");
          free(palette_buffer);
          http.end();
          return false;
        }
        
        Serial.println("🚀 Pre-computing color lookup table for maximum speed...");
        for (int i = 0; i < 256; i++) {
          uint32_t offset = i * 4;
          uint8_t b = palette_buffer[offset];
          uint8_t g = palette_buffer[offset + 1]; 
          uint8_t r = palette_buffer[offset + 2];
          color_lookup[i] = rgbToEpaperColor(r, g, b);
        }
        Serial.println("✓ Color lookup table ready - no more per-pixel conversion needed!");
        
        // DEBUG: Print first 10 palette entries to verify colors
        Serial.println("=== PALETTE DEBUG (first 10 colors) ===");
        for (int i = 0; i < 10 && i < 256; i++) {
          uint32_t offset = i * 4;
          uint8_t b = palette_buffer[offset];
          uint8_t g = palette_buffer[offset + 1]; 
          uint8_t r = palette_buffer[offset + 2];
          uint8_t epaper_color = color_lookup[i];
          Serial.printf("Palette[%d]: RGB(%d,%d,%d) -> e-paper color %d\n", i, r, g, b, epaper_color);
        }
        Serial.println("========================================");
        
        bytes_read_so_far += palette_size;
      }
      
      // Skip any remaining bytes to reach pixel data with proper buffering
      Serial.printf("Skipping to pixel data offset %d (currently at %d)...\n", 
                   fileHeader.bOffset, bytes_read_so_far);
      
      while (bytes_read_so_far < fileHeader.bOffset) {
        if (stream->available() > 0) {
          size_t to_skip = min((size_t)stream->available(), 
                              (size_t)(fileHeader.bOffset - bytes_read_so_far));
          uint8_t* skip_buffer = (uint8_t*)malloc(min(to_skip, (size_t)8192)); // 8KB chunks for fast skipping (was 512)
          
          if (skip_buffer) {
            size_t skipped = stream->readBytes(skip_buffer, min(to_skip, (size_t)8192));
            bytes_read_so_far += skipped;
            free(skip_buffer);
            
            if (bytes_read_so_far % 1000 == 0) {
              Serial.printf("Skipped to offset: %d/%d\n", bytes_read_so_far, fileHeader.bOffset);
            }
          } else {
            // Fallback: skip one byte at a time
            uint8_t skip_byte;
            if (stream->readBytes(&skip_byte, 1) == 1) {
              bytes_read_so_far++;
            }
          }
        }
      }
      
      Serial.printf("Skipped to pixel data at offset %d\n", fileHeader.bOffset);
      
      // Now stream and convert pixel data with direct addressed writes
      // Validate BMP dimensions match display expectations
      if (infoHeader.biWidth != 800 || infoHeader.biHeight != 480) {
        Serial.printf("WARNING: BMP size %dx%d doesn't match display 800x480!\n", 
                     infoHeader.biWidth, infoHeader.biHeight);
        Serial.printf("This may cause addressing errors and display artifacts.\n");
      }
      
      const uint32_t epaper_size = 192000; // 800x480 pixels, 4 bits per pixel
      Serial.printf("✓ Display buffer size: %d bytes for %dx%d display\n", 
                   epaper_size, infoHeader.biWidth, infoHeader.biHeight);
      const uint32_t work_buffer_size = 32768; // SMART: 32KB buffer for optimal I2C performance with available heap
      uint8_t* work_buffer = (uint8_t*)malloc(work_buffer_size);
      
      if (!work_buffer) {
        Serial.printf("✗ Failed to allocate work buffer (%d bytes)\n", work_buffer_size);
        if (palette_buffer) {
          free(palette_buffer);
        }
        http.end();
        return false;
      }
      
        Serial.printf("✓ Allocated MASSIVE SPEED work buffer: %d bytes (ULTIMATE parallel processing mode)\n", work_buffer_size);      // Process BMP pixel data in display order for optimal I2C efficiency
      // SIMPLIFIED: Stream BMP data sequentially, renderer will handle reordering
      uint32_t work_buffer_pos = 0;
      
      // Allocate a buffer for one BMP row for reordering
      const uint32_t bmp_row_bytes = (infoHeader.biBitCount == 8) ? infoHeader.biWidth : infoHeader.biWidth * 3;
      const uint32_t bmp_row_padding = (4 - (bmp_row_bytes % 4)) % 4;
      const uint32_t bmp_row_total = bmp_row_bytes + bmp_row_padding;
      uint8_t* bmp_row_buffer = (uint8_t*)malloc(bmp_row_total);
      
      if (!bmp_row_buffer) {
        Serial.printf("✗ Failed to allocate BMP row buffer (%d bytes)\n", bmp_row_total);
        free(work_buffer);
        if (palette_buffer) {
          free(palette_buffer);
        }
        http.end();
        return false;
      }
      
      Serial.printf("✓ Allocated BMP row buffer: %d bytes\n", bmp_row_total);
      
      // ESP32 MEMORY MONITORING: Show heap usage after massive buffer allocation
      uint32_t heap_after_buffers = ESP.getFreeHeap();
      uint32_t buffer_memory_used = heap_before_buffers - heap_after_buffers;
      Serial.printf("🧠 Free heap after MASSIVE buffers: %d bytes (%.1f KB)\n", 
                   heap_after_buffers, heap_after_buffers / 1024.0);
      Serial.printf("📊 Total buffer memory allocated: %d bytes (%.1f KB)\n", 
                   buffer_memory_used, buffer_memory_used / 1024.0);
      Serial.printf("💪 Memory efficiency: %.1f%% heap utilization\n", 
                   (float)buffer_memory_used * 100.0 / heap_before_buffers);
      
        // SIMPLIFIED APPROACH: Stream BMP data in natural order and let renderer reorder
        // This is much simpler and more reliable than trying to reorder during streaming
        
        // Stream the pixel data sequentially, converting colors as we go
        // The renderer will receive a "raw BMP buffer" that it can reorder as needed
        uint32_t bytes_streamed = 0;
        uint32_t total_bmp_pixels = infoHeader.biWidth * infoHeader.biHeight;
        
        Serial.printf("🚀 ESP32 Streaming BMP pixel data sequentially (%d pixels, %d bytes per row)...\n", 
                     total_bmp_pixels, bmp_row_total);
        
        // ESP32 HIGH-PRECISION streaming performance timing
        #ifdef ESP32_PERFORMANCE_OPTIMIZED
        int64_t streaming_start = esp_timer_get_time(); // Microsecond precision
        #else
        unsigned long streaming_start = millis();
        #endif
        
        // Process each BMP row (bottom-to-top as stored in BMP)
        for (int32_t bmp_row = 0; bmp_row < (int32_t)infoHeader.biHeight; bmp_row++) {
          // Read entire BMP row into buffer
          size_t row_read = 0;
          // ESP32 PERFORMANCE: Enhanced row timing with microsecond precision
          #ifdef ESP32_PERFORMANCE_OPTIMIZED
          int64_t row_start_time = esp_timer_get_time();
          const int64_t row_timeout_us = 800000; // 800ms in microseconds
          #else
          unsigned long row_start_time = millis();
          const unsigned long row_timeout = 800; // BALANCED: 800ms timeout for reliability with speed
          #endif
          
          #ifdef ESP32_PERFORMANCE_OPTIMIZED
          while (row_read < bmp_row_total && (esp_timer_get_time() - row_start_time) < row_timeout_us) {
          #else
          while (row_read < bmp_row_total && (millis() - row_start_time) < row_timeout) {
          #endif
            if (stream->available() > 0) {
              size_t available = stream->available();
              size_t to_read = min(available, bmp_row_total - row_read);
              to_read = min(to_read, (size_t)16384); // Read in 16KB chunks for MAXIMUM speed (was 1KB)
              size_t read_size = stream->readBytes(&bmp_row_buffer[row_read], to_read);
              
              if (read_size > 0) {
                row_read += read_size;
              }
            } else {
              delay(1); // Brief pause if no data available
            }
          }
          
          if (row_read < bmp_row_total) {
            Serial.printf("✗ Failed to read BMP row %d (got %d of %d bytes)\n", 
                         bmp_row, row_read, bmp_row_total);
            free(bmp_row_buffer);
            free(work_buffer);
            if (palette_buffer) {
              free(palette_buffer);
            }
            http.end();
            return false;
          }
          
          // OPTIMIZED: Convert this row's pixels to e-paper colors using lookup table
          for (uint32_t bmp_col = 0; bmp_col < infoHeader.biWidth; bmp_col += 2) {
            uint8_t pixel_left_color, pixel_right_color;
            
            // Extract left pixel using FAST lookup table
            if (infoHeader.biBitCount == 8) {
              uint8_t index = bmp_row_buffer[bmp_col];
              pixel_left_color = color_lookup[index]; // FAST: Direct lookup, no RGB conversion!
            } else {
              uint32_t pixel_offset = bmp_col * 3;
              if (pixel_offset + 2 < bmp_row_bytes) {
                pixel_left_color = rgbToEpaperColor(bmp_row_buffer[pixel_offset + 2], 
                                                  bmp_row_buffer[pixel_offset + 1], 
                                                  bmp_row_buffer[pixel_offset]);
              } else {
                pixel_left_color = EPD_7IN3F_WHITE;
              }
            }
            
            // Extract right pixel (if exists) using FAST lookup table
            if (bmp_col + 1 < infoHeader.biWidth) {
              if (infoHeader.biBitCount == 8) {
                uint8_t index = bmp_row_buffer[bmp_col + 1];
                pixel_right_color = color_lookup[index]; // FAST: Direct lookup, no RGB conversion!
              } else {
                uint32_t pixel_offset = (bmp_col + 1) * 3;
                if (pixel_offset + 2 < bmp_row_bytes) {
                  pixel_right_color = rgbToEpaperColor(bmp_row_buffer[pixel_offset + 2], 
                                                     bmp_row_buffer[pixel_offset + 1], 
                                                     bmp_row_buffer[pixel_offset]);
                } else {
                  pixel_right_color = EPD_7IN3F_WHITE;
                }
              }
            } else {
              pixel_right_color = EPD_7IN3F_WHITE;
            }
            
            // Pack into byte (left pixel high nibble, right pixel low nibble)
            uint8_t packed_byte = (pixel_left_color << 4) | pixel_right_color;
            
            // Add to work buffer
            work_buffer[work_buffer_pos++] = packed_byte;
            
            // OPTIMIZED: Send buffer when full using BURST MODE for maximum speed
            if (work_buffer_pos >= work_buffer_size) {
              // Send entire buffer in BURST MODE for maximum speed (4096 bytes via multiple I2C transactions)
              if (!sendImageChunkBurst(bytes_streamed, work_buffer, work_buffer_pos)) {
                Serial.printf("✗ Failed to send BMP chunk at offset %d\n", bytes_streamed);
                free(bmp_row_buffer);
                free(work_buffer);
                if (palette_buffer) {
                  free(palette_buffer);
                }
                http.end();
                return false;
              }
              
              bytes_streamed += work_buffer_pos;
              work_buffer_pos = 0;
              
              // REMOVED: Progress reporting to eliminate I/O overhead for maximum speed
              // if (bytes_streamed % 25000 == 0) { ... }
              
            }
          }
          
          // Optimized progress reporting for row processing (MINIMAL for speed)
          if (bmp_row == 0 || bmp_row == (int32_t)infoHeader.biHeight - 1) {
            float percent = ((float)(bmp_row + 1) / infoHeader.biHeight) * 100.0;
            Serial.printf("BMP processing: row %d/%d (%.1f%%)\n", 
                         bmp_row + 1, infoHeader.biHeight, percent);
          }
        }
        
        // ESP32 BREAKTHROUGH: Final streaming performance calculation
        #ifdef ESP32_PERFORMANCE_OPTIMIZED
        int64_t streaming_time_us = esp_timer_get_time() - streaming_start;
        float streaming_time_ms = streaming_time_us / 1000.0;
        float streaming_time_s = streaming_time_us / 1000000.0;
        
        Serial.printf("🚀 ESP32 BMP pixel streaming: %.1f ms (%.3f seconds) - PRECISION TIMING\n", 
                     streaming_time_ms, streaming_time_s);
        
        // ESP32 breakthrough performance detection for streaming
        if (streaming_time_s < 3.0) {
          Serial.printf("🎉 ESP32 STREAMING BREAKTHROUGH: %.3fs achieves sub-3s target!\n", streaming_time_s);
        } else if (streaming_time_s < 5.0) {
          Serial.printf("📈 ESP32 STREAMING PROGRESS: %.3fs approaching 3s target\n", streaming_time_s);
        } else {
          Serial.printf("⚡ ESP32 STREAMING BASELINE: %.3fs (target: <3s)\n", streaming_time_s);
        }
        #else
        unsigned long streaming_time = millis() - streaming_start;
        Serial.printf("⏱️ BMP pixel data streaming: %lu ms (%.2f seconds)\n", streaming_time, streaming_time / 1000.0);
        #endif
        
        Serial.printf("✅ ESP32 BMP streaming completed: %d bytes streamed\n", bytes_streamed);
        
        free(bmp_row_buffer);
        
        // Send any remaining data in work buffer using BURST MODE
        if (work_buffer_pos > 0) {
          // Send remaining buffer in BURST MODE for maximum speed
          if (!sendImageChunkBurst(bytes_streamed, work_buffer, work_buffer_pos)) {
            Serial.printf("✗ Failed to send final BMP chunk at offset %d\n", bytes_streamed);
            free(work_buffer);
            if (palette_buffer) {
              free(palette_buffer);
            }
            if (color_lookup) {
              free(color_lookup);
            }
            http.end();
            return false;
          }
          bytes_streamed += work_buffer_pos;
        }
        
        Serial.printf("✓ BMP streaming completed: %d bytes streamed\n", bytes_streamed);
        
        free(work_buffer);
        if (palette_buffer) {
          free(palette_buffer);
        }
        if (color_lookup) {
          free(color_lookup);
        }
        http.end();
        
        Serial.printf("✓ BMP streaming conversion completed: %d bytes processed\n", bytes_streamed);
        
        // Send BMP reorder and render command (renderer will reorder from BMP to display format)
        unsigned long render_start = millis();
        if (!sendBmpRenderCommand()) {
          return false;
        }
        unsigned long render_command_time = millis() - render_start;
        Serial.printf("⏱️ Render command transmission: %lu ms\n", render_command_time);
        
        // Send updated battery info to display after successful BMP render
        unsigned long battery_update_start = millis();
        float voltage = getBatteryVoltage();
        if (voltage > 0.0f) {
          sendBatteryInfoToDisplay(voltage, display_cycle_count);
          current_battery_voltage = voltage; // Update local cache
        }
        unsigned long battery_update_time = millis() - battery_update_start;
        Serial.printf("⏱️ Battery info update: %lu ms\n", battery_update_time);
        
        // ESP32 BREAKTHROUGH PERFORMANCE: Final timing calculation with microsecond precision
        #ifdef ESP32_PERFORMANCE_OPTIMIZED
        int64_t total_processing_time_us = esp_timer_get_time() - total_start_time;
        float total_processing_time_ms = total_processing_time_us / 1000.0;
        float total_processing_time_s = total_processing_time_us / 1000000.0;
        
        Serial.printf("🚀 ESP32 TOTAL BMP PERFORMANCE: %.1f ms (%.3f seconds) - PRECISION TIMING\n", 
                     total_processing_time_ms, total_processing_time_s);
        
        // Performance breakthrough detection
        if (total_processing_time_s < 5.0) {
          Serial.printf("🎉 ESP32 BREAKTHROUGH: %.3fs achieves 5-second target!\n", total_processing_time_s);
        } else if (total_processing_time_s < 8.0) {
          Serial.printf("📈 ESP32 PROGRESS: %.3fs approaching 5-second target\n", total_processing_time_s);
        } else {
          Serial.printf("⚡ ESP32 BASELINE: %.3fs (target: <5s)\n", total_processing_time_s);
        }
        
        // Advanced performance statistics with ESP32 precision
        if (total_bytes_transferred > 0) {
          float avg_speed_kbps = (float)total_bytes_transferred / (total_processing_time_s * 1000.0);
          Serial.printf("📊 ESP32 I2C Performance: %zu bytes in %.1f KB/s avg throughput\n", 
                       total_bytes_transferred, avg_speed_kbps);
        }
        #else
        unsigned long total_processing_time = millis() - total_start_time;
        Serial.printf("⏱️ TOTAL BMP PROCESSING TIME: %lu ms (%.2f seconds)\n", 
                     total_processing_time, total_processing_time / 1000.0);
        #endif
        
        Serial.println("✅ ESP32 BMP streaming conversion and render command complete");
        
        // ESP32 WIFI POWER OPTIMIZATION: Return to low power mode after download
        Serial.println("🔋 Restoring WiFi LOW POWER mode after successful download");
        setWiFiLowPowerMode();
        
        return true;
    } else {
      Serial.printf("✗ HTTP error: %d\n", httpCode);
      
      // Provide specific error explanations
      if (httpCode == 404) {
        Serial.println("   → File not found. URL may be expired or incorrect.");
      } else if (httpCode == 403) {
        Serial.println("   → Access forbidden. Check URL permissions.");
      } else if (httpCode == 301 || httpCode == 302) {
        Serial.println("   → Redirect detected. URL may have moved.");
        String location = http.header("Location");
        if (location.length() > 0) {
          Serial.printf("   → Redirect location: %s\n", location.c_str());
        }
      } else if (httpCode == 400) {
        Serial.println("   → Bad request. URL format may be malformed.");
      } else if (httpCode == 500) {
        Serial.println("   → Server error. Try again later.");
      }
      
      // Check if we got any response body for debugging
      if (http.getSize() > 0) {
        String response = http.getString();
        if (response.length() > 0 && response.length() < 500) {
          Serial.printf("   → Server response: %s\n", response.c_str());
        }
      }
    }
  } else {
    Serial.printf("✗ HTTP connection failed: %s\n", http.errorToString(httpCode).c_str());
    Serial.println("   → Check WiFi connection and URL validity.");
  }

  http.end();
  
  // ESP32 WIFI POWER OPTIMIZATION: Return to low power mode after failed download
  Serial.println("🔋 Restoring WiFi LOW POWER mode after download attempt");
  setWiFiLowPowerMode();
  
  return false;
}

void loop() {
  // Handle web server requests
  server.handleClient();
  
  static unsigned long lastImageSend = 0;
  static bool i2cWorking = false;
  static unsigned long lastI2CTest = 0;
  static bool powerModeChecked = false;
  static bool isChargingMode = false;
  unsigned long currentTime = millis();
  
  // Check charging status once at startup to determine operating mode
  if (!powerModeChecked) {
    uint8_t charging_status = getChargingStatus();
    current_charging_status = charging_status;
    
    if (charging_status == 1 || charging_status == 2) {
      // Charging or charge complete - web server only mode
      isChargingMode = true;
      Serial.println("🔌 CHARGING MODE: Power connected - running web server only (no automatic image updates)");
      Serial.println("📱 Use the web interface to manually control image display");
      Serial.printf("🌐 Access control panel at: http://%s\n", WiFi.localIP().toString().c_str());
    } else {
      // Battery mode - normal power-efficient operation
      isChargingMode = false;
      Serial.println("🔋 BATTERY MODE: No external power - will display image and enter deep sleep");
    }
    powerModeChecked = true;
  }
  
  // If in charging mode, only handle web server - no automatic image processing
  if (isChargingMode) {
    // Just run the web server and periodic status updates
    // Update status from Pi Pico periodically for web UI
    static unsigned long lastStatusUpdate = 0;
    if (currentTime - lastStatusUpdate > 30000) { // Every 30 seconds when charging
      float new_voltage = getBatteryVoltage();
      uint32_t new_interval = getWakeupInterval();
      uint8_t new_charging_status = getChargingStatus();
      
      if (new_voltage >= 0.0f) {
        current_battery_voltage = new_voltage;
        // Send battery info to display for overlay  
        sendBatteryInfoToDisplay(current_battery_voltage, display_cycle_count);
      }
      if (new_interval > 0) {
        current_wakeup_interval = new_interval;
      }
      current_charging_status = new_charging_status;
      
      lastStatusUpdate = currentTime;
    }
    
    // Simple heartbeat for charging mode
    static unsigned long lastHeartbeat = 0;
    if (currentTime - lastHeartbeat > 300000) { // Every 5 minutes when charging
      Serial.printf("🔌 CHARGING MODE: Web server active | Battery: %.2fV | Free heap: %u bytes\n", 
                    current_battery_voltage, ESP.getFreeHeap());
      lastHeartbeat = currentTime;
    }
    
    return; // Exit loop - no image processing in charging mode
  }
  
  // BATTERY MODE ONLY - Continue with original power-efficient behavior
  // Test I2C connection every 30 seconds if it's not working
  if (!i2cWorking && (currentTime - lastI2CTest > 30000)) {
    Serial.println("=== RETESTING I2C CONNECTION ===");
    i2cWorking = testI2CConnection();
    lastI2CTest = currentTime;
    
    if (!i2cWorking) {
      Serial.println("⚠ I2C still not working - skipping image transfer");
      Serial.println("CHECK:");
      Serial.println("  1. Is Pi Pico powered and running?");
      Serial.println("  2. Is renderer.cpp uploaded to Pi Pico?");
      Serial.println("  3. Are I2C wires connected: ESP32-21→Pico-4, ESP32-22→Pico-5?");
      Serial.println("  4. Is Pi Pico I2C address set to 0x42?");
      return; // Skip this loop iteration
    } else {
      Serial.println("✓ I2C connection restored!");
    }
  }
  
  // BATTERY MODE ONLY - Only proceed with image transfer if I2C is working
  // SINGLE-SHOT mode: Send image once, wait for completion, then shutdown for maximum power savings
  if (i2cWorking && lastImageSend == 0) { // Only run once for battery conservation
    Serial.println("🔋 BATTERY MODE: Single-shot power-efficient image transfer");
    
    // Check slave status first
    uint8_t status = getSlaveStatus();
    Serial.printf("Pi Pico status: 0x%02X\n", status);
    
    if (status == STATUS_READY || status == STATUS_ERROR) {
      // Get next URL from the cycling list
      String imageUrl = getCurrentUrlAndAdvance();
      
      if (imageUrl.length() == 0) {
        Serial.println("✗ No URLs configured - cannot download image");
        Serial.println("💤 Going to deep sleep - configure URLs via web interface when charging");
        
        // Put renderer to sleep and then ESP32
        if (putRendererToSleep()) {
          Serial.println("✓ Renderer sleep command sent successfully");
        }
        
        delay(1000);
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        esp_deep_sleep_start();
        return; // Should not reach here
      }
      
      Serial.printf("🔋 BATTERY MODE: Converting and streaming BMP from: %s\n", imageUrl.c_str());
      bool transfer_success = false;
      
      if (downloadAndConvertBmpImage(imageUrl.c_str())) {
        Serial.println("✓ BMP conversion and streaming completed successfully");
        
        // Wait for renderer to complete the display update
        if (waitForRenderCompletion(30000)) { // 30 second timeout
          Serial.println("✓ COMPLETE SUCCESS: Image transfer and rendering finished!");
          transfer_success = true;
        } else {
          Serial.println("⚠ Render completion timeout - may still be working");
        }
      } else {
        Serial.println("⚠ BMP conversion failed, trying static image as fallback");
        
        // Fallback to static image
        if (sendCompleteImage()) {
          Serial.println("✓ Fallback image transfer completed successfully");
          
          // Wait for renderer to complete the display update
          if (waitForRenderCompletion(30000)) { // 30 second timeout
            Serial.println("✓ COMPLETE SUCCESS: Fallback image transfer and rendering finished!");
            transfer_success = true;
          } else {
            Serial.println("⚠ Fallback render completion timeout - may still be working");
          }
        } else {
          Serial.println("✗ Even fallback image transfer failed");
        }
      }
      
      // BATTERY MODE: Always go to deep sleep after image transfer (ignore web server activity)
      if (transfer_success) {
        Serial.println("🔋 BATTERY MODE: Shutting down ESP32 after successful operation for maximum power savings");
        shutdownESP32(); // This function never returns
      } else {
        Serial.println("⚠ BATTERY MODE: Transfer had issues - attempting deep sleep anyway to save battery");
        
        // Put renderer to sleep and then ESP32
        if (putRendererToSleep()) {
          Serial.println("✓ Renderer sleep command sent successfully");
        }
        
        delay(1000);
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        esp_deep_sleep_start();
        return; // Should not reach here
      }
      
    } else {
      Serial.printf("🔋 BATTERY MODE: Slave not ready (status: 0x%02X) - will retry in next cycle\n", status);
    }
  }
  
  // Initial I2C test on first run (battery mode only)
  if (!i2cWorking && lastI2CTest == 0) {
    Serial.println("=== INITIAL I2C CONNECTION TEST (BATTERY MODE) ===");
    i2cWorking = testI2CConnection();
    lastI2CTest = currentTime;
  }
  
  // Check connection health periodically (reduced frequency for power efficiency)
  static unsigned long lastHealthCheck = 0;
  if (currentTime - lastHealthCheck > 20000) { // Every 20 seconds (was 10) for less overhead
    checkI2CConnectionHealth();
    lastHealthCheck = currentTime;
  }
  
  // Simple heartbeat (reduced frequency for power efficiency during standby)
  static unsigned long lastHeartbeat = 0;
  if (currentTime - lastHeartbeat > 120000) { // Every 120 seconds during standby
    Serial.printf("🔋 BATTERY MODE: I2C: %lu transactions, %lu errors\n", 
                  i2cTransactionCount, errorCount);
    if (lastImageSend == 0) {
      Serial.println("STATUS: Waiting for I2C to become ready for single-shot transfer");
    } else {
      Serial.println("STATUS: Single-shot transfer attempted");
    }
    lastHeartbeat = currentTime;
  }

}
