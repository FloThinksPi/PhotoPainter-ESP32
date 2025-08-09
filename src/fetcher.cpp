/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2025 Florian Braun (FloThinksPi)
 */

#include <Arduino.h>
#include <WiFiManager.h>
#include <HTTPClient.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <ImageData.h>
#include <esp_sleep.h> // For deep sleep power management

// Forward declarations
bool downloadAndStreamImage(const char* url);
bool downloadAndConvertBmpImage(const char* url);

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

const int CHUNK_SIZE = 123; // Match actual I2C data capacity (128 - 5 byte header)

// I2C Pin Configuration - Using ESP32 hardware I2C pins
// These are the default ESP32 I2C pins for master mode
int I2C_SDA_PIN = 21;   // ESP32 pin 21 connects to Renderer pin 4 (SDA)  
int I2C_SCL_PIN = 22;   // ESP32 pin 22 connects to Renderer pin 5 (SCL)
#define PI_PICO_I2C_ADDRESS 0x42  // Pi Pico I2C slave address
 
WiFiManager wm;

// ULTRA-OPTIMIZED buffer sizes for maximum power efficiency
static constexpr size_t BUFFER_SIZE = 1024; // Doubled for faster HTTP reads
static constexpr size_t STREAM_BUFFER_SIZE = 2048; // Large streaming buffer
uint8_t i2c_buffer[BUFFER_SIZE] = { 0 };
uint8_t stream_buffer[STREAM_BUFFER_SIZE] = { 0 }; // Dedicated streaming buffer
uint32_t currentChunk = 0;  // Current chunk being sent
uint32_t totalChunks = (sizeof(Image7color) + CHUNK_SIZE - 1) / CHUNK_SIZE; // Total chunks needed (192000/123 = 1561)

// I2C command definitions for master->slave communication
#define CMD_WRITE_CHUNK       0x01
#define CMD_WRITE_CHUNK_ADDR  0x04  // Write chunk to specific address offset
#define CMD_RENDER_IMAGE      0x02  
#define CMD_GET_STATUS        0x03
#define CMD_RENDER_BMP_ORDER  0x05  // Render image data that's in BMP order (needs reordering)

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
  Serial.printf("Clock: 800kHz\n");
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

// Send a chunk of data to Pi Pico slave with improved retry and delays
bool sendImageChunk(uint32_t chunk_id, const uint8_t* data, size_t data_size) {
  // I2C buffer limit: 128 bytes buffer - 5 bytes header = 123 bytes max data
  if (data_size > 123) data_size = 123; // Max data size to fit in I2C transaction

  // More conservative retry loop - up to 5 attempts with small delays
  for (int attempt = 0; attempt < 5; attempt++) {
    if (attempt > 0) {
      delay(10); // Small delay between retries for I2C stability
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
      if (attempt >= 2) { // Log after 3 attempts
        Serial.printf("✗ Buffer overflow: only %d of %d bytes queued for chunk %d (attempt %d)\n", 
                      written, data_size, chunk_id, attempt + 1);
      }
      Wire.endTransmission(); // Clean up
      errorCount++;
      continue; // Retry with delay
    }
    
    uint8_t error = Wire.endTransmission();
    
    if (error == 0) {
      lastI2CActivity = millis();
      i2cTransactionCount++;
      return true; // Success!
    } else {
      if (attempt >= 2) { // Log after 3 attempts
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
  
  Serial.printf("✗ Failed to send chunk %d after %d attempts\n", chunk_id, 5);
  return false; // All attempts failed
}

// Send a chunk of data to specific address offset in Pi Pico slave memory with improved retry
bool sendImageChunkToAddress(uint32_t address_offset, const uint8_t* data, size_t data_size) {
  // I2C buffer limit: 128 bytes buffer - 9 bytes header = 119 bytes max data
  if (data_size > 119) data_size = 119; // Max data size to fit in I2C transaction with address

  // More conservative retry loop - up to 5 attempts with small delays
  for (int attempt = 0; attempt < 5; attempt++) {
    if (attempt > 0) {
      delay(10); // Small delay between retries for I2C stability
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
      if (attempt >= 2) { // Log after 3 attempts
        Serial.printf("✗ Buffer overflow: only %d of %d bytes queued for address 0x%08X (attempt %d)\n", 
                      written, data_size, address_offset, attempt + 1);
      }
      Wire.endTransmission(); // Clean up
      errorCount++;
      continue; // Retry with delay
    }
    
    uint8_t error = Wire.endTransmission();
    
    if (error == 0) {
      lastI2CActivity = millis();
      i2cTransactionCount++;
      return true; // Success!
    } else {
      if (attempt >= 2) { // Log after 3 attempts
        Serial.printf("✗ Failed to send chunk to address 0x%08X (attempt %d, error: %d)\n", 
                      address_offset, attempt + 1, error);
        if (error == 5) {
          Serial.println("   → I2C timeout - slave may be busy processing data");
        }
      }
      errorCount++;
    }
  }
  
  Serial.printf("✗ Failed to send chunk to address 0x%08X after %d attempts\n", address_offset, 5);
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
  
  // Disconnect WiFi to save power during shutdown
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  
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
  
  uint32_t totalChunks = (image_size + CHUNK_SIZE - 1) / CHUNK_SIZE;
  Serial.printf("Starting image transfer: %d bytes in %d chunks\n", image_size, totalChunks);
  
  // Reset chunk counter
  currentChunk = 0;
  
  // Send all chunks at maximum speed
  unsigned long transfer_start = millis();
  for (uint32_t chunk = 0; chunk < totalChunks; chunk++) {
    uint32_t offset = chunk * CHUNK_SIZE;
    size_t remaining = image_size - offset;
    size_t chunk_size = (remaining > CHUNK_SIZE) ? CHUNK_SIZE : remaining;
    
    if (!sendImageChunk(chunk, &image_data[offset], chunk_size)) {
      Serial.printf("✗ Failed to send chunk %d after retries\n", chunk);
      return false;
    }
    
    // Less frequent progress reporting - every 100 chunks or at completion
    if (chunk % 100 == 0 || chunk == totalChunks - 1) {
      float percent = ((float)(chunk + 1) / totalChunks) * 100.0;
      uint32_t bytes_sent = (chunk + 1) * CHUNK_SIZE;
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
  
  // ULTRA-FAST startup - minimal delay for maximum power efficiency
  Serial.println("Instant I2C bus initialization...");
  
  Serial.printf("Using ESP32 I2C master pins SDA=%d, SCL=%d\n", I2C_SDA_PIN, I2C_SCL_PIN);
  
  // Initialize I2C master with specified pins
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000); // Reduced to 400kHz for more reliable communication (was 800kHz)
  Wire.setTimeout(2000); // Longer timeout for more reliable error detection

  Serial.println("✓ I2C master initialization complete");
  Serial.printf("RELIABLE config: SDA=%d, SCL=%d, Clock=400kHz, Timeout=2s\n", 
                I2C_SDA_PIN, I2C_SCL_PIN);
  Serial.printf("Communicating with Pi Pico slave at address 0x%02X\n", PI_PICO_I2C_ADDRESS);
  Serial.println("Physical connections:");
  Serial.println("  ESP32 pin 21 (SDA) ↔ Pi Pico pin 4 (SDA)");
  Serial.println("  ESP32 pin 22 (SCL) ↔ Pi Pico pin 5 (SCL)");
  
  // Reset connection variables
  lastI2CActivity = 0;
  i2cTransactionCount = 0;
  errorCount = 0;
  currentChunk = 0;
  
  return true;
}

void setup() {
  Serial.begin(115200);
  
  Serial.println("=== ULTRA-OPTIMIZED ESP32 I2C IMAGE FETCHER ===");
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
  Serial.println("===================");
  
  // Initialize WiFi Manager with BALANCED timeouts for reliable connection
  Serial.println("Initializing WiFi Manager (BALANCED mode)...");
  wm.setBreakAfterConfig(true);
  wm.setConfigPortalTimeout(600); // 600s timeout for config portal (plenty of time to configure)
  wm.setConnectTimeout(30); // 30s connection timeout as requested - wait longer before AP mode
  wm.setConfigPortalBlocking(false); // Don't block if portal fails
  wm.setSaveConfigCallback([]() {
    Serial.println("WiFi config saved - immediate restart");
  });
  
  if (!wm.autoConnect("ESP32PhotoFrame", "photoframe123")) {
    Serial.println("WiFi connection failed after 30s - continuing with I2C only mode");
    Serial.println("Power-efficient operation: WiFi disabled to save battery");
  } else {
    Serial.println("✓ WiFi connected successfully");
    Serial.printf("IP address: %s\n", WiFi.localIP().toString().c_str());
    
    // BALANCED WiFi optimizations for reliable connection with good power efficiency
    WiFi.setSleep(WIFI_PS_NONE); // Power saving with reliable connection
    WiFi.setAutoReconnect(true); // Enable auto-reconnect for reliable connection
    WiFi.persistent(true); // Keep WiFi settings persistent across reboots
    Serial.println("✓ Balanced WiFi optimizations enabled");
  }
  
  Serial.println("=== ESP32 ULTRA-FAST I2C Master READY ===");
}

// Download and stream image directly to Pi Pico with MAXIMUM SPEED
bool downloadAndStreamImage(const char* url) {
  Serial.printf("Starting ULTRA-FAST streaming download from: %s\n", url);
  
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
      
      // Limit to reasonable size for e-paper display
      if (contentLength > 300000) { // 300KB limit
        Serial.printf("⚠ File too large (%d bytes), limiting to 300KB\n", contentLength);
        contentLength = 300000;
      }
      
      WiFiClient* stream = http.getStreamPtr();
      size_t bytesRead = 0;
      uint32_t chunk_id = 0;
      unsigned long start_time = millis(); // For transfer speed calculation
      
      Serial.printf("ULTRA-FAST streaming %d bytes in chunks...\n", contentLength);
      
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
              size_t chunk_size = min((size_t)CHUNK_SIZE, readSize - bytes_processed);
              
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
              Serial.printf("ULTRA-FAST stream: %d/%d bytes (%.1f%%, %.1f KB/s)\n", 
                           bytesRead, contentLength, percent, kbps);
            }
          
          }
        }
      }
      
      Serial.printf("✓ ULTRA-FAST streaming completed: %d bytes in %d chunks\n", bytesRead, chunk_id);
      http.end();
      
      // Send render command immediately
      if (!sendRenderCommand()) {
        return false;
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
  Serial.printf("Starting BMP download and streaming conversion from: %s\n", url);
  
  HTTPClient http;
  http.begin(url);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  
  // Add browser-like headers to help with complex URLs
  http.addHeader("User-Agent", "ESP32PhotoFrame/1.0");
  http.addHeader("Accept", "image/bmp,image/*,*/*");
  http.addHeader("Accept-Encoding", "identity"); // Disable compression for simplicity
  http.addHeader("Connection", "close");
  
  // Set BALANCED timeout for reliable downloads
  http.setTimeout(8000); // 8 seconds for reliable BMP downloads
  http.setConnectTimeout(2000); // 2s connection timeout for reliable connection
  http.setReuse(false); // Don't reuse connections for faster cleanup
  
  Serial.println("Making HTTP request with enhanced headers...");
  int httpCode = http.GET();
  
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
      
      WiFiClient* stream = http.getStreamPtr();
      
      // Read BMP headers first
      const size_t header_size = sizeof(BMPFILEHEADER) + sizeof(BMPINFOHEADER);
      uint8_t* header_buffer = (uint8_t*)malloc(header_size);
      
      if (!header_buffer) {
        Serial.printf("✗ Failed to allocate header buffer (%d bytes)\n", header_size);
        http.end();
        return false;
      }
      
      // Read headers with ULTRA-AGGRESSIVE buffering and timeout
      size_t header_read = 0;
      unsigned long start_time = millis();
      const unsigned long timeout = 2000; // 2 second timeout (was 3s) for maximum power efficiency
      
      Serial.printf("Reading BMP headers (%d bytes)...\n", header_size);
      
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
        
        // Read palette with ULTRA-FAST buffering
        size_t palette_read = 0;
        start_time = millis();
        
        Serial.printf("Reading BMP palette (%d bytes)...\n", palette_size);
        
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
        
        if (palette_read < palette_size) {
          Serial.printf("✗ Failed to read BMP palette (got %d of %d bytes after %lu ms)\n", 
                       palette_read, palette_size, millis() - start_time);
          free(palette_buffer);
          http.end();
          return false;
        }
        
        Serial.printf("✓ Successfully read BMP palette (%d bytes)\n", palette_read);
        
        // DEBUG: Print first 10 palette entries to verify colors
        Serial.println("=== PALETTE DEBUG (first 10 colors) ===");
        for (int i = 0; i < 10 && i < 256; i++) {
          uint32_t offset = i * 4;
          uint8_t b = palette_buffer[offset];
          uint8_t g = palette_buffer[offset + 1]; 
          uint8_t r = palette_buffer[offset + 2];
          uint8_t epaper_color = rgbToEpaperColor(r, g, b);
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
          uint8_t* skip_buffer = (uint8_t*)malloc(min(to_skip, (size_t)512)); // 512 byte chunks
          
          if (skip_buffer) {
            size_t skipped = stream->readBytes(skip_buffer, min(to_skip, (size_t)512));
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
      const uint32_t work_buffer_size = 119; // Max data per addressed I2C transaction
      uint8_t* work_buffer = (uint8_t*)malloc(work_buffer_size);
      
      if (!work_buffer) {
        Serial.printf("✗ Failed to allocate work buffer (%d bytes)\n", work_buffer_size);
        if (palette_buffer) {
          free(palette_buffer);
        }
        http.end();
        return false;
      }
      
      Serial.printf("✓ Allocated work buffer: %d bytes (streaming mode)\n", work_buffer_size);
      
      // Process BMP pixel data in display order for optimal I2C efficiency
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
      
        // SIMPLIFIED APPROACH: Stream BMP data in natural order and let renderer reorder
        // This is much simpler and more reliable than trying to reorder during streaming
        
        // Stream the pixel data sequentially, converting colors as we go
        // The renderer will receive a "raw BMP buffer" that it can reorder as needed
        uint32_t bytes_streamed = 0;
        uint32_t total_bmp_pixels = infoHeader.biWidth * infoHeader.biHeight;
        
        Serial.printf("Streaming BMP pixel data sequentially (%d pixels, %d bytes per row)...\n", 
                     total_bmp_pixels, bmp_row_total);
        
        // Process each BMP row (bottom-to-top as stored in BMP)
        for (int32_t bmp_row = 0; bmp_row < (int32_t)infoHeader.biHeight; bmp_row++) {
          // Read entire BMP row into buffer
          size_t row_read = 0;
          unsigned long row_start_time = millis();
          const unsigned long row_timeout = 1000; // 1s timeout (was 2s) for maximum power efficiency
          
          while (row_read < bmp_row_total && (millis() - row_start_time) < row_timeout) {
            if (stream->available() > 0) {
              size_t available = stream->available();
              size_t to_read = min(available, bmp_row_total - row_read);
              size_t read_size = stream->readBytes(&bmp_row_buffer[row_read], to_read);
              
              if (read_size > 0) {
                row_read += read_size;
              }
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
          
          // Convert this row's pixels to e-paper colors and stream sequentially
          for (uint32_t bmp_col = 0; bmp_col < infoHeader.biWidth; bmp_col += 2) {
            uint8_t pixel_left_color, pixel_right_color;
            
            // Extract left pixel
            if (infoHeader.biBitCount == 8) {
              uint8_t index = bmp_row_buffer[bmp_col];
              uint8_t r, g, b;
              paletteToRgb(index, palette_buffer, &r, &g, &b);
              pixel_left_color = rgbToEpaperColor(r, g, b);
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
            
            // Extract right pixel (if exists)
            if (bmp_col + 1 < infoHeader.biWidth) {
              if (infoHeader.biBitCount == 8) {
                uint8_t index = bmp_row_buffer[bmp_col + 1];
                uint8_t r, g, b;
                paletteToRgb(index, palette_buffer, &r, &g, &b);
                pixel_right_color = rgbToEpaperColor(r, g, b);
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
            
            // Send buffer when full
            if (work_buffer_pos >= work_buffer_size) {
              if (!sendImageChunkToAddress(bytes_streamed, work_buffer, work_buffer_pos)) {
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
              
              // Progress reporting
              if (bytes_streamed % 10000 == 0) {
                float percent = ((float)bytes_streamed / epaper_size) * 100.0;
                Serial.printf("BMP streaming: %d/%d bytes (%.1f%%)\n", 
                             bytes_streamed, epaper_size, percent);
              }
              
            }
          }
          
          // Progress reporting for row processing
          if (bmp_row % 50 == 0) {
            float percent = ((float)(bmp_row + 1) / infoHeader.biHeight) * 100.0;
            Serial.printf("BMP processing: row %d/%d (%.1f%%)\n", 
                         bmp_row + 1, infoHeader.biHeight, percent);
          }
        }
        
        Serial.printf("✓ BMP streaming completed: %d bytes streamed\n", bytes_streamed);
        
        free(bmp_row_buffer);
        
        // Send any remaining data in work buffer
        if (work_buffer_pos > 0) {
          if (!sendImageChunkToAddress(bytes_streamed, work_buffer, work_buffer_pos)) {
            Serial.printf("✗ Failed to send final BMP chunk at offset %d\n", bytes_streamed);
            free(work_buffer);
            if (palette_buffer) {
              free(palette_buffer);
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
        http.end();
        
        Serial.printf("✓ BMP streaming conversion completed: %d bytes processed\n", bytes_streamed);
        
        // Send BMP reorder and render command (renderer will reorder from BMP to display format)
        if (!sendBmpRenderCommand()) {
          return false;
        }
        
        Serial.println("✓ BMP streaming conversion and render command complete");
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
  return false;
}

void loop() {
  static unsigned long lastImageSend = 0;
  static bool i2cWorking = false;
  static unsigned long lastI2CTest = 0;
  unsigned long currentTime = millis();
  
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
  
  // Only proceed with image transfer if I2C is working
  // SINGLE-SHOT mode: Send image once, wait for completion, then shutdown for maximum power savings
  if (i2cWorking && lastImageSend == 0) { // Only run once for battery conservation
    Serial.println("=== SINGLE-SHOT POWER-EFFICIENT IMAGE TRANSFER ===");
    
    // Check slave status first
    uint8_t status = getSlaveStatus();
    Serial.printf("Pi Pico status: 0x%02X\n", status);
    
    if (status == STATUS_READY || status == STATUS_ERROR) {
      const char* imageUrl = "https://raw.githubusercontent.com/FloThinksPi/PhotoPainter-ESP32/refs/heads/main/ImageConverter/Examples/arabella.bmp";
      
      Serial.printf("POWER-EFFICIENT converting and streaming BMP from: %s\n", imageUrl);
      bool transfer_success = false;
      
      if (downloadAndConvertBmpImage(imageUrl)) {
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
      
      // Shut down ESP32 after successful completion to save maximum battery power
      if (transfer_success) {
        Serial.println("🔋 MAXIMUM POWER SAVINGS: Shutting down ESP32 after successful operation");
        shutdownESP32(); // This function never returns
      } else {
        Serial.println("⚠ Transfer had issues - staying awake for troubleshooting");
        // Mark as attempted to prevent continuous retries
        lastImageSend = currentTime;
      }
      
    } else {
      Serial.printf("Slave not ready (status: 0x%02X) - will retry in next cycle\n", status);
    }
  }
  
  // Initial I2C test on first run
  if (!i2cWorking && lastI2CTest == 0) {
    Serial.println("=== INITIAL I2C CONNECTION TEST ===");
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
    Serial.printf("POWER-EFFICIENT I2C: %lu transactions, %lu errors\n", 
                  i2cTransactionCount, errorCount);
    if (lastImageSend == 0) {
      Serial.println("STATUS: Waiting for I2C to become ready for single-shot transfer");
    } else {
      Serial.println("STATUS: Single-shot transfer attempted - staying awake for troubleshooting");
    }
    lastHeartbeat = currentTime;
  }

}
