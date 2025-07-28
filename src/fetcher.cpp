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

static constexpr size_t BUFFER_SIZE = CHUNK_SIZE;
uint8_t i2c_buffer[BUFFER_SIZE] = { 0 };
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
  Serial.printf("Clock: 600kHz\n");
  Serial.printf("Image size: %d bytes\n", sizeof(Image7color));
  Serial.printf("Total chunks: %d\n", totalChunks);
  Serial.println("=====================================");
}

// Send a chunk of data to Pi Pico slave
bool sendImageChunk(uint32_t chunk_id, const uint8_t* data, size_t data_size) {
  // I2C buffer limit: 128 bytes buffer - 5 bytes header = 123 bytes max data
  if (data_size > 123) data_size = 123; // Max data size to fit in I2C transaction
  
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
    Serial.printf("✗ Buffer overflow: only %d of %d bytes queued for chunk %d\n", 
                  written, data_size, chunk_id);
    Wire.endTransmission(); // Clean up
    errorCount++;
    return false;
  }
  
  uint8_t error = Wire.endTransmission();
  
  if (error == 0) {
    lastI2CActivity = millis();
    i2cTransactionCount++;
    return true;
  } else {
    Serial.printf("✗ Failed to send chunk %d (error: %d)\n", chunk_id, error);
    errorCount++;
    return false;
  }
}

// Send a chunk of data to specific address offset in Pi Pico slave memory
bool sendImageChunkToAddress(uint32_t address_offset, const uint8_t* data, size_t data_size) {
  // I2C buffer limit: 128 bytes buffer - 9 bytes header = 119 bytes max data
  if (data_size > 119) data_size = 119; // Max data size to fit in I2C transaction with address
  
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
    Serial.printf("✗ Buffer overflow: only %d of %d bytes queued for address 0x%08X\n", 
                  written, data_size, address_offset);
    Wire.endTransmission(); // Clean up
    errorCount++;
    return false;
  }
  
  uint8_t error = Wire.endTransmission();
  
  if (error == 0) {
    lastI2CActivity = millis();
    i2cTransactionCount++;
    return true;
  } else {
    Serial.printf("✗ Failed to send chunk to address 0x%08X (error: %d)\n", address_offset, error);
    errorCount++;
    return false;
  }
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
  
  // Send all chunks
  for (uint32_t chunk = 0; chunk < totalChunks; chunk++) {
    uint32_t offset = chunk * CHUNK_SIZE;
    size_t remaining = image_size - offset;
    size_t chunk_size = (remaining > CHUNK_SIZE) ? CHUNK_SIZE : remaining;
    
    if (!sendImageChunk(chunk, &image_data[offset], chunk_size)) {
      Serial.printf("✗ Failed to send chunk %d\n", chunk);
      return false;
    }
    
    // Small delay between chunks 
    delay(2); // Minimal delay for fast transfers
    
    // Enhanced progress reporting - every 50 chunks or at completion
    if (chunk % 50 == 0 || chunk == totalChunks - 1) {
      float percent = ((float)(chunk + 1) / totalChunks) * 100.0;
      uint32_t bytes_sent = (chunk + 1) * CHUNK_SIZE;
      if (bytes_sent > image_size) bytes_sent = image_size;
      
      Serial.printf("📊 Progress: %d/%d chunks (%.2f%%) - %d/%d bytes sent\n", 
                    chunk + 1, totalChunks, percent,
                    bytes_sent, image_size);
    
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
  
  // Add delay to ensure Pi Pico slave is ready
  Serial.println("Waiting for I2C bus to stabilize...");
  delay(1000); // Reduced from 2000ms to 1000ms
  
  Serial.printf("Using ESP32 I2C master pins SDA=%d, SCL=%d\n", I2C_SDA_PIN, I2C_SCL_PIN);
  
  // Initialize I2C master with specified pins
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(600000); // Back to 600kHz - this was working correctly

  Serial.println("✓ I2C master initialization complete");
  Serial.printf("Active configuration: SDA=%d, SCL=%d, Clock=600kHz\n", 
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
  delay(1000);
  
  Serial.println("=== ESP32 I2C IMAGE FETCHER STARTING ===");
  Serial.printf("Free heap: available\n");
  Serial.printf("Image data size: %d bytes\n", sizeof(Image7color));
  
  // Initialize I2C communication
  if (!initI2C()) {
    Serial.println("FATAL: I2C initialization failed!");
    while (1) {
      delay(1000);
      Serial.println("I2C init failed, system halted");
    }
  }
  
  // Initialize WiFi Manager for potential future features
  Serial.println("Initializing WiFi Manager...");
  wm.setBreakAfterConfig(true);
  
  if (!wm.autoConnect("ESP32PhotoFrame", "photoframe123")) {
    Serial.println("WiFi connection failed, continuing with I2C only");
  } else {
    Serial.println("✓ WiFi connected");
    Serial.printf("IP address: %s\n", WiFi.localIP().toString().c_str());
  }
  
  Serial.println("=== ESP32 I2C Master READY ===");
}

// Download and stream image directly to Pi Pico
bool downloadAndStreamImage(const char* url) {
  Serial.printf("Starting streaming download from: %s\n", url);
  
  HTTPClient http;
  http.begin(url);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  
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
      uint8_t chunk_buffer[CHUNK_SIZE];
      
      Serial.printf("Streaming %d bytes in chunks...\n", contentLength);
      
      while (http.connected() && bytesRead < contentLength) {
        size_t available = stream->available();
        if (available) {
          // Read up to one chunk size
          size_t toRead = min(available, (size_t)CHUNK_SIZE);
          toRead = min(toRead, (size_t)(contentLength - bytesRead));
          
          size_t readSize = stream->readBytes(chunk_buffer, toRead);
          
          if (readSize > 0) {
            // Send this chunk via I2C immediately
            if (!sendImageChunk(chunk_id, chunk_buffer, readSize)) {
              Serial.printf("✗ Failed to send streaming chunk %d\n", chunk_id);
              http.end();
              return false;
            }
            
            chunk_id++;
            bytesRead += readSize;
            
            // Progress reporting
            if (bytesRead % 5000 == 0) { // Every 5KB
              float percent = ((float)bytesRead / contentLength) * 100.0;
              Serial.printf("Streamed: %d/%d bytes (%.1f%%)\n", 
                           bytesRead, contentLength, percent);
            }
            
            // Small delay for I2C stability
            delay(2);
          }
        } else {
          delay(1);
        }
      }
      
      Serial.printf("✓ Streaming completed: %d bytes in %d chunks\n", bytesRead, chunk_id);
      http.end();
      
      // Send render command
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
  
  // Set longer timeout for complex decryption URLs
  http.setTimeout(30000); // 30 seconds
  
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
      
      // Small delay to ensure stream is properly established
      delay(100);
      
      WiFiClient* stream = http.getStreamPtr();
      
      // Read BMP headers first
      const size_t header_size = sizeof(BMPFILEHEADER) + sizeof(BMPINFOHEADER);
      uint8_t* header_buffer = (uint8_t*)malloc(header_size);
      
      if (!header_buffer) {
        Serial.printf("✗ Failed to allocate header buffer (%d bytes)\n", header_size);
        http.end();
        return false;
      }
      
      // Read headers with proper buffering and timeout
      size_t header_read = 0;
      unsigned long start_time = millis();
      const unsigned long timeout = 10000; // 10 second timeout
      
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
        } else {
          delay(10); // Wait for more data
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
        
        // Read palette with proper buffering
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
          } else {
            delay(10); // Wait for more data
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
            } else {
              delay(1);
            }
          }
        } else {
          delay(10); // Wait for more data
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
          const unsigned long row_timeout = 5000;
          
          while (row_read < bmp_row_total && (millis() - row_start_time) < row_timeout) {
            if (stream->available() > 0) {
              size_t available = stream->available();
              size_t to_read = min(available, bmp_row_total - row_read);
              size_t read_size = stream->readBytes(&bmp_row_buffer[row_read], to_read);
              
              if (read_size > 0) {
                row_read += read_size;
              }
            } else {
              delay(1);
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
              
              delay(1);
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
  unsigned long currentTime = millis();
  
  // Send image immediately on first run, then every 30 seconds
  if (lastImageSend == 0 || (currentTime - lastImageSend > 30000)) {
    Serial.println("=== Starting new image transfer cycle ===");
    
    // Check slave status first
    uint8_t status = getSlaveStatus();
    Serial.printf("Pi Pico status: 0x%02X\n", status);
    
    if (status == STATUS_READY || status == STATUS_ERROR) {
      const char* imageUrl = "https://raw.githubusercontent.com/FloThinksPi/PhotoPainter-ESP32/refs/heads/main/ImageConverter/Examples/pattern_indexed.bmp";
      
      Serial.printf("Converting and streaming BMP from: %s\n", imageUrl);
      if (downloadAndConvertBmpImage(imageUrl)) {
        Serial.println("✓ BMP conversion and rendering completed successfully");
      } else {
        Serial.println("⚠ BMP conversion failed, sending static image as fallback");
        
        // Fallback to static image
        if (sendCompleteImage()) {
          Serial.println("✓ Fallback image transfer completed successfully");
        } else {
          Serial.println("✗ Even fallback image transfer failed");
        }
      }
    } else {
      Serial.printf("Slave not ready (status: 0x%02X) - skipping this cycle\n", status);
    }
    
    lastImageSend = currentTime;
  }
  
  // Check connection health periodically
  static unsigned long lastHealthCheck = 0;
  if (currentTime - lastHealthCheck > 5000) { // Every 5 seconds
    checkI2CConnectionHealth();
    lastHealthCheck = currentTime;
  }
  
  // Simple heartbeat (reduced frequency)
  static unsigned long lastHeartbeat = 0;
  if (currentTime - lastHeartbeat > 30000) { // Every 30 seconds (was 10)
    Serial.printf("I2C Heartbeat: %lu transactions, %lu errors\n", 
                  i2cTransactionCount, errorCount);
    lastHeartbeat = currentTime;
  }
  
  delay(100); // Small delay to prevent busy loop
}
