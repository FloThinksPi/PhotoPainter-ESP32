#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Home Assistant Python Script for WhatsApp Image Dithering
This script processes images received via WhatsApp and converts them
to dithered BMP format suitable for e-paper displays.

Place this file in: /config/python_scripts/whatsapp_image_dithering.py
"""

import hashlib
import hmac
import base64
import requests
from PIL import Image, ImageEnhance
import io

# WhatsApp decryption imports
from Crypto.Cipher import AES

# E-paper 7-color palette (matching ESP32 code)
EPAPER_PALETTE = [
    (0, 0, 0),        # Black
    (255, 255, 255),  # White
    (0, 255, 0),      # Green
    (0, 0, 255),      # Blue
    (255, 0, 0),      # Red
    (255, 255, 0),    # Yellow
    (255, 165, 0),    # Orange (standard web color)
]

# WhatsApp decryption constants
WHATSAPP_MEDIA_KEYS = {
    "image": b"WhatsApp Image Keys",
    "video": b"WhatsApp Video Keys", 
    "audio": b"WhatsApp Audio Keys",
    "document": b"WhatsApp Document Keys",
    "image/webp": b"WhatsApp Image Keys",
    "image/jpeg": b"WhatsApp Image Keys",
    "image/png": b"WhatsApp Image Keys",
}

def hkdf_expand(key, length, app_info=b""):
    """HKDF key expansion for WhatsApp media decryption."""
    key = hmac.new(b"\0"*32, key, hashlib.sha256).digest()
    key_stream = b""
    key_block = b""
    block_index = 1
    while len(key_stream) < length:
        key_block = hmac.new(
            key,
            msg=key_block + app_info + (chr(block_index).encode("utf-8")),
            digestmod=hashlib.sha256
        ).digest()
        block_index += 1
        key_stream += key_block
    return key_stream[:length]

def aes_unpad(data):
    """Remove PKCS7 padding from decrypted data."""
    return data[:-ord(data[len(data)-1:])]

def aes_decrypt(key, ciphertext, iv):
    """AES CBC decryption."""
    cipher = AES.new(key, AES.MODE_CBC, iv)
    plaintext = cipher.decrypt(ciphertext)
    return aes_unpad(plaintext)

def decrypt_whatsapp_media(encrypted_data, media_key, media_type="image"):
    """Decrypt WhatsApp encrypted media file."""
    try:
        # Expand the media key
        expanded_key = hkdf_expand(media_key, 112, WHATSAPP_MEDIA_KEYS[media_type])
        
        # Extract components
        iv = expanded_key[:16]          # First 16 bytes: IV
        cipher_key = expanded_key[16:48]  # Next 32 bytes: AES key
        mac_key = expanded_key[48:80]     # Next 32 bytes: MAC key
        
        # Split encrypted data (last 10 bytes are MAC)
        file_data = encrypted_data[:-10]
        mac = encrypted_data[-10:]
        
        # Decrypt the data
        decrypted_data = aes_decrypt(cipher_key, file_data, iv)
        
        return decrypted_data
        
    except Exception as e:
        logger.error(f"Failed to decrypt WhatsApp media: {e}")
        return None

def create_full_palette(base_palette):
    """Create a full 256-color palette from base colors."""
    return base_palette + [(0, 0, 0)] * (256 - len(base_palette))

def resize_image(image, target_size=(800, 480), mode='crop', quality_filter=Image.LANCZOS):
    """Resize image according to specified mode."""
    target_width, target_height = target_size
    original_width, original_height = image.size
    
    if mode == 'stretch':
        # Simple stretch - may distort image
        return image.resize(target_size, quality_filter)
    
    elif mode == 'scale':
        # Scale to fit with padding (letterboxing/pillarboxing)
        scale_ratio = min(target_width / original_width, target_height / original_height)
        
        new_width = int(original_width * scale_ratio)
        new_height = int(original_height * scale_ratio)
        
        # Resize the image
        resized = image.resize((new_width, new_height), quality_filter)
        
        # Create white background and center the resized image
        result = Image.new('RGB', target_size, (255, 255, 255))
        x_offset = (target_width - new_width) // 2
        y_offset = (target_height - new_height) // 2
        result.paste(resized, (x_offset, y_offset))
        
        return result
    
    elif mode == 'crop':
        # Scale to fill and crop excess
        scale_ratio = max(target_width / original_width, target_height / original_height)
        
        new_width = int(original_width * scale_ratio)
        new_height = int(original_height * scale_ratio)
        
        # Resize the image
        resized = image.resize((new_width, new_height), quality_filter)
        
        # Calculate crop box to center the image
        x_offset = (new_width - target_width) // 2
        y_offset = (new_height - target_height) // 2
        
        crop_box = (x_offset, y_offset, x_offset + target_width, y_offset + target_height)
        return resized.crop(crop_box)
    
    else:
        raise ValueError(f"Unknown resize mode: {mode}")

def enhance_image_for_epaper(image, brightness=1.25, contrast=1.15, color_saturation=1.40):
    """Enhance image brightness, contrast, and color saturation for better e-paper results."""
    # Enhance brightness
    brightness_enhancer = ImageEnhance.Brightness(image)
    enhanced_image = brightness_enhancer.enhance(brightness)
    
    # Enhance contrast
    contrast_enhancer = ImageEnhance.Contrast(enhanced_image)
    enhanced_image = contrast_enhancer.enhance(contrast)
    
    # Enhance color saturation
    color_enhancer = ImageEnhance.Color(enhanced_image)
    enhanced_image = color_enhancer.enhance(color_saturation)
    
    return enhanced_image

def add_text_overlay(image, sender_name, display_date, message_text=""):
    """Add subtle text overlay positioned for bottom-center after 90° left rotation."""
    from PIL import ImageDraw, ImageFont
    import datetime
    
    # Create a copy of the image to avoid modifying the original
    overlay_image = image.copy()
    draw = ImageDraw.Draw(overlay_image)
    
    # Set default date if not provided
    if not display_date:
        display_date = datetime.datetime.now().strftime('%d.%m.%Y')
    
    # Use smaller, more subtle font
    try:
        font_size = max(10, image.width // 80)  # Smaller font
        font_paths = [
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
            "/System/Library/Fonts/Arial.ttf", 
            "/Windows/Fonts/arial.ttf",
            "arial.ttf"
        ]
        font = None
        for font_path in font_paths:
            try:
                font = ImageFont.truetype(font_path, font_size)
                break
            except:
                continue
        if font is None:
            font = ImageFont.load_default()
    except:
        font = ImageFont.load_default()

    # Prepare all text lines in one compact box
    text_lines = []
    text_lines.append(f"{sender_name} • {display_date}")
    
    # Add message text if provided (keep it short)
    if message_text and message_text.strip():
        # Truncate message if too long and add to same line
        max_msg_length = 40
        clean_message = message_text.strip()
        if len(clean_message) > max_msg_length:
            clean_message = clean_message[:max_msg_length-3] + "..."
        text_lines.append(f"💬 {clean_message}")
    
    # Calculate dimensions for the compact text box
    max_text_width = 0
    line_height = font_size + 2
    
    try:
        for line in text_lines:
            bbox = draw.textbbox((0, 0), line, font=font)
            line_width = bbox[2] - bbox[0]
            max_text_width = max(max_text_width, line_width)
    except:
        # Fallback for older PIL versions
        for line in text_lines:
            line_width, _ = draw.textsize(line, font=font)
            max_text_width = max(max_text_width, line_width)
    
    # Add padding
    padding = 8
    box_width = max_text_width + (padding * 2)
    box_height = len(text_lines) * line_height + (padding * 2)
    
    # Position at bottom-center (this will be right-center after 90° left rotation)
    x = (image.width - box_width) // 2  # Center horizontally
    y = image.height - box_height - 10  # Bottom with small margin
    
    # Draw semi-transparent background (more subtle)
    bg_rect = [x, y, x + box_width, y + box_height]
    draw.rectangle(bg_rect, fill=(255, 255, 255, 200))  # Semi-transparent white
    draw.rectangle(bg_rect, outline=(128, 128, 128), width=1)  # Light gray border
    
    # Draw text lines
    for i, line in enumerate(text_lines):
        text_y = y + padding + (i * line_height)
        text_x = x + padding
        draw.text((text_x, text_y), line, fill=(64, 64, 64), font=font)  # Dark gray text
    
    return overlay_image

def apply_epaper_palette(image, dither_mode='enhanced', brightness=1.25, contrast=1.15, saturation=1.40):
    """Apply e-paper 7-color palette with enhanced image processing."""
    
    # Create full 256-color palette
    full_palette = create_full_palette(EPAPER_PALETTE)
    
    # Create palette image
    palette_image = Image.new("P", (1, 1))
    
    # Flatten the palette for PIL
    flat_palette = []
    for color in full_palette:
        flat_palette.extend(color)
    
    palette_image.putpalette(flat_palette)
    
    # Handle different dither modes
    if dither_mode == 'none':
        dither_setting = Image.NONE
    elif dither_mode == 'enhanced':
        # Enhanced mode: brighten, increase contrast and color saturation before dithering
        image = enhance_image_for_epaper(image, brightness=brightness, contrast=contrast, color_saturation=saturation)
        dither_setting = Image.FLOYDSTEINBERG
    else:  # floyd (standard)
        dither_setting = Image.FLOYDSTEINBERG
    
    # Apply quantization with palette
    quantized = image.quantize(dither=dither_setting, palette=palette_image)
    
    # Return as indexed color image (P mode) instead of RGB
    return quantized

def generate_phone_hash(phone_number, salt):
    """Generate a secure hash of the phone number with salt."""
    combined = f"{phone_number}{salt}"
    return hashlib.sha256(combined.encode()).hexdigest()[:16]  # Use first 16 characters

def download_and_decrypt_image(url, media_key_b64, timeout=30):
    """Download encrypted WhatsApp image and decrypt it."""
    try:
        # Download the encrypted file
        response = requests.get(url, timeout=timeout, stream=True)
        response.raise_for_status()
        
        # Read the encrypted data
        encrypted_data = response.content
        
        logger.info(f"Downloaded encrypted file: {len(encrypted_data)} bytes")
        
        # Decode the media key from base64
        media_key = base64.b64decode(media_key_b64)
        logger.info(f"Media key decoded: {len(media_key)} bytes")
        
        # Decrypt the data
        decrypted_data = decrypt_whatsapp_media(encrypted_data, media_key, "image")
        
        if decrypted_data is None:
            logger.error("Failed to decrypt image data")
            return None
            
        logger.info(f"Decrypted file: {len(decrypted_data)} bytes")
        
        # Use BytesIO instead of temporary files (Home Assistant compatible)
        image_bytes = io.BytesIO(decrypted_data)
        
        # Open the decrypted image with PIL directly from bytes
        with Image.open(image_bytes) as img:
            # Convert to RGB if needed
            if img.mode != 'RGB':
                img = img.convert('RGB')
            # Make a copy to avoid issues with the BytesIO stream
            image_copy = img.copy()
        
        return image_copy
        
    except requests.exceptions.RequestException as e:
        logger.error(f"Failed to download encrypted image from {url}: {e}")
        return None
    except Exception as e:
        logger.error(f"Failed to process encrypted image: {e}")
        return None

def process_whatsapp_image(phone_number, image_url, media_key_b64, phone_salt, sender_name="", display_date=None, message_text=""):
    """Main function to process WhatsApp image and save as dithered BMP."""
    
    try:
        # Log the processing start
        logger.info(f"Starting image processing for {sender_name} ({phone_number})")
        
        # Download and decrypt the image
        logger.info(f"Downloading and decrypting image from: {image_url}")
        image = download_and_decrypt_image(image_url, media_key_b64)
        
        if image is None:
            logger.error("Failed to download and decrypt image")
            return False
        
        logger.info(f"Image processed successfully: {image.size[0]}x{image.size[1]} pixels, mode: {image.mode}")
        
        # Auto-rotate portrait images to landscape for 800x480 display
        if image.size[1] > image.size[0]:  # Portrait orientation
            logger.info(f"Auto-rotating portrait image to landscape")
            image = image.rotate(90, expand=True)
            logger.info(f"Image rotated: {image.size[0]}x{image.size[1]} pixels")
        
        # Resize image to 800x480
        target_size = (800, 480)
        resized_image = resize_image(image, target_size, mode='crop')
        logger.info(f"Image resized to: {resized_image.size[0]}x{resized_image.size[1]} pixels")
        
        # Add text overlay with sender name, date and message
        text_image = add_text_overlay(resized_image, sender_name, display_date, message_text)
        logger.info(f"Text overlay added for {sender_name}" + (f" with message: '{message_text[:30]}...'" if message_text else ""))
        
        # Apply e-paper palette with enhanced dithering
        final_image = apply_epaper_palette(text_image, dither_mode='enhanced')
        logger.info(f"E-paper palette applied, final mode: {final_image.mode}")
        
        # Generate filename hash
        phone_hash = generate_phone_hash(phone_number, phone_salt)
        output_filename = f"{phone_hash}.bmp"
        
        # Create output path (Home Assistant compatible)
        output_dir = "/config/www/PhotoPainter"
        output_path = f"{output_dir}/{output_filename}"
        
        # Save as BMP
        final_image.save(output_path, format='BMP')
        
        logger.info(f"Image processed successfully!")
        logger.info(f"Saved to: {output_path}")
        logger.info(f"Phone hash: {phone_hash}")
        
        # Optional: Log sender name if provided
        if sender_name.strip():
            logger.info(f"Sender: {sender_name}")
        
        return True
        
    except Exception as e:
        logger.error(f"Error processing WhatsApp image: {e}")
        import traceback
        logger.error(f"Traceback: {traceback.format_exc()}")
        return False

# Home Assistant Python Script Entry Point
if __name__ == "__main__":
    # Command-line execution for debugging
    import argparse
    import sys
    import os
    import tempfile
    from pathlib import Path
    
    print("🔧 WhatsApp Image Dithering - Debug Mode")
    
    parser = argparse.ArgumentParser(description='Debug WhatsApp image processing')
    parser.add_argument('--phone', required=True, help='Phone number')
    parser.add_argument('--url', required=True, help='Image URL')
    parser.add_argument('--key', required=True, help='Media key (base64)')
    parser.add_argument('--salt', default='debug_salt', help='Phone salt')
    parser.add_argument('--name', default='Debug User', help='Sender name')
    parser.add_argument('--date', default=None, help='Display date (DD.MM.YYYY format)')
    parser.add_argument('--text', default='', help='Additional text to display on image')
    
    args = parser.parse_args()
    
    # Create a simple logger for debugging
    class DebugLogger:
        def info(self, msg): print(f"ℹ️  {msg}")
        def error(self, msg): print(f"❌ {msg}")
    
    # Mock the Home Assistant logger
    logger = DebugLogger()
    
    # Override file handling functions for command-line mode
    def download_and_decrypt_image_debug(url, media_key_b64, timeout=30):
        """Debug version that can use tempfiles."""
        try:
            # Download the encrypted file
            response = requests.get(url, timeout=timeout, stream=True)
            response.raise_for_status()
            
            # Read the encrypted data
            encrypted_data = response.content
            
            logger.info(f"Downloaded encrypted file: {len(encrypted_data)} bytes")
            
            # Decode the media key from base64
            media_key = base64.b64decode(media_key_b64)
            logger.info(f"Media key decoded: {len(media_key)} bytes")
            
            # Decrypt the data
            decrypted_data = decrypt_whatsapp_media(encrypted_data, media_key, "image")
            
            if decrypted_data is None:
                logger.error("Failed to decrypt image data")
                return None
                
            logger.info(f"Decrypted file: {len(decrypted_data)} bytes")
            
            # Create a temporary file for the decrypted image
            with tempfile.NamedTemporaryFile(delete=False, suffix='.jpg') as temp_file:
                temp_file.write(decrypted_data)
                temp_file_path = temp_file.name
            
            # Open the decrypted image with PIL
            with Image.open(temp_file_path) as img:
                # Convert to RGB if needed
                if img.mode != 'RGB':
                    img = img.convert('RGB')
                # Make a copy to avoid issues with the temporary file
                image_copy = img.copy()
            
            # Clean up temporary file
            os.unlink(temp_file_path)
            
            return image_copy
            
        except requests.exceptions.RequestException as e:
            logger.error(f"Failed to download encrypted image from {url}: {e}")
            return None
        except Exception as e:
            logger.error(f"Failed to process encrypted image: {e}")
            return None
    
    def process_whatsapp_image_debug(phone_number, image_url, media_key_b64, phone_salt, sender_name="", display_date=None, message_text=""):
        """Debug version that can use Path and file operations."""
        try:
            # Log the processing start
            logger.info(f"Starting image processing for {sender_name} ({phone_number})")
            
            # Download and decrypt the image
            logger.info(f"Downloading and decrypting image from: {image_url}")
            image = download_and_decrypt_image_debug(image_url, media_key_b64)
            
            if image is None:
                logger.error("Failed to download and decrypt image")
                return False
            
            logger.info(f"Image processed successfully: {image.size[0]}x{image.size[1]} pixels, mode: {image.mode}")
            
            # Auto-rotate portrait images to landscape for 800x480 display
            if image.size[1] > image.size[0]:  # Portrait orientation
                logger.info(f"Auto-rotating portrait image to landscape")
                image = image.rotate(90, expand=True)
                logger.info(f"Image rotated: {image.size[0]}x{image.size[1]} pixels")
            
            # Resize image to 800x480
            target_size = (800, 480)
            resized_image = resize_image(image, target_size, mode='crop')
            logger.info(f"Image resized to: {resized_image.size[0]}x{resized_image.size[1]} pixels")
            
            # Add text overlay with sender name, date and message
            text_image = add_text_overlay(resized_image, sender_name, display_date, message_text)
            logger.info(f"Text overlay added for {sender_name}" + (f" with message: '{message_text[:30]}...'" if message_text else ""))
            
            # Apply e-paper palette with enhanced dithering
            final_image = apply_epaper_palette(text_image, dither_mode='enhanced')
            logger.info(f"E-paper palette applied, final mode: {final_image.mode}")
            
            # Generate filename hash
            phone_hash = generate_phone_hash(phone_number, phone_salt)
            output_filename = f"{phone_hash}.bmp"
            
            # Ensure output directory exists
            output_dir = Path("/config/www/PhotoPainter")
            output_dir.mkdir(parents=True, exist_ok=True)
            
            # Full output path
            output_path = output_dir / output_filename
            
            # Save as BMP
            final_image.save(str(output_path), format='BMP')
            
            # Get file size
            file_size = output_path.stat().st_size
            
            logger.info(f"Image processed successfully!")
            logger.info(f"Saved to: {output_path}")
            logger.info(f"File size: {file_size:,} bytes ({file_size / 1024:.1f} KB)")
            logger.info(f"Phone hash: {phone_hash}")
            
            # Optional: Log sender name if provided
            if sender_name.strip():
                logger.info(f"Sender: {sender_name}")
            
            return True
            
        except Exception as e:
            logger.error(f"Error processing WhatsApp image: {e}")
            import traceback
            logger.error(f"Traceback: {traceback.format_exc()}")
            return False
    
    # Run the debug processing function
    success = process_whatsapp_image_debug(
        phone_number=args.phone,
        image_url=args.url,
        media_key_b64=args.key,
        phone_salt=args.salt,
        sender_name=args.name,
        display_date=args.date,
        message_text=args.text
    )
    
    if success:
        print("✅ Processing completed successfully!")
        sys.exit(0)
    else:
        print("❌ Processing failed!")
        sys.exit(1)
        
else:
    # Home Assistant execution - Enhanced logging
    try:
        logger.info("🔧 WhatsApp Image Dithering - Home Assistant Mode Starting")
        
        # Log all available data for debugging
        logger.info(f"Available data keys: {list(data.keys()) if 'data' in locals() else 'No data available'}")
        
        phone_number = data.get('phone_number')
        image_url = data.get('image_url')
        media_key_b64 = data.get('media_key')
        phone_salt = data.get('phone_salt')
        sender_name = data.get('sender_name', '')
        
        logger.info(f"Parameters - Phone: {phone_number}, URL: {bool(image_url)}, Key: {bool(media_key_b64)}, Salt: {phone_salt}, Name: {sender_name}")

        if not phone_number or not image_url or not media_key_b64 or not phone_salt:
            logger.error(f"❌ Missing required parameters:")
            logger.error(f"  - phone_number: {phone_number}")
            logger.error(f"  - image_url: {'Present' if image_url else 'Missing'}")
            logger.error(f"  - media_key: {'Present' if media_key_b64 else 'Missing'}")
            logger.error(f"  - phone_salt: {phone_salt}")
        else:
            # Test imports first
            try:
                from Crypto.Cipher import AES
                logger.info("✅ pycryptodome import successful")
            except ImportError as e:
                logger.error(f"❌ pycryptodome import failed: {e}")
                logger.error("Please install: pip install pycryptodome")
                raise
            
            try:
                from PIL import Image, ImageEnhance
                logger.info("✅ PIL import successful")
            except ImportError as e:
                logger.error(f"❌ PIL import failed: {e}")
                logger.error("Please install: pip install Pillow")
                raise
            
            try:
                import requests
                logger.info("✅ requests import successful")
            except ImportError as e:
                logger.error(f"❌ requests import failed: {e}")
                logger.error("Please install: pip install requests")
                raise
            
            logger.info("📦 All imports successful, starting image processing...")
            success = process_whatsapp_image(phone_number, image_url, media_key_b64, phone_salt, sender_name)
            if success:
                logger.info("✅ WhatsApp image processing completed successfully")
            else:
                logger.error("❌ WhatsApp image processing failed")
                
    except Exception as e:
        logger.error(f"❌ Critical error in Home Assistant execution: {e}")
        import traceback
        logger.error(f"Traceback: {traceback.format_exc()}")
