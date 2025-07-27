#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2025 Florian Braun (FloThinksPi)
#
"""
ESP32 Photo Frame Image Converter
Converts images (JPEG, PNG, BMP, HEIF/HEIC, etc.) to 480x800 BMP format
with 7-color e-paper display palette and dithering.
"""

import sys
import os
import os.path
from PIL import Image
import argparse
from pathlib import Path

# Import Apple HEIF support for iPhone photos
try:
    from pillow_heif import register_heif_opener
    register_heif_opener()
    HEIF_SUPPORT = True
    print("✅ Apple HEIF/HEIC support enabled")
except ImportError:
    HEIF_SUPPORT = False
    print("⚠️  Apple HEIF/HEIC support not available (install pillow-heif)")

# Import AVIF support for modern web images
try:
    import pillow_avif
    AVIF_SUPPORT = True
    print("✅ AVIF support enabled")
except ImportError:
    AVIF_SUPPORT = False
    print("⚠️  AVIF support not available (install pillow-avif-plugin)")

# E-paper 7-color palette (matching ESP32 code)
EPAPER_PALETTE = [
    (0, 0, 0),        # Black
    (255, 255, 255),  # White
    (0, 255, 0),      # Green
    (0, 0, 255),      # Blue
    (255, 0, 0),      # Red
    (255, 255, 0),    # Yellow
    (255, 128, 0),    # Orange
]

# Extend palette to 256 colors (PIL requirement)
FULL_PALETTE = EPAPER_PALETTE + [(0, 0, 0)] * (256 - len(EPAPER_PALETTE))

def create_argument_parser():
    """Create and configure the argument parser."""
    supported_formats = "JPEG, PNG, BMP, TIFF, WEBP"
    if HEIF_SUPPORT:
        supported_formats += ", HEIF/HEIC (iPhone photos)"
    if AVIF_SUPPORT:
        supported_formats += ", AVIF (modern web format)"
    
    parser = argparse.ArgumentParser(
        description='Convert images to 480x800 BMP format for ESP32 e-paper display',
        epilog=f'Supported formats: {supported_formats}'
    )
    
    parser.add_argument(
        'image_file', 
        type=str, 
        help='Input image file path'
    )
    
    parser.add_argument(
        '--orientation', '--dir',
        choices=['landscape', 'portrait', 'auto'], 
        default='auto',
        help='Target orientation (output is always 480x800 portrait)'
    )
    
    parser.add_argument(
        '--mode', 
        choices=['scale', 'crop', 'stretch'], 
        default='scale',
        help='Resize mode: scale (fit with padding), crop (fill and crop), stretch (distort to fit)'
    )
    
    parser.add_argument(
        '--dither', 
        choices=['none', 'floyd'], 
        default='floyd',
        help='Dithering algorithm: none or floyd-steinberg'
    )
    
    parser.add_argument(
        '--format',
        choices=['indexed', 'rgb'],
        default='indexed',
        help='Output format: indexed (8-bit, smaller file) or rgb (24-bit, compatible)'
    )
    
    parser.add_argument(
        '--output', '-o',
        type=str,
        help='Output file path (default: input_name_converted.bmp)'
    )
    
    parser.add_argument(
        '--quality',
        choices=['fast', 'good', 'best'],
        default='good',
        help='Resize quality: fast (nearest), good (bilinear), best (lanczos)'
    )
    
    parser.add_argument(
        '--preview',
        action='store_true',
        help='Show preview of converted image'
    )
    
    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='Verbose output'
    )
    
    return parser

def validate_input_file(filepath):
    """Validate input file exists and is readable."""
    if not os.path.isfile(filepath):
        print(f'❌ Error: File "{filepath}" does not exist')
        sys.exit(1)
    
    # Check if it's a HEIF/HEIC file without support
    file_ext = Path(filepath).suffix.lower()
    if file_ext in ['.heif', '.heic'] and not HEIF_SUPPORT:
        print(f'❌ Error: HEIF/HEIC file detected but support not installed')
        print('   Install Apple photo support with: pip install pillow-heif')
        sys.exit(1)
    
    # Check if it's an AVIF file without support
    if file_ext in ['.avif'] and not AVIF_SUPPORT:
        print(f'❌ Error: AVIF file detected but support not installed')
        print('   Install AVIF support with: pip install pillow-avif-plugin')
        sys.exit(1)
    
    try:
        with Image.open(filepath) as img:
            img.verify()
    except Exception as e:
        if file_ext in ['.heif', '.heic']:
            print(f'❌ Error: Cannot read HEIF/HEIC file "{filepath}"')
            print('   Make sure pillow-heif is installed: pip install pillow-heif')
        elif file_ext in ['.avif']:
            print(f'❌ Error: Cannot read AVIF file "{filepath}"')
            print('   Make sure pillow-avif-plugin is installed: pip install pillow-avif-plugin')
        else:
            print(f'❌ Error: Cannot read image file "{filepath}": {e}')
        sys.exit(1)
    
    return True

def determine_target_size(input_size, orientation):
    """Determine target dimensions - always outputs 480x800 (portrait) for ESP32 display."""
    # ESP32 e-paper display is always 480x800 (portrait orientation)
    # Images will be automatically rotated if needed to fit this format
    return 480, 800

def get_resize_filter(quality):
    """Get PIL resize filter based on quality setting."""
    filters = {
        'fast': Image.NEAREST,
        'good': Image.BILINEAR,
        'best': Image.LANCZOS
    }
    return filters.get(quality, Image.BILINEAR)

def resize_image(image, target_size, mode, quality_filter):
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

def apply_epaper_palette(image, use_dither):
    """Apply e-paper 7-color palette with optional dithering."""
    # Create palette image
    palette_image = Image.new("P", (1, 1))
    
    # Flatten the palette for PIL
    flat_palette = []
    for color in FULL_PALETTE:
        flat_palette.extend(color)
    
    palette_image.putpalette(flat_palette)
    
    # Convert dither setting
    dither_mode = Image.FLOYDSTEINBERG if use_dither else Image.NONE
    
    # Apply quantization with palette
    quantized = image.quantize(dither=dither_mode, palette=palette_image)
    
    # Return as indexed color image (P mode) instead of RGB
    # This will create much smaller BMP files (8-bit per pixel instead of 24-bit)
    return quantized

def generate_output_filename(input_path, output_path, mode):
    """Generate output filename if not specified."""
    if output_path:
        return output_path
    
    input_path = Path(input_path)
    stem = input_path.stem
    return str(input_path.parent / f"{stem}_{mode}_epaper.bmp")

def print_image_info(image, label, verbose=False):
    """Print image information."""
    if verbose:
        print(f"📊 {label}: {image.size[0]}x{image.size[1]} pixels, mode: {image.mode}")

def main():
    """Main conversion function."""
    parser = create_argument_parser()
    args = parser.parse_args()
    
    # Validate input
    validate_input_file(args.image_file)
    
    if args.verbose:
        print(f"🔄 Converting: {args.image_file}")
        file_ext = Path(args.image_file).suffix.lower()
        if file_ext in ['.heif', '.heic']:
            print("📱 Detected Apple HEIF/HEIC format (iPhone photo)")
        elif file_ext in ['.avif']:
            print("🌐 Detected AVIF format (modern web image)")
    
    try:
        # Open and convert to RGB
        with Image.open(args.image_file) as input_image:
            if input_image.mode != 'RGB':
                input_image = input_image.convert('RGB')
            
            print_image_info(input_image, "Input image", args.verbose)
            
            # Auto-rotate landscape images to portrait for 480x800 display
            # If width > height, rotate 90 degrees to make height the longer dimension
            if input_image.size[0] > input_image.size[1]:  # Landscape orientation
                if args.verbose:
                    print(f"🔄 Auto-rotating landscape image ({input_image.size[0]}x{input_image.size[1]} → portrait)")
                input_image = input_image.rotate(90, expand=True)  # Rotate 90° counter-clockwise
                print_image_info(input_image, "Rotated image", args.verbose)
            
            # Target is always 480x800 for ESP32 display
            target_size = determine_target_size(input_image.size, args.orientation)
            
            if args.verbose:
                print(f"🎯 Target size: {target_size[0]}x{target_size[1]} ({args.orientation})")
                print(f"🔧 Resize mode: {args.mode}")
                print(f"🎨 Dithering: {'Floyd-Steinberg' if args.dither == 'floyd' else 'None'}")
                print(f"📦 Format: {'8-bit indexed' if args.format == 'indexed' else '24-bit RGB'}")
            
            # Resize image
            quality_filter = get_resize_filter(args.quality)
            resized_image = resize_image(input_image, target_size, args.mode, quality_filter)
            print_image_info(resized_image, "Resized image", args.verbose)
            
            # Apply e-paper palette
            use_dither = (args.dither == 'floyd')
            final_image = apply_epaper_palette(resized_image, use_dither)
            
            # Convert to RGB if requested (for compatibility)
            if args.format == 'rgb':
                final_image = final_image.convert('RGB')
                if args.verbose:
                    print("🔄 Converted to RGB for compatibility")
            
            print_image_info(final_image, "Final image", args.verbose)
            
            # Generate output filename
            output_path = generate_output_filename(args.image_file, args.output, args.mode)
            
            # Save as BMP
            final_image.save(output_path, format='BMP')
            
            # Get file size
            file_size = os.path.getsize(output_path)
            
            # Calculate theoretical 24-bit size for comparison
            width, height = final_image.size
            theoretical_24bit_size = 54 + (width * height * 3)  # BMP header + 24-bit pixel data
            
            print(f"✅ Successfully converted to: {output_path}")
            print(f"📁 File size: {file_size:,} bytes ({file_size / 1024:.1f} KB)")
            
            if args.format == 'indexed':
                savings = theoretical_24bit_size - file_size
                savings_percent = (savings / theoretical_24bit_size) * 100
                print(f"💾 Space saved: {savings:,} bytes ({savings_percent:.1f}%) vs 24-bit RGB")
                print(f"📊 Size comparison: 8-bit indexed vs 24-bit RGB = {file_size / 1024:.1f} KB vs {theoretical_24bit_size / 1024:.1f} KB")
            
            # Show preview if requested
            if args.preview:
                try:
                    final_image.show()
                except Exception as e:
                    if args.verbose:
                        print(f"⚠️  Could not show preview: {e}")
    
    except Exception as e:
        print(f"❌ Error during conversion: {e}")
        if args.verbose:
            import traceback
            traceback.print_exc()
        sys.exit(1)

if __name__ == '__main__':
    main()
