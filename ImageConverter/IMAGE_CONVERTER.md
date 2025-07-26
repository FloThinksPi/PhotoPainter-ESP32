# ESP32 Photo Frame Image Converter

This tool converts images to the optimal format for your ESP32 e-paper photo frame.

## Features

- **Multi-format support**: JPEG, PNG, BMP, TIFF, WEBP, HEIF/HEIC (iPhone photos), AVIF (modern web images)
- **Apple photo support**: Native HEIF/HEIC format from modern iPhones and iPads
- **Modern web format support**: AVIF format with superior compression and quality
- **Automatic sizing**: Converts to 800×480 for landscape or 480×800 for portrait
- **7-color palette**: Optimized for e-paper displays (Black, White, Red, Green, Blue, Yellow, Orange)
- **Smart resizing**: Scale (fit with padding), crop (fill and trim), or stretch modes
- **Dithering**: Floyd-Steinberg dithering for better color reproduction
- **Quality options**: Fast, good, or best resize algorithms

## Quick Start

```bash
# Basic conversion (auto-detects orientation)
python dither.py photo.jpg

# iPhone HEIC photo with landscape cropping
python dither.py IMG_1234.HEIC --orientation landscape --mode crop

# Modern AVIF web image
python dither.py modern_image.avif --orientation landscape --mode crop

# Portrait with no dithering
python dither.py image.jpeg --orientation portrait --dither none
```

## Command Line Options

| Option | Values | Description |
|--------|--------|-------------|
| `--orientation` | `landscape`, `portrait`, `auto` | Target orientation (default: auto) |
| `--mode` | `scale`, `crop`, `stretch` | Resize method (default: scale) |
| `--dither` | `floyd`, `none` | Dithering algorithm (default: floyd) |
| `--format` | `indexed`, `rgb` | Output format: indexed (8-bit, smaller) or rgb (24-bit, compatible) |
| `--quality` | `fast`, `good`, `best` | Resize quality (default: good) |
| `--output` | `path` | Custom output filename |
| `--preview` | - | Show preview after conversion |
| `--verbose` | - | Detailed output |

## Resize Modes

- **Scale**: Fits image with white padding (maintains aspect ratio)
- **Crop**: Fills display and crops excess (maintains aspect ratio)
- **Stretch**: Stretches to fit exactly (may distort image)

## Examples

```bash
# High quality landscape conversion with preview (small file)
python dither.py vacation.jpg --orientation landscape --quality best --preview

# iPhone HEIC photo with portrait mode and cropping
python dither.py IMG_5678.HEIC --orientation portrait --mode crop --format indexed

# Modern AVIF web image with best quality
python dither.py web_image.avif --orientation landscape --quality best --format indexed

# Quick conversion with custom output name (8-bit indexed)
python dither.py source.jpeg --mode scale --output display_image.bmp

# Maximum compression - fast resize with no dithering
python dither.py photo.jpg --quality fast --dither none --format indexed

# Process Apple Live Photo (HEIC format)
python dither.py LivePhoto.HEIC --verbose --preview
```

## Windows Batch Script

Use the included `convert-image.bat` for easier Windows usage:

```cmd
convert-image.bat photo.jpg
convert-image.bat IMG_1234.HEIC --mode crop --preview
```

## Output

- Creates a BMP file optimized for your e-paper display
- **8-bit indexed** (default): ~400KB file size - **3x smaller!**
- **24-bit RGB** (compatible): ~1.1MB file size
- Uses exact 7-color palette matching your ESP32 code
- Ready to upload to your ESP32 photo frame

## Requirements

- Python 3.6+
- Pillow (PIL) library: `pip install Pillow`
- Apple photo support: `pip install pillow-heif` (for HEIF/HEIC files)
- Modern web format support: `pip install pillow-avif-plugin` (for AVIF files)

### Installation

```bash
# Install all dependencies including Apple and modern web format support
pip install -r requirements.txt

# Or install manually
pip install Pillow pillow-heif pillow-avif-plugin
```

## Color Palette

The converter uses the exact 7-color palette from the ESP32 code:
- Black (0, 0, 0)
- White (255, 255, 255)
- Green (0, 255, 0)
- Blue (0, 0, 255)
- Red (255, 0, 0)
- Yellow (255, 255, 0)
- Orange (255, 128, 0)

## Tips

1. **For iPhone photos**: HEIC files are automatically supported with pillow-heif
2. **For modern web images**: AVIF files are supported with pillow-avif-plugin
3. **For photos**: Use `--mode crop` to fill the entire display
4. **For graphics**: Use `--mode scale` to preserve all content
5. **For smallest files**: Use `--format indexed` (default, 3x smaller)
6. **For compatibility**: Use `--format rgb` if you have issues
6. **For speed**: Use `--quality fast` and `--dither none`
7. **For quality**: Use `--quality best` and `--dither floyd`
8. **Test first**: Use `--preview` to see results before uploading
