#!/bin/bash
# WhatsApp Image Processing Shell Script for Home Assistant
# Place this file at: /config/scripts/whatsapp_image_processor.sh
# Make executable with: chmod +x /config/scripts/whatsapp_image_processor.sh

set -e  # Exit on any error

# Configuration
VENV_PATH="/config/scripts/python_venv/whatsapp_dithering"
SCRIPT_PATH="/config/scripts/whatsapp_image_dithering.py"

# Function to log with timestamp to stdout
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1"
}

# Function to log error and exit
error_exit() {
    log "ERROR: $1"
    exit 1
}

log "🔧 Starting WhatsApp Image Processing"

# Check if required arguments are provided
if [ $# -lt 4 ]; then
    error_exit "Missing required arguments. Usage: $0 <phone_number> <image_url> <media_key> <phone_salt> [sender_name] [display_date] [image_text]"
fi

PHONE_NUMBER="$1"
IMAGE_URL="$2"
MEDIA_KEY="$3"
PHONE_SALT="$4"
SENDER_NAME="${5:-Unknown}"
DISPLAY_DATE="${6:-$(date '+%d.%m.%Y')}"
IMAGE_TEXT="${7:-}"

log "📋 Parameters received:"
log "  - Phone: $PHONE_NUMBER"
log "  - URL: [${#IMAGE_URL} characters]"
log "  - Key: [${#MEDIA_KEY} characters]"
log "  - Salt: $PHONE_SALT"
log "  - Sender: $SENDER_NAME"
log "  - Display Date: $DISPLAY_DATE"
log "  - Image Text: ${IMAGE_TEXT:-'(none)'}"

# Create virtual environment if it doesn't exist
if [ ! -d "$VENV_PATH" ]; then
    log "📦 Creating virtual environment at $VENV_PATH"
    python3 -m venv "$VENV_PATH" || error_exit "Failed to create virtual environment"
fi

# Activate virtual environment
log "🔌 Activating virtual environment"
source "$VENV_PATH/bin/activate" || error_exit "Failed to activate virtual environment"

# Install required packages if not already installed
log "📥 Checking/installing required packages"
pip install --quiet pycryptodome Pillow requests || error_exit "Failed to install required packages"

# Check if Python script exists
if [ ! -f "$SCRIPT_PATH" ]; then
    error_exit "Python script not found at $SCRIPT_PATH"
fi

# Run the Python script
log "🚀 Running Python image processing script"
python "$SCRIPT_PATH" \
    --phone "$PHONE_NUMBER" \
    --url "$IMAGE_URL" \
    --key "$MEDIA_KEY" \
    --salt "$PHONE_SALT" \
    --name "$SENDER_NAME" \
    --date "$DISPLAY_DATE" \
    --text "$IMAGE_TEXT"

PYTHON_EXIT_CODE=$?

# Deactivate virtual environment
deactivate

if [ $PYTHON_EXIT_CODE -eq 0 ]; then
    log "✅ WhatsApp image processing completed successfully"
    exit 0
else
    error_exit "Python script failed with exit code $PYTHON_EXIT_CODE"
fi
