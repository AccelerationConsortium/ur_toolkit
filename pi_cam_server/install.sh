#!/bin/bash
# Pi Camera Server One-Command Installer
# For fresh Raspberry Pi OS (Debian Bookworm)
# Usage: curl -sSL https://raw.githubusercontent.com/kelvinchow23/robot_system_tools/master/install.sh | bash

set -e

echo "🍓 Pi Camera Server One-Command Installer"
echo "=========================================="
echo ""

# Check if running on Pi
if ! grep -q "Raspberry Pi" /proc/device-tree/model 2>/dev/null; then
    echo "⚠️  Warning: This doesn't appear to be a Raspberry Pi"
    echo "   This script is designed for Pi deployment"
    read -p "Continue anyway? (y/n): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Install git if not present
if ! command -v git &> /dev/null; then
    echo "📦 Installing git..."
    sudo apt update
    sudo apt install -y git
fi

# Get current user
CURRENT_USER=$(whoami)
USER_HOME=$(eval echo ~$CURRENT_USER)

# Clone repository to user's home directory
REPO_DIR="$USER_HOME/robot_system_tools"
if [ -d "$REPO_DIR" ]; then
    echo "📂 Repository already exists, updating..."
    cd "$REPO_DIR"
    git pull
else
    echo "📂 Cloning repository to $REPO_DIR..."
    cd "$USER_HOME"
    git clone https://github.com/kelvinchow23/robot_system_tools.git
fi

# Run the setup script
echo "🚀 Running camera server setup..."
cd "$REPO_DIR/pi_cam_server"
chmod +x setup.sh
./setup.sh

echo ""
echo "✅ Installation complete!"
echo ""
echo "📂 Camera server installed to: $REPO_DIR/pi_cam_server"
echo "� Running as user: $CURRENT_USER"
echo "�🔧 Service status: sudo systemctl status camera-server"
echo "📷 Test manually: cd $REPO_DIR/pi_cam_server && python3 camera_server.py"
echo ""
