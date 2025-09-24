#!/bin/bash
# Pi Camera Server Setup Script
# Simple setup using system packages only - no virtual environments!

set -e  # Exit on any error

echo "🍓 Pi Camera Server Setup (System Packages Only)"
echo "================================================="

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

# Get the directory where this script is located (should be the git repo)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

echo "📂 Working from git repo: $REPO_DIR"
echo ""

# Update system
echo "📦 Updating system packages..."
sudo apt update
sudo apt upgrade -y

# Install system dependencies (no pip, no venv!)
echo "🔧 Installing system dependencies..."
sudo apt install -y python3 python3-dev
sudo apt install -y libcamera-apps libcamera-dev  
sudo apt install -y python3-libcamera python3-kms++
sudo apt install -y python3-numpy python3-yaml
sudo apt install -y python3-picamera2

# Optional: Install opencv from system packages if available
echo "� Installing OpenCV (optional)..."
sudo apt install -y python3-opencv || echo "⚠️  OpenCV not available in repos, skipping..."

# Create photos directory in repo
mkdir -p "$SCRIPT_DIR/photos"

# Enable camera
echo "📷 Enabling camera interface..."
sudo raspi-config nonint do_camera 0

# Make camera server executable
chmod +x "$SCRIPT_DIR/camera_server.py"

# Get current user for systemd service
CURRENT_USER=$(whoami)
CURRENT_GROUP=$(id -gn)

# Create systemd service (runs directly from git repo)
echo "⚙️  Creating systemd service..."
sudo tee /etc/systemd/system/camera-server.service > /dev/null <<EOF
[Unit]
Description=Robot Camera Server
After=network.target
Wants=network.target

[Service]
Type=simple
User=$CURRENT_USER
Group=$CURRENT_GROUP
WorkingDirectory=$SCRIPT_DIR
ExecStart=/usr/bin/python3 $SCRIPT_DIR/camera_server.py
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

# Reload systemd and enable service
sudo systemctl daemon-reload
sudo systemctl enable camera-server

# Start the service immediately
echo "🚀 Starting camera server service..."
sudo systemctl start camera-server

# Check if service started successfully
sleep 2
if sudo systemctl is-active --quiet camera-server; then
    echo "✅ Camera server service started successfully"
    echo "📡 Service will auto-start on boot"
else
    echo "⚠️  Service created but failed to start"
    echo "   You can start it manually after reboot"
fi

# Set hostname (optional)
read -p "🏷️  Set hostname to 'picam-server'? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "picam-server" | sudo tee /etc/hostname
    sudo sed -i 's/127.0.1.1.*/127.0.1.1\tpicam-server/' /etc/hosts
    echo "✅ Hostname set to picam-server"
fi

echo ""
echo "✅ Pi Camera Server setup complete!"
echo ""
echo "📂 Server running from: $SCRIPT_DIR"
echo "📷 Photos saved to: $SCRIPT_DIR/photos"
echo "🚀 Service enabled for auto-start on boot"
echo ""
echo "📋 Camera server is now running and will:"
echo "   ✅ Start automatically when Pi boots"
echo "   ✅ Restart automatically if it crashes"
echo "   ✅ Listen on port 2222 for connections"
echo ""
echo "🔧 Service management commands:"
echo "   Status: sudo systemctl status camera-server"
echo "   Logs:   sudo journalctl -u camera-server -f"
echo "   Stop:   sudo systemctl stop camera-server"
echo "   Start:  sudo systemctl start camera-server"
echo "   Restart: sudo systemctl restart camera-server"
echo ""
echo "🧪 Test from client computer:"
echo "   python3 test_client.py $(hostname -I | awk '{print $1}')"
echo ""
echo "🔗 Access from client:"
echo "   python3 camera_client.py <PI_IP>"
echo ""
