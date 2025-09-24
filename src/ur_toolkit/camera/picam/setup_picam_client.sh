#!/bin/bash
# Setup script for Robot System Tools - Client Side
# For workstation/laptop to connect to Pi camera server

echo "🤖 Robot System Tools - Client Setup"
echo "======================================"

# Check Python version
echo "🐍 Checking Python version..."
python_version=$(python3 --version 2>/dev/null || echo "Python not found")
echo "   $python_version"

if ! command -v python3 &> /dev/null; then
    echo "❌ Python 3 is required but not installed"
    exit 1
fi

# Install client dependencies
echo ""
echo "📦 Installing client dependencies..."
pip3 install -r requirements.txt

if [ $? -eq 0 ]; then
    echo "✅ Dependencies installed successfully!"
else
    echo "❌ Failed to install dependencies"
    exit 1
fi

# Test camera client
echo ""
echo "🧪 Testing camera client..."
if [ -f "client_config.yaml" ]; then
    echo "   Config file found: client_config.yaml"
    echo "   Update the 'host' field with your Pi's IP address"
else
    echo "   Creating default config file..."
    cat > client_config.yaml << EOF
# Pi Camera Client Configuration
host: "192.168.1.100"  # Replace with your Pi's IP address
port: 8080
username: "pi"
password: "raspberry"

# Camera settings
resolution: [1920, 1080]
capture_timeout: 10
photo_directory: "photos"
EOF
    echo "✅ Created client_config.yaml"
    echo "   Please update the 'host' field with your Pi's IP address"
fi

echo ""
echo "🎉 Client setup complete!"
echo ""
echo "Next steps:"
echo "1. Set up the Pi camera server:"
echo "   curl -sSL https://raw.githubusercontent.com/your-repo/main/pi_cam_server/setup.sh | bash"
echo ""
echo "2. Update client_config.yaml with your Pi's IP address"
echo ""
echo "3. Test the connection:"
echo "   python3 test_camera_with_config.py"
echo ""
echo "4. For camera calibration:"
echo "   python3 calibrate_camera.py"
echo ""
echo "5. For AprilTag detection:"
echo "   python3 test_apriltag_detection.py"
echo ""
echo "📚 See README.md for detailed usage instructions"
