#!/bin/bash
# Simple script to list ZED cameras via USB

echo "==========================================================="
echo "ZED Camera USB Detection"
echo "==========================================================="
echo ""

echo "Scanning for ZED/Stereolabs devices on USB..."
lsusb | grep -i "STEREOLABS\|ZED"

if [ $? -ne 0 ]; then
    echo "No ZED cameras found on USB bus"
    echo ""
    echo "Troubleshooting:"
    echo "  1. Check USB connections (use USB 3.0 ports)"
    echo "  2. Verify cameras have power"
    echo "  3. Try: lsusb  (to see all USB devices)"
else
    echo ""
    echo "==========================================================="
    echo "ZED cameras detected! To get serial numbers:"
    echo "==========================================================="
    echo ""
    echo "You need to install the ZED SDK and pyzed:"
    echo "  1. Install CUDA: https://developer.nvidia.com/cuda-downloads"
    echo "  2. Install ZED SDK: https://www.stereolabs.com/developers/release"
    echo "  3. pip install pyzed"
    echo ""
    echo "Then run: python3 teleop/scripts/detect_zed_cameras.py"
    echo ""
    echo "OR manually configure ZED camera serial numbers in:"
    echo "  teleop/config/camera_config.yaml"
fi
echo ""
