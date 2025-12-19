#!/usr/bin/env python3
"""
ZED Camera Detection Script

Detects all connected ZED cameras and displays their serial numbers.
Use these serial numbers to update config/camera_config.yaml
"""

import sys
import subprocess

print("\n" + "=" * 60)
print("ZED Camera Detection")
print("=" * 60 + "\n")

# Check if pyzed is installed
try:
    import pyzed.sl as sl
    print("✓ pyzed module found\n")
except ImportError as e:
    print("❌ ERROR: pyzed module not found!")
    print("\nTo install:")
    print("  1. Install CUDA Toolkit (required by ZED SDK)")
    print("  2. Install ZED SDK from: https://www.stereolabs.com/developers/release")
    print("  3. pip install pyzed")
    print("\nNote: ZED cameras require CUDA to be installed")
    sys.exit(1)

# Check USB devices for ZED cameras
print("Scanning USB devices...")
try:
    result = subprocess.run(['lsusb'], capture_output=True, text=True)
    usb_devices = result.stdout
    zed_devices = [line for line in usb_devices.split('\n') if 'STEREOLABS' in line or 'ZED' in line.upper()]
    
    if zed_devices:
        print("✓ Found ZED devices on USB:")
        for dev in zed_devices:
            print(f"  {dev}")
        print()
    else:
        print("⚠ No ZED devices found on USB bus")
        print("  This means cameras are NOT physically connected/detected by the OS")
        print()
except Exception as e:
    print(f"⚠ Could not scan USB: {e}\n")

# Try to detect ZED cameras using SDK
try:
    camera_list = sl.Camera.get_device_list()
    
    print(f"ZED SDK initialized successfully")
    print(f"Number of ZED cameras found: {len(camera_list)}\n")
    
    if len(camera_list) == 0:
        print("=" * 60)
        print("❌ No ZED cameras detected!")
        print("=" * 60)
        print("\nTroubleshooting:")
        print("  1. **PLUG IN THE ZED CAMERAS**")
        print("  2. Check USB connections (use USB 3.0 ports - blue connectors)")
        print("  3. Verify cameras have power (LED should be lit)")
        print("  4. After plugging in, wait 5-10 seconds for enumeration")
        print("  5. Check if CUDA is installed: nvcc --version")
        print("  6. Check if ZED SDK is installed: ls /usr/local/zed/")
        print("  7. Try running ZED Diagnostic tool: /usr/local/zed/tools/ZED\ Diagnostic")
        print("  8. Check USB permissions:")
        print("     sudo chmod 666 /dev/bus/usb/*/*")
        print()
    else:
        print("=" * 60)
        print(f"✓ Found {len(camera_list)} ZED camera(s)!")
        print("=" * 60)
        print()
        
        for i, cam_info in enumerate(camera_list, 1):
            try:
                serial = str(cam_info.serial_number)
                model = str(cam_info.camera_model)
                
                # Camera model names
                model_names = {
                    'ZED': 'ZED',
                    'ZED_M': 'ZED Mini',
                    'ZED2': 'ZED 2',
                    'ZED2i': 'ZED 2i',
                    'ZED_X': 'ZED X',
                    'ZED_XM': 'ZED X Mini'
                }
                
                model_name = model_names.get(model, model)
                
                print(f"Camera {i}:")
                print(f"  Model:           {model_name}")
                print(f"  Serial Number:   {serial}")
                print()
            except Exception as e:
                print(f"Camera {i}: Error reading info - {e}")
                print()
        
        print("=" * 60)
        print("Next Steps:")
        print("=" * 60)
        print("1. Copy the serial numbers above")
        print("2. Update: teleop/config/camera_config.yaml")
        print("3. Add ZED camera serial numbers to the 'zed_cameras' section")
        print()
        print("Example configuration:")
        print()
        print("cameras:")
        print("  # RealSense cameras (palm-mounted)")
        print("  palm_left: '337322073311'")
        print("  palm_right: '337122075822'")
        print()
        print("zed_cameras:")
        if len(camera_list) > 0:
            print(f"  stage_center: '{camera_list[0].serial_number}'")
            if len(camera_list) > 1:
                print(f"  stage_left: '{camera_list[1].serial_number}'")
            if len(camera_list) > 2:
                print(f"  stage_right: '{camera_list[2].serial_number}'")
        else:
            print("  stage_center: 'YOUR_SERIAL_HERE'")
            print("  stage_left: 'YOUR_SERIAL_HERE'")
            print("  stage_right: 'YOUR_SERIAL_HERE'")
        print()
        print("zed_settings:")
        print("  fps: 30")
        print("  resolution: 'HD720'  # Options: HD2K, HD1080, HD720, VGA")
        print()

except Exception as e:
    print("=" * 60)
    print("❌ ERROR during ZED camera detection!")
    print("=" * 60)
    print(f"\nError details: {e}")
    print(f"Error type: {type(e).__name__}")
    print("\nThis could indicate:")
    print("  - CUDA not installed or not configured properly")
    print("  - ZED SDK not installed")
    print("  - USB permission issues")
    print("  - Driver problems")
    print("\nCheck:")
    print("  1. CUDA installation: nvcc --version")
    print("  2. ZED SDK installation: ls /usr/local/zed/")
    print("  3. Run ZED Diagnostic: /usr/local/zed/tools/ZED\\ Diagnostic")
    print()
    sys.exit(1)
