#!/usr/bin/env python3
"""
RealSense Camera Detection Script

Detects all connected Intel RealSense cameras and displays their serial numbers.
Use these serial numbers to update config/camera_config.yaml
"""

import sys
import os
import subprocess
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from utils.realsense_multi_capture import list_realsense_cameras
import yaml

print("\n" + "=" * 60)
print("RealSense Camera Detection")
print("=" * 60 + "\n")

# Check if pyrealsense2 is installed
try:
    import pyrealsense2 as rs
    print("✓ pyrealsense2 module found\n")
except ImportError as e:
    print("❌ ERROR: pyrealsense2 module not found!")
    print("\nTo install:")
    print("  conda install -c conda-forge pyrealsense2")
    print("  # OR")
    print("  pip install pyrealsense2")
    print("\nIf already installed, make sure you're in the correct conda environment.")
    sys.exit(1)

# Check USB devices for Intel RealSense
print("Scanning USB devices...")
try:
    result = subprocess.run(['lsusb'], capture_output=True, text=True)
    usb_devices = result.stdout
    intel_devices = [line for line in usb_devices.split('\n') if 'Intel' in line]
    
    if intel_devices:
        print("✓ Found Intel devices on USB:")
        for dev in intel_devices:
            print(f"  {dev}")
        print()
    else:
        print("⚠ No Intel devices found on USB bus")
        print("  This means cameras are NOT physically connected/detected by the OS")
        print()
except Exception as e:
    print(f"⚠ Could not scan USB: {e}\n")

# Check for RealSense SDK tools
print("Checking for RealSense SDK tools...")
try:
    result = subprocess.run(['which', 'rs-enumerate-devices'], 
                          capture_output=True, text=True)
    if result.returncode == 0:
        print(f"✓ RealSense SDK tools found: {result.stdout.strip()}\n")
    else:
        print("⚠ RealSense SDK command-line tools not found (optional)\n")
except Exception as e:
    print("⚠ Could not check for SDK tools\n")

def main():
    try:
        cameras = list_realsense_cameras()
    except Exception as e:
        print(f"❌ Error detecting cameras: {e}")
        print("\nMake sure pyrealsense2 is installed:")
        print("  pip install pyrealsense2")
        return
    
    try:
        ctx = rs.context()
        devices = ctx.query_devices()
        
        print(f"RealSense context created successfully")
        print(f"Number of devices found: {len(devices)}\n")
        
        if len(devices) == 0:
            print("=" * 60)
            print("❌ No RealSense cameras detected!")
            print("=" * 60)
            print("\nTroubleshooting:")
            print("  1. **PLUG IN THE CAMERAS** - No Intel devices found on USB")
            print("  2. Check USB connections (use USB 3.0 ports - blue connectors)")
            print("  3. Verify cameras have power (LED should be lit)")
            print("  4. After plugging in, wait 5-10 seconds for enumeration")
            print("  5. Try running: rs-enumerate-devices")
            print("  6. Check USB permissions:")
            print("     sudo chmod 666 /dev/bus/usb/*/*")
            print("  7. Try different USB ports")
            print("\n  If still not working, run with sudo to check permissions:")
            print("     sudo -E python3 teleop/scripts/detect_cameras.py")
            print()
            return
        else:
            print("=" * 60)
            print(f"✓ Found {len(devices)} RealSense camera(s)!")
            print("=" * 60)
            print()
            
            for i, dev in enumerate(devices, 1):
                try:
                    serial = dev.get_info(rs.camera_info.serial_number)
                    name = dev.get_info(rs.camera_info.name)
                    fw_version = dev.get_info(rs.camera_info.firmware_version)
                    usb_type = dev.get_info(rs.camera_info.usb_type_descriptor)
                    
                    print(f"Camera {i}:")
                    print(f"  Name:            {name}")
                    print(f"  Serial Number:   {serial}")
                    print(f"  Firmware:        {fw_version}")
                    print(f"  USB Type:        {usb_type}")
                    print()
                except Exception as e:
                    print(f"Camera {i}: Error reading info - {e}")
                    print()
            
            print("=" * 60)
            print("Next Steps:")
            print("=" * 60)
            print("1. Copy the serial numbers above")
            print("2. Edit: teleop/config/camera_config.yaml")
            print("3. Replace 'REPLACE_WITH_SERIAL' with actual serial numbers")
            print()
            print("Example:")
            print("  cameras:")
            print(f"    stage_cam: \"{devices[0].get_info(rs.camera_info.serial_number)}\"")
            print("    palm_left: \"YOUR_SERIAL_HERE\"")
            print("    palm_right: \"YOUR_SERIAL_HERE\"")
            print()
    except Exception as e:
        print("=" * 60)
        print("❌ ERROR during camera detection!")
        print("=" * 60)
        print(f"\nError details: {e}")
        print(f"Error type: {type(e).__name__}")
        print("\nThis could indicate:")
        print("  - USB permission issues")
        print("  - Driver problems")
        print("  - Hardware connection issues")
        print("\nTry running with sudo to test permissions:")
        print("  sudo -E python3 teleop/scripts/detect_cameras.py")
        print()
        return
    
    # Generate example config
    print("\n" + "="*60)
    print("Example camera_config.yaml:")
    print("="*60 + "\n")
    
    config_example = {
        'cameras': {},
        'settings': {
            'fps': 30,
            'width': 640,
            'height': 480
        }
    }
    
    # Suggest names based on number of cameras
    suggested_names = ['stage_cam', 'side_left', 'side_right', 'palm_left', 'palm_right']
    for i, cam in enumerate(cameras):
        name = suggested_names[i] if i < len(suggested_names) else f'camera_{i+1}'
        config_example['cameras'][name] = cam['serial']
    
    print(yaml.dump(config_example, default_flow_style=False))
    
    print("\n💡 Copy this to: config/camera_config.yaml")
    print("   Then edit the camera names as needed\n")


if __name__ == "__main__":
    main()
