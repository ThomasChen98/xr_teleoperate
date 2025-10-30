#!/usr/bin/env python3
"""
Test script for RealSense multi-camera capture

Tests the palm camera capture system in isolation before full teleoperation.
"""

import sys
import os
import time
import yaml
import cv2
import numpy as np

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

from utils.realsense_multi_capture import MultiRealsenseCapture

def test_realsense_capture():
    """Test RealSense camera capture with live preview"""
    
    print("\n" + "=" * 60)
    print("RealSense Multi-Camera Capture Test")
    print("=" * 60 + "\n")
    
    # Load camera configuration
    camera_config_path = os.path.join(current_dir, 'config/camera_config.yaml')
    
    if not os.path.exists(camera_config_path):
        print(f"❌ Camera config file not found: {camera_config_path}")
        print("Please create config/camera_config.yaml with your camera serial numbers")
        print("Run: python3 teleop/scripts/detect_cameras.py to find serial numbers")
        return False
    
    with open(camera_config_path, 'r') as f:
        camera_yaml = yaml.safe_load(f)
    
    camera_config = camera_yaml.get('cameras', {})
    camera_settings = camera_yaml.get('settings', {})
    
    # Filter out placeholder serial numbers
    active_cameras = {name: serial for name, serial in camera_config.items() 
                     if serial != "REPLACE_WITH_SERIAL" and serial}
    
    if len(active_cameras) == 0:
        print("❌ No valid camera serial numbers found in config")
        print("Please update config/camera_config.yaml with actual serial numbers")
        return False
    
    print(f"Found {len(active_cameras)} configured camera(s):")
    for name, serial in active_cameras.items():
        print(f"  - {name}: {serial}")
    print()
    
    # Initialize multi-camera capture
    print("Initializing multi-camera capture system...")
    capture = MultiRealsenseCapture(
        camera_config=active_cameras,
        fps=camera_settings.get('fps', 30),
        width=camera_settings.get('width', 640),
        height=camera_settings.get('height', 480)
    )
    
    # Start capture
    success = capture.start()
    if not success:
        print("❌ Failed to start camera capture")
        return False
    
    print("✓ Camera capture started successfully!")
    print(f"  Active cameras: {capture.get_active_cameras()}")
    print()
    
    print("=" * 60)
    print("LIVE PREVIEW - Press 'q' to quit, 's' to save snapshot")
    print("=" * 60)
    print()
    
    try:
        # Give cameras time to stabilize
        time.sleep(1.0)
        
        frame_count = 0
        start_time = time.time()
        snapshot_count = 0
        
        while True:
            # Get latest frames
            frames = capture.get_latest_frames()
            
            # Display frames
            display_windows = []
            for cam_name, frame in frames.items():
                if frame is not None:
                    # Resize for display
                    display_frame = cv2.resize(frame, (480, 360))
                    
                    # Add camera name and frame count
                    cv2.putText(display_frame, f"{cam_name}", 
                              (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                              1, (0, 255, 0), 2)
                    cv2.putText(display_frame, f"Frame: {frame_count}", 
                              (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 
                              0.7, (0, 255, 0), 2)
                    
                    cv2.imshow(cam_name, display_frame)
                    display_windows.append(cam_name)
            
            # Calculate FPS
            frame_count += 1
            if frame_count % 30 == 0:
                elapsed = time.time() - start_time
                fps = frame_count / elapsed
                print(f"Average FPS: {fps:.2f} | Active cameras: {len(display_windows)}")
            
            # Handle key press
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("\nQuitting...")
                break
            elif key == ord('s'):
                # Save snapshot
                snapshot_dir = os.path.join(current_dir, 'test_snapshots')
                os.makedirs(snapshot_dir, exist_ok=True)
                
                timestamp = int(time.time() * 1000)
                for cam_name, frame in frames.items():
                    if frame is not None:
                        filename = f"{cam_name}_{timestamp}.jpg"
                        filepath = os.path.join(snapshot_dir, filename)
                        cv2.imwrite(filepath, cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
                
                snapshot_count += 1
                print(f"✓ Snapshot #{snapshot_count} saved to {snapshot_dir}/")
        
        # Final statistics
        total_time = time.time() - start_time
        avg_fps = frame_count / total_time
        
        print("\n" + "=" * 60)
        print("TEST RESULTS")
        print("=" * 60)
        print(f"Total frames captured: {frame_count}")
        print(f"Total time: {total_time:.2f} seconds")
        print(f"Average FPS: {avg_fps:.2f}")
        print(f"Active cameras: {len(display_windows)}")
        print(f"Snapshots saved: {snapshot_count}")
        print()
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    
    finally:
        # Cleanup
        print("Stopping camera capture...")
        capture.stop()
        cv2.destroyAllWindows()
        print("✓ Cameras stopped cleanly")
        print()
    
    return True


if __name__ == "__main__":
    success = test_realsense_capture()
    sys.exit(0 if success else 1)
