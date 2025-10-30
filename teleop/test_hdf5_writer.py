#!/usr/bin/env python3
"""
Test script to verify EpisodeWriterHDF5 works correctly
"""

import numpy as np
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from utils.episode_writer_hdf5 import EpisodeWriterHDF5

def test_writer():
    print("Testing EpisodeWriterHDF5...")
    
    # Create writer
    writer = EpisodeWriterHDF5(
        save_dir='./utils/data/test_hdf5',
        robot_name='H1_2',
        fps=60
    )
    
    print(f"Writer created, will save to: {writer.filepath}")
    
    # Start recording
    writer.start_recording()
    print("Recording started")
    
    # Add some test timesteps
    num_arm_joints = 14
    num_hand_joints = 12
    num_total_joints = num_arm_joints + num_hand_joints
    
    num_timesteps = 10
    for i in range(num_timesteps):
        qpos = np.random.rand(num_total_joints).astype(np.float32)
        qvel = np.random.rand(num_total_joints).astype(np.float32)
        action = np.random.rand(num_total_joints).astype(np.float32)
        
        # Create test image
        image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        images = {'ego_cam': image}
        
        writer.add_timestep(qpos, qvel, action, images)
        print(f"Added timestep {i+1}/{num_timesteps}")
    
    # Stop recording
    print("Stopping recording...")
    writer.stop_recording()
    
    print(f"\nTest complete! File saved to: {writer.filepath}")
    print("\nNow verify the file:")
    print(f"  python3 verify_hdf5_format.py {writer.filepath}")
    
    return writer.filepath

if __name__ == "__main__":
    filepath = test_writer()
