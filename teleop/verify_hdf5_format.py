#!/usr/bin/env python3
"""
Verification script to check HDF5 episode format compatibility with msc_humanoid_visual

Usage:
    python3 verify_hdf5_format.py /path/to/episode_0.hdf5
"""

import h5py
import sys
import numpy as np


def verify_hdf5_format(filepath):
    """Verify HDF5 file has the correct structure for msc_humanoid_visual"""
    
    print(f"\n{'='*60}")
    print(f"Verifying: {filepath}")
    print(f"{'='*60}\n")
    
    errors = []
    warnings = []
    
    try:
        with h5py.File(filepath, 'r') as f:
            # Check required top-level groups/datasets
            if 'observations' not in f:
                errors.append("Missing required group: /observations")
            if 'action' not in f:
                errors.append("Missing required dataset: /action")
            
            # Check observations structure
            if 'observations' in f:
                obs = f['observations']
                
                if 'qpos' not in obs:
                    errors.append("Missing required dataset: /observations/qpos")
                else:
                    qpos = obs['qpos']
                    print(f"✓ qpos shape: {qpos.shape} dtype: {qpos.dtype}")
                
                if 'qvel' not in obs:
                    errors.append("Missing required dataset: /observations/qvel")
                else:
                    qvel = obs['qvel']
                    print(f"✓ qvel shape: {qvel.shape} dtype: {qvel.dtype}")
                
                # Check shapes match
                if 'qpos' in obs and 'qvel' in obs:
                    if obs['qpos'].shape != obs['qvel'].shape:
                        errors.append(f"Shape mismatch: qpos {obs['qpos'].shape} != qvel {obs['qvel'].shape}")
                
                # Check images
                if 'images' not in obs:
                    warnings.append("No images group found in /observations/images")
                else:
                    images = obs['images']
                    print(f"\n✓ Found {len(images.keys())} camera(s):")
                    for cam_name in images.keys():
                        cam_data = images[cam_name]
                        print(f"  - {cam_name}: shape {cam_data.shape} dtype {cam_data.dtype}")
                        
                        # Verify image format
                        if len(cam_data.shape) != 4:
                            errors.append(f"Camera {cam_name} has wrong dims: {len(cam_data.shape)}, expected 4 (T, H, W, C)")
                        elif cam_data.shape[-1] != 3:
                            errors.append(f"Camera {cam_name} has wrong channels: {cam_data.shape[-1]}, expected 3 (RGB)")
                        if cam_data.dtype != np.uint8:
                            warnings.append(f"Camera {cam_name} is not uint8 (got {cam_data.dtype})")
            
            # Check action
            if 'action' in f:
                action = f['action']
                print(f"\n✓ action shape: {action.shape} dtype: {action.dtype}")
                
                # Check action shape matches qpos
                if 'observations' in f and 'qpos' in f['observations']:
                    if action.shape != f['observations/qpos'].shape:
                        errors.append(f"Shape mismatch: action {action.shape} != qpos {f['observations/qpos'].shape}")
            
            # Check metadata
            print(f"\n✓ Metadata attributes:")
            for attr_name in f.attrs.keys():
                print(f"  - {attr_name}: {f.attrs[attr_name]}")
            
            required_attrs = ['episode_length', 'fps', 'robot_name', 'timestamp']
            for attr in required_attrs:
                if attr not in f.attrs:
                    warnings.append(f"Missing recommended attribute: {attr}")
            
            # Summary
            print(f"\n{'='*60}")
            if errors:
                print(f"ERRORS ({len(errors)}):")
                for err in errors:
                    print(f"  - {err}")
            else:
                print(f"No errors found!")
            
            if warnings:
                print(f"\nWARNINGS ({len(warnings)}):")
                for warn in warnings:
                    print(f"  - {warn}")
            
            print(f"{'='*60}\n")
            
            return len(errors) == 0
            
    except FileNotFoundError:
        print(f"File not found: {filepath}")
        return False
    except Exception as e:
        print(f"Error reading file: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 verify_hdf5_format.py <path_to_episode.hdf5>")
        print("\nExample:")
        print("  python3 verify_hdf5_format.py ./utils/data/pick\\ cube/episode_0.hdf5")
        sys.exit(1)
    
    filepath = sys.argv[1]
    success = verify_hdf5_format(filepath)
    
    sys.exit(0 if success else 1)
