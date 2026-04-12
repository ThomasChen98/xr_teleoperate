#!/usr/bin/env python3
"""
Quick script to check HDF5 file contents and verify it has real data
"""

import h5py
import numpy as np
import sys

def check_hdf5_data(filepath):
    """Check HDF5 file structure and verify data is real (not zeros/placeholders)"""
    
    print(f"\n{'='*70}")
    print(f"Inspecting: {filepath}")
    print(f"{'='*70}\n")
    
    try:
        with h5py.File(filepath, 'r') as f:
            # Print file structure
            print("File Structure:")
            print("-" * 70)
            def print_structure(name, obj):
                indent = "  " * (name.count('/'))
                if isinstance(obj, h5py.Dataset):
                    shape = obj.shape
                    dtype = obj.dtype
                    size_mb = obj.nbytes / (1024 * 1024)
                    print(f"{indent}{name.split('/')[-1]}: Dataset {shape} {dtype} ({size_mb:.2f} MB)")
                elif isinstance(obj, h5py.Group):
                    print(f"{indent}{name.split('/')[-1]}/: Group")
            f.visititems(print_structure)
            
            print("\n" + "="*70)
            print("Data Verification:")
            print("="*70)
            
            # Check observations/qpos
            if 'observations/qpos' in f:
                qpos = f['observations/qpos'][:]
                print(f"\n✓ qpos:")
                print(f"  Shape: {qpos.shape}")
                print(f"  Dtype: {qpos.dtype}")
                print(f"  Min: {np.min(qpos):.6f}, Max: {np.max(qpos):.6f}, Mean: {np.mean(qpos):.6f}")
                print(f"  Std: {np.std(qpos):.6f}")
                print(f"  Non-zero values: {np.count_nonzero(qpos)}/{qpos.size} ({100*np.count_nonzero(qpos)/qpos.size:.1f}%)")
                print(f"  Sample values (first timestep): {qpos[0][:10]}")
                print(f"  Sample values (last timestep): {qpos[-1][:10]}")
                
                # Check if all zeros
                if np.allclose(qpos, 0):
                    print("  ⚠ WARNING: All values are zero!")
                elif np.allclose(qpos, qpos[0]):
                    print("  ⚠ WARNING: All timesteps are identical!")
                else:
                    print("  ✓ Data appears to vary across timesteps")
            
            # Check observations/qvel
            if 'observations/qvel' in f:
                qvel = f['observations/qvel'][:]
                print(f"\n✓ qvel:")
                print(f"  Shape: {qvel.shape}")
                print(f"  Dtype: {qvel.dtype}")
                print(f"  Min: {np.min(qvel):.6f}, Max: {np.max(qvel):.6f}, Mean: {np.mean(qvel):.6f}")
                print(f"  Std: {np.std(qvel):.6f}")
                print(f"  Non-zero values: {np.count_nonzero(qvel)}/{qvel.size} ({100*np.count_nonzero(qvel)/qvel.size:.1f}%)")
                print(f"  Sample values (first timestep): {qvel[0][:10]}")
                print(f"  Sample values (last timestep): {qvel[-1][:10]}")
                
                if np.allclose(qvel, 0):
                    print("  ⚠ WARNING: All values are zero!")
                elif np.allclose(qvel, qvel[0]):
                    print("  ⚠ WARNING: All timesteps are identical!")
                else:
                    print("  ✓ Data appears to vary across timesteps")
            
            # Check action
            if 'action' in f:
                action = f['action'][:]
                print(f"\n✓ action:")
                print(f"  Shape: {action.shape}")
                print(f"  Dtype: {action.dtype}")
                print(f"  Min: {np.min(action):.6f}, Max: {np.max(action):.6f}, Mean: {np.mean(action):.6f}")
                print(f"  Std: {np.std(action):.6f}")
                print(f"  Non-zero values: {np.count_nonzero(action)}/{action.size} ({100*np.count_nonzero(action)/action.size:.1f}%)")
                print(f"  Sample values (first timestep): {action[0][:10]}")
                print(f"  Sample values (last timestep): {action[-1][:10]}")
                
                if np.allclose(action, 0):
                    print("  ⚠ WARNING: All values are zero!")
                elif np.allclose(action, action[0]):
                    print("  ⚠ WARNING: All timesteps are identical!")
                else:
                    print("  ✓ Data appears to vary across timesteps")
            
            # Check images
            if 'observations/images' in f:
                images_group = f['observations/images']
                print(f"\n✓ Images:")
                for cam_name in images_group.keys():
                    cam_data = images_group[cam_name]
                    print(f"\n  Camera: {cam_name}")
                    print(f"    Shape: {cam_data.shape}")
                    print(f"    Dtype: {cam_data.dtype}")
                    
                    # Sample a few frames to check
                    if cam_data.shape[0] > 0:
                        sample_frame = cam_data[0]
                        print(f"    Sample frame stats: min={np.min(sample_frame)}, max={np.max(sample_frame)}, mean={np.mean(sample_frame):.2f}")
                        
                        # Check if all zeros or all same value
                        if np.allclose(sample_frame, 0):
                            print(f"    ⚠ WARNING: First frame is all zeros!")
                        elif np.allclose(sample_frame, sample_frame[0,0,0]):
                            print(f"    ⚠ WARNING: First frame is uniform!")
                        else:
                            print(f"    ✓ Frame has varied pixel values")
                        
                        # Check if all frames are the same
                        if cam_data.shape[0] > 1:
                            if np.allclose(cam_data[0], cam_data[-1]):
                                print(f"    ⚠ WARNING: First and last frames are identical!")
                            else:
                                print(f"    ✓ Frames vary across time")
            
            # Check locomotion data if present
            if 'observations/loco_state' in f:
                loco_state = f['observations/loco_state'][:]
                print(f"\n✓ loco_state:")
                print(f"  Shape: {loco_state.shape}")
                print(f"  Dtype: {loco_state.dtype}")
                print(f"  Min: {np.min(loco_state):.6f}, Max: {np.max(loco_state):.6f}, Mean: {np.mean(loco_state):.6f}")
                print(f"  Sample values (first timestep): {loco_state[0]}")
                print(f"  Sample values (last timestep): {loco_state[-1]}")
            
            if 'loco_action' in f:
                loco_action = f['loco_action'][:]
                print(f"\n✓ loco_action:")
                print(f"  Shape: {loco_action.shape}")
                print(f"  Dtype: {loco_action.dtype}")
                print(f"  Min: {np.min(loco_action):.6f}, Max: {np.max(loco_action):.6f}, Mean: {np.mean(loco_action):.6f}")
                print(f"  Sample values (first timestep): {loco_action[0]}")
                print(f"  Sample values (last timestep): {loco_action[-1]}")
            
            # Print metadata
            print(f"\n{'='*70}")
            print("Metadata:")
            print("="*70)
            for attr_name in sorted(f.attrs.keys()):
                print(f"  {attr_name}: {f.attrs[attr_name]}")
            
            print(f"\n{'='*70}")
            print("Summary:")
            print("="*70)
            
            # Overall assessment
            has_data = False
            if 'observations/qpos' in f:
                qpos = f['observations/qpos'][:]
                if not np.allclose(qpos, 0) and not np.allclose(qpos, qpos[0]):
                    has_data = True
                    print("✓ qpos contains real varying data")
                else:
                    print("✗ qpos appears to be placeholder/empty data")
            
            if 'observations/images' in f:
                images_group = f['observations/images']
                has_images = False
                for cam_name in images_group.keys():
                    cam_data = images_group[cam_name]
                    if cam_data.shape[0] > 0:
                        sample = cam_data[0]
                        if not np.allclose(sample, 0) and not np.allclose(sample, sample[0,0,0]):
                            has_images = True
                            break
                if has_images:
                    print("✓ Images contain real data")
                else:
                    print("✗ Images appear to be placeholder/empty data")
            
            print(f"\n{'='*70}\n")
            
    except FileNotFoundError:
        print(f"✗ File not found: {filepath}")
        return False
    except Exception as e:
        print(f"✗ Error reading file: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    return True

if __name__ == "__main__":
    filepath = sys.argv[1] if len(sys.argv) > 1 else "teleop/utils/data/bookworm/episode_0.hdf5"
    check_hdf5_data(filepath)

