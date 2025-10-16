"""
HDF5 Episode Writer compatible with msc_humanoid_visual format

This writer saves teleoperation data in HDF5 format that can be directly
visualized using msc_humanoid_visual/utils/data_replay.py

File structure:
/observations/
    qpos: [T, N_joints] - joint positions (arm + hand)
    qvel: [T, N_joints] - joint velocities
    images/
        cam_high: [T, H, W, 3] - RGB images
        cam_low: [T, H, W, 3]
        cam_left_wrist: [T, H, W, 3]
        cam_right_wrist: [T, H, W, 3]
/action: [T, N_joints] - target joint positions
/metadata:
    episode_length: int
    fps: int
    robot_name: str
    timestamp: str
"""

import h5py
import numpy as np
import os
from datetime import datetime
import logging_mp

logger_mp = logging_mp.get_logger(__name__)


class EpisodeWriterHDF5:
    def __init__(self, save_dir='./utils/data/', robot_name='robot', fps=60):
        """
        Initialize HDF5 episode writer for msc_humanoid_visual compatible format
        
        Args:
            save_dir: Directory to save episodes
            robot_name: Name of the robot (e.g., 'H1_2', 'G1_29')
            fps: Recording frequency in Hz
        """
        self.save_dir = save_dir
        self.robot_name = robot_name
        self.fps = fps
        
        os.makedirs(save_dir, exist_ok=True)
        
        # Find next episode number
        self.episode_idx = 0
        while os.path.exists(os.path.join(save_dir, f'episode_{self.episode_idx}.hdf5')):
            self.episode_idx += 1
        
        self.filepath = os.path.join(save_dir, f'episode_{self.episode_idx}.hdf5')
        
        # Data buffers
        self.qpos_buffer = []
        self.qvel_buffer = []
        self.action_buffer = []
        self.image_buffers = {}  # {camera_name: [frames]}
        
        self.recording = False
        
        logger_mp.info(f"EpisodeWriterHDF5 initialized: {self.filepath}")
    
    def start_recording(self):
        """Start a new episode recording"""
        self.recording = True
        self.qpos_buffer = []
        self.qvel_buffer = []
        self.action_buffer = []
        self.image_buffers = {}
        logger_mp.info(f"Started recording episode {self.episode_idx}")
    
    def add_timestep(self, qpos, qvel, action, images=None):
        """
        Add a single timestep to the episode
        
        Args:
            qpos: Joint positions array [N] (arm + hand joints)
            qvel: Joint velocities array [N] (arm + hand joints)
            action: Target joint positions array [N] (arm + hand targets)
            images: Dict of {camera_name: image_array} where image is [H, W, 3] RGB uint8
        """
        if not self.recording:
            return
        
        self.qpos_buffer.append(np.array(qpos, dtype=np.float32))
        self.qvel_buffer.append(np.array(qvel, dtype=np.float32))
        self.action_buffer.append(np.array(action, dtype=np.float32))
        
        if images is not None:
            for camera_name, image in images.items():
                if camera_name not in self.image_buffers:
                    self.image_buffers[camera_name] = []
                # Ensure uint8 RGB format
                if image.dtype != np.uint8:
                    image = (np.clip(image, 0, 1) * 255).astype(np.uint8)
                # Ensure RGB (not BGR)
                if image.shape[2] == 3:
                    self.image_buffers[camera_name].append(image)
    
    def stop_recording(self):
        """Stop recording and save to HDF5 file"""
        if not self.recording:
            logger_mp.warning("Not currently recording")
            return
        
        self.recording = False
        
        if len(self.qpos_buffer) == 0:
            logger_mp.warning("No data to save")
            return
        
        try:
            # Convert buffers to numpy arrays
            qpos_data = np.array(self.qpos_buffer, dtype=np.float32)  # [T, N]
            qvel_data = np.array(self.qvel_buffer, dtype=np.float32)  # [T, N]
            action_data = np.array(self.action_buffer, dtype=np.float32)  # [T, N]
            
            episode_length = len(self.qpos_buffer)
            
            logger_mp.info(f"Saving episode {self.episode_idx}: {episode_length} timesteps")
            logger_mp.info(f"  qpos buffer length: {len(self.qpos_buffer)}")
            logger_mp.info(f"  qvel buffer length: {len(self.qvel_buffer)}")
            logger_mp.info(f"  action buffer length: {len(self.action_buffer)}")
            
            # Verify shapes match
            if qpos_data.shape != qvel_data.shape:
                logger_mp.error(f"Shape mismatch: qpos {qpos_data.shape} != qvel {qvel_data.shape}")
            if qpos_data.shape != action_data.shape:
                logger_mp.error(f"Shape mismatch: qpos {qpos_data.shape} != action {action_data.shape}")
            
            # Save to HDF5 in msc_humanoid_visual format
            with h5py.File(self.filepath, 'w') as f:
                # Create observations group
                obs_group = f.create_group('observations')
                obs_group.create_dataset('qpos', data=qpos_data, compression='gzip')
                obs_group.create_dataset('qvel', data=qvel_data, compression='gzip')
                
                # Create images subgroup
                if self.image_buffers:
                    logger_mp.info(f"  Saving {len(self.image_buffers)} camera streams (this may take a moment)...")
                    images_group = obs_group.create_group('images')
                    for cam_idx, (camera_name, frames) in enumerate(self.image_buffers.items(), 1):
                        logger_mp.info(f"    [{cam_idx}/{len(self.image_buffers)}] Compressing {camera_name}...")
                        image_data = np.array(frames, dtype=np.uint8)  # [T, H, W, 3]
                        images_group.create_dataset(
                            camera_name, 
                            data=image_data,
                            compression='gzip',
                            compression_opts=4
                        )
                        logger_mp.info(f"    [{cam_idx}/{len(self.image_buffers)}] Saved {camera_name}: {image_data.shape}")
                else:
                    logger_mp.warning("  No images to save")
                
                # Save actions
                f.create_dataset('action', data=action_data, compression='gzip')
                logger_mp.info(f"  Saved action: {action_data.shape}")
                
                # Save metadata as attributes
                f.attrs['episode_length'] = episode_length
                f.attrs['fps'] = self.fps
                f.attrs['robot_name'] = self.robot_name
                f.attrs['timestamp'] = datetime.now().isoformat()
                logger_mp.info(f"  Saved metadata: episode_length={episode_length}, fps={self.fps}, robot={self.robot_name}")
            
            logger_mp.info(f"Episode saved successfully: {self.filepath}")
            logger_mp.info(f"  qpos: {qpos_data.shape}, qvel: {qvel_data.shape}, action: {action_data.shape}")
            
            # Prepare for next episode
            self.episode_idx += 1
            self.filepath = os.path.join(self.save_dir, f'episode_{self.episode_idx}.hdf5')
            
        except Exception as e:
            logger_mp.error(f"Error saving episode: {e}")
            import traceback
            traceback.print_exc()
    
    def is_recording(self):
        """Check if currently recording"""
        return self.recording
    
    def get_current_length(self):
        """Get number of timesteps in current recording"""
        return len(self.qpos_buffer)
