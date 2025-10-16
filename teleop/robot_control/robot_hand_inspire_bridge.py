"""
Inspire Hand Controller with Bridge Support

This controller works with Inspire FTP hands connected via network to the laptop.
Uses inspire_sdkpy to create bridges that translate DDS commands to Modbus for physical hands.

Author: Adapted from h1_2_policy_control.py
"""

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from inspire_sdkpy import inspire_hand_defaut, inspire_dds, inspire_sdk
from inspire_sdkpy.inspire_dds import inspire_hand_ctrl, inspire_hand_state

from teleop.robot_control.hand_retargeting import HandRetargeting, HandType
import numpy as np
import threading
import time
from multiprocessing import Process, Array

import logging_mp
logger_mp = logging_mp.get_logger(__name__)

Inspire_Num_Motors = 6

# DDS topics for bridge-based control (separate left/right)
kTopicInspireLeftCommand = "rt/inspire_hand/ctrl/l"
kTopicInspireLeftState = "rt/inspire_hand/state/l"
kTopicInspireRightCommand = "rt/inspire_hand/ctrl/r"
kTopicInspireRightState = "rt/inspire_hand/state/r"

class Inspire_Bridge_Controller:
    """
    Inspire hand controller with bridge support for network-connected hands.
    
    Args:
        left_hand_array: Input array with left hand skeleton data from XR device
        right_hand_array: Input array with right hand skeleton data from XR device
        dual_hand_data_lock: Lock for synchronizing hand state/action arrays
        dual_hand_state_array: Output array for hand states
        dual_hand_action_array: Output array for hand actions
        fps: Control frequency (default: 100Hz)
        Unit_Test: Enable unit testing mode
        simulation_mode: Enable simulation mode
        network_interface: Network interface name (e.g., "eno1", "eth0")
        left_hand_ip: IP address of left hand (e.g., "192.168.123.211")
        right_hand_ip: IP address of right hand (e.g., "192.168.123.210")
    """
    
    INSPIRE_HAND_OPEN = 1000      # Fully open hand
    INSPIRE_HAND_CLOSED = 0       # Fully closed hand
    
    def __init__(self, left_hand_array, right_hand_array, 
                 dual_hand_data_lock=None, dual_hand_state_array=None,
                 dual_hand_action_array=None, fps=100.0, Unit_Test=False, 
                 simulation_mode=False,
                 network_interface="eno1",
                 left_hand_ip="192.168.123.211",
                 right_hand_ip="192.168.123.210"):
        
        logger_mp.info("Initialize Inspire_Bridge_Controller...")
        self.fps = fps
        self.Unit_Test = Unit_Test
        self.simulation_mode = simulation_mode
        
        # Bridge configuration
        self.network_interface = network_interface
        self.left_hand_ip = left_hand_ip
        self.right_hand_ip = right_hand_ip
        self.left_bridge_handler = None
        self.right_bridge_handler = None
        self.bridge_threads = []
        self.bridge_running = False
        
        # Shared arrays for hand commands (output from control process)
        self.left_hand_cmd_array = Array('d', Inspire_Num_Motors, lock=True)
        self.right_hand_cmd_array = Array('d', Inspire_Num_Motors, lock=True)
        # Initialize with open position
        for i in range(Inspire_Num_Motors):
            self.left_hand_cmd_array[i] = float(self.INSPIRE_HAND_OPEN)
            self.right_hand_cmd_array[i] = float(self.INSPIRE_HAND_OPEN)
        
        # Hand retargeting
        if not self.Unit_Test:
            self.hand_retargeting = HandRetargeting(HandType.INSPIRE_HAND)
        else:
            self.hand_retargeting = HandRetargeting(HandType.INSPIRE_HAND_Unit_Test)

        # Note: DDS is already initialized by arm controller
        # ChannelFactoryInitialize is NOT called here
        
        # Initialize hand command publishers (separate left/right)
        self.LeftHandCmd_publisher = ChannelPublisher(kTopicInspireLeftCommand, inspire_hand_ctrl)
        self.LeftHandCmd_publisher.Init()
        
        self.RightHandCmd_publisher = ChannelPublisher(kTopicInspireRightCommand, inspire_hand_ctrl)
        self.RightHandCmd_publisher.Init()
        
        logger_mp.info("Hand DDS publishers initialized")
        
        # Initialize hand state subscribers
        self.LeftHandState_subscriber = ChannelSubscriber(kTopicInspireLeftState, inspire_hand_state)
        self.LeftHandState_subscriber.Init()
        
        self.RightHandState_subscriber = ChannelSubscriber(kTopicInspireRightState, inspire_hand_state)
        self.RightHandState_subscriber.Init()
        
        # Shared Arrays for hand states
        self.left_hand_state_array = Array('d', Inspire_Num_Motors, lock=True)  
        self.right_hand_state_array = Array('d', Inspire_Num_Motors, lock=True)

        # Initialize bridges before starting control
        if not simulation_mode:
            self.init_bridges()
        else:
            logger_mp.info("Simulation mode - skipping bridge initialization")
        
        # Initialize subscribe thread
        self.subscribe_state_thread = threading.Thread(target=self._subscribe_hand_state)
        self.subscribe_state_thread.daemon = True
        self.subscribe_state_thread.start()

        # Wait for hand state subscription (with timeout for simulation)
        timeout = 20.0  # 20 second timeout
        start_time = time.time()
        while True:
            if any(self.right_hand_state_array):
                break
            if time.time() - start_time > timeout:
                if simulation_mode:
                    logger_mp.warning("[Inspire_Bridge_Controller] No hand state in simulation mode - continuing anyway")
                    break
                else:
                    logger_mp.error("[Inspire_Bridge_Controller] Timeout waiting for hand state - check hand connections")
                    break
            time.sleep(1)
            logger_mp.warning("[Inspire_Bridge_Controller] Waiting to subscribe hand state...")
        
        logger_mp.info("[Inspire_Bridge_Controller] Hand state subscription ready")

        # Start hand control process (computes targets, doesn't publish)
        hand_control_process = Process(
            target=self.control_process, 
            args=(left_hand_array, right_hand_array, self.left_hand_state_array, 
                  self.right_hand_state_array, self.left_hand_cmd_array, 
                  self.right_hand_cmd_array, dual_hand_data_lock, 
                  dual_hand_state_array, dual_hand_action_array))
        hand_control_process.daemon = True
        hand_control_process.start()
        
        # Start publishing thread in main process (reads from cmd arrays and publishes)
        self.publish_thread = threading.Thread(target=self._publish_hand_commands)
        self.publish_thread.daemon = True
        self.publish_thread.start()

        logger_mp.info("Initialize Inspire_Bridge_Controller OK!\n")

    def init_bridges(self):
        """Initialize bridges that connect DDS to physical hands via Modbus"""
        try:
            logger_mp.info("Initializing hand bridges...")
            
            # Create left hand bridge
            self.left_bridge_handler = inspire_sdk.ModbusDataHandler(
                ip=self.left_hand_ip,
                LR='l',
                device_id=1,
                initDDS=False,  # DDS already initialized by arm controller
                network=self.network_interface
            )
            logger_mp.info(f"Left hand bridge created (IP: {self.left_hand_ip})")
            
            # Create right hand bridge
            self.right_bridge_handler = inspire_sdk.ModbusDataHandler(
                ip=self.right_hand_ip,
                LR='r',
                device_id=1,
                initDDS=False,  # DDS already initialized by arm controller
                network=self.network_interface
            )
            logger_mp.info(f"Right hand bridge created (IP: {self.right_hand_ip})")
            
            # Start bridge threads
            self.start_bridge_threads()
            
        except Exception as e:
            logger_mp.error(f"Failed to initialize bridges: {e}")
            logger_mp.error("Hands will not be controlled - check hand IPs and network")
            raise

    def start_bridge_threads(self):
        """Start bridge threads for both hands"""
        self.bridge_running = True
        
        # Left hand bridge thread
        left_thread = threading.Thread(
            target=self.run_bridge,
            args=(self.left_bridge_handler, "Left Hand"),
            daemon=True
        )
        left_thread.start()
        self.bridge_threads.append(left_thread)
        
        # Right hand bridge thread
        right_thread = threading.Thread(
            target=self.run_bridge,
            args=(self.right_bridge_handler, "Right Hand"),
            daemon=True
        )
        right_thread.start()
        self.bridge_threads.append(right_thread)
        
        logger_mp.info("Both hand bridges started!")
        time.sleep(1)  # Let bridges initialize

    def run_bridge(self, handler, name):
        """Run a single bridge (DDS ↔ Modbus)"""
        logger_mp.info(f"{name} bridge thread started")
        
        while self.bridge_running:
            try:
                # This reads from physical hand and publishes state to DDS
                # Also listens for DDS commands and sends them to physical hands
                handler.read()
                time.sleep(0.02)  # ~50Hz bridge update
            except Exception as e:
                logger_mp.error(f"{name} bridge error: {e}")
                time.sleep(0.1)

    def stop_bridges(self):
        """Stop all bridge threads"""
        logger_mp.info("Stopping hand bridges...")
        self.bridge_running = False
        for thread in self.bridge_threads:
            thread.join(timeout=2)
        logger_mp.info("Hand bridges stopped")

    def _subscribe_hand_state(self):
        """Subscribe to hand state messages from both hands"""
        while True:
            try:
                # Read left hand state
                left_msg = self.LeftHandState_subscriber.Read()
                if left_msg is not None and hasattr(left_msg, 'angle_cur'):
                    for idx in range(min(Inspire_Num_Motors, len(left_msg.angle_cur))):
                        self.left_hand_state_array[idx] = left_msg.angle_cur[idx]
                
                # Read right hand state
                right_msg = self.RightHandState_subscriber.Read()
                if right_msg is not None and hasattr(right_msg, 'angle_cur'):
                    for idx in range(min(Inspire_Num_Motors, len(right_msg.angle_cur))):
                        self.right_hand_state_array[idx] = right_msg.angle_cur[idx]
                        
            except Exception as e:
                logger_mp.debug(f"Hand state read error: {e}")
                
            time.sleep(0.002)

    def _publish_hand_commands(self):
        """Publish hand commands from shared arrays"""
        logger_mp.info("Hand command publishing thread started")
        while True:
            try:
                left_q_target = np.array(self.left_hand_cmd_array[:])
                right_q_target = np.array(self.right_hand_cmd_array[:])
                self.ctrl_dual_hand(left_q_target, right_q_target)
            except Exception as e:
                logger_mp.error(f"Hand command publish error: {e}")
            time.sleep(1 / self.fps)

    def ctrl_dual_hand(self, left_q_target, right_q_target):
        """
        Send position commands to both hands using inspire_hand_ctrl message format
        
        Args:
            left_q_target: 6-element array of left hand joint targets (in inspire units 0-1000)
            right_q_target: 6-element array of right hand joint targets (in inspire units 0-1000)
        """
        # Create left hand command (angle mode)
        left_cmd = inspire_hand_defaut.get_inspire_hand_ctrl()
        left_cmd.angle_set = [int(angle) for angle in left_q_target]
        left_cmd.pos_set = [0] * Inspire_Num_Motors
        left_cmd.force_set = [0] * Inspire_Num_Motors
        left_cmd.speed_set = [0] * Inspire_Num_Motors
        left_cmd.mode = 0b0001  # Angle control mode
        
        # Create right hand command (angle mode)
        right_cmd = inspire_hand_defaut.get_inspire_hand_ctrl()
        right_cmd.angle_set = [int(angle) for angle in right_q_target]
        right_cmd.pos_set = [0] * Inspire_Num_Motors
        right_cmd.force_set = [0] * Inspire_Num_Motors
        right_cmd.speed_set = [0] * Inspire_Num_Motors
        right_cmd.mode = 0b0001  # Angle control mode
        
        # Publish commands
        self.LeftHandCmd_publisher.Write(left_cmd)
        self.RightHandCmd_publisher.Write(right_cmd)
    
    def control_process(self, left_hand_array, right_hand_array, left_hand_state_array, 
                       right_hand_state_array, left_hand_cmd_array, right_hand_cmd_array,
                       dual_hand_data_lock=None, 
                       dual_hand_state_array=None, dual_hand_action_array=None):
        """Main control loop that runs in separate process"""
        self.running = True

        # Initialize with fully open hands
        left_q_target = np.full(Inspire_Num_Motors, float(self.INSPIRE_HAND_OPEN))
        right_q_target = np.full(Inspire_Num_Motors, float(self.INSPIRE_HAND_OPEN))

        try:
            while self.running:
                start_time = time.time()
                
                # Get dual hand skeleton data from XR device
                with left_hand_array.get_lock():
                    left_hand_data = np.array(left_hand_array[:]).reshape(25, 3).copy()
                with right_hand_array.get_lock():
                    right_hand_data = np.array(right_hand_array[:]).reshape(25, 3).copy()

                # Read current hand states
                state_data = np.concatenate((
                    np.array(left_hand_state_array[:]), 
                    np.array(right_hand_state_array[:])
                ))

                # Check if hand data has been initialized (not all zeros and not default position)
                is_right_hand_valid = not np.all(right_hand_data == 0.0)
                is_left_hand_valid = not np.all(left_hand_data[4] == np.array([-1.13, 0.3, 0.15]))
                
                if is_right_hand_valid and is_left_hand_valid:
                    # Retarget hand skeleton to inspire hand angles
                    ref_left_value = left_hand_data[self.hand_retargeting.left_indices[1,:]] - \
                                    left_hand_data[self.hand_retargeting.left_indices[0,:]]
                    ref_right_value = right_hand_data[self.hand_retargeting.right_indices[1,:]] - \
                                     right_hand_data[self.hand_retargeting.right_indices[0,:]]

                    left_q_target = self.hand_retargeting.left_retargeting.retarget(ref_left_value)[
                        self.hand_retargeting.left_dex_retargeting_to_hardware]
                    right_q_target = self.hand_retargeting.right_retargeting.retarget(ref_right_value)[
                        self.hand_retargeting.right_dex_retargeting_to_hardware]

                    # Convert from radians to inspire hand units (0-1000)
                    # ...existing normalization code...
                    
                    def normalize(val, min_val, max_val):
                        """Normalize value to [0, 1000] with inversion (high rad = closed = low inspire value)"""
                        normalized = np.clip((max_val - val) / (max_val - min_val), 0.0, 1.0)
                        return normalized * 1000.0

                    for idx in range(Inspire_Num_Motors):
                        if idx <= 3:  # Finger joints
                            left_q_target[idx] = normalize(left_q_target[idx], 0.0, 1.7)
                            right_q_target[idx] = normalize(right_q_target[idx], 0.0, 1.7)
                        elif idx == 4:  # Thumb bend
                            left_q_target[idx] = normalize(left_q_target[idx], 0.0, 0.5)
                            right_q_target[idx] = normalize(right_q_target[idx], 0.0, 0.5)
                        elif idx == 5:  # Thumb rotation
                            left_q_target[idx] = normalize(left_q_target[idx], -0.1, 1.3)
                            right_q_target[idx] = normalize(right_q_target[idx], -0.1, 1.3)

                # Get dual hand action for recording
                action_data = np.concatenate((left_q_target, right_q_target))
                if dual_hand_state_array is not None and dual_hand_action_array is not None:
                    with dual_hand_data_lock:
                        dual_hand_state_array[:] = state_data
                        dual_hand_action_array[:] = action_data

                # Write commands to shared arrays (publishing thread will send them)
                with left_hand_cmd_array.get_lock():
                    left_hand_cmd_array[:] = left_q_target
                with right_hand_cmd_array.get_lock():
                    right_hand_cmd_array[:] = right_q_target
                
                # Maintain control frequency
                current_time = time.time()
                time_elapsed = current_time - start_time
                sleep_time = max(0, (1 / self.fps) - time_elapsed)
                time.sleep(sleep_time)
                
        except KeyboardInterrupt:
            logger_mp.info("Inspire_Bridge_Controller control process interrupted")
        finally:
            logger_mp.info("Inspire_Bridge_Controller control process has been closed.")


# Joint index mapping (same as original)
class Inspire_Right_Hand_JointIndex:
    kRightHandPinky = 0
    kRightHandRing = 1
    kRightHandMiddle = 2
    kRightHandIndex = 3
    kRightHandThumbBend = 4
    kRightHandThumbRotation = 5

class Inspire_Left_Hand_JointIndex:
    kLeftHandPinky = 6
    kLeftHandRing = 7
    kLeftHandMiddle = 8
    kLeftHandIndex = 9
    kLeftHandThumbBend = 10
    kLeftHandThumbRotation = 11