import numpy as np
import time
import argparse
import cv2
from multiprocessing import shared_memory, Value, Array, Lock
import threading
import logging_mp
logging_mp.basic_config(level=logging_mp.INFO)
logger_mp = logging_mp.get_logger(__name__)

import os 
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

import yaml

from televuer import TeleVuerWrapper
from teleop.robot_control.robot_arm import G1_29_ArmController, G1_23_ArmController, H1_2_ArmController, H1_ArmController
from teleop.robot_control.robot_arm_ik import G1_29_ArmIK, G1_23_ArmIK, H1_2_ArmIK, H1_ArmIK
from teleop.robot_control.robot_hand_unitree import Dex3_1_Controller, Dex1_1_Gripper_Controller
from teleop.robot_control.robot_hand_inspire import Inspire_Controller
from teleop.robot_control.robot_hand_inspire_bridge import Inspire_Bridge_Controller
from teleop.robot_control.robot_hand_brainco import Brainco_Controller
from teleop.image_server.image_client import ImageClient
from teleop.utils.episode_writer import EpisodeWriter
from teleop.utils.episode_writer_hdf5 import EpisodeWriterHDF5
from teleop.utils.realsense_multi_capture import MultiRealsenseCapture
from sshkeyboard import listen_keyboard, stop_listening

# for simulation
from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
def publish_reset_category(category: int,publisher): # Scene Reset signal
    msg = String_(data=str(category))
    publisher.Write(msg)
    logger_mp.info(f"published reset category: {category}")

# state transition
start_signal = False
running = True
should_toggle_recording = False
is_recording = False
def on_press(key):
    global running, start_signal, should_toggle_recording
    if key == 'r':
        start_signal = True
        logger_mp.info("Program start signal received.")
    elif key == 'q' and start_signal == True:
        logger_mp.info("=" * 60)
        logger_mp.info("QUIT: 'q' pressed - Initiating shutdown sequence...")
        logger_mp.info("=" * 60)
        stop_listening()
        running = False
    elif key == 's' and start_signal == True:
        should_toggle_recording = True
    else:
        logger_mp.info(f"{key} was pressed, but no action is defined for this key.")
listen_keyboard_thread = threading.Thread(target=listen_keyboard, kwargs={"on_press": on_press, "until": None, "sequential": False,}, daemon=True)
listen_keyboard_thread.start()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--task_dir', type = str, default = './utils/data/', help = 'path to save data')
    parser.add_argument('--frequency', type = float, default = 60.0, help = 'control loop frequency (Hz)')
    parser.add_argument('--recording-frequency', type = float, default = 30.0, help = 'data recording frequency (Hz), can be lower than control frequency')

    # basic control parameters
    parser.add_argument('--xr-mode', type=str, choices=['hand', 'controller'], default='hand', help='Select XR device tracking source')
    parser.add_argument('--arm', type=str, choices=['G1_29', 'G1_23', 'H1_2', 'H1'], default='G1_29', help='Select arm controller')
    parser.add_argument('--ee', type=str, choices=['dex1', 'dex3', 'inspire1', 'brainco'], help='Select end effector controller')
    # mode flags
    parser.add_argument('--motion', action = 'store_true', help = 'Enable motion control mode')
    parser.add_argument('--headless', action='store_true', help='Enable headless mode (no display)')
    parser.add_argument('--sim', action = 'store_true', help = 'Enable isaac simulation mode')
    parser.add_argument('--record', action = 'store_true', help = 'Enable data recording')
    parser.add_argument('--task-name', type = str, default = 'pick cube', help = 'task name for recording')
    parser.add_argument('--task-goal', type = str, default = 'e.g. pick the red cube on the table.', help = 'task goal for recording')

    # multi-camera parameters
    parser.add_argument('--use-multi-camera', action='store_true', help='Enable multi-camera capture from laptop-connected RealSense cameras')
    parser.add_argument('--camera-config', type=str, default='./config/camera_config.yaml', help='Path to camera configuration file')

    # inspire hand bridge parameters (for laptop-connected hands)
    parser.add_argument('--inspire-bridge', action='store_true', help='Use bridge mode for Inspire hands (hands connected to laptop via network)')
    parser.add_argument('--network-interface', type=str, default='eno1', help='Network interface for hand bridge (e.g., eno1, eth0, wlan0)')
    parser.add_argument('--left-hand-ip', type=str, default='192.168.123.211', help='IP address of left Inspire hand')
    parser.add_argument('--right-hand-ip', type=str, default='192.168.123.210', help='IP address of right Inspire hand')

    args = parser.parse_args()
    logger_mp.info(f"args: {args}")

    # image client: img_config should be the same as the configuration in image_server.py (of Robot's development computing unit)
    if args.sim:
        img_config = {
            'fps': 30,
            'head_camera_type': 'realsense',
            'head_camera_image_shape': [480, 640],  # Head camera resolution
            'head_camera_id_numbers': [0],
            'wrist_camera_type': 'opencv',
            'wrist_camera_image_shape': [480, 640],  # Wrist camera resolution
            'wrist_camera_id_numbers': [2, 4],
        }
    else:
        # Real hardware configuration
        if args.inspire_bridge:
            # When using inspire bridge, only head camera is available (no wrist cameras)
            img_config = {
                'fps': 30,
                'head_camera_type': 'realsense',
                'head_camera_image_shape': [480, 640],  # Single head camera resolution
                'head_camera_id_numbers': [0],  # ONE head camera from robot
                # No wrist cameras in inspire bridge mode
            }
        else:
            # Standard real hardware - single head camera from robot
            # Wrist cameras come from laptop via --use-multi-camera flag
            img_config = {
                'fps': 30,
                'head_camera_type': 'realsense',
                'head_camera_image_shape': [480, 640],  # Single head camera resolution
                'head_camera_id_numbers': [0],  # ONE head camera from robot
                # No wrist cameras in config - use --use-multi-camera for wrist cams
            }


    ASPECT_RATIO_THRESHOLD = 2.0 # If the aspect ratio exceeds this value, it is considered binocular
    if len(img_config['head_camera_id_numbers']) > 1 or (img_config['head_camera_image_shape'][1] / img_config['head_camera_image_shape'][0] > ASPECT_RATIO_THRESHOLD):
        BINOCULAR = True
    else:
        BINOCULAR = False
    if 'wrist_camera_type' in img_config:
        WRIST = True
    else:
        WRIST = False
    
    if BINOCULAR and not (img_config['head_camera_image_shape'][1] / img_config['head_camera_image_shape'][0] > ASPECT_RATIO_THRESHOLD):
        tv_img_shape = (img_config['head_camera_image_shape'][0], img_config['head_camera_image_shape'][1] * 2, 3)
    else:
        tv_img_shape = (img_config['head_camera_image_shape'][0], img_config['head_camera_image_shape'][1], 3)

    tv_img_shm = shared_memory.SharedMemory(create = True, size = np.prod(tv_img_shape) * np.uint8().itemsize)
    tv_img_array = np.ndarray(tv_img_shape, dtype = np.uint8, buffer = tv_img_shm.buf)

    if WRIST and args.sim:
        wrist_img_shape = (img_config['wrist_camera_image_shape'][0], img_config['wrist_camera_image_shape'][1] * 2, 3)
        wrist_img_shm = shared_memory.SharedMemory(create = True, size = np.prod(wrist_img_shape) * np.uint8().itemsize)
        wrist_img_array = np.ndarray(wrist_img_shape, dtype = np.uint8, buffer = wrist_img_shm.buf)
        img_client = ImageClient(tv_img_shape = tv_img_shape, tv_img_shm_name = tv_img_shm.name, 
                                 wrist_img_shape = wrist_img_shape, wrist_img_shm_name = wrist_img_shm.name, server_address="127.0.0.1")
    elif WRIST and not args.sim:
        wrist_img_shape = (img_config['wrist_camera_image_shape'][0], img_config['wrist_camera_image_shape'][1] * 2, 3)
        wrist_img_shm = shared_memory.SharedMemory(create = True, size = np.prod(wrist_img_shape) * np.uint8().itemsize)
        wrist_img_array = np.ndarray(wrist_img_shape, dtype = np.uint8, buffer = wrist_img_shm.buf)
        img_client = ImageClient(tv_img_shape = tv_img_shape, tv_img_shm_name = tv_img_shm.name, 
                                 wrist_img_shape = wrist_img_shape, wrist_img_shm_name = wrist_img_shm.name)
    else:
        img_client = ImageClient(tv_img_shape = tv_img_shape, tv_img_shm_name = tv_img_shm.name)

    image_receive_thread = threading.Thread(target = img_client.receive_process, daemon = True)
    image_receive_thread.daemon = True
    image_receive_thread.start()

    # Initialize multi-camera capture system (laptop-connected RealSense cameras)
    multi_camera_capture = None
    if args.use_multi_camera:
        logger_mp.info("=" * 60)
        logger_mp.info("MULTI-CAMERA SETUP: Loading configuration...")
        logger_mp.info("=" * 60)
        
        # Load camera configuration
        camera_config_path = os.path.join(current_dir, args.camera_config)
        if not os.path.exists(camera_config_path):
            logger_mp.error(f"Camera config file not found: {camera_config_path}")
            logger_mp.error("Please create config/camera_config.yaml with your camera serial numbers")
            logger_mp.error("Run: python3 teleop/scripts/detect_cameras.py to find serial numbers")
        else:
            with open(camera_config_path, 'r') as f:
                camera_yaml = yaml.safe_load(f)
            
            camera_config = camera_yaml.get('cameras', {})
            camera_settings = camera_yaml.get('settings', {})
            
            # Filter out placeholder serial numbers
            active_cameras = {name: serial for name, serial in camera_config.items() 
                            if serial != "REPLACE_WITH_SERIAL" and serial}
            
            if len(active_cameras) == 0:
                logger_mp.warning("No valid camera serial numbers found in config")
                logger_mp.warning("Please update config/camera_config.yaml with actual serial numbers")
                logger_mp.warning("Run: python3 teleop/scripts/detect_cameras.py to find them")
            else:
                logger_mp.info(f"Found {len(active_cameras)} configured RealSense camera(s):")
                for name, serial in active_cameras.items():
                    logger_mp.info(f"  - {name}: {serial}")
                
                # Initialize multi-camera capture
                multi_camera_capture = MultiRealsenseCapture(
                    camera_config=active_cameras,
                    fps=camera_settings.get('fps', 30),
                    width=camera_settings.get('width', 640),
                    height=camera_settings.get('height', 480)
                )
                
                # Start capture (runs in background threads)
                success = multi_camera_capture.start()
                if success:
                    logger_mp.info("=" * 60)
                    logger_mp.info("MULTI-CAMERA: RealSense capture started successfully")
                    logger_mp.info(f"Active cameras: {multi_camera_capture.get_active_cameras()}")
                    logger_mp.info("=" * 60)
                else:
                    logger_mp.error("Failed to start RealSense multi-camera capture")
                    multi_camera_capture = None

    # Initialize DDS with network interface if using inspire bridge mode
    dds_already_initialized = False
    if args.ee == "inspire1" and args.inspire_bridge:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        logger_mp.info(f"Initializing DDS with network interface: {args.network_interface}")
        if args.sim:
            ChannelFactoryInitialize(1, args.network_interface)
        else:
            ChannelFactoryInitialize(0, args.network_interface)
        logger_mp.info("DDS initialized with network interface for hand bridge")
        dds_already_initialized = True

    # television: obtain hand pose data from the XR device and transmit the robot's head camera image to the XR device.
    tv_wrapper = TeleVuerWrapper(binocular=BINOCULAR, use_hand_tracking=args.xr_mode == "hand", img_shape=tv_img_shape, img_shm_name=tv_img_shm.name, 
                                 return_state_data=True, return_hand_rot_data = False)

    # arm
    if args.arm == "G1_29":
        arm_ik = G1_29_ArmIK()
        arm_ctrl = G1_29_ArmController(motion_mode=args.motion, simulation_mode=args.sim, dds_already_initialized=args.inspire_bridge)
    elif args.arm == "G1_23":
        arm_ik = G1_23_ArmIK()
        arm_ctrl = G1_23_ArmController(motion_mode=args.motion, simulation_mode=args.sim, dds_already_initialized=args.inspire_bridge)
    elif args.arm == "H1_2":
        arm_ik = H1_2_ArmIK()
        arm_ctrl = H1_2_ArmController(simulation_mode=args.sim, dds_already_initialized=args.inspire_bridge)
    elif args.arm == "H1":
        arm_ik = H1_ArmIK()
        arm_ctrl = H1_ArmController(simulation_mode=args.sim, dds_already_initialized=args.inspire_bridge)

    # end-effector
    if args.ee == "dex3":
        left_hand_pos_array = Array('d', 75, lock = True)      # [input]
        right_hand_pos_array = Array('d', 75, lock = True)     # [input]
        dual_hand_data_lock = Lock()
        dual_hand_state_array = Array('d', 14, lock = False)   # [output] current left, right hand state(14) data.
        dual_hand_action_array = Array('d', 14, lock = False)  # [output] current left, right hand action(14) data.
        hand_ctrl = Dex3_1_Controller(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim)
    elif args.ee == "dex1":
        left_gripper_value = Value('d', 0.0, lock=True)        # [input]
        right_gripper_value = Value('d', 0.0, lock=True)       # [input]
        dual_gripper_data_lock = Lock()
        dual_gripper_state_array = Array('d', 2, lock=False)   # current left, right gripper state(2) data.
        dual_gripper_action_array = Array('d', 2, lock=False)  # current left, right gripper action(2) data.
        gripper_ctrl = Dex1_1_Gripper_Controller(left_gripper_value, right_gripper_value, dual_gripper_data_lock, dual_gripper_state_array, dual_gripper_action_array, simulation_mode=args.sim)
    elif args.ee == "inspire1":
        left_hand_pos_array = Array('d', 75, lock = True)      # [input]
        right_hand_pos_array = Array('d', 75, lock = True)     # [input]
        dual_hand_data_lock = Lock()
        dual_hand_state_array = Array('d', 12, lock = False)   # [output] current left, right hand state(12) data.
        dual_hand_action_array = Array('d', 12, lock = False)  # [output] current left, right hand action(12) data.
        # Choose controller based on bridge mode
        if args.inspire_bridge:
            logger_mp.info("Using Inspire Bridge Controller (hands connected to laptop)")
            hand_ctrl = Inspire_Bridge_Controller(
                left_hand_pos_array, right_hand_pos_array, 
                dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array, 
                simulation_mode=args.sim,
                network_interface=args.network_interface,
                left_hand_ip=args.left_hand_ip,
                right_hand_ip=args.right_hand_ip
            )
            
            # Verification routine: Test hand open/close to confirm it works
            if not args.sim:
                logger_mp.info("=" * 60)
                logger_mp.info("INSPIRE BRIDGE VERIFICATION: Testing hand open/close...")
                logger_mp.info("=" * 60)
                
                # Test sequence: Open -> Close -> Open -> Half
                test_positions = [
                    (1000, "FULLY OPEN"),
                    (0, "FULLY CLOSED"),
                    (1000, "FULLY OPEN"),
                    (500, "HALF OPEN"),
                ]
                
                for position, description in test_positions:
                    logger_mp.info(f"Testing hands: {description} (position={position})")
                    # Create target arrays for all 6 joints per hand
                    import numpy as np
                    left_target = np.full(6, float(position))
                    right_target = np.full(6, float(position))
                    
                    # Send command to both hands
                    hand_ctrl.ctrl_dual_hand(left_target, right_target)
                    
                    # Wait to allow movement
                    time.sleep(2.0)
                    
                    # Read and display current state
                    with dual_hand_data_lock:
                        left_state = dual_hand_state_array[:6]
                        right_state = dual_hand_state_array[6:]
                    logger_mp.info(f"  Left hand state:  {[f'{s:.0f}' for s in left_state]}")
                    logger_mp.info(f"  Right hand state: {[f'{s:.0f}' for s in right_state]}")
                
                logger_mp.info("=" * 60)
                logger_mp.info("VERIFICATION COMPLETE - Hands opened to default position")
                logger_mp.info("=" * 60)
        else:
            logger_mp.info("Using standard Inspire Controller (hands connected to PC2)")
            hand_ctrl = Inspire_Controller(
                left_hand_pos_array, right_hand_pos_array, 
                dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array, 
                simulation_mode=args.sim
            )

    elif args.ee == "brainco":
        left_hand_pos_array = Array('d', 75, lock = True)      # [input]
        right_hand_pos_array = Array('d', 75, lock = True)     # [input]
        dual_hand_data_lock = Lock()
        dual_hand_state_array = Array('d', 12, lock = False)   # [output] current left, right hand state(12) data.
        dual_hand_action_array = Array('d', 12, lock = False)  # [output] current left, right hand action(12) data.
        hand_ctrl = Brainco_Controller(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim)
    else:
        pass

    # simulation mode
    if args.sim:
        reset_pose_publisher = ChannelPublisher("rt/reset_pose/cmd", String_)
        reset_pose_publisher.Init()
        from teleop.utils.sim_state_topic import start_sim_state_subscribe
        sim_state_subscriber = start_sim_state_subscribe()

    # controller + motion mode
    if args.xr_mode == "controller" and args.motion:
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
        sport_client = LocoClient()
        sport_client.SetTimeout(0.0001)
        sport_client.Init()
    
    # record + headless mode
    # Recording runs at recording_frequency (default 30Hz) while control runs at frequency (default 60Hz)
    recording_frame_accumulator = 0.0  # For sub-sampling
    if args.record and args.headless:
        # Use HDF5 writer for msc_humanoid_visual compatibility
        recorder = EpisodeWriterHDF5(
            save_dir=args.task_dir + args.task_name,
            robot_name=args.arm,
            fps=args.recording_frequency
        )
        logger_mp.info(f"HDF5 Episode recorder initialized (headless mode)")
        logger_mp.info(f"  Control at {args.frequency}Hz, recording at {args.recording_frequency}Hz")
    elif args.record and not args.headless:
        # Use HDF5 writer for msc_humanoid_visual compatibility
        recorder = EpisodeWriterHDF5(
            save_dir=args.task_dir + args.task_name,
            robot_name=args.arm,
            fps=args.recording_frequency
        )
        logger_mp.info(f"HDF5 Episode recorder initialized")
        logger_mp.info(f"  Control at {args.frequency}Hz, recording at {args.recording_frequency}Hz")
        
    try:
        logger_mp.info("Please enter the start signal (enter 'r' to start the subsequent program)")
        while not start_signal:
            time.sleep(0.01)
        arm_ctrl.speed_gradual_max()
        while running:
            start_time = time.time()

            if not args.headless:
                tv_resized_image = cv2.resize(tv_img_array, (tv_img_shape[1] // 2, tv_img_shape[0] // 2))
                cv2.imshow("record image", tv_resized_image)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    stop_listening()
                    running = False
                    if args.sim:
                        publish_reset_category(2, reset_pose_publisher)
                elif key == ord('s'):
                    should_toggle_recording = True
                elif key == ord('a'):
                    if args.sim:
                        publish_reset_category(2, reset_pose_publisher)

            if args.record and should_toggle_recording:
                should_toggle_recording = False
                if not is_recording:
                    recorder.start_recording()
                    recording_frame_accumulator = 0.0  # Reset accumulator for new recording
                    is_recording = True
                    logger_mp.info(f"==> Recording started (at {args.recording_frequency}Hz)")
                else:
                    recorder.stop_recording()
                    is_recording = False
                    logger_mp.info("==> Recording stopped and saved")
                    if args.sim:
                        publish_reset_category(1, reset_pose_publisher)
            # get input data
            tele_data = tv_wrapper.get_motion_state_data()
            if (args.ee == "dex3" or args.ee == "inspire1" or args.ee == "brainco") and args.xr_mode == "hand":
                with left_hand_pos_array.get_lock():
                    left_hand_pos_array[:] = tele_data.left_hand_pos.flatten()
                with right_hand_pos_array.get_lock():
                    right_hand_pos_array[:] = tele_data.right_hand_pos.flatten()
            elif args.ee == "dex1" and args.xr_mode == "controller":
                with left_gripper_value.get_lock():
                    left_gripper_value.value = tele_data.left_trigger_value
                with right_gripper_value.get_lock():
                    right_gripper_value.value = tele_data.right_trigger_value
            elif args.ee == "dex1" and args.xr_mode == "hand":
                with left_gripper_value.get_lock():
                    left_gripper_value.value = tele_data.left_pinch_value
                with right_gripper_value.get_lock():
                    right_gripper_value.value = tele_data.right_pinch_value
            else:
                pass        
            
            # high level control
            if args.xr_mode == "controller" and args.motion:
                # quit teleoperate
                if tele_data.tele_state.right_aButton:
                    stop_listening()
                    running = False
                # command robot to enter damping mode. soft emergency stop function
                if tele_data.tele_state.left_thumbstick_state and tele_data.tele_state.right_thumbstick_state:
                    sport_client.Damp()
                # control, limit velocity to within 0.3
                sport_client.Move(-tele_data.tele_state.left_thumbstick_value[1]  * 0.3,
                                  -tele_data.tele_state.left_thumbstick_value[0]  * 0.3,
                                  -tele_data.tele_state.right_thumbstick_value[0] * 0.3)

            # get current robot state data.
            current_lr_arm_q  = arm_ctrl.get_current_dual_arm_q()
            current_lr_arm_dq = arm_ctrl.get_current_dual_arm_dq()

            # solve ik using motor data and wrist pose, then use ik results to control arms.
            time_ik_start = time.time()
            sol_q, sol_tauff  = arm_ik.solve_ik(tele_data.left_arm_pose, tele_data.right_arm_pose, current_lr_arm_q, current_lr_arm_dq)
            time_ik_end = time.time()
            logger_mp.debug(f"ik:\t{round(time_ik_end - time_ik_start, 6)}")
            arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)

            # record data (sub-sampled to recording_frequency)
            # Control runs at args.frequency (e.g., 60Hz), recording at args.recording_frequency (e.g., 30Hz)
            if args.record and is_recording:
                # Accumulate frames based on recording rate ratio
                # e.g., 30Hz recording / 60Hz control = 0.5, so record every 2nd control frame
                frame_increment = args.recording_frequency / args.frequency
                recording_frame_accumulator += frame_increment
                
                if recording_frame_accumulator >= 1.0:
                    recording_frame_accumulator -= 1.0
                    
                    # Get hand state and actions
                    if args.ee == "dex3" and args.xr_mode == "hand":
                        with dual_hand_data_lock:
                            hand_state = np.array(dual_hand_state_array[:])  # [14] - left+right
                            hand_action = np.array(dual_hand_action_array[:])
                    elif args.ee == "dex1":
                        with dual_gripper_data_lock:
                            hand_state = np.array(dual_gripper_state_array[:])  # [2] - left+right
                            hand_action = np.array(dual_gripper_action_array[:])
                    elif (args.ee == "inspire1" or args.ee == "brainco") and args.xr_mode == "hand":
                        with dual_hand_data_lock:
                            hand_state = np.array(dual_hand_state_array[:])  # [12] - left+right
                            hand_action = np.array(dual_hand_action_array[:])
                    else:
                        hand_state = np.array([])
                        hand_action = np.array([])
                    
                    # Combine arm and hand states into full qpos/qvel/action
                    full_qpos = np.concatenate([current_lr_arm_q, hand_state])
                    full_qvel = np.concatenate([current_lr_arm_dq, np.zeros_like(hand_state)])  # Hand velocities not available
                    full_action = np.concatenate([sol_q, hand_action])
                    
                    # Prepare camera images in msc_humanoid_visual format
                    images = {}
                    current_tv_image = tv_img_array.copy()
                    
                    if BINOCULAR:
                        # Split binocular image into left and right
                        images["ego_cam"] = current_tv_image[:, :tv_img_shape[1]//2]  # Left eye
                        # Could also save right eye as separate camera if needed
                    else:
                        images["ego_cam"] = current_tv_image
                    
                    # Add wrist cameras if available
                    if WRIST:
                        current_wrist_image = wrist_img_array.copy()
                        images["cam_left_wrist"] = current_wrist_image[:, :wrist_img_shape[1]//2]
                        images["cam_right_wrist"] = current_wrist_image[:, wrist_img_shape[1]//2:]
                    
                    # Add timestep to HDF5 episode
                    recorder.add_timestep(
                        qpos=full_qpos,
                        qvel=full_qvel,
                        action=full_action,
                        images=images
                    )

            current_time = time.time()
            time_elapsed = current_time - start_time
            sleep_time = max(0, (1 / args.frequency) - time_elapsed)
            time.sleep(sleep_time)
            logger_mp.debug(f"main process sleep: {sleep_time}")

    except KeyboardInterrupt:
        logger_mp.info("=" * 60)
        logger_mp.info("SHUTDOWN: KeyboardInterrupt received, exiting program...")
        logger_mp.info("=" * 60)
    finally:
        logger_mp.info("Shutting down robot control...")
        arm_ctrl.ctrl_dual_arm_go_home()
        if args.sim:
            sim_state_subscriber.stop_subscribe()
        tv_img_shm.close()
        tv_img_shm.unlink()
        if WRIST:
            wrist_img_shm.close()
            wrist_img_shm.unlink()
        
        # Stop multi-camera capture if running
        if multi_camera_capture is not None:
            logger_mp.info("Stopping RealSense multi-camera capture...")
            multi_camera_capture.stop()
        
        if args.record:
            # Stop and save any active recording
            if recorder.is_recording():
                logger_mp.info("=" * 60)
                logger_mp.info("SAVING EPISODE: Please wait, this may take 30-60 seconds...")
                logger_mp.info("DO NOT interrupt or data will be lost!")
                logger_mp.info("=" * 60)
                recorder.stop_recording()
                logger_mp.info("=" * 60)
                logger_mp.info("EPISODE SAVED SUCCESSFULLY!")
                logger_mp.info("=" * 60)
        listen_keyboard_thread.join()
        logger_mp.info("Program exited cleanly.")
        exit(0)
