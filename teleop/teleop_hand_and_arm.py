import numpy as np
import time
import argparse
import cv2
import struct
from multiprocessing import shared_memory, Value, Array, Lock
import threading
import concurrent.futures
import logging_mp
logging_mp.basic_config(level=logging_mp.INFO)
logger_mp = logging_mp.get_logger(__name__)

import os 
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from televuer import TeleVuerWrapper
from teleop.robot_control.robot_arm import G1_29_ArmController, G1_23_ArmController, H1_2_ArmController, H1_ArmController
from teleop.robot_control.robot_arm_ik import G1_29_ArmIK, G1_23_ArmIK, H1_2_ArmIK, H1_ArmIK
from teleop.robot_control.robot_hand_unitree import Dex3_1_Controller, Dex1_1_Gripper_Controller
# from teleop.robot_control.robot_hand_inspire import Inspire_Controller
# from teleop.robot_control.robot_hand_inspire_bridge import Inspire_Bridge_Controller
# from teleop.robot_control.robot_hand_brainco import Brainco_Controller
from teleop.image_server.image_client import ImageClient
from teleop.utils.episode_writer import EpisodeWriter
from teleop.utils.episode_writer_hdf5 import EpisodeWriterHDF5
from sshkeyboard import listen_keyboard, stop_listening

# for simulation
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
try:
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_
    DEX3_BINARY_AVAILABLE = True
except Exception:
    DEX3_BINARY_AVAILABLE = False
# Note: Locomotion data is extracted from lowstate (via arm controller) - no separate subscription needed
def publish_reset_category(category: int,publisher): # Scene Reset signal
    msg = String_(data=str(category))
    publisher.Write(msg)
    logger_mp.info(f"published reset category: {category}")

# ============================================================================
# LOWSTATE PARSING HELPERS
# Extract locomotion data from lowstate (IMU, controller inputs)
# ============================================================================

def parse_wireless_remote(wireless_remote):
    """
    Parse wireless_remote bytes (40 bytes) into controller inputs.
    Based on Unitree SDK example: wireless_controller.py
    
    Returns:
        loco_action: np.array [20] - [Lx, Ly, Rx, Ry, L1, L2, R1, R2, A, B, X, Y, Up, Down, Left, Right, Select, Start, F1, F3]
    """
    # Joystick values (float, -1 to 1)
    Lx = struct.unpack('<f', bytes(wireless_remote[4:8]))[0]
    Rx = struct.unpack('<f', bytes(wireless_remote[8:12]))[0]
    Ry = struct.unpack('<f', bytes(wireless_remote[12:16]))[0]
    Ly = struct.unpack('<f', bytes(wireless_remote[20:24]))[0]
    
    # Button states (from bytes 2 and 3)
    data1, data2 = wireless_remote[2], wireless_remote[3]
    R1 = float((data1 >> 0) & 1)
    L1 = float((data1 >> 1) & 1)
    Start = float((data1 >> 2) & 1)
    Select = float((data1 >> 3) & 1)
    R2 = float((data1 >> 4) & 1)
    L2 = float((data1 >> 5) & 1)
    F1 = float((data1 >> 6) & 1)
    F3 = float((data1 >> 7) & 1)
    A = float((data2 >> 0) & 1)
    B = float((data2 >> 1) & 1)
    X = float((data2 >> 2) & 1)
    Y = float((data2 >> 3) & 1)
    Up = float((data2 >> 4) & 1)
    Right = float((data2 >> 5) & 1)
    Down = float((data2 >> 6) & 1)
    Left = float((data2 >> 7) & 1)
    
    # Return as [20] array: joysticks (4) + buttons (16)
    return np.array([
        Lx, Ly, Rx, Ry,  # Joysticks
        L1, L2, R1, R2, A, B, X, Y, Up, Down, Left, Right, Select, Start, F1, F3  # Buttons
    ], dtype=np.float32)


def parse_lowstate_to_loco_data(lowstate_msg, robot_type='G1_29'):
    """
    Parse raw lowstate message into loco_state and loco_action arrays.
    
    Args:
        lowstate_msg: Raw lowstate message from arm_ctrl.get_lowstate_raw()
        robot_type: Robot type for determining knee joint indices
    
    Returns:
        loco_state: np.array [17] - [mode, rpy(3), quat(4), accel(3), gyro(3), leg_joints(3)]
        loco_action: np.array [20] - [joysticks(4), buttons(16)]
        or (None, None) if lowstate_msg is None
    """
    if lowstate_msg is None:
        return None, None
    
    # Extract IMU data
    imu = lowstate_msg.imu_state
    mode_machine = float(lowstate_msg.mode_machine)
    
    # RPY (roll, pitch, yaw) - in radians
    rpy = np.array([imu.rpy[0], imu.rpy[1], imu.rpy[2]], dtype=np.float32)
    
    # Quaternion (w, x, y, z)
    quat = np.array([imu.quaternion[0], imu.quaternion[1], imu.quaternion[2], imu.quaternion[3]], dtype=np.float32)
    
    # Accelerometer (x, y, z)
    accel = np.array([imu.accelerometer[0], imu.accelerometer[1], imu.accelerometer[2]], dtype=np.float32)
    
    # Gyroscope (x, y, z)
    gyro = np.array([imu.gyroscope[0], imu.gyroscope[1], imu.gyroscope[2]], dtype=np.float32)
    
    # Leg joint positions (knee joints as height proxy)
    # G1_29: left knee = 3, right knee = 9 (based on G1_29_JointIndex)
    # G1_23: similar indices
    if robot_type in ['G1_29', 'G1_23']:
        left_knee = lowstate_msg.motor_state[3].q if len(lowstate_msg.motor_state) > 9 else 0.0
        right_knee = lowstate_msg.motor_state[9].q if len(lowstate_msg.motor_state) > 9 else 0.0
    else:  # H1_2, H1
        left_knee = lowstate_msg.motor_state[3].q if len(lowstate_msg.motor_state) > 9 else 0.0
        right_knee = lowstate_msg.motor_state[9].q if len(lowstate_msg.motor_state) > 9 else 0.0
    avg_knee = (left_knee + right_knee) / 2.0
    leg_joints = np.array([left_knee, right_knee, avg_knee], dtype=np.float32)
    
    # Combine into loco_state [17]
    loco_state = np.concatenate([
        [mode_machine],  # [0]
        rpy,             # [1:4]
        quat,            # [4:8]
        accel,           # [8:11]
        gyro,            # [11:14]
        leg_joints       # [14:17]
    ])
    
    # Parse wireless_remote for controller inputs
    loco_action = parse_wireless_remote(lowstate_msg.wireless_remote)
    
    return loco_state, loco_action

# ============================================================================
# RESET POSE CONFIGURATION
# Robot-specific reset poses (arms forward at chest height)
# These are 4x4 SE3 transformation matrices for wrist targets
# ============================================================================

# Pose matching threshold: 30 degrees in radians for each arm joint
POSE_MATCH_THRESHOLD_RAD = np.deg2rad(30)  # ~0.524 radians

def get_reset_wrist_poses(robot_type):
    """
    Get the reset wrist poses (SE3 matrices) for a given robot type.
    Arms forward at chest height, palms facing down/inward.
    
    Returns: (left_wrist_pose, right_wrist_pose) as 4x4 numpy arrays
    """
    if robot_type == "G1_29":
        # G1_29: Arms forward at chest height
        left_pose = np.array([
            [1, 0, 0, 0.30],   # Forward 30cm
            [0, 1, 0, 0.25],   # Left 25cm from center
            [0, 0, 1, 0.05],   # Slightly above waist
            [0, 0, 0, 1]
        ], dtype=np.float64)
        right_pose = np.array([
            [1, 0, 0, 0.30],   # Forward 30cm
            [0, 1, 0, -0.25],  # Right 25cm from center
            [0, 0, 1, 0.05],   # Slightly above waist
            [0, 0, 0, 1]
        ], dtype=np.float64)
    elif robot_type == "G1_23":
        # G1_23: Similar but adjusted for different arm config
        left_pose = np.array([
            [1, 0, 0, 0.30],
            [0, 1, 0, 0.25],
            [0, 0, 1, 0.05],
            [0, 0, 0, 1]
        ], dtype=np.float64)
        right_pose = np.array([
            [1, 0, 0, 0.30],
            [0, 1, 0, -0.25],
            [0, 0, 1, 0.05],
            [0, 0, 0, 1]
        ], dtype=np.float64)
    elif robot_type == "H1_2":
        # H1_2: Taller robot, arms forward at chest height
        # H1_2 uses scale_arms internally (human_arm_length=0.60, robot_arm_length=0.75)
        # So we specify poses in human scale, they get scaled up
        left_pose = np.array([
            [1, 0, 0, 0.25],   # Forward (will be scaled)
            [0, 1, 0, 0.25],   # Left
            [0, 0, 1, 0.10],   # Chest height
            [0, 0, 0, 1]
        ], dtype=np.float64)
        right_pose = np.array([
            [1, 0, 0, 0.25],
            [0, 1, 0, -0.25],
            [0, 0, 1, 0.10],
            [0, 0, 0, 1]
        ], dtype=np.float64)
    elif robot_type == "H1":
        # H1: Simpler arm configuration
        left_pose = np.array([
            [1, 0, 0, 0.25],
            [0, 1, 0, 0.25],
            [0, 0, 1, 0.10],
            [0, 0, 0, 1]
        ], dtype=np.float64)
        right_pose = np.array([
            [1, 0, 0, 0.25],
            [0, 1, 0, -0.25],
            [0, 0, 1, 0.10],
            [0, 0, 0, 1]
        ], dtype=np.float64)
    else:
        raise ValueError(f"Unknown robot type: {robot_type}")
    
    return left_pose, right_pose

def check_arm_pose_match(user_arm_q, reset_arm_q, threshold_rad):
    """
    Check if all arm joints are within threshold.
    Only compares arm joints, ignores hand/finger joints.
    
    Args:
        user_arm_q: Current IK solution from user tracking (arm joints only)
        reset_arm_q: IK solution for reset pose (arm joints only)
        threshold_rad: Maximum allowed difference per joint in radians
    
    Returns:
        (is_matched, max_diff_deg): Tuple of match status and max difference in degrees
    """
    diff = np.abs(user_arm_q - reset_arm_q)
    max_diff_rad = np.max(diff)
    max_diff_deg = np.rad2deg(max_diff_rad)
    is_matched = np.all(diff < threshold_rad)
    return is_matched, max_diff_deg


def update_binary_grasp(value, previous_state, open_threshold, close_threshold):
    """Apply hysteresis to compute stable binary grasp state."""
    if value >= close_threshold:
        return 1.0
    if value <= open_threshold:
        return 0.0
    return previous_state


def build_franka16_binary_vectors(arm_q, arm_dq, arm_action, left_grasp, right_grasp):
    """Build canonical 16D [L7, R7, Lgrasp, Rgrasp] vectors."""
    qpos = np.zeros(16, dtype=np.float32)
    qvel = np.zeros(16, dtype=np.float32)
    action = np.zeros(16, dtype=np.float32)
    qpos[:14] = np.asarray(arm_q, dtype=np.float32)[:14]
    qvel[:14] = np.asarray(arm_dq, dtype=np.float32)[:14]
    action[:14] = np.asarray(arm_action, dtype=np.float32)[:14]
    qpos[14] = np.float32(left_grasp)
    qpos[15] = np.float32(right_grasp)
    action[14] = np.float32(left_grasp)
    action[15] = np.float32(right_grasp)
    return qpos, qvel, action


class Dex3BinaryController:
    """Direct dex3 command publisher for controller-mode binary open/close."""
    # Contact-aware adaptive gains:
    # - use stronger gains for small errors (free-space motion)
    # - lower gains for large persistent errors (likely rigid-object contact)
    KP_HIGH = 1.5
    KD_HIGH = 0.2
    KP_LOW = 0.55
    KD_LOW = 0.08
    ERR_SMALL_RAD = 0.08
    ERR_LARGE_RAD = 0.45

    # GR00T-style claw presets for G1 + Dex3 (7DoF per hand).
    LEFT_OPEN_Q = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
    RIGHT_OPEN_Q = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
    # Keep thumb yaw neutral (joint0=0.0) and close thumb inward with joints 1/2.
    LEFT_CLOSE_Q = np.array([0.0, 1, 1, -1.5, -1.5, -1.5, -1.5], dtype=np.float32)
    RIGHT_CLOSE_Q = np.array([0.0, -1, -1, 1.5, 1.5, 1.5, 1.5], dtype=np.float32)

    def __init__(self, simulation_mode=False):
        if not DEX3_BINARY_AVAILABLE:
            raise RuntimeError("Dex3 binary controller requires unitree_hg HandCmd messages")
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize

        ChannelFactoryInitialize(1 if simulation_mode else 0)
        self.left_pub = ChannelPublisher("rt/dex3/left/cmd", HandCmd_)
        self.left_pub.Init()
        self.right_pub = ChannelPublisher("rt/dex3/right/cmd", HandCmd_)
        self.right_pub.Init()
        self.left_state_sub = ChannelSubscriber("rt/dex3/left/state", HandState_)
        self.left_state_sub.Init()
        self.right_state_sub = ChannelSubscriber("rt/dex3/right/state", HandState_)
        self.right_state_sub.Init()
        self.left_state_q = np.zeros(7, dtype=np.float32)
        self.right_state_q = np.zeros(7, dtype=np.float32)

        self.left_msg = unitree_hg_msg_dds__HandCmd_()
        self.right_msg = unitree_hg_msg_dds__HandCmd_()
        self._init_msg(self.left_msg)
        self._init_msg(self.right_msg)

    @staticmethod
    def _init_msg(msg):
        q = 0.0
        dq = 0.0
        tau = 0.0
        kp = 1.5
        kd = 0.2
        for joint_id in range(7):
            mode = (joint_id & 0x0F) | ((0x01 & 0x07) << 4)
            msg.motor_cmd[joint_id].mode = mode
            msg.motor_cmd[joint_id].q = q
            msg.motor_cmd[joint_id].dq = dq
            msg.motor_cmd[joint_id].tau = tau
            msg.motor_cmd[joint_id].kp = kp
            msg.motor_cmd[joint_id].kd = kd

    @classmethod
    def _adaptive_pd_from_error(cls, abs_err):
        if abs_err <= cls.ERR_SMALL_RAD:
            return cls.KP_HIGH, cls.KD_HIGH
        if abs_err >= cls.ERR_LARGE_RAD:
            return cls.KP_LOW, cls.KD_LOW
        span = cls.ERR_LARGE_RAD - cls.ERR_SMALL_RAD
        alpha = (abs_err - cls.ERR_SMALL_RAD) / span
        kp = cls.KP_HIGH + (cls.KP_LOW - cls.KP_HIGH) * alpha
        kd = cls.KD_HIGH + (cls.KD_LOW - cls.KD_HIGH) * alpha
        return kp, kd

    def _refresh_state(self):
        left_msg = self.left_state_sub.Read()
        right_msg = self.right_state_sub.Read()
        if left_msg is not None:
            self.left_state_q = np.array([left_msg.motor_state[i].q for i in range(7)], dtype=np.float32)
        if right_msg is not None:
            self.right_state_q = np.array([right_msg.motor_state[i].q for i in range(7)], dtype=np.float32)

    def ctrl_binary(self, left_close, right_close):
        left_target = self.LEFT_CLOSE_Q if left_close >= 0.5 else self.LEFT_OPEN_Q
        right_target = self.RIGHT_CLOSE_Q if right_close >= 0.5 else self.RIGHT_OPEN_Q
        self._refresh_state()
        for joint_id in range(7):
            self.left_msg.motor_cmd[joint_id].q = float(left_target[joint_id])
            self.right_msg.motor_cmd[joint_id].q = float(right_target[joint_id])
            left_err = abs(float(left_target[joint_id]) - float(self.left_state_q[joint_id]))
            right_err = abs(float(right_target[joint_id]) - float(self.right_state_q[joint_id]))
            left_kp, left_kd = self._adaptive_pd_from_error(left_err)
            right_kp, right_kd = self._adaptive_pd_from_error(right_err)
            self.left_msg.motor_cmd[joint_id].kp = left_kp
            self.left_msg.motor_cmd[joint_id].kd = left_kd
            self.right_msg.motor_cmd[joint_id].kp = right_kp
            self.right_msg.motor_cmd[joint_id].kd = right_kd
        self.left_pub.Write(self.left_msg)
        self.right_pub.Write(self.right_msg)

    def stop(self):
        return

# state transition
start_signal = False
running = True
should_toggle_recording = False
is_recording = False
is_paused = False  # Paused state between episodes (robot at reset pose, waiting for user)
should_force_resume = False  # Manual override to force resume tracking
should_reconnect_hands = False  # Request to reconnect inspire hands

# Waist yaw control via keyboard (G1 only)
# Use toggle approach instead of held keys (on_release causes TeleVuer issues)
# Press LEFT to start/stop turning right, press RIGHT to start/stop turning left
waist_yaw_direction = 0  # -1 = turning left, 0 = stopped, +1 = turning right
WAIST_YAW_SPEED = 0.02  # radians per loop iteration (~1.1 deg at 30Hz)

def on_press(key):
    global running, start_signal, should_toggle_recording, should_force_resume, should_reconnect_hands
    global waist_yaw_direction
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
    elif key == 'c' and start_signal == True:
        should_force_resume = True
        logger_mp.info("Force resume signal received (key 'c')")
    elif key == 'h' and start_signal == True:
        should_reconnect_hands = True
        logger_mp.info("Hand reconnect signal received (key 'h')")
    elif key == 'left' and start_signal == True:
        # Toggle: press to start turning right (positive yaw), press again to stop
        waist_yaw_direction = WAIST_YAW_SPEED if waist_yaw_direction <= 0 else 0
        logger_mp.info(f"Waist yaw: {'turning RIGHT' if waist_yaw_direction > 0 else 'STOPPED'}")
    elif key == 'right' and start_signal == True:
        # Toggle: press to start turning left (negative yaw), press again to stop
        waist_yaw_direction = -WAIST_YAW_SPEED if waist_yaw_direction >= 0 else 0
        logger_mp.info(f"Waist yaw: {'turning LEFT' if waist_yaw_direction < 0 else 'STOPPED'}")

# NOTE: Do NOT use on_release callback - it breaks TeleVuer subprocess communication
listen_keyboard_thread = threading.Thread(target=listen_keyboard, kwargs={"on_press": on_press, "until": None, "sequential": False,}, daemon=True)
listen_keyboard_thread.start()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--task_dir', type = str, default = './utils/data/', help = 'path to save data')
    parser.add_argument('--frequency', type = float, default = 30.0, help = 'save data\'s frequency (Hz)')

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

    # inspire hand bridge parameters (for laptop-connected hands)
    parser.add_argument('--inspire-bridge', action='store_true', help='Use bridge mode for Inspire hands (hands connected to laptop via network)')
    parser.add_argument('--network-interface', type=str, default='eno1', help='Network interface for hand bridge (e.g., eno1, eth0, wlan0)')
    parser.add_argument('--left-hand-ip', type=str, default='192.168.123.211', help='IP address of left Inspire hand')
    parser.add_argument('--right-hand-ip', type=str, default='192.168.123.210', help='IP address of right Inspire hand')
    
    # debug flags
    parser.add_argument('--debug', action='store_true', help='Enable debug output for locomotion and other data')
    parser.add_argument(
        '--save-layout',
        type=str,
        choices=['legacy', 'franka16_binary'],
        default='legacy',
        help='Recorded qpos/action layout. Use franka16_binary for canonical 16D [L7,R7,Lgrasp,Rgrasp].',
    )
    parser.add_argument('--grasp-open-threshold', type=float, default=0.35, help='Hysteresis open threshold for binary grasp.')
    parser.add_argument('--grasp-close-threshold', type=float, default=0.65, help='Hysteresis close threshold for binary grasp.')
    parser.add_argument(
        '--controller-grasp-source',
        type=str,
        choices=['trigger', 'pinch'],
        default='trigger',
        help='Controller-mode source signal for binary grasp.',
    )

    args = parser.parse_args()
    logger_mp.info(f"args: {args}")
    if args.grasp_open_threshold >= args.grasp_close_threshold:
        raise ValueError("grasp-open-threshold must be < grasp-close-threshold")

    use_franka16_binary = args.save_layout == 'franka16_binary'
    left_grasp_binary = 0.0
    right_grasp_binary = 0.0
    use_dex3_binary_controller = bool(
        use_franka16_binary and args.xr_mode == "controller" and args.ee == "dex3"
    )

    # image client: img_config should match image_server.py on the robot (resolution / stitch order).
    # When the server sends head|wide (hconcat), set wide_camera_image_shape to the wide panel [H, W]
    # so the client fills wide_img_shm and HDF5 gets both ego_cam and wide_cam.
    img_config = {
        'fps': 30,
        'head_camera_type': 'realsense',
        'head_camera_image_shape': [480, 640],  # head panel [H, W]
        'head_camera_id_numbers': [0],
        'wide_camera_image_shape': [480, 640],  # wide panel [H, W]; omit if image_server sends head only
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

    if WRIST and img_config.get('wide_camera_image_shape'):
        raise ValueError(
            "img_config cannot combine wrist_camera_* with wide_camera_image_shape "
            "(different ZMQ stripe layouts)."
        )
    HAS_WIDE = bool(img_config.get('wide_camera_image_shape')) and not WRIST
    
    if BINOCULAR and not (img_config['head_camera_image_shape'][1] / img_config['head_camera_image_shape'][0] > ASPECT_RATIO_THRESHOLD):
        tv_img_shape = (img_config['head_camera_image_shape'][0], img_config['head_camera_image_shape'][1] * 2, 3)
    else:
        tv_img_shape = (img_config['head_camera_image_shape'][0], img_config['head_camera_image_shape'][1], 3)

    tv_img_shm = shared_memory.SharedMemory(create = True, size = np.prod(tv_img_shape) * np.uint8().itemsize)
    tv_img_array = np.ndarray(tv_img_shape, dtype = np.uint8, buffer = tv_img_shm.buf)

    # Create wrist shared memory if needed (but NOT ImageClient yet - ZMQ is not fork-safe)
    wrist_img_shape = None
    wrist_img_shm = None
    wrist_img_array = None
    if WRIST:
        wrist_img_shape = (img_config['wrist_camera_image_shape'][0], img_config['wrist_camera_image_shape'][1] * 2, 3)
        wrist_img_shm = shared_memory.SharedMemory(create = True, size = np.prod(wrist_img_shape) * np.uint8().itemsize)
        wrist_img_array = np.ndarray(wrist_img_shape, dtype = np.uint8, buffer = wrist_img_shm.buf)

    wide_img_shape = None
    wide_img_shm = None
    wide_img_array = None
    if HAS_WIDE:
        w_h, w_w = img_config['wide_camera_image_shape']
        wide_img_shape = (w_h, w_w, 3)
        wide_img_shm = shared_memory.SharedMemory(create=True, size=np.prod(wide_img_shape) * np.uint8().itemsize)
        wide_img_array = np.ndarray(wide_img_shape, dtype=np.uint8, buffer=wide_img_shm.buf)
    
    # NOTE: ImageClient will be created AFTER TeleVuerWrapper because:
    # - TeleVuerWrapper spawns a subprocess (fork)
    # - ZMQ sockets are created in ImageClient.__init__()
    # - ZMQ sockets are NOT fork-safe - they get corrupted if created before fork
    img_client = None
    image_receive_thread = None

    # Initialize DDS with network interface if using inspire bridge mode
    # This MUST be done before creating arm controller to ensure correct network interface
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

    # NOW create ImageClient - AFTER TeleVuerWrapper fork, so ZMQ socket won't be corrupted
    logger_mp.info("Creating ImageClient (after TeleVuerWrapper fork for ZMQ safety)...")
    _img_client_kwargs = {
        "tv_img_shape": tv_img_shape,
        "tv_img_shm_name": tv_img_shm.name,
        "debug": args.debug,
    }
    if WRIST:
        _img_client_kwargs["wrist_img_shape"] = wrist_img_shape
        _img_client_kwargs["wrist_img_shm_name"] = wrist_img_shm.name
    if HAS_WIDE:
        _img_client_kwargs["wide_img_shape"] = wide_img_shape
        _img_client_kwargs["wide_img_shm_name"] = wide_img_shm.name
    if args.sim:
        _img_client_kwargs["server_address"] = "127.0.0.1"
    img_client = ImageClient(**_img_client_kwargs)
    
    # Start image receive thread immediately after ImageClient creation
    image_receive_thread = threading.Thread(target=img_client.receive_process, daemon=True)
    image_receive_thread.start()
    logger_mp.info("Image receive thread started")
    if HAS_WIDE:
        logger_mp.info("Wide camera strip enabled: HDF5 will store observations/images/wide_cam alongside ego_cam")

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

    # Get reset pose for this robot type (arms forward at chest height)
    reset_left_wrist_pose, reset_right_wrist_pose = get_reset_wrist_poses(args.arm)
    logger_mp.info(f"Reset pose initialized for {args.arm}")
    
    # Pre-compute the reset pose IK solution (will be updated when entering pause)
    reset_arm_q = None  # Will be computed when entering paused state
    pause_pose_match_logged = False  # To avoid spamming logs

    # end-effector
    if args.ee == "dex3":
        if use_dex3_binary_controller:
            logger_mp.info("Using Dex3BinaryController for controller-mode discrete open/close.")
            hand_ctrl = Dex3BinaryController(simulation_mode=args.sim)
            left_hand_pos_array = None
            right_hand_pos_array = None
            dual_hand_data_lock = None
            dual_hand_state_array = None
            dual_hand_action_array = None
        else:
            left_hand_pos_array = Array('d', 75, lock = True)      # [input]
            right_hand_pos_array = Array('d', 75, lock = True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 14, lock = False)   # [output] current left, right hand state(14) data.
            dual_hand_action_array = Array('d', 14, lock = False)  # [output] current left, right hand action(14) data.
            hand_ctrl = Dex3_1_Controller(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim, dds_already_initialized=True)
    elif args.ee == "dex1":
        left_gripper_value = Value('d', 0.0, lock=True)        # [input]
        right_gripper_value = Value('d', 0.0, lock=True)       # [input]
        dual_gripper_data_lock = Lock()
        dual_gripper_state_array = Array('d', 2, lock=False)   # current left, right gripper state(2) data.
        dual_gripper_action_array = Array('d', 2, lock=False)  # current left, right gripper action(2) data.
        gripper_ctrl = Dex1_1_Gripper_Controller(left_gripper_value, right_gripper_value, dual_gripper_data_lock, dual_gripper_state_array, dual_gripper_action_array, simulation_mode=args.sim, dds_already_initialized=True)
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
    
    # Locomotion data is extracted from lowstate (which the arm controller already subscribes to)
    # No separate subscription needed - we use arm_ctrl.get_lowstate_raw() in the main loop
    # This provides: IMU (rpy, quaternion, accel, gyro), mode_machine, and wireless_remote (controller inputs)
    if args.motion and (args.record or args.debug):
        logger_mp.info("[LOCO] Locomotion data will be extracted from lowstate (IMU + controller inputs)")
        logger_mp.info("       loco_state [17]: mode, rpy(3), quaternion(4), accel(3), gyro(3), leg_joints(3)")
        logger_mp.info("       loco_action [20]: joysticks(4), buttons(16)")
    
    # record + headless mode
    if args.record and args.headless:
        # Use HDF5 writer for msc_humanoid_visual compatibility
        recorder = EpisodeWriterHDF5(
            save_dir=args.task_dir + args.task_name,
            robot_name=args.arm,
            fps=args.frequency
        )
        logger_mp.info(f"HDF5 Episode recorder initialized (headless mode)")
    elif args.record and not args.headless:
        # Use HDF5 writer for msc_humanoid_visual compatibility
        recorder = EpisodeWriterHDF5(
            save_dir=args.task_dir + args.task_name,
            robot_name=args.arm,
            fps=args.frequency
        )
        logger_mp.info(f"HDF5 Episode recorder initialized")
    
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
                elif key == ord('c'):
                    should_force_resume = True
                    logger_mp.info("Force resume signal received (key 'c' in window)")
                elif key == ord('h'):
                    should_reconnect_hands = True
                    logger_mp.info("Hand reconnect signal received (key 'h' in window)")
                elif key == ord('a'):
                    if args.sim:
                        publish_reset_category(2, reset_pose_publisher)

            if args.record and should_toggle_recording:
                should_toggle_recording = False
                if not is_recording:
                    recorder.start_recording(
                        metadata={
                            "data_layout": args.save_layout,
                            "action_dim": 16 if use_franka16_binary else None,
                            "state_dim": 16 if use_franka16_binary else None,
                            "grasp_encoding": "binary" if use_franka16_binary else "continuous",
                            "grasp_open_threshold": args.grasp_open_threshold if use_franka16_binary else None,
                            "grasp_close_threshold": args.grasp_close_threshold if use_franka16_binary else None,
                            "controller_grasp_source": args.controller_grasp_source,
                        }
                    )
                    is_recording = True
                    logger_mp.info("==> Recording started")
                else:
                    # check logs
                    recorder.stop_recording()
                    is_recording = False
                    logger_mp.info("==> Recording stopped and saved")
                    if args.sim:
                        publish_reset_category(1, reset_pose_publisher)
                    # Enter paused state - robot will go to reset pose
                    is_paused = True
                    pause_pose_match_logged = False
                    # Compute reset pose IK solution using current arm state as seed
                    current_lr_arm_q_for_reset = arm_ctrl.get_current_dual_arm_q()
                    reset_arm_q, _ = arm_ik.solve_ik(reset_left_wrist_pose, reset_right_wrist_pose, 
                                                     current_lr_arm_q_for_reset, None)
                    logger_mp.info("=" * 60)
                    logger_mp.info("PAUSED: Robot moving to reset pose (arms forward)")
                    logger_mp.info("Match the pose to resume tracking, or press 'c' to force resume")
                    logger_mp.info("=" * 60)
            
            # Handle force resume
            if should_force_resume:
                should_force_resume = False
                if is_paused:
                    is_paused = False
                    logger_mp.info("=" * 60)
                    logger_mp.info("RESUMED: Tracking reactivated via manual override ('c' key)")
                    logger_mp.info("=" * 60)
            
            # Handle hand reconnect request
            if should_reconnect_hands:
                should_reconnect_hands = False
                if args.ee == "inspire1" and args.inspire_bridge:
                    if hasattr(hand_ctrl, 'request_reconnect'):
                        success_left, success_right = hand_ctrl.request_reconnect()
                        status = hand_ctrl.get_connection_status()
                        logger_mp.info(f"Hand connection status - Left: {status['left']}, Right: {status['right']}")
                    else:
                        logger_mp.warning("Hand controller doesn't support reconnect (missing request_reconnect method)")
                else:
                    logger_mp.info("Hand reconnect only available for inspire bridge mode (--inspire-bridge flag)")
            
            # get input data
            tele_data = tv_wrapper.get_motion_state_data()
            
            # Debug: confirm VR poses received from headset
            if args.debug:
                logger_mp.info(f"[VR POSE] Left: ({tele_data.left_arm_pose[0,3]:.3f}, {tele_data.left_arm_pose[1,3]:.3f}, {tele_data.left_arm_pose[2,3]:.3f}) "
                              f"Right: ({tele_data.right_arm_pose[0,3]:.3f}, {tele_data.right_arm_pose[1,3]:.3f}, {tele_data.right_arm_pose[2,3]:.3f})")
            
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

            if args.xr_mode == "controller":
                if args.controller_grasp_source == "pinch":
                    left_signal = float(tele_data.left_pinch_value)
                    right_signal = float(tele_data.right_pinch_value)
                else:
                    # On some controller stacks trigger is reported high at rest.
                    # Invert so: not pressed -> open, pressed -> close.
                    left_signal = 1.0 - float(tele_data.left_trigger_value)
                    right_signal = 1.0 - float(tele_data.right_trigger_value)

                left_grasp_binary = update_binary_grasp(
                    left_signal,
                    left_grasp_binary,
                    args.grasp_open_threshold,
                    args.grasp_close_threshold,
                )
                right_grasp_binary = update_binary_grasp(
                    right_signal,
                    right_grasp_binary,
                    args.grasp_open_threshold,
                    args.grasp_close_threshold,
                )

                if use_dex3_binary_controller:
                    hand_ctrl.ctrl_binary(
                        left_close=left_grasp_binary,
                        right_close=right_grasp_binary,
                    )
            
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

            if is_paused:
                # PAUSED STATE: Robot holds reset pose, waits for user to match
                # Command robot to stay at reset pose
                sol_q = reset_arm_q
                sol_tauff = np.zeros_like(reset_arm_q)  # No torque feedforward when holding
                arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)
                
                # Compute what the user's IK solution would be (to check if they match reset pose)
                user_sol_q, _ = arm_ik.solve_ik(tele_data.left_arm_pose, tele_data.right_arm_pose, 
                                                 current_lr_arm_q, current_lr_arm_dq)
                
                # Check if user pose matches reset pose
                pose_matched, max_diff_deg = check_arm_pose_match(user_sol_q, reset_arm_q, POSE_MATCH_THRESHOLD_RAD)
                
                if pose_matched:
                    is_paused = False
                    logger_mp.info("=" * 60)
                    logger_mp.info("RESUMED: User pose matched reset pose!")
                    logger_mp.info("Tracking reactivated - ready for next episode")
                    logger_mp.info("=" * 60)
                else:
                    # Log pose mismatch periodically (not every frame)
                    if not pause_pose_match_logged or int(time.time()) % 3 == 0:
                        logger_mp.info(f"[PAUSED] Waiting for pose match... max joint diff: {max_diff_deg:.1f}° (need < 15°)")
                        pause_pose_match_logged = True
            else:
                # NORMAL TRACKING: solve ik using motor data and wrist pose
                time_ik_start = time.time()
                sol_q, sol_tauff = arm_ik.solve_ik(tele_data.left_arm_pose, tele_data.right_arm_pose, current_lr_arm_q, current_lr_arm_dq)
                time_ik_end = time.time()
                logger_mp.debug(f"ik:\t{round(time_ik_end - time_ik_start, 6)}")
                arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)
                
                # Debug: confirm robot command sent
                if args.debug:
                    logger_mp.info(f"[ROBOT CMD] IK solved in {(time_ik_end - time_ik_start)*1000:.1f}ms, "
                                  f"L=[{sol_q[0]:.2f},{sol_q[1]:.2f},{sol_q[2]:.2f}...] "
                                  f"R=[{sol_q[7]:.2f},{sol_q[8]:.2f},{sol_q[9]:.2f}...]")
            
            # Apply waist yaw control from keyboard (G1 only) - toggle based
            # LEFT arrow = start/stop rotating right (positive yaw)
            # RIGHT arrow = start/stop rotating left (negative yaw)
            if args.arm in ['G1_29', 'G1_23'] and waist_yaw_direction != 0:
                current_waist_yaw = arm_ctrl.get_waist_yaw_target()
                new_waist_yaw = current_waist_yaw + waist_yaw_direction
                arm_ctrl.ctrl_waist_yaw(new_waist_yaw)

            # Debug waist yaw state (works even when not recording, G1 only)
            if args.debug and args.arm in ['G1_29', 'G1_23'] and not is_recording:
                waist_yaw_current = arm_ctrl.get_current_waist_yaw()
                waist_yaw_target = arm_ctrl.get_waist_yaw_target()
                logger_mp.info(f"[WAIST DEBUG] current={waist_yaw_current:.3f} rad, target={waist_yaw_target:.3f} rad")

            # record data
            if args.record and is_recording:
                # Get hand state and actions
                if args.ee == "dex3" and args.xr_mode == "hand" and not use_dex3_binary_controller:
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

                if use_franka16_binary:
                    if args.xr_mode != "controller":
                        # In non-controller mode, infer coarse binary grasp from hand actions if available.
                        if hand_action.size >= 2:
                            left_signal = float(np.mean(hand_action[: hand_action.size // 2]))
                            right_signal = float(np.mean(hand_action[hand_action.size // 2 :]))
                            left_grasp_binary = update_binary_grasp(
                                left_signal,
                                left_grasp_binary,
                                args.grasp_open_threshold,
                                args.grasp_close_threshold,
                            )
                            right_grasp_binary = update_binary_grasp(
                                right_signal,
                                right_grasp_binary,
                                args.grasp_open_threshold,
                                args.grasp_close_threshold,
                            )
                    full_qpos, full_qvel, full_action = build_franka16_binary_vectors(
                        current_lr_arm_q,
                        current_lr_arm_dq,
                        sol_q,
                        left_grasp_binary,
                        right_grasp_binary,
                    )
                else:
                    # Combine arm and hand states into full qpos/qvel/action
                    # For G1 robots, also include waist yaw as the last joint
                    if args.arm in ['G1_29', 'G1_23']:
                        waist_yaw_state = arm_ctrl.get_current_waist_yaw()
                        waist_yaw_target = arm_ctrl.get_waist_yaw_target()
                        full_qpos = np.concatenate([current_lr_arm_q, hand_state, [waist_yaw_state]])
                        full_qvel = np.concatenate([current_lr_arm_dq, np.zeros_like(hand_state), [0.0]])  # Waist vel not tracked
                        full_action = np.concatenate([sol_q, hand_action, [waist_yaw_target]])
                        if args.debug:
                            logger_mp.info(f"[WAIST] state={waist_yaw_state:.3f} rad, target={waist_yaw_target:.3f} rad")
                    else:
                        full_qpos = np.concatenate([current_lr_arm_q, hand_state])
                        full_qvel = np.concatenate([current_lr_arm_dq, np.zeros_like(hand_state)])
                        full_action = np.concatenate([sol_q, hand_action])
                
                # Prepare camera images in msc_humanoid_visual format
                # IMPORTANT: Use .copy() to ensure each frame is stored independently
                # Without .copy(), numpy views may all reference the same underlying memory
                images = {}
                current_tv_image = tv_img_array.copy()
                
                if BINOCULAR:
                    # Split binocular image into left and right
                    images["ego_cam"] = current_tv_image[:, :tv_img_shape[1]//2].copy()
                else:
                    images["ego_cam"] = current_tv_image.copy()
                
                # Add wrist cameras if available
                if WRIST:
                    current_wrist_image = wrist_img_array.copy()
                    images["cam_left_wrist"] = current_wrist_image[:, :wrist_img_shape[1]//2].copy()
                    images["cam_right_wrist"] = current_wrist_image[:, wrist_img_shape[1]//2:].copy()

                if HAS_WIDE:
                    images["wide_cam"] = wide_img_array.copy()

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
        logger_mp.info("=" * 60)
        logger_mp.info("CLEANUP: Shutting down all components...")
        logger_mp.info("=" * 60)
        
        # Stop keyboard listener first (signal it to stop)
        logger_mp.info("[CLEANUP] Stopping keyboard listener...")
        try:
            stop_listening()
        except Exception as e:
            logger_mp.debug(f"[CLEANUP] stop_listening: {e}")
        
        # Stop image client
        logger_mp.info("[CLEANUP] Stopping image client...")
        img_client.running = False
        try:
            if image_receive_thread is not None:
                image_receive_thread.join(timeout=1)
        except Exception as e:
            logger_mp.debug(f"[CLEANUP] Image thread join: {e}")
        
        # Stop hand/gripper controller
        if args.ee == "dex3":
            logger_mp.info("[CLEANUP] Stopping Dex3 hand controller...")
            hand_ctrl.stop()
        elif args.ee == "dex1":
            logger_mp.info("[CLEANUP] Stopping Dex1 gripper controller...")
            gripper_ctrl.stop()
        
        # Send robot arms home
        logger_mp.info("[CLEANUP] Sending robot arms to home position...")
        try:
            arm_ctrl.ctrl_dual_arm_go_home()
        except Exception as e:
            logger_mp.warning(f"[CLEANUP] Failed to send arms home: {e}")
        
        # Stop simulation state subscriber if in sim mode
        if args.sim:
            sim_state_subscriber.stop_subscribe()
        
        # Save any active recording
        if args.record:
            if recorder.is_recording():
                logger_mp.info("=" * 60)
                logger_mp.info("SAVING EPISODE: Please wait, this may take 30-60 seconds...")
                logger_mp.info("DO NOT interrupt or data will be lost!")
                logger_mp.info("=" * 60)
                recorder.stop_recording()
                logger_mp.info("=" * 60)
                logger_mp.info("EPISODE SAVED SUCCESSFULLY!")
                logger_mp.info("=" * 60)
        
        # Clean up shared memory
        logger_mp.info("[CLEANUP] Cleaning up shared memory...")
        try:
            tv_img_shm.close()
            tv_img_shm.unlink()
        except Exception as e:
            logger_mp.warning(f"[CLEANUP] Failed to cleanup tv_img_shm: {e}")
        
        if WRIST:
            try:
                wrist_img_shm.close()
                wrist_img_shm.unlink()
            except Exception as e:
                logger_mp.warning(f"[CLEANUP] Failed to cleanup wrist_img_shm: {e}")

        if HAS_WIDE:
            try:
                wide_img_shm.close()
                wide_img_shm.unlink()
            except Exception as e:
                logger_mp.warning(f"[CLEANUP] Failed to cleanup wide_img_shm: {e}")
        
        # Wait for keyboard listener thread to finish (already signaled to stop earlier)
        try:
            listen_keyboard_thread.join(timeout=1)
            if listen_keyboard_thread.is_alive():
                logger_mp.debug("[CLEANUP] Keyboard thread still alive, will be killed on exit")
        except Exception as e:
            logger_mp.debug(f"[CLEANUP] Keyboard thread join: {e}")
        
        logger_mp.info("=" * 60)
        logger_mp.info("Program exited cleanly.")
        logger_mp.info("=" * 60)
        
        # Force exit to kill any remaining daemon threads
        import os
        os._exit(0)
