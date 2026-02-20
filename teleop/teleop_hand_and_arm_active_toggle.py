import argparse
import os
import sys
import threading
import time
from multiprocessing import Array, Lock, Value, shared_memory

import cv2
import logging_mp
import numpy as np
from sshkeyboard import listen_keyboard, stop_listening

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from televuer import TeleVuerWrapper
from teleop.image_server.image_client import ImageClient
from teleop.robot_control.robot_arm import (
    G1_23_ArmController,
    G1_29_ArmController,
    H1_2_ArmController,
    H1_ArmController,
)
from teleop.robot_control.robot_arm_ik import G1_23_ArmIK, G1_29_ArmIK, H1_2_ArmIK, H1_ArmIK
from teleop.robot_control.robot_hand_unitree import Dex1_1_Gripper_Controller, Dex3_1_Controller
from teleop.utils.episode_writer_hdf5 import EpisodeWriterHDF5

# for simulation
from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

logging_mp.basic_config(level=logging_mp.INFO)
logger_mp = logging_mp.get_logger(__name__)


POSE_MATCH_THRESHOLD_RAD = np.deg2rad(30)
HAND_SIDE_DIMS = {
    "dex3": 7,
    "dex1": 1,
    "inspire1": 6,
    "brainco": 6,
}


def publish_reset_category(category: int, publisher):
    msg = String_(data=str(category))
    publisher.Write(msg)
    logger_mp.info(f"published reset category: {category}")


def get_reset_wrist_poses(robot_type):
    if robot_type in ("G1_29", "G1_23"):
        left_pose = np.array(
            [[1, 0, 0, 0.30], [0, 1, 0, 0.25], [0, 0, 1, 0.05], [0, 0, 0, 1]],
            dtype=np.float64,
        )
        right_pose = np.array(
            [[1, 0, 0, 0.30], [0, 1, 0, -0.25], [0, 0, 1, 0.05], [0, 0, 0, 1]],
            dtype=np.float64,
        )
    elif robot_type in ("H1_2", "H1"):
        left_pose = np.array(
            [[1, 0, 0, 0.25], [0, 1, 0, 0.25], [0, 0, 1, 0.10], [0, 0, 0, 1]],
            dtype=np.float64,
        )
        right_pose = np.array(
            [[1, 0, 0, 0.25], [0, 1, 0, -0.25], [0, 0, 1, 0.10], [0, 0, 0, 1]],
            dtype=np.float64,
        )
    else:
        raise ValueError(f"Unknown robot type: {robot_type}")
    return left_pose, right_pose


def check_arm_pose_match(user_arm_q, reset_arm_q, threshold_rad):
    diff = np.abs(user_arm_q - reset_arm_q)
    max_diff_deg = np.rad2deg(np.max(diff))
    return np.all(diff < threshold_rad), max_diff_deg


def split_hand_lr(hand_vec, ee):
    side_dim = HAND_SIDE_DIMS.get(ee, 0)
    if side_dim <= 0:
        return np.array([], dtype=np.float32), np.array([], dtype=np.float32)
    hand_vec = np.asarray(hand_vec, dtype=np.float32).flatten()
    if hand_vec.shape[0] < 2 * side_dim:
        padded = np.zeros(2 * side_dim, dtype=np.float32)
        padded[: hand_vec.shape[0]] = hand_vec
        hand_vec = padded
    return hand_vec[:side_dim].copy(), hand_vec[side_dim : 2 * side_dim].copy()


def build_hand_with_inactive_hold(hand_state, hand_action, ee, active_side_is_right):
    left_state, right_state = split_hand_lr(hand_state, ee)
    left_action, right_action = split_hand_lr(hand_action, ee)
    if left_action.size == 0 and right_action.size == 0:
        return np.array([], dtype=np.float32)
    if active_side_is_right:
        left_action = left_state.copy()
    else:
        right_action = right_state.copy()
    return np.concatenate([left_action, right_action]).astype(np.float32)


def extract_active_hand_7(hand_action, ee, active_side_is_right):
    left_action, right_action = split_hand_lr(hand_action, ee)
    side_action = right_action if active_side_is_right else left_action
    out = np.zeros(7, dtype=np.float32)
    out[: min(7, side_action.shape[0])] = side_action[:7]
    return out


# global state
start_signal = False
running = True
should_toggle_recording = False
is_recording = False
is_paused = False
should_force_resume = False
should_reconnect_hands = False
active_side_is_right = True  # default as requested

# Waist yaw control via keyboard (G1 only)
waist_yaw_direction = 0
WAIST_YAW_SPEED = 0.02


def on_press(key):
    global running, start_signal, should_toggle_recording, should_force_resume, should_reconnect_hands
    global waist_yaw_direction, active_side_is_right
    if key == "r":
        start_signal = True
        logger_mp.info("Program start signal received.")
    elif key == "q" and start_signal:
        stop_listening()
        running = False
    elif key == "s" and start_signal:
        should_toggle_recording = True
    elif key == "c" and start_signal:
        should_force_resume = True
    elif key == "h" and start_signal:
        should_reconnect_hands = True
    elif key == "t" and start_signal:
        active_side_is_right = not active_side_is_right
        logger_mp.info(f"Active side switched to: {'RIGHT' if active_side_is_right else 'LEFT'}")
    elif key == "left" and start_signal:
        waist_yaw_direction = WAIST_YAW_SPEED if waist_yaw_direction <= 0 else 0
        logger_mp.info(f"Waist yaw: {'turning RIGHT' if waist_yaw_direction > 0 else 'STOPPED'}")
    elif key == "right" and start_signal:
        waist_yaw_direction = -WAIST_YAW_SPEED if waist_yaw_direction >= 0 else 0
        logger_mp.info(f"Waist yaw: {'turning LEFT' if waist_yaw_direction < 0 else 'STOPPED'}")


listen_keyboard_thread = threading.Thread(
    target=listen_keyboard,
    kwargs={"on_press": on_press, "until": None, "sequential": False},
    daemon=True,
)
listen_keyboard_thread.start()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--task_dir", type=str, default="./utils/data/", help="path to save data")
    parser.add_argument("--frequency", type=float, default=30.0, help="save data frequency (Hz)")
    parser.add_argument("--xr-mode", type=str, choices=["hand", "controller"], default="hand")
    parser.add_argument("--arm", type=str, choices=["G1_29", "G1_23", "H1_2", "H1"], default="G1_29")
    parser.add_argument("--ee", type=str, choices=["dex1", "dex3", "inspire1", "brainco"])
    parser.add_argument("--motion", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--sim", action="store_true")
    parser.add_argument("--record", action="store_true")
    parser.add_argument("--task-name", type=str, default="pick cube")
    parser.add_argument("--task-goal", type=str, default="e.g. pick the red cube on the table.")
    parser.add_argument("--inspire-bridge", action="store_true")
    parser.add_argument("--network-interface", type=str, default="eno1")
    parser.add_argument("--left-hand-ip", type=str, default="192.168.123.211")
    parser.add_argument("--right-hand-ip", type=str, default="192.168.123.210")
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()
    logger_mp.info(f"args: {args}")

    img_config = {
        "fps": 30,
        "head_camera_type": "realsense",
        "head_camera_image_shape": [480, 640],
        "head_camera_id_numbers": [0],
    }
    aspect_ratio_threshold = 2.0
    binocular = (
        len(img_config["head_camera_id_numbers"]) > 1
        or (img_config["head_camera_image_shape"][1] / img_config["head_camera_image_shape"][0] > aspect_ratio_threshold)
    )
    wrist = "wrist_camera_type" in img_config
    if binocular and not (
        img_config["head_camera_image_shape"][1] / img_config["head_camera_image_shape"][0] > aspect_ratio_threshold
    ):
        tv_img_shape = (img_config["head_camera_image_shape"][0], img_config["head_camera_image_shape"][1] * 2, 3)
    else:
        tv_img_shape = (img_config["head_camera_image_shape"][0], img_config["head_camera_image_shape"][1], 3)

    tv_img_shm = shared_memory.SharedMemory(create=True, size=np.prod(tv_img_shape) * np.uint8().itemsize)
    tv_img_array = np.ndarray(tv_img_shape, dtype=np.uint8, buffer=tv_img_shm.buf)

    wrist_img_shape = None
    wrist_img_shm = None
    wrist_img_array = None
    if wrist:
        wrist_img_shape = (img_config["wrist_camera_image_shape"][0], img_config["wrist_camera_image_shape"][1] * 2, 3)
        wrist_img_shm = shared_memory.SharedMemory(create=True, size=np.prod(wrist_img_shape) * np.uint8().itemsize)
        wrist_img_array = np.ndarray(wrist_img_shape, dtype=np.uint8, buffer=wrist_img_shm.buf)

    dds_already_initialized = False
    if args.ee == "inspire1" and args.inspire_bridge:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize

        if args.sim:
            ChannelFactoryInitialize(1, args.network_interface)
        else:
            ChannelFactoryInitialize(0, args.network_interface)
        dds_already_initialized = True

    tv_wrapper = TeleVuerWrapper(
        binocular=binocular,
        use_hand_tracking=args.xr_mode == "hand",
        img_shape=tv_img_shape,
        img_shm_name=tv_img_shm.name,
        return_state_data=True,
        return_hand_rot_data=False,
    )

    if wrist and args.sim:
        img_client = ImageClient(
            tv_img_shape=tv_img_shape,
            tv_img_shm_name=tv_img_shm.name,
            wrist_img_shape=wrist_img_shape,
            wrist_img_shm_name=wrist_img_shm.name,
            server_address="127.0.0.1",
            debug=args.debug,
        )
    elif wrist and not args.sim:
        img_client = ImageClient(
            tv_img_shape=tv_img_shape,
            tv_img_shm_name=tv_img_shm.name,
            wrist_img_shape=wrist_img_shape,
            wrist_img_shm_name=wrist_img_shm.name,
            debug=args.debug,
        )
    else:
        img_client = ImageClient(tv_img_shape=tv_img_shape, tv_img_shm_name=tv_img_shm.name, debug=args.debug)

    image_receive_thread = threading.Thread(target=img_client.receive_process, daemon=True)
    image_receive_thread.start()

    if args.arm == "G1_29":
        arm_ik = G1_29_ArmIK()
        arm_ctrl = G1_29_ArmController(
            motion_mode=args.motion, simulation_mode=args.sim, dds_already_initialized=dds_already_initialized
        )
    elif args.arm == "G1_23":
        arm_ik = G1_23_ArmIK()
        arm_ctrl = G1_23_ArmController(
            motion_mode=args.motion, simulation_mode=args.sim, dds_already_initialized=dds_already_initialized
        )
    elif args.arm == "H1_2":
        arm_ik = H1_2_ArmIK()
        arm_ctrl = H1_2_ArmController(simulation_mode=args.sim, dds_already_initialized=dds_already_initialized)
    else:
        arm_ik = H1_ArmIK()
        arm_ctrl = H1_ArmController(simulation_mode=args.sim, dds_already_initialized=dds_already_initialized)

    reset_left_wrist_pose, reset_right_wrist_pose = get_reset_wrist_poses(args.arm)
    reset_arm_q = None
    pause_pose_match_logged = False

    if args.ee == "dex3":
        left_hand_pos_array = Array("d", 75, lock=True)
        right_hand_pos_array = Array("d", 75, lock=True)
        dual_hand_data_lock = Lock()
        dual_hand_state_array = Array("d", 14, lock=False)
        dual_hand_action_array = Array("d", 14, lock=False)
        hand_ctrl = Dex3_1_Controller(
            left_hand_pos_array,
            right_hand_pos_array,
            dual_hand_data_lock,
            dual_hand_state_array,
            dual_hand_action_array,
            simulation_mode=args.sim,
            dds_already_initialized=True,
        )
    elif args.ee == "dex1":
        left_gripper_value = Value("d", 0.0, lock=True)
        right_gripper_value = Value("d", 0.0, lock=True)
        dual_gripper_data_lock = Lock()
        dual_gripper_state_array = Array("d", 2, lock=False)
        dual_gripper_action_array = Array("d", 2, lock=False)
        gripper_ctrl = Dex1_1_Gripper_Controller(
            left_gripper_value,
            right_gripper_value,
            dual_gripper_data_lock,
            dual_gripper_state_array,
            dual_gripper_action_array,
            simulation_mode=args.sim,
            dds_already_initialized=True,
        )
    elif args.ee == "inspire1":
        from teleop.robot_control.robot_hand_inspire import Inspire_Controller

        left_hand_pos_array = Array("d", 75, lock=True)
        right_hand_pos_array = Array("d", 75, lock=True)
        dual_hand_data_lock = Lock()
        dual_hand_state_array = Array("d", 12, lock=False)
        dual_hand_action_array = Array("d", 12, lock=False)
        if args.inspire_bridge:
            from teleop.robot_control.robot_hand_inspire_bridge import Inspire_Bridge_Controller

            hand_ctrl = Inspire_Bridge_Controller(
                left_hand_pos_array,
                right_hand_pos_array,
                dual_hand_data_lock,
                dual_hand_state_array,
                dual_hand_action_array,
                simulation_mode=args.sim,
                network_interface=args.network_interface,
                left_hand_ip=args.left_hand_ip,
                right_hand_ip=args.right_hand_ip,
            )
        else:
            hand_ctrl = Inspire_Controller(
                left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array
            )
    elif args.ee == "brainco":
        from teleop.robot_control.robot_hand_brainco import Brainco_Controller

        left_hand_pos_array = Array("d", 75, lock=True)
        right_hand_pos_array = Array("d", 75, lock=True)
        dual_hand_data_lock = Lock()
        dual_hand_state_array = Array("d", 12, lock=False)
        dual_hand_action_array = Array("d", 12, lock=False)
        hand_ctrl = Brainco_Controller(
            left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array
        )

    if args.sim:
        reset_pose_publisher = ChannelPublisher("rt/reset_pose/cmd", String_)
        reset_pose_publisher.Init()
        from teleop.utils.sim_state_topic import start_sim_state_subscribe

        sim_state_subscriber = start_sim_state_subscribe()

    if args.record:
        recorder = EpisodeWriterHDF5(save_dir=args.task_dir + args.task_name, robot_name=args.arm, fps=args.frequency)

    if args.xr_mode == "controller" and args.motion:
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

        sport_client = LocoClient()
        sport_client.SetTimeout(0.0001)
        sport_client.Init()

    left_hand_hold_pose = None
    right_hand_hold_pose = None
    left_gripper_hold_value = 0.0
    right_gripper_hold_value = 0.0

    try:
        logger_mp.info("Please enter start signal ('r').")
        while not start_signal:
            time.sleep(0.01)
        arm_ctrl.speed_gradual_max()
        logger_mp.info("Active side control enabled: key 't' toggles LEFT/RIGHT (default RIGHT).")

        while running:
            start_time = time.time()
            if not args.headless:
                tv_resized_image = cv2.resize(tv_img_array, (tv_img_shape[1] // 2, tv_img_shape[0] // 2))
                cv2.imshow("record image", tv_resized_image)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    stop_listening()
                    running = False
                elif key == ord("s"):
                    should_toggle_recording = True
                elif key == ord("c"):
                    should_force_resume = True
                elif key == ord("h"):
                    should_reconnect_hands = True
                elif key == ord("t"):
                    active_side_is_right = not active_side_is_right
                    logger_mp.info(f"Active side switched to: {'RIGHT' if active_side_is_right else 'LEFT'}")

            if args.record and should_toggle_recording:
                should_toggle_recording = False
                if not is_recording:
                    recorder.start_recording(
                        metadata={
                            "action_layout": "active_side(0_left_1_right)+active_arm_7+active_hand_7+waist_yaw_1",
                            "action_side_encoding": "0=left,1=right",
                            "action_dim": 16,
                            "active_toggle_key": "t",
                            "active_default_side": "right",
                        }
                    )
                    is_recording = True
                    logger_mp.info("==> Recording started")
                else:
                    recorder.stop_recording()
                    is_recording = False
                    logger_mp.info("==> Recording stopped and saved")
                    if args.sim:
                        publish_reset_category(1, reset_pose_publisher)
                    is_paused = True
                    pause_pose_match_logged = False
                    current_lr_arm_q_for_reset = arm_ctrl.get_current_dual_arm_q()
                    reset_arm_q, _ = arm_ik.solve_ik(
                        reset_left_wrist_pose, reset_right_wrist_pose, current_lr_arm_q_for_reset, None
                    )

            if should_force_resume:
                should_force_resume = False
                if is_paused:
                    is_paused = False

            if should_reconnect_hands:
                should_reconnect_hands = False
                if args.ee == "inspire1" and args.inspire_bridge and hasattr(hand_ctrl, "request_reconnect"):
                    hand_ctrl.request_reconnect()
                    status = hand_ctrl.get_connection_status()
                    logger_mp.info(f"Hand connection status - Left: {status['left']}, Right: {status['right']}")

            tele_data = tv_wrapper.get_motion_state_data()
            if (args.ee in ("dex3", "inspire1", "brainco")) and args.xr_mode == "hand":
                left_current = tele_data.left_hand_pos.flatten()
                right_current = tele_data.right_hand_pos.flatten()
                if left_hand_hold_pose is None:
                    left_hand_hold_pose = left_current.copy()
                    right_hand_hold_pose = right_current.copy()
                if active_side_is_right:
                    right_hand_hold_pose = right_current.copy()
                else:
                    left_hand_hold_pose = left_current.copy()
                left_write = left_current if not active_side_is_right else left_hand_hold_pose
                right_write = right_current if active_side_is_right else right_hand_hold_pose
                with left_hand_pos_array.get_lock():
                    left_hand_pos_array[:] = left_write
                with right_hand_pos_array.get_lock():
                    right_hand_pos_array[:] = right_write
            elif args.ee == "dex1":
                if args.xr_mode == "controller":
                    left_current_val = tele_data.left_trigger_value
                    right_current_val = tele_data.right_trigger_value
                else:
                    left_current_val = tele_data.left_pinch_value
                    right_current_val = tele_data.right_pinch_value
                if active_side_is_right:
                    right_gripper_hold_value = right_current_val
                else:
                    left_gripper_hold_value = left_current_val
                left_value = left_current_val if not active_side_is_right else left_gripper_hold_value
                right_value = right_current_val if active_side_is_right else right_gripper_hold_value
                with left_gripper_value.get_lock():
                    left_gripper_value.value = left_value
                with right_gripper_value.get_lock():
                    right_gripper_value.value = right_value

            if args.xr_mode == "controller" and args.motion:
                if tele_data.tele_state.right_aButton:
                    stop_listening()
                    running = False
                if tele_data.tele_state.left_thumbstick_state and tele_data.tele_state.right_thumbstick_state:
                    sport_client.Damp()
                sport_client.Move(
                    -tele_data.tele_state.left_thumbstick_value[1] * 0.3,
                    -tele_data.tele_state.left_thumbstick_value[0] * 0.3,
                    -tele_data.tele_state.right_thumbstick_value[0] * 0.3,
                )

            current_lr_arm_q = arm_ctrl.get_current_dual_arm_q()
            current_lr_arm_dq = arm_ctrl.get_current_dual_arm_dq()

            if is_paused:
                sol_q = reset_arm_q
                sol_tauff = np.zeros_like(reset_arm_q)
                arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)
                user_sol_q, _ = arm_ik.solve_ik(
                    tele_data.left_arm_pose, tele_data.right_arm_pose, current_lr_arm_q, current_lr_arm_dq
                )
                pose_matched, max_diff_deg = check_arm_pose_match(user_sol_q, reset_arm_q, POSE_MATCH_THRESHOLD_RAD)
                if pose_matched:
                    is_paused = False
                elif not pause_pose_match_logged or int(time.time()) % 3 == 0:
                    logger_mp.info(f"[PAUSED] Waiting for pose match... max joint diff: {max_diff_deg:.1f} deg")
                    pause_pose_match_logged = True
                sol_q_full = sol_q.copy()
            else:
                sol_q_full, sol_tauff_full = arm_ik.solve_ik(
                    tele_data.left_arm_pose, tele_data.right_arm_pose, current_lr_arm_q, current_lr_arm_dq
                )
                sol_q = sol_q_full.copy()
                sol_tauff = sol_tauff_full.copy()
                if active_side_is_right:
                    sol_q[:7] = current_lr_arm_q[:7]
                    sol_tauff[:7] = 0.0
                else:
                    sol_q[7:14] = current_lr_arm_q[7:14]
                    sol_tauff[7:14] = 0.0
                arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)

            if args.arm in ("G1_29", "G1_23") and waist_yaw_direction != 0:
                current_waist_yaw = arm_ctrl.get_waist_yaw_target()
                arm_ctrl.ctrl_waist_yaw(current_waist_yaw + waist_yaw_direction)

            if args.record and is_recording:
                if args.ee == "dex3" and args.xr_mode == "hand":
                    with dual_hand_data_lock:
                        hand_state = np.array(dual_hand_state_array[:], dtype=np.float32)
                        hand_action = np.array(dual_hand_action_array[:], dtype=np.float32)
                elif args.ee == "dex1":
                    with dual_gripper_data_lock:
                        hand_state = np.array(dual_gripper_state_array[:], dtype=np.float32)
                        hand_action = np.array(dual_gripper_action_array[:], dtype=np.float32)
                elif (args.ee in ("inspire1", "brainco")) and args.xr_mode == "hand":
                    with dual_hand_data_lock:
                        hand_state = np.array(dual_hand_state_array[:], dtype=np.float32)
                        hand_action = np.array(dual_hand_action_array[:], dtype=np.float32)
                else:
                    hand_state = np.array([], dtype=np.float32)
                    hand_action = np.array([], dtype=np.float32)

                hand_action_for_robot = build_hand_with_inactive_hold(hand_state, hand_action, args.ee, active_side_is_right)
                active_hand_7 = extract_active_hand_7(hand_action, args.ee, active_side_is_right)
                active_arm_7 = sol_q_full[7:14] if active_side_is_right else sol_q_full[:7]
                active_side_value = np.float32(1.0 if active_side_is_right else 0.0)

                if args.arm in ("G1_29", "G1_23"):
                    waist_yaw_state = arm_ctrl.get_current_waist_yaw()
                    waist_yaw_target = arm_ctrl.get_waist_yaw_target()
                    full_qpos = np.concatenate([current_lr_arm_q, hand_state, [waist_yaw_state]]).astype(np.float32)
                    full_qvel = np.concatenate([current_lr_arm_dq, np.zeros_like(hand_state), [0.0]]).astype(np.float32)
                else:
                    waist_yaw_target = 0.0
                    full_qpos = np.concatenate([current_lr_arm_q, hand_state]).astype(np.float32)
                    full_qvel = np.concatenate([current_lr_arm_dq, np.zeros_like(hand_state)]).astype(np.float32)

                compact_action = np.concatenate(
                    [[active_side_value], np.asarray(active_arm_7, dtype=np.float32), active_hand_7, [np.float32(waist_yaw_target)]]
                ).astype(np.float32)

                images = {}
                current_tv_image = tv_img_array.copy()
                if binocular:
                    images["ego_cam"] = current_tv_image[:, : tv_img_shape[1] // 2].copy()
                else:
                    images["ego_cam"] = current_tv_image.copy()
                if wrist:
                    current_wrist_image = wrist_img_array.copy()
                    images["cam_left_wrist"] = current_wrist_image[:, : wrist_img_shape[1] // 2].copy()
                    images["cam_right_wrist"] = current_wrist_image[:, wrist_img_shape[1] // 2 :].copy()

                recorder.add_timestep(qpos=full_qpos, qvel=full_qvel, action=compact_action, images=images)

            time_elapsed = time.time() - start_time
            time.sleep(max(0, (1 / args.frequency) - time_elapsed))

    except KeyboardInterrupt:
        logger_mp.info("KeyboardInterrupt received, exiting.")
    finally:
        try:
            stop_listening()
        except Exception:
            pass
        try:
            img_client.running = False
            image_receive_thread.join(timeout=1)
        except Exception:
            pass
        try:
            if args.ee == "dex3":
                hand_ctrl.stop()
            elif args.ee == "dex1":
                gripper_ctrl.stop()
        except Exception:
            pass
        try:
            arm_ctrl.ctrl_dual_arm_go_home()
        except Exception:
            pass
        if args.sim:
            sim_state_subscriber.stop_subscribe()
        if args.record and recorder.is_recording():
            recorder.stop_recording()
        try:
            tv_img_shm.close()
            tv_img_shm.unlink()
        except Exception:
            pass
        if wrist:
            try:
                wrist_img_shm.close()
                wrist_img_shm.unlink()
            except Exception:
                pass
        os._exit(0)
