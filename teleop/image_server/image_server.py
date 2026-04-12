"""
image_server — stream stitched head + wide camera frames over ZMQ (JPEG).

Run (from repo home):
  python3 image_server/image_server.py
  python3 image_server/image_server.py 10 -10

Startup wide-camera Dynamixel angles (deg), same as g1_comp_servo_service
test_joint0_control / test_joint1_control (motor IDs 0 and 1):
  • No angle args  → joint0=0, joint1=-20
  • Two args        → joint0, joint1 (e.g. 10 -10)

Options:
  -s / --serial       Dynamixel device (default /dev/ttyUSB0 or WIDE_SERVO_SERIAL)
  --servo-config      Path to g1_comp_servo_service/config/config.yaml

Serial access: user should be in group 'dialout' (see ls -l /dev/ttyUSB*).
Do not run the whole script with sudo — user-site packages (e.g. pyrealsense2)
will not load.

At the bottom of this file, edit the config dict in the __main__ block for
camera serials, resolution, and ZMQ port (ImageServer default 5555).
"""

import argparse
import cv2
import math
import os
import sys
import time
import struct
import yaml
import zmq
from collections import deque
from pathlib import Path
import numpy as np
import pyrealsense2 as rs
import logging_mp
logger_mp = logging_mp.get_logger(__name__, level=logging_mp.DEBUG)

# Dynamixel SDK (same tree as g1_comp_servo_service tests); requires: pip install pyserial
_DXL_SRC = Path.home() / "g1_comp_servo_service/thirdparty/DynamixelSDK/python/src"
if _DXL_SRC.is_dir() and str(_DXL_SRC) not in sys.path:
    sys.path.insert(0, str(_DXL_SRC))


def _clamp(value, lower, upper):
    return max(lower, min(value, upper))


def _angle2encoder(desired_angle, servo_limit_encoder, lim_lo, lim_hi, direction):
    """Match g1_comp_servo_service/include/utilities.h angle2encoder()."""
    desired_angle = _clamp(desired_angle, lim_lo, lim_hi)
    servo_encoder_resolution = 4096 / (2 * math.pi)
    joint_range = lim_hi - lim_lo
    encoder_range = joint_range * (math.pi / 180) * servo_encoder_resolution
    return direction * (desired_angle - lim_lo) * (encoder_range / joint_range) + servo_limit_encoder


def _serial_permission_help(serial_device):
    return (
        f"Permission denied opening {serial_device}. On Linux, serial devices are usually "
        f"group-owned by 'dialout' (see ls -l {serial_device}). Add your user to that group:\n"
        f"  sudo usermod -aG dialout $USER\n"
        f"Then open a new login session, or in this shell run: newgrp dialout\n"
        f"Do not run this whole script with sudo: root's Python will not see packages installed "
        f"with pip --user (e.g. pyrealsense2). Fix serial permissions instead."
    )


def _apply_wide_camera_servo_targets(joint0_deg, joint1_deg, serial_device, config_path):
    """
    Set wide-camera Dynamixel heading motors (IDs 0 and 1), matching
    test_joint0_control.cpp / test_joint1_control.cpp (enable, P/D gains, goal position).
    """
    try:
        from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
    except ImportError as e:
        logger_mp.error(
            "[Image Server] dynamixel_sdk not importable (install pyserial; SDK path %s): %s",
            _DXL_SRC,
            e,
        )
        raise

    with open(config_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    if not cfg.get("has_calibrate"):
        raise RuntimeError("Servo config has has_calibrate=0; run test_calibration first.")

    j0_lim = cfg["joint0"]
    j1_lim = cfg["joint1"]
    direction = cfg["direction"]
    s0_cal = float(cfg["servo0_calibration"])
    s1_cal = float(cfg["servo1_calibration"])

    enc0 = int(round(_angle2encoder(joint0_deg, s0_cal, float(j0_lim[0]), float(j0_lim[1]), float(direction[0]))))
    enc1 = int(round(_angle2encoder(joint1_deg, s1_cal, float(j1_lim[0]), float(j1_lim[1]), float(direction[1]))))

    ADDR_TORQUE_ENABLE = 64
    POSITION_D_GAIN = 80
    POSITION_P_GAIN = 84
    ADDR_GOAL_POSITION = 116
    PROTOCOL_VERSION = 2.0
    BAUDRATE = 1_000_000
    KP, KD = 500, 300

    port = PortHandler(serial_device)
    packet = PacketHandler(PROTOCOL_VERSION)
    try:
        if not port.openPort():
            raise RuntimeError(f"Failed to open serial port {serial_device}")
        if not port.setBaudRate(BAUDRATE):
            port.closePort()
            raise RuntimeError(f"Failed to set baud rate {BAUDRATE} on {serial_device}")
    except (PermissionError, OSError) as e:
        if getattr(e, "errno", None) == 13:
            raise RuntimeError(_serial_permission_help(serial_device)) from e
        raise
    except Exception as e:
        # pyserial raises serial.serialutil.SerialException on open failure
        errno = getattr(e, "errno", None)
        msg = str(e).lower()
        if errno == 13 or "permission denied" in msg or "access denied" in msg:
            raise RuntimeError(_serial_permission_help(serial_device)) from e
        raise

    try:
        for dxl_id, goal_enc in ((0, enc0), (1, enc1)):
            if packet.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 1) != COMM_SUCCESS:
                logger_mp.warning("[Image Server] Torque enable failed for id %s", dxl_id)
            if packet.write4ByteTxRx(port, dxl_id, POSITION_P_GAIN, KP) != COMM_SUCCESS:
                logger_mp.warning("[Image Server] P gain write failed for id %s", dxl_id)
            if packet.write4ByteTxRx(port, dxl_id, POSITION_D_GAIN, KD) != COMM_SUCCESS:
                logger_mp.warning("[Image Server] D gain write failed for id %s", dxl_id)
            if packet.write4ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, goal_enc) != COMM_SUCCESS:
                logger_mp.warning("[Image Server] Goal position failed for id %s goal=%s", dxl_id, goal_enc)
        logger_mp.info(
            "[Image Server] Wide camera servos: joint0=%.3f deg -> enc %s (id 0), joint1=%.3f deg -> enc %s (id 1)",
            joint0_deg,
            enc0,
            joint1_deg,
            enc1,
        )
    finally:
        port.closePort()


class RealSenseCamera(object):
    def __init__(self, img_shape, fps, serial_number=None, enable_depth=False) -> None:
        """
        img_shape: [height, width]
        serial_number: serial number
        """
        self.img_shape = img_shape
        self.fps = fps
        self.serial_number = serial_number
        self.enable_depth = enable_depth

        align_to = rs.stream.color
        self.align = rs.align(align_to)
        self.init_realsense()

    def init_realsense(self):

        self.pipeline = rs.pipeline()
        config = rs.config()
        if self.serial_number is not None:
            config.enable_device(self.serial_number)

        config.enable_stream(rs.stream.color, self.img_shape[1], self.img_shape[0], rs.format.bgr8, self.fps)

        if self.enable_depth:
            config.enable_stream(rs.stream.depth, self.img_shape[1], self.img_shape[0], rs.format.z16, self.fps)

        profile = self.pipeline.start(config)
        self._device = profile.get_device()
        if self._device is None:
            logger_mp.error('[Image Server] pipe_profile.get_device() is None .')
        if self.enable_depth:
            assert self._device is not None
            depth_sensor = self._device.first_depth_sensor()
            self.g_depth_scale = depth_sensor.get_depth_scale()

        self.intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()

        if self.enable_depth:
            depth_frame = aligned_frames.get_depth_frame()

        if not color_frame:
            return None

        color_image = np.asanyarray(color_frame.get_data())
        # color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        depth_image = np.asanyarray(depth_frame.get_data()) if self.enable_depth else None
        return color_image, depth_image

    def release(self):
        self.pipeline.stop()


class OpenCVCamera():
    def __init__(self, device_id, img_shape, fps):
        """
        decive_id: /dev/video* or *
        img_shape: [height, width]
        """
        self.id = device_id
        self.fps = fps
        self.img_shape = img_shape
        self.cap = cv2.VideoCapture(self.id, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.img_shape[0])
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.img_shape[1])
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        # Test if the camera can read frames
        if not self._can_read_frame():
            logger_mp.error(f"[Image Server] Camera {self.id} Error: Failed to initialize the camera or read frames. Exiting...")
            self.release()

    def _can_read_frame(self):
        success, _ = self.cap.read()
        return success

    def release(self):
        self.cap.release()

    def get_frame(self):
        ret, color_image = self.cap.read()
        if not ret:
            return None
        return color_image


class ImageServer:
    def __init__(self, config, port = 5555, Unit_Test = False):
        """
        config example1:
        {
            'fps':30                                                          # frame per second
            'head_camera_type': 'opencv',                                     # opencv or realsense
            'head_camera_image_shape': [480, 1280],                           # Head camera resolution  [height, width]
            'head_camera_id_numbers': [0],                                    # '/dev/video0' (opencv)
            'wide_camera_type': 'realsense', 
            'wide_camera_image_shape': [480, 640],                           # wide camera resolution  [height, width]
            'wide_camera_id_numbers': ["218622271789", "241222076627"],      # realsense camera's serial number
        }

        config example2:
        {
            'fps':30                                                          # frame per second
            'head_camera_type': 'realsense',                                  # opencv or realsense
            'head_camera_image_shape': [480, 640],                            # Head camera resolution  [height, width]
            'head_camera_id_numbers': ["218622271739"],                       # realsense camera's serial number
            'wide_camera_type': 'opencv', 
            'wide_camera_image_shape': [480, 640],                           # wide camera resolution  [height, width]
            'wide_camera_id_numbers': [0,1],                                 # '/dev/video0' and '/dev/video1' (opencv)
        }

        If you are not using the wide camera, you can comment out its configuration, like this below:
        config:
        {
            'fps':30                                                          # frame per second
            'head_camera_type': 'opencv',                                     # opencv or realsense
            'head_camera_image_shape': [480, 1280],                           # Head camera resolution  [height, width]
            'head_camera_id_numbers': [0],                                    # '/dev/video0' (opencv)
            #'wide_camera_type': 'realsense', 
            #'wide_camera_image_shape': [480, 640],                           # wide camera resolution  [height, width]
            #'wide_camera_id_numbers': ["218622271789", "241222076627"],      # serial number (realsense)
        }
        """
        logger_mp.info(config)
        self.fps = config.get('fps', 30)
        self.head_camera_type = config.get('head_camera_type', 'opencv')
        self.head_image_shape = config.get('head_camera_image_shape', [480, 640])      # (height, width)
        self.head_camera_id_numbers = config.get('head_camera_id_numbers', [0])

        self.wide_camera_type = config.get('wide_camera_type', None)
        self.wide_image_shape = config.get('wide_camera_image_shape', [480, 640])    # (height, width)
        self.wide_camera_id_numbers = config.get('wide_camera_id_numbers', None)

        self.port = port
        self.Unit_Test = Unit_Test


        # Initialize head cameras
        self.head_cameras = []
        if self.head_camera_type == 'opencv':
            for device_id in self.head_camera_id_numbers:
                camera = OpenCVCamera(device_id=device_id, img_shape=self.head_image_shape, fps=self.fps)
                self.head_cameras.append(camera)
        elif self.head_camera_type == 'realsense':
            for serial_number in self.head_camera_id_numbers:
                camera = RealSenseCamera(img_shape=self.head_image_shape, fps=self.fps, serial_number=serial_number)
                self.head_cameras.append(camera)
        else:
            logger_mp.warning(f"[Image Server] Unsupported head_camera_type: {self.head_camera_type}")

        # Initialize wide cameras if provided
        self.wide_cameras = []
        if self.wide_camera_type and self.wide_camera_id_numbers:
            if self.wide_camera_type == 'opencv':
                for device_id in self.wide_camera_id_numbers:
                    camera = OpenCVCamera(device_id=device_id, img_shape=self.wide_image_shape, fps=self.fps)
                    self.wide_cameras.append(camera)
            elif self.wide_camera_type == 'realsense':
                for serial_number in self.wide_camera_id_numbers:
                    camera = RealSenseCamera(img_shape=self.wide_image_shape, fps=self.fps, serial_number=serial_number)
                    self.wide_cameras.append(camera)
            else:
                logger_mp.warning(f"[Image Server] Unsupported wide_camera_type: {self.wide_camera_type}")

        # Set ZeroMQ context and socket
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(f"tcp://*:{self.port}")

        if self.Unit_Test:
            self._init_performance_metrics()

        for cam in self.head_cameras:
            if isinstance(cam, OpenCVCamera):
                logger_mp.info(f"[Image Server] Head camera {cam.id} resolution: {cam.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)} x {cam.cap.get(cv2.CAP_PROP_FRAME_WIDTH)}")
            elif isinstance(cam, RealSenseCamera):
                logger_mp.info(f"[Image Server] Head camera {cam.serial_number} resolution: {cam.img_shape[0]} x {cam.img_shape[1]}")
            else:
                logger_mp.warning("[Image Server] Unknown camera type in head_cameras.")

        for cam in self.wide_cameras:
            if isinstance(cam, OpenCVCamera):
                logger_mp.info(f"[Image Server] wide camera {cam.id} resolution: {cam.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)} x {cam.cap.get(cv2.CAP_PROP_FRAME_WIDTH)}")
            elif isinstance(cam, RealSenseCamera):
                logger_mp.info(f"[Image Server] wide camera {cam.serial_number} resolution: {cam.img_shape[0]} x {cam.img_shape[1]}")
            else:
                logger_mp.warning("[Image Server] Unknown camera type in wide_cameras.")

        logger_mp.info("[Image Server] Image server has started, waiting for client connections...")



    def _init_performance_metrics(self):
        self.frame_count = 0  # Total frames sent
        self.time_window = 1.0  # Time window for FPS calculation (in seconds)
        self.frame_times = deque()  # Timestamps of frames sent within the time window
        self.start_time = time.time()  # Start time of the streaming

    def _update_performance_metrics(self, current_time):
        # Add current time to frame times deque
        self.frame_times.append(current_time)
        # Remove timestamps outside the time window
        while self.frame_times and self.frame_times[0] < current_time - self.time_window:
            self.frame_times.popleft()
        # Increment frame count
        self.frame_count += 1

    def _print_performance_metrics(self, current_time):
        if self.frame_count % 30 == 0:
            elapsed_time = current_time - self.start_time
            real_time_fps = len(self.frame_times) / self.time_window
            logger_mp.info(f"[Image Server] Real-time FPS: {real_time_fps:.2f}, Total frames sent: {self.frame_count}, Elapsed time: {elapsed_time:.2f} sec")

    def _close(self):
        for cam in self.head_cameras:
            cam.release()
        for cam in self.wide_cameras:
            cam.release()
        self.socket.close()
        self.context.term()
        logger_mp.info("[Image Server] The server has been closed.")

    def send_process(self):
        try:
            while True:
                head_frames = []
                for cam in self.head_cameras:
                    if self.head_camera_type == 'opencv':
                        color_image = cam.get_frame()
                        if color_image is None:
                            logger_mp.error("[Image Server] Head camera frame read is error.")
                            break
                    elif self.head_camera_type == 'realsense':
                        color_image, depth_iamge = cam.get_frame()
                        if color_image is None:
                            logger_mp.error("[Image Server] Head camera frame read is error.")
                            break
                    head_frames.append(color_image)
                if len(head_frames) != len(self.head_cameras):
                    break
                head_color = cv2.hconcat(head_frames)
                
                if self.wide_cameras:
                    wide_frames = []
                    for cam in self.wide_cameras:
                        if self.wide_camera_type == 'opencv':
                            color_image = cam.get_frame()
                            if color_image is None:
                                logger_mp.error("[Image Server] wide camera frame read is error.")
                                break
                        elif self.wide_camera_type == 'realsense':
                            color_image, depth_iamge = cam.get_frame()
                            if color_image is None:
                                logger_mp.error("[Image Server] wide camera frame read is error.")
                                break
                        wide_frames.append(color_image)
                    wide_color = cv2.hconcat(wide_frames)

                    # Concatenate head and wide frames
                    full_color = cv2.hconcat([head_color, wide_color])
                else:
                    full_color = head_color

                ret, buffer = cv2.imencode('.jpg', full_color)
                if not ret:
                    logger_mp.error("[Image Server] Frame imencode is failed.")
                    continue

                jpg_bytes = buffer.tobytes()

                if self.Unit_Test:
                    timestamp = time.time()
                    frame_id = self.frame_count
                    header = struct.pack('dI', timestamp, frame_id)  # 8-byte double, 4-byte unsigned int
                    message = header + jpg_bytes
                else:
                    message = jpg_bytes

                self.socket.send(message)

                if self.Unit_Test:
                    current_time = time.time()
                    self._update_performance_metrics(current_time)
                    self._print_performance_metrics(current_time)

        except KeyboardInterrupt:
            logger_mp.warning("[Image Server] Interrupted by user.")
        finally:
            self._close()


if __name__ == "__main__":
    default_servo_cfg = (
        Path(__file__).resolve().parent.parent / "g1_comp_servo_service" / "config" / "config.yaml"
    )
    parser = argparse.ArgumentParser(
        description="Stream head + wide camera over ZMQ. Optional: set wide-camera Dynamixel "
        "angles (deg) at startup — same mapping as test_joint0_control / test_joint1_control (motor IDs 0 and 1)."
    )
    parser.add_argument(
        "wide_servo_deg",
        nargs="*",
        type=float,
        metavar="DEG",
        help="Optional: joint0 and joint1 angles (deg) for motor ids 0 and 1. "
        "If omitted, defaults are applied (0 and -20).",
    )
    parser.add_argument(
        "-s",
        "--serial",
        default=os.environ.get("WIDE_SERVO_SERIAL", "/dev/ttyUSB0"),
        help="Dynamixel serial device (default: /dev/ttyUSB0 or WIDE_SERVO_SERIAL).",
    )
    parser.add_argument(
        "--servo-config",
        type=Path,
        default=default_servo_cfg,
        help=f"Path to g1_comp_servo_service config.yaml (default: {default_servo_cfg})",
    )
    args = parser.parse_args()

    default_joint0_deg = 0.0
    default_joint1_deg = -20.0

    if len(args.wide_servo_deg) not in (0, 2):
        parser.error("Provide zero or two wide servo angles (joint0_deg joint1_deg), e.g. 10 -10")

    if len(args.wide_servo_deg) == 2:
        j0, j1 = args.wide_servo_deg[0], args.wide_servo_deg[1]
    else:
        j0, j1 = default_joint0_deg, default_joint1_deg

    _apply_wide_camera_servo_targets(j0, j1, args.serial, args.servo_config)

    config = {
        'fps': 30,
        'head_camera_type': 'realsense',
        'head_camera_image_shape': [480, 640],
        'head_camera_id_numbers': ["406122070312"],
        'wide_camera_type': 'realsense',
        'wide_camera_image_shape': [480, 640],
        'wide_camera_id_numbers': ["246322302856"],
    }

    server = ImageServer(config, Unit_Test=False)
    server.send_process()