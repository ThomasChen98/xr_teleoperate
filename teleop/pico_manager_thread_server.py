# Pico SMPL stream server for body tracking visualization

"""

# Recommended Command Line Arguments:
    # With VR3 PT visualization (by --vis_vr3pt) and optional SMPL body visualization (by --vis_smpl)
    # If you want to enable waist tracking in the VR3 PT visualization, please add --waist_tracking
    python pico_manager_thread_server.py --manager \
        --vis_vr3pt --vis_smpl \
        --waist_tracking

    # VR3 PT visualization only (without SMPL body) — lower latency
    python pico_manager_thread_server.py --manager --vis_vr3pt

# DEBUG VR3 PT VISUALIZATION:
    # A standalone test mode that captures one live frame and visualizes it.
    python pico_manager_thread_server.py --vr3pt_live

# TIMING COMPARISON:
    # The visualizer automatically reports timing every 5 seconds when running:
    #   [Vis Timing] vr3pt: X.XXms | smpl: X.XXms | render: X.XXms | vr3pt_only: X.XXms | both(vr3pt+smpl): X.XXms

"""

from collections import defaultdict, deque
from datetime import datetime
from enum import Enum, IntEnum
import os
import subprocess
import sys
import threading
import time

import msgpack
import numpy as np
from scipy.spatial.transform import Rotation as R, Rotation as sRot
import torch
import zmq

try:
    import cv2
except ImportError:
    cv2 = None

from gear_sonic.utils.teleop.zmq.zmq_poller import ZMQPoller
from gear_sonic.trl.utils.rotation_conversion import decompose_rotation_aa
from gear_sonic.trl.utils.torch_transform import (
    angle_axis_to_quaternion,
    compute_human_joints,
    quat_apply,
    quat_inv,
    quaternion_to_angle_axis,
    quaternion_to_rotation_matrix,
)

try:
    from gear_sonic.utils.teleop.zmq.zmq_planner_sender import (
        build_command_message,
        build_planner_message,
        pack_pose_message,
    )
except ImportError:

    def build_command_message(*args, **kwargs) -> bytes:
        raise RuntimeError("build_command_message unavailable")

    def build_planner_message(*args, **kwargs) -> bytes:
        raise RuntimeError("build_planner_message unavailable")

    def pack_pose_message(*args, **kwargs) -> bytes:
        raise RuntimeError("pack_pose_message unavailable")

try:
    from decoupled_wbc.data.hdf5_episode_writer import LiveEpisodeWriterHDF5
except ImportError:
    LiveEpisodeWriterHDF5 = None

try:
    from decoupled_wbc.control.sensor.composed_camera import ComposedCameraClientSensor
except ImportError:
    ComposedCameraClientSensor = None


class XRImageStreamSubscriber:
    """Subscribe to the raw JPEG xr_teleoperate image server stream."""

    def __init__(
        self,
        *,
        server_address: str,
        port: int,
        receive_timeout_ms: int = 1000,
        unit_test: bool = False,
    ) -> None:
        if cv2 is None:
            raise RuntimeError("OpenCV is required to decode image server frames")
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.SUB)
        self._socket.setsockopt(zmq.CONFLATE, 1)
        self._socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self._socket.setsockopt(zmq.RCVTIMEO, max(1, int(receive_timeout_ms)))
        self._socket.connect(f"tcp://{server_address}:{port}")
        self._unit_test = bool(unit_test)
        self._last_frame: np.ndarray | None = None

    def read(self) -> np.ndarray:
        try:
            message = self._socket.recv()
        except zmq.Again as exc:
            raise TimeoutError("Timed out waiting for image frame.") from exc

        payload = message[12:] if self._unit_test else message
        encoded = np.frombuffer(payload, dtype=np.uint8)
        frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if frame is None:
            raise RuntimeError("Failed to decode image frame from xr_teleoperate image server.")
        self._last_frame = frame
        return frame

    def read_latest_or_last(self) -> tuple[np.ndarray, bool]:
        try:
            return self.read(), False
        except TimeoutError:
            if self._last_frame is None:
                raise
            return self._last_frame.copy(), True

    def close(self) -> None:
        self._socket.close()
        self._context.term()


try:
    from gear_sonic.isaac_utils.rotations import remove_smpl_base_rot, smpl_root_ytoz_up
except ImportError:
    print("Warning: gear_sonic.isaac_utils.rotations not available.")
    remove_smpl_base_rot = None
    smpl_root_ytoz_up = None

try:
    import xrobotoolkit_sdk as xrt
except ImportError:
    xrt = None


def _start_xrt_service_and_init() -> None:
    """Best-effort startup for the XRoboToolkit runtime service."""
    subprocess.Popen(["bash", "/opt/apps/roboticsservice/runService.sh"])
    xrt.init()


def _try_get_body_poses_np() -> np.ndarray | None:
    """Return body poses if readable, even when the SDK readiness bit is stale."""
    if xrt is None:
        return None
    try:
        body_poses = xrt.get_body_joints_pose()
    except Exception:
        return None
    try:
        body_poses_np = np.asarray(body_poses, dtype=np.float32)
    except Exception:
        return None
    if body_poses_np.ndim != 2:
        return None
    if body_poses_np.shape[0] < 3 or body_poses_np.shape[1] < 7:
        return None
    if not np.isfinite(body_poses_np).all():
        return None
    return body_poses_np


def _wait_for_xrt_body_tracking(
    *,
    timeout_s: float = 5.0,
    continue_without_data: bool = False,
) -> np.ndarray | None:
    """Wait for body tracking without deadlocking forever on a flaky readiness flag."""
    print("Waiting for body tracking data...")
    deadline = time.monotonic() + max(0.0, float(timeout_s))
    while True:
        body_poses_np = _try_get_body_poses_np()
        flag_ready = False
        try:
            flag_ready = bool(xrt.is_body_data_available())
        except Exception:
            flag_ready = False

        if body_poses_np is not None:
            if not flag_ready:
                print(
                    "[XRT] Body pose stream is readable even though "
                    "is_body_data_available() is false; proceeding."
                )
            else:
                print("Body data available!")
            return body_poses_np
        if flag_ready:
            print("[XRT] Body tracking reported ready; proceeding.")
            return None

        if time.monotonic() >= deadline:
            if continue_without_data:
                print(
                    "[XRT] No body data yet; continuing startup and waiting in the "
                    "background reader instead of blocking forever."
                )
                return None
            raise TimeoutError(
                "Timed out waiting for Pico/XR body tracking data. "
                "The VR service may be running, but body pose frames are not arriving."
            )

        print("waiting for body data...")
        time.sleep(1.0)

try:
    from sshkeyboard import listen_keyboard, stop_listening
except ImportError:
    listen_keyboard = None
    stop_listening = None

try:
    from gear_sonic.utils.teleop.solver.hand.g1_gripper_ik_solver import (
        G1GripperInverseKinematicsSolver,
    )
except ImportError:
    print("Warning: G1GripperInverseKinematicsSolver not available.")
    G1GripperInverseKinematicsSolver = None

try:
    from gear_sonic.utils.teleop.vis.vr3pt_pose_visualizer import VR3PtPoseVisualizer
except ImportError:
    print("Warning: VR3PtPoseVisualizer not available (pyvista may not be installed).")
    VR3PtPoseVisualizer = None

try:
    from gear_sonic.utils.teleop.vis.vr3pt_pose_visualizer import get_g1_key_frame_poses
except ImportError:
    print("Warning: get_g1_key_frame_poses not available (pyvista may not be installed).")
    get_g1_key_frame_poses = None


class LocomotionMode(IntEnum):
    """Locomotion mode enum for robot movement."""

    IDLE = 0
    SLOW_WALK = 1
    WALK = 2
    RUN = 3
    IDLE_SQUAT = 4
    IDLE_KNEEL_TWO_LEGS = 5
    IDLE_KNEEL = 6
    IDLE_LYING_FACE_DOWN = 7
    CRAWLING = 8
    IDLE_BOXING = 9
    WALK_BOXING = 10
    LEFT_PUNCH = 11
    RIGHT_PUNCH = 12
    RANDOM_PUNCH = 13
    ELBOW_CRAWLING = 14
    LEFT_HOOK = 15
    RIGHT_HOOK = 16
    FORWARD_JUMP = 17
    STEALTH_WALK = 18
    INJURED_WALK = 19


class StreamMode(Enum):
    OFF = 0
    POSE = 1
    PLANNER = 2
    PLANNER_FROZEN_UPPER_BODY = 3
    POSE_PAUSE = 4
    PLANNER_VR_3PT = 5


class KeyboardRecordToggle:
    """Keyboard bridge for emitting edge-triggered record/discard controls."""

    def __init__(self):
        self._toggle_requested = False
        self._abort_requested = False
        self._lock = threading.Lock()
        self._thread = None
        self._active = False

    def _on_press(self, key):
        key = str(key).lower()
        if key == "s":
            with self._lock:
                self._toggle_requested = True
            print("[Keyboard] 's' pressed -> start/stop recording request queued")
            return
        if key == "x":
            with self._lock:
                self._abort_requested = True
            print("[Keyboard] 'x' pressed -> discard current episode request queued")
            return

    def _listen(self):
        try:
            listen_keyboard(on_press=self._on_press)
        except Exception as e:
            print(f"[Keyboard] listener stopped: {e}")

    def start(self):
        if listen_keyboard is None:
            print("[Keyboard] sshkeyboard not available, 's' recording toggle disabled")
            return
        if not sys.stdin.isatty():
            print("[Keyboard] stdin is not a TTY, 's' recording toggle disabled")
            return
        if self._active:
            return
        self._thread = threading.Thread(target=self._listen, daemon=True)
        self._thread.start()
        self._active = True
        print("[Keyboard] Press 's' to start/stop data collection, 'x' to discard")

    def consume_toggle(self) -> bool:
        with self._lock:
            toggle = self._toggle_requested
            self._toggle_requested = False
            return toggle

    def consume_abort(self) -> bool:
        with self._lock:
            abort = self._abort_requested
            self._abort_requested = False
            return abort

    def stop(self):
        if not self._active:
            return
        if stop_listening is not None:
            try:
                stop_listening()
            except Exception:
                pass
        if self._thread is not None:
            self._thread.join(timeout=0.5)
        self._thread = None
        self._active = False


class EpisodeControlPublisher:
    """Publish exporter-compatible episode control keys through a shared file queue."""

    DEFAULT_QUEUE_PATH = "/tmp/gr00t_episode_control_queue.txt"

    def __init__(self):
        self.queue_path = os.environ.get("GR00T_EPISODE_CONTROL_QUEUE", self.DEFAULT_QUEUE_PATH)
        self._active = True
        queue_dir = os.path.dirname(self.queue_path)
        if queue_dir:
            os.makedirs(queue_dir, exist_ok=True)
        print(f"[EpisodeControl] Publishing episode control to queue file {self.queue_path}")

    def publish_key(self, key: str) -> bool:
        try:
            with open(self.queue_path, "a", encoding="utf-8") as f:
                f.write(f"{time.time():.6f} {key}\n")
            return True
        except Exception as e:
            print(f"[EpisodeControl] Failed to publish '{key}': {e}")
            return False

    def close(self):
        self._active = False


class TeleopEpisodeRecorder:
    """Write HDF5 episodes directly from manager + deploy feedback."""

    LEFT_ARM_SLICE = slice(15, 22)
    RIGHT_ARM_SLICE = slice(22, 29)
    IMAGE_KEY_MAP = {
        "ego_cam": ("ego_cam", "cam_head", "head", "ego_view"),
        "ego_cam_2": ("ego_cam_2", "cam_wrist", "wrist", "cam_left_wrist", "left_wrist"),
    }

    def __init__(
        self,
        *,
        save_hdf5: bool,
        root_output_dir: str,
        dataset_name: str | None,
        task_prompt: str | None,
        hdf5_subdir: str,
        robot_name: str,
        fps: float,
        flush_every: int,
        zmq_feedback_host: str,
        zmq_feedback_port: int,
        camera_host: str,
        camera_port: int,
        camera_head_width: int | None = None,
        camera_secondary_width: int | None = None,
        show_image_stream: bool = False,
        image_receive_timeout_ms: int = 1000,
    ):
        self.enabled = False
        self.dataset_name = (
            dataset_name.strip().lower()
            if dataset_name
            else f"{datetime.now().strftime('%Y-%m-%d-%H-%M-%S')}-sonic-teleop"
        )
        self.task_prompt = self._normalize_task_prompt(task_prompt)
        self.dataset_root = os.path.join(root_output_dir, self.dataset_name)
        self.hdf5_save_dir = (
            os.path.join(self.dataset_root, hdf5_subdir) if hdf5_subdir else self.dataset_root
        )
        self.poller = None
        self.writer = None
        self.xr_image_subscriber = None
        self.legacy_image_subscriber = None
        self._xr_image_thread = None
        self._legacy_image_thread = None
        self._latest_xr_image_msg = None
        self._latest_legacy_image_msg = None
        self._latest_image_lock = threading.Lock()
        self._camera_endpoint = f"{camera_host}:{camera_port}"
        self._camera_head_width = (
            None if camera_head_width is None or int(camera_head_width) <= 0 else int(camera_head_width)
        )
        self._camera_secondary_width = (
            None
            if camera_secondary_width is None or int(camera_secondary_width) <= 0
            else int(camera_secondary_width)
        )
        self._show_image_stream = bool(show_image_stream)
        self._image_receive_timeout_ms = int(image_receive_timeout_ms)
        self._image_window_name = "GR00T Teleop Image Stream"
        self._xr_waiting_for_first_frame_logged = False
        self._xr_stream_reusing_last = False

        if save_hdf5 and LiveEpisodeWriterHDF5 is None:
            print("[HDF5Recorder] h5py writer unavailable, HDF5 recording disabled")
            save_hdf5 = False

        if save_hdf5:
            self.poller = ZMQPoller(
                host=zmq_feedback_host,
                port=zmq_feedback_port,
                topic="g1_debug",
            )
            try:
                self.writer = LiveEpisodeWriterHDF5(
                    save_dir=self.hdf5_save_dir,
                    robot_name=robot_name,
                    fps=fps,
                    flush_every=flush_every,
                )
            except Exception as e:
                print(f"[HDF5Recorder] {e}; HDF5 recording disabled")
                try:
                    self.poller.close()
                except Exception:
                    pass
                self.poller = None
                self.writer = None
            else:
                self.enabled = True
                print(
                    f"[HDF5Recorder] HDF5 saving enabled -> {self.hdf5_save_dir} "
                    f"(dataset: {self.dataset_name})"
                )
                print(f"[HDF5Recorder] Dataset root: {self.dataset_root}")
                print(f"[HDF5Recorder] HDF5 episodes will be written under {self.hdf5_save_dir}")

        if self._show_image_stream and cv2 is None:
            print("[TeleopImageStream] OpenCV unavailable, disabling image stream viewer")
            self._show_image_stream = False

        needs_image_stream = self.enabled or self._show_image_stream
        if not needs_image_stream:
            return

        if cv2 is None:
            print("[HDF5Recorder] OpenCV unavailable, image server stream disabled")
            return

        if ComposedCameraClientSensor is not None:
            try:
                self.legacy_image_subscriber = ComposedCameraClientSensor(
                    server_ip=camera_host,
                    port=camera_port,
                    receive_timeout_ms=self._image_receive_timeout_ms,
                )
                self._legacy_image_thread = threading.Thread(
                    target=self._legacy_image_reader_loop,
                    name="teleop-legacy-image-reader",
                    daemon=True,
                )
                self._legacy_image_thread.start()
                print(
                    "[TeleopImageStream] Legacy GR00T image client enabled in parallel "
                    f"for compatibility on {self._camera_endpoint}"
                )
            except Exception as e:
                self.legacy_image_subscriber = None
                print(
                    "[TeleopImageStream] Legacy GR00T image client unavailable on "
                    f"{self._camera_endpoint}: {e}"
                )

        try:
            self.xr_image_subscriber = XRImageStreamSubscriber(
                server_address=camera_host,
                port=camera_port,
                receive_timeout_ms=self._image_receive_timeout_ms,
            )
            self._xr_image_thread = threading.Thread(
                target=self._xr_image_reader_loop,
                name="teleop-xr-image-reader",
                daemon=True,
            )
            self._xr_image_thread.start()
            if self.enabled:
                print(f"[HDF5Recorder] Recording images from G1 image server {self._camera_endpoint}")
            if self._show_image_stream:
                print(
                    "[TeleopImageStream] Viewer enabled. "
                    f"Streaming camera frames from {self._camera_endpoint} "
                    "(press q in the image window to close the viewer)"
                )
        except Exception as e:
            self.xr_image_subscriber = None
            print(
                f"[HDF5Recorder] Failed to connect to G1 image server "
                f"{self._camera_endpoint}: {e}"
            )

    def is_recording(self) -> bool:
        return self.writer is not None and self.writer.is_recording()

    def current_length(self) -> int:
        if self.writer is None:
            return 0
        return self.writer.current_length()

    @staticmethod
    def _normalize_task_prompt(task_prompt: str | None) -> str | None:
        if task_prompt is None:
            return None
        normalized = task_prompt.strip().lower()
        return normalized or None

    @staticmethod
    def _binary_gripper_state(hand_q: np.ndarray, atol: float = 1e-4) -> np.float32:
        hand_q = np.asarray(hand_q, dtype=np.float32).reshape(-1)
        is_open = np.all(np.abs(hand_q) <= atol)
        return np.float32(0.0 if is_open else 1.0)

    @classmethod
    def _compact_q(cls, body_q: np.ndarray, left_hand_q: np.ndarray, right_hand_q: np.ndarray) -> np.ndarray:
        body_q = np.asarray(body_q, dtype=np.float32).reshape(-1)
        return np.concatenate(
            [
                body_q[cls.LEFT_ARM_SLICE],
                body_q[cls.RIGHT_ARM_SLICE],
                np.array(
                    [
                        cls._binary_gripper_state(left_hand_q),
                        cls._binary_gripper_state(right_hand_q),
                    ],
                    dtype=np.float32,
                ),
            ]
        ).astype(np.float32)

    @classmethod
    def _compact_dq(cls, body_dq: np.ndarray) -> np.ndarray:
        body_dq = np.asarray(body_dq, dtype=np.float32).reshape(-1)
        return np.concatenate(
            [
                body_dq[cls.LEFT_ARM_SLICE],
                body_dq[cls.RIGHT_ARM_SLICE],
                np.zeros(2, dtype=np.float32),
            ]
        ).astype(np.float32)

    @staticmethod
    def _full_joint_state(
        body_q: np.ndarray,
        left_hand_q: np.ndarray | None = None,
        right_hand_q: np.ndarray | None = None,
    ) -> np.ndarray:
        components = [np.asarray(body_q, dtype=np.float32).reshape(-1)]
        for hand_q in (left_hand_q, right_hand_q):
            if hand_q is None:
                continue
            hand_arr = np.asarray(hand_q, dtype=np.float32).reshape(-1)
            if hand_arr.size > 0:
                components.append(hand_arr)
        return np.concatenate(components).astype(np.float32)

    @staticmethod
    def _full_joint_velocity(
        body_dq: np.ndarray,
        left_hand_q: np.ndarray | None = None,
        right_hand_q: np.ndarray | None = None,
    ) -> np.ndarray:
        components = [np.asarray(body_dq, dtype=np.float32).reshape(-1)]
        for hand_q in (left_hand_q, right_hand_q):
            if hand_q is None:
                continue
            hand_arr = np.asarray(hand_q, dtype=np.float32).reshape(-1)
            if hand_arr.size > 0:
                components.append(np.zeros_like(hand_arr, dtype=np.float32))
        return np.concatenate(components).astype(np.float32)

    def _build_recording_metadata(self, *, mode_key: str, mode_name: str, trigger_source: str) -> dict[str, str]:
        metadata = {
            "dataset_name": self.dataset_name,
            "data_layout": "g1_binary_gripper_16d",
            "image_layout": "ego_cam,ego_cam_2",
            mode_key: mode_name,
            "trigger_source": trigger_source,
        }
        if self.task_prompt is not None:
            metadata["task_prompt"] = self.task_prompt
        return metadata

    def _xr_image_reader_loop(self) -> None:
        while self.xr_image_subscriber is not None:
            try:
                frame, reused_last = self.xr_image_subscriber.read_latest_or_last()
            except TimeoutError:
                if not self._xr_waiting_for_first_frame_logged:
                    print(
                        "[TeleopImageStream] Waiting for first image frame from "
                        f"{self._camera_endpoint}"
                    )
                    self._xr_waiting_for_first_frame_logged = True
                continue
            except Exception as e:
                if self.xr_image_subscriber is not None:
                    print(f"[HDF5Recorder] XR image subscriber stopped: {e}")
                break
            if frame is None:
                continue
            self._xr_waiting_for_first_frame_logged = False
            if reused_last and not self._xr_stream_reusing_last:
                print("[camera] image timeout; reusing last frame")
            elif not reused_last and self._xr_stream_reusing_last:
                print(f"[TeleopImageStream] Image stream resumed from {self._camera_endpoint}")
            self._xr_stream_reusing_last = reused_last
            message = self._build_xr_image_message(frame)
            with self._latest_image_lock:
                self._latest_xr_image_msg = message
            self._show_image_stream_frame(message, reused_last=reused_last)

    def _build_xr_image_message(self, frame: np.ndarray) -> dict[str, dict[str, np.ndarray] | dict[str, float]]:
        images = self._split_xr_composite_frame(
            frame,
            head_width=self._camera_head_width,
            secondary_width=self._camera_secondary_width,
        )
        now = time.time()
        timestamps = {key: now for key in images}
        return {"timestamps": timestamps, "images": images}

    @staticmethod
    def _split_xr_composite_frame(
        frame: np.ndarray,
        *,
        head_width: int | None = None,
        secondary_width: int | None = None,
    ) -> dict[str, np.ndarray]:
        image = np.asarray(frame, dtype=np.uint8)
        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError(f"Expected image [H, W, 3], got shape={image.shape}")

        total_width = int(image.shape[1])
        image_height = int(image.shape[0])

        if head_width is None:
            aspect_ratio = float(total_width) / max(1.0, float(image_height))
            head_width = total_width // 2 if aspect_ratio >= 1.8 else total_width

        head_width = max(1, min(int(head_width), total_width))
        head = image[:, :head_width]

        if total_width <= head_width:
            return {"ego_cam": head}

        remaining = total_width - head_width
        if secondary_width is None:
            secondary_width = remaining
        secondary_width = max(1, min(int(secondary_width), remaining))
        secondary = image[:, head_width : head_width + secondary_width]

        if secondary.shape[1] <= 0:
            return {"ego_cam": head}

        if secondary.shape[:2] != head.shape[:2]:
            secondary = TeleopEpisodeRecorder._resize_for_strip(secondary, head.shape[0])
            if secondary.shape[1] != head.shape[1]:
                secondary = cv2.resize(secondary, (head.shape[1], head.shape[0]))

        return {
            "ego_cam": head,
            "ego_cam_2": secondary,
        }

    def _legacy_image_reader_loop(self) -> None:
        while self.legacy_image_subscriber is not None:
            try:
                if hasattr(self.legacy_image_subscriber, "read_latest_or_last"):
                    message, _ = self.legacy_image_subscriber.read_latest_or_last()
                else:
                    message = self.legacy_image_subscriber.read()
            except TimeoutError:
                continue
            except Exception as e:
                if self.legacy_image_subscriber is not None:
                    print(
                        "[TeleopImageStream] Legacy GR00T image client stopped "
                        f"on {self._camera_endpoint}: {e}"
                    )
                break
            if message is None:
                continue
            with self._latest_image_lock:
                self._latest_legacy_image_msg = {
                    "timestamps": dict(message.get("timestamps", {})),
                    "images": {
                        key: np.asarray(value).copy()
                        for key, value in message.get("images", {}).items()
                    },
                }

    def _get_latest_image_msg(self) -> dict | None:
        with self._latest_image_lock:
            source = self._latest_xr_image_msg
            if source is None:
                return None
            return {
                "timestamps": dict(source.get("timestamps", {})),
                "images": {
                    key: np.asarray(value).copy()
                    for key, value in source.get("images", {}).items()
                },
            }

    @staticmethod
    def _resize_for_strip(image: np.ndarray, target_height: int) -> np.ndarray:
        if cv2 is None:
            return image
        if image.shape[0] == target_height:
            return image
        scale = target_height / max(1, image.shape[0])
        target_width = max(1, int(round(image.shape[1] * scale)))
        return cv2.resize(image, (target_width, target_height))

    def _show_image_stream_frame(self, image_msg: dict, *, reused_last: bool) -> None:
        if not self._show_image_stream or cv2 is None:
            return

        images = image_msg.get("images", {})
        if not images:
            return

        ordered_keys = [key for key in ("ego_cam", "ego_cam_2", "ego_view", "head") if key in images]
        ordered_keys.extend(key for key in images if key not in ordered_keys)

        frames = []
        valid_images = []
        for key in ordered_keys:
            arr = np.asarray(images[key], dtype=np.uint8)
            if arr.ndim != 3:
                continue
            valid_images.append((key, arr))
        if not valid_images:
            return

        target_height = min(image.shape[0] for _, image in valid_images)
        for key, image in valid_images:
            frame = self._resize_for_strip(image, target_height).copy()
            cv2.putText(
                frame,
                key,
                (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            frames.append(frame)

        combined = np.hstack(frames)
        status = "LIVE" if not reused_last else "REUSING LAST FRAME"
        color = (0, 255, 0) if not reused_last else (0, 0, 255)
        cv2.putText(
            combined,
            f"{self._camera_endpoint} | {status}",
            (10, max(24, combined.shape[0] - 12)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2,
            cv2.LINE_AA,
        )
        cv2.imshow(self._image_window_name, combined)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            self._show_image_stream = False
            cv2.destroyWindow(self._image_window_name)
            print("[TeleopImageStream] Viewer closed by user")

    @classmethod
    def _remap_images(cls, image_msg: dict | None) -> tuple[dict[str, np.ndarray], dict[str, float]]:
        if not image_msg:
            return {}, {}

        source_images = image_msg.get("images", {})
        source_timestamps = image_msg.get("timestamps", {})
        recorded_images: dict[str, np.ndarray] = {}
        recorded_timestamps: dict[str, float] = {}
        reference_image = None

        for target_key, source_keys in cls.IMAGE_KEY_MAP.items():
            for source_key in source_keys:
                image = source_images.get(source_key)
                if image is None:
                    continue
                arr = np.asarray(image, dtype=np.uint8)
                if arr.ndim != 3:
                    continue
                if cv2 is not None:
                    arr = cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)
                recorded_images[target_key] = arr
                timestamp = source_timestamps.get(source_key)
                if timestamp is not None:
                    recorded_timestamps[target_key] = float(timestamp)
                break

        if not recorded_images:
            return {}, {}

        return recorded_images, recorded_timestamps

    def start_recording(self, *, trigger_source: str, mode_name: str) -> None:
        if self.writer is None:
            print("[HDF5Recorder] HDF5 recording is disabled")
            return
        if self.writer.is_recording():
            return
        path = self.writer.start_recording(
            metadata=self._build_recording_metadata(
                mode_key="mode_at_start",
                mode_name=mode_name,
                trigger_source=trigger_source,
            )
        )
        print(f"[HDF5Recorder] Started recording via {trigger_source}: {path}")

    def stop_recording(self, *, trigger_source: str, mode_name: str, discarded: bool = False) -> None:
        if self.writer is None or not self.writer.is_recording():
            return
        episode_length = self.writer.current_length()
        path = self.writer.stop_recording(
            discarded=discarded,
            final_metadata=self._build_recording_metadata(
                mode_key="mode_at_stop",
                mode_name=mode_name,
                trigger_source=trigger_source,
            ),
        )
        if discarded:
            print(
                f"[HDF5Recorder] Discarded episode via {trigger_source} "
                f"(episode length: {episode_length} frames)"
            )
        else:
            print(
                f"[HDF5Recorder] Saved episode via {trigger_source} "
                f"(episode length: {episode_length} frames)"
            )
        if path is not None:
            print(f"[HDF5Recorder] Episode path: {path}")

    def poll_and_record(self, *, mode_name: str, extras: dict | None = None) -> bool:
        if self.writer is None or not self.writer.is_recording() or self.poller is None:
            return False

        data = self.poller.get_data(quiet=True)
        if data is None:
            return False

        unpacked = msgpack.unpackb(data, raw=False)
        required_keys = (
            "body_q_measured",
            "body_dq",
            "left_hand_q_measured",
            "right_hand_q_measured",
            "last_action",
            "last_left_hand_action",
            "last_right_hand_action",
        )
        if any(key not in unpacked for key in required_keys):
            missing = [key for key in required_keys if key not in unpacked]
            print(f"[HDF5Recorder] Missing feedback keys, skipped frame: {missing}")
            return False

        body_q = np.asarray(unpacked["body_q_measured"], dtype=np.float32)
        body_dq = np.asarray(unpacked["body_dq"], dtype=np.float32)
        left_hand_q = np.asarray(unpacked["left_hand_q_measured"], dtype=np.float32)
        right_hand_q = np.asarray(unpacked["right_hand_q_measured"], dtype=np.float32)
        last_action = np.asarray(unpacked["last_action"], dtype=np.float32)
        last_left_hand_action = np.asarray(unpacked["last_left_hand_action"], dtype=np.float32)
        last_right_hand_action = np.asarray(unpacked["last_right_hand_action"], dtype=np.float32)
        image_msg = self._get_latest_image_msg()
        recorded_images, recorded_timestamps = self._remap_images(image_msg)
        frame_extras: dict[str, np.ndarray | np.float64 | np.int64] = {
            "observations/full_qpos": self._full_joint_state(body_q, left_hand_q, right_hand_q),
            "observations/full_qvel": self._full_joint_velocity(body_dq, left_hand_q, right_hand_q),
            "actions/full_joint_action": self._full_joint_state(
                last_action,
                last_left_hand_action,
                last_right_hand_action,
            ),
            "timestamps/proprio": np.float64(time.time()),
            "timestamps/main_loop": np.float64(time.monotonic()),
            "frame_index": np.int64(self.writer.current_length()),
        }
        if extras:
            frame_extras.update(extras)
        for camera_name, timestamp in recorded_timestamps.items():
            frame_extras[f"timestamps/images/{camera_name}"] = np.float64(timestamp)
        _ = mode_name

        self.writer.add_timestep(
            qpos=self._compact_q(body_q, left_hand_q, right_hand_q),
            qvel=self._compact_dq(body_dq),
            action=self._compact_q(last_action, last_left_hand_action, last_right_hand_action),
            images=recorded_images,
            extras=frame_extras,
        )
        return True

    def close(self) -> None:
        if self.writer is not None and self.writer.is_recording():
            self.stop_recording(trigger_source="shutdown", mode_name="shutdown", discarded=False)
        if self.poller is not None:
            try:
                self.poller.close()
            except Exception:
                pass
        self.poller = None
        xr_image_subscriber = self.xr_image_subscriber
        self.xr_image_subscriber = None
        if xr_image_subscriber is not None:
            try:
                xr_image_subscriber.close()
            except Exception:
                pass
        legacy_image_subscriber = self.legacy_image_subscriber
        self.legacy_image_subscriber = None
        if legacy_image_subscriber is not None:
            try:
                legacy_image_subscriber.close()
            except Exception:
                pass
        if self._xr_image_thread is not None:
            self._xr_image_thread.join(timeout=1.0)
        self._xr_image_thread = None
        if self._legacy_image_thread is not None:
            self._legacy_image_thread.join(timeout=1.0)
        self._legacy_image_thread = None
        if cv2 is not None and self._show_image_stream:
            try:
                cv2.destroyWindow(self._image_window_name)
            except Exception:
                pass


### Parse 3 point pose from SMPL
#
# OFFSETS: Rotation corrections applied to each keypoint to align SMPL joint frames
# with the desired robot/visualization coordinate convention.
#
# Index mapping (based on [0, 22, 23, 12].index(joint_id)):
#   - OFFSETS[0]: Root/Pelvis (joint 0)
#   - OFFSETS[1]: Left Wrist (joint 22)
#   - OFFSETS[2]: Right Wrist (joint 23)
#   - OFFSETS[3]: Neck (joint 12) - more stable than Head (joint 15) for body tracking
#
# Scipy euler rotation convention:
#   - Lowercase "xyz" = EXTRINSIC rotations (about the FIXED/ORIGINAL frame's axes)
#   - Uppercase "XYZ" = INTRINSIC rotations (about the ROTATING body's axes)
#
# For EXTRINSIC "xyz" with angles [a, b, c]:
#   All rotations are about the ORIGINAL frame's axes (before any rotation):
#     R_total = R_z(c) @ R_y(b) @ R_x(a)   (matrix multiplication order)
#   Applied as: first rotate 'a' about original X, then 'b' about original Y, then 'c' about original Z
#
# For INTRINSIC "XYZ" with angles [a, b, c]:
#   Each rotation is about the CURRENT (rotated) frame's axis:
#     R_total = R_x(a) @ R_y(b) @ R_z(c)   (matrix multiplication order)
#   Applied as: first rotate 'a' about X, then 'b' about NEW Y, then 'c' about NEW Z
#
OFFSETS = [
    sRot.from_euler("xyz", [0, 0, -90], degrees=True),  # Root: yaw -90° about fixed Z
    sRot.from_euler("xyz", [90, 0, 0], degrees=True),  # L-Wrist: roll +90° about fixed X
    sRot.from_euler(
        "xyz", [-90, 0, 180], degrees=True
    ),  # R-Wrist: roll -90° about fixed X, then yaw 180° about fixed Z
    sRot.from_euler("xyz", [0, 0, -90], degrees=True),  # Neck: yaw -90° about fixed Z
]


def _compute_rel_transform(pose, world_frame, scalar_first=True):
    """
    Transform a pose from Unity coordinate frame to robot coordinate frame.

    Args:
        pose: np.ndarray shape (7,) - [x, y, z, qx, qy, qz, qw] in Unity frame
        world_frame: np.ndarray shape (7,) - reference frame to compute relative transform
        scalar_first: bool - if True, quaternion is [qw, qx, qy, qz]; if False, [qx, qy, qz, qw]

    Returns:
        rel_pos: np.ndarray (3,) - position in robot frame
        rel_rot: np.ndarray (4,) - quaternion [qw, qx, qy, qz] in robot frame

    Coordinate transform matrix Q converts Unity (Y-up, left-handed) to Robot (Z-up, right-handed):
        Unity:  X-right, Y-up, Z-forward
        Robot:  X-forward, Y-left, Z-up
    """
    world_frame = world_frame.copy()

    # Q transforms Unity coordinates to Robot coordinates
    # Unity [x, y, z] -> Robot [-x, z, y]
    Q = np.array([[-1, 0, 0], [0, 0, 1], [0, 1, 0.0]])
    pose[:3] = Q @ pose[:3]
    world_frame[:3] = Q @ world_frame[:3]
    rot_base = sRot.from_quat(world_frame[3:], scalar_first=scalar_first).as_matrix()
    rot = sRot.from_quat(pose[3:], scalar_first=scalar_first).as_matrix()
    rel_rot = sRot.from_matrix(Q @ (rot_base.T @ rot) @ Q.T)
    rel_pos = sRot.from_matrix(Q @ rot_base.T @ Q.T).apply(pose[:3] - world_frame[:3])
    return rel_pos, rel_rot.as_quat(scalar_first=True)


def _process_3pt_pose(smpl_pose_np):
    """
    Extract 3-point VR pose (L-Wrist, R-Wrist, Neck) from full SMPL body joint poses.

    NOTE: We use Neck (joint 12) instead of Head (joint 15) because:
      - Neck is more rigidly coupled to the torso
      - Head has high DoF (looking around) which doesn't reflect body pose
      - Neck provides more stable tracking for upper body orientation

    Args:
        smpl_pose_np: np.ndarray shape (24, 7) - 24 SMPL joints, each [x, y, z, qx, qy, qz, qw]
                      in Unity frame (scalar-last quaternion format)

    Returns:
        vr_3pt_pose: np.ndarray shape (3, 7) - 3 keypoints in robot frame
                     Each row is [x, y, z, qw, qx, qy, qz] (scalar-FIRST quaternion format)
                     Row 0: Left Wrist (SMPL joint 22)
                     Row 1: Right Wrist (SMPL joint 23)
                     Row 2: Neck (SMPL joint 12)

                     IMPORTANT: Positions and orientations are RELATIVE TO ROOT (pelvis).

    Processing Steps:
        1. Transform all 24 joints from Unity frame to robot frame
        2. Extract 4 keypoints: Root(0), L-Wrist(22), R-Wrist(23), Neck(12)
        3. Apply per-joint rotation OFFSETS to align joint frames
        4. Make L-Wrist, R-Wrist, Neck relative to Root (both position and orientation)
        5. Return only the 3 non-root keypoints

    Note: Position calibration (wrist offsets, neck kinematic chain) is done in
          ThreePointPose.apply_calibration() to ensure consistency with calibrated
          orientations.
    """

    # Defensive copy: _compute_rel_transform modifies pose[:3] in-place, which would
    # corrupt the caller's array (e.g. PicoReader._latest) and cause wrong results
    # if the same sample is processed more than once.
    smpl_pose_np = smpl_pose_np.copy()

    # =========================================================================
    # STEP 1: Transform all joints from Unity frame to robot frame
    # =========================================================================
    # Input: smpl_pose_np[i] = [x, y, z, qx, qy, qz, qw] in Unity frame (scalar-last)
    # Output: body_poses[i] = [x, y, z, qw, qx, qy, qz] in robot frame (scalar-first)
    body_poses = np.zeros((smpl_pose_np.shape[0], 7), dtype=np.float32)
    for i in range(smpl_pose_np.shape[0]):
        pos, orn = _compute_rel_transform(
            smpl_pose_np[i], [0, 0, 0, 0, 0, 0, 1], scalar_first=False
        )
        body_poses[i, :3] = pos  # Position in robot frame
        body_poses[i, 3:] = orn  # Quaternion [qw, qx, qy, qz] in robot frame

    # =========================================================================
    # STEP 2 & 3: Extract 4 keypoints and apply rotation OFFSETS
    # =========================================================================
    # We only care about these SMPL joint indices:
    #   - Joint 0:  Root/Pelvis (reference frame)
    #   - Joint 22: Left Wrist
    #   - Joint 23: Right Wrist
    #   - Joint 12: Neck (more stable than Head joint 15)
    #
    # kp_poses maps these to indices 0, 1, 2, 3 respectively
    positions = np.array([[p[0], p[1], p[2]] for p in body_poses])
    kp_poses = np.zeros((4, 7), dtype=np.float32)

    for i, pose in enumerate(body_poses):
        if i not in [0, 22, 23, 12]:
            continue  # Skip joints we don't care about

        pos = positions[i]

        # Map SMPL joint index to our keypoint index (0-3)
        # rel_i: 0=Root, 1=L-Wrist, 2=R-Wrist, 3=Neck
        rel_i = [0, 22, 23, 12].index(i)

        # Extract quaternion and apply rotation offset
        # pose[3:7] is [qw, qx, qy, qz] (scalar-first from _compute_rel_transform)
        quat = np.array([pose[3], pose[4], pose[5], pose[6]])

        # Apply offset: new_rotation = original_rotation * OFFSET
        # This post-multiplies the offset (intrinsic rotation)
        rot_quat = (sRot.from_quat(quat, scalar_first=True) * OFFSETS[rel_i]).as_quat(
            scalar_first=False
        )

        kp_poses[rel_i, 3:] = rot_quat  # Store as scalar-last temporarily for scipy compatibility
        kp_poses[rel_i, :3] = pos

    # =========================================================================
    # STEP 4: Make positions and orientations RELATIVE TO ROOT
    # =========================================================================
    # This transforms everything into the root's local coordinate frame.
    # After this step:
    #   - Root's position would be (0,0,0) and orientation identity (but we don't return root)
    #   - Other keypoints are expressed relative to root
    root_pos = kp_poses[0, :3].copy()
    root_quat = kp_poses[0, 3:].copy()  # Still scalar-last for scipy

    for i in range(1, 4):
        # Position: subtract root position, then rotate by inverse of root orientation
        kp_poses[i, :3] = sRot.from_quat(root_quat).inv().apply(kp_poses[i, :3] - root_pos)

        # Orientation: compute relative rotation (root_inv * keypoint_rot)
        # Result stored as scalar-FIRST [qw, qx, qy, qz]
        kp_poses[i, 3:] = (
            sRot.from_quat(root_quat).inv() * sRot.from_quat(kp_poses[i, 3:])
        ).as_quat(scalar_first=True)

    # =========================================================================
    # STEP 5: Return only L-Wrist, R-Wrist, Neck (skip Root)
    # =========================================================================
    # NOTE: Position and orientation calibration (including neck position via kinematic
    #       chain) is done in ThreePointPose.apply_calibration() to ensure consistency
    #       between calibrated orientation and computed neck position.
    # kp_poses[1:] = indices 1, 2, 3 = L-Wrist, R-Wrist, Neck
    # Each row: [x, y, z, qw, qx, qy, qz] relative to root, scalar-first quaternion
    return kp_poses[1:]


# =============================================================================
# VR 3-Point Pose Visualization Functions
# =============================================================================


def run_vr3pt_visualizer_test():
    """
    Standalone test for VR 3-point pose visualizer using PyVista.
    Run this to verify the reference frames are displayed correctly.
    """
    if VR3PtPoseVisualizer is None:
        raise ImportError("VR3PtPoseVisualizer not available. Install pyvista: pip install pyvista")

    print("=" * 60)
    print("VR 3-Point Pose Visualizer Test (PyVista)")
    print("=" * 60)
    print("\nExpected reference frames (all with RGB axes for XYZ):")
    print("  1. WHITE ball at origin (0, 0, 0) - World frame")
    print("  2. CYAN ball at (0, 0, 0.35) - Looking forward (identity)")
    print("  3. MAGENTA ball at (0, 0.4, 0.25) - Looking left (yaw +90°)")
    print("  4. YELLOW ball at (0.4, 0, 0.15) - Looking down (pitch +90°)")
    print("\nClose the window to exit.")
    print("=" * 60)

    visualizer = VR3PtPoseVisualizer(axis_length=0.08, ball_radius=0.015, with_g1_robot=True)
    visualizer.show_static()


def run_vr3pt_live_visualizer():
    """
    Live visualizer for real VR 3-point pose data from Pico.
    Captures one frame from Pico and displays it alongside reference frames.
    """
    if xrt is None:
        raise ImportError(
            "XRoboToolkit SDK not available. Install xrobotoolkit_sdk to use live visualizer."
        )

    if VR3PtPoseVisualizer is None:
        raise ImportError("VR3PtPoseVisualizer not available. Install pyvista: pip install pyvista")

    print("=" * 60)
    print("VR 3-Point Pose Live Visualizer (PyVista)")
    print("=" * 60)

    # Initialize XRT
    _start_xrt_service_and_init()
    body_poses_np = _wait_for_xrt_body_tracking(timeout_s=15.0, continue_without_data=False)

    print("Body data available! Capturing VR 3-point pose...")

    # Capture body poses and compute vr_3pt_pose
    if body_poses_np is None:
        body_poses_np = _try_get_body_poses_np()
    if body_poses_np is None:
        raise RuntimeError("Failed to read Pico body tracking pose after readiness check.")

    # Process to get 3-point pose (L-Wrist, R-Wrist, Neck)
    vr_3pt_pose = _process_3pt_pose(body_poses_np)

    print(f"\nCaptured vr_3pt_pose shape: {vr_3pt_pose.shape}")
    print(f"  L-Wrist: pos={vr_3pt_pose[0, :3]}, quat_wxyz={vr_3pt_pose[0, 3:]}")
    print(f"  R-Wrist: pos={vr_3pt_pose[1, :3]}, quat_wxyz={vr_3pt_pose[1, 3:]}")
    print(f"  Neck:    pos={vr_3pt_pose[2, :3]}, quat_wxyz={vr_3pt_pose[2, 3:]}")

    print("\nDisplaying visualization...")
    print("Close the window to exit.")
    print("=" * 60)

    visualizer = VR3PtPoseVisualizer(axis_length=0.08, ball_radius=0.015, with_g1_robot=True)
    visualizer.show_with_vr_pose(vr_3pt_pose)


def run_vr3pt_realtime_visualizer(update_hz: int = 10):
    """
    Real-time visualizer for VR 3-point pose data from Pico.
    Continuously updates the visualization with live data.

    Args:
        update_hz: Update rate in Hz (default 10)
    """
    if xrt is None:
        raise ImportError(
            "XRoboToolkit SDK not available. Install xrobotoolkit_sdk to use realtime visualizer."
        )

    if VR3PtPoseVisualizer is None:
        raise ImportError("VR3PtPoseVisualizer not available. Install pyvista: pip install pyvista")

    print("=" * 60)
    print("VR 3-Point Pose Real-time Visualizer (PyVista)")
    print("=" * 60)

    # Initialize XRT
    _start_xrt_service_and_init()
    _wait_for_xrt_body_tracking(timeout_s=15.0, continue_without_data=False)

    print("Body data available! Starting real-time visualization...")
    print(f"Update rate: {update_hz} Hz")
    print("Close the window or press 'q' to exit.")
    print("=" * 60)

    # Use the VR3PtPoseVisualizer for real-time visualization with G1 robot
    visualizer = VR3PtPoseVisualizer(axis_length=0.08, ball_radius=0.015, with_g1_robot=True)
    visualizer.create_realtime_plotter(interactive=True)

    try:
        while visualizer.is_open:
            # Get new data from Pico
            body_poses = xrt.get_body_joints_pose()
            body_poses_np = np.array(body_poses)
            vr_3pt_pose = _process_3pt_pose(body_poses_np)

            # Update visualization
            visualizer.update_vr_poses(vr_3pt_pose)
            visualizer.render()

            time.sleep(1.0 / update_hz)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        visualizer.close()


def process_smpl_joints(body_pose, global_orient, transl):
    """Process SMPL parameters to compute local joints.

    Args:
        body_pose: Body pose tensor, shape (T, 69)
        global_orient: Global orientation tensor, shape (T, 3)
        transl: Translation tensor, shape (T, 3)

    Returns:
        Dictionary with processed joints and parameters
    """
    # Convert global_orient to quaternion and apply transformations (robust if utils missing)
    global_orient_quat = angle_axis_to_quaternion(global_orient)
    if smpl_root_ytoz_up is not None:
        global_orient_quat = smpl_root_ytoz_up(global_orient_quat)
    global_orient_new = quaternion_to_angle_axis(global_orient_quat)

    # Compute joints and vertices using SMPL model (single forward pass)
    joints = compute_human_joints(
        body_pose=body_pose[..., :63],
        global_orient=global_orient_new,
    )  # (*, 24, 3)

    # Apply base rotation removal and compute local joints
    if remove_smpl_base_rot is not None:
        global_orient_quat = remove_smpl_base_rot(global_orient_quat, w_last=False)

    global_orient_quat_inv = quat_inv(global_orient_quat).unsqueeze(1).repeat(1, joints.shape[1], 1)
    smpl_joints_local = quat_apply(global_orient_quat_inv, joints)
    global_orient_mat = quaternion_to_rotation_matrix(global_orient_quat)
    global_orient_6d = global_orient_mat[..., :2].reshape(1, 6)

    return {
        "smpl_pose": body_pose,
        "joints": joints,
        "smpl_joints_local": smpl_joints_local,
        "global_orient_quat": global_orient_quat,
        "global_orient_6d": global_orient_6d,
        "adjusted_transl": transl,
    }


def generate_finger_data(hand: str, trigger: float, grip: float) -> np.ndarray:
    """
    Generate finger position data from Pico controller button states.

    Args:
        hand: "left" or "right"
        trigger: Trigger button value (0-1)
        grip: Grip button value (0-1)

    Returns:
        Array of shape [25, 4, 4] representing fingertip positions
    """
    fingertips = np.zeros([25, 4, 4])

    thumb = 0
    middle = 10
    # Control thumb based on shoulder button state (index 4 is thumb tip)
    fingertips[4 + thumb, 0, 3] = 1.0  # open thumb
    if trigger > 0.5:
        fingertips[4 + middle, 0, 3] = 1.0  # close middle

    return fingertips


# Joystick deadzone threshold
JOYSTICK_DEADZONE = 0.15


class YawAccumulator:
    """Accumulates yaw heading angle based on joystick input."""

    def __init__(self, yaw_gain: float = 1.5, deadzone: float = JOYSTICK_DEADZONE):
        self.yaw_gain = yaw_gain
        self.deadzone = deadzone
        self.reset()

    def reset(self):
        """Reset facing direction to default (1,0,0)."""
        self.heading = [1.0, 0.0, 0.0]
        self.yaw_angle_rad = 0.0
        self.dyaw = 0.0
        print("YawAccumulator: reset yaw angle to 0.0")

    def yaw_angle(self) -> float:
        """Get current yaw angle in radians."""
        return self.yaw_angle_rad

    def yaw_angle_change(self) -> float:
        """Get current yaw angle change in radians."""
        return self.dyaw

    def update(self, rx: float, dt: float) -> list[float]:
        """
        Update facing direction based on right stick x-axis input.

        Args:
            rx: Right stick x-axis value (-1 to 1)
            dt: Time delta in seconds

        Returns:
            Facing direction as [x, y, 0.0]
        """
        self.dyaw = self.yaw_gain * (-rx) * dt
        if abs(rx) >= self.deadzone:
            self.yaw_angle_rad += self.dyaw
            self.heading = [np.cos(self.yaw_angle_rad), np.sin(self.yaw_angle_rad), 0.0]
        return self.heading


def compute_from_body_poses(parent_indices: list, device, body_poses_np: np.ndarray):
    """
    Compute local joints and body orientation from provided body_poses_np.
    """
    positions = body_poses_np[:, :3]
    global_quats = body_poses_np[:, [6, 3, 4, 5]]

    # Convert to local rotations
    global_rots = sRot.from_quat(global_quats, scalar_first=True)
    global_rots = global_rots * sRot.from_euler("y", 180, degrees=True)

    local_rots = []
    for i in range(24):
        if parent_indices[i] == -1:
            local_rots.append(global_rots[i])
        else:
            local_rot = global_rots[parent_indices[i]].inv() * global_rots[i]
            local_rots.append(local_rot)

    pose_aa = np.array([rot.as_rotvec() for rot in local_rots])

    body_pose = torch.from_numpy(pose_aa[1:].flatten()).float().to(device).unsqueeze(0)
    global_orient = torch.from_numpy(pose_aa[0]).float().to(device).unsqueeze(0)
    transl = torch.from_numpy(positions[0]).float().to(device).unsqueeze(0)

    return process_smpl_joints(body_pose, global_orient, transl)


# def compute_latest_frame(parent_indices: list, device) -> tuple[np.ndarray, np.ndarray]:
#     """
#     Pull body data from XRoboToolkit, compute local SMPL joints and body orientation.
#     Returns (smpl_joints_local_np [24,3], global_orient_quat_np [4,])
#     """
#     body_poses = xrt.get_body_joints_pose()
#     body_poses_np = np.array(body_poses)
#     return compute_from_body_poses(parent_indices, device, body_poses_np)


def init_hand_ik_solvers():
    """Initialize hand IK solvers if available."""
    if G1GripperInverseKinematicsSolver is not None:
        left_solver = G1GripperInverseKinematicsSolver(side="left")
        right_solver = G1GripperInverseKinematicsSolver(side="right")
        print("Hand IK solvers initialized")
        return left_solver, right_solver
    print("Warning: Hand IK solvers not available")
    return None, None


def get_controller_inputs():
    """Fetch controller button/trigger states from XRoboToolkit."""
    left_trigger = xrt.get_left_trigger()
    right_trigger = xrt.get_right_trigger()
    left_grip = xrt.get_left_grip()
    right_grip = xrt.get_right_grip()
    left_menu_button = xrt.get_left_menu_button()
    return left_menu_button, left_trigger, right_trigger, left_grip, right_grip


def get_controller_axes():
    """Fetch joystick axes (lx, ly, rx, ry). Falls back to zeros if not available."""
    if xrt is None:
        return 0.0, 0.0, 0.0, 0.0
    try:
        left_axis = xrt.get_left_axis()  # expected [x, y]
        right_axis = xrt.get_right_axis()  # expected [x, y]
        lx = float(left_axis[0]) if len(left_axis) >= 1 else 0.0
        ly = float(left_axis[1]) if len(left_axis) >= 2 else 0.0
        rx = float(right_axis[0]) if len(right_axis) >= 1 else 0.0
        ry = float(right_axis[1]) if len(right_axis) >= 2 else 0.0
        return lx, ly, rx, ry
    except Exception:
        return 0.0, 0.0, 0.0, 0.0


def get_menu_buttons():
    """Fetch both menu buttons (left, right). Falls back to False if not available."""
    if xrt is None:
        return False, False

    def _safe_btn(attr):
        try:
            fn = getattr(xrt, attr)
            return bool(fn())
        except Exception:
            return False

    left = _safe_btn("get_left_menu_button")
    right = _safe_btn("get_right_menu_button")
    return left, right


def get_axis_clicks():
    """Fetch both axis click buttons (left, right). Falls back to False if not available."""
    if xrt is None:
        return False, False

    def _safe_btn(attr):
        try:
            fn = getattr(xrt, attr)
            return bool(fn())
        except Exception:
            return False

    left = _safe_btn("get_left_axis_click")
    right = _safe_btn("get_right_axis_click")
    return left, right


def get_face_buttons():
    """Fetch primary face buttons A and X. Returns (a_pressed, x_pressed)."""
    if xrt is None:
        return False, False
    try:
        a_pressed = bool(xrt.get_A_button())
        x_pressed = bool(xrt.get_X_button())
        return a_pressed, x_pressed
    except Exception:
        return False, False


def get_abxy_buttons():
    """Fetch A,B,X,Y face buttons as booleans (a,b,x,y)."""
    if xrt is None:
        return False, False, False, False
    try:
        a_pressed = bool(xrt.get_A_button())
        b_pressed = bool(xrt.get_B_button())
        x_pressed = bool(xrt.get_X_button())
        y_pressed = bool(xrt.get_Y_button())
        return a_pressed, b_pressed, x_pressed, y_pressed
    except Exception:
        return False, False, False, False


def compute_hand_joints_from_inputs(
    left_solver, right_solver, left_trigger, left_grip, right_trigger, right_grip
) -> tuple[np.ndarray, np.ndarray]:
    """Compute left/right hand joints using IK solvers, or zeros if unavailable."""
    if left_solver is not None and right_solver is not None:
        left_finger_data = generate_finger_data("left", left_trigger, left_grip)
        right_finger_data = generate_finger_data("right", right_trigger, right_grip)
        left_hand_joints = left_solver({"position": left_finger_data})
        right_hand_joints = right_solver({"position": right_finger_data})
    else:
        left_hand_joints = np.zeros((1, 7), dtype=np.float32)
        right_hand_joints = np.zeros((1, 7), dtype=np.float32)
    return left_hand_joints, right_hand_joints


def _quat_lerp_normalized(q0: np.ndarray, q1: np.ndarray, alpha: float) -> np.ndarray:
    """
    Linear interpolate two quaternions and renormalize. Input shape (4,), xyzw order.
    Ensures shortest path by flipping sign if dot < 0.
    """
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
    q = (1.0 - alpha) * q0 + alpha * q1
    norm = np.linalg.norm(q)
    if norm > 0:
        q = q / norm
    return q


def _interp_pose_axis_angle(
    prev_pose: np.ndarray, curr_pose: np.ndarray, alpha: float
) -> np.ndarray:
    """
    Interpolate axis-angle joint poses by converting to quats, lerp-normalize, then back.
    prev_pose, curr_pose: (21,3) axis-angle (rotvec)
    Returns (21,3) axis-angle.
    """
    prev_quats = sRot.from_rotvec(prev_pose.reshape(-1, 3)).as_quat()  # (N,4) xyzw
    curr_quats = sRot.from_rotvec(curr_pose.reshape(-1, 3)).as_quat()
    out_quats = np.empty_like(prev_quats)
    for i in range(prev_quats.shape[0]):
        out_quats[i] = _quat_lerp_normalized(prev_quats[i], curr_quats[i], alpha)
    out_pose = sRot.from_quat(out_quats).as_rotvec().reshape(prev_pose.shape)
    return out_pose


class PicoReader:
    """
    Background reader that pulls Pico/XRT data as fast as possible and computes dt/FPS.
    """

    def __init__(self, max_queue_size: int = 15):
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._last_t = None
        self._fps_ema = 0.0
        self._last_stamp_ns = None
        self._latest = None
        self._lock = threading.Lock()

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=1.0)

    def get_latest(self):
        with self._lock:
            return self._latest

    def _run(self):
        last_report = time.time()
        next_probe_time = 0.0
        while not self._stop.is_set():
            body_poses_np = None
            flag_ready = False
            try:
                flag_ready = bool(xrt.is_body_data_available())
            except Exception:
                flag_ready = False

            if flag_ready:
                body_poses_np = _try_get_body_poses_np()
            elif time.monotonic() >= next_probe_time:
                body_poses_np = _try_get_body_poses_np()
                next_probe_time = time.monotonic() + 0.25

            if body_poses_np is None:
                time.sleep(0.001)
                continue

            try:
                stamp_ns = xrt.get_time_stamp_ns()
            except Exception:
                stamp_ns = time.time_ns()
            prev_stamp_ns = self._last_stamp_ns
            if prev_stamp_ns is not None and stamp_ns == prev_stamp_ns:
                time.sleep(0.000001)
                continue
            # Compute device-based dt/fps using timestamp deltas (ns -> s)
            device_dt = ((stamp_ns - prev_stamp_ns) * 1e-9) if prev_stamp_ns is not None else 0.0
            if device_dt > 0.0:
                inst = 1.0 / device_dt
                self._fps_ema = inst if self._fps_ema == 0.0 else (0.9 * self._fps_ema + 0.1 * inst)
            self._last_stamp_ns = stamp_ns
            t_realtime = time.time()
            t_monotonic = time.monotonic()
            try:
                sample = {
                    "body_poses_np": body_poses_np,
                    "timestamp_realtime": t_realtime,
                    "timestamp_monotonic": t_monotonic,
                    "timestamp_ns": stamp_ns,
                    "dt": device_dt,
                    "fps": self._fps_ema,
                }
                with self._lock:
                    self._latest = sample
                now = time.time()
                if now - last_report >= 5.0:
                    print(
                        f"[PicoReader] dt_ts: {device_dt*1000.0:.2f} ms, fps: {self._fps_ema:.2f}"
                    )
                    last_report = now
            except Exception as e:
                print(f"[PicoReader] read error: {e}")


def _pose_stream_common(
    socket,
    buffer_size: int,
    num_frames_to_send: int,
    target_fps: int,
    use_cuda: bool,
    record_dir: str,
    record_format: str,
    hdf5_recorder: TeleopEpisodeRecorder | None = None,
    stop_event: threading.Event | None = None,
    log_prefix: str = "PoseLoop",
    enable_vis_vr3pt: bool = False,
    with_g1_robot: bool = True,
    enable_waist_tracking: bool = False,
    enable_smpl_vis: bool = False,
):
    """Shared pose streaming loop used by run_pico."""
    if xrt is None:
        raise ImportError(
            "XRoboToolkit SDK not available. Install xrobotoolkit_sdk to run pose streaming."
        )

    # Create reader and start it
    reader = PicoReader(max_queue_size=buffer_size)
    reader.start()

    # Create 3-point pose processor with visualization settings
    three_point = ThreePointPose(
        enable_vis_vr3pt=enable_vis_vr3pt,
        with_g1_robot=with_g1_robot,
        enable_waist_tracking=enable_waist_tracking,
        enable_smpl_vis=enable_smpl_vis,
        log_prefix=log_prefix,
    )
    keyboard_toggle = KeyboardRecordToggle()
    keyboard_toggle.start()

    streamer = PoseStreamer(
        socket=socket,
        reader=reader,
        three_point=three_point,
        num_frames_to_send=num_frames_to_send,
        target_fps=target_fps,
        use_cuda=use_cuda,
        record_dir=record_dir,
        record_format=record_format,
        keyboard_toggle=keyboard_toggle,
        hdf5_recorder=hdf5_recorder,
        log_prefix=log_prefix,
    )

    if stop_event is None:
        stop_event = threading.Event()

    try:
        while not stop_event.is_set():
            streamer.run_once()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup resources
        keyboard_toggle.stop()
        reader.stop()
        three_point.close()


class ThreePointPose:
    """
    Encapsulates everything around calculating 3-point pose from SMPL input.

    This includes:
    - Processing SMPL poses to extract 3-point VR pose (L-Wrist, R-Wrist, Neck)
    - Calibration logic to align VR poses with G1 robot
    - Optional visualization of 3-point poses

    Calibration is done in two steps:
    1. Neck orientation: Captures initial neck orientation to align subsequent poses as upright
    2. Wrist positions: Aligns wrist positions to match G1 robot key frame positions
    """

    # Kinematic chain constants for neck position (matches VR3PtPoseVisualizer)
    TORSO_LINK_OFFSET_Z = 0.05  # meters from root to torso_link
    NECK_LINK_LENGTH = 0.35  # meters from torso_link to neck along neck's local Z

    def __init__(
        self,
        enable_vis_vr3pt: bool = False,
        with_g1_robot: bool = True,
        enable_waist_tracking: bool = False,
        enable_smpl_vis: bool = False,
        log_prefix: str = "ThreePointPose",
        robot_model=None,
    ):
        """
        Initialize 3-point pose processor.

        Args:
            enable_vis_vr3pt: Whether to enable VR 3pt pose visualization (requires display)
            with_g1_robot: Whether to include G1 robot in visualization
            enable_waist_tracking: Whether to enable waist tracking in visualization
            enable_smpl_vis: Whether to render SMPL body joints in the VR3pt visualizer
            log_prefix: Prefix for log messages
            robot_model: Optional pre-instantiated RobotModel. If None, will create one.
                        Used for FK-based calibration (no display required).
        """
        self.log_prefix = log_prefix
        self.with_g1_robot = with_g1_robot
        self.enable_waist_tracking = enable_waist_tracking
        self.enable_smpl_vis = enable_smpl_vis

        # Robot model for FK-based calibration (headless, no display required)
        self._robot_model = robot_model
        if self._robot_model is None:
            from gear_sonic.data.robot_model.instantiation.g1 import (
                instantiate_g1_robot_model,
            )

            self._robot_model = instantiate_g1_robot_model()
            print(f"[{log_prefix}] Robot model loaded for FK calibration")

        # Optional visualization (requires display + PyVista)
        self.vr3pt_visualizer = None
        if enable_vis_vr3pt:
            if VR3PtPoseVisualizer is None:
                raise ImportError(
                    "VR3PtPoseVisualizer could not be imported but --vis_vr3pt was requested. "
                    "Ensure pyvista is installed: pip install pyvista"
                )
            self.vr3pt_visualizer = VR3PtPoseVisualizer(
                axis_length=0.08,
                ball_radius=0.015,
                with_g1_robot=with_g1_robot,
                robot_model=self._robot_model,
                enable_waist_tracking=enable_waist_tracking,
                enable_smpl_vis=enable_smpl_vis,
            )
            self.vr3pt_visualizer.create_realtime_plotter(interactive=True)
            g1_str = " with G1 robot" if with_g1_robot else ""
            waist_str = " + waist tracking" if enable_waist_tracking else ""
            smpl_str = " + SMPL body" if enable_smpl_vis else ""
            print(f"[{log_prefix}] VR 3pt pose visualization enabled{g1_str}{waist_str}{smpl_str}")

        # Calibration state — triggered explicitly by calibrate_now() or reset_with_measured_q()
        self._calibration_pending = False
        self._calibration_neck_quat_inv: np.ndarray | None = None  # inv(initial neck quat)
        self._calibration_lwrist_offset: np.ndarray | None = None  # position offset
        self._calibration_rwrist_offset: np.ndarray | None = None
        self._calibration_lwrist_rot_offset: sRot | None = None  # orientation offset
        self._calibration_rwrist_rot_offset: sRot | None = None
        # Override robot q for FK during recalibration (e.g. measured joints for VR 3PT)
        self._override_robot_q: np.ndarray | None = None

    @property
    def is_pending(self) -> bool:
        """Check if calibration is pending."""
        return self._calibration_pending

    @property
    def is_calibrated(self) -> bool:
        """Check if calibration has been captured."""
        return self._calibration_neck_quat_inv is not None

    def process_smpl_pose(
        self,
        smpl_pose_np: np.ndarray,
        smpl_joints_local: np.ndarray | None = None,
    ) -> np.ndarray:
        """
        Process SMPL pose to extract and calibrate 3-point VR pose.

        Args:
            smpl_pose_np: np.ndarray shape (24, 7) - 24 SMPL joints
            smpl_joints_local: Optional np.ndarray shape (24, 3) - SMPL local joint
                               positions for body visualization. If provided and SMPL
                               visualization is enabled, the joint spheres are updated.

        Returns:
            vr_3pt_pose: np.ndarray shape (3, 7) - Calibrated 3-point pose
                         [L-Wrist, R-Wrist, Neck], each row [x, y, z, qw, qx, qy, qz]
        """
        # Extract raw 3-point pose from SMPL
        vr_3pt_pose_raw = _process_3pt_pose(smpl_pose_np)

        # Capture calibration on first valid frame (or after reset)
        if self._calibration_pending:
            self._capture_calibration(vr_3pt_pose_raw)

        # Apply calibration to get the final pose
        vr_3pt_pose = self._apply_calibration(vr_3pt_pose_raw)

        if self.vr3pt_visualizer is not None:
            self.vr3pt_visualizer.update_from_vr_pose(vr_3pt_pose, waist_scale=1.0)
            if smpl_joints_local is not None:
                self.vr3pt_visualizer.update_smpl_joints(smpl_joints_local)
            self.vr3pt_visualizer.render()

        return vr_3pt_pose

    def close(self) -> None:
        """Close and cleanup visualizer resources."""
        if self.vr3pt_visualizer is not None:
            try:
                self.vr3pt_visualizer.close()
            except Exception as e:
                print(f"[{self.log_prefix}] Warning: Error closing VR3pt visualizer: {e}")

    def calibrate_now(self, body_poses_np: np.ndarray) -> bool:
        """Calibrate using current SMPL frame against FK of all-zero body joints.
        Operator should be in zero-reference pose when calling this."""
        try:
            vr_3pt_pose_raw = _process_3pt_pose(body_poses_np)
            self._override_robot_q = np.zeros(29, dtype=np.float64)
            self._capture_calibration(vr_3pt_pose_raw)
            print(f"[{self.log_prefix}] Calibration completed (zero-pose reference)")
            return True
        except Exception as e:
            print(f"[{self.log_prefix}] Calibration failed: {e}")
            import traceback

            traceback.print_exc()
            return False

    def _capture_calibration(self, vr_3pt_pose: np.ndarray) -> None:
        """Capture calibration offsets from vr_3pt_pose against G1 FK reference.
        If neck calibration already exists (e.g. from calibrate_now), it is preserved
        to avoid jumps from SMPL noise during recalibration."""

        # Step 1: Neck orientation — only capture if not already set
        if self._calibration_neck_quat_inv is None:
            neck_quat_wxyz = vr_3pt_pose[2, 3:].copy()
            neck_rot = sRot.from_quat(neck_quat_wxyz, scalar_first=True)
            self._calibration_neck_quat_inv = neck_rot.inv().as_quat(scalar_first=True)
        calib_inv_rot = sRot.from_quat(self._calibration_neck_quat_inv, scalar_first=True)

        # Step 2: Rotate VR wrist positions/orientations by neck inverse
        lwrist_pos_corrected = calib_inv_rot.apply(vr_3pt_pose[0, :3].copy())
        rwrist_pos_corrected = calib_inv_rot.apply(vr_3pt_pose[1, :3].copy())
        lwrist_rot_corrected = calib_inv_rot * sRot.from_quat(vr_3pt_pose[0, 3:], scalar_first=True)
        rwrist_rot_corrected = calib_inv_rot * sRot.from_quat(vr_3pt_pose[1, 3:], scalar_first=True)

        # Step 3: Get G1 FK reference poses
        if self._robot_model is None:
            raise RuntimeError(
                "Robot model is required for calibration but was not loaded. "
                "Ensure the G1 robot model and URDF are available."
            )
        if get_g1_key_frame_poses is None:
            raise RuntimeError(
                "get_g1_key_frame_poses could not be imported. "
                "Ensure gear_sonic.utils.teleop.vis.vr3pt_pose_visualizer is available."
            )

        # Convert 29-DOF override to full model config if needed
        if self._override_robot_q is not None:
            robot_q = self._robot_model.get_configuration_from_actuated_joints(
                body_actuated_joint_values=self._override_robot_q[:29]
            )
        else:
            robot_q = None
        g1_poses = get_g1_key_frame_poses(self._robot_model, q=robot_q)

        g1_lwrist_pos = g1_poses["left_wrist"]["position"]
        g1_rwrist_pos = g1_poses["right_wrist"]["position"]
        g1_lwrist_rot = sRot.from_quat(
            g1_poses["left_wrist"]["orientation_wxyz"], scalar_first=True
        )
        g1_rwrist_rot = sRot.from_quat(
            g1_poses["right_wrist"]["orientation_wxyz"], scalar_first=True
        )

        # Compute position offsets: calibrated = neck_corrected - offset
        self._calibration_lwrist_offset = lwrist_pos_corrected - g1_lwrist_pos
        self._calibration_rwrist_offset = rwrist_pos_corrected - g1_rwrist_pos

        # Compute orientation offsets: calibrated = rot_offset * neck_corrected
        self._calibration_lwrist_rot_offset = g1_lwrist_rot * lwrist_rot_corrected.inv()
        self._calibration_rwrist_rot_offset = g1_rwrist_rot * rwrist_rot_corrected.inv()

        self._calibration_pending = False
        self._override_robot_q = None

        # Log summary
        source = "override q" if g1_lwrist_pos.any() else "default/zero"
        print(
            f"[{self.log_prefix}] Calibration captured (FK ref: {source}):\n"
            f"  L-Wrist pos offset: [{self._calibration_lwrist_offset[0]:.4f}, "
            f"{self._calibration_lwrist_offset[1]:.4f}, {self._calibration_lwrist_offset[2]:.4f}]\n"
            f"  R-Wrist pos offset: [{self._calibration_rwrist_offset[0]:.4f}, "
            f"{self._calibration_rwrist_offset[1]:.4f}, {self._calibration_rwrist_offset[2]:.4f}]"
        )

    def _apply_calibration(self, vr_3pt_pose: np.ndarray) -> np.ndarray:
        """Apply stored calibration offsets to raw VR 3-point pose."""
        if self._calibration_neck_quat_inv is None:
            return vr_3pt_pose

        calibrated = vr_3pt_pose.copy()
        calib_inv_rot = sRot.from_quat(self._calibration_neck_quat_inv, scalar_first=True)

        # Neck orientation: calibrated = inv(initial) * current
        neck_rot = sRot.from_quat(vr_3pt_pose[2, 3:], scalar_first=True)
        calibrated[2, 3:] = (calib_inv_rot * neck_rot).as_quat(scalar_first=True)

        # Wrist positions: rotate by neck inverse, then subtract offset
        if self._calibration_lwrist_offset is not None:
            calibrated[0, :3] = (
                calib_inv_rot.apply(vr_3pt_pose[0, :3]) - self._calibration_lwrist_offset
            )
        if self._calibration_rwrist_offset is not None:
            calibrated[1, :3] = (
                calib_inv_rot.apply(vr_3pt_pose[1, :3]) - self._calibration_rwrist_offset
            )

        # Wrist orientations: rot_offset * (neck_inv * current)
        if self._calibration_lwrist_rot_offset is not None:
            lw_corrected = calib_inv_rot * sRot.from_quat(vr_3pt_pose[0, 3:], scalar_first=True)
            calibrated[0, 3:] = (self._calibration_lwrist_rot_offset * lw_corrected).as_quat(
                scalar_first=True
            )
        if self._calibration_rwrist_rot_offset is not None:
            rw_corrected = calib_inv_rot * sRot.from_quat(vr_3pt_pose[1, 3:], scalar_first=True)
            calibrated[1, 3:] = (self._calibration_rwrist_rot_offset * rw_corrected).as_quat(
                scalar_first=True
            )

        # Neck position via kinematic chain: root → torso_link (+Z) → neck (along calibrated Z)
        neck_z = sRot.from_quat(calibrated[2, 3:], scalar_first=True).apply([0, 0, 1])
        calibrated[2, :3] = (
            np.array([0, 0, self.TORSO_LINK_OFFSET_Z]) + self.NECK_LINK_LENGTH * neck_z
        ).astype(np.float32)

        return calibrated

    def _clear_calibration(self):
        """Clear all calibration state."""
        self._calibration_neck_quat_inv = None
        self._calibration_lwrist_offset = None
        self._calibration_rwrist_offset = None
        self._calibration_lwrist_rot_offset = None
        self._calibration_rwrist_rot_offset = None
        self._override_robot_q = None

    def reset(self) -> None:
        """Reset calibration. Next process_smpl_pose() call will recalibrate."""
        self._clear_calibration()
        self._calibration_pending = True
        print(f"[{self.log_prefix}] Calibration reset, will re-calibrate on next frame")

    def reset_with_measured_q(self, body_q_measured: np.ndarray) -> None:
        """Recalibrate wrist offsets using measured robot joints (29 DOFs).
        Preserves neck calibration to avoid jumps from SMPL noise.
        Next process_smpl_pose() will recompute wrist offsets against FK of these joints."""
        # Preserve neck calibration — only clear wrist offsets
        self._calibration_lwrist_offset = None
        self._calibration_rwrist_offset = None
        self._calibration_lwrist_rot_offset = None
        self._calibration_rwrist_rot_offset = None
        self._override_robot_q = body_q_measured.copy()
        self._calibration_pending = True
        print(f"[{self.log_prefix}] Wrist recalibration pending (neck preserved, measured q)")


class PoseStreamer:
    """Encapsulates the pose streaming loop state and logic."""

    def __init__(
        self,
        socket,
        reader: PicoReader,
        three_point: ThreePointPose,
        num_frames_to_send: int,
        target_fps: int,
        use_cuda: bool,
        record_dir: str,
        record_format: str,
        keyboard_toggle: KeyboardRecordToggle | None = None,
        hdf5_recorder: TeleopEpisodeRecorder | None = None,
        log_prefix: str = "PoseLoop",
    ):
        self.socket = socket
        self.reader = reader
        self.num_frames_to_send = num_frames_to_send
        self.target_fps = target_fps
        self.record_dir = record_dir
        self.log_prefix = log_prefix
        self.keyboard_toggle = keyboard_toggle
        self.hdf5_recorder = hdf5_recorder

        # Injected dependencies
        self.reader = reader
        self.three_point = three_point

        self.device = (
            torch.device("cuda") if use_cuda and torch.cuda.is_available() else torch.device("cpu")
        )

        if record_dir:
            os.makedirs(record_dir, exist_ok=True)
        self.record_idx = 0

        self.left_hand_ik_solver, self.right_hand_ik_solver = init_hand_ik_solvers()
        self.parent_indices = [
            -1,
            0,
            0,
            0,
            1,
            2,
            3,
            4,
            5,
            6,
            7,
            8,
            9,
            9,
            9,
            12,
            13,
            14,
            16,
            17,
            18,
            19,
            20,
            22,
            23,
        ][:24]

        self.step = 0
        self.last_fps_report = time.time()
        self.fps_counter = 0
        # NOTE: Sleep budget set to 95% of the ideal frame period so that the actual
        # FPS lands closer to target_fps despite per-frame processing overhead.
        self.frame_time = 0.95 / max(1, target_fps)
        self.frame_buffer = defaultdict(lambda: deque(maxlen=num_frames_to_send))

        self.prev_stamp_ns = None
        self.prev_smpl_pose_np = None
        self.prev_smpl_joints_np = None
        self.prev_body_quat_np = None
        self.next_target_ns = None
        self.frame_start = time.time()

        # Data collection button state tracking (edge-triggered)
        self.toggle_data_collection_last = False
        self.toggle_data_abort_last = False

        self.buffer_cleared = (
            True  # Start with buffer cleared - wait for full buffer before first send
        )
        self.yaw_accumulator = YawAccumulator()

    def reset_yaw(self):
        """Called when entering pose mode. Resets yaw only.
        Calibration is triggered separately by the operator (A+B+X+Y → calibrate_now)."""
        self.yaw_accumulator.reset()

    def on_mode_exit(self):
        self.frame_buffer.clear()
        self.prev_stamp_ns = None
        self.prev_smpl_pose_np = None
        self.prev_smpl_joints_np = None
        self.prev_body_quat_np = None
        self.next_target_ns = None
        self.buffer_cleared = True
        self.step = 0

    def run_once(self):
        """Execute one iteration of the pose streaming loop."""
        sample = self.reader.get_latest()

        if sample is None:
            time.sleep(0.005)
            return

        latest_data = compute_from_body_poses(
            self.parent_indices, self.device, sample["body_poses_np"]
        )
        (left_menu_button, left_trigger, right_trigger, left_grip, right_grip) = (
            get_controller_inputs()
        )
        # Get A and B button states for data collection control
        a_pressed, b_pressed, x_pressed, y_pressed = get_abxy_buttons()

        # Data collection toggle logic (edge-triggered)
        # A = start/stop recording
        # B = discard current trajectory
        toggle_data_collection_tmp = a_pressed
        toggle_data_abort_tmp = b_pressed

        # Detect rising edge
        toggle_data_collection = toggle_data_collection_tmp and not self.toggle_data_collection_last
        toggle_data_abort = toggle_data_abort_tmp and not self.toggle_data_abort_last
        self.toggle_data_collection_last = toggle_data_collection_tmp
        self.toggle_data_abort_last = toggle_data_abort_tmp
        toggle_data_collection_source = "controller A" if toggle_data_collection else None
        toggle_data_abort_source = "controller B" if toggle_data_abort else None

        if self.keyboard_toggle is not None and self.keyboard_toggle.consume_toggle():
            toggle_data_collection = True
            toggle_data_collection_source = "keyboard s"
        if self.keyboard_toggle is not None and self.keyboard_toggle.consume_abort():
            toggle_data_abort = True
            toggle_data_abort_source = "keyboard x"

        if toggle_data_collection_source is not None:
            current_length = self.hdf5_recorder.current_length() if self.hdf5_recorder else 0
            print(
                f"[{self.log_prefix}] Start/stop recording requested via "
                f"{toggle_data_collection_source} (episode length: {current_length} frames)"
            )
        if toggle_data_abort_source is not None:
            current_length = self.hdf5_recorder.current_length() if self.hdf5_recorder else 0
            print(
                f"[{self.log_prefix}] Discard episode requested via "
                f"{toggle_data_abort_source} (episode length: {current_length} frames)"
            )

        if toggle_data_collection and self.hdf5_recorder is not None:
            if self.hdf5_recorder.is_recording():
                self.hdf5_recorder.stop_recording(
                    trigger_source=toggle_data_collection_source or "pose_toggle",
                    mode_name="POSE",
                    discarded=False,
                )
            else:
                self.hdf5_recorder.start_recording(
                    trigger_source=toggle_data_collection_source or "pose_toggle",
                    mode_name="POSE",
                )
        if toggle_data_abort and self.hdf5_recorder is not None:
            self.hdf5_recorder.stop_recording(
                trigger_source=toggle_data_abort_source or "pose_abort",
                mode_name="POSE",
                discarded=True,
            )

        left_hand_joints, right_hand_joints = compute_hand_joints_from_inputs(
            self.left_hand_ik_solver,
            self.right_hand_ik_solver,
            left_trigger,
            left_grip,
            right_trigger,
            right_grip,
        )
        smpl_pose_np = (
            latest_data["smpl_pose"].detach().cpu().numpy()[:, :63].reshape(-1, 21, 3)[0]
        ).astype(np.float32)
        smpl_joints_np = (
            latest_data["smpl_joints_local"].detach().cpu().numpy()[0].astype(np.float32)
        )
        body_quat_np = (
            latest_data["global_orient_quat"].detach().cpu().numpy()[0].astype(np.float32)
        )
        curr_stamp_ns = int(sample.get("timestamp_ns", 0))
        step_ns = int(1e9 / max(1, self.target_fps))
        if self.prev_stamp_ns is None:
            self.prev_stamp_ns = curr_stamp_ns
            self.prev_smpl_pose_np = smpl_pose_np
            self.prev_smpl_joints_np = smpl_joints_np
            self.prev_body_quat_np = body_quat_np
            self.next_target_ns = curr_stamp_ns
            return
        if curr_stamp_ns <= self.prev_stamp_ns:
            return
        if self.next_target_ns is None:
            self.next_target_ns = self.prev_stamp_ns + step_ns
        if self.next_target_ns < self.prev_stamp_ns:
            self.next_target_ns = self.prev_stamp_ns
        if self.next_target_ns > curr_stamp_ns:
            return
        denom = float(curr_stamp_ns - self.prev_stamp_ns)
        alpha = float(self.next_target_ns - self.prev_stamp_ns) / denom if denom > 0.0 else 1.0
        if alpha < 0.0:
            alpha = 0.0
        elif alpha > 1.0:
            alpha = 1.0
        use_joints = (1.0 - alpha) * self.prev_smpl_joints_np + alpha * smpl_joints_np
        use_pose = _interp_pose_axis_angle(self.prev_smpl_pose_np, smpl_pose_np, alpha).astype(
            np.float32
        )
        use_body_quat = _quat_lerp_normalized(self.prev_body_quat_np, body_quat_np, alpha).astype(
            np.float32
        )
        N = len(self.frame_buffer["frame_index"])

        ##### From @Jiefeng for directly setting the joint position ######
        joint_pos = np.zeros(29)
        body_pose = use_pose.reshape(-1, 21, 3)

        SMPL_L_ELBOW_IDX = 17
        SMPL_L_WRIST_IDX = 19
        SMPL_R_ELBOW_IDX = 18
        SMPL_R_WRIST_IDX = 20

        # G1_L_ELBOW_IDX = 0
        G1_L_WRIST_ROLL_IDX = 23
        G1_L_WRIST_PITCH_IDX = 25
        G1_L_WRIST_YAW_IDX = 27

        # G1_R_ELBOW_IDX = 0
        G1_R_WRIST_ROLL_IDX = 24  # Done
        G1_R_WRIST_PITCH_IDX = 26
        G1_R_WRIST_YAW_IDX = 28
        smpl_l_elbow_aa = body_pose[:, SMPL_L_ELBOW_IDX]
        smpl_l_wrist_aa = body_pose[:, SMPL_L_WRIST_IDX]
        smpl_r_elbow_aa = body_pose[:, SMPL_R_ELBOW_IDX]
        smpl_r_wrist_aa = body_pose[:, SMPL_R_WRIST_IDX]

        g1_l_elbow_axis = np.array([0, 1, 0])
        g1_l_elbow_q_twist, g1_l_elbow_q_swing = decompose_rotation_aa(
            smpl_l_elbow_aa, g1_l_elbow_axis
        )

        g1_r_elbow_axis = np.array([0, 1, 0])
        g1_r_elbow_q_twist, g1_r_elbow_q_swing = decompose_rotation_aa(
            smpl_r_elbow_aa, g1_r_elbow_axis
        )

        # Move elbow roll/yaw into wrist while preserving wrist pitch from SMPL
        l_elbow_swing_euler = R.from_quat(g1_l_elbow_q_swing[:, [1, 2, 3, 0]]).as_euler(
            "XYZ", degrees=False
        )
        r_elbow_swing_euler = R.from_quat(g1_r_elbow_q_swing[:, [1, 2, 3, 0]]).as_euler(
            "XYZ", degrees=False
        )

        l_wrist_euler = R.from_rotvec(smpl_l_wrist_aa).as_euler("XYZ", degrees=False)
        r_wrist_euler = R.from_rotvec(smpl_r_wrist_aa).as_euler("XYZ", degrees=False)

        g1_l_wrist_roll = l_elbow_swing_euler[:, 0] + l_wrist_euler[:, 0]
        g1_l_wrist_pitch = -l_wrist_euler[:, 1]
        g1_l_wrist_yaw = l_elbow_swing_euler[:, 2] + l_wrist_euler[:, 2]

        g1_r_wrist_roll = -(r_elbow_swing_euler[:, 0] + r_wrist_euler[:, 0])
        g1_r_wrist_pitch = -r_wrist_euler[:, 1]
        g1_r_wrist_yaw = r_elbow_swing_euler[:, 2] + r_wrist_euler[:, 2]

        joint_pos[G1_L_WRIST_ROLL_IDX] = g1_l_wrist_roll[0]
        joint_pos[G1_L_WRIST_PITCH_IDX] = -g1_l_wrist_pitch[0]
        joint_pos[G1_L_WRIST_YAW_IDX] = g1_l_wrist_yaw[0]

        joint_pos[G1_R_WRIST_ROLL_IDX] = g1_r_wrist_roll[0]
        joint_pos[G1_R_WRIST_PITCH_IDX] = g1_r_wrist_pitch[0]
        joint_pos[G1_R_WRIST_YAW_IDX] = g1_r_wrist_yaw[0]

        # Process SMPL pose to get calibrated 3-point VR pose and update visualization
        # Pass SMPL local joints for optional body visualization in the VR3Pt viewer
        smpl_joints_for_vis = (
            latest_data["smpl_joints_local"].detach().cpu().numpy()[0]
            if self.three_point.enable_smpl_vis
            else None
        )
        vr_3pt_pose = self.three_point.process_smpl_pose(
            sample["body_poses_np"], smpl_joints_local=smpl_joints_for_vis
        )
        ##### From @Jiefeng for directly setting the joint position ######

        self.frame_buffer["smpl_pose"].append(use_pose)
        self.frame_buffer["smpl_joints"].append(use_joints)
        self.frame_buffer["body_quat_w"].append(use_body_quat)
        self.frame_buffer["frame_index"].append(int(self.step))
        self.frame_buffer["joint_pos"].append(joint_pos)
        pico_dt = float(sample.get("dt", 0.0))
        pico_fps = float(sample.get("fps", 0.0))
        N = len(self.frame_buffer["frame_index"])

        # Wait for buffer to be completely filled before sending first message after clearing
        buffer_is_full = len(self.frame_buffer["frame_index"]) >= self.num_frames_to_send
        if buffer_is_full and self.buffer_cleared:
            # Buffer is now full with fresh data, can start sending
            self.buffer_cleared = False

        # Get joystick axes for yaw accumulation
        _, _, rx, _ = get_controller_axes()
        self.yaw_accumulator.update(rx, self.frame_time)

        # Only send if buffer is full and we're not waiting for fresh data
        if buffer_is_full and not self.buffer_cleared:
            numpy_data = {
                "smpl_pose": np.stack((self.frame_buffer["smpl_pose"]), axis=0),
                "smpl_joints": np.stack((self.frame_buffer["smpl_joints"]), axis=0),
                "body_quat_w": np.stack((self.frame_buffer["body_quat_w"]), axis=0),
                "joint_pos": np.stack((self.frame_buffer["joint_pos"]), axis=0),
                "joint_vel": np.zeros((N, 29)),
                "vr_position": vr_3pt_pose[:, :3].flatten(),
                "vr_orientation": vr_3pt_pose[:, 3:].flatten(),
                "frame_index": np.array((self.frame_buffer["frame_index"]), dtype=np.int64),
                "left_trigger": np.array([left_trigger], dtype=np.float32),
                "right_trigger": np.array([right_trigger], dtype=np.float32),
                "left_grip": np.array([left_grip], dtype=np.float32),
                "right_grip": np.array([right_grip], dtype=np.float32),
                "pico_dt": np.array([pico_dt], dtype=np.float32),
                "pico_fps": np.array([pico_fps], dtype=np.float32),
                "timestamp_realtime": np.array(
                    [sample.get("timestamp_realtime", 0.0)], dtype=np.float64
                ),
                "timestamp_monotonic": np.array(
                    [sample.get("timestamp_monotonic", 0.0)], dtype=np.float64
                ),
                "left_hand_joints": left_hand_joints.reshape(-1).astype(np.float32),
                "right_hand_joints": right_hand_joints.reshape(-1).astype(np.float32),
                "toggle_data_collection": np.array([toggle_data_collection], dtype=bool),
                "toggle_data_abort": np.array([toggle_data_abort], dtype=bool),
                "heading_increment": np.array(
                    [self.yaw_accumulator.yaw_angle_change()], dtype=np.float32
                ),
            }

            packed_message = pack_pose_message(numpy_data, topic="pose")
            self.socket.send(packed_message)

            if self.record_dir:
                out_path = os.path.join(self.record_dir, f"pose_{self.record_idx:06d}.npz")
                np.savez_compressed(out_path, **numpy_data)
                self.record_idx += 1

        if self.hdf5_recorder is not None:
            self.hdf5_recorder.poll_and_record(
                mode_name="POSE",
                extras={
                    "vr/position": np.asarray(vr_3pt_pose[:, :3].flatten(), dtype=np.float32),
                    "vr/orientation": np.asarray(vr_3pt_pose[:, 3:].flatten(), dtype=np.float32),
                    "teleop/left_trigger": np.asarray([left_trigger], dtype=np.float32),
                    "teleop/right_trigger": np.asarray([right_trigger], dtype=np.float32),
                    "teleop/left_grip": np.asarray([left_grip], dtype=np.float32),
                    "teleop/right_grip": np.asarray([right_grip], dtype=np.float32),
                    "teleop/pico_dt": np.asarray([pico_dt], dtype=np.float32),
                    "teleop/pico_fps": np.asarray([pico_fps], dtype=np.float32),
                    "timestamps/realtime": np.asarray(
                        [sample.get("timestamp_realtime", 0.0)], dtype=np.float64
                    ),
                    "timestamps/monotonic": np.asarray(
                        [sample.get("timestamp_monotonic", 0.0)], dtype=np.float64
                    ),
                },
            )

        self.step += 1
        self.next_target_ns += step_ns
        self.prev_stamp_ns = curr_stamp_ns
        self.prev_smpl_pose_np = smpl_pose_np
        self.prev_smpl_joints_np = smpl_joints_np
        self.prev_body_quat_np = body_quat_np
        self.fps_counter += 1
        current_time = time.time()
        if current_time - self.last_fps_report >= 5.0:
            fps = self.fps_counter / (current_time - self.last_fps_report)
            print(f"[{self.log_prefix}] FPS: {fps:.2f}, Step: {self.step}")
            self.fps_counter = 0
            self.last_fps_report = current_time
        elapsed = time.time() - self.frame_start
        if elapsed < self.frame_time:
            time.sleep(self.frame_time - elapsed)
        self.frame_start = time.time()


def run_pico(
    buffer_size: int = 15,
    port: int = 5556,
    num_frames_to_send: int = 5,
    target_fps: int = 50,
    use_cuda: bool = False,
    record_dir: str = "",
    record_format: str = "npz",
    save_hdf5: bool = False,
    root_output_dir: str = "outputs",
    dataset_name: str | None = None,
    task_prompt: str | None = None,
    hdf5_subdir: str = "",
    hdf5_robot_name: str = "G1",
    hdf5_flush_every: int = 1,
    zmq_feedback_host: str = "localhost",
    zmq_feedback_port: int = 5557,
    camera_host: str = "192.168.123.164",
    camera_port: int = 5555,
    camera_head_width: int | None = None,
    camera_secondary_width: int | None = None,
    show_image_stream: bool = False,
    enable_vis_vr3pt: bool = False,
    with_g1_robot: bool = True,
    enable_waist_tracking: bool = False,
    enable_smpl_vis: bool = False,
):
    """Run Pico body tracking with real-time visualization and ZMQ streaming."""
    if xrt is None:
        raise ImportError(
            "XRoboToolkit SDK not available. Install xrobotoolkit_sdk to run Pico streaming."
        )
    _start_xrt_service_and_init()
    _wait_for_xrt_body_tracking(timeout_s=5.0, continue_without_data=True)
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind(f"tcp://*:{port}")
    time.sleep(0.1)
    print(f"ZMQ socket bound to port {port}")
    if build_command_message is not None and build_planner_message is not None:
        try:
            socket.send(build_command_message(start=False, stop=False, planner=False))
            socket.send(build_planner_message(0, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], -1.0, -1.0))
        except Exception as e:
            print(f"Warning: failed to send initial command/planner messages: {e}")
    hdf5_recorder = TeleopEpisodeRecorder(
        save_hdf5=save_hdf5,
        root_output_dir=root_output_dir,
        dataset_name=dataset_name,
        task_prompt=task_prompt,
        hdf5_subdir=hdf5_subdir,
        robot_name=hdf5_robot_name,
        fps=target_fps,
        flush_every=hdf5_flush_every,
        zmq_feedback_host=zmq_feedback_host,
        zmq_feedback_port=zmq_feedback_port,
        camera_host=camera_host,
        camera_port=camera_port,
        camera_head_width=camera_head_width,
        camera_secondary_width=camera_secondary_width,
        show_image_stream=show_image_stream,
    )
    try:
        _pose_stream_common(
            socket=socket,
            buffer_size=buffer_size,
            num_frames_to_send=num_frames_to_send,
            target_fps=target_fps,
            use_cuda=use_cuda,
            record_dir=record_dir,
            record_format=record_format,
            hdf5_recorder=hdf5_recorder,
            stop_event=None,
            log_prefix="Main",
            enable_vis_vr3pt=enable_vis_vr3pt,
            with_g1_robot=with_g1_robot,
            enable_waist_tracking=enable_waist_tracking,
            enable_smpl_vis=enable_smpl_vis,
        )
    finally:
        hdf5_recorder.close()
        socket.close()
        context.term()
        print("Threads stopped, ZMQ socket closed")


class FeedbackReader:
    """Reads feedback from robot via ZMQ and processes measured upper body position to use as frozen targets."""

    def __init__(self, zmq_feedback_host: str = "localhost", zmq_feedback_port: int = 5557):
        self.poller = ZMQPoller(host=zmq_feedback_host, port=zmq_feedback_port, topic="g1_debug")

        self.upper_body_joint_indices = self._get_upper_body_joint_indices()

        self.upper_body_position_target = None
        self.left_hand_position_target = None
        self.right_hand_position_target = None
        # Full body joint configuration (29 DOFs) as measured from robot,
        # used for FK when recalibrating VR 3PT tracking against actual robot pose
        self.full_body_q_measured: np.ndarray | None = None

    def _get_upper_body_joint_indices(self) -> list[int]:
        # TODO: get from robot model, not hardcoded
        # robot_model = instantiate_g1_robot_model()
        # return robot_model.get_joint_group_indices("upper_body")
        return [12, 13, 14, 15, 22, 16, 23, 17, 24, 18, 25, 19, 26, 20, 27, 21, 28]

    def poll_feedback(self):
        """Poll for feedback once, and update internal state."""
        (
            self.upper_body_position_target,
            self.left_hand_position_target,
            self.right_hand_position_target,
            self.full_body_q_measured,
        ) = self._process_upper_body_position_targets()
        print("[PlannerLoop] Saved upper body position target:", self.upper_body_position_target)

    def _process_upper_body_position_targets(
        self,
    ) -> tuple[np.ndarray | None, np.ndarray | None, np.ndarray | None, np.ndarray | None]:
        data = self.poller.get_data()

        if data is None:
            print("[PlannerLoop] No feedback data received")
            return None, None, None, None

        unpacked = msgpack.unpackb(data, raw=False)
        full_body_q = None
        if "body_q_measured" in unpacked:
            body_q_swizzled = unpacked["body_q_measured"]
            full_body_q = np.array(body_q_swizzled, dtype=np.float64)
            body_q = [body_q_swizzled[i] for i in self.upper_body_joint_indices]
        else:
            print("[PlannerLoop] body_q_measured not in feedback data")
            body_q = None

        if "left_hand_q_measured" in unpacked:
            left_hand_q = unpacked["left_hand_q_measured"]
        else:
            print("[PlannerLoop] left_hand_q_measured not in feedback data")
            left_hand_q = None

        if "right_hand_q_measured" in unpacked:
            right_hand_q = unpacked["right_hand_q_measured"]
        else:
            print("[PlannerLoop] right_hand_q_measured not in feedback data")
            right_hand_q = None

        return body_q, left_hand_q, right_hand_q, full_body_q


class PlannerStreamer:
    """Encapsulates the planner control loop state and logic."""

    def __init__(
        self,
        socket,
        reader: PicoReader,
        three_point: ThreePointPose,
        poll_hz: int = 20,
        zmq_feedback_host: str = "localhost",
        zmq_feedback_port: int = 5557,
        keyboard_toggle: KeyboardRecordToggle | None = None,
        hdf5_recorder: TeleopEpisodeRecorder | None = None,
    ):
        self.socket = socket
        self.reader = reader
        self.three_point = three_point
        self.keyboard_toggle = keyboard_toggle
        self.hdf5_recorder = hdf5_recorder
        self.feedback_reader = FeedbackReader(
            zmq_feedback_host=zmq_feedback_host, zmq_feedback_port=zmq_feedback_port
        )

        self.dt = 1.0 / max(1, poll_hz)
        # Current locomotion mode, default IDLE
        self.mode = LocomotionMode.IDLE
        self.prev_ab = False
        self.prev_xy = False
        self.toggle_data_collection_last = False
        self.toggle_data_abort_last = False
        # Persistent facing buffer (unit vector on XY plane)
        self.yaw_accumulator = YawAccumulator()
        self.last_send = time.time()
        self.last_xrt_timestamp = None
        self._recording_active = False
        self._episode_length_frames = 0

        # Hand IK solvers for trigger-controlled hand open/close in VR 3PT mode
        self.left_hand_ik_solver, self.right_hand_ik_solver = init_hand_ik_solvers()

    def reset_yaw(self):
        """Called when entering planner mode. Resets state for fresh start."""
        self.yaw_accumulator.reset()

    def _handle_vr3pt_episode_controls(
        self,
        *,
        a_pressed: bool,
        b_pressed: bool,
        x_pressed: bool,
        y_pressed: bool,
    ) -> None:
        """Forward VR_3PT save/discard button events to the exporter."""
        toggle_data_collection_tmp = a_pressed and not (b_pressed or x_pressed or y_pressed)
        toggle_data_abort_tmp = b_pressed and not (a_pressed or x_pressed or y_pressed)

        toggle_data_collection = (
            toggle_data_collection_tmp and not self.toggle_data_collection_last
        )
        toggle_data_abort = toggle_data_abort_tmp and not self.toggle_data_abort_last
        self.toggle_data_collection_last = toggle_data_collection_tmp
        self.toggle_data_abort_last = toggle_data_abort_tmp

        toggle_data_collection_source = "controller A" if toggle_data_collection else None
        toggle_data_abort_source = "controller B" if toggle_data_abort else None

        if self.keyboard_toggle is not None and self.keyboard_toggle.consume_toggle():
            toggle_data_collection = True
            toggle_data_collection_source = "keyboard s"
        if self.keyboard_toggle is not None and self.keyboard_toggle.consume_abort():
            toggle_data_abort = True
            toggle_data_abort_source = "keyboard x"

        if toggle_data_collection:
            is_recording = self.hdf5_recorder.is_recording() if self.hdf5_recorder is not None else False
            action = "stop" if is_recording else "start"
            print(
                f"[PlannerLoop] {action.capitalize()} recording requested via "
                f"{toggle_data_collection_source} "
                f"(episode length: {self.hdf5_recorder.current_length() if self.hdf5_recorder else self._episode_length_frames} frames)"
            )
            if self.hdf5_recorder is not None:
                if is_recording:
                    self.hdf5_recorder.stop_recording(
                        trigger_source=toggle_data_collection_source or "vr3pt_toggle",
                        mode_name="VR_3PT",
                        discarded=False,
                    )
                    self._recording_active = False
                else:
                    self.hdf5_recorder.start_recording(
                        trigger_source=toggle_data_collection_source or "vr3pt_toggle",
                        mode_name="VR_3PT",
                    )
                    self._recording_active = True
                    self._episode_length_frames = 0

        if toggle_data_abort:
            print(
                f"[PlannerLoop] Discard episode requested via {toggle_data_abort_source} "
                f"(episode length: {self.hdf5_recorder.current_length() if self.hdf5_recorder else self._episode_length_frames} frames)"
            )
            if self.hdf5_recorder is not None:
                self.hdf5_recorder.stop_recording(
                    trigger_source=toggle_data_abort_source or "vr3pt_abort",
                    mode_name="VR_3PT",
                    discarded=True,
                )
                self._recording_active = False
                self._episode_length_frames = 0

    def save_upper_body_position_target(self):
        """Poll feedback and save upper body position target."""
        self.feedback_reader.poll_feedback()

    def recalibrate_for_vr3pt(self):
        """
        Recalibrate VR 3-point pose tracking using the robot's current measured joints.

        Polls the g1_debug feedback to get the robot's actual joint state, then
        schedules recalibration so VR tracking aligns with the robot's current pose.
        This prevents sudden jumps when entering VR 3PT mode from PLANNER mode.
        """
        self.feedback_reader.poll_feedback()
        if self.feedback_reader.full_body_q_measured is not None:
            self.three_point.reset_with_measured_q(self.feedback_reader.full_body_q_measured)
            print("[PlannerLoop] VR 3PT recalibration scheduled with measured robot pose")
        else:
            # Fallback: use zeros if no feedback available
            print(
                "[PlannerLoop] WARNING: No feedback data for VR 3PT recalibration, "
                "using zero body_q as fallback"
            )
            self.three_point.reset_with_measured_q(np.zeros(29, dtype=np.float64))

    def run_once(self, stream_mode: StreamMode):
        """Execute one iteration of the planner control loop."""
        try:
            # Avoid sending old commands if XRT timestamp hasn't advanced, in case of headset disconnect
            xrt_timestamp = xrt.get_time_stamp_ns()
            if xrt_timestamp == self.last_xrt_timestamp:
                return
            self.last_xrt_timestamp = xrt_timestamp

            # A+B => next mode; X+Y => previous mode (rising edges)
            a_pressed, b_pressed, x_pressed, y_pressed = get_abxy_buttons()
            if stream_mode == StreamMode.PLANNER_VR_3PT:
                self._handle_vr3pt_episode_controls(
                    a_pressed=a_pressed,
                    b_pressed=b_pressed,
                    x_pressed=x_pressed,
                    y_pressed=y_pressed,
                )
            ab_now = bool(a_pressed) and bool(b_pressed)
            xy_now = bool(x_pressed) and bool(y_pressed)
            if ab_now and not self.prev_ab:
                self.mode = LocomotionMode(min(LocomotionMode.INJURED_WALK, self.mode + 1))
                print(f"[PlannerLoop] Mode -> {self.mode.value}: {self.mode.name}")
            if xy_now and not self.prev_xy:
                self.mode = LocomotionMode(max(LocomotionMode.IDLE, self.mode - 1))
                print(f"[PlannerLoop] Mode -> {self.mode.value}: {self.mode.name}")
            self.prev_ab = ab_now
            self.prev_xy = xy_now

            # Read axes/joysticks to control movement, facing, speed and mode
            lx, ly, rx, ry = get_controller_axes()

            # Facing from RIGHT stick: continuous yaw based on rx (right = turn right, left = turn left)
            facing = self.yaw_accumulator.update(rx, self.dt)

            raw_mag = np.hypot(lx, ly)
            raw_mag = np.clip(raw_mag, 0.0, 1.0)
            if np.abs(raw_mag) < JOYSTICK_DEADZONE:
                mag = 0.0
                speed = -1.0
                mode_to_send = LocomotionMode.IDLE
            else:
                mag = (raw_mag - JOYSTICK_DEADZONE) / (1.0 - JOYSTICK_DEADZONE)
                if mag > 1.0:
                    mag = 1.0
                mode_to_send = self.mode

                if self.mode == LocomotionMode.SLOW_WALK:
                    speed = 0.1 + 0.5 * mag  # 0.1 .. 0.6
                elif self.mode == LocomotionMode.WALK:
                    speed = -1.0
                elif self.mode == LocomotionMode.RUN:
                    speed = 1.5 + 3 * mag  # 1.5 .. 4.5
                else:
                    speed = mag  # default 0 .. 1.0

            denom = raw_mag if raw_mag > 0.0 else 1.0
            scale = mag / denom
            movement_local = np.array([-lx, ly]) * scale
            perp_x, perp_y = -facing[1], facing[0]
            rotation_facing = np.array([[perp_x, perp_y], [facing[0], facing[1]]])
            movement_global = rotation_facing @ movement_local

            movement = [movement_global[0], movement_global[1], 0.0]

            upper_body_position = None
            left_hand_position = None
            right_hand_position = None
            if stream_mode == StreamMode.PLANNER_FROZEN_UPPER_BODY:
                upper_body_position = self.feedback_reader.upper_body_position_target
                left_hand_position = self.feedback_reader.left_hand_position_target
                right_hand_position = self.feedback_reader.right_hand_position_target

            vr_3pt_position = None
            vr_3pt_orientation = None
            vr_3pt_compliance = None
            if stream_mode == StreamMode.PLANNER_VR_3PT:
                sample = self.reader.get_latest()
                if sample is not None:
                    print("[PlannerLoop] Sending VR 3-point pose as target")
                    vr_3pt_pose = self.three_point.process_smpl_pose(sample["body_poses_np"])
                    vr_3pt_position = (vr_3pt_pose[:, :3].flatten()).tolist()
                    vr_3pt_orientation = vr_3pt_pose[:, 3:].flatten().tolist()

                # Compute hand joints from trigger/grip inputs so operator can
                # control hand open/close while in VR 3PT mode
                (
                    left_menu_button,
                    left_trigger,
                    right_trigger,
                    left_grip,
                    right_grip,
                ) = get_controller_inputs()
                lh_joints, rh_joints = compute_hand_joints_from_inputs(
                    self.left_hand_ik_solver,
                    self.right_hand_ik_solver,
                    left_trigger,
                    left_grip,
                    right_trigger,
                    right_grip,
                )
                left_hand_position = lh_joints.reshape(-1).astype(np.float32).tolist()
                right_hand_position = rh_joints.reshape(-1).astype(np.float32).tolist()

            msg = build_planner_message(
                mode_to_send.value,
                movement,
                facing,
                speed=speed,
                height=-1.0,
                upper_body_position=upper_body_position,
                left_hand_position=left_hand_position,
                right_hand_position=right_hand_position,
                vr_3pt_position=vr_3pt_position,
                vr_3pt_orientation=vr_3pt_orientation,
                vr_3pt_compliance=vr_3pt_compliance,
            )
            self.socket.send(msg)
            if stream_mode == StreamMode.PLANNER_VR_3PT and self._recording_active:
                recorded = False
                if self.hdf5_recorder is not None:
                    recorded = self.hdf5_recorder.poll_and_record(
                        mode_name="VR_3PT",
                        extras={
                            "teleop/movement": np.asarray(movement, dtype=np.float32),
                            "teleop/facing": np.asarray(facing, dtype=np.float32),
                            "teleop/speed": np.asarray([speed], dtype=np.float32),
                            "teleop/locomotion_mode": np.asarray([mode_to_send.value], dtype=np.int32),
                            "vr/position": np.asarray(vr_3pt_position or [], dtype=np.float32),
                            "vr/orientation": np.asarray(vr_3pt_orientation or [], dtype=np.float32),
                        },
                    )
                if recorded:
                    self._episode_length_frames += 1
        except Exception as e:
            import traceback

            print(f"[PlannerLoop] error: {e}")
            traceback.print_exc()
            raise

        # pacing
        now = time.time()
        sleep_t = self.dt - (now - self.last_send)
        if sleep_t > 0:
            time.sleep(sleep_t)
        self.last_send = time.time()


def run_pico_manager(
    port: int = 5556,
    buffer_size: int = 15,
    num_frames_to_send: int = 5,
    target_fps: int = 50,
    use_cuda: bool = False,
    record_dir: str = "",
    record_format: str = "npz",
    save_hdf5: bool = False,
    root_output_dir: str = "outputs",
    dataset_name: str | None = None,
    task_prompt: str | None = None,
    hdf5_subdir: str = "",
    hdf5_robot_name: str = "G1",
    hdf5_flush_every: int = 1,
    zmq_feedback_host: str = "localhost",
    zmq_feedback_port: int = 5557,
    camera_host: str = "192.168.123.164",
    camera_port: int = 5555,
    camera_head_width: int | None = None,
    camera_secondary_width: int | None = None,
    show_image_stream: bool = False,
    enable_vis_vr3pt: bool = False,
    with_g1_robot: bool = True,
    enable_waist_tracking: bool = False,
    enable_smpl_vis: bool = False,
):
    """
    Manager: creates shared PUB socket and runs pose/planner streamers based on current mode.
    Controller input:
      A+X: Toggle between planner and pose mode
      A+B+X+Y: Toggle policy start/stop
    """
    if xrt is None:
        raise ImportError(
            "XRoboToolkit SDK not available. Install xrobotoolkit_sdk to run the manager."
        )
    _start_xrt_service_and_init()
    _wait_for_xrt_body_tracking(timeout_s=5.0, continue_without_data=True)

    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind(f"tcp://*:{port}")
    time.sleep(0.1)
    print(f"[Manager] ZMQ socket bound to port {port}")

    # Print available locomotion modes
    try:
        print("[Manager] Available modes:")
        for mode in LocomotionMode:
            print(f"  {mode.value}: {mode.name}")
    except Exception:
        pass

    # Create shared reader and 3-point pose processor
    reader = PicoReader(max_queue_size=buffer_size)
    reader.start()

    three_point = ThreePointPose(
        enable_vis_vr3pt=enable_vis_vr3pt,
        with_g1_robot=with_g1_robot,
        enable_waist_tracking=enable_waist_tracking,
        enable_smpl_vis=enable_smpl_vis,
        log_prefix="PoseLoop",
    )
    keyboard_toggle = KeyboardRecordToggle()
    keyboard_toggle.start()
    hdf5_recorder = TeleopEpisodeRecorder(
        save_hdf5=save_hdf5,
        root_output_dir=root_output_dir,
        dataset_name=dataset_name,
        task_prompt=task_prompt,
        hdf5_subdir=hdf5_subdir,
        robot_name=hdf5_robot_name,
        fps=target_fps,
        flush_every=hdf5_flush_every,
        zmq_feedback_host=zmq_feedback_host,
        zmq_feedback_port=zmq_feedback_port,
        camera_host=camera_host,
        camera_port=camera_port,
        camera_head_width=camera_head_width,
        camera_secondary_width=camera_secondary_width,
        show_image_stream=show_image_stream,
    )

    pose_streamer = PoseStreamer(
        socket=socket,
        reader=reader,
        three_point=three_point,
        num_frames_to_send=num_frames_to_send,
        target_fps=target_fps,
        use_cuda=use_cuda,
        record_dir=record_dir,
        record_format=record_format,
        keyboard_toggle=keyboard_toggle,
        hdf5_recorder=hdf5_recorder,
        log_prefix="PoseLoop",
    )
    planner_streamer = PlannerStreamer(
        socket=socket,
        reader=reader,
        three_point=three_point,
        poll_hz=20,
        zmq_feedback_host=zmq_feedback_host,
        zmq_feedback_port=zmq_feedback_port,
        keyboard_toggle=keyboard_toggle,
        hdf5_recorder=hdf5_recorder,
    )

    # State machine diagram:
    #
    #   Chain 1 (by_pressed enters/exits, left_axis_click toggles sub-mode):
    #     POSE <--(by)--> PLANNER_FROZEN_UPPER_BODY <--(left_axis_click)--> PLANNER_VR_3PT
    #                                                                         |
    #                                                                    (by)--> POSE
    #
    #   Chain 2 (ax_pressed enters/exits, left_axis_click toggles sub-mode):
    #     POSE <--(ax)--> PLANNER <--(left_axis_click)--> PLANNER_VR_3PT
    #                                                        |
    #                                                   (ax)--> POSE
    #
    #   Emergency stop from any mode: A+B+X+Y (start_combo) --> OFF
    #   POSE_PAUSE: left_menu_button held --> POSE_PAUSE, released --> POSE
    #
    print("Manager controls: A+X=toggle mode, A+B+X+Y=start/stop policy")
    current_mode = StreamMode.OFF
    # Track which mode VR_3PT was entered from, so left_axis_click returns to it.
    # Will be either PLANNER or PLANNER_FROZEN_UPPER_BODY.
    vr3pt_parent_mode = StreamMode.PLANNER
    try:
        prev_ax_pressed = False
        prev_by_pressed = False
        prev_start_combo = False
        prev_left_axis_click = False
        while True:
            # Poll Pico controller for buttons/axes
            a_pressed, b_pressed, x_pressed, y_pressed = get_abxy_buttons()

            left_menu_button, _, _, _, _ = get_controller_inputs()

            left_axis_click, _ = get_axis_clicks()

            # Rising edge: A+X pressed together -> toggle POSE/PLANNER mode
            ax_pressed = (a_pressed) and (x_pressed)

            # Rising edge: B+Y pressed together -> toggle POSE/PLANNER_FROZEN_UPPER_BODY mode
            by_pressed = (b_pressed) and (y_pressed)

            # Rising edge: A+B+X+Y pressed together -> toggle policy start/stop (planner=True)
            start_combo = (a_pressed) and (b_pressed) and (x_pressed) and (y_pressed)

            new_mode = current_mode
            if current_mode == StreamMode.OFF:
                if start_combo and not prev_start_combo:
                    new_mode = StreamMode.PLANNER
                    # Calibrate VR 3pt tracking NOW: operator should be in zero-ref pose.
                    # Uses the current Pico SMPL frame + FK of all-zero body joints.
                    sample = reader.get_latest()
                    if sample is not None:
                        three_point.calibrate_now(sample["body_poses_np"])
                    else:
                        print("[Manager] WARNING: No SMPL data available for calibration")

            elif current_mode == StreamMode.PLANNER:
                # Chain 2: POSE <--(ax)--> PLANNER <--(left_axis_click)--> VR_3PT
                if start_combo and not prev_start_combo:
                    new_mode = StreamMode.OFF
                elif ax_pressed and not prev_ax_pressed:
                    new_mode = StreamMode.POSE
                elif left_axis_click and not prev_left_axis_click:
                    new_mode = StreamMode.PLANNER_VR_3PT

            elif current_mode == StreamMode.POSE:
                if start_combo and not prev_start_combo:
                    new_mode = StreamMode.OFF
                elif ax_pressed and not prev_ax_pressed:
                    new_mode = StreamMode.PLANNER  # Enter chain 2
                elif by_pressed and not prev_by_pressed:
                    new_mode = StreamMode.PLANNER_FROZEN_UPPER_BODY  # Enter chain 1
                elif left_menu_button:
                    new_mode = StreamMode.POSE_PAUSE

            elif current_mode == StreamMode.PLANNER_FROZEN_UPPER_BODY:
                # Chain 1: POSE <--(by)--> FROZEN <--(left_axis_click)--> VR_3PT
                if start_combo and not prev_start_combo:
                    new_mode = StreamMode.OFF
                elif by_pressed and not prev_by_pressed:
                    new_mode = StreamMode.POSE
                elif left_axis_click and not prev_left_axis_click:
                    new_mode = StreamMode.PLANNER_VR_3PT

            elif current_mode == StreamMode.POSE_PAUSE:
                if start_combo and not prev_start_combo:
                    new_mode = StreamMode.OFF
                elif not left_menu_button:
                    new_mode = StreamMode.POSE

            elif current_mode == StreamMode.PLANNER_VR_3PT:
                # VR_3PT is reachable from both chains:
                #   left_axis_click → return to parent (PLANNER or FROZEN)
                #   ax_pressed      → POSE (chain 2 exit)
                #   by_pressed      → POSE (chain 1 exit)
                if start_combo and not prev_start_combo:
                    new_mode = StreamMode.OFF
                elif left_axis_click and not prev_left_axis_click:
                    new_mode = vr3pt_parent_mode  # Return to parent mode
                elif ax_pressed and not prev_ax_pressed:
                    new_mode = StreamMode.POSE
                elif by_pressed and not prev_by_pressed:
                    new_mode = StreamMode.POSE

            # Handle mode transitions before running loop
            if new_mode != current_mode:
                if current_mode == StreamMode.POSE:
                    pose_streamer.on_mode_exit()

                # Track parent when entering VR_3PT
                if new_mode == StreamMode.PLANNER_VR_3PT:
                    vr3pt_parent_mode = current_mode
                    print(f"[Manager] VR_3PT parent: {vr3pt_parent_mode.name}")

                if new_mode == StreamMode.POSE:
                    pose_streamer.reset_yaw()
                elif new_mode == StreamMode.PLANNER and current_mode != StreamMode.PLANNER_VR_3PT:
                    # Only reset yaw when freshly entering PLANNER from POSE,
                    # not when returning from VR_3PT sub-mode
                    planner_streamer.reset_yaw()
                elif new_mode == StreamMode.PLANNER_FROZEN_UPPER_BODY:
                    if current_mode != StreamMode.PLANNER_VR_3PT:
                        # Freshly entering from POSE: reset yaw and grab initial targets
                        planner_streamer.reset_yaw()
                    # Always re-grab the latest robot state as frozen targets,
                    # whether entering from POSE or returning from VR_3PT
                    # (the old targets are stale after VR_3PT moved the arms)
                    planner_streamer.save_upper_body_position_target()
                elif new_mode == StreamMode.PLANNER_VR_3PT:
                    # Recalibrate VR tracking against the robot's actual current pose
                    # (read via g1_debug feedback + FK) to prevent sudden jumps
                    planner_streamer.recalibrate_for_vr3pt()

            # Run one iteration of the new mode
            if new_mode == StreamMode.POSE:
                pose_streamer.run_once()
            elif (
                new_mode == StreamMode.PLANNER
                or new_mode == StreamMode.PLANNER_FROZEN_UPPER_BODY
                or new_mode == StreamMode.PLANNER_VR_3PT
            ):
                planner_streamer.run_once(new_mode)

            # Make sure to send command messages after loop iteration to ensure data arrives before mode switch
            if new_mode != current_mode:
                if new_mode == StreamMode.OFF:
                    socket.send(build_command_message(start=False, stop=True, planner=True))
                    exit()
                elif (
                    new_mode == StreamMode.PLANNER
                    or new_mode == StreamMode.PLANNER_FROZEN_UPPER_BODY
                    or new_mode == StreamMode.PLANNER_VR_3PT
                ):
                    socket.send(build_command_message(start=True, stop=False, planner=True))
                elif new_mode == StreamMode.POSE:
                    socket.send(build_command_message(start=True, stop=False, planner=False))

                print(f"[Manager] StreamMode switch: {current_mode.name} -> {new_mode.name}")
                current_mode = new_mode

            prev_ax_pressed = ax_pressed
            prev_by_pressed = by_pressed
            prev_start_combo = start_combo
            prev_left_axis_click = left_axis_click

    except KeyboardInterrupt:
        print("\nStopping manager...")
    finally:
        # Cleanup resources
        keyboard_toggle.stop()
        hdf5_recorder.close()
        reader.stop()
        three_point.close()
        socket.close()
        context.term()
        print("[Manager] Shutdown complete")


if __name__ == "__main__":

    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--buffer_size", type=int, default=15, help="Sliding window buffer size")
    parser.add_argument("--port", type=int, default=5556, help="ZMQ server port (default: 5556)")
    parser.add_argument(
        "--num_frames_to_send", type=int, default=5, help="Number of frames to send (default: 200)"
    )
    parser.add_argument("--target_fps", type=int, default=50, help="Target loop FPS (default: 50)")
    parser.add_argument(
        "--cuda", action="store_true", help="Use CUDA for tensors and model (default: CPU)"
    )
    parser.add_argument(
        "--record_dir",
        type=str,
        default="",
        help="Directory to save sent batches (default: disabled)",
    )
    parser.add_argument(
        "--record_format",
        type=str,
        default="npz",
        help="Recording format: 'npz' or 'bin' (default: npz)",
    )
    parser.add_argument(
        "--save_hdf5",
        action="store_true",
        help="Enable live HDF5 recording directly from the current teleop pipeline",
    )
    parser.add_argument(
        "--root_output_dir",
        type=str,
        default="outputs",
        help="Root output directory for integrated HDF5 datasets",
    )
    parser.add_argument(
        "--dataset_name",
        type=str,
        default=None,
        help="Dataset name for integrated HDF5 recording (default: auto-generated timestamped name)",
    )
    parser.add_argument(
        "--task_prompt",
        type=str,
        default=None,
        help="Optional language task prompt stored in HDF5 attrs['task_prompt']",
    )
    parser.add_argument(
        "--hdf5_subdir",
        type=str,
        default="",
        help="Optional subdirectory under the dataset root for HDF5 episodes",
    )
    parser.add_argument(
        "--hdf5_robot_name",
        type=str,
        default="G1",
        help="Robot name stored in HDF5 attrs['robot_name']",
    )
    parser.add_argument(
        "--hdf5_flush_every",
        type=int,
        default=1,
        help="Flush integrated HDF5 writer every N timesteps",
    )
    parser.add_argument(
        "--manager",
        action="store_true",
        help="Run manager with planner and pose threads (interactive)",
    )
    parser.add_argument(
        "--zmq_feedback_host",
        type=str,
        default="localhost",
        help="ZMQ feedback host (default: localhost)",
    )
    parser.add_argument(
        "--zmq_feedback_port",
        type=int,
        default=5557,
        help="ZMQ feedback port (default: 5557)",
    )
    parser.add_argument(
        "--camera_host",
        type=str,
        default="192.168.123.164",
        help="G1 image server host for HDF5 image recording (default: 192.168.123.164)",
    )
    parser.add_argument(
        "--camera_port",
        type=int,
        default=5555,
        help="G1 image server port for HDF5 image recording (default: 5555)",
    )
    parser.add_argument(
        "--camera_head_width",
        type=int,
        default=None,
        help="Optional pixel width of the first camera inside the composite image stream",
    )
    parser.add_argument(
        "--camera_secondary_width",
        type=int,
        default=None,
        help="Optional pixel width of the second camera inside the composite image stream",
    )
    parser.add_argument(
        "--img_stream_viewer",
        action="store_true",
        help="Open a live image-server viewer during teleoperation to verify camera streaming",
    )
    parser.add_argument(
        "--vr3pt_test",
        action="store_true",
        help="Run VR 3-point pose visualizer test (reference frames only)",
    )
    parser.add_argument(
        "--vr3pt_live",
        action="store_true",
        help="Capture one frame of VR 3-point pose and visualize with reference frames",
    )
    parser.add_argument(
        "--vr3pt_realtime",
        action="store_true",
        help="Run standalone real-time VR 3-point pose visualizer",
    )
    parser.add_argument(
        "--vis_vr3pt",
        action="store_true",
        help="Enable inline VR 3-point pose visualization in pose streaming mode",
    )
    parser.add_argument(
        "--vr3pt_hz",
        type=int,
        default=10,
        help="Update rate for real-time VR visualization in Hz (default: 10)",
    )
    parser.add_argument(
        "--no_g1",
        action="store_true",
        help="Disable G1 robot visualization in VR 3pt pose view (G1 is shown by default)",
    )
    parser.add_argument(
        "--waist_tracking",
        action="store_true",
        help="Enable G1 robot waist to follow VR head orientation (disabled by default for performance)",
    )
    parser.add_argument(
        "--vis_smpl",
        action="store_true",
        help="Enable SMPL body joint visualization (24 joint spheres) in the VR3pt viewer",
    )
    args = parser.parse_args()

    # Standalone VR3Pt test modes (exit after finishing)
    if args.vr3pt_test:
        print("Running VR 3-point pose visualizer test...")
        run_vr3pt_visualizer_test()
        print("VR 3-point pose visualizer test completed")
        exit(0)

    if args.vr3pt_live:
        print("Running VR 3-point pose live capture...")
        run_vr3pt_live_visualizer()
        print("VR 3-point pose live visualizer completed")
        exit(0)

    if args.vr3pt_realtime:
        print("Running VR 3-point pose real-time visualizer...")
        run_vr3pt_realtime_visualizer(update_hz=args.vr3pt_hz)
        print("VR 3-point pose real-time visualizer completed")
        exit(0)

    # Main execution modes
    # G1 robot visualization is enabled by default when vis_vr3pt is used
    with_g1_robot = not args.no_g1

    if args.manager:
        run_pico_manager(
            port=args.port,
            buffer_size=args.buffer_size,
            num_frames_to_send=args.num_frames_to_send,
            target_fps=args.target_fps,
            use_cuda=args.cuda,
            record_dir=args.record_dir,
            record_format=args.record_format,
            save_hdf5=args.save_hdf5,
            root_output_dir=args.root_output_dir,
            dataset_name=args.dataset_name,
            task_prompt=args.task_prompt,
            hdf5_subdir=args.hdf5_subdir,
            hdf5_robot_name=args.hdf5_robot_name,
            hdf5_flush_every=args.hdf5_flush_every,
            zmq_feedback_host=args.zmq_feedback_host,
            zmq_feedback_port=args.zmq_feedback_port,
            camera_host=args.camera_host,
            camera_port=args.camera_port,
            camera_head_width=args.camera_head_width,
            camera_secondary_width=args.camera_secondary_width,
            show_image_stream=args.img_stream_viewer,
            enable_vis_vr3pt=args.vis_vr3pt,
            with_g1_robot=with_g1_robot,
            enable_waist_tracking=args.waist_tracking,
            enable_smpl_vis=args.vis_smpl,
        )
    else:
        # Run legacy single-thread pose streaming
        run_pico(
            buffer_size=args.buffer_size,
            port=args.port,
            num_frames_to_send=args.num_frames_to_send,
            target_fps=args.target_fps,
            use_cuda=args.cuda,
            record_dir=args.record_dir,
            record_format=args.record_format,
            save_hdf5=args.save_hdf5,
            root_output_dir=args.root_output_dir,
            dataset_name=args.dataset_name,
            task_prompt=args.task_prompt,
            hdf5_subdir=args.hdf5_subdir,
            hdf5_robot_name=args.hdf5_robot_name,
            hdf5_flush_every=args.hdf5_flush_every,
            zmq_feedback_host=args.zmq_feedback_host,
            zmq_feedback_port=args.zmq_feedback_port,
            camera_host=args.camera_host,
            camera_port=args.camera_port,
            camera_head_width=args.camera_head_width,
            camera_secondary_width=args.camera_secondary_width,
            show_image_stream=args.img_stream_viewer,
            enable_vis_vr3pt=args.vis_vr3pt,
            with_g1_robot=with_g1_robot,
            enable_waist_tracking=args.waist_tracking,
            enable_smpl_vis=args.vis_smpl,
        )
