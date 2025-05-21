import os, sys
this_file = os.path.abspath(__file__)
project_root = os.path.abspath(os.path.join(os.path.dirname(this_file), '..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

import time
import numpy as np
from multiprocessing import shared_memory
from open_television import TeleVision

def run_test_television():
    # image
    image_shape = (480, 640 * 2, 3)
    image_shm = shared_memory.SharedMemory(create=True, size=np.prod(image_shape) * np.uint8().itemsize)
    image_array = np.ndarray(image_shape, dtype=np.uint8, buffer=image_shm.buf)

    # from image_server.image_client import ImageClient
    # import threading
    # image_client = ImageClient(tv_img_shape = image_shape, tv_img_shm_name = image_shm.name, image_show=True, server_address="127.0.0.1")
    # image_receive_thread = threading.Thread(target = image_client.receive_process, daemon = True)
    # image_receive_thread.daemon = True
    # image_receive_thread.start()

    # television
    use_hand_track = True
    tv = TeleVision(binocular = True, use_hand_tracking = use_hand_track, img_shape = image_shape, img_shm_name = image_shm.name, webrtc=False)

    try:
        input("Press Enter to start television test...")
        running = True
        while running:
            print("=" * 80)
            print("Common Data (always available):")
            print(f"head_pose shape: {tv.head_pose.shape}\n{tv.head_pose}\n")
            print(f"left_arm_pose shape: {tv.left_arm_pose.shape}\n{tv.left_arm_pose}\n")
            print(f"right_arm_pose shape: {tv.right_arm_pose.shape}\n{tv.right_arm_pose}\n")
            print("=" * 80)

            if use_hand_track:
                print("Hand Tracking Data:")
                print(f"left_hand_positions shape: {tv.left_hand_positions.shape}\n{tv.left_hand_positions}\n")
                print(f"right_hand_positions shape: {tv.right_hand_positions.shape}\n{tv.right_hand_positions}\n")
                print(f"left_hand_orientations shape: {tv.left_hand_orientations.shape}\n{tv.left_hand_orientations}\n")
                print(f"right_hand_orientations shape: {tv.right_hand_orientations.shape}\n{tv.right_hand_orientations}\n")
                print(f"left_hand_pinch_state: {tv.left_hand_pinch_state}")
                print(f"left_hand_pinch_value: {tv.left_hand_pinch_value}")
                print(f"left_hand_squeeze_state: {tv.left_hand_squeeze_state}")
                print(f"left_hand_squeeze_value: {tv.left_hand_squeeze_value}")
                print(f"right_hand_pinch_state: {tv.right_hand_pinch_state}")
                print(f"right_hand_pinch_value: {tv.right_hand_pinch_value}")
                print(f"right_hand_squeeze_state: {tv.right_hand_squeeze_state}")
                print(f"right_hand_squeeze_value: {tv.right_hand_squeeze_value}")
            else:
                print("Controller Data:")
                print(f"left_controller_trigger_state: {tv.left_controller_trigger_state}")
                print(f"left_controller_trigger_value: {tv.left_controller_trigger_value}")
                print(f"left_controller_squeeze_state: {tv.left_controller_squeeze_state}")
                print(f"left_controller_squeeze_value: {tv.left_controller_squeeze_value}")
                print(f"left_controller_thumbstick_state: {tv.left_controller_thumbstick_state}")
                print(f"left_controller_thumbstick_value: {tv.left_controller_thumbstick_value}")
                print(f"left_controller_aButton: {tv.left_controller_aButton}")
                print(f"left_controller_bButton: {tv.left_controller_bButton}")
                print(f"right_controller_trigger_state: {tv.right_controller_trigger_state}")
                print(f"right_controller_trigger_value: {tv.right_controller_trigger_value}")
                print(f"right_controller_squeeze_state: {tv.right_controller_squeeze_state}")
                print(f"right_controller_squeeze_value: {tv.right_controller_squeeze_value}")
                print(f"right_controller_thumbstick_state: {tv.right_controller_thumbstick_state}")
                print(f"right_controller_thumbstick_value: {tv.right_controller_thumbstick_value}")
                print(f"right_controller_aButton: {tv.right_controller_aButton}")
                print(f"right_controller_bButton: {tv.right_controller_bButton}")
            print("=" * 80)
            time.sleep(0.03)
    except KeyboardInterrupt:
        running = False
        print("KeyboardInterrupt, exiting program...")
    finally:
        image_shm.unlink()
        image_shm.close()
        print("Finally, exiting program...")
        exit(0)

if __name__ == '__main__':
    run_test_television()