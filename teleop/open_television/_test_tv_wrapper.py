import os, sys
this_file = os.path.abspath(__file__)
project_root = os.path.abspath(os.path.join(os.path.dirname(this_file), '..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

import numpy as np
import time
from multiprocessing import shared_memory
from open_television import TeleVisionWrapper

def run_test_tv_wrapper():
    image_shape = (480, 640 * 2, 3)
    image_shm = shared_memory.SharedMemory(create=True, size=np.prod(image_shape) * np.uint8().itemsize)
    image_array = np.ndarray(image_shape, dtype=np.uint8, buffer=image_shm.buf)

    # from image_server.image_client import ImageClient
    # import threading
    # image_client = ImageClient(tv_img_shape = image_shape, tv_img_shm_name = image_shm.name, image_show=True, server_address="127.0.0.1")
    # image_receive_thread = threading.Thread(target = image_client.receive_process, daemon = True)
    # image_receive_thread.daemon = True
    # image_receive_thread.start()
    
    use_hand_track=True
    tv_wrapper = TeleVisionWrapper(binocular=True, use_hand_tracking=use_hand_track, img_shape=image_shape, img_shm_name=image_shm.name, 
                                   return_state_data=True, return_hand_rot_data = True)
    try:
        input("Press Enter to start tv_wrapper test...")
        running = True
        while running:
            start_time = time.time()
            teleData = tv_wrapper.get_motion_state_data()

            print("=== TeleData Snapshot ===")
            print("[Head Rotation Matrix]:\n", teleData.head_rotation)
            print("[Left EE Pose]:\n", teleData.left_arm_pose)
            print("[Right EE Pose]:\n", teleData.right_arm_pose)

            if use_hand_track:
                print("[Left Hand Position] shape {}:\n{}".format(teleData.left_hand_pos.shape, teleData.left_hand_pos))
                print("[Right Hand Position] shape {}:\n{}".format(teleData.right_hand_pos.shape, teleData.right_hand_pos))
                if teleData.left_hand_rot is not None:
                    print("[Left Hand Rotation] shape {}:\n{}".format(teleData.left_hand_rot.shape, teleData.left_hand_rot))
                if teleData.right_hand_rot is not None:
                    print("[Right Hand Rotation] shape {}:\n{}".format(teleData.right_hand_rot.shape, teleData.right_hand_rot))
                if teleData.left_pinch_value is not None:
                    print("[Left Pinch Value]: {:.2f}".format(teleData.left_pinch_value))
                if teleData.right_pinch_value is not None:
                    print("[Right Pinch Value]: {:.2f}".format(teleData.right_pinch_value))
                if teleData.tele_state:
                    state = teleData.tele_state
                    print("[Hand State]:")
                    print(f"  Left Pinch state: {state.left_pinch_state}")
                    print(f"  Left Squeeze: {state.left_squeeze_state} ({state.left_squeeze_value:.2f})")
                    print(f"  Right Pinch state: {state.right_pinch_state}")
                    print(f"  Right Squeeze: {state.right_squeeze_state} ({state.right_squeeze_value:.2f})")
            else:
                print(f"[Left Trigger Value]: {teleData.left_trigger_value:.2f}")
                print(f"[Right Trigger Value]: {teleData.right_trigger_value:.2f}")
                if teleData.tele_state:
                    state = teleData.tele_state
                    print("[Controller State]:")
                    print(f"  Left Trigger: {state.left_trigger_state}")
                    print(f"  Left Squeeze: {state.left_squeeze_ctrl_state} ({state.left_squeeze_ctrl_value:.2f})")
                    print(f"  Left Thumbstick: {state.left_thumbstick_state} ({state.left_thumbstick_value})")
                    print(f"  Left A/B Buttons: A={state.left_aButton}, B={state.left_bButton}")
                    print(f"  Right Trigger: {state.right_trigger_state}")
                    print(f"  Right Squeeze: {state.right_squeeze_ctrl_state} ({state.right_squeeze_ctrl_value:.2f})")
                    print(f"  Right Thumbstick: {state.right_thumbstick_state} ({state.right_thumbstick_value})")
                    print(f"  Right A/B Buttons: A={state.right_aButton}, B={state.right_bButton}")

            current_time = time.time()
            time_elapsed = current_time - start_time
            sleep_time = max(0, 0.033 - time_elapsed)
            time.sleep(sleep_time)
            # print(f"main process sleep: {sleep_time}")

    except KeyboardInterrupt:
        running = False
        print("KeyboardInterrupt, exiting program...")
    finally:
        image_shm.unlink()
        image_shm.close()
        print("Finally, exiting program...")
        exit(0)

if __name__ == '__main__':
    run_test_tv_wrapper()