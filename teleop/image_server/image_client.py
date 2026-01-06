import cv2
import zmq
import numpy as np
import time
import struct
from collections import deque
from multiprocessing import shared_memory
import logging_mp
logger_mp = logging_mp.get_logger(__name__)

class ImageClient:
    def __init__(self, tv_img_shape = None, tv_img_shm_name = None, wrist_img_shape = None, wrist_img_shm_name = None, 
                       image_show = False, server_address = "192.168.123.164", port = 5555, Unit_Test = False):
        """
        tv_img_shape: User's expected head camera resolution shape (H, W, C). It should match the output of the image service terminal.

        tv_img_shm_name: Shared memory is used to easily transfer images across processes to the Vuer.

        wrist_img_shape: User's expected wrist camera resolution shape (H, W, C). It should maintain the same shape as tv_img_shape.

        wrist_img_shm_name: Shared memory is used to easily transfer images.
        
        image_show: Whether to display received images in real time.

        server_address: The ip address to execute the image server script.

        port: The port number to bind to. It should be the same as the image server.

        Unit_Test: When both server and client are True, it can be used to test the image transfer latency, \
                   network jitter, frame loss rate and other information.
        """
        self.running = True
        self._image_show = image_show
        self._server_address = server_address
        self._port = port

        self.tv_img_shape = tv_img_shape
        self.wrist_img_shape = wrist_img_shape

        self.tv_enable_shm = False
        if self.tv_img_shape is not None and tv_img_shm_name is not None:
            self.tv_image_shm = shared_memory.SharedMemory(name=tv_img_shm_name)
            self.tv_img_array = np.ndarray(tv_img_shape, dtype = np.uint8, buffer = self.tv_image_shm.buf)
            self.tv_enable_shm = True
        
        self.wrist_enable_shm = False
        if self.wrist_img_shape is not None and wrist_img_shm_name is not None:
            self.wrist_image_shm = shared_memory.SharedMemory(name=wrist_img_shm_name)
            self.wrist_img_array = np.ndarray(wrist_img_shape, dtype = np.uint8, buffer = self.wrist_image_shm.buf)
            self.wrist_enable_shm = True

        # Performance evaluation parameters
        self._enable_performance_eval = Unit_Test
        if self._enable_performance_eval:
            self._init_performance_metrics()
        
        # Queue monitoring for latency debugging
        self._queue_monitor_enabled = True
        self._queue_check_interval = 300  # Check every N frames
        self._queue_frame_counter = 0
        self._queue_depth_history = deque(maxlen=10)  # Track recent queue depths

    def _init_performance_metrics(self):
        self._frame_count = 0  # Total frames received
        self._last_frame_id = -1  # Last received frame ID

        # Real-time FPS calculation using a time window
        self._time_window = 1.0  # Time window size (in seconds)
        self._frame_times = deque()  # Timestamps of frames received within the time window

        # Data transmission quality metrics
        self._latencies = deque()  # Latencies of frames within the time window
        self._lost_frames = 0  # Total lost frames
        self._total_frames = 0  # Expected total frames based on frame IDs

    def _update_performance_metrics(self, timestamp, frame_id, receive_time):
        # Update latency
        latency = receive_time - timestamp
        self._latencies.append(latency)

        # Remove latencies outside the time window
        while self._latencies and self._frame_times and self._latencies[0] < receive_time - self._time_window:
            self._latencies.popleft()

        # Update frame times
        self._frame_times.append(receive_time)
        # Remove timestamps outside the time window
        while self._frame_times and self._frame_times[0] < receive_time - self._time_window:
            self._frame_times.popleft()

        # Update frame counts for lost frame calculation
        expected_frame_id = self._last_frame_id + 1 if self._last_frame_id != -1 else frame_id
        if frame_id != expected_frame_id:
            lost = frame_id - expected_frame_id
            if lost < 0:
                logger_mp.info(f"[Image Client] Received out-of-order frame ID: {frame_id}")
            else:
                self._lost_frames += lost
                logger_mp.warning(f"[Image Client] Detected lost frames: {lost}, Expected frame ID: {expected_frame_id}, Received frame ID: {frame_id}")
        self._last_frame_id = frame_id
        self._total_frames = frame_id + 1

        self._frame_count += 1

    def _print_performance_metrics(self, receive_time):
        if self._frame_count % 30 == 0:
            # Calculate real-time FPS
            real_time_fps = len(self._frame_times) / self._time_window if self._time_window > 0 else 0

            # Calculate latency metrics
            if self._latencies:
                avg_latency = sum(self._latencies) / len(self._latencies)
                max_latency = max(self._latencies)
                min_latency = min(self._latencies)
                jitter = max_latency - min_latency
            else:
                avg_latency = max_latency = min_latency = jitter = 0

            # Calculate lost frame rate
            lost_frame_rate = (self._lost_frames / self._total_frames) * 100 if self._total_frames > 0 else 0

            logger_mp.info(f"[Image Client] Real-time FPS: {real_time_fps:.2f}, Avg Latency: {avg_latency*1000:.2f} ms, Max Latency: {max_latency*1000:.2f} ms, \
                  Min Latency: {min_latency*1000:.2f} ms, Jitter: {jitter*1000:.2f} ms, Lost Frame Rate: {lost_frame_rate:.2f}%")
    
    def _process_message(self, message, receive_time):
        """Process a single received message (extracted for reuse)"""
        if self._enable_performance_eval:
            header_size = struct.calcsize('dI')
            try:
                header = message[:header_size]
                jpg_bytes = message[header_size:]
                timestamp, frame_id = struct.unpack('dI', header)
            except struct.error as e:
                logger_mp.warning(f"[Image Client] Error unpacking header: {e}, discarding message.")
                return
        else:
            jpg_bytes = message
            timestamp, frame_id = None, None
        
        # Decode image
        np_img = np.frombuffer(jpg_bytes, dtype=np.uint8)
        current_image = cv2.imdecode(np_img, cv2.IMREAD_COLOR)
        if current_image is None:
            logger_mp.warning("[Image Client] Failed to decode image.")
            return

        if self.tv_enable_shm:
            try:
                src_data = np.array(current_image[:, :self.tv_img_shape[1]])
                np.copyto(self.tv_img_array, src_data)
                # Debug: log every 30th frame to confirm updates are happening
                if hasattr(self, '_shm_debug_count'):
                    self._shm_debug_count += 1
                else:
                    self._shm_debug_count = 0
                if self._shm_debug_count % 30 == 0:
                    logger_mp.info(f"[SHM DEBUG] Frame {self._shm_debug_count}: copied {src_data.shape}, mean={self.tv_img_array.mean():.1f}")
            except Exception as e:
                logger_mp.error(f"[SHM ERROR] copyto failed: {e}, src={current_image.shape}, dst={self.tv_img_array.shape}")
        
        if self.wrist_enable_shm:
            np.copyto(self.wrist_img_array, np.array(current_image[:, -self.wrist_img_shape[1]:]))
        
        if self._image_show:
            height, width = current_image.shape[:2]
            resized_image = cv2.resize(current_image, (width // 2, height // 2))
            cv2.imshow('Image Client Stream', resized_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False

        if self._enable_performance_eval and timestamp is not None:
            self._update_performance_metrics(timestamp, frame_id, receive_time)
            self._print_performance_metrics(receive_time)

    def _check_zmq_queue_depth(self):
        """
        Check ZMQ queue depth by counting pending messages using non-blocking recv.
        This helps diagnose latency buildup issues.
        """
        queue_depth = 0
        pending_messages = []
        
        # Use non-blocking recv to drain and count all pending messages
        while True:
            try:
                msg = self._socket.recv(zmq.NOBLOCK)
                queue_depth += 1
                pending_messages.append(msg)
            except zmq.Again:
                # No more messages in queue
                break
        
        self._queue_depth_history.append(queue_depth)
        avg_depth = sum(self._queue_depth_history) / len(self._queue_depth_history)
        
        if queue_depth > 0:
            logger_mp.warning(f"[ZMQ Queue Monitor] Queue depth: {queue_depth} messages pending, "
                            f"avg over last {len(self._queue_depth_history)} checks: {avg_depth:.1f}")
            if queue_depth > 10:
                logger_mp.error(f"[ZMQ Queue Monitor] ⚠️  HIGH QUEUE DEPTH DETECTED! "
                              f"This will cause increasing latency over time. "
                              f"Consider enabling ZMQ_CONFLATE to fix.")
        else:
            logger_mp.debug(f"[ZMQ Queue Monitor] Queue depth: 0 (healthy)")
        
        # Return the pending messages so they can still be processed
        return pending_messages
    
    def _close(self):
        self._socket.close()
        self._context.term()
        if self._image_show:
            cv2.destroyAllWindows()
        logger_mp.info("Image client has been closed.")

    
    def receive_process(self):
        # Set up ZeroMQ context and socket
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.SUB)
        self._socket.connect(f"tcp://{self._server_address}:{self._port}")
        self._socket.setsockopt_string(zmq.SUBSCRIBE, "")
        # Set receive timeout so we can check self.running periodically
        self._socket.setsockopt(zmq.RCVTIMEO, 500)  # 500ms timeout

        logger_mp.info("Image client has started, waiting to receive data...")
        _loop_count = 0
        _last_log_time = time.time()
        try:
            while self.running:
                _loop_count += 1
                # Log every 5 seconds to confirm thread is alive
                if time.time() - _last_log_time > 5.0:
                    logger_mp.info(f"[IMG THREAD] Still running: loop={_loop_count}, running={self.running}")
                    _last_log_time = time.time()
                
                # Periodically check ZMQ queue depth for latency debugging
                self._queue_frame_counter += 1
                if self._queue_monitor_enabled and self._queue_frame_counter % self._queue_check_interval == 0:
                    pending_msgs = self._check_zmq_queue_depth()
                    # Process any pending messages that were drained during the check
                    for msg in pending_msgs:
                        self._process_message(msg, time.time())
                
                # Receive message (with timeout so we can check self.running)
                try:
                    message = self._socket.recv()
                except zmq.Again:
                    # Timeout - loop back and check self.running
                    continue
                receive_time = time.time()
                
                # Process the received message
                self._process_message(message, receive_time)

        except KeyboardInterrupt:
            logger_mp.info("Image client interrupted by user.")
        except Exception as e:
            if self.running:  # Only log if not intentionally stopped
                logger_mp.warning(f"[Image Client] An error occurred while receiving data: {e}")
        finally:
            logger_mp.info(f"[IMG THREAD] Exited: running={self.running}, loop_count={_loop_count}")
            self._close()

if __name__ == "__main__":
    # example1
    # tv_img_shape = (480, 1280, 3)
    # img_shm = shared_memory.SharedMemory(create=True, size=np.prod(tv_img_shape) * np.uint8().itemsize)
    # img_array = np.ndarray(tv_img_shape, dtype=np.uint8, buffer=img_shm.buf)
    # img_client = ImageClient(tv_img_shape = tv_img_shape, tv_img_shm_name = img_shm.name)
    # img_client.receive_process()

    # example2
    # Initialize the client with performance evaluation enabled
    # client = ImageClient(image_show = True, server_address='127.0.0.1', Unit_Test=True) # local test
    client = ImageClient(image_show = True, server_address='192.168.123.164', Unit_Test=False) # deployment test
    client.receive_process()