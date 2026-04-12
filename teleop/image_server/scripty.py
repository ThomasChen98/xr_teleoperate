import numpy as np
from multiprocessing import shared_memory

# Same config as teleop
tv_img_shape = (480, 640, 3)

# Create shared memory
tv_img_shm = shared_memory.SharedMemory(create=True, size=np.prod(tv_img_shape) * np.uint8().itemsize)
tv_img_array = np.ndarray(tv_img_shape, dtype=np.uint8, buffer=tv_img_shm.buf)

# Import after creating shm
from image_server.image_client import ImageClient

# Create client with shared memory + image_show for debugging
client = ImageClient(
    tv_img_shape=tv_img_shape, 
    tv_img_shm_name=tv_img_shm.name,
    image_show=True,  # Also show window to verify
    server_address='192.168.123.164'
)

try:
    client.receive_process()
except KeyboardInterrupt:
    pass
finally:
    tv_img_shm.close()
    tv_img_shm.unlink()
