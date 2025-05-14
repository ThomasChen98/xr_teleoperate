from setuptools import setup, find_packages

setup(
    name='open_television',
    version='0.0.1',
    description='XR vision and hand/controller interface for unitree robotics',
    author='silencht',
    packages=find_packages(),
    install_requires=[
        'numpy==1.23.0',
        'opencv_contrib_python==4.10.0.82',
        'opencv_python==4.9.0.80',
        'aiohttp==3.9.5',
        'aiohttp_cors==0.7.0',
        'aiortc==1.8.0',
        'av==11.0.0',
        'vuer==0.0.42rc16',
    ],
    python_requires='>=3.8',
)