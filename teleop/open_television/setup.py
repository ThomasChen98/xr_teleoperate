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
        # 'vuer==0.0.32rc7',    # avp \ pico, hand tracking all work fine. but it not support controller tracking.

        # 'vuer==0.0.35',       # avp hand tracking works fine. pico is fine too, but the right eye occasionally goes black for a short time at start.

        # from 'vuer==0.0.36rc1', # avp hand tracking only shows a flat RGB image — there's no stereo view. Pico (hand / controller tracking) is the same, 
        #                           and sometimes the right eye occasionally goes black for a short time at start. 
        #                           Both avp / pico can display the hand or controller marker, which looks like a black box.

        # to 'vuer==0.0.42rc16',  # avp hand tracking only shows a flat RGB image — there's no stereo view. Pico (hand / controller tracking) is the same, 
        #                           and sometimes the right eye occasionally goes black for a short time at start. 
        #                           Both avp / pico can display the hand or controller marker, which looks like RGB three-axis color coordinates.

        # from 'vuer==0.0.42',  # avp hand tracking only shows a flat RGB image — there's no stereo view.
        # to 'vuer==0.0.45',      pico hand tracking also only shows a flat RGB image — there's no stereo view. Unable to retrieve hand tracking data.
        #                         pico controller tracking also only shows a flat RGB image — there's no stereo view. Controller data can be retrieved.

        # from 'vuer==0.0.46'  # avp hand tracking work fine.  
        # to
        # 'vuer==0.0.56',        # In pico hand tracking mode, using a hand gesture to click the "Virtual Reality" button 
        #                        causes the screen to stay black and stuck loading. But if the button is clicked with the controller instead, everything works fine.
        #                        In pico controller tracking mode, using controller to click the "Virtual Reality" button 
        #                        causes the screen to stay black and stuck loading. But if the button is clicked with a hand gesture instead, everything works fine.
        #                        avp \ pico all have hand marker visualization (RGB three-axis color coordinates).
        
        'vuer==0.0.60',        # a good version
                               # https://github.com/unitreerobotics/avp_teleoperate/issues/53
                               # https://github.com/vuer-ai/vuer/issues/45
                               # https://github.com/vuer-ai/vuer/issues/65

    ],
    python_requires='>=3.8',
)