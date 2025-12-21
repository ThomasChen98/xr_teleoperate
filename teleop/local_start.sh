#!/usr/bin/env bash
cd "$HOME/xr_teleoperate/teleop"

# conda should already be initialized in .bashrc via `conda init`
conda activate tv
python3 teleop_hand_and_arm.py --xr-mode=hand --arm=H1_2 --record --task-name=boxAction --use-multi-camera
