#!/usr/bin/env bash

expect << 'EOF'
set timeout -1

# 1) SSH into robot, auto-accept host key
spawn ssh -o StrictHostKeyChecking=no unitree@192.168.123.163

# 2) Password prompt
expect {
    -re "[Pp]assword:" {
        send "Unitree0408\r"
    }
}

# 3) ROS prompt: ros:foxy(1) noetic(2) ?
expect {
    "ros:foxy(1) noetic(2) ?" {
        send "1\r"
        exp_continue
    }
    -re {\$ $} {
        # reached shell prompt
    }
}

# 4) Run image server
send "cd image_server && python3 image_server.py\r"

# 5) Keep interactive control
interact
EOF
