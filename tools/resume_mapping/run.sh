#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source /home/ssm/slam_ws/install/setup.bash
cd /home/ssm/slam_ws
exec /usr/bin/python3 /home/ssm/slam_ws/Studying_SLAM/tools/resume_mapping/resume_mapper.py "$@"
