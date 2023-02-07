#!/bin/bash
echo "Now running the start_camera.sh script"
source ~/venv/bin/activate
roslaunch hsrb_human_following start_camera.launch webcam:=true publish_dummy_frames:=true camera_xyz:="0 0 1.4" camera_rpy:="0 0 0" sim:=true