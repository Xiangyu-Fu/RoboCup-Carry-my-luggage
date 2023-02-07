#!/bin/bash
echo "Now running the grasp detection script"
roslaunch gpd_ros gpd.launch

# roslaunch plane_segmentation plane_segmentation_hsrb.launch