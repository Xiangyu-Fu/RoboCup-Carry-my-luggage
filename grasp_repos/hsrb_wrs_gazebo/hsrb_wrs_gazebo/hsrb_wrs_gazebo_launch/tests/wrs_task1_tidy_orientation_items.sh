#!/bin/sh

rosservice call /gazebo/delete_model '{model_name: task1_tool_ycb_040_large_marker_1}'
rosrun gazebo_ros spawn_model -database ycb_040_large_marker -sdf -model task1_tool_ycb_040_large_marker_1 -reference_frame wrc_container_a::link -x 0 -y 0 -z 0.2 -R -1.57

rosservice call /gazebo/delete_model '{model_name: task1_kitchenitem_ycb_030_fork_1}'
rosrun gazebo_ros spawn_model -database ycb_030_fork -sdf -model task1_kitchenitem_ycb_030_fork_1 -reference_frame wrc_container_a::link -x 0 -y 0 -z 0.2 -P -1.57

rosservice call /gazebo/delete_model '{model_name: task1_tool_ycb_031_spoon_1}'
rosrun gazebo_ros spawn_model -database ycb_031_spoon -sdf -model task1_kitchenitem_ycb_031_spoon_1 -reference_frame wrc_container_a::link -x 0 -y 0 -z 0.2 -R -1.57
