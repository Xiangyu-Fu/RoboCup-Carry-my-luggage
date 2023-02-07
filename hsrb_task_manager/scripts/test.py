#!/usr/bin/env python
# Copyright (C) 2017 Toyota Motor Corporation

import sys

import geometry_msgs.msg
import moveit_commander
import rospy
from tf import TransformListener

print(rospy.get_param_names())