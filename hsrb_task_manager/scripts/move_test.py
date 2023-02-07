#!/usr/bin/env python

import rospy
import smach
import smach_ros
import random
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# define states
class FindTargat(smach.State):
    def __init__(self):
        # define the outcome of the state
        smach.State.__init__(self, outcomes=['aborted', 'succeeded'])
    # replace this part by your method
    def execute(self, userdata):
        rospy.loginfo('Executing state FindTargat')
        if random.random() < 0.9:
            rospy.loginfo('Target not found')
            return 'aborted'
        else:
            rospy.loginfo('Target found')
            return 'succeeded'
        