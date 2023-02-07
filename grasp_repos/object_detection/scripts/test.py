#!/usr/bin/env python
# Copyright (C) 2017 Toyota Motor Corporation

import sys

import geometry_msgs.msg
import moveit_commander
import rospy
from tf import TransformListener

class MoveItIKDemo(object):
    def __init__(self, wait=0.0):
        # initialize
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_demo', anonymous=True)

        arm = moveit_commander.MoveGroupCommander('arm',
                                                  wait_for_servers=0.0)
        head = moveit_commander.MoveGroupCommander('head',
                                                   wait_for_servers=0.0)
        # whole_body = moveit_commander.MoveGroupCommander('whole_body',
        #                                                  wait_for_servers=0.0)
        # whole_body_weighted \
        #     = moveit_commander.MoveGroupCommander('whole_body_weighted',
        #                                           wait_for_servers=0.0)
        whole_body_light \
            = moveit_commander.MoveGroupCommander('whole_body_light',
                                                  wait_for_servers=0.0)

        gripper = moveit_commander.MoveGroupCommander("gripper",
                                            wait_for_servers=0.0)

        # move_to_neutral
        rospy.loginfo("step1: move_to_neutral")
        arm.set_named_target("neutral")
        arm.go()
        rospy.logdebug("done")
        rospy.sleep(wait)
        
        gripper.set_joint_value_target("hand_motor_joint", 1.0)
        gripper.go()
        
        self.tf_listener_ = TransformListener()
        p1 = geometry_msgs.msg.PoseStamped()
        p1.header.frame_id = "/base_footprint"
        p1.header.stamp = rospy.Time(0)
        p1.pose.position.x = 0.7
        p1.pose.position.y = 0.164
        p1.pose.position.z = 0.5
        p1.pose.orientation.x = 0.707
        p1.pose.orientation.z = 0.707
        self.tf_listener_.waitForTransform('/base_footprint',
                                             '/hand_palm_link',
                                             rospy.Time(),
                                             rospy.Duration(4))
        p_in_hand = self.tf_listener_.transformPose("/hand_palm_link", p1)

        # move hand forward
        rospy.loginfo("step2: move hand forward")
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = "hand_palm_link"
        p.pose.position.x = 0.1
        whole_body_light.set_joint_value_target(p_in_hand)
        whole_body_light.go()
        rospy.logdebug("done")
        rospy.sleep(wait)

        # finalize
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == "__main__":
    MoveItIKDemo(float(sys.argv[1]) if len(sys.argv) > 1 else 0.0)
