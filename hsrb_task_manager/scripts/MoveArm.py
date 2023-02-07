#!/usr/bin/env python
from copy import deepcopy
import math
import sys

import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import rospy
import shape_msgs.msg
from tf.transformations import quaternion_from_euler, quaternion_multiply
import trajectory_msgs.msg
from visualization_msgs.msg import Marker, MarkerArray
from tf import TransformListener
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import smach
import smach_ros


class MoveArm(object):
    def __init__(self, object_name="sports ball", wait=0.0):
        # init_node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_arm", anonymous=True)
        self.reference_frame = "map"

        # define the outcome of the state
        smach.State.__init__(self, outcomes=['aborted', 'succeeded'])
        
        # define the constants
        self.object_name = object_name
        self.centriods = []
        self.wait = wait
        self.goals = [0, 0, 0, 0, 0]
        
        # initialize the moveit_commander
        self.client = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)
        self.goal = MoveBaseGoal()
        self.arm = moveit_commander.MoveGroupCommander("arm",
                                                  wait_for_servers=0.0)
        self.base = moveit_commander.MoveGroupCommander("base",
                                                   wait_for_servers=0.0)
        self.gripper = moveit_commander.MoveGroupCommander("gripper",
                                                      wait_for_servers=0.0)
        self.head = moveit_commander.MoveGroupCommander("head",
                                                   wait_for_servers=0.0)
        self.whole_body_light \
            = moveit_commander.MoveGroupCommander('whole_body_light',
                                                  wait_for_servers=0.0)
        self.scene = moveit_commander.PlanningSceneInterface()
        self.whole_body_light.allow_replanning(True)
        self.whole_body_light.set_planning_time(20)
        self.whole_body_light.set_pose_reference_frame(self.reference_frame)
        self.end_effector = self.whole_body_light.get_end_effector_link()
        self.gripper.set_joint_value_target("hand_motor_joint", 1.0)
        self.gripper.go()
        rospy.sleep(1)
        
        # remove all objects
        self.scene.remove_attached_object(self.end_effector)
        self.scene.remove_world_object()
        rospy.sleep(1)
        
        rospy.loginfo("go to initial place")
        self.arm.set_joint_value_target("wrist_flex_joint", -1.9)
        self.arm.go()
        rospy.logdebug("done")
        rospy.sleep(wait)
        rospy.loginfo("finish initialization")
        
        self.add_box("table",
                     [0.45, 1.2, 0.03],
                     [0.8, 0.0, 0.4 - 0.03 / 2])
        self.whole_body_light.set_support_surface_name("table")
        
        # self.add_box("target1",
        #              [0.08, 0.08, 0.08],
        #              [0.75, 0.2, 0.43 + 0.08 / 2])
        rospy.Subscriber("/text_markers", MarkerArray, self.get_centroid_callback)
        
    def get_centroid_callback(self, msg):
        for i in range(len(msg.markers)):
            if msg.markers[i].text == self.object_name and self.goals[0] == 0:
                # print(msg.markers[i].pose.position)
                # self.centriods = msg.markers[i].pose.position
                print(self.centriods)
                                
                # move hand forward
                self.tf_listener_ = TransformListener()
                p1 = geometry_msgs.msg.PoseStamped()
                p1.header.frame_id = "/base_footprint"
                p1.header.stamp = rospy.Time(0)
                p1.pose.position.x = msg.markers[i].pose.position.x
                p1.pose.position.y = msg.markers[i].pose.position.y
                p1.pose.position.z = msg.markers[i].pose.position.z - 0.126
                p1.pose.orientation.x = 0.707
                p1.pose.orientation.z = 0.707
                self.tf_listener_.waitForTransform('/base_footprint',
                                                    '/hand_palm_link',
                                                    rospy.Time(),
                                                    rospy.Duration(4))
                p_in_hand = self.tf_listener_.transformPose("/hand_palm_link",
                                                             p1) 
                
                rospy.loginfo("Step 1: move hand forward")
                self.whole_body_light.set_joint_value_target(p_in_hand)
                self.whole_body_light.go()
                rospy.loginfo("done")
                rospy.sleep(self.wait)  
                self.goals[0] = 1
                
                rospy.loginfo("Step 2: start to grasp")
                self.gripper.set_joint_value_target("hand_motor_joint", 0.20)
                self.gripper.go()
                
                rospy.loginfo("Step 3: back to nuetral position")
                self.arm.set_named_target("neutral")
                self.arm.go()
                
                # TODO: Change this part to acml repo and localize another table
                rospy.loginfo("Step 4: robot move")
                self.client.wait_for_server()
                self.goal.target_pose.header.frame_id = "map"
                self.goal.target_pose.header.stamp = rospy.Time.now()
                self.goal.target_pose.pose.position.x = -0.12
                self.goal.target_pose.pose.position.y = -0.476
                self.goal.target_pose.pose.position.z = 0.0
                self.goal.target_pose.pose.orientation.x = 0.0
                self.goal.target_pose.pose.orientation.y = 0.0
                self.goal.target_pose.pose.orientation.z = -0.0322
                self.goal.target_pose.pose.orientation.w = 0.999
                rospy.loginfo("Send Goal")
                self.client.send_goal(self.goal)
                rospy.sleep(self.wait)
                
                # here we use FK to place the object
                rospy.loginfo("Step 5: Arm move forward and place the object")
                self.arm.set_joint_value_target("arm_lift_joint", 0.25)
                self.arm.go()
                self.arm.set_joint_value_target("arm_flex_joint", -1.2)
                self.arm.set_joint_value_target("wrist_flex_joint", -0.3)
                self.arm.go()
                rospy.sleep(self.wait)
                
                rospy.loginfo("Step 6: start to place")
                self.gripper.set_joint_value_target("hand_motor_joint", 0.5)
                self.gripper.go()
                
                rospy.loginfo("Step 7: back to nuetral position")
                self.arm.set_named_target("neutral")
                self.arm.go()
                
                
                
                
    def add_box(self, name, size, pos):
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = self.reference_frame
        p.pose.position.x = pos[0]
        p.pose.position.y = pos[1]
        p.pose.position.z = pos[2]
        p.pose.orientation.w = 1.0
        self.scene.add_box(name, p, size)    
    

# if __name__ == '__main__':
#     try:
#         MoveArm()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass
