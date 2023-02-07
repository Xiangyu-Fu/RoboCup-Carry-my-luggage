#!/usr/bin/env python
import rospy
import smach
import smach_ros
import random
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import roslaunch
import os
import rosnode
from move_test import *
import geometry_msgs.msg
import moveit_commander
from visualization_msgs.msg import Marker, MarkerArray
from tf import TransformListener
import sys
import rosparam
import actionlib

# define states
class FindTargat(smach.State):
    def __init__(self, object_name="cup", wait=3.0):
        # define the outcome of the state
        smach.State.__init__(self, outcomes=['aborted', 'succeeded'])

        # init_node
        moveit_commander.roscpp_initialize(sys.argv)

        # define the constants
        self.object_name = object_name
        self.centriods = []
        self.wait = wait
        self.rc_goals = False
        self.find_target_state = False

    def get_centroid_callback(self, msg):
        for i in range(len(msg.markers)):
            if msg.markers[i].text == self.object_name and not self.find_target_state:
              rospy.loginfo("Find target!")
              # self.centriods = msg.markers[i].pose.position
              self.find_target_state = True
              rospy.set_param('centriod', [msg.markers[i].pose.position.x,
                                           msg.markers[i].pose.position.y,
                                           msg.markers[i].pose.position.z,])
              print(rospy.get_param('centriod'))


    # replace this part by your method
    def execute(self, userdata):
        sub_once = None
        rospy.loginfo('Executing state FindTargat')
        get_c_sub = rospy.Subscriber("/text_markers", MarkerArray, self.get_centroid_callback)
        rospy.sleep(10)
        get_c_sub.unregister()
        if not self.find_target_state:
            rospy.loginfo('Target not found')
            return 'aborted'
        else:
            rospy.loginfo('Target found')
            self.find_target_state = False
            return 'succeeded'


class Grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

        self.reference_frame = "map"

        # initialize the moveit_commander
        self.goal = MoveBaseGoal()

        self.end_effector = whole_body_light.get_end_effector_link()
        gripper.set_joint_value_target("hand_motor_joint", 1.0)
        gripper.go()

        # self.add_box("table",
        #       [0.45, 1.2, 0.03],
        #       [0.8, 0.0, 0.4 - 0.03 / 2])
        # self.whole_body_light.set_support_surface_name("table")

    def add_box(self, name, size, pos):
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = self.reference_frame
        p.pose.position.x = pos[0]
        p.pose.position.y = pos[1]
        p.pose.position.z = pos[2]
        p.pose.orientation.w = 1.0
        scene.add_box(name, p, size)    
    

    def execute(self, userdata):
        centriod_pt = rospy.get_param('centriod')

        # move hand forward
        self.tf_listener_ = TransformListener()
        p1 = geometry_msgs.msg.PoseStamped()
        p1.header.frame_id = "/base_footprint"
        p1.header.stamp = rospy.Time(0)
        p1.pose.position.x = centriod_pt[0] - 0.1
        p1.pose.position.y = centriod_pt[1] 
        p1.pose.position.z = 0.48
        p1.pose.orientation.x = 0.707
        p1.pose.orientation.z = 0.707
        self.tf_listener_.waitForTransform('/base_footprint',
                                            '/hand_palm_link',
                                            rospy.Time(),
                                            rospy.Duration(4))
        p_in_hand = self.tf_listener_.transformPose("/hand_palm_link",
                                                      p1) 
        
        rospy.loginfo("Step 1: move hand forward")
        whole_body_light.set_joint_value_target(p_in_hand)
        whole_body_light.go()
        rospy.loginfo("done")

        rospy.loginfo("Step 2: start to grasp")
        gripper.set_joint_value_target("hand_motor_joint", -0.2)
        gripper.go()
        
        rospy.loginfo("Step 3: back to nuetral position")
        arm.set_named_target("neutral")
        arm.go()
        
        rospy.loginfo('Executing state Grasp')
        return 'succeeded'


class Place(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

        self.reference_frame = "map"

        # initialize the moveit_commander
        self.goal = MoveBaseGoal()

    def execute(self, userdata):
        rospy.loginfo("Step 1: Arm move forward and place the object")
        arm.set_joint_value_target("arm_lift_joint", 0.05)
        arm.go()
        arm.set_joint_value_target("arm_flex_joint", -1.2)
        arm.set_joint_value_target("wrist_flex_joint", -0.3)
        arm.go()
        rospy.sleep(3)

        rospy.loginfo("Step 2: start to place")
        gripper.set_joint_value_target("hand_motor_joint", 1.0)
        gripper.go()

        rospy.loginfo("Step 3: back to nuetral position")
        arm.set_named_target("neutral")
        arm.go()

        find_target_state = False

        rospy.loginfo('Executing state Grasp')
        return 'succeeded'


def acml_localization():
  ws_path = "/home/rcah02/workspace/tutorial5"
  uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
  roslaunch.configure_logging(uuid)
  launch = roslaunch.parent.ROSLaunchParent(uuid, [ws_path+"/src/hsrb_amcl/launch/amcl_hsrb_sim.launch"])
  launch.start()
  rospy.loginfo("started amcl")

  rospy.sleep(3)

  while(True):
    running_nodes = rosnode.get_node_names()
    runing = False
    for i in running_nodes:
      if i == "/amcl":
        runing = True
        continue
    if runing != True:
      break
  launch.shutdown()


def move_arm_initial(moveit_commander):
    arm.set_named_target("neutral")
    arm.go()
    

def nav_cb(userdata, goal):
    navGoal = MoveBaseGoal()
    navGoal.target_pose.header.frame_id = "map"
    if userdata.navGoalInd == 1:
        rospy.loginfo('Navagate to table one')
        waypoint = rospy.get_param('/way_points/table_one')
        userdata.navGoalInd = 2
    elif userdata.navGoalInd == 2:
        rospy.loginfo('Navagate to table two')
        waypoint = rospy.get_param('/way_points/table_two')
        userdata.navGoalInd = 1
    navGoal.target_pose.pose.position.x = waypoint["x"]
    navGoal.target_pose.pose.position.y = waypoint["y"]
    navGoal.target_pose.pose.orientation.z = waypoint["z"]
    navGoal.target_pose.pose.orientation.w = waypoint["w"]
    return navGoal


# main
def main():
    rospy.init_node('hsrb_state_machine')

    # initialise moveit
    moveit_commander.roscpp_initialize(sys.argv)
    global arm 
    global base
    global gripper
    global head
    global whole_body_light
    global scene
    
    arm = moveit_commander.MoveGroupCommander("arm",
                                            wait_for_servers=0.0)
    base = moveit_commander.MoveGroupCommander("base",
                                            wait_for_servers=0.0)
    gripper = moveit_commander.MoveGroupCommander("gripper",
                                                wait_for_servers=0.0)
    head = moveit_commander.MoveGroupCommander("head",
                                            wait_for_servers=0.0)
    whole_body_light \
        = moveit_commander.MoveGroupCommander('whole_body_light',
                                            wait_for_servers=0.0)

    whole_body_light.allow_replanning(True)
    whole_body_light.set_planning_time(20)
    whole_body_light.set_pose_reference_frame("map")
    scene = moveit_commander.PlanningSceneInterface()

    move_arm_initial(moveit_commander)
    # Create a SMACH state machine  
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
    # Define user data for state machine
    sm.userdata.navGoalInd = 1

    # acml localization
    # acml_localization()

    with sm:
        smach.StateMachine.add('NAVIGATION_TO_TABLE_ONE',
                                smach_ros.SimpleActionState("move_base/move", MoveBaseAction, goal_cb = nav_cb, input_keys=['navGoalInd'], output_keys=['navGoalInd']), 
                                transitions={'succeeded':'FIND_TARGET_ON_TABLE_ONE',
                                            'aborted':'aborted'})

        smach.StateMachine.add('NAVIGATION_TO_TABLE_TWO',
                        smach_ros.SimpleActionState("move_base/move", MoveBaseAction, goal_cb = nav_cb, input_keys=['navGoalInd'], output_keys=['navGoalInd']), 
                        transitions={'succeeded':'FIND_TARGET_ON_TABLE_TWO',
                                    'aborted':'aborted'})

        smach.StateMachine.add('FIND_TARGET_ON_TABLE_ONE', FindTargat(), 
                                transitions={'succeeded':'GRASP_ONE', 
                                            'aborted':'NAVIGATION_TO_TABLE_TWO'})

        smach.StateMachine.add('FIND_TARGET_ON_TABLE_TWO', FindTargat(), 
                                transitions={'succeeded':'GRASP_TWO', 
                                            'aborted':'NAVIGATION_TO_TABLE_ONE'})


        smach.StateMachine.add('GRASP_ONE', Grasp(), 
                                transitions={'succeeded':'NAVIGATION_TO_TABLE_TWO_AFTER_GRASP'})

        smach.StateMachine.add('GRASP_TWO', Grasp(), 
                                transitions={'succeeded':'NAVIGATION_TO_TABLE_ONE_AFTER_GRASP'})

        smach.StateMachine.add('NAVIGATION_TO_TABLE_TWO_AFTER_GRASP',
                                smach_ros.SimpleActionState("move_base/move", MoveBaseAction, goal_cb = nav_cb, input_keys=['navGoalInd'], output_keys=['navGoalInd']), 
                                transitions={'succeeded':'PLACE',
                                            'aborted':'NAVIGATION_TO_TABLE_ONE'})

        smach.StateMachine.add('NAVIGATION_TO_TABLE_ONE_AFTER_GRASP',
                                smach_ros.SimpleActionState("move_base/move", MoveBaseAction, goal_cb = nav_cb, input_keys=['navGoalInd'], output_keys=['navGoalInd']), 
                                transitions={'succeeded':'PLACE',
                                            'aborted':'NAVIGATION_TO_TABLE_TWO'})

        smach.StateMachine.add('PLACE', Place(), 
                                transitions={'succeeded':'NAVIGATION_TO_TABLE_ONE'})

    # Use a introspection for visulize the state machine
    sis = smach_ros.IntrospectionServer('example_server', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
