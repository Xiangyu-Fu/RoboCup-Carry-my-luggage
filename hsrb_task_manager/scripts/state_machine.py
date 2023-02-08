#!/usr/bin/env python
import rospy
import smach
import smach_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from move_test import *
import moveit_commander
from visualization_msgs.msg import Marker, MarkerArray
from tf import TransformListener
import sys
import tf
import numpy as np
import tf.transformations as tr


from sensor_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
import math
from visualization_msgs.msg import Marker, MarkerArray
import controller_manager_msgs.srv
import trajectory_msgs.msg
from gpd_ros.msg import GraspConfigList

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

# packages for subprocess
import os
from subprocess import Popen, PIPE
import signal


class CtrlHsrb:
    def __init__(self):
        # initialize ROS publisher
        self.head_pub = rospy.Publisher(
            '/hsrb/head_trajectory_controller/command',
            trajectory_msgs.msg.JointTrajectory, queue_size=10)
        self.arm_pub = rospy.Publisher('/hsrb/arm_trajectory_controller/command',
                      trajectory_msgs.msg.JointTrajectory, queue_size=10)

        # wait to establish connection between the controller
        while self.head_pub.get_num_connections() == 0:
            rospy.sleep(0.1)

        # make sure the controller is running
        rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
        list_controllers = rospy.ServiceProxy(
            '/hsrb/controller_manager/list_controllers',
            controller_manager_msgs.srv.ListControllers)
        running = False
        while running is False:
            rospy.sleep(0.1)
            for c in list_controllers().controller:
                if c.name == 'head_trajectory_controller' and c.state == 'running':
                    running = True

    def head_move_to_initial(self):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [0.0, 0.0]
        p.velocities = [0, 0]
        p.time_from_start = rospy.Duration(3)
        traj.points = [p]

        # publish ROS message
        self.head_pub.publish(traj)

    def head_move_to(self, pan=0, tilt=-0.55):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [pan, tilt]
        p.velocities = [0, 0]
        p.time_from_start = rospy.Duration(3)
        traj.points = [p]

        # publish ROS message
        self.head_pub.publish(traj)

    def hand_move_to(self, arm_roll_joint = -1.5, wrist_flex_joint = -1.5):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["arm_lift_joint", "arm_flex_joint",
                    "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [0, 0, arm_roll_joint, wrist_flex_joint, 0]
        p.velocities = [0, 0, 0, 0, 0]
        p.time_from_start = rospy.Duration(3)
        traj.points = [p]

        # publish ROS message
        self.arm_pub.publish(traj)


class GoToInitPos1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'])
        self.init_pos = rospy.get_param('/way_points/init_pos_1')
        

    def execute(self, ud):

        self.move_head = CtrlHsrb()
        self.move_head.head_move_to_initial()
        self.move_head.hand_move_to(-1.5, -1.5)

        # return 'succeeded'
        # tell the action client that we want to spin a thread by default
        move_base = actionlib.SimpleActionClient('move_base/move', MoveBaseAction)
        # wait for the action server to come up
        move_base.wait_for_server()

        rospy.loginfo('Executing state GO_TO_INIT_POS1')
        navGoal = MoveBaseGoal()
        navGoal.target_pose.header.frame_id = "map"
        navGoal.target_pose.pose.position.x = self.init_pos["x"]
        navGoal.target_pose.pose.position.y = self.init_pos["y"]
        navGoal.target_pose.pose.orientation.z = self.init_pos["z"]
        navGoal.target_pose.pose.orientation.w = self.init_pos["w"]
        move_base.send_goal(navGoal)
        if move_base.wait_for_result(rospy.Duration(10)): # wait for 20 seconds
            rospy.loginfo('Executing state GO_TO_INIT_POS1 SUCCEEDED')
            return 'succeeded'
        else:
            rospy.logerr('Executing state GO_TO_INIT_POS1 FAILED')
            return 'aborted'


class GoToInitPos2(smach.State):
    """
    this state is used to go to the initial position 2, which make the move_base can work again
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'])
        self.init_pos = rospy.get_param('/way_points/init_pos_2')
        
    def execute(self, ud):

        self.move_head = CtrlHsrb()
        self.move_head.head_move_to_initial()
        self.move_head.hand_move_to(-1.5, -1.5)

        # return 'succeeded'
        # tell the action client that we want to spin a thread by default
        move_base = actionlib.SimpleActionClient('move_base/move', MoveBaseAction)
        # wait for the action server to come up
        move_base.wait_for_server()

        rospy.loginfo('Executing state GO_TO_INIT_POS2')
        navGoal = MoveBaseGoal()
        navGoal.target_pose.header.frame_id = "map"
        navGoal.target_pose.pose.position.x = self.init_pos["x"]
        navGoal.target_pose.pose.position.y = self.init_pos["y"]
        navGoal.target_pose.pose.orientation.z = self.init_pos["z"]
        navGoal.target_pose.pose.orientation.w = self.init_pos["w"]
        move_base.send_goal(navGoal)
        if move_base.wait_for_result(rospy.Duration(20)): # wait for 20 seconds
            rospy.loginfo('Executing state GO_TO_INIT_POS2 SUCCEEDED')
            return 'succeeded'
        else:
            rospy.logerr('Executing state GO_TO_INIT_POS2 FAILED')
            return 'aborted'


class GoToIntermediatePos(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'])
        self.intermediate_pos = rospy.get_param('/way_points/intermediate_pos')
        

    def execute(self, ud):

        self.move_head = CtrlHsrb()
        self.move_head.head_move_to_initial()
        self.move_head.hand_move_to(-1.5, -1.5)

        # return 'succeeded'
        # tell the action client that we want to spin a thread by default
        move_base = actionlib.SimpleActionClient('move_base/move', MoveBaseAction)
        # wait for the action server to come up
        move_base.wait_for_server()

        rospy.loginfo('Executing state GO_TO_INTERMEDIATE_POS')
        navGoal = MoveBaseGoal()
        navGoal.target_pose.header.frame_id = "map"
        navGoal.target_pose.pose.position.x = self.intermediate_pos["x"]
        navGoal.target_pose.pose.position.y = self.intermediate_pos["y"]
        navGoal.target_pose.pose.orientation.z = self.intermediate_pos["z"]
        navGoal.target_pose.pose.orientation.w = self.intermediate_pos["w"]
        move_base.send_goal(navGoal)
        if move_base.wait_for_result(rospy.Duration(20)): # wait for 20 seconds
            rospy.loginfo('Executing state GO_TO_INTERMEDIATE_POS SUCCEEDED')
            return 'succeeded'
        else:
            rospy.logerr('Executing state GO_TO_INTERMEDIATE_POS FAILED')
            return 'aborted'


class GetObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'])
        self.flag = False

        self.object_names = rospy.get_param('/object_names')
        self.obj_1_name = self.object_names['object_left']
        self.obj_2_name = self.object_names['object_right']
    

    def cal_angle_2points(self, point1, point2):
        # coordinates of two points 
        x1 = point1[0] 
        y1 = point1[1]
        z1 = point1[2]

        x2 = point2[0]
        y2 = point2[1]
        z2 = point2[2]

        # calculate the vector joining the two points 
        vx = x2 - x1 
        vy = y2 - y1 
        vz = z2 - z1 

        # calculate angle between the vector and z-axis 
        mag = math.sqrt(vx**2 + vy**2 + vz**2) 
        dot_product = vz / mag 
        angle = math.acos(dot_product) 

        # convert angle to degrees 
        angle = angle * 180 / math.pi 
        return angle
        

    def execute(self, ud):
        rospy.loginfo('Executing state GET_OBJECT')

        if rospy.get_param('/use_sim'):
            self.flag = True
            rospy.set_param('~target_obj', self.obj_1_name)
            return 'succeeded'

        # run the get obejct subprocess
        get_object_sproc = Popen("bash /home/athome/catkin_ws/src/hsrb_task_manager/launch/get_object.sh", shell=True, preexec_fn=os.setsid)

        # control the head
        self.move_head = CtrlHsrb()
        self.move_head.head_move_to_initial()

        listener = tf.TransformListener()

        left_num = 0
        right_num = 0
        angle_threshold = 60
        num_threshold = 10

        time_count = rospy.Time.now()

        while not self.flag:
            try:
                (right_wrist_pos,rot) = listener.lookupTransform('/base_link', '/right_wrist_hsrb', rospy.Time(0))
                (right_elbow_pos,rot) = listener.lookupTransform('/base_link', '/right_elbow_hsrb', rospy.Time(0))
                (left_wrist_pos,rot) = listener.lookupTransform('/base_link', '/left_wrist_hsrb', rospy.Time(0))
                (left_elbow_pos,rot) = listener.lookupTransform('/base_link', '/left_elbow_hsrb', rospy.Time(0))

                angle_right_arm = self.cal_angle_2points(right_wrist_pos, right_elbow_pos)
                angle_left_arm = self.cal_angle_2points(left_wrist_pos, left_elbow_pos)

                if rospy.Time.now() - time_count > rospy.Duration(90):
                    rospy.logerr('Executing state GET_OBJECT FAILED')
                    rospy.logerr('Please check the hri human detection node ...')
                    rospy.logwarn('Now using the default object: ' + self.obj_1_name)
                    rospy.set_param('~target_obj', self.obj_1_name)
                    self.flag = True
                    os.killpg(os.getpgid(get_object_sproc.pid), signal.SIGTERM)

                    return 'aborted'
                
                if angle_right_arm > angle_threshold:
                    right_num += 1
                    left_num = 0
                    if right_num > num_threshold:
                        right_num = 0
                        # the global variable object is used to store the object name
                        rospy.loginfo('[GET OBJECT] right arm')
                        rospy.set_param('~target_obj', self.obj_1_name)
                        self.flag = True
                        
                if angle_left_arm > angle_threshold:
                    left_num += 1
                    right_num = 0
                    if left_num > num_threshold:
                        left_num = 0
                        # the global variable object is used to store the object name
                        rospy.loginfo('[GET OBJECT] left arm')
                        rospy.set_param('~target_obj', self.obj_2_name)
                        self.flag = True

                rospy.Rate(10).sleep()

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        # kill the subprocess
        os.killpg(os.getpgid(get_object_sproc.pid), signal.SIGTERM) 
        rospy.loginfo('Executing state GET_OBJECT SUCCEEDED')
        return 'succeeded'


class GoToTable(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'])

        # get different table position from param server
        self.obj_1_pos = rospy.get_param('/way_points/object_one')
        self.obj_2_pos = rospy.get_param('/way_points/object_two')

        self.object_names = rospy.get_param('/object_names')
        self.obj_1_name = self.object_names['object_left']
        self.obj_2_name = self.object_names['object_right']

        self.flag = False
        

    def execute(self, ud):
        # tell the action client that we want to spin a thread by default
        move_base = actionlib.SimpleActionClient('move_base/move', MoveBaseAction)
        # wait for the action server to come up
        move_base.wait_for_server()

        if rospy.get_param('/use_sim'):
            rospy.set_param('~target_obj', 'bottle')

        self.target_name = rospy.get_param('~target_obj')
        rospy.loginfo('Get object name: ' + self.target_name)

        while not self.flag:
            if self.target_name == self.obj_1_name:
                rospy.loginfo('Executing state GO_TO_TABLE_LEFT')
                navGoal = MoveBaseGoal()
                navGoal.target_pose.header.frame_id = "map"
                navGoal.target_pose.pose.position.x = self.obj_1_pos["x"]
                navGoal.target_pose.pose.position.y = self.obj_1_pos["y"]
                navGoal.target_pose.pose.orientation.z = self.obj_1_pos["z"]
                navGoal.target_pose.pose.orientation.w = self.obj_1_pos["w"]
                rospy.loginfo('target position x: ' + str(navGoal.target_pose.pose.position.x))
                rospy.loginfo('target position y: ' + str(navGoal.target_pose.pose.position.y))
                # get the feedback
                move_base.send_goal(navGoal)
                # wait for 20 seconds
                if move_base.wait_for_result(rospy.Duration(20)):
                    rospy.loginfo('Executing state GO_TO_TABLE SUCCEEDED')
                    return 'succeeded'
                else:
                    rospy.logerr('Executing state GO_TO_TABLE FAILED')
                    return 'aborted'
            
            elif self.target_name == self.obj_2_name:
                rospy.loginfo('Executing state GO_TO_TABLE_RIGHT')
                navGoal = MoveBaseGoal()
                navGoal.target_pose.header.frame_id = "map"
                navGoal.target_pose.pose.position.x = self.obj_2_pos["x"]
                navGoal.target_pose.pose.position.y = self.obj_2_pos["y"]
                navGoal.target_pose.pose.orientation.z = self.obj_2_pos["z"]
                navGoal.target_pose.pose.orientation.w = self.obj_2_pos["w"]
                # get the feedback
                move_base.send_goal(navGoal)
                # wait for 20 seconds
                if move_base.wait_for_result(rospy.Duration(20)):
                    rospy.loginfo('Executing state GO_TO_TABLE SUCCEEDED')
                    return 'succeeded'
                else:
                    rospy.logerr('Executing state GO_TO_TABLE FAILED')
                    return 'aborted'
            else:
                rospy.Rate(10).sleep()
                continue
                

class Grasp(smach.State):
    def __init__(self, wait=3.0):
        # define the outcome of the state
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'])

        self.flag = False
        self.wait=wait

    def grasp_callback(self, msg):
        if len(msg.grasps) == 0:
            rospy.logwarn("No grasps found")
            return
        self.selected_grasp = msg.grasps[0]
        # print(self.selected_grasp)

        # Create a PoseStamped message from the GraspConfig message
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = msg.header.frame_id
        # ------ transform GraspConfig to PoseStamped ------
        p.pose.position = self.selected_grasp.position
        R =np.array([[self.selected_grasp.approach.x, self.selected_grasp.binormal.x, self.selected_grasp.axis.x,0],
            [self.selected_grasp.approach.y, self.selected_grasp.binormal.y, self.selected_grasp.axis.y,0],
            [self.selected_grasp.approach.z, self.selected_grasp.binormal.z, self.selected_grasp.axis.z,0],
            [0,0,0,1]])
        Q = tr.quaternion_from_matrix(R)
        p.pose.orientation.x = Q[0]
        p.pose.orientation.y = Q[1]
        p.pose.orientation.z = Q[2]
        p.pose.orientation.w = Q[3]
        # ------ add p to tf tree ------
        self.br.sendTransform((p.pose.position.x, p.pose.position.y, p.pose.position.z),
        (p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w),
        rospy.Time.now(),
        "selected_grasp",
        p.header.frame_id)

        # Rotate the selected grasp frame to /hand_palm_link
        p_rot = geometry_msgs.msg.PoseStamped()
        p_rot.header.frame_id = "/selected_grasp"

        qx = tr.quaternion_about_axis(np.pi,(1,0,0))
        qy = tr.quaternion_about_axis(np.pi/2, (0,1,0))
        q = tr.quaternion_multiply(qx,qy)
        p_rot.pose.orientation.x=q[0]
        p_rot.pose.orientation.y=q[1]
        p_rot.pose.orientation.z=q[2]
        p_rot.pose.orientation.w=q[3]

        # ----------------- transform final grasp position to /odom -------------------
        self.tf_listener.waitForTransform('odom','/selected_grasp', rospy.Time(), rospy.Duration(4.0))
        self.p_final = self.tf_listener.transformPose('odom', p_rot)
        # ------------ add final grasp pose to tf tree ------------
        self.br.sendTransform((self.p_final.pose.position.x, self.p_final.pose.position.y, self.p_final.pose.position.z),
        (self.p_final.pose.orientation.x, self.p_final.pose.orientation.y, self.p_final.pose.orientation.z, self.p_final.pose.orientation.w),
        rospy.Time.now(),
        "/p_final",
        self.p_final.header.frame_id)
        # rospy.loginfo(self.p_final)
        
        # set an intermidate position wrt. final grasp pose
        way_point = geometry_msgs.msg.PoseStamped()
        way_point.header.frame_id = "/p_final"
        way_point.pose.position.x = 0
        way_point.pose.position.y = 0
        way_point.pose.position.z = -0.15
        way_point.pose.orientation.w = 1 

        # Transform the pose from /p_final to odom
        self.tf_listener.waitForTransform('odom','/p_final', rospy.Time(), rospy.Duration(4.0))
        self.way_point = self.tf_listener.transformPose('odom', way_point)

        self.br.sendTransform((self.way_point.pose.position.x, self.way_point.pose.position.y, self.way_point.pose.position.z),
        (self.way_point.pose.orientation.x, self.way_point.pose.orientation.y, self.way_point.pose.orientation.z, self.way_point.pose.orientation.w),
        rospy.Time.now(),
        "/way_point",
        self.way_point.header.frame_id)   

        self.flag = True


    
    def execute(self, ud):
        rospy.loginfo("start grasping")

        # set odom publisher 
        odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)

        self.tf_listener = tf.TransformListener()
        

        # initialize moveit commander
        moveit_commander.roscpp_initialize(sys.argv)

        self.br = tf.TransformBroadcaster()

        self.p_final=geometry_msgs.msg.PoseStamped()
        self.way_point=geometry_msgs.msg.PoseStamped()
       
        self.arm = moveit_commander.MoveGroupCommander('arm',
                                                  wait_for_servers=0.0)

        self.gripper = moveit_commander.MoveGroupCommander("gripper",
                                                      wait_for_servers=0.0)

        self.whole_body_light \
            = moveit_commander.MoveGroupCommander('whole_body_light',
                                                  wait_for_servers=0.0)

        self.reference_frame = "odom"
        self.scene = moveit_commander.PlanningSceneInterface()
        self.whole_body_light.allow_replanning(True)
        self.whole_body_light.set_planning_time(20)
        self.whole_body_light.set_pose_reference_frame(self.reference_frame)
        
        print(self.whole_body_light.get_current_joint_values())
        if len(self.whole_body_light.get_current_joint_values()) == 0:
            print(self.whole_body_light.get_current_joint_values())
            rospy.logerr("No joint values found, please check the async spinner. Make sure you have enough computational power to run the moveit")

        force_sensor_capture = ForceSensorCapture()
        # Get initial data of force sensor
        pre_force_list = force_sensor_capture.get_current_force()
        print("pre_force_list: ",pre_force_list)

        
        rospy.loginfo("open gripper")
        self.end_effector = self.whole_body_light.get_end_effector_link()
        self.gripper.set_joint_value_target("hand_motor_joint", 1.0)
        self.gripper.go()
        rospy.logdebug("ready to take grasp pose...")
        rospy.sleep(3.0)
  
        self.move_head = CtrlHsrb()
        self.move_head.head_move_to(tilt=-0.66)
        self.move_head.hand_move_to()
        rospy.loginfo('Waiting for GRASP pose...')

        ''' The gpd and plane segmentation nodes are not stable, so run it in another computer.'''
        if not rospy.get_param("/use_distributed"):
            gpd_subprocess = Popen("bash /home/athome/catkin_ws/src/hsrb_task_manager/launch/run_gpd.sh", shell=True, preexec_fn=os.setsid)
            rospy.sleep(7.0)

            plane_seg_subprocess = Popen("bash /home/athome/catkin_ws/src/hsrb_task_manager/launch/plane_segmentation.sh", shell=True, preexec_fn=os.setsid)

        rospy.Subscriber('/detect_grasps/clustered_grasps', GraspConfigList, self.grasp_callback)

        while not self.flag:
            pass
        
        # update the odom
        (pos, rot) = self.tf_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        self.odom =Odometry()
        self.odom.pose.pose.position.x = pos[0]
        self.odom.pose.pose.position.y = pos[1]
        self.odom.pose.pose.position.z = pos[2]
        self.odom.pose.pose.orientation.x = rot[0]
        self.odom.pose.pose.orientation.y = rot[1]
        self.odom.pose.pose.orientation.z = rot[2]
        self.odom.pose.pose.orientation.w = rot[3]

        # publish the odom
        odom_pub.publish(self.odom)


        # check the status of the gpd subprocess
        if not rospy.get_param("/use_distributed"):
            os.killpg(os.getpgid(gpd_subprocess.pid), signal.SIGTERM) 
            os.killpg(os.getpgid(plane_seg_subprocess.pid), signal.SIGTERM)

        # ----------------- move to intermidate position -------------------
        rospy.loginfo("go to intermidate position!")  
        # rospy.loginfo(self.way_point)
        self.whole_body_light.set_joint_value_target(self.way_point)
        self.whole_body_light.set_start_state(self.whole_body_light.get_current_state())
        self.whole_body_light.go()
        rospy.loginfo("done")
        rospy.sleep(2.0)

        
        # ----------------- move to final grasp position -------------------
        rospy.loginfo("go to grasp position")
        self.p_final.pose.position.z = self.p_final.pose.position.z-0.03
        # rospy.loginfo(self.p_final)
        self.whole_body_light.set_joint_value_target(self.p_final)
        self.whole_body_light.go()
        rospy.loginfo("done")
        rospy.sleep(self.wait)

        rospy.loginfo("grasp!")
        self.gripper.set_joint_value_target("hand_motor_joint", -0.1)
        self.gripper.go()

        rospy.loginfo("arm back")
        self.arm.set_named_target("neutral")
        self.arm.go()
        rospy.loginfo("GRASP DONE!")
        rospy.sleep(2.0)

        # Wait until force sensor data become stable
        rospy.sleep(1.0)
        post_force_list = force_sensor_capture.get_current_force()
        print("post_force_list: ",post_force_list)
        force_difference = force_sensor_capture.compute_difference(pre_force_list, post_force_list)
        print("force different: ",force_difference)
         # Convert newton to gram
        weight = round(force_difference / 9.81 * 1000, 1)
        print("weight: ",weight)
        if force_difference > 6:
            return 'succeeded'
        else:
            return 'aborted'


class ForceSensorCapture(object):
    """Subscribe and hold force sensor data"""

    def __init__(self):
        self._force_data_x = 0.0
        self._force_data_y = 0.0
        self._force_data_z = 0.0

        # Subscribe force torque sensor data from HSRB
        ft_sensor_topic = '/hsrb/wrist_wrench/raw'
        self._wrist_wrench_sub = rospy.Subscriber(
            ft_sensor_topic, WrenchStamped, self.__ft_sensor_cb)

        # Wait for connection
        try:
            rospy.wait_for_message(ft_sensor_topic, WrenchStamped,
                                   timeout=10.0)
        except Exception as e:
            rospy.logerr(e)
            sys.exit(1)

    def get_current_force(self):
        return [self._force_data_x, self._force_data_y, self._force_data_z]

    def __ft_sensor_cb(self, data):
        self._force_data_x = data.wrench.force.x
        self._force_data_y = data.wrench.force.y
        self._force_data_z = data.wrench.force.z

    def compute_difference(self, pre_data_list, post_data_list):
        if (len(pre_data_list) != len(post_data_list)):
            raise ValueError('Argument lists differ in length')
        # Calcurate square sum of difference
        square_sums = sum([math.pow(b - a, 2)
                        for (a, b) in zip(pre_data_list, post_data_list)])
        return math.sqrt(square_sums)



class FollowPerson(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'])
        self.tf_listener = tf.TransformListener()

        self.enable_back = rospy.get_param('~enable_back', True)
        self.max_vx = rospy.get_param('~max_vx', 0.2)
        self.max_va = rospy.get_param('~max_va', 2.0)
        self.gain_vx = rospy.get_param('~gain_vx', 1.0)
        self.gain_va = rospy.get_param('~gain_va', 1.0)
        self.distance = rospy.get_param('~distance', 0)
        self.timeout = rospy.get_param('~timeout', 5.0)

        self.last_time = rospy.Time.now()

        # set up the globel twist for cmd vel
        self.twist = Twist()
        self.vx = 0.0
        self.va = 0.0

        self.find_person = False
        self.person_stop = False
        if rospy.get_param('/use_sim'):
            self.target_pos_x = 2.6
            self.target_pos_y = 1.1
        else:
            self.target_pos_x = rospy.get_param('~target_pos_x', 0.2)
            self.target_pos_y = rospy.get_param('~target_pos_y', 0.2)

        self.left_link = rospy.get_param('~left_link', '/left_ankle_hsrb')
        self.right_link = rospy.get_param('~right_link', '/right_ankle_hsrb')

    def tracks_callback(self, tracks_msg):
        target = tracks_msg.tracks
    
        if len(target) == 0:
            print("No target found")
            self.find_person = False
            return self.find_person
        else:
            print("Find target")
            self.find_person = True
            # stop the robot if find target
            # self.vx = 0.0
            # self.va = 0.0
            # self.cmd_vel_pub.publish(self.twist)
        
        if self.find_person == True:
            point = PointStamped()
            point.header = tracks_msg.header
            point.point = target[0].pos

            # set up the goal
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'

            try:
                self.tf_listener.waitForTransform('map', 'base_link', tracks_msg.header.stamp, rospy.Duration(0.5))
                target_pos = self.tf_listener.transformPoint('map', point)
                goal.target_pose.header.stamp = rospy.Time.now()
                x = target_pos.point.x
                y = target_pos.point.y
                if (x - self.target_x) ** 2 + (y - self.target_y) ** 2 < 1.5:
                    self.person_stop = True
                    print("Person stop")
                else:
                    self.person_stop = False
                    print("Person moving")
                # intersec_x, intersec_y = self.get_intersec_point(target_pos.point.x, target_pos.point.y, self.distance)
                # goal.target_pose.pose.position.x = intersec_x
                # goal.target_pose.pose.position.y = intersec_y
                # goal.target_pose.pose.orientation.w = 1.0
                # print('goal:', goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
                # self.move_base.send_goal(goal)
                rospy.sleep(0.5)

            except:
                print('failed to lookup transform between map and', 'base_link')
                self.va = 0.0
                self.vx = 0.0
                return

            # calculate the rotate angle
            theta = math.atan2(point.point.y, point.point.x)

            va = min(self.max_va, max(-self.max_va, theta * self.gain_va))
            vx = 0.0
            if abs(theta) < math.radians(45):
                # print("target_pos.point.x:", target_pos.point.x)
                vx = (point.point.x - self.distance) * self.gain_vx
                min_vx = -self.max_vx if self.enable_back else 0.0
                vx = min(self.max_vx, max(min_vx, vx))
            else:
                print('rotation too big')

            self.vx = 1.4*vx
            self.va = 3.0*va   
            print('vx:', self.vx, 'va:', self.va)      
            self.last_time = rospy.Time.now()
            self.twist.linear.x = self.vx
            self.twist.angular.z = self.va
            self.cmd_vel_pub.publish(self.twist)


    def low_pass_filter(self, x, x_old, alpha):
        """
        low pass filter to make the output value more smooth
        :param x:   current value
        :param x_old:   old value
        :param alpha:   the weight of old value
        """
        return alpha * x + (1.0 - alpha) * x_old

    def get_intersec_point(self, x, y, r):
        # calculate the distance between the point and the origin
        d = math.sqrt(x**2 + y**2)

        # calculate the the intersection point
        x_intersection = x - r*x/d
        y_intersection = y - r*y/d

        return x_intersection, y_intersection

    def command_pub(self):
        """
        publish the command velocity and rotate angularity, use one function to avoid the conflict
        """
        self.twist.linear.x = self.low_pass_filter(self.vx, self.vx_old, 0.9)
        self.twist.angular.z = self.low_pass_filter(self.va, self.va_old, 0.9)
        self.vx_old = self.vx
        self.va_old = self.va
        self.cmd_vel_pub.publish(self.twist)

    def head_move_spin(self):
        # rotate the head in sinus
        head_pos = math.sin(rospy.Time.now().to_sec() * 0.3) * 1.0
    #     self.move_head.head_move_to(head_pos)

    def whole_body_spin(self):
        """
        rotate the whole body in sinus, when there is no target found
        """
        # rotate the whole body in sinus
        self.twist.linear.x = 0.0
        self.twist.angular.z = math.sin(rospy.Time.now().to_sec() * 0.15) * 0.5
        self.cmd_vel_pub.publish(self.twist)

        try:
            (right_ankle_hsrb,rot) = self.listener.lookupTransform('/base_link', self.right_link, rospy.Time(0))
            return True

        except:
            rospy.loginfo("No person found")
            return False

    def get_person_pos(self):
        try:
            (right_ankle_hsrb,rot) = self.listener.lookupTransform('/base_link', self.right_link, rospy.Time(0))
            (left_ankle_hsrb,rot) = self.listener.lookupTransform('/base_link', self.left_link, rospy.Time(0))

            (right_ankle_map,rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            (left_ankle_map,rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
    

            # get the center of the person to the map
            self.person_x_map = (right_ankle_map[0] + left_ankle_map[0]) / 2.0
            self.person_y_map = (right_ankle_map[1] + left_ankle_map[1]) / 2.0

            rospy.loginfo("distance to target: %f", math.sqrt((self.person_x_map - self.target_pos_x)**2 + (self.person_y_map - self.target_pos_y)**2))
            # detect if the person is near to the target pos
            if math.sqrt((self.person_x_map - self.target_pos_x)**2 + (self.person_y_map - self.target_pos_y)**2) < 1.0:
                self.person_stop = True
                self.stop_robot()
                rospy.loginfo('Arrived at the target position')

            if self.stop_sign_detect():
                self.person_stop = True
                self.stop_robot()
                rospy.loginfo('Detect the stop sign')

            # get the center of the person to the base_link
            self.person_x = (right_ankle_hsrb[0] + left_ankle_hsrb[0]) / 2.0
            self.person_y = (right_ankle_hsrb[1] + left_ankle_hsrb[1]) / 2.0

            self.person_x = self.low_pass_filter(self.person_x, self.person_x_old, 0.8)
            self.person_y = self.low_pass_filter(self.person_y, self.person_y_old, 0.8)

            self.person_x_old = self.person_x
            self.person_y_old = self.person_y

            return True

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn('target lost')
            return False

    def move_robot(self):
        """
        move the robot to the target position
        """
        # calculate the rotate angle
        theta = math.atan2(self.person_y, self.person_x)

        va = min(self.max_va, max(-self.max_va, theta * self.gain_va))
        vx = 0.0
        if abs(theta) < math.radians(45):
            # print("target_pos.point.x:", target_pos.point.x)
            vx = (self.person_x - self.distance) * self.gain_vx
            min_vx = -self.max_vx if self.enable_back else 0.0
            vx = min(self.max_vx, max(min_vx, vx))
        else:
            print('rotation too big')

        self.vx = self.gain_vx*vx
        self.va = self.gain_va*va

        # print('vx:', self.vx, 'va:', self.va)      
        self.last_time = rospy.Time.now()
        self.twist.linear.x = self.vx
        self.twist.angular.z = self.va
        self.cmd_vel_pub.publish(self.twist)

        # update the last time
        self.last_time = rospy.Time.now()

    def stop_sign_detect(self):
        """
        detect if there is a stop sign in front of the robot
        """
        try:
            (wrist_pos,rot) = self.listener.lookupTransform('/left_wrist_hsrb', '/left_shoulder_hsrb', rospy.Time(0))
            print('wrist_pos:', wrist_pos)
            # check the z value of the wrist
            if abs(wrist_pos[2]) < 0.08:
                if abs(wrist_pos[1])<0.08:
                    if abs(wrist_pos[0])>0.2:
                        return True
            return False

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return False

    def stop_robot(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)

    def spin(self):
        if not self.person_stop:
            if (rospy.Time.now() - self.last_time).to_sec() > self.timeout:
                # if no target found, stop the robot
                print('no target found, roate the robot to find the target ... ')
                self.last_time = rospy.Time.now()
                self.whole_body_spin()

            else:
                # get the filtered person position
                if self.get_person_pos():
                    self.move_robot()
        else:
            self.stop_robot()

   
    def execute(self, userdata):
        rospy.loginfo('Executing state FOLLOW_PERSON')

        '''
        !!! Notice the following code is for the monocular people following package, but now it have been deprecated 
        # run the start camera subprocess
        camera_subprocess = Popen("bash /home/athome/catkin_ws/src/hsrb_task_manager/launch/start_camera.sh", shell=True)
        # run the human following subprocess 
        following_subprocess = Popen("bash /home/athome/catkin_ws/src/hsrb_task_manager/launch/human_follow_env.sh", shell=True)

        # tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient('move_base/move', MoveBaseAction)
        # wait for the action server to come up
        self.move_base.wait_for_server()
        '''
        
        # run the get obejct subprocess, here we reuse the get object subprocess
        following_subprocess = Popen("bash /home/athome/catkin_ws/src/hsrb_task_manager/launch/get_object.sh", shell=True, preexec_fn=os.setsid)
        rospy.sleep(5)

        # move the head to initial position
        self.move_head = CtrlHsrb()
        self.move_head.head_move_to_initial()

        # define the transform listener
        self.listener = tf.TransformListener()

        # set up the publisher and subscriber
        self.cmd_vel_pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=20)
        # self.tracks_sub = rospy.Subscriber('/monocular_people_tracking/tracks', TrackArray, self.tracks_callback)

        # wait for the tf to be ready
        tf_ready = False
        while not tf_ready:
            try:
                (right_ankle_hsrb,rot) = self.listener.lookupTransform('/base_link', self.right_link, rospy.Time(0))
                (left_ankle_hsrb,rot) = self.listener.lookupTransform('/base_link', self.left_link, rospy.Time(0))

                # get the initial person position
                self.person_x_old = (right_ankle_hsrb[0] + left_ankle_hsrb[0]) / 2.0
                self.person_y_old = (right_ankle_hsrb[1] + left_ankle_hsrb[1]) / 2.0

                tf_ready = True

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("Failed to get the person position Trying again ...")
                # rospy.logwarn("right_ankle_hsrb: %s, left_ankle_hsrb: %s", self.right_link, self.left_link)
                rospy.sleep(1.0)

        while not self.person_stop:
            self.spin()
            rospy.sleep(0.4)

        if self.person_stop == True:
            # kill the subprocess to save the resource of computer
            os.killpg(os.getpgid(following_subprocess.pid), signal.SIGTERM) 
            return 'succeeded'
        else: 
            os.killpg(os.getpgid(following_subprocess.pid), signal.SIGTERM) 
            return 'aborted'


class Place(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'])

        self.reference_frame = "map"

        # initialize the moveit_commander
        self.goal = MoveBaseGoal()

    def execute(self, userdata):
        arm = moveit_commander.MoveGroupCommander("arm",
                                            wait_for_servers=0.0)
        gripper = moveit_commander.MoveGroupCommander("gripper",
                                            wait_for_servers=0.0)

        rospy.loginfo("Step 1: Arm move forward and place the object")
        arm.set_joint_value_target("arm_lift_joint", 0.25)
        arm.set_joint_value_target("arm_roll_joint", 0.0)
        arm.set_joint_value_target("arm_flex_joint", -1.0)
        arm.set_joint_value_target("wrist_flex_joint", -0.3)
        arm.go()

        rospy.loginfo("Step 2: start to place")
        gripper.set_joint_value_target("hand_motor_joint", 1.0)
        gripper.go()

        rospy.sleep(1.0)

        rospy.loginfo("Step 3: back to nuetral position")
        arm.set_named_target("neutral")
        arm.go()

        rospy.loginfo('Executing state Grasp')
        return 'succeeded'


def main():
    # Initialize the node
    rospy.init_node('hsrb_task_manager_node')

    # set up some global variables
    global object 
    object = "None"

    # Create a SMACH state machine  
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    with sm:
        # smach.StateMachine.add('GoToTable_t', GoToTable(), transitions={'succeeded':'GraspTest_t', 'aborted':'GoToTable', 'preempted':'preempted'})
        # smach.StateMachine.add('GraspTest_t', Grasp(), transitions={'succeeded':'succeeded', 'aborted':'GoToTable', 'preempted':'preempted'})
        # Add states to the container
        smach.StateMachine.add('GoToInitPos1', GoToInitPos1(), transitions={'succeeded':'GetObject', 'aborted':'GoToInitPos2', 'preempted':'preempted'})
        smach.StateMachine.add('GoToInitPos2', GoToInitPos2(), transitions={'succeeded':'GoToInitPos1', 'aborted':'GoToInitPos1', 'preempted':'preempted'})
        smach.StateMachine.add('GetObject', GetObject(), transitions={'succeeded':'GoToTable', 'aborted':'GoToInitPos1', 'preempted':'preempted'})
        smach.StateMachine.add('GoToTable', GoToTable(), transitions={'succeeded':'Grasp', 'aborted':'GoToTable', 'preempted':'preempted'})
        smach.StateMachine.add('Grasp', Grasp(), transitions={'succeeded':'GoToInitPos', 'aborted':'GoToIntermediatePos', 'preempted':'preempted'})

        smach.StateMachine.add('GoToInitPos', GoToInitPos1(), transitions={'succeeded':'FollowPerson', 'aborted':'GoToIntermediatePos', 'preempted':'preempted'})
        smach.StateMachine.add('GoToIntermediatePos', GoToIntermediatePos(), transitions={'succeeded':'GoToInitPos', 'aborted':'GoToInitPos', 'preempted':'preempted'})
        
        smach.StateMachine.add('FollowPerson', FollowPerson(), transitions={'succeeded':'Place', 'aborted':'GoToInitPos1', 'preempted':'preempted'})
        smach.StateMachine.add('Place', Place(), transitions={'succeeded':'succeeded', 'aborted':'GoToInitPos1', 'preempted':'preempted'})

        # smach.StateMachine.add('GoToInitPos3', GoToInitPos1(), transitions={'succeeded':'succeeded', 'aborted':'GoToInitPos3', 'preempted':'preempted'})

    # Use a introspection for visulize the state machine
    sis = smach_ros.IntrospectionServer('example_server', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()