#!/usr/bin/env python3
import controller_manager_msgs.srv
import rospy
import trajectory_msgs.msg
import math

class CtrlHsrb:
    def __init__(self):
        # initialize ROS publisher
        self.head_pub = rospy.Publisher(
            '/hsrb/head_trajectory_controller/command',
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

    def head_move_to(self, pan, tilt=0):
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

