#!/usr/bin/env python3

import sys
import rospy
import tf
import geometry_msgs.msg
from gpd_ros.msg import GraspConfigList
import numpy as np
import tf.transformations as tr
import moveit_commander
import tf2_geometry_msgs

class GraspToPose:
    def __init__(self,wait=0.0):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('grasp_to_pose')
        self.wait=wait
        self.tf_listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        self.p_hand=geometry_msgs.msg.PoseStamped()
       
        # self.arm = moveit_commander.MoveGroupCommander('arm',
        #                                           wait_for_servers=0.0)

        self.gripper = moveit_commander.MoveGroupCommander("gripper",
                                                      wait_for_servers=10.0)

        self.whole_body_light \
            = moveit_commander.MoveGroupCommander('whole_body_light',
                                                  wait_for_servers=10.0)

        self.reference_frame = "odom"
        self.scene = moveit_commander.PlanningSceneInterface()
        self.whole_body_light.allow_replanning(True)
        self.whole_body_light.set_planning_time(20)
        self.whole_body_light.set_pose_reference_frame(self.reference_frame)
       
        # rospy.loginfo("move_to_neutral")
        # self.arm.set_named_target("neutral")
        # self.arm.go()
        # rospy.sleep(0.0)

        rospy.loginfo("open gripper")
        self.end_effector = self.whole_body_light.get_end_effector_link()
        self.gripper.set_joint_value_target("hand_motor_joint", 0.7)
        self.gripper.go()
        rospy.logdebug("done")
        rospy.sleep(0.0)
        # ------------------test position ------------------
        # p_test = geometry_msgs.msg.PoseStamped()
        # p_test.header.frame_id = "/hand_palm_link"
        # p_test.pose.position.x = 0
        # p_test.pose.position.y = 0
        # p_test.pose.position.z = 0.15
        # p_test.pose.orientation.w = 1
        # self.br.sendTransform((p_test.pose.position.x, p_test.pose.position.y, p_test.pose.position.z),
        # (p_test.pose.orientation.x, p_test.pose.orientation.y, p_test.pose.orientation.z, p_test.pose.orientation.w),
        # rospy.Time.now(),
        # "/p_test",
        # p_test.header.frame_id)     
        # # ------------------test position ---------------------

        # rospy.loginfo(p_test)

        # self.arm.set_joint_value_target(p_test) # works when not near the table
        # self.arm.go()
        # rospy.logdebug("done")
        # rospy.sleep(5.0)

        # Subscriber to clustered grasps topic
        if len(self.whole_body_light.get_current_joint_values()) == 0:
            rospy.logerr("No joint values found, please check the async spinner. Make sure you have enough computational power to run the moveit")
        rospy.Subscriber('/detect_grasps/clustered_grasps', GraspConfigList, self.grasp_callback)

        
        # Publisher for transformed poses
        # self.p_hand_pub = rospy.Publisher('/p_hand', geometry_msgs.msg.PoseStamped, queue_size=1)
         # Publish the transformed pose
        


    def grasp_callback(self, msg):
        if len(msg.grasps) == 0:
            print("No grasps found", end="\r")
            return
        self.selected_grasp = msg.grasps[0]
        print(self.selected_grasp)

        # Create a PoseStamped message from the GraspConfig message
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = msg.header.frame_id

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


        rospy.loginfo(p)
        # while not rospy.is_shutdown():
        self.br.sendTransform((p.pose.position.x, p.pose.position.y, p.pose.position.z),
        (p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w),
        rospy.Time.now(),
        "selected_grasp",
        p.header.frame_id)

        # Rotate the grasp frame according the /hand_palm_link
        p_rot = geometry_msgs.msg.PoseStamped()
        p_rot.header.frame_id = "/selected_grasp"

        qx = tr.quaternion_about_axis(np.pi,(1,0,0))
        qy = tr.quaternion_about_axis(np.pi/2, (0,1,0))
        q = tr.quaternion_multiply(qx,qy)
        p_rot.pose.orientation.x=q[0]
        p_rot.pose.orientation.y=q[1]
        p_rot.pose.orientation.z=q[2]
        p_rot.pose.orientation.w=q[3]
        # self.br.sendTransform((p_rot.pose.position.x, p_rot.pose.position.y, p_rot.pose.position.z),
        # (p_rot.pose.orientation.x, p_rot.pose.orientation.y, p_rot.pose.orientation.z, p_rot.pose.orientation.w),
        # rospy.Time.now(),
        # "p_rot",
        # p_rot.header.frame_id)

        # ----------------- transform final grasp position to /hand_palm_link -------------------
        self.tf_listener.waitForTransform('odom','/selected_grasp', rospy.Time(), rospy.Duration(4.0))
        p_final = self.tf_listener.transformPose('odom', p_rot)
        p_final.pose.position.x = p_final.pose.position.x -0.025 

        self.br.sendTransform((p_final.pose.position.x, p_final.pose.position.y, p_final.pose.position.z),
        (p_final.pose.orientation.x, p_final.pose.orientation.y, p_final.pose.orientation.z, p_final.pose.orientation.w),
        rospy.Time.now(),
        "/p_final",
        p_final.header.frame_id)
        rospy.loginfo(p_final)

        # set the way_point wrt. grasp pose after rotation
        way_point = geometry_msgs.msg.PoseStamped()
        way_point.header.frame_id = "/p_final"
        way_point.pose.position.x = 0
        way_point.pose.position.y = 0
        way_point.pose.position.z = -0.15
        way_point.pose.orientation.w = 1 
        rospy.loginfo(way_point)

        # Transform the pose from base_link to hand_palm_link
        self.tf_listener.waitForTransform('odom','/p_final', rospy.Time(), rospy.Duration(4.0))
        way_point_map = self.tf_listener.transformPose('odom', way_point)

        self.br.sendTransform((way_point_map.pose.position.x, way_point_map.pose.position.y, way_point_map.pose.position.z),
        (way_point_map.pose.orientation.x, way_point_map.pose.orientation.y, way_point_map.pose.orientation.z, way_point_map.pose.orientation.w),
        rospy.Time.now(),
        "/way_point",
        way_point_map.header.frame_id)   

        # ----------------- transform intermidate position to /hand_palm_link -------------------
        rospy.loginfo("go to intermidate position!")  
        self.whole_body_light.set_joint_value_target(way_point_map)
        self.whole_body_light.go()
        rospy.loginfo("done")
        rospy.sleep(2.0)

        
        # ----------------- transform final grasp position to /hand_palm_link -------------------
        rospy.loginfo("go to grasp position")
        rospy.loginfo(p_final)
        self.whole_body_light.set_joint_value_target(p_final)
        self.whole_body_light.go()
        rospy.loginfo("done")
        rospy.sleep(self.wait)

        rospy.loginfo("grasp!")
        self.gripper.set_joint_value_target("hand_motor_joint", 0.20)
        self.gripper.go()

        # rospy.loginfo("arm back")
        # self.arm.set_named_target("neutral")
        # self.arm.go()
        # rospy.loginfo("GRASP DONE!")
        # rospy.sleep(self.wait)
       
if __name__ == '__main__':
    try:
        grasp_to_pose = GraspToPose()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass