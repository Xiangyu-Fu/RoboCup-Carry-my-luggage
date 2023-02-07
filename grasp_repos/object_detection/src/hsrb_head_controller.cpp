/// @brief Copyright (C) 2016 Toyota Motor Corporatio
#include <controller_manager_msgs/ControllerState.h>
#include <controller_manager_msgs/ListControllers.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "hsrb_head_controller");

  // initalize ROS publisher
  ros::NodeHandle n;
  ros::Publisher pub1 = n.advertise<trajectory_msgs::JointTrajectory>("/hsrb/head_trajectory_controller/command", 10);
  ros::Publisher pub = n.advertise<trajectory_msgs::JointTrajectory>("/hsrb/arm_trajectory_controller/command", 10);
  // wait to establish connection between the controller
  while (pub.getNumSubscribers() == 0) {
    ros::Duration(0.1).sleep();
  }

  // make sure the controller is running
  ros::ServiceClient client = n.serviceClient<controller_manager_msgs::ListControllers>(
      "/hsrb/controller_manager/list_controllers");
  controller_manager_msgs::ListControllers list_controllers;
  bool running = false;
  while (running == false) {
    ros::Duration(0.1).sleep();
    if (client.call(list_controllers)) {
      for (unsigned int i = 0; i < list_controllers.response.controller.size(); i++) {
        controller_manager_msgs::ControllerState c = list_controllers.response.controller[i];
        if (c.name == "head_trajectory_controller" && c.state == "running") {
          running = true;
        }
        if (c.name == "arm_trajectory_controller" && c.state == "running") {
          running = true;
        }
      }
    }
  }

  // fill ROS message
  trajectory_msgs::JointTrajectory traj1;

  traj1.joint_names.push_back("head_pan_joint");
  traj1.joint_names.push_back("head_tilt_joint");

  traj1.points.resize(1);

  traj1.points[0].positions.resize(2);
  traj1.points[0].positions[0] = -0.0;
  traj1.points[0].positions[1] = -0.55;
  traj1.points[0].velocities.resize(2);
  for (size_t i = 0; i < 2; ++i) {
    traj1.points[0].velocities[i] = 0.0;
  }
  traj1.points[0].time_from_start = ros::Duration(3.0);
  
  
    // fill ROS message
  trajectory_msgs::JointTrajectory traj;

  traj.joint_names.push_back("arm_lift_joint");
  traj.joint_names.push_back("arm_flex_joint");
  traj.joint_names.push_back("arm_roll_joint");
  traj.joint_names.push_back("wrist_flex_joint");
  traj.joint_names.push_back("wrist_roll_joint");

  traj.points.resize(1);

  traj.points[0].positions.resize(5);
  traj.points[0].positions[0] = 0.0;
  traj.points[0].positions[1] = 0.0;
  traj.points[0].positions[2] = -1.5;
  traj.points[0].positions[3] = -2.5;
  traj.points[0].positions[4] = 0.0;
  traj.points[0].velocities.resize(5);
  for (size_t i = 0; i < 5; ++i) {
    traj.points[0].velocities[i] = 0.0;
  }
  traj.points[0].time_from_start = ros::Duration(3.0);

  // publish ROS message
  pub.publish(traj);

  // publish ROS message
  pub1.publish(traj1);

  ros::spinOnce();
  ros::Duration(3.0).sleep();

  return 0;
}