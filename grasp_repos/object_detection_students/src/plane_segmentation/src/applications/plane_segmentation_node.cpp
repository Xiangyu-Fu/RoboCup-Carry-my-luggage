#include <plane_segmentation/plane_segmentation.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plane_segmentation");
  ros::NodeHandle nh;
  
  // Set the correct topic name of the robot
  std::string pointcloud_topic_name = "/hsrb/head_rgbd_sensor/depth_registered/rectified_points";

  // Set the name of a frame on the floor/ground of the robot (height=0)
  std::string base_frame_name = "base_footprint";

  // construct the object
  PlaneSegmentation segmentation(
    pointcloud_topic_name, 
    base_frame_name);
  
  // initialize the object
  if(!segmentation.initalize(nh))
  {
    ROS_ERROR_STREAM("Error init PlaneSegmentation");
    return -1;
  }

  // update the processing
  ros::Rate rate(0.5);
  while(ros::ok())
  {
    segmentation.update(ros::Time::now());
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
