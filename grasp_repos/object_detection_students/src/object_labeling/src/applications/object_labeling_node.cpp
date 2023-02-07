#include <object_labeling/object_labeling.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plane_segmentation");
  ros::NodeHandle nh;
  ROS_INFO_STREAM("Starting object_labeling_node");

  //#>>>>TODO: Set the correct topic names and frames
  std::string objects_cloud_topic = "/segmentation/objects_cloud"; // = "?"
  std::string camera_info_topic = "/hsrb/head_rgbd_sensor/rgb/camera_info";
  std::string camera_frame = "/head_rgbd_sensor_link";

  ObjectLabeling labeling(
    objects_cloud_topic,
    camera_info_topic,
    camera_frame);

  // Init
  if(!labeling.initalize(nh))
  {
    return -1;
    ROS_INFO_STREAM("Error init ObjectLabeling");
  }

  // Run
  ros::Rate rate(30);
  while(ros::ok())
  {
    labeling.update(ros::Time::now());
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
