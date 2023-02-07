#include <object_labeling/object_labeling.h>

ObjectLabeling::ObjectLabeling(
    const std::string& objects_cloud_topic_, 
    const std::string& camera_info_topic,
    const std::string& camera_frame) :
  is_cloud_updated_(false),
  has_camera_info_(false),
  objects_cloud_topic_(objects_cloud_topic_),
  camera_info_topic_(camera_info_topic),
  camera_frame_(camera_frame),
  K_(Eigen::Matrix3d::Zero())
{
}

ObjectLabeling::~ObjectLabeling()
{
}

bool ObjectLabeling::initalize(ros::NodeHandle& nh)
{
  //#>>>>TODO: subscribe to objects pointcloud published by the plane_segmentation_node [DONE]
  //#>>>>TODO: subscribe to bounding boxes from yolo (object_labeling_node)[DONE]
  //#>>>>TODO: subscribe to camera info from robot to obtain the camera matrix K[DONE]
  object_point_cloud_sub_ = nh.subscribe<sensor_msgs::PointCloud2> (objects_cloud_topic_, 10, &ObjectLabeling::cloudCallback, this);
  object_detections_sub_ = nh.subscribe<darknet_ros_msgs::BoundingBoxes> ("/darknet_ros/bounding_boxes", 10, &ObjectLabeling::detectionCallback, this); // topic name?
  camera_info_sub_ = nh.subscribe<sensor_msgs::CameraInfo> (camera_info_topic_, 10, &ObjectLabeling::cameraInfoCallback, this);


  //#>>>>TODO: publish the labled objects as PointCloudl type (see typedefs in header)
  labeled_object_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2> ("/clustered_object_cloud", 10);

  // publish the LABELED object names as visulaization marker (http://wiki.ros.org/rviz/DisplayTypes/Marker)
  text_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/text_markers", 1);
  
  
  //#>>>>TODO: publish object position 3d
  // obj_3d_pub_ = nh.advertise<geometry_msgs::PointStamped>("/segmentation/point3D",100);
  
  // init internal pointclouds for processing (again pcl uses pointers)
  object_point_cloud_.reset(new PointCloud);    // holds unlabled object point cloud
  labeled_point_cloud_.reset(new PointCloudl);  // holds labled object point cloud

  //#>>>>TODO: setup a mapping from class names the ones given by yolo (see yolo bounding_boxes message for classes)
  //#>>>>TODO: to lables / ids (number from 1 to n) used by the pointcloud
  //#>>>>Note: We will use 0 as 'unkown' type 
  dict_["bottle"] = 1;
  dict_["cell phone"] = 2;
  dict_["bowl"] = 3;
  dict_["cup"] = 4;
  dict_["sports ball"] = 5;


  ros::Duration(5).sleep();
  
  // ... bananna, cup, apple, ...
  ROS_INFO("ObjectLabeling::initalize() done");
  return true;
}

void ObjectLabeling::update(const ros::Time& time)
{
  // camera info and point cloud available
  if(is_cloud_updated_ && has_camera_info_)
  {
    is_cloud_updated_ = false;

    // label the objects in pointcloud based on 2d bounding boxes 
    if(!labelObjects(object_point_cloud_, labeled_point_cloud_))
      return;

    //#>>>>TODO: publish labeled_point_cloud_ to ros
    labeled_object_cloud_pub_.publish(labeled_point_cloud_);

    //#>>>>TODO: publish text_markers_ to ros
    text_marker_pub_.publish(text_markers_);

  }
}

bool ObjectLabeling::labelObjects(CloudPtr& input, CloudPtrl& output)
{
  //#>>>>GOAL: Split input pointcloud into seperate blobs, compute centroid,
  //#>>>>GOAL: project centorid into the camera image and match with bounding box
  //#>>>>GOAL: finally label the pointcloud with object type

  // First we need to cluster the input cloud into seperated clusters,
  // each of them represents an object on the table.

  //#>>>>TODO: Use EuclideanClusterExtraction to seperate the pointcloud into clusters [DONE]
  //#>>>>Hint: https://pcl.readthedocs.io/projects/tutorials/en/master/cluster_extraction.html?highlight=EuclideanClusterExtraction
  //#>>>>Hint: use the euclidean cluster extraction object from the header file
  
  // holds the extracted cluster indices (just a integer for identifiction)
  std::vector<pcl::PointIndices> cluster_indices;

  // extract the clusters
  pcl::EuclideanClusterExtraction <PointT> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setInputCloud (input);
  ec.extract (cluster_indices);

  //#>>>>TODO: Iterate over each cluster and compute its centroid point (= mean)
  //#>>>>TODO: Push the centroid into the vector of centroids
  std::vector<Eigen::Vector3d> centroids;
  for (const auto& cluster : cluster_indices)
  {
    Eigen::Vector3d centroid(0,0,0);
    for (const auto& index : cluster.indices)
    {
      centroid += input->points[index].getVector3fMap().cast<double>();
    }
    centroid /= cluster.indices.size();
    centroids.push_back(centroid);
  }


  // Next we need to find the pixel coordinates of the centroids within the 2d
  // camera image. This projection is handled by the camera matrix
  // First, the centorids need to be transformed from the pointcloud frame into the
  // camera frame.

  //#>>>>TODO: Get the homogenous transformation matrix of the base frame with respect
  //#>>>>TODO: to the camera frame.
  //#>>>>Hint: look up the transformation through the tf tree: tfListener_.lookupTransform(...)
  //#>>>>TODO: Convert the tf::StampedTransform into an Eigen::Affine3d
  //#>>>>Hint: tf::transformTFToEigen(...) can do the job
  Eigen::Affine3d T_base_camera; 
  tf::StampedTransform transform;
  tfListener_.lookupTransform(camera_frame_, "base_footprint", ros::Time(0), transform);
  tf::transformTFToEigen(transform, T_base_camera);

  //#>>>>TODO: Transform the centorids into the camera frame by multiplying them 
  //#>>>>TODO: with the transformation that takes a point in the pointcloud frame and turns it
  //#>>>>TODO: into a point in the camera frame. Do this for all centroids.
  std::vector<Eigen::Vector3d> centroids_camera;
  for (const auto& centroid : centroids) // ?
  {
    Eigen::Vector3d centroid_camera = T_base_camera * centroid;
    centroids_camera.push_back(centroid_camera);
    //ROS_INFO_STREAM("centroid_camera: " << centroid_camera.transpose());
  }

  //#>>>>TODO: Project the transformed centorids into the camera plane by using the camera matrix K
  //#>>>>Hint: Multiplying a 3d vector with the 3x3 camera matrix gives a vector in R^3
  //#>>>>Hint: To get pixel coordinates in R^2 you need to convert them to homogenous 2d coordinates
  //#>>>>Hint: ( = divide by the last component and drop the one in third component.)
  std::vector<Eigen::Vector2d> pixel_centroids; // = ?
  std::vector<Eigen::Vector3d> pos_vector;
  for (int i=0; i<centroids_camera.size(); i++)
  {
    Eigen::Vector3d pos_vector = K_ * centroids_camera[i];
    //ROS_INFO_STREAM("pos_vector: " << pos_vector.transpose());
    Eigen::Vector2d pixel_centroid_pro = pos_vector.head(2)/pos_vector[2];
    pixel_centroids.push_back(pixel_centroid_pro);
  }

  // Now the centorids of each cluster are given as pixel coordinates in the 2d image
  // plane of the camera. What remains is to find the bounding box that matches to each of 
  // those controids.

  //#>>>>TODO: Find the best match between pixel_centroids and detections_
  //#>>>>Hint: Use the euclidian distance between the pixel_centroids and the boundingbox centers
  //#>>>>Hint: For each bounding box find the closest cenroid 
  //#>>>>TODO: If a cluster cant be matched (no bounding boxes left) assign 0 as labe
  

  std::vector<int> assigned_labels(cluster_indices.size(), 0);                  // lables of each centroid
  std::vector<std::string> assigned_classes(cluster_indices.size(), "unknown"); // class names of each centroid

  if(cluster_indices.size() > 0)
  {
    for(size_t i = 0; i < detections_.size(); ++i)
    {
      // get the bounding box we want to find the closest cenroid 
      const darknet_ros_msgs::BoundingBox& bounding_box = detections_[i];

      //#>>>>TODO: For all cenroids compute the distance to the boudning box
      //#>>>>TODO: select the clostes as match and get its index in pixel_centroids
      int match = 0; // = ?
      Eigen::Vector2d  bounding_box_center = Eigen::Vector2d(bounding_box.xmin + bounding_box.xmax, 
                                                            bounding_box.ymin + bounding_box.ymax)/2;
      ROS_INFO_STREAM("bounding_box_center: " << bounding_box_center.transpose());
      for (size_t j = 0; j < pixel_centroids.size(); ++j)
      {
        float dist = (pixel_centroids[j] - bounding_box_center).norm();
        //ROS_INFO_STREAM("centroid " << j << " at " << pixel_centroids[j].transpose() << " has distance " << dist << " to bounding box " << i);
        if (dist < (pixel_centroids[match] - bounding_box_center).norm())
        {
          match = j;
          //ROS_INFO_STREAM("centroid " << j <<" at "<< pixel_centroids[j].transpose() << " is closer to bounding box " << i);
        }
      }
      
      // remember the label of match
      if(dict_.find(bounding_box.Class) != dict_.end())
      {
        ROS_INFO_STREAM("match " << match << " to bounding box " << i << " with class " << bounding_box.Class);
        assigned_labels[match] = dict_[bounding_box.Class]; // set match to defined class index
        assigned_classes[match] = bounding_box.Class;       // set match to class name
        //ROS_INFO_STREAM("Assigned label " << bounding_box.Class << " to cluster " << match << "\n");
      }
    }
  }
 

  // relabel the point cloud
  output->points.clear();
  output->header = input->header;
  PointTl pt;
  int i = 0;
  std::vector<pcl::PointIndices>::const_iterator cit = cluster_indices.begin();
  for(; cit != cluster_indices.end(); ++cit, ++i ) 
  {
    // relabel all the points inside cluster
    std::vector<int>::const_iterator it = cit->indices.begin();
    for(; it != cit->indices.end(); ++it ) 
    {
      PointT& cpt = input->points[*it];
      pt.x = cpt.x;
      pt.y = cpt.y;
      pt.z = cpt.z;
      pt.label = assigned_labels[i];    // Note: To test clustering without the matching stuff just use pt.lable = i
      output->points.push_back( pt );
    }
  }

  // create a text marker that displays the assigned class name (assigned_classes) 
  // at the 3d position of the corresponding centroid
  text_markers_.markers.resize(assigned_classes.size());
  for(size_t i = 0; i < assigned_classes.size(); ++i)
  {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.text = assigned_classes[i];
    marker.pose.position.x = centroids[i](0);
    marker.pose.position.y = centroids[i](1);
    marker.pose.position.z = centroids[i](2) + 0.1;     
    // marker.pose.position.x = centroids.col(i).x();
    // marker.pose.position.y = centroids.col(i).y();
    // marker.pose.position.z = centroids.col(i).z() + 0.1;
    marker.color.a = 1.0;
    marker.scale.z = 0.1;
    marker.id = i;
    marker.header.frame_id = input->header.frame_id;
    marker.header.stamp = ros::Time::now();
    text_markers_.markers[i] = marker;
  }

  return true;
}


void ObjectLabeling::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  // convert to pcl
  is_cloud_updated_ = true;
  //#>>>>TODO: convert to pcl and store in object_point_cloud_ [DONE]
  //#>>>>Hint: pcl::fromROSMsg()
  pcl::fromROSMsg(*msg, *object_point_cloud_);
}

void ObjectLabeling::detectionCallback(const darknet_ros_msgs::BoundingBoxesConstPtr &msg)
{
  //#>>>>TODO: copy the YOLO bounding boxes [DONE]
  detections_ = msg->bounding_boxes;
}

void ObjectLabeling::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
{
  // copy camera info
  has_camera_info_ = true;
  Eigen::Matrix3d K = Eigen::Matrix3d::Zero();

  //#>>>>TODO: copy the 3x3 camera matrix to K_ [DONE]
  //#>>>>Hint: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
  for(size_t i = 0; i < 9; ++i)
  {
    K(i) = msg->K[i];
  }
  K_ = K.transpose();
}