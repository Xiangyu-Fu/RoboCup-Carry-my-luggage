# Tutorial 4 - Vision

### Goal:
* Learn how to use robot's camera
* Learn how to work with pointclouds
* Learn how to use simple image classifiers

### Task 1 - Setting up Darknet

The first task is getting YOLO3 running on your computer. YOLO stands for 
'You Only Look Once' and is described in this [YOLO9000_Better_Faster__Stronger.pdf](https://arxiv.org/pdf/1612.08242.pdf.pdf). It is one of the easiest to use image classifiers with ros integration.

* First, Create new workspace

* Visit [darknet](https://github.com/leggedrobotics/darknet_ros) and follow the instructions
in the readme file to clone the project.

* Create a new package 'object_detection' and add a launch file that feeds the 2d RGB camera image of the robots head
camera into the YOLO. 
  - Note: you can find the robots camera topic via 'rostopic list'

* Launch empty gazebo world provided in the package object_detection_world

* Tilt the robot head by calling its trajectory controller to look at the table.
  - Note: Use the hsrb 'head_trajectory_controller' or tiago 'head_controller'

* Place objects on the tilted table
  - Note: If you download new models place them in the packages 'models' folder

* Finally, launch your 'object_detection' package and check if YOLO detects some of the objects.
You will also find a new message containing the bounding box of detected objects.
  - Note: Play with the light conditions to improve the detection

### Task 2 - Pointcloud segementation with PCL

The second task is done in the package 'plane_segmentation'. The goal of this package is to segement the pointcloud of the RGB-D camera into the **table plane** and a pointcloud that hold **all objects**.
The goal is to publish two pointclouds: the table pointcloud and the objects pointcloud.

For processing of pointclouds we are gooing to use [PCL](http://wiki.ros.org/perception_pcl?distro=kinectic). 
Many useful c++ examples can be found [online](https://pcl.readthedocs.io/projects/tutorials/en/master/#segmentation)

Explaining concepts such as:
  - Downsampling of pointclouds
  - Segmentations based on Ransac
  - Clustering based on Euclidian distance
  - Filtering of points based on geometric constrains
  - Transformation of pointclouds from one frame into another
You will need to check some of these tutorials to complete the task.

Follow the todos inside the 'plane_segmentation' package to segment the pointcloud and remove all unwanted artifacts such as points beloning to the floor or table legs. The steps are as follows:
Inside plane_segmentation.cpp:
  1) Complete initalize() function
  2) Complete the cloudCallback() function
  3) Complete the preProcessCloud() function
  4) Finish the segmentCloud() function
Inside plane_segmentation_node.cpp:
  1) Initalize the object and set the correct frame and topic names

Note: Whenever you finished a step you can comment out the missing code and publish
your pointcloud to check how the result looks like. This is easier than doing everything at once!

### Task 3 - Pointcloud labeling (2d -> 3d)

The last task takes the 2d bounding boxes of yolo and matches the with the 3d objects inside the object point cloud.

For this we first cluster the object point cloud into blobs of individual objects. 
Then, for each blob we compute it's 3d centroid (x,y,z).
Next, these centoids are transforment into the camera frame and projected into the 2d image plane (u,v).
Finally, we check which bounding box matches best to the projected centroids.

Follow the todos inside 'object_labeling' to complete the task.
Inside object_labeling.cpp:
  1) Complete initalize() function
  2) Complete the callback functions()
  3) Move to the labelObjects() function and work on the todos
  4) Complete the update() function
Inside object_labeling_node.cpp:
  1) Initalize the object and set the correct frame and topic names

Note: Whenever you finished a step (e.g clustering) you can comment out the missing code and publish
your pointcloud to check how the result looks like. This is easier than doing everything at once!

### Task 4 - Execute on real robot

Put the robot in front of a real table, place some object and launch the code.
Does it work (Hint: Sim2Real Gap...). Try to tune the parameters to detect at last
some of the objects.