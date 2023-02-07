# 1. create workspace

```
    mkdir -p robo_ros/final_ws/src
    cd final_ws
    catkin_make
```
# 2. clone darknet package
```
    cd src
    git clone --recursive git@github.com:leggedrobotics/darknet_ros.git
    cd ../
    catkin_make -DCMAKE_BUILD_TYPE=Release

    or

    catkin build darknet_ros -DCMAKE_BUILD_TYPE=Release
```
# 3. clone and build GPD package
```
    cd src
    git clone https://github.com/atenpas/gpd
    cd gpd
    mkdir build && cd build
    cmake ..
    make -j

    sudo make install # install GPD as a library
    
    # back to src folder
    cd ~/robo_ros/final_ws/src/
    git clone https://github.com/atenpas/gpd_ros
    cd .. #back to ws
    source devel/setup.bash
    catkin_make

```
If building the package does not work, try to modify the compiler flags, `CMAKE_CXX_FLAGS`, in the file CMakeLists.txt.

# Modify directory
modify directory in 
[cfg/ros_eigen_params.cfg](cfg/ros_eigen_params.cfg)

gpd.launch



# parameter tuning
 in [cfg/eigen_params.cfg](cfg/eigen_params.cfg)

 ## 1. improve the number of grasps found
 **workspace**: [minX, maxX, minY, maxY, minZ, maxZ], centered at the origin of the point cloud frame (as small as possible)
 **num_sample**: the number of samples that are drawn from the point cloud to detect grasp (as large as possible)
 ## 2. improve runtime
 **num_threads**: the number of (physical) CPU cores available. 
 ### check CPUs:
 ```
    grep -c ^processor /proc/cpuinfo
    or
    lscpu --extended
    or 
    cat /sys/devices/system/cpu/present
 ```
 ## 3. camera position
 **camera_position**: it enables PCL to estimate the correct normals direction (which is to point toward the camera). Alternatively, using the ROS wrapper, multiple camera positions can be provided.

 ## 4. filter_approach_direction

# run CMake with additional arguments
to check all possible CMake options:
in `build` folder
```
ccmake ..
```
## darknet
```
git clone --recursive https://github.com/leggedrobotics/darknet_ros.git
```

## launch environment
```

roslaunch object_detection hsrb.launch

rosrun object_detection grasptopose.py

roslaunch gpd_ros gpd.launch 

```
## !!

remember to modify the config file in your `gpd` folder, e.g.,
`<path_to_gpd>/cfg/ros_eigen_params.cfg`
modify the path in the ROS launch file that points to the
config file that you changed in the previous step

# TODO
1. modify cfg file: hand measurements



## new commands
roslaunch object_detection hsrb_mode.launch
rosrun object_detection grasptopose.py
roslaunch gpd_ros gpd.launch
