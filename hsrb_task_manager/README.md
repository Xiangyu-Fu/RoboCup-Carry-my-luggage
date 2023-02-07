# robocup_final_project
robocup_final_proj

## 1. TODO & progress

###  NOW all passed the build
- [DONE] Tensorflow version not correct --> Docker 
- tf tree may have bug

### 1.1 Robot control
$ ihsrb
In []: omni_base.<Tab>
The available choices are as follows:

In []: omni_base.
 cancel_goal()
 create_follow_trajectory_goal()
 create_go_pose_goal()
 execute()
 follow_trajectory()
 get_pose()
 go()
 go_abs()
 go_pose()
 go_rel()
 is_moving()
 is_succeeded()
 move()
 pose

 whole_body.move_to_neutral()


rosrun rviz rviz  -d `rospack find hsrb_common_launch`/config/hsrb_display_full_hsrc.rviz

rosbag record -a -o hsrb.bag

rosbag record -o test.bag -e /hsrb/head_rgbd_sensor/rgb/.* /tf /tf_static /hsrb/head_rgbd_sensor/depth_registered/.*

rosbag record -o test.bag /tf /tf_static /hsrb/head_rgbd_sensor/rgb/camera_info /hsrb/head_rgbd_sensor/rgb/image_raw /hsrb/head_rgbd_sensor/rgb/image_raw/compressed

rosbag record -o cloudpoint.bag /hsrb/head_rgbd_sensor/depth_registered/rectified_points

rosbag record -o test.bag /tf /tf_static /hsrb/head_center_camera/camera_info /hsrb/head_center_camera/image_raw /hsrb/head_center_camera/image_raw/compressed

/hsrb/head_center_camera/image_raw

rosbag record -o cmd_vel.bag -e /hsrb/command_velocity
rosbag record -o track.bag -e 
## 2. Issues when building workspace
### venv 

https://www.liaoxuefeng.com/wiki/1016959663602400/1019273143120480

Create a virtual environment
```bash
virtualenv ENV_NAME --python=python3.x
```

virtualenv tf_env --python=python3.6

virtualenv -p /usr/bin/python3.7 py37

virtualenv penv --system-site-packages --python=python3.7

Venv
---
Activate the venv
```bash
source tf_venv/bin/activate
```

Deactivate the venv
```bash
deactivate
```


tf-pose-estimation (https://github.com/koide3/tf-pose-estimation)
---

### llvm-config required for installing llvmlite
sudo apt install llvm-10
sudo update-alternatives --install /usr/bin/llvm-config llvm-config /usr/bin/llvm-config-10 50


sudo apt install python3.6-distutils


cd tf_pose/pafprocess
swig -python -c++ pafprocess.i && python3 setup.py build_ext --inplace

cd ../..
sudo python3 setup.py install


pip install opencv-python

pip3 install TensorFlow==1.15.2

### the latest numba has some tbb version mismatch issues
sudo pip3 install numba==0.45.0

### swig required for building pafprocess
sudo apt install swig

Debug
--- 
**error**: command 'x86_64-linux-gnu-g++' failed: No such file or directory
```
sudo apt-get install python3.6-dev
sudo apt-get install build-essential
```


## 3. configure libs to fit local env
### 3.1 establish env

- python 3.7.16
- tensorflow-gpu 1.15.5

To install the env please use the following commands:
```
pip install "tensorflow-gpu<2"
```


### 3.2 run test env
some useful command
```
cd catkin_ws
source devel/setup.bash

source ~/venv/bin/activate
```

---
First step, open roscore
```
roscore
```

---
Set `use_sim_time` to `true` and start visualization:
```
rosparam set use_sim_time true

cd /tmp
wget https://github.com/koide3/monocular_person_following/raw/master/rviz/monocular_person_following.rviz
rviz -d monocular_person_following.rviz
```

> NOTICE: Everytime when you open a new roscore(includes roslaunch), you need set the `use_sim_time` to true to run the rviz.

---

> ! using visualenv
> source ~/venv/bin/activate

Launch `usb_cam` and `tf_publishers` in sim mode:
```
roslaunch monocular_person_following start_robot.launch webcam:=true publish_dummy_frames:=true camera_xyz:="0 0 1.4" camera_rpy:="0 0 0" sim:=true
```

Here running `start_robot.launch`, what's it job?
**for test bag**

```
roslaunch hsrb_human_following start_camera.launch webcam:=true publish_dummy_frames:=true camera_xyz:="0 0 1.4" camera_rpy:="0 0 0" sim:=true
```


---

> ! using visualenv

Launch `tf_openpose` and `monocular_person_following`:

```
roslaunch monocular_person_following jetson_person_following.launch camera_name:=/top_front_camera/qhd allow_growth:=true
```
**for test bag**
```
roslaunch hsrb_human_following human_following.launch allow_growth:=true
```


Here running `jetson_person_following.launch`.

---
Play rosbag:

**for default bag**
```
rosbag play --clock mono00.bag
```

**for test bag**
```
cd
cd Documents/
rosbag play --clock test.bag
```

### 3.4 new package

catkin_create_pkg hsrb_human_following std_msgs rospy roscpp

roslaunch hsrb_human_following test.launch
roslaunch hsrb_human_following start_robot.launch webcam:=true publish_dummy_frames:=true



### bugs
```co
Warning: TF_REPEATED_DATA ignoring data with redundant timestamp for frame wrist_flex_link at time 1672400442.715247 according to authority unknown_publisher
         at line 278 in /tmp/binarydeb/ros-noetic-tf2-0.7.6/src/buffer_core.cpp
```

###Potential Movement topic

# 3. real robot

``` bash
hsrb_mode

rosrun rviz rviz  -d `rospack find hsrb_common_launch`/config/hsrb_display_full_hsrc.rviz

source ~/venv/bin/activate

roslaunch hsrb_human_following start_camera.launch webcam:=true publish_dummy_frames:=true camera_xyz:="0 0 1.4" camera_rpy:="0 0 0" sim:=true

roslaunch hsrb_human_following human_following.launch allow_growth:=true
roslaunch hsrb_task_manager task_manager.launch
```

## New commands

``` bash
hsrb_mode
roslaunch hsrb_task_manager task_manager.launch
```

Now using navigation to control the robot.

## map creation

sudo cp ~/Documents/map.pgm /etc/opt/tmc/robot/conf.d/lab-map

sudo scp /home/athome/Documents/map.pgm administrator@hsrb.local:~/Documents

## amcl

$ roslaunch hsrb_amcl amcl_hsrb_sim.launch

# TODO

1. Integrate the Grasp package, now it pass the compiling 
2. Test the grasp repo in the real robot
3. Fix the navigation bug and get the way point
4. Test the whole pipeline whitout the pointing part


