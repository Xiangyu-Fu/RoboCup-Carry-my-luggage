# Robotcup final project "Carry my luggage"

In this project, the robot is asked
to follow the command from human to pick up
a specified object, carry it, follow the human to
another room, and eventually, hand the object over
to the human again. To complete these tasks, the
robot has to detect human posture, grasp objects,
follow human.


![image](https://user-images.githubusercontent.com/54738414/232415961-cf3f7eb2-f69b-4913-844c-c48c55b65f02.png)

Watch the video:

[![Watch the video](https://i9.ytimg.com/vi_webp/M-j3pvWNksI/mq1.webp?sqp=CKjV1qMG-oaymwEmCMACELQB8quKqQMa8AEB-AH-CYAC0AWKAgwIABABGFggZShAMA8%3D&rs=AOn4CLB8AKelkInYAicwfLljrCPukb485w&retry=4)](https://youtu.be/M-j3pvWNksI)

## 1. Setup the env
```bash
cd src/grasp_repos/gpd
mkdir build && cd build
cmake ..
make -j 
# it will take several mins
sudo make install

cd ../../../..

catkin build
# some err will occur, run the build again
catkin build
```

## 2. Run in the simulation env
Start the simulation env.
```bash
cd your_workspace
source devel/setup.bash
roslaunch hsrb_task_manager sim_env.launch
```

Open a new terminal
```bash
cd catkin_ws
source devel/setup.bash
roslaunch hsrb_task_manager task_manager.launch use_sim:=true
```

## 3. Run in the real robot
rviz(optional)
```bash
rosrun rviz rviz  -d `rospack find hsrb_common_launch`/config/hsrb_display_full_hsrc.rviz
```
Before run the robot, you need to localize the robot correctly, you can use Rviz or amcl to localize the robot.
```bash
roslaunch hsrb_amcl amcl_hsrb_sim.launch
```

Once the location of robot is settle down, you can start the state machine.

```bash
cd your_workspace
source devel/setup.bash 
roslaunch hsrb_task_manager task_manager.launch 
```

If you want to run the nodes separately (only in `hsrb_mode`), set the `/use_distributed` param to `true`.
```bash
roslaunch hsrb_task_manager task_manager.launch use_distributed:=true 
```
After that you need to run the point cloud plane segmentaton and gpd grasp pose generation node in another computer.


## 4. Trouble shooting
If the hri_fullbody not work correctly, please use source command again.
```bash
source devel/setup.bash
```
