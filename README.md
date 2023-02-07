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
    # some err will occur, run the command again
    catkin build
```

## 2. Run in the simulation env
```bash
cd catkin_ws
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

```bash
cd catkin_ws
source devel/setup.bash
roslaunch hsrb_task_manager task_manager.launch
```


