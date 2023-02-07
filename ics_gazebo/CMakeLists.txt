cmake_minimum_required(VERSION 2.8.3)
project(ics_gazebo)
find_package(catkin REQUIRED COMPONENTS)
catkin_package()

foreach(dir config launch Media models scripts worlds)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

