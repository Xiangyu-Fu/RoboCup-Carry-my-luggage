cmake_minimum_required(VERSION 3.0.2)
project(hsrb_task_manager)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
catkin_install_python(PROGRAMS
  scripts/task_manager.py
  scripts/state_machine.py
  scripts/task_manager_ex.py
  scripts/test.py
  scripts/move_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch USE_TEST_DEPENDENCIES)
endif()
