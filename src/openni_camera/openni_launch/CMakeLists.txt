cmake_minimum_required(VERSION 2.8.3)
project(openni_launch)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)

# unit tests are enabled selectively
if (CATKIN_ENABLE_TESTING)
  # check that all launch file dependencies are declared correctly
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
  roslaunch_add_file_check(launch/includes)
endif ()
