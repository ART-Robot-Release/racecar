cmake_minimum_required(VERSION 2.8.3)
project(rgbd_launch)

find_package(catkin)

catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)

install(PROGRAMS scripts/hydro-upgrade-notice
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)
  roslaunch_add_file_check(launch)
  roslaunch_add_file_check(test)
  add_rostest(test/camera.test)
endif()
