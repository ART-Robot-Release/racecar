cmake_minimum_required(VERSION 2.8.3)
project(openni_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY model DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY test  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} USE_SOURCE_PERMISSIONS)
if (CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
  if("$ENV{ROS_DISTRO}" STRGREATER "jade")
    add_rostest(test/sample_kobuki.test)
  endif()
endif ()
