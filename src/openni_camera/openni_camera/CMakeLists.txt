cmake_minimum_required(VERSION 2.8.3)
project(openni_camera)

find_package(catkin COMPONENTS camera_info_manager dynamic_reconfigure image_transport nodelet roscpp sensor_msgs REQUIRED)
find_package(Boost COMPONENTS system filesystem thread REQUIRED)

find_package(PkgConfig)
pkg_check_modules(PC_LIBOPENNI REQUIRED libopenni)
pkg_check_modules(PC_LIBUSB REQUIRED libusb-1.0)

find_package(Log4cxx QUIET)
if(NOT LOG4CXX_LIBRARIES)  
    # backup plan, hope it is in the system path
  find_library(LOG4CXX_LIBRARIES log4cxx)
endif()
if(NOT LOG4CXX_LIBRARIES)
  message(ERROR "log4cxx required")
endif()

generate_dynamic_reconfigure_options(cfg/OpenNI.cfg)

catkin_package(
  INCLUDE_DIRS include  
  LIBRARIES openni_nodelet openni_driver  
  CATKIN_DEPENDS camera_info_manager dynamic_reconfigure image_transport nodelet roscpp
  DEPENDS Boost PC_LIBOPENNI
)

include_directories(include 
                    ${PC_LIBOPENNI_INCLUDE_DIRS} 
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS} 
)
add_definitions(${PC_LIBOPENNI_CFLAGS_OTHER})
 

# OpenNI Driver standalone library
add_library(openni_driver src/openni_driver.cpp
                          src/openni_device.cpp
                          src/openni_exception.cpp
                          src/openni_device_primesense.cpp
                          src/openni_device_kinect.cpp
                          src/openni_device_xtion.cpp
                          src/openni_device_oni.cpp
                          src/openni_image_yuv_422.cpp
                          src/openni_image_bayer_grbg.cpp
                          src/openni_image_rgb24.cpp
                          src/openni_ir_image.cpp
                          src/openni_depth_image.cpp
)

target_link_libraries(openni_driver ${catkin_LIBRARIES}
                                    ${Boost_LIBRARIES} 
                                    ${PC_LIBOPENNI_LIBRARIES}
                                    ${PC_LIBUSB_LIBRARIES}
                                    ${LOG4CXX_LIBRARIES}
)

# ROS nodelets library
add_library(openni_nodelet src/nodelets/driver.cpp)
target_link_libraries(openni_nodelet openni_driver ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(openni_nodelet ${PROJECT_NAME}_gencfg)

# Node version of ROS driver
add_executable(openni_node src/nodes/openni_node.cpp)
target_link_libraries(openni_node openni_driver ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(openni_node ${PROJECT_NAME}_gencfg)

install(TARGETS openni_driver openni_nodelet openni_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# add xml file
install(FILES openni_nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


