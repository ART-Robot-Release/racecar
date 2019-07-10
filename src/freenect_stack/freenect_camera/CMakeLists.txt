cmake_minimum_required(VERSION 2.8.3)
project(freenect_camera)
find_package(catkin REQUIRED
             COMPONENTS
             camera_info_manager
             diagnostic_updater
             dynamic_reconfigure
             image_transport
             nodelet
             roscpp
             sensor_msgs
             pluginlib )

# Resolve system dependency on libfreenect, which does not provide a
# CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFREENECT REQUIRED libfreenect)
pkg_check_modules(LOG4CXX REQUIRED liblog4cxx)

find_library(LIBFREENECT_LIBRARY
  NAMES freenect
  PATHS ${LIBFREENECT_LIBRARY_DIRS}
)

# auto-generate dynamic reconfiguration GUI before catkin_package()
generate_dynamic_reconfigure_options(cfg/Freenect.cfg)

# resolve boost thread references
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIR}
                    ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
                    ${LIBFREENECT_INCLUDE_DIRS}
                    ${LOG4CXX_INCLUDE_DIRS})

# build the node and nodelet
add_executable(freenect_node src/nodes/freenect_node.cpp)
target_link_libraries(freenect_node
                      ${catkin_LIBRARIES}
                      ${LIBFREENECT_LIBRARY}
                      ${Boost_LIBRARY})
add_dependencies(freenect_node ${PROJECT_NAME}_gencfg)

add_library(freenect_nodelet src/nodelets/driver.cpp )
target_link_libraries(freenect_nodelet
                      ${catkin_LIBRARIES}
                      ${LIBFREENECT_LIBRARY}
                      ${Boost_LIBRARY}
                      ${LOG4CXX_LIBRARIES})
add_dependencies(freenect_nodelet ${PROJECT_NAME}_gencfg)

catkin_package(DEPENDS
               libfreenect
               CATKIN_DEPENDS
               camera_info_manager
               diagnostic_updater
               dynamic_reconfigure
               image_transport
               nodelet
               roscpp
               sensor_msgs)

# install the node and nodelet
install(TARGETS freenect_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        COMPONENT main)

install(TARGETS freenect_nodelet
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# add xml file
install(FILES freenect_nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
