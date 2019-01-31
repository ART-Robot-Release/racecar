cmake_minimum_required(VERSION 2.8.3)
project(art_imu)


add_definitions(-std=c++11)
find_package(catkin REQUIRED roscpp sensor_msgs tf cmake_modules)

find_package(catkin REQUIRED COMPONENTS)


include_directories(${catkin_INCLUDE_DIRS})

find_package(Eigen REQUIRED)

catkin_package(
)

add_executable(art_imu
  src/art_imu.cc
)

target_link_libraries(art_imu
  ${catkin_LIBRARIES}
)
