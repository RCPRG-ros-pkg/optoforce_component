cmake_minimum_required(VERSION 2.8.3)
project(optoforce_component)


find_package(catkin REQUIRED COMPONENTS
  roscpp rtt_ros rtt_roscomm geometry_msgs sensor_msgs optoforce_can can_driver)

find_package(Eigen3 REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${BOOST_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

orocos_component(optoforce_component src/optoforce_component.cpp)

target_link_libraries(optoforce_component ${catkin_LIBRARIES})

orocos_generate_package()

