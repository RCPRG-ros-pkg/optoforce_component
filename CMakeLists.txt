cmake_minimum_required(VERSION 2.8.3)
project(optoforce_component)


find_package(catkin REQUIRED COMPONENTS
  roscpp rtt_ros rtt_roscomm barrett_hand_controller_msgs geometry_msgs sensor_msgs cmake_modules optoforce_can can_driver)

find_package(Eigen REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${BOOST_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

orocos_component(optoforce_component src/optoforce_component.cpp)

target_link_libraries(optoforce_component ${catkin_LIBRARIES})

orocos_generate_package()

