cmake_minimum_required(VERSION 3.5)
project(coffee_delivery)

#find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grasping_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

set(common_depends
  geometry_msgs
  grasping_msgs
  pcl_conversions
  pcl_ros
  rclcpp
  rclcpp_action
  rclcpp_components
  sensor_msgs
  shape_msgs
  tf2_ros
  rosbag2_cpp
  visualization_msgs
)

### Build coffee_delivery library
add_library(coffee_delivery SHARED
	src/hole_plate_segmentation.cpp
)
ament_target_dependencies(coffee_delivery ${common_depends})
target_link_libraries(coffee_delivery ${PCL_LIBRARIES})

### Build basic_hole_perception
add_library(basic_hole_perception SHARED
  src/ros_hole_perception.cpp
)
ament_target_dependencies(basic_hole_perception ${common_depends})
target_link_libraries(basic_hole_perception coffee_delivery ${PCL_LIBRARIES})

rclcpp_components_register_node(basic_hole_perception
  PLUGIN "Coffee_Deliver::BasicHolePerception"
  EXECUTABLE basic_hole_perception_node
)

### Build BagToPCLConversion
add_library(BagToPCLConversion SHARED
  src/bag_to_pcl_conversion.cpp
)
ament_target_dependencies(BagToPCLConversion ${common_depends})
target_link_libraries(BagToPCLConversion ${PCL_LIBRARIES})
rclcpp_components_register_node(BagToPCLConversion
  PLUGIN "Coffee_Deliver::BagToPCLConversion"
  EXECUTABLE BagToPCLConversion_node
)

add_executable(pcl_hole_perception src/pcl_hole_perception.cpp)
target_link_libraries(pcl_hole_perception ${PCL_LIBRARIES})


### Install
install(
  DIRECTORY include/
  DESTINATION include
)


install(
  TARGETS
    coffee_delivery
	basic_hole_perception 
	BagToPCLConversion
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PACKAGE_NAME}
)

install(DIRECTORY
  launch
  models

  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  pcl_hole_perception
  DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(coffee_delivery)
ament_export_dependencies(
  geometry_msgs
  grasping_msgs
  pcl_conversions
  pcl_ros
  rclcpp
  rclcpp_action
  rclcpp_components
  sensor_msgs
  shape_msgs
  tf2_ros
  visualization_msgs
)
ament_package()
