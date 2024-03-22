#ifndef COFFEE_SERVICE_PERCEPTION__HOLE_PERCEPTION_ESTIMATE_CENTER_H_
#define COFFEE_SERVICE_PERCEPTION__HOLE_PERCEPTION_ESTIMATE_CENTER_H_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>
#include <sstream>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/detail/pose__struct.hpp"
#include "rclcpp/logging.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/detail/marker__struct.hpp"
#include "visualization_msgs/msg/detail/marker_array__struct.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "pcl/common/io.h"
#include "pcl/common/centroid.h"
#include <pcl/common/common.h>

#include <pcl/point_cloud.h> 
#include "pcl/point_types.h"
#include "pcl_ros/transforms.hpp"


namespace Coffee_Service_Perception
{
/**
 *  @brief Class that process a point cloud to get the hole position relative to base link.
 */  
class EstimateCenter
{

public:
  /**
   *  @brief Constructor, loads pipeline using ROS parameters.
   */
  explicit EstimateCenter(rclcpp::Node::SharedPtr node);

  
  // Function to estimate circle parameters using least squares fitting
  void estimateCircleParams(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& cloud_vector, 
                            std::vector<float>& xc, std::vector<float>& yc, 
                            std::vector<float>& zc, std::vector<float>& r, std::vector<std::string> & process_descriptions); 

  // Mark the center of find
  void marker(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & plate_cloud, 
                std::vector<std::string> & process_descriptions,
                std::vector<float>& xc, std::vector<float>& yc, std::vector<float>& zc, 
                std::vector<float>& r, geometry_msgs::msg::Point & plate_center,
                visualization_msgs::msg::MarkerArray & marker_array); 


private:
  rclcpp::Clock::SharedPtr clock_;
};

}  

#endif 

