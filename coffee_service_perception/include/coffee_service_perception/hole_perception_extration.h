#ifndef COFFEE_SERVICE_PERCEPTION__HOLE_PERCEPTION_EXTRATION_H_
#define COFFEE_SERVICE_PERCEPTION__HOLE_PERCEPTION_EXTRATION_H_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>
#include <Eigen/Eigen>
#include <sstream>

#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "geometry_msgs/msg/detail/pose__struct.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "pcl/common/io.h"
#include "pcl/common/centroid.h"
#include <pcl/common/common.h>

#include "pcl/filters/extract_indices.h"
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/point_cloud.h> 
#include "pcl/point_types.h"
#include "pcl_ros/transforms.hpp"

#include "pcl/segmentation/extract_clusters.h"
#include "pcl/segmentation/sac_segmentation.h"

#include <pcl/surface/convex_hull.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

namespace Coffee_Service_Perception
{
/**
 *  @brief Segement Hole and Plate
 */  
class HoleExtration
{

public:
  /**
   *  @brief Constructor, loads pipeline using ROS parameters.
   */
  explicit HoleExtration (rclcpp::Node::SharedPtr node);
  
// Extract the hole
  void hole_extration(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & plate_cloud, 
                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr &hole_cloud, 
                      std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& cloud_vector,
                      std::vector<std::string> & process_descriptions);

  // Rearrage the order of cloud in vector depending on the distance
  void hole_number(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& cloud_vector,
                    std::vector<std::string> & process_descriptions);

  // Project the border of hole to 2D plane, then use convex vull to filter border out
  void hole_projection(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& cloud_vector, 
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr &hole_cloud,
                       std::vector<std::string> & process_descriptions);


private:
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> outliers_filter;
  rclcpp::Clock::SharedPtr clock_;
};

}  

#endif 
