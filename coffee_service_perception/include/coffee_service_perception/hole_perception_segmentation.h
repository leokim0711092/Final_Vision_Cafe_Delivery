#ifndef COFFEE_SERVICE_PERCEPTION__HOLE_PERCEPTION_SEGMENTATION_H_
#define COFFEE_SERVICE_PERCEPTION__HOLE_PERCEPTION_SEGMENTATION_H_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>
#include <Eigen/Eigen>
#include <sstream>

#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "geometry_msgs/msg/detail/pose__struct.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point.hpp"


#include "pcl/common/io.h"
#include "pcl/common/centroid.h"
#include <pcl/common/common.h>

#include "pcl/filters/voxel_grid.h"
#include <pcl/filters/voxel_grid.h>
#include "pcl/filters/extract_indices.h"


#include <pcl/point_cloud.h> 
#include "pcl/point_types.h"
#include "pcl_ros/transforms.hpp"

#include "pcl/segmentation/extract_clusters.h"
#include "pcl/segmentation/sac_segmentation.h"

#include <pcl/ModelCoefficients.h>

#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

namespace Coffee_Service_Perception
{
/**
 *  @brief Segement Hole and Plate
 */  
class Segmentation
{

public:
  /**
   *  @brief Constructor, loads pipeline using ROS parameters.
   */
  explicit Segmentation (rclcpp::Node::SharedPtr node);
  
  //Segment the plate and hole
  bool segment(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_input, 
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr plate_cloud, 
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr hole_cloud,
                std::vector<std::string> & process_descriptions);
  

private:

  pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid_;

  pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> segment_plate; 

  rclcpp::Clock::SharedPtr clock_;
};

}  

#endif 



