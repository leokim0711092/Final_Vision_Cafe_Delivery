#ifndef COFFEE_SERVICE_PERCEPTION__HOLE_PLATE_SEGMENTATION_H_
#define COFFEE_SERVICE_PERCEPTION__HOLE_PLATE_SEGMENTATION_H_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "grasping_msgs/msg/object.hpp"

#include "pcl/common/io.h"
#include "pcl/point_types.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/passthrough.h"

namespace Coffee_Service_Perception
{

/**
 *  @brief Class that segments a point cloud into objects and supporting surfaces.
 */  
class HolePlateSegmentation
{
public:
  /**
   *  @brief Constructor, loads pipeline using ROS parameters.
   *  @param node Node instance to use for accessing parameters.
   */
  explicit HolePlateSegmentation(rclcpp::Node::SharedPtr node);

  /**
   *  @brief Split a cloud into objects and supporting surfaces.
   *  @param cloud The point cloud to segment. This cloud should already be
   *         transformed into a coordinate frame where XY plane is horizontal.
   *  @param objects The vector to fill in with objects found.
   *  @param supports The vector to fill in with support surfaces found.
   *  @param plate_cloud A colored cloud of plate found (if output_clouds).
   *  @param hole_cloud A colored cloud of hole found (if output_clouds).
   */
//   bool segment(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
//                pcl::PointCloud<pcl::PointXYZRGB>& plate_cloud,
//                pcl::PointCloud<pcl::PointXYZRGB>& hole_cloud);

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr Segmentation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud);

private:
//   pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid_;.
  pcl::PassThrough<pcl::PointXYZRGB> voxel_grid_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_cloud_pub_;

  std::shared_ptr<sensor_msgs::msg::PointCloud2> filter_cloud_msg;

  pcl::SACSegmentation<pcl::PointXYZRGB> segment_;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> extract_clusters_;
  pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices_;

  rclcpp::Clock::SharedPtr clock_;
};

}  

#endif 
