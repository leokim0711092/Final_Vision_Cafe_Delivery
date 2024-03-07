#include "coffee_delivery/hole_plate_segmentation.h"

#include <Eigen/Eigen>
#include <string>
#include <vector>

#include "boost/lexical_cast.hpp"

#include "pcl/common/centroid.h"
#include "pcl_conversions/pcl_conversions.h"


namespace Coffee_Deliver {

using std::placeholders::_1;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("hole_plate_segmentation");

HolePlateSegmentation::HolePlateSegmentation(
    rclcpp::Node::SharedPtr node) {
  // Save clock
  clock_ = node->get_clock();

  // cluster_tolerance: minimum separation distance of two objects
  double cluster_tolerance =
      node->declare_parameter<double>("cluster_tolerance", 0.01);

  extract_clusters_.setClusterTolerance(cluster_tolerance);

  // cluster_min_size: minimum size of an object
  int cluster_min_size = node->declare_parameter<int>("cluster_min_size", 50);
  extract_clusters_.setMinClusterSize(cluster_min_size);

  // voxel grid the data before segmenting
//   double leaf_size;
//   leaf_size = node->declare_parameter<double>("voxel_leaf_size", 0.005);
//   voxel_grid_.setLeafSize(leaf_size, leaf_size, leaf_size);

// Subscribe to head camera cloud
    rclcpp::QoS points_qos(10);
    points_qos.best_effort();
    cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "filter_cloud", points_qos,
        std::bind(&HolePlateSegmentation::cloud_callback, this, _1));
// 
    rclcpp::QoS qos(1);
    qos.best_effort();
    voxel_cloud_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "voxel_cloud", qos);

  // segment objects
  segment_.setOptimizeCoefficients(true);
  segment_.setModelType(pcl::SACMODEL_PLANE);
  segment_.setMaxIterations(100);
  segment_.setDistanceThreshold(cluster_tolerance);
}

void HolePlateSegmentation::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    filter_cloud_msg = msg;
      // Convert to point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud =
        std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::fromROSMsg(*msg, *cloud);
    voxel_grid_.setInputCloud(cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    voxel_grid_.filter(*cloud_filtered);
    sensor_msgs::msg::PointCloud2 cloud_msg;
    
    std::cout<< "Hi1233" << std::endl;

    pcl::toROSMsg(*cloud_filtered, cloud_msg);
    voxel_cloud_pub_->publish(cloud_msg);
    
  }

// bool HolePlateSegmentation::segment(
//     const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_input,
//     pcl::PointCloud<pcl::PointXYZRGB> &plate_cloud,
//     pcl::PointCloud<pcl::PointXYZRGB> &hole_cloud
//     ) {

//     RCLCPP_INFO(LOGGER, "object support segmentation starting...");
//     // Convert to point cloud
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
//         RCLCPP_INFO(LOGGER, "Cloud create..");

//     // if(filter_cloud_msg == nullptr) {
//     //     RCLCPP_INFO(LOGGER, "filter colud msg nothing");
//     //     return false;
//     // }
//     RCLCPP_INFO(LOGGER, "Check msg.");

//     // pcl::fromROSMsg(*filter_cloud_msg, *cloud);
//     // RCLCPP_INFO(LOGGER, "filter msg %s", filter_cloud_msg->header.frame_id.c_str());

// //   // process the cloud with a voxel grid
// //     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

// //     std::cout<< "Hi1233" << std::endl;

//     voxel_grid_.setInputCloud(cloud_input);
// //     std::cout<< "Hi" << std::endl;

// //     voxel_grid_.filter(*cloud_filtered);
//     // RCLCPP_DEBUG(LOGGER, "Filtered for transformed Z, now %d points.",
//     //            static_cast<int>(cloud_filtered->points.size()));

// //   // remove support planes
// //   pcl::PointCloud<pcl::PointXYZRGB>::Ptr non_horizontal_planes;
// //   non_horizontal_planes = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
// //   std::vector<pcl::ModelCoefficients::Ptr>
// //       plane_coefficients; // coefs of all planes found
// //   int thresh = cloud_filtered->points.size() / 8;
// //   while (cloud_filtered->points.size() > 500) {
//     // pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
//     // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

// //     // Segment the largest planar component from the remaining cloud
//     // segment_.setInputCloud(cloud);
//     // segment_.segment(*inliers, *coefficients);
// //     // TODO(enhancement): make configurable?
// //     // TODO(enhancement): make this based on "can we grasp object"
// //     if (inliers->indices.size() < static_cast<size_t>(thresh)) {
// //       RCLCPP_DEBUG(LOGGER, "No more planes to remove.");
// //       break;
// //     }

// //     // Extract planar part for message
// //     pcl::PointCloud<pcl::PointXYZRGB> plane;
// //     pcl::ExtractIndices<pcl::PointXYZRGB> extract;
// //     extract.setInputCloud(cloud_filtered);
// //     extract.setIndices(inliers);
// //     extract.setNegative(false);
// //     extract.filter(plane);

//   RCLCPP_INFO(LOGGER, "object support segmentation done processing.");
//   return true;
// }


// pcl::PointCloud<pcl::PointXYZRGB>::Ptr HolePlateSegmentation::Segmentation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud) {
//     // Process the input_cloud, perform segmentation, etc.

//     // For demonstration, let's just create a new point cloud with the same points as input_cloud
//     RCLCPP_INFO(LOGGER, "object support segmentation starting...");

//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//     RCLCPP_INFO(LOGGER, "Input");

//     voxel_grid_.setInputCloud(input_cloud);
//         RCLCPP_INFO(LOGGER, "Output");
//     voxel_grid_.setFilterFieldName("x");
//     voxel_grid_.setFilterLimits(-0.7, 0.2);
//     voxel_grid_.filter(*segmented_cloud);

//     // Return the segmented_cloud
//     return segmented_cloud;
// }

} // namespace simple_grasping

