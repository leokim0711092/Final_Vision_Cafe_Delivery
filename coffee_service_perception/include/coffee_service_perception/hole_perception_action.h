#ifndef COFFEE_SERVICE_PERCEPTION__HOLE_PERCEPTION_ACTION_H_
#define COFFEE_SERVICE_PERCEPTION__HOLE_PERCEPTION_ACTION_H_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>
#include <Eigen/Eigen>
#include <sstream>

#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "geometry_msgs/msg/detail/pose__struct.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <rclcpp_action/rclcpp_action.hpp>
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/detail/marker__struct.hpp"
#include "visualization_msgs/msg/detail/marker_array__struct.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "custom_interfaces/action/find_hole_position.hpp"

#include "pcl/common/io.h"
#include "pcl/common/centroid.h"
#include <pcl/common/common.h>

#include "pcl/filters/passthrough.h"
#include "pcl/filters/voxel_grid.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include "pcl/filters/extract_indices.h"
#include <pcl/filters/project_inliers.h>

#include <pcl/point_cloud.h> 
#include "pcl/point_types.h"
#include "pcl_ros/transforms.hpp"

#include "pcl/segmentation/extract_clusters.h"
#include "pcl/segmentation/sac_segmentation.h"

#include <pcl/surface/convex_hull.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

namespace Coffee_Service_Perception
{
/**
 *  @brief Class that process a point cloud to get the hole position relative to base link.
 */  
class HolePerceptionAction : public rclcpp::Node 
{
  using FindHolesAction =
      custom_interfaces::action::FindHolePosition;
  using FindHoleActionGoal =
      rclcpp_action::ServerGoalHandle<FindHolesAction>;
public:
  /**
   *  @brief Constructor, loads pipeline using ROS parameters.
   */
  explicit HolePerceptionAction (const rclcpp::NodeOptions &options);

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  
  //Segment the plate and hole
  bool segment(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_input, 
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr plate_cloud, 
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr hole_cloud);
  
  // Extract the hole
  void hole_extration(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & plate_cloud, 
                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr &hole_cloud, 
                      std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& cloud_vector);

  // Rearrage the order of cloud in vector depending on the distance
  void hole_number(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& cloud_vector);

  // Project the border of hole to 2D plane, then use convex vull to filter border out
  void hole_projection(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& cloud_vector, 
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr &hole_cloud);

  // Function to estimate circle parameters using least squares fitting
  void estimateCircleParams(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& cloud_vector, 
                            std::vector<float>& xc, std::vector<float>& yc, 
                            float zc, std::vector<float>& r, bool debug ); 

  // Mark the center of find
  void marker(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & plate_cloud, 
                std::vector<float>& xc, std::vector<float>& yc, float zc, 
                std::vector<float>& r); 

  rclcpp_action::GoalResponse handle_goal( const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const FindHolesAction::Goal> goal_handle);

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<FindHoleActionGoal> goal_handle);

  void handle_accepted(const std::shared_ptr<FindHoleActionGoal> goal_handle);

  void execute(const std::shared_ptr<FindHoleActionGoal> goal_handle);

private:
  bool debug_;
  bool find_objects_;

  std::vector<geometry_msgs::msg::Point> holes_position_;
  std::vector<std::string> process_descriptions;
  std::vector<float> radius_;

  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  std::string world_frame_;
  std::string camera_topic_;

  double cluster_tolerance;
  rclcpp_action::Server<FindHolesAction>::SharedPtr server_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filter_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr plate_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr holes_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colored_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  
  pcl::PassThrough<pcl::PointXYZRGB> range_filter_x;
  pcl::PassThrough<pcl::PointXYZRGB> range_filter_y;
  pcl::PassThrough<pcl::PointXYZRGB> range_filter_z;
  pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid_;

  pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> segment_plate; 

  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB>
          outliers_filter;

  rclcpp::Clock::SharedPtr clock_;
};

}  

#endif 



