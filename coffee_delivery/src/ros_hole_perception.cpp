#include <memory>
#include <string>
#include <vector>

#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_listener.h"

#include "pcl/common/io.h"
#include "pcl/filters/passthrough.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "pcl_ros/transforms.hpp"

namespace Coffee_Deliver {

static const rclcpp::Logger LOGGER = rclcpp::get_logger("Hole perception");

using std::placeholders::_1;
using std::placeholders::_2;

class BasicHolePerception : public rclcpp::Node {

public:
  explicit BasicHolePerception(const rclcpp::NodeOptions &options)
      : rclcpp::Node("basic_hole_perception", options), debug_(true) {
    // Store clock
    clock_ = this->get_clock();

    // use_debug: enable/disable output of a cloud containing object points
    debug_ = this->declare_parameter<bool>("debug_topics", true);

    // frame_id: frame to transform cloud to (should be XY horizontal)
    world_frame_ =
        this->declare_parameter<std::string>("frame_id", "base_link");

    // Publish debugging views
    if (debug_) {
      rclcpp::QoS qos(1);
      qos.best_effort();
      filter_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "filter_cloud", qos);
    }

    // Range filter for cloud
    range_filter_x.setFilterFieldName("x");
    range_filter_x.setFilterLimits(-0.7, 0.2);

    range_filter_z.setFilterFieldName("z");
    range_filter_z.setFilterLimits(0, 0.95);

    //StatisticalOutlierRemoval filter
    outliers_filter.setMeanK (150);
    outliers_filter.setStddevMulThresh (0.8);

    // Setup TF2
    buffer_.reset(new tf2_ros::Buffer(this->get_clock()));
    listener_.reset(new tf2_ros::TransformListener(*buffer_));

    // Subscribe to head camera cloud
    rclcpp::QoS points_qos(10);
    points_qos.best_effort();
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/wrist_rgbd_depth_sensor/points", points_qos,
        std::bind(&BasicHolePerception::cloud_callback, this, _1));

    RCLCPP_INFO(LOGGER, "basic_grasping_perception initialized");
  }

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    // Convert to point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud =
        std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::fromROSMsg(*msg, *cloud);

    RCLCPP_DEBUG(LOGGER, "Cloud recieved with %d points.",
                 static_cast<int>(cloud->points.size()));

    // Filter out noisy long-range points
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_z(
        new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_x(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    range_filter_z.setInputCloud(cloud);
    range_filter_z.filter(*cloud_filtered_z);

    range_filter_x.setInputCloud(cloud_filtered_z);
    range_filter_x.filter(*cloud_filtered_x);

    RCLCPP_DEBUG(LOGGER, "Filtered for range, now %d points.",
                 static_cast<int>(cloud_filtered_x->points.size()));
    RCLCPP_DEBUG(LOGGER, "Filtered for range, now %d points.",
                 static_cast<int>(cloud_filtered_z->points.size()));

    //StatisticalOutlierRemoval filter
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_outlier(
        new pcl::PointCloud<pcl::PointXYZRGB>);

    outliers_filter.setInputCloud(cloud_filtered_x);
    outliers_filter.filter(*cloud_filtered_outlier);

    if (debug_) {
      sensor_msgs::msg::PointCloud2 cloud_msg;

      pcl::toROSMsg(*cloud_filtered_outlier, cloud_msg);
      filter_cloud_pub_->publish(cloud_msg);
    }



    // Transform to grounded (base_link)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    if (!pcl_ros::transformPointCloud(world_frame_, *cloud_filtered_x,
                                      *cloud_transformed, *buffer_)) {
      RCLCPP_ERROR(LOGGER, "Error transforming to frame %s",
                   world_frame_.c_str());
      return;
    }
  }



  bool debug_;

  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  std::string world_frame_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filter_cloud_pub_;

  rclcpp::Clock::SharedPtr clock_;

  pcl::PassThrough<pcl::PointXYZRGB> range_filter_x;
  pcl::PassThrough<pcl::PointXYZRGB> range_filter_z;

  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> outliers_filter;

};

} // namespace Coffee_Deliver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(Coffee_Deliver::BasicHolePerception)