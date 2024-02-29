#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <iostream>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/io/pcd_io.h"
#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/reader.hpp"
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp_components/register_node_macro.hpp> // Add this header for component registration

namespace Coffee_Deliver {

using namespace std::chrono_literals;

class BagToPCLConversion : public rclcpp::Node
{
public:
  explicit BagToPCLConversion(const rclcpp::NodeOptions & options)
  : Node("bag_to_pcl_conversion", options)
  {
    // Define the path to the bag file
    std::string bag_filename = "/home/user/ros2_ws/src/my_bag/my_bag_0.db3";

    // Open the bag file
    rosbag2_cpp::Reader reader;
    reader.open({bag_filename});

    // Get the topics and types in the bag
    auto topics = reader.get_all_topics_and_types();
    if (topics.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No topics found in the bag file.");
      return;
    }
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;

    // Read messages from the bag file
    while (reader.has_next()) {
      rosbag2_storage::SerializedBagMessageSharedPtr bag_message = reader.read_next();

      if (bag_message->topic_name == "/wrist_rgbd_depth_sensor/points") {
        sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        rclcpp::SerializedMessage ros_message(*bag_message->serialized_data);
        serialization.deserialize_message(&ros_message, &(*cloud_msg));

        pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
        pcl::fromROSMsg(*cloud_msg, pcl_cloud);
        pcl_cloud_ += pcl_cloud;

        }
    }

    // Save the combined PCL point cloud to a PCD file
    pcl::io::savePCDFileBinary("output_pcl_file_.pcd", pcl_cloud_);

  }

private:
  pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud_;
};

} // namespace Coffee_Deliver

// Register the node as a component
RCLCPP_COMPONENTS_REGISTER_NODE(Coffee_Deliver::BagToPCLConversion)
