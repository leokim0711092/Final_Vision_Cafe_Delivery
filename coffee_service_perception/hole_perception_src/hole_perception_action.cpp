#include "coffee_service_perception/hole_perception_action.h"

#include "coffee_service_perception/hole_perception_extration.h"
#include "coffee_service_perception/hole_perception_segmentation.h"
#include "geometry_msgs/msg/detail/point__struct.hpp"
#include <cstddef>

namespace Coffee_Service_Perception {

static const rclcpp::Logger LOGGER = rclcpp::get_logger("Hole perception action");

using std::placeholders::_1;
using std::placeholders::_2;

HolePerceptionAction::HolePerceptionAction(const rclcpp::NodeOptions &options)
      : rclcpp::Node("hole_perception_action", options), debug_(true), find_objects_(false){
    
    // Store clock
    clock_ = this->get_clock();

    loop_set = this->declare_parameter<int>("loop_set", 1);
    count_callback = 0;

    // use_debug: enable/disable output of a cloud containing object points
    debug_ = this->declare_parameter<bool>("debug_topics", true);

    // frame_id: frame to transform cloud to (should be XY horizontal)
    world_frame_ =
        this->declare_parameter<std::string>("frame_id", "base_link");

    camera_topic_ =
        this->declare_parameter<std::string>("camera_topic", "/wrist_rgbd_depth_sensor/points");

    cluster_tolerance = this->declare_parameter<double>("cluster_tolerance", 0.01);

    // Publish debugging views
    if (debug_) {
      rclcpp::QoS qos(1);
      qos.best_effort();
      filter_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "filter_cloud", qos);
      plate_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "plate_cloud", qos);
      holes_cloud_pub_ =
          this->create_publisher<sensor_msgs::msg::PointCloud2>("holes_cloud",
                                                                qos);
      colored_cloud_pub_ =
          this->create_publisher<sensor_msgs::msg::PointCloud2>("color_hole_cloud",
                                                                qos);
      marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("marker", qos);
    }


    // Range filter for cloud3
    range_filter_x.setFilterFieldName("x");
    range_filter_x.setFilterLimits(-0.6, 0.0);

    range_filter_y.setFilterFieldName("y");
    range_filter_y.setFilterLimits(-0.35, 0.28);

    range_filter_z.setFilterFieldName("z");
    range_filter_z.setFilterLimits(-0.6, 0.0);

    // StatisticalOutlierRemoval filter
    outliers_filter.setMeanK(50);
    outliers_filter.setStddevMulThresh(2.5);

    // Setup TF2
    buffer_.reset(new tf2_ros::Buffer(this->get_clock()));
    listener_.reset(new tf2_ros::TransformListener(*buffer_));

    // Subscribe to head camera cloud
    rclcpp::QoS points_qos(10);
    if (camera_topic_ =="/wrist_rgbd_depth_sensor/points") {
        points_qos.best_effort();
    }else {
        points_qos.reliable();
    }

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        camera_topic_ , points_qos,
        std::bind(&HolePerceptionAction::cloud_callback, this, _1));

    RCLCPP_INFO(LOGGER, "cafe_delivery_perception initialized");
    plate_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    hole_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    // Setup actionlib server
    server_ = rclcpp_action::create_server<FindHolesAction>(
        this->get_node_base_interface(), this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(), "find_hole",
        std::bind(&HolePerceptionAction::handle_goal, this, _1, _2),
        std::bind(&HolePerceptionAction::handle_cancel, this, _1),
        std::bind(&HolePerceptionAction::handle_accepted, this, _1));

}

rclcpp_action::GoalResponse HolePerceptionAction::handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const FindHolesAction::Goal> goal_handle) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

rclcpp_action::CancelResponse HolePerceptionAction::handle_cancel(const std::shared_ptr<FindHoleActionGoal> goal_handle) {
    return rclcpp_action::CancelResponse::ACCEPT;
}

void HolePerceptionAction::handle_accepted(const std::shared_ptr<FindHoleActionGoal> goal_handle) {
    if (!segmentation_ || !hole_extration_ || !estimate_center_) {
      estimate_center_.reset(new EstimateCenter(this->shared_from_this()));
      // Create Hole Extration instance
      hole_extration_.reset(new HoleExtration(this->shared_from_this()));

      // Create Segmentation instance
      segmentation_.reset(new Segmentation(this->shared_from_this()));
    }

    // Break off a thread
    std::thread{std::bind(&HolePerceptionAction::execute, this, _1), goal_handle}.detach();
  }

  void HolePerceptionAction::execute(const std::shared_ptr<FindHoleActionGoal> goal_handle) {
    
    auto result = std::make_shared<FindHolesAction::Result>();
    auto feedback = std::make_shared<FindHolesAction::Feedback>();

    const auto goal = goal_handle->get_goal();

    if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
    }

    // Get objects
    find_objects_ = goal->find_hole;
    rclcpp::Time t = clock_->now();
    while (find_objects_ == true) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      if (clock_->now() - t > rclcpp::Duration::from_seconds(3.0)) {
        find_objects_ = false;
        goal_handle->abort(result);
        RCLCPP_ERROR(LOGGER, "Failed to get camera data in alloted time.");
        return;
      }
    }
    feedback->process_description = process_descriptions;
    goal_handle->publish_feedback(feedback);

    result->hole_position = holes_position_;
    result->hole_radius = radius_;

    goal_handle->succeed(result);
  }



void HolePerceptionAction::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    count_callback++;

    if (!find_objects_) {
        return;
    }

    // Remove last time store
    holes_position_.clear();
    process_descriptions.clear();
    radius_.clear();

    // Convert to point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud =
        std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::fromROSMsg(*msg, *cloud);

    // Transform to grounded (base_link)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (!pcl_ros::transformPointCloud(world_frame_, *cloud,
                                      *cloud_transformed, *buffer_)) {
      RCLCPP_ERROR(LOGGER, "Error transforming to frame %s",
                   world_frame_.c_str());
      return;
    }

    RCLCPP_DEBUG(LOGGER, "Cloud recieved with %d points.",
                 static_cast<int>(cloud->points.size()));

    // Filter out noisy long-range points
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(
        new pcl::PointCloud<pcl::PointXYZRGB>);

    range_filter_z.setInputCloud(cloud_transformed);
    range_filter_z.filter(*cloud_filtered);
    RCLCPP_DEBUG(LOGGER, "Filtered for range, now %d points.",
                 static_cast<int>(cloud_filtered->points.size()));
    
    range_filter_x.setInputCloud(cloud_filtered);
    range_filter_x.filter(*cloud_filtered);

    range_filter_y.setInputCloud(cloud_filtered);
    range_filter_y.filter(*cloud_filtered);

    RCLCPP_DEBUG(LOGGER, "Filtered for range, now %d points.",
                 static_cast<int>(cloud_filtered->points.size()));

    // StatisticalOutlierRemoval filter    
    outliers_filter.setMeanK(50);
    outliers_filter.setStddevMulThresh(2.5);
    outliers_filter.setInputCloud(cloud_filtered);
    outliers_filter.filter(*cloud_filtered);

    if (debug_) {
      sensor_msgs::msg::PointCloud2 cloud_msg;

      pcl::toROSMsg(*cloud_filtered, cloud_msg);
      filter_cloud_pub_->publish(cloud_msg);
    }

    // Run segmentation
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plate_cloud_store(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr hole_cloud_store(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_vector;

    if (debug_) {
      plate_cloud->header.frame_id = cloud_filtered->header.frame_id;
      hole_cloud->header.frame_id = cloud_filtered->header.frame_id;
    }
        // segment(cloud_filtered, plate_cloud, hole_cloud);
        segmentation_->segment(cloud_filtered, plate_cloud_store, hole_cloud_store, process_descriptions);
        *plate_cloud += *plate_cloud_store;
        *hole_cloud += *hole_cloud_store;
        hole_extration_->hole_extration(plate_cloud, hole_cloud, cloud_vector, process_descriptions);
    // callback loop to collect more point cloud
    if (count_callback < loop_set) { 
        return;
    }

    if (cloud_vector.size() > 0) {
        hole_extration_->hole_number(cloud_vector, process_descriptions);
        hole_extration_->hole_projection(cloud_vector, hole_cloud, process_descriptions);
        std::vector<float> xc, yc, r;
        float zc;   
        estimate_center_->estimateCircleParams(cloud_vector, xc, yc, zc ,r,  process_descriptions, true);
        visualization_msgs::msg::MarkerArray marker_array;
        estimate_center_->marker(plate_cloud, xc, yc, zc ,r, marker_array);
        marker_pub_->publish(marker_array);
        
        //For action Result
        for (size_t i = 0; i < xc.size(); ++i) {
            geometry_msgs::msg::Point point;
            point.x = xc[i];
            point.y = yc[i];
            point.z = zc;
            holes_position_.push_back(point);
            radius_.push_back(r[i]);
        }
    }else {
        RCLCPP_INFO(LOGGER, "No hole detect to place coffee");
        process_descriptions.push_back("No hole detect to place coffee");
    }

    if (debug_) {
      sensor_msgs::msg::PointCloud2 cloud_msg;
      sensor_msgs::msg::PointCloud2 store_msg;

      pcl::toROSMsg(*plate_cloud, cloud_msg);
      plate_cloud_pub_->publish(cloud_msg);

      pcl::toROSMsg(*hole_cloud, cloud_msg);
      holes_cloud_pub_->publish(cloud_msg);


      for (size_t i = 0; i < cloud_vector.size(); ++i) {
        pcl::toROSMsg(*cloud_vector[i], cloud_msg);
        if( i == 0){
            store_msg = cloud_msg; 
        }else{
         // Concatenate the point cloud data
        store_msg.width += cloud_msg.width;
        store_msg.row_step += cloud_msg.row_step;
        store_msg.data.insert(store_msg.data.end(), cloud_msg.data.begin(), cloud_msg.data.end());
        }
      }        

      colored_cloud_pub_->publish(store_msg);

    }
    count_callback = 0;
    plate_cloud->clear();
    hole_cloud->clear();
    find_objects_ = false;
    process_descriptions.push_back("perception process finsih");

  }


};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(Coffee_Service_Perception::HolePerceptionAction)

