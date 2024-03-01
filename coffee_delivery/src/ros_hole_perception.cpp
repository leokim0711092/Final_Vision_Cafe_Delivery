#include <memory>
#include <string>
#include <vector>
#include <Eigen/Eigen>

#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_listener.h"

#include "coffee_delivery/hole_plate_segmentation.h"

#include "pcl/common/io.h"
#include "pcl/common/centroid.h"
#include <pcl/common/common.h>

#include "pcl/filters/passthrough.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl_ros/transforms.hpp"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include "pcl/point_types.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/segmentation/sac_segmentation.h"
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

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

  // cluster_tolerance: minimum separation distance of two objects
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

      hole1_cloud_pub_ =
          this->create_publisher<sensor_msgs::msg::PointCloud2>("hole_1_cloud",
                                                                qos);
      hole2_cloud_pub_ =
          this->create_publisher<sensor_msgs::msg::PointCloud2>("hole_2_cloud",
                                                                qos);
      hole3_cloud_pub_ =
          this->create_publisher<sensor_msgs::msg::PointCloud2>("hole_3_cloud",
                                                                qos);
      hole4_cloud_pub_ =
          this->create_publisher<sensor_msgs::msg::PointCloud2>("hole_4_cloud",
                                                                qos);
    }

    // Range filter for cloud
    range_filter_x.setFilterFieldName("x");
    range_filter_x.setFilterLimits(-0.7, 0.2);

    range_filter_z.setFilterFieldName("z");
    range_filter_z.setFilterLimits(0, 0.95);

    // StatisticalOutlierRemoval filter
    outliers_filter.setMeanK(150);
    outliers_filter.setStddevMulThresh(0.8);

    //   voxel grid the data before segmenting
    double leaf_size;
    leaf_size = this->declare_parameter<double>("voxel_leaf_size", 0.005);
    voxel_grid_.setLeafSize(leaf_size, leaf_size, leaf_size);

    //segment plate
    segment_plate.setOptimizeCoefficients(true);
    segment_plate.setModelType(pcl::SACMODEL_PLANE);
    segment_plate.setNormalDistanceWeight(0.1);
    segment_plate.setMaxIterations(100);
    segment_plate.setDistanceThreshold(cluster_tolerance);

    //segment hole
    segment_hole.setOptimizeCoefficients(true);
    segment_hole.setModelType (pcl::SACMODEL_CYLINDER);
    segment_hole.setNormalDistanceWeight(0.1);
    segment_hole.setMethodType (pcl::SAC_RANSAC);
    segment_hole.setMaxIterations(10000);
    segment_hole.setRadiusLimits (0, 0.04);
    segment_hole.setDistanceThreshold(cluster_tolerance);

    // Setup TF2
    buffer_.reset(new tf2_ros::Buffer(this->get_clock()));
    listener_.reset(new tf2_ros::TransformListener(*buffer_));

    // Subscribe to head camera cloud
    rclcpp::QoS points_qos(10);
    points_qos.best_effort();
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/wrist_rgbd_depth_sensor/points", points_qos,
        std::bind(&BasicHolePerception::cloud_callback, this, _1));

    RCLCPP_INFO(LOGGER, "cafe_delivery_perception initialized");

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

    // StatisticalOutlierRemoval filter
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
    if (!pcl_ros::transformPointCloud(world_frame_, *cloud_filtered_outlier,
                                      *cloud_transformed, *buffer_)) {
      RCLCPP_ERROR(LOGGER, "Error transforming to frame %s",
                   world_frame_.c_str());
      return;
    }

    // Run segmentation
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plate_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr hole_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_vector;
    // Create four instances of point clouds and store their pointers in the vector
    for (int i = 0; i < 4; ++i) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr hole_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud_vector.push_back(hole_cloud);
    }
    // pcl::PointCloud<pcl::PointXYZRGB> plate_cloud;
    // pcl::PointCloud<pcl::PointXYZRGB> hole_cloud;
    if (debug_) {
      plate_cloud->header.frame_id = cloud_transformed->header.frame_id;
      hole_cloud->header.frame_id = cloud_transformed->header.frame_id;
    }
    segment(cloud_transformed, plate_cloud, hole_cloud);
    hole_seperation(plate_cloud, hole_cloud, cloud_vector);
    hole_number(cloud_vector);

    if (true) {
      sensor_msgs::msg::PointCloud2 cloud_msg;

      pcl::toROSMsg(*plate_cloud, cloud_msg);
      plate_cloud_pub_->publish(cloud_msg);

      pcl::toROSMsg(*hole_cloud, cloud_msg);
      holes_cloud_pub_->publish(cloud_msg);

      pcl::toROSMsg(*cloud_vector[0], cloud_msg);
      hole1_cloud_pub_->publish(cloud_msg);

      pcl::toROSMsg(*cloud_vector[1], cloud_msg);
      hole2_cloud_pub_->publish(cloud_msg);

      pcl::toROSMsg(*cloud_vector[2], cloud_msg);
      hole3_cloud_pub_->publish(cloud_msg);

      pcl::toROSMsg(*cloud_vector[3], cloud_msg);
      hole4_cloud_pub_->publish(cloud_msg);

    }
  }

  bool segment(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_input,
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr plate_cloud,
             pcl::PointCloud<pcl::PointXYZRGB>::Ptr hole_cloud) {

    RCLCPP_INFO(LOGGER, "object support segmentation starting...");
    // Convert to point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    // process the cloud with a voxel grid
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_cloud_filtered(new
    pcl::PointCloud<pcl::PointXYZRGB>);


    voxel_grid_.setInputCloud(cloud_input);
    voxel_grid_.filter(*voxel_cloud_filtered);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_plate (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_hole (new pcl::PointCloud<pcl::Normal>);

    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    
    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (voxel_cloud_filtered);
    ne.setKSearch (50);
    ne.compute (*cloud_normals_plate);

    segment_plate.setInputCloud(voxel_cloud_filtered);
    segment_plate.setInputNormals(cloud_normals_plate);
    segment_plate.segment(*inliers, *coefficients);

    // Extract planar part for message
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_plane;
    extract_plane.setInputCloud(voxel_cloud_filtered);
    extract_plane.setIndices(inliers);
    extract_plane.setNegative(false);
    extract_plane.filter(*plate_cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_store(new
    pcl::PointCloud<pcl::PointXYZRGB>);

    //Remove the planar inliers, extract the rest
    extract_plane.setNegative (true);
    extract_plane.filter(*cloud_filtered_store);
    extract_normals.setNegative (true);
    extract_normals.setInputCloud (cloud_normals_plate);
    extract_normals.setIndices (inliers);
    extract_normals.filter (*cloud_normals_hole);

    pcl::PointIndices::Ptr inliers_hole(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_hole(new pcl::ModelCoefficients);

    segment_hole.setInputCloud(cloud_filtered_store);
    segment_hole.setInputNormals(cloud_normals_hole);
    segment_hole.segment(*inliers_hole, *coefficients_hole);

    // Extract planar part for message
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_hole;
    extract_hole.setInputCloud(cloud_filtered_store);
    extract_hole.setIndices(inliers_hole);
    extract_hole.setNegative(true);
    extract_hole.filter(*cloud_filtered_store);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB>
          outlier_filt;

    outlier_filt.setMeanK(60);
    outlier_filt.setStddevMulThresh(1.0);
    outlier_filt.setInputCloud(cloud_filtered_store);
    outlier_filt.filter(*hole_cloud);

    RCLCPP_INFO(LOGGER, "object support segmentation done processing.");
    return true;
  }

  void hole_seperation( pcl::PointCloud<pcl::PointXYZRGB>::Ptr plate_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr hole_cloud, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_vector){
        
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> cluster_extraction;
    cluster_extraction.setInputCloud(hole_cloud);
    cluster_extraction.setClusterTolerance(0.01); // Set the cluster tolerance (max. distance between points in a cluster)
    cluster_extraction.setMinClusterSize(100);    // Set the minimum number of points required for a cluster
    cluster_extraction.setMaxClusterSize(10000);   // Set the maximum number of points allowed for a cluster
    std::vector<pcl::PointIndices> clusters;
    cluster_extraction.extract(clusters);
    
    RCLCPP_INFO(LOGGER, "Hole cloud size: %zu", hole_cloud->size());
    RCLCPP_INFO(LOGGER, "Cluster number: %zu", clusters.size());
    
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices_;
    extract_indices_.setInputCloud(hole_cloud);

    for (size_t i = 0; i < clusters.size(); i++) {
    extract_indices_.setIndices(pcl::PointIndicesPtr(new pcl::PointIndices(clusters[i])));
    extract_indices_.filter(*cloud_vector[i]);
    RCLCPP_INFO(LOGGER, "Cluster number: %zu", cloud_vector[i]->size());

    }

  }
  
  void hole_number(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_vector){
        
    // Find centroid
    Eigen::Vector4f centroid;
    std::vector<Eigen::Vector4f> centroid_vector;
    std::vector<Eigen::Vector2f> centroid_xy_vector;

    for (size_t i = 0; i < cloud_vector.size(); ++i) {
        pcl::compute3DCentroid(*cloud_vector[i], centroid);
        centroid_vector.push_back(centroid);  
        centroid_xy_vector.push_back(centroid.head<2>()); // Extract only x and y coordinates
    }

    // Calculate distances from each centroid to origin (0, 0, 0)
    std::vector<float> distances;
    for (const auto& centroid : centroid_xy_vector) {
        float distance = centroid.norm(); // Calculate Euclidean norm
        distances.push_back(distance);
        RCLCPP_INFO(LOGGER, "Distance %zu is: %f", distances.size(), distance);
    }

    // Rearrange cloud_vector based on distances (sort it)
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> sorted_cloud_vector;
    std::vector<std::size_t> indices(cloud_vector.size());
    std::iota(indices.begin(), indices.end(), 0); 
    std::sort(indices.begin(), indices.end(), [&](std::size_t i, std::size_t j) {
        return distances[i] < distances[j]; // Sort based on distances
    });

    for (const auto& index : indices) {
        sorted_cloud_vector.push_back(cloud_vector[index]);
    }

    // Assign sorted_cloud_vector back to cloud_vector
    cloud_vector = sorted_cloud_vector;


    // Define colors for each cluster
    std::vector<std::vector<uint8_t>> cluster_colors = {
        {0, 255, 0},    // Green
        {255, 255, 0},  // Yellow
        {0, 0, 255},    // Blue
        {255, 0, 0}     // Red
    };

    int color_index = 0; // Index for selecting color from cluster_colors

    for (size_t i = 0; i < cloud_vector.size(); ++i) {

        std::vector<uint8_t> color = cluster_colors[i];
            for (auto& point : cloud_vector[i]->points) {
                    point.r = color[0];
                    point.g = color[1];
                    point.b = color[2];
                }
    }

  }


  bool debug_;

  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  std::string world_frame_;
  double cluster_tolerance;

  //   std::shared_ptr<HolePlateSegmentation> segmentation_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filter_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr plate_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr holes_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr hole1_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr hole2_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr hole3_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr hole4_cloud_pub_;


  pcl::PassThrough<pcl::PointXYZRGB> range_filter_x;
  pcl::PassThrough<pcl::PointXYZRGB> range_filter_z;
  pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid_;
//   pcl::SACSegmentation<pcl::PointXYZRGB,  pcl::Normal> segment_plate;
//   pcl::SACSegmentation<pcl::PointXYZRGB> segment_hole;
  pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> segment_plate; 
  pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> segment_hole; 


  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB>
          outliers_filter;

  rclcpp::Clock::SharedPtr clock_;
};

} // namespace Coffee_Deliver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(Coffee_Deliver::BasicHolePerception)