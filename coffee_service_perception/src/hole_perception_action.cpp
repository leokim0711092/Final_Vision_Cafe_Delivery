#include "coffee_service_perception/hole_perception_action.h"
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

    //   voxel grid the data before segmenting
    double leaf_size;
    leaf_size = this->declare_parameter<double>("voxel_leaf_size", 0.005);
    voxel_grid_.setLeafSize(leaf_size, leaf_size, leaf_size);

    //segment plate
    segment_plate.setOptimizeCoefficients(true);
    segment_plate.setModelType(pcl::SACMODEL_PLANE);
    segment_plate.setNormalDistanceWeight(0.005);
    segment_plate.setMaxIterations(100);
    segment_plate.setDistanceThreshold(0.01);

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

    if (!find_objects_) {
        return;
    }

    // Remove last time store
    holes_position_.clear();
    process_descriptions.clear();
    radius_.clear();

    // rclcpp::Rate r(0.5);
    // r.sleep();
    // Convert to point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud =
        std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::fromROSMsg(*msg, *cloud);

    // Transform to grounded (base_link)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(
        new pcl::PointCloud<pcl::PointXYZRGB>);
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
    outliers_filter.setInputCloud(cloud_filtered);
    outliers_filter.filter(*cloud_filtered);

    if (debug_) {
      sensor_msgs::msg::PointCloud2 cloud_msg;

      pcl::toROSMsg(*cloud_filtered, cloud_msg);
      filter_cloud_pub_->publish(cloud_msg);
    }

    // Run segmentation
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plate_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr hole_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_vector;

    if (debug_) {
      plate_cloud->header.frame_id = cloud_filtered->header.frame_id;
      hole_cloud->header.frame_id = cloud_filtered->header.frame_id;
    }
        segment(cloud_filtered, plate_cloud, hole_cloud);
        hole_extration(plate_cloud, hole_cloud, cloud_vector);
        sensor_msgs::msg::PointCloud2 cloud_msg;

       pcl::toROSMsg(*plate_cloud, cloud_msg);
       plate_cloud_pub_->publish(cloud_msg);

    if (cloud_vector.size() > 0) {
        hole_number(cloud_vector);
        hole_projection(cloud_vector, hole_cloud);
        std::vector<float> xc, yc, r;
        float zc;   
        estimateCircleParams(cloud_vector, xc, yc, zc ,r, true);
        marker(plate_cloud, xc, yc, zc ,r);
    }else {
        RCLCPP_INFO(LOGGER, "No hole detect to place coffee");
        process_descriptions.push_back("No hole detect to place coffee");
    }

    if (debug_) {
      sensor_msgs::msg::PointCloud2 cloud_msg;
      sensor_msgs::msg::PointCloud2 store_msg;

    //   pcl::toROSMsg(*plate_cloud, cloud_msg);
    //   plate_cloud_pub_->publish(cloud_msg);

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

    find_objects_ = false;

  }

//Segment the plate and hole
bool HolePerceptionAction::segment(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_input,
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr plate_cloud,
             pcl::PointCloud<pcl::PointXYZRGB>::Ptr hole_cloud) {

    RCLCPP_INFO(LOGGER, "object support segmentation starting...");
    process_descriptions.push_back("object support segmentation starting...");
    
    // Convert to point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    // process the cloud with a voxel grid
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);


    voxel_grid_.setInputCloud(cloud_input);
    voxel_grid_.filter(*voxel_cloud_filtered);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_plate (new pcl::PointCloud<pcl::Normal>);

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

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*plate_cloud, centroid);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_store(new
    pcl::PointCloud<pcl::PointXYZRGB>);

    float radius = 0.125;

    //Remove the planar inliers, extract the rest
    extract_plane.setNegative (true);
    extract_plane.filter(*cloud_filtered_store);
    
    // Create the condition for removal
    for (size_t i = 0; i < cloud_filtered_store->size(); ++i) {
        float dx = cloud_filtered_store->points[i].x - centroid.x();
        float dy = cloud_filtered_store->points[i].y - centroid.y();
        float distance = std::sqrt(dx * dx + dy * dy); // Euclidean distance from center
        if (distance <= radius) {
            hole_cloud->points.push_back(cloud_filtered_store->points[i]);
        }
    }
   
    RCLCPP_INFO(LOGGER, "object support segmentation done processing.");
    process_descriptions.push_back("object support segmentation done processing.");

    return true;
}


// Extract the hole
void HolePerceptionAction::hole_extration( pcl::PointCloud<pcl::PointXYZRGB>::Ptr & plate_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &hole_cloud, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& cloud_vector){
        
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> cluster_extraction;
    cluster_extraction.setInputCloud(hole_cloud);
    cluster_extraction.setClusterTolerance(0.01); // Set the cluster tolerance (max. distance between points in a cluster)
    cluster_extraction.setMinClusterSize(100);    // Set the minimum number of points required for a cluster
    cluster_extraction.setMaxClusterSize(1000);   // Set the maximum number of points allowed for a cluster
    std::vector<pcl::PointIndices> clusters;
    cluster_extraction.extract(clusters);
                
    if (debug_) {
        RCLCPP_INFO(LOGGER, "Hole cloud size: %zu, Exstract %zu Cluster", hole_cloud->size(), clusters.size());
    }
    
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices_;
    extract_indices_.setInputCloud(hole_cloud);

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> store_vector;

    for (size_t i = 0; i < clusters.size(); i++) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr hole_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    store_vector.push_back(hole_cloud);

    extract_indices_.setIndices(pcl::PointIndicesPtr(new pcl::PointIndices(clusters[i])));
    extract_indices_.filter(*store_vector[i]);
    }

    std::vector<std::vector<size_t>> groups;

    // Remove the coffee cup
    for(size_t i = 0; i < store_vector.size(); i++){

        Eigen::Vector4f centroid_1;
        pcl::compute3DCentroid(*store_vector[i], centroid_1);
        
        std::vector<size_t> index;

        for(size_t j = i+1; j < store_vector.size(); j++){
            
            Eigen::Vector4f centroid_2;
            pcl::compute3DCentroid(*store_vector[j], centroid_2);
            float dx = centroid_2.x() - centroid_1.x();
            float dy = centroid_2.y() - centroid_1.y();
            float centroid_distance = std::sqrt(dx * dx + dy * dy); // Euclidean distance from center

            if (centroid_distance <= 0.07) {
                auto it = std::find(index.begin(), index.end(), i);
                if (it == index.end()) index.push_back(i);
                
                it = std::find(index.begin(), index.end(), j);
                if (it == index.end()) index.push_back(j);
            }
        }
        if (index.size() > 0) {
            groups.push_back(index);
        }
    }
    size_t count = 0;

    for (const auto& group : groups) {

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        for(size_t i = 0; i < group.size(); i++){
            *cloud += *store_vector[group[i]];
        }
        cloud_vector.push_back(cloud);
        RCLCPP_INFO(LOGGER, "Cluster %zu size: %zu", count+1 ,cloud_vector[count]->size());

        std::stringstream ss;
        ss << "Cluster " << (count + 1) << " size: " << cloud_vector[count]->size();
        std::string s = ss.str();

        process_descriptions.push_back(s);

        count++;
    }

    
    for(size_t i = 0; i < store_vector.size(); i++){
        
        bool find = false;

        for (const auto& index : groups) {

        auto it = std::find(index.begin(), index.end(), i);

            if (it != index.end()) {       
                find = true;
                break;
            }
        }

        if (!find) {       
            cloud_vector.push_back(store_vector[i]);
            
            if (debug_) {
                RCLCPP_INFO(LOGGER, "Cluster %zu size: %zu", count+1 ,cloud_vector[count]->size());
            }
            count++;
        }
    }

    RCLCPP_INFO(LOGGER, "Hole seperation done");
    process_descriptions.push_back("Hole seperation done");

}
  
// Rearrage the order of cloud in vector depending on the distance
void HolePerceptionAction::hole_number(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& cloud_vector){
    process_descriptions.push_back("Start reorder point cloud");

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
    }

    // Rearrange cloud_vector based on distances (sort it)
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> sorted_cloud_vector;
    std::vector<std::size_t> indices(cloud_vector.size());
    std::iota(indices.begin(), indices.end(), 0); 
    std::sort(indices.begin(), indices.end(), [&](std::size_t i, std::size_t j) {
        return distances[i] > distances[j]; // Sort based on distances
    });

    for (const auto& index : indices) {
        sorted_cloud_vector.push_back(cloud_vector[index]);
    }

    // Assign sorted_cloud_vector back to cloud_vector
    cloud_vector = sorted_cloud_vector;
    process_descriptions.push_back("Finish reorder the cloud vector");

  }

  // Project the border of hole to 2D plane, then use convex vull to filter border out
void HolePerceptionAction::hole_projection(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& cloud_vector, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &hole_cloud){
    
    process_descriptions.push_back("Start project point cloud");

    pcl::PointXYZRGB Min_pt, Max_pt;
    pcl::getMinMax3D(*hole_cloud, Min_pt, Max_pt);
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> proj_vector;

   for (size_t i = 0; i < cloud_vector.size(); ++i) {

        // Create a set of planar coefficients with X=Y=0,Z=min_z
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
        coefficients->values.resize (4);
        coefficients->values[0] = coefficients->values[1] = 0;
        coefficients->values[2] = 1;
        coefficients->values[3] = -Min_pt.z;

        // Create the filtering object
        pcl::ProjectInliers<pcl::PointXYZRGB> proj;
        proj.setModelType (pcl::SACMODEL_PLANE);
        proj.setInputCloud (cloud_vector[i]);
        proj.setModelCoefficients (coefficients);
        proj.filter (*cloud_vector[i]);        

        pcl::ConvexHull<pcl::PointXYZRGB> convex_hull;
        convex_hull.setInputCloud(cloud_vector[i]);
        convex_hull.setDimension(2);
        convex_hull.reconstruct(*cloud_vector[i]);
        
    }
    process_descriptions.push_back("Finish project point cloud");

}

  // Function to estimate circle parameters using least squares fitting
void HolePerceptionAction::estimateCircleParams(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& cloud_vector, std::vector<float>& xc, std::vector<float>& yc, float zc, std::vector<float>& r, bool debug = true) {
 
    pcl::PointXYZRGB min_pt, max_pt;
    pcl::getMinMax3D(*cloud_vector[0], min_pt, max_pt);
    zc = (2*min_pt.z + 0.14)/2.0;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> store_vector;
    
    size_t count=0;
    for (size_t i = 0; i < cloud_vector.size(); ++i) {
        // Number of points
        int N = cloud_vector[i]->size();

        // Prepare matrices
        Eigen::MatrixXd A(N, 3);
        Eigen::MatrixXd B(N, 1);
        for (int j = 0; j < N; ++j) {
            A(j, 0) = cloud_vector[i]->points[j].x;
            A(j, 1) = cloud_vector[i]->points[j].y;
            A(j, 2) = 1;
            B(j) = cloud_vector[i]->points[j].x * cloud_vector[i]->points[j].x + cloud_vector[i]->points[j].y * cloud_vector[i]->points[j].y;
        }

        // Least square approximation
        Eigen::Vector3d X = (A.transpose() * A).ldlt().solve(A.transpose() * B);

        float radius = sqrt(4 * X(2) + X(0) * X(0) + X(1) * X(1)) / 2;
        // if (radius<= 0.0365 && radius >= 0.0335 ) {
            // Calculate circle parameters
            xc.push_back(X(0) / 2);
            yc.push_back(X(1) / 2);
            r.push_back(sqrt(4 * X(2) + X(0) * X(0) + X(1) * X(1)) / 2);

            if (debug) {
                // RCLCPP_INFO(LOGGER, "Hole %zu center is: (%f, %f, %f)", i+1, xc[i]+ 13.9 - 0.01,  yc[i]-18.56, zc+1.032); // for cup with cover
                RCLCPP_INFO(LOGGER, "Hole %zu center in gazebo is: (%f, %f, %f)", count+1, xc[count]+ 13.9 - 0.04- 0.01,  yc[count]-18.56 + 0.04, zc+1.032); // for cup without cover, this value is related to the radius 0.04, 0.01 is the deviation
                RCLCPP_INFO(LOGGER, "Hole %zu center in gazebo is: (%f, %f, %f)", count+1, xc[count]- 0.01,  yc[count], zc+1.032); // for cup without cover, this value is related to the inertia pose we set

                RCLCPP_INFO(LOGGER, "Hole %zu center is: (%f, %f, %f)", count+1, xc[count],  yc[count], zc);

                
                std::stringstream ss;
                ss << "Hole " << (count + 1) << " center is: (" << xc[count] << ", " << yc[count] << ", " << zc << ")";
                std::string s = ss.str();
                process_descriptions.push_back(s);

                RCLCPP_INFO(LOGGER, "Hole %zu radius is: %f", count+1, r[count]);
            }

            count++;
            store_vector.push_back(cloud_vector[i]);
        // }

    }

    cloud_vector = store_vector;
    std::vector<std::vector<uint8_t>> cluster_colors;

    if(cloud_vector.size() >0 && debug){

        // Generate cluster colors
        for (size_t i = 0; i < cloud_vector.size(); ++i) {
            // Lighten the blue color based on its index
            double brightness_factor = static_cast<double>(i) / cloud_vector.size();
            std::vector<uint8_t> blue_color = {0, 0, static_cast<uint8_t>( 255 - brightness_factor * 230 )};
            cluster_colors.push_back(blue_color);
        }


        // Color the first cloud to green
        for (size_t i = 0; i < cloud_vector[0]->points.size(); ++i) {
            cloud_vector[0]->points[i].r = 0;  
            cloud_vector[0]->points[i].g = 255;  
            cloud_vector[0]->points[i].b = 0;  
        }

        // Color the other clouds to blue
        for (size_t i = 1; i < cloud_vector.size(); ++i) {

            std::vector<uint8_t> color = cluster_colors[i];
                for (auto& point : cloud_vector[i]->points) {
                        point.r = color[0];
                        point.g = color[1];
                        point.b = color[2];
                    }
        }
    }
}

// Mark the center of find
void HolePerceptionAction::marker(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & plate_cloud, std::vector<float>& xc, std::vector<float>& yc, float zc, std::vector<float>& r) {
    process_descriptions.push_back("Start publish marker");

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> store_vector;
    store_vector.push_back(plate_cloud);
    
    // hole_projection(store_vector, plate_cloud);
    // std::vector<float> xc_p, yc_p, r_p;
    // float zc_p;   
    // estimateCircleParams(store_vector, xc_p, yc_p, zc_p ,r_p, false);

    // Create marker messages for each hole position
    visualization_msgs::msg::MarkerArray marker_array;
    // Find centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*plate_cloud, centroid);

    visualization_msgs::msg::Marker marker_center;
    marker_center.header.frame_id = "base_link";
    marker_center.ns = "center"; // Namespace with index
    marker_center.type = visualization_msgs::msg::Marker::SPHERE;
    marker_center.action = visualization_msgs::msg::Marker::ADD;
    marker_center.scale.x = 0.01; // Adjust scale as needed
    marker_center.scale.y = 0.01;
    marker_center.scale.z = 0.01;
    marker_center.id = 100;
    marker_center.color.r = 1.0;
    marker_center.color.g = 0.0;
    marker_center.color.b = 0.0;
    marker_center.color.a = 1.0;
    
    // RCLCPP_INFO(LOGGER, "Plate center is: (%f, %f, %f)", xc_p[0], yc_p[0],  zc_p);

    // marker_center.pose.position.x = xc_p[0];
    // marker_center.pose.position.y = yc_p[0];
    // marker_center.pose.position.z = -0.5325;

    marker_center.pose.position.x = centroid.x();
    marker_center.pose.position.y = centroid.y();
    marker_center.pose.position.z = centroid.z();
    marker_array.markers.push_back(marker_center);

    for (size_t i = 0; i < xc.size(); ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.ns = "holes_" + std::to_string(i); // Namespace with index
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.01; // Adjust scale as needed
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        marker.id = i;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker.pose.position.x = xc[i];
        marker.pose.position.y = yc[i];
        marker.pose.position.z = centroid.z();

        marker_array.markers.push_back(marker);
        
        //For action Result
        geometry_msgs::msg::Point point;
        point.x = xc[i];
        point.y = yc[i];
        point.z = centroid.z();
        holes_position_.push_back(point);
        radius_.push_back(r[i]);
    }
    
    marker_pub_->publish(marker_array);
    process_descriptions.push_back("Finish publish marker");

}


};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(Coffee_Service_Perception::HolePerceptionAction)

