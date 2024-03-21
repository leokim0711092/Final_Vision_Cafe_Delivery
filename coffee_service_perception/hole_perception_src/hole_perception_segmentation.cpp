#include "coffee_service_perception/hole_perception_segmentation.h"

namespace Coffee_Service_Perception {

static const rclcpp::Logger LOGGER = rclcpp::get_logger("Hole Plate Segmentation");

Segmentation::Segmentation(rclcpp::Node::SharedPtr node){
    
    // Store clock
    clock_ = node->get_clock();

    //   voxel grid the data before segmenting
    double leaf_size = 0.005;
    voxel_grid_.setLeafSize(leaf_size, leaf_size, leaf_size);

    //segment plate
    segment_plate.setOptimizeCoefficients(true);
    segment_plate.setModelType(pcl::SACMODEL_PLANE);
    segment_plate.setNormalDistanceWeight(0.005);
    segment_plate.setMaxIterations(100);
    segment_plate.setDistanceThreshold(0.01);

}

//Segment the plate and hole
bool Segmentation::segment(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_input,
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr plate_cloud,
             pcl::PointCloud<pcl::PointXYZRGB>::Ptr hole_cloud,
             std::vector<std::string> & process_descriptions) {

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


};


