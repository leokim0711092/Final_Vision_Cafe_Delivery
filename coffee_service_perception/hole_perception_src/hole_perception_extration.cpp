#include "coffee_service_perception/hole_perception_extration.h"
#include "rclcpp/logging.hpp"

namespace Coffee_Service_Perception {

static const rclcpp::Logger LOGGER = rclcpp::get_logger("Hole perception extration");


HoleExtration::HoleExtration(rclcpp::Node::SharedPtr node){
    
    // Store clock
    clock_ = node->get_clock();
    // StatisticalOutlierRemoval filter
    outliers_filter.setStddevMulThresh(1.0);


}


// Extract the hole
void HoleExtration::hole_extration( pcl::PointCloud<pcl::PointXYZRGB>::Ptr & plate_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &hole_cloud, 
                                    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& cloud_vector, std::vector<std::string> & process_descriptions){
        
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> cluster_extraction;
    cluster_extraction.setInputCloud(hole_cloud);
    cluster_extraction.setClusterTolerance(0.01); // Set the cluster tolerance (max. distance between points in a cluster)
    cluster_extraction.setMinClusterSize(100);    // Set the minimum number of points required for a cluster
    cluster_extraction.setMaxClusterSize(100000);   // Set the maximum number of points allowed for a cluster
    std::vector<pcl::PointIndices> clusters;
    cluster_extraction.extract(clusters);
                
    RCLCPP_INFO(LOGGER, "Hole cloud size: %zu, Exstract %zu Cluster", hole_cloud->size(), clusters.size());

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
        //action feedback
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
            

            RCLCPP_INFO(LOGGER, "Cluster %zu size: %zu", count+1 ,cloud_vector[count]->size());
            //action feedback
            std::stringstream ss;
            ss << "Cluster " << (count + 1) << " size: " << cloud_vector[count]->size();
            std::string s = ss.str();
            process_descriptions.push_back(s);

            count++;
        }
    }

    RCLCPP_INFO(LOGGER, "Hole extration done");
    process_descriptions.push_back("Hole extration done");

}
  
// Rearrage the order of cloud in vector depending on the distance
void HoleExtration::hole_number(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& cloud_vector, std::vector<std::string> & process_descriptions){
    
    process_descriptions.push_back("Rearrange the clout vector according to hole distance to origin");

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

  }

  // Project the border of hole to 2D plane, then use convex vull to filter border out
void HoleExtration::hole_projection(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& cloud_vector, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &hole_cloud,
                                    std::vector<std::string> & process_descriptions){
    
    process_descriptions.push_back("Hole project to min z");

    pcl::PointXYZRGB Min_pt, Max_pt;
    pcl::getMinMax3D(*hole_cloud, Min_pt, Max_pt);

   for (size_t i = 0; i < cloud_vector.size(); ++i) {

        // pcl::PointXYZRGB Min_pt, Max_pt;
        // pcl::getMinMax3D(*cloud_vector[i], Min_pt, Max_pt);

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
        
        outliers_filter.setMeanK(static_cast<int>(cloud_vector[i]->size()));
        outliers_filter.setInputCloud(cloud_vector[i]);
        outliers_filter.filter(*cloud_vector[i]);
    }

}

};
