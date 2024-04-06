#include "coffee_service_perception/hole_perception_estimate_center.h"

namespace Coffee_Service_Perception {

static const rclcpp::Logger LOGGER = rclcpp::get_logger("Hole perception estimate center");


EstimateCenter::EstimateCenter(rclcpp::Node::SharedPtr node){
    
    // Store clock
    clock_ = node->get_clock();

}


  // Function to estimate circle parameters using least squares fitting
void EstimateCenter::estimateCircleParams(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& cloud_vector, std::vector<float>& xc, std::vector<float>& yc, std::vector<float>&  zc, std::vector<float>& r, 
                                         std::vector<std::string> & process_descriptions) {
    process_descriptions.push_back("Estimate hole center position and radius of each hole");

    for (size_t i = 0; i < cloud_vector.size(); ++i) {

        pcl::PointXYZRGB min_pt, max_pt;
        pcl::getMinMax3D(*cloud_vector[i], min_pt, max_pt);
        zc.push_back((2*min_pt.z + 0.10)/2.0); //cup height = 0.14
    }
    
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

            
            // RCLCPP_INFO(LOGGER, "Hole %zu center is: (%f, %f, %f)", i+1, xc[i]+ 13.9 - 0.01,  yc[i]-18.56, zc+1.032); // for cup with cover
            // RCLCPP_INFO(LOGGER, "Hole %zu center in gazebo is: (%f, %f, %f)", count+1, xc[count]+ 13.9 - 0.04- 0.01,  yc[count]-18.56 + 0.04, zc+1.032); // for cup without cover, this value is related to the radius 0.04, 0.01 is the deviation
            // RCLCPP_INFO(LOGGER, "Hole %zu center in gazebo is: (%f, %f, %f)", count+1, xc[count]- 0.01,  yc[count], zc+1.032); // for cup without cover, this value is related to the inertia pose we set

            RCLCPP_INFO(LOGGER, "Hole %zu center is: (%f, %f, %f)", count+1, xc[count],  yc[count], zc[count]);
            RCLCPP_INFO(LOGGER, "Hole %zu radius is: %f", count+1, r[count]);

            //action feedback
            std::stringstream ss;
            ss << "Hole " << (count + 1) << " center is: (" << xc[count] << ", " << yc[count] << ", " << zc[count] << ")";
            std::string s = ss.str();
            process_descriptions.push_back(s);
            count++;
            store_vector.push_back(cloud_vector[i]);
        // }

    }

    cloud_vector = store_vector;
    std::vector<std::vector<uint8_t>> cluster_colors;

    if(cloud_vector.size() >0){

        // Generate cluster colors
        for (size_t i = 0; i < cloud_vector.size(); ++i) {
            // Lighten the blue color based on its index
            double brightness_factor = static_cast<double>(i) / cloud_vector.size();
            std::vector<uint8_t> blue_color = {0, 0, static_cast<uint8_t>( 25 + brightness_factor * 230 )};
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
                    for (size_t j = 0; j < cloud_vector[i]->points.size(); ++j) {
                        cloud_vector[i]->points[j].r = color[0];
                        cloud_vector[i]->points[j].g = color[1];
                        cloud_vector[i]->points[j].b = color[2];
                    }
        }
    }
}

// Mark the center of find
void EstimateCenter::marker(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & plate_cloud, std::vector<std::string> & process_descriptions,
                            std::vector<float>& xc, std::vector<float>& yc, std::vector<float>& zc, std::vector<float>& r, geometry_msgs::msg::Point & plate_center,
                            visualization_msgs::msg::MarkerArray & marker_array) {

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> store_vector;
    store_vector.push_back(plate_cloud);
    
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

    marker_center.pose.position.x = centroid.x();
    marker_center.pose.position.y = centroid.y();
    marker_center.pose.position.z = centroid.z();
    marker_array.markers.push_back(marker_center);
    plate_center.x = centroid.x();
    plate_center.y = centroid.y();
    plate_center.z = centroid.z();

    //action feedback
    std::stringstream ss;
    ss << "Plate Center is (" << centroid.x() << ", " << centroid.y() << ", " << centroid.z() << ")";
    std::string s = ss.str();
    process_descriptions.push_back(s);

    for (size_t i = 0; i < xc.size(); ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.ns = "holes_center_" + std::to_string(i); // Namespace with index
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
        
    }

    for (size_t i = 0; i < xc.size(); ++i) {

        // marker_array.markers.push_back(marker);
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.ns = "holes_" + std::to_string(i); // Namespace with index
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.id = i;
        marker.color.r = 0.0;
        marker.color.g = 0.4;
        marker.color.b = 0.9;
        marker.color.a = 1.0; // Make sure to set the alpha to something non-zero to make it visible.

        marker.scale.x = 0.01; // Thickness of the line

        // Define the number of points around the circle
        const int points = 72;
        const float angle_step = 2.0 * M_PI / points;

        // Calculate points around the circle and add them to the marker
        for (int j = 0; j <= points; ++j) { // Use <= to close the circle
            float angle = j * angle_step;
            float x = xc[i] + r[i] * cos(angle);
            float y = yc[i] + r[i] * sin(angle);
            float z = centroid.z(); // Assuming centroid.z is defined elsewhere

            geometry_msgs::msg::Point p;
            p.x = x;
            p.y = y;
            p.z = z;
            marker.points.push_back(p);
        }
        marker_array.markers.push_back(marker);

    }


}


};