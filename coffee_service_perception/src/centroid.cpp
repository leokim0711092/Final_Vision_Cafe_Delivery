  void hole_position(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_input, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &cloud_vector, std::vector<geometry_msgs::msg::Point> &point_vector){
    
    pcl::PointXYZRGB min_pt, max_pt;
    pcl::getMinMax3D(*cloud_vector[0], min_pt, max_pt);

    Eigen::Vector4f centroid;


    for (size_t i = 0; i < cloud_vector.size(); i++) {
        pcl::compute3DCentroid(*cloud_vector[i], centroid);
        geometry_msgs::msg::Point point;
        point.x = centroid.x();
        point.y = centroid.y();
        point.z = (2*max_pt.z + 0.1)/2.0;
        point_vector.push_back(point);  

        RCLCPP_INFO(LOGGER, "Hole %zu X Y Z is: (%f, %f, %f)", i+1, point.x+13.9, point.y-18.56, point.z+1.032);

    }



  } 