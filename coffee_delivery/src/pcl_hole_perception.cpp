#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
int
main ()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr passthrough_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);  
  pcl::PCDReader cloud_reader;
  pcl::PCDWriter cloud_writer;

  std::string path="/home/user/ros2_ws/";
  // Reading the cloud
  cloud_reader.read (path+std::string("output_pcl_file_.pcd"),*cloud);

  // Passthrough cloud
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 0.95);
  //pass.setNegative (true);
  pass.filter(*passthrough_cloud);

  // Write
  cloud_writer.write<pcl::PointXYZRGB>(path+std::string("passthrough.pcd"),*passthrough_cloud, false);


  return (0);
}