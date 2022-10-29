#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp" 
#include "sensor_msgs/msg/point_cloud2.hpp"

// #include <pcl/point_types.h>
#include <pcl/point_cloud.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/ModelCoefficients.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/segmentation/sac_segmentation.h>

// #include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

//#include <vector>

/*
 * Node to practice processing point cloud data.
 */ 
class PointCloudProcessorNode : public rclcpp::Node {

public:
  PointCloudProcessorNode() : Node("point_cloud_processor_node") {
    point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/kinect/points", rclcpp::SensorDataQoS(), std::bind(&PointCloudProcessorNode::point_cloud_processor_callback, this, std::placeholders::_1));
    processed_point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/processed/pointcloud", rclcpp::SensorDataQoS());
    euclidean_cluster_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("input", rclcpp::SensorDataQoS());
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr processed_point_cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr euclidean_cluster_publisher_;

  void point_cloud_processor_callback(sensor_msgs::msg::PointCloud2::SharedPtr pointcloud) const {
    // pcl::PCLPointCloud2 cloud_blob;// = new pcl::PCLPointCloud2ConstPtr();
    // pcl_conversions::toPCL(*pointcloud, cloud_blob);
  // pcl::PCLPointCloud2::Ptr cloud_filtered_blob (new pcl::PCLPointCloud2);
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);

  // // Create the filtering object: downsample the dataset using a leaf size of 1cm
  // pcl::VoxelGrid<sensor_msgs::msg::PointCloud2> sor;
  // const PointCloudConstPtr cloud_blob = new PointCloudConstPtr();
  // sor.setInputCloud (cloud_blob);
  // sor.setLeafSize (0.01f, 0.01f, 0.01f);
  // sor.filter (*cloud_filtered_blob);

  // // Convert to the templated PointCloud
  // pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

  // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  // pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // // Create the segmentation object
  // pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // // Optional
  // seg.setOptimizeCoefficients (true);
  // // Mandatory
  // seg.setModelType (pcl::SACMODEL_PLANE);
  // seg.setMethodType (pcl::SAC_RANSAC);
  // seg.setMaxIterations (1000);
  // seg.setDistanceThreshold (0.02);
  // seg.setInputCloud (cloud_filtered);
    
  // int i = 0, nr_points = (int) cloud_filtered->points.size ();
  // pcl::IndicesPtr remaining (new std::vector<int>);
  // remaining->resize (nr_points);
  // for (size_t i = 0; i < remaining->size (); ++i) { (*remaining)[i] = static_cast<int>(i); }

  // // While 30% of the original cloud is still there
  // while (remaining->size () > 0.5 * nr_points)
  // {
  //   // Segment the largest planar component from the remaining cloud
  //   seg.setIndices (remaining);
  //   seg.segment (*inliers, *coefficients);
  //   if (inliers->indices.size () == 0) break;

  //   // Extract the inliers
  //   std::vector<int>::iterator it = remaining->begin();
  //   for (size_t i = 0; i < inliers->indices.size (); ++i)
  //   {
  //     int curr = inliers->indices[i];
  //     // Remove it from further consideration.
  //     while (it != remaining->end() && *it < curr) { ++it; }
  //     if (it == remaining->end()) break;
  //     if (*it == curr) it = remaining->erase(it);
  //   }
  //   i++;
  // }
  // std::cout << "Found " << i << " planes." << std::endl;

  // // Color all the non-planar things.
  // for (std::vector<int>::iterator it = remaining->begin(); it != remaining->end(); ++it)
  // {
  //   uint8_t r = 0, g = 255, b = 0;
  //   uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
  //   cloud_filtered->at(*it).rgb = *reinterpret_cast<float*>(&rgb);
  // }

  // // Publish the planes we found.
  // pcl::PCLPointCloud2 outcloud;
  // pcl::toPCLPointCloud2 (*cloud_filtered, outcloud);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::fromPCLPointCloud2(outcloud,*temp_cloud);
  // sensor_msgs::msg::PointCloud2 output;
  // pcl::toROSMsg<pcl::PointXYZ>(*temp_cloud, output);
  
  // processed_point_cloud_publisher_.publish (output);










    RCLCPP_INFO(this->get_logger(), "Received a Point Cloud 3D scan!, [%d, %d]", pointcloud->height, pointcloud->width);
    processed_point_cloud_publisher_->publish(*pointcloud);
    euclidean_cluster_publisher_->publish(*pointcloud);

    // convert ros to pcl
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pointcloud, *cloud);


    std::cerr << "Point cloud data: " << cloud->size () << " points" << std::endl;
  for (const auto& point: *cloud)
    std::cerr << "    " << point.x << " "
                        << point.y << " "
                        << point.z << std::endl;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  for (const auto& idx: inliers->indices)
    std::cerr << idx << "    " << cloud->points[idx].x << " "
                               << cloud->points[idx].y << " "
                               << cloud->points[idx].z << std::endl;

  }


  

};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudProcessorNode>());
  rclcpp::shutdown();
  return 0;
}