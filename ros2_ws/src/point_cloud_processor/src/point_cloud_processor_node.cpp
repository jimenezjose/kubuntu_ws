#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp" 
#include "sensor_msgs/msg/point_cloud2.hpp"

/*
 * Node to practice processing point cloud data.
 */ 
class PointCloudProcessorNode : public rclcpp::Node {

public:
  PointCloudProcessorNode() : Node("point_cloud_processor_node") {
    point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/kinect/points", rclcpp::SensorDataQoS(), std::bind(&PointCloudProcessorNode::point_cloud_processor_callback, this, std::placeholders::_1));
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;

  void point_cloud_processor_callback(sensor_msgs::msg::PointCloud2::SharedPtr pointcloud) const {
    RCLCPP_INFO(this->get_logger(), "Received a Point Cloud 3D scan!, [%d, %d]", pointcloud->height, pointcloud->width);
  }

};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudProcessorNode>());
  rclcpp::shutdown();
  return 0;
}