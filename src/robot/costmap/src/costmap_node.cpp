#include <chrono>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Defaults here are used if params.yaml doesn't set them
  double resolution = this->declare_parameter("resolution", 0.1);
  int width = this->declare_parameter("width", 300);
  int height = this->declare_parameter("height", 300);
  double inflation_radius = this->declare_parameter("inflation_radius", 1.0);
  int max_cost = this->declare_parameter("max_cost", 100);

  costmap_.initCostmap(resolution, width, height, inflation_radius, max_cost);

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  costmap_.updateFromScan(*msg);

  auto costmap = costmap_.getCostmap();
  // Same timestamp and frame as the scan: this grid is centered on the lidar at that moment
  costmap.header = msg->header;
  costmap_pub_->publish(costmap);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
