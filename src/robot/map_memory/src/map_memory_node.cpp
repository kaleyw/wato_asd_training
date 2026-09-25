#include <chrono>
#include <cmath>

#include "map_memory_node.hpp"

namespace
{

// A quaternion describes a 3D rotation. The robot drives on flat ground, so the only
// rotation we care about is yaw: which way it's facing, around the vertical axis.
double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  double resolution = this->declare_parameter("resolution", 0.2);
  int width = this->declare_parameter("width", 160);
  int height = this->declare_parameter("height", 160);
  double origin_x = this->declare_parameter("origin_x", -16.0);
  double origin_y = this->declare_parameter("origin_y", -16.0);
  std::string frame_id = this->declare_parameter("frame_id", std::string("sim_world"));
  update_distance_ = this->declare_parameter("update_distance", 1.5);
  max_update_interval_ = this->declare_parameter("max_update_interval", 5.0);
  last_update_time_ = this->now();
  int update_period_ms = this->declare_parameter("update_period_ms", 1000);

  map_memory_.initMap(resolution, width, height, origin_x, origin_y, frame_id);

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(update_period_ms), std::bind(&MapMemoryNode::updateMap, this));
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  if (!have_odom_) {
    return;  // can't place a costmap on the map without knowing where the robot is
  }
  latest_costmap_ = *msg;
  // Remember where the robot was when this costmap was made, so they get merged together
  costmap_x_ = robot_x_;
  costmap_y_ = robot_y_;
  costmap_yaw_ = robot_yaw_;
  have_costmap_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  robot_yaw_ = yawFromQuaternion(msg->pose.pose.orientation);
  have_odom_ = true;

  // Always take the first costmap right away; after that, only once we've moved far enough
  double distance = std::hypot(robot_x_ - last_update_x_, robot_y_ - last_update_y_);
  if (!updated_once_ || distance >= update_distance_) {
    should_update_ = true;
  }
}

void MapMemoryNode::updateMap() {
  // Also refresh every so often while standing still. Otherwise a bad first snapshot (say,
  // taken while the simulator was still loading) would stick until the robot drove 1.5 m.
  bool stale = updated_once_ && (this->now() - last_update_time_).seconds() >= max_update_interval_;

  if ((should_update_ || stale) && have_costmap_) {
    map_memory_.integrateCostmap(latest_costmap_, costmap_x_, costmap_y_, costmap_yaw_);
    last_update_x_ = costmap_x_;
    last_update_y_ = costmap_y_;
    last_update_time_ = this->now();
    updated_once_ = true;
    should_update_ = false;
    RCLCPP_INFO(this->get_logger(), "Map updated at (%.2f, %.2f)", last_update_x_, last_update_y_);
  }

  // Publish every tick, even without an update, so the planner always has a map to work with
  auto map = map_memory_.getMap();
  map.header.stamp = this->now();
  map_pub_->publish(map);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
