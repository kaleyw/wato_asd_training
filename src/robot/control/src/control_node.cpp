#include <chrono>
#include <cmath>

#include "control_node.hpp"

namespace
{

// Same as in map memory: the only rotation that matters on flat ground is yaw
double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  double lookahead_distance = this->declare_parameter("lookahead_distance", 1.0);
  double goal_tolerance = this->declare_parameter("goal_tolerance", 0.3);
  double linear_speed = this->declare_parameter("linear_speed", 0.5);
  double max_angular_speed = this->declare_parameter("max_angular_speed", 1.0);
  double rotate_in_place_angle = this->declare_parameter("rotate_in_place_angle", 1.0);
  control_.setParams(lookahead_distance, goal_tolerance, linear_speed,
                     max_angular_speed, rotate_in_place_angle);

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  path_.clear();
  for (const auto& pose : msg->poses) {
    path_.emplace_back(pose.pose.position.x, pose.pose.position.y);
  }
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  robot_yaw_ = yawFromQuaternion(msg->pose.pose.orientation);
  have_odom_ = true;
}

void ControlNode::controlLoop() {
  if (!have_odom_) {
    return;
  }

  double linear, angular;
  bool driving = control_.computeCommand(path_, robot_x_, robot_y_, robot_yaw_, linear, angular);

  // Send the zero command once when stopping, then go quiet, so we don't fight anyone
  // driving the robot by hand while there's no path
  if (!driving && stopped_) {
    return;
  }
  stopped_ = !driving;

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = linear;
  cmd.angular.z = angular;
  cmd_vel_pub_->publish(cmd);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
