#include <chrono>
#include <cmath>

#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  int obstacle_threshold = this->declare_parameter("obstacle_threshold", 50);
  double cost_weight = this->declare_parameter("cost_weight", 3.0);
  goal_tolerance_ = this->declare_parameter("goal_tolerance", 0.5);
  goal_timeout_ = this->declare_parameter("goal_timeout", 90.0);
  planner_.setParams(obstacle_threshold, cost_weight);

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = *msg;
  have_map_ = true;
  // The map may have revealed new obstacles, so plan again from where the robot is now
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_ = *msg;
  goal_start_time_ = this->now();
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  RCLCPP_INFO(this->get_logger(), "New goal: (%.2f, %.2f)", goal_.point.x, goal_.point.y);
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  have_odom_ = true;
}

void PlannerNode::timerCallback() {
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    return;
  }

  if (goalReached()) {
    RCLCPP_INFO(this->get_logger(), "Goal reached!");
    state_ = State::WAITING_FOR_GOAL;
    publishEmptyPath();
  } else if ((this->now() - goal_start_time_).seconds() > goal_timeout_) {
    RCLCPP_WARN(this->get_logger(), "Gave up on goal after %.0f s", goal_timeout_);
    state_ = State::WAITING_FOR_GOAL;
    publishEmptyPath();
  }
}

bool PlannerNode::goalReached() const {
  return std::hypot(goal_.point.x - robot_x_, goal_.point.y - robot_y_) < goal_tolerance_;
}

void PlannerNode::planPath() {
  if (!have_map_ || !have_odom_) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: missing map or robot position");
    return;
  }

  std::vector<std::pair<double, double>> points;
  if (!planner_.planPath(current_map_, robot_x_, robot_y_, goal_.point.x, goal_.point.y, points)) {
    // Keep the old path (if any) rather than stopping; the next map update will try again
    return;
  }

  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = current_map_.header.frame_id;
  for (const auto& [x, y] : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  path_pub_->publish(path);
}

void PlannerNode::publishEmptyPath() {
  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = current_map_.header.frame_id;
  path_pub_->publish(path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
