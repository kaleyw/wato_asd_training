#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    // Runs at 10 Hz: computes and sends the next velocity command
    void controlLoop();

  private:
    robot::ControlCore control_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    std::vector<std::pair<double, double>> path_;
    double robot_x_ = 0.0, robot_y_ = 0.0, robot_yaw_ = 0.0;
    bool have_odom_ = false;
    // True once we've told the robot to stop, so we send the stop only once
    bool stopped_ = true;
};

#endif
