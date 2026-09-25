#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);

    void setParams(double lookahead_distance, double goal_tolerance, double linear_speed,
                   double max_angular_speed, double rotate_in_place_angle);

    // Pure pursuit: given the path (world points) and the robot's pose, works out how fast to
    // drive (linear) and turn (angular). Returns false when the robot should stop instead
    // (no path, or already at the end of it).
    bool computeCommand(const std::vector<std::pair<double, double>>& path,
                        double robot_x, double robot_y, double robot_yaw,
                        double& linear, double& angular) const;

  private:
    // Index of the first path point at least lookahead_distance_ away, searching forward
    // from the point closest to the robot
    size_t findLookaheadIndex(const std::vector<std::pair<double, double>>& path,
                              double robot_x, double robot_y) const;

    rclcpp::Logger logger_;
    double lookahead_distance_ = 1.0;
    double goal_tolerance_ = 0.3;
    double linear_speed_ = 0.5;
    double max_angular_speed_ = 1.0;
    double rotate_in_place_angle_ = 1.0;
};

}

#endif
