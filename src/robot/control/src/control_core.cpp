#include <algorithm>
#include <cmath>
#include <limits>

#include "control_core.hpp"

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

void ControlCore::setParams(double lookahead_distance, double goal_tolerance, double linear_speed,
                            double max_angular_speed, double rotate_in_place_angle) {
  lookahead_distance_ = lookahead_distance;
  goal_tolerance_ = goal_tolerance;
  linear_speed_ = linear_speed;
  max_angular_speed_ = max_angular_speed;
  rotate_in_place_angle_ = rotate_in_place_angle;
}

size_t ControlCore::findLookaheadIndex(const std::vector<std::pair<double, double>>& path,
                                       double robot_x, double robot_y) const {
  // Start from the closest point, so we never chase points the robot has already passed
  size_t closest = 0;
  double closest_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < path.size(); ++i) {
    double d = std::hypot(path[i].first - robot_x, path[i].second - robot_y);
    if (d < closest_dist) {
      closest_dist = d;
      closest = i;
    }
  }

  for (size_t i = closest; i < path.size(); ++i) {
    if (std::hypot(path[i].first - robot_x, path[i].second - robot_y) >= lookahead_distance_) {
      return i;
    }
  }
  // Near the end of the path every point is closer than the lookahead: aim at the last one
  return path.size() - 1;
}

bool ControlCore::computeCommand(const std::vector<std::pair<double, double>>& path,
                                 double robot_x, double robot_y, double robot_yaw,
                                 double& linear, double& angular) const {
  linear = 0.0;
  angular = 0.0;
  if (path.empty()) {
    return false;
  }

  const auto& goal = path.back();
  double dist_to_goal = std::hypot(goal.first - robot_x, goal.second - robot_y);
  if (dist_to_goal < goal_tolerance_) {
    return false;
  }

  const auto& target = path[findLookaheadIndex(path, robot_x, robot_y)];

  // Put the target in the robot's own frame (x = ahead, y = left): the reverse of what
  // map memory does. Shift so the robot is at the origin, then rotate by -yaw.
  double dx = target.first - robot_x;
  double dy = target.second - robot_y;
  double local_x = std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
  double local_y = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;

  // Angle to the target, as the robot sees it: 0 = straight ahead, + = to the left
  double heading_error = std::atan2(local_y, local_x);

  // Pure pursuit assumes the target is roughly ahead. If it's off to the side or behind,
  // an arc would be a huge loop, so turn on the spot to face it first.
  if (std::abs(heading_error) > rotate_in_place_angle_) {
    angular = std::copysign(max_angular_speed_, heading_error);
    return true;
  }

  // The arc from the robot through the target has curvature 2y / L^2 (L = distance to target).
  // Driving at speed v along an arc of curvature k means turning at v * k.
  double dist_sq = local_x * local_x + local_y * local_y;
  double curvature = 2.0 * local_y / dist_sq;

  // Ease off approaching the goal so the robot doesn't overshoot it
  linear = linear_speed_ * std::clamp(dist_to_goal / lookahead_distance_, 0.3, 1.0);
  angular = std::clamp(linear * curvature, -max_angular_speed_, max_angular_speed_);
  return true;
}

}
