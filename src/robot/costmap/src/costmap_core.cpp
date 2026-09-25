#include <algorithm>
#include <cmath>

#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

void CostmapCore::initCostmap(double resolution, int width, int height,
                              double inflation_radius, int max_cost) {
  inflation_radius_ = inflation_radius;
  max_cost_ = max_cost;

  costmap_.info.resolution = resolution;
  costmap_.info.width = width;
  costmap_.info.height = height;

  // Put the robot in the middle: the bottom-left corner is half the grid behind and to the right
  costmap_.info.origin.position.x = -width * resolution / 2.0;
  costmap_.info.origin.position.y = -height * resolution / 2.0;
  costmap_.info.origin.orientation.w = 1.0;

  costmap_.data.assign(width * height, 0);
}

void CostmapCore::updateFromScan(const sensor_msgs::msg::LaserScan& scan) {
  // Start from a blank grid every scan, so obstacles that moved away don't linger
  std::fill(costmap_.data.begin(), costmap_.data.end(), 0);

  std::vector<std::pair<int, int>> obstacles;
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    double range = scan.ranges[i];
    // Skip beams that hit nothing (inf) or gave garbage readings
    if (!(range > scan.range_min && range < scan.range_max)) {
      continue;
    }

    double angle = scan.angle_min + i * scan.angle_increment;
    double x = range * std::cos(angle);
    double y = range * std::sin(angle);

    int col, row;
    if (convertToGrid(x, y, col, row)) {
      costmap_.data[row * costmap_.info.width + col] = max_cost_;
      obstacles.emplace_back(col, row);
    }
  }

  inflateObstacles(obstacles);
}

bool CostmapCore::convertToGrid(double x, double y, int& col, int& row) const {
  const auto& info = costmap_.info;
  // floor, not round: cell 20 covers everything from 20.0 up to just under 21.0
  col = static_cast<int>(std::floor((x - info.origin.position.x) / info.resolution));
  row = static_cast<int>(std::floor((y - info.origin.position.y) / info.resolution));

  return col >= 0 && col < static_cast<int>(info.width) &&
         row >= 0 && row < static_cast<int>(info.height);
}

void CostmapCore::inflateObstacles(const std::vector<std::pair<int, int>>& obstacles) {
  const auto& info = costmap_.info;
  const int width = static_cast<int>(info.width);
  const int height = static_cast<int>(info.height);
  const int radius_cells = static_cast<int>(std::ceil(inflation_radius_ / info.resolution));

  for (const auto& [ox, oy] : obstacles) {
    // Only look at the square of cells around the obstacle that could be within the radius
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        int col = ox + dx;
        int row = oy + dy;
        if (col < 0 || col >= width || row < 0 || row >= height) {
          continue;
        }

        double distance = std::hypot(dx, dy) * info.resolution;
        if (distance > inflation_radius_) {
          continue;
        }

        int cost = static_cast<int>(max_cost_ * (1.0 - distance / inflation_radius_));
        int8_t& cell = costmap_.data[row * width + col];
        // Never lower a cell: a nearby obstacle may already have given it a higher cost
        if (cost > cell) {
          cell = static_cast<int8_t>(cost);
        }
      }
    }
  }
}

}
