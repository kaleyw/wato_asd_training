#include <algorithm>
#include <cmath>
#include <vector>

#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

void MapMemoryCore::initMap(double resolution, int width, int height,
                            double origin_x, double origin_y, const std::string& frame_id) {
  map_.header.frame_id = frame_id;
  map_.info.resolution = resolution;
  map_.info.width = width;
  map_.info.height = height;
  map_.info.origin.position.x = origin_x;
  map_.info.origin.position.y = origin_y;
  map_.info.origin.orientation.w = 1.0;
  map_.data.assign(width * height, UNKNOWN);
}

void MapMemoryCore::integrateCostmap(const nav_msgs::msg::OccupancyGrid& costmap,
                                     double robot_x, double robot_y, double robot_yaw) {
  const auto& c_info = costmap.info;
  const auto& m_info = map_.info;
  const int map_width = static_cast<int>(m_info.width);
  const int map_height = static_cast<int>(m_info.height);
  const double cos_yaw = std::cos(robot_yaw);
  const double sin_yaw = std::sin(robot_yaw);

  // The map's cells are bigger than the costmap's, so several costmap cells land in the same
  // map cell. Collect this costmap's contribution first, keeping the highest (most cautious)
  // value per map cell, then write it into the map.
  std::vector<int8_t> incoming(map_.data.size(), UNKNOWN);

  for (int row = 0; row < static_cast<int>(c_info.height); ++row) {
    for (int col = 0; col < static_cast<int>(c_info.width); ++col) {
      int8_t value = costmap.data[row * c_info.width + col];
      if (value == UNKNOWN) {
        continue;  // the costmap didn't see this cell, so it has nothing to say about it
      }

      // Center of this cell in the robot's frame (meters)
      double local_x = c_info.origin.position.x + (col + 0.5) * c_info.resolution;
      double local_y = c_info.origin.position.y + (row + 0.5) * c_info.resolution;

      // Rotate by the robot's heading, then shift by the robot's position: robot frame -> world
      double world_x = robot_x + local_x * cos_yaw - local_y * sin_yaw;
      double world_y = robot_y + local_x * sin_yaw + local_y * cos_yaw;

      // World meters -> map cell (same idea as the costmap's convertToGrid)
      int map_col = static_cast<int>(std::floor((world_x - m_info.origin.position.x) / m_info.resolution));
      int map_row = static_cast<int>(std::floor((world_y - m_info.origin.position.y) / m_info.resolution));
      if (map_col < 0 || map_col >= map_width || map_row < 0 || map_row >= map_height) {
        continue;
      }

      int8_t& slot = incoming[map_row * map_width + map_col];
      slot = std::max(slot, value);
    }
  }

  // New data wins over old wherever the costmap saw something; unseen cells keep their memory
  for (size_t i = 0; i < map_.data.size(); ++i) {
    if (incoming[i] != UNKNOWN) {
      map_.data[i] = incoming[i];
    }
  }
}

}
