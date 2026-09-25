#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    // Creates an empty (all unknown) global map. origin is its bottom-left corner in the world.
    void initMap(double resolution, int width, int height,
                 double origin_x, double origin_y, const std::string& frame_id);

    // Merges a robot-centered costmap into the global map.
    // robot_x/robot_y/robot_yaw: where the costmap's center was in the world when it was made.
    void integrateCostmap(const nav_msgs::msg::OccupancyGrid& costmap,
                          double robot_x, double robot_y, double robot_yaw);

    const nav_msgs::msg::OccupancyGrid& getMap() const { return map_; }

    static constexpr int8_t UNKNOWN = -1;

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid map_;
};

}

#endif
