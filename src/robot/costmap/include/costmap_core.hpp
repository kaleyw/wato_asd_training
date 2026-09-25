#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    // Sets the grid size and inflation settings. Call once before updating.
    void initCostmap(double resolution, int width, int height,
                     double inflation_radius, int max_cost);

    // Rebuilds the whole costmap from one laser scan
    void updateFromScan(const sensor_msgs::msg::LaserScan& scan);

    const nav_msgs::msg::OccupancyGrid& getCostmap() const { return costmap_; }

  private:
    // Converts a point in meters (robot frame) to a grid cell. Returns false if it's off the grid.
    bool convertToGrid(double x, double y, int& col, int& row) const;
    void inflateObstacles(const std::vector<std::pair<int, int>>& obstacles);

    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid costmap_;
    double inflation_radius_ = 1.0;
    int max_cost_ = 100;
};

}

#endif
