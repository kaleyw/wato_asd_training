#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    // Runs on a timer: merges the latest costmap if the robot has moved far enough, then publishes
    void updateMap();

  private:
    robot::MapMemoryCore map_memory_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double update_distance_;
    double max_update_interval_;

    // Where the robot is now
    double robot_x_ = 0.0, robot_y_ = 0.0, robot_yaw_ = 0.0;
    bool have_odom_ = false;

    // The latest costmap, plus where the robot was when it arrived
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    double costmap_x_ = 0.0, costmap_y_ = 0.0, costmap_yaw_ = 0.0;
    bool have_costmap_ = false;

    // Where the robot was the last time the map was updated
    double last_update_x_ = 0.0, last_update_y_ = 0.0;
    rclcpp::Time last_update_time_;
    bool updated_once_ = false;
    bool should_update_ = false;
};

#endif
