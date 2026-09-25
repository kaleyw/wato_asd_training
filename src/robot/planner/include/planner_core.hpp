#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <functional>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

// ------------------- Supporting Structures -------------------

// 2D grid index
struct CellIndex
{
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex &other) const
  {
    return (x == other.x && y == other.y);
  }

  bool operator!=(const CellIndex &other) const
  {
    return (x != other.x || y != other.y);
  }
};

// Hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const
  {
    // A simple hash combining x and y
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

// Structure representing a node in the A* open set
struct AStarNode
{
  CellIndex index;
  double f_score;  // f = g + h

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

// Comparator for the priority queue (min-heap by f_score)
struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b)
  {
    // We want the node with the smallest f_score on top
    return a.f_score > b.f_score;
  }
};

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    // obstacle_threshold: cells costing more than this are treated as walls.
    // cost_weight: how strongly the path avoids cells that are merely close to obstacles.
    void setParams(int obstacle_threshold, double cost_weight);

    // Runs A* on the map from start to goal (world meters).
    // On success, fills path with world points from start to goal and returns true.
    bool planPath(const nav_msgs::msg::OccupancyGrid& map,
                  double start_x, double start_y, double goal_x, double goal_y,
                  std::vector<std::pair<double, double>>& path);

  private:
    bool worldToGrid(const nav_msgs::msg::OccupancyGrid& map, double x, double y, CellIndex& cell) const;
    int cellCost(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& cell) const;
    bool isBlocked(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& cell) const;

    rclcpp::Logger logger_;
    int obstacle_threshold_ = 50;
    double cost_weight_ = 3.0;
};

}

#endif
