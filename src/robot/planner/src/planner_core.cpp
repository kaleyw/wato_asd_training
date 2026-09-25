#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <unordered_set>

#include "planner_core.hpp"

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
: logger_(logger) {}

void PlannerCore::setParams(int obstacle_threshold, double cost_weight) {
  obstacle_threshold_ = obstacle_threshold;
  cost_weight_ = cost_weight;
}

bool PlannerCore::worldToGrid(const nav_msgs::msg::OccupancyGrid& map, double x, double y,
                              CellIndex& cell) const {
  const auto& info = map.info;
  cell.x = static_cast<int>(std::floor((x - info.origin.position.x) / info.resolution));
  cell.y = static_cast<int>(std::floor((y - info.origin.position.y) / info.resolution));
  return cell.x >= 0 && cell.x < static_cast<int>(info.width) &&
         cell.y >= 0 && cell.y < static_cast<int>(info.height);
}

int PlannerCore::cellCost(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& cell) const {
  int value = map.data[cell.y * map.info.width + cell.x];
  // Unknown space is treated as free. If something is there, the map will update
  // once the robot gets close enough to see it, and we'll replan around it.
  return value < 0 ? 0 : value;
}

bool PlannerCore::isBlocked(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& cell) const {
  return cellCost(map, cell) > obstacle_threshold_;
}

bool PlannerCore::planPath(const nav_msgs::msg::OccupancyGrid& map,
                           double start_x, double start_y, double goal_x, double goal_y,
                           std::vector<std::pair<double, double>>& path) {
  path.clear();
  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  const double resolution = map.info.resolution;

  CellIndex start, goal;
  if (!worldToGrid(map, start_x, start_y, start) || !worldToGrid(map, goal_x, goal_y, goal)) {
    RCLCPP_WARN(logger_, "Start or goal is outside the map");
    return false;
  }
  if (isBlocked(map, goal)) {
    RCLCPP_WARN(logger_, "Goal is inside or too close to an obstacle");
    return false;
  }
  // The start is allowed to be blocked: the robot may already be close to a wall,
  // and it still needs a path out.

  // h(n): straight-line distance to the goal. It never overestimates the real cost,
  // which is what guarantees A* finds the cheapest path.
  auto heuristic = [&](const CellIndex& c) {
    return std::hypot(c.x - goal.x, c.y - goal.y) * resolution;
  };

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;       // cheapest known cost to reach each cell
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;  // which cell we came from, to rebuild the path
  std::unordered_set<CellIndex, CellIndexHash> closed;                // cells already finalized

  g_score[start] = 0.0;
  open.emplace(start, heuristic(start));

  // 8 neighbours: 4 straight, 4 diagonal
  const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  bool found = false;
  while (!open.empty()) {
    CellIndex current = open.top().index;
    open.pop();

    // The same cell can be in the queue several times with different scores; only the
    // first (cheapest) time it comes out counts
    if (closed.count(current)) {
      continue;
    }
    closed.insert(current);

    if (current == goal) {
      found = true;
      break;
    }

    for (int i = 0; i < 8; ++i) {
      CellIndex next(current.x + dx[i], current.y + dy[i]);
      if (next.x < 0 || next.x >= width || next.y < 0 || next.y >= height) {
        continue;
      }
      if (closed.count(next) || isBlocked(map, next)) {
        continue;
      }
      // Don't cut diagonally between two blocked cells: the robot would clip the corner
      if (dx[i] != 0 && dy[i] != 0 &&
          (isBlocked(map, CellIndex(current.x + dx[i], current.y)) ||
           isBlocked(map, CellIndex(current.x, current.y + dy[i])))) {
        continue;
      }

      // g(n): distance travelled, made more expensive the closer the cell is to an obstacle
      double step = std::hypot(dx[i], dy[i]) * resolution;
      double penalty = 1.0 + cost_weight_ * cellCost(map, next) / 100.0;
      double tentative_g = g_score[current] + step * penalty;

      auto it = g_score.find(next);
      if (it == g_score.end() || tentative_g < it->second) {
        g_score[next] = tentative_g;
        came_from[next] = current;
        open.emplace(next, tentative_g + heuristic(next));
      }
    }
  }

  if (!found) {
    RCLCPP_WARN(logger_, "A* found no path to the goal");
    return false;
  }

  // Walk backwards from the goal to the start, then flip it around
  std::vector<CellIndex> cells;
  for (CellIndex c = goal; c != start; c = came_from[c]) {
    cells.push_back(c);
  }
  cells.push_back(start);
  std::reverse(cells.begin(), cells.end());

  // Grid cell -> world meters (center of the cell)
  for (const auto& c : cells) {
    double x = map.info.origin.position.x + (c.x + 0.5) * resolution;
    double y = map.info.origin.position.y + (c.y + 0.5) * resolution;
    path.emplace_back(x, y);
  }
  return true;
}

}
