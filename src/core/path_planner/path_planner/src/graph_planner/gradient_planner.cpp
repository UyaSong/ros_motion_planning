/**
 * *********************************************************
 *
 * @file: gradient_planner.cpp
 * @brief: Contains the Gradient-based planner class
 * @author: Yang Haodong
 * @date: 2025-3-14
 * @version: 1.0
 *
 * Copyright (c) 2025, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <queue>
#include <vector>
#include <unordered_set>

#include <costmap_2d/cost_values.h>

#include "path_planner/graph_planner/gradient_planner.h"

namespace rmp
{
namespace path_planner
{
namespace
{
constexpr int GRIDMAP_FREE_VAL = 0;
constexpr int GRIDMAP_OBSTACLE_VAL = 1;
}  // namespace

std::vector<GradientPathPlanner::Node> GradientPathPlanner::motions_ = {
  { 0, 1, 1.0 },          { 1, 0, 1.0 },           { 0, -1, 1.0 },          { -1, 0, 1.0 },
  { 1, 1, std::sqrt(2) }, { 1, -1, std::sqrt(2) }, { -1, 1, std::sqrt(2) }, { -1, -1, std::sqrt(2) },
};

/**
 * @brief Construct a new AStar object
 * @param costmap   the environment for path planning
 * @param dijkstra   using diksktra implementation
 * @param gbfs       using gbfs implementation
 */
GradientPathPlanner::GradientPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, double obstacle_factor)
  : PathPlanner(costmap_ros, obstacle_factor)
  , grid_map_(costmap_ros->getCostmap()->getSizeInCellsY(), costmap_ros->getCostmap()->getSizeInCellsX()){};

/**
 * @brief A* implementation
 * @param start          start node
 * @param goal           goal node
 * @param path           optimal path consists of Node
 * @param expand         containing the node been search during the process
 * @return true if path found, else false
 */
bool GradientPathPlanner::plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand)
{
  double m_start_x, m_start_y, m_goal_x, m_goal_y;
  if ((!validityCheck(start.x(), start.y(), m_start_x, m_start_y)) ||
      (!validityCheck(goal.x(), goal.y(), m_goal_x, m_goal_y)))
  {
    return false;
  }

  if (!is_grid_map_valid_)
  {
    unsigned char* char_map = costmap_ros_->getCostmap()->getCharMap();
    for (size_t i = 0; i < map_size_; ++i)
    {
      int x, y;
      index2Grid(i, x, y);
      grid_map_(y, x) = char_map[i] >= costmap_2d::LETHAL_OBSTACLE * 0.5 ? GRIDMAP_OBSTACLE_VAL : GRIDMAP_FREE_VAL;
    }
    is_grid_map_valid_ = true;
  }

  if (goal_ != goal)
  {
    if (!_calObstacleHeuristic(grid_map_, Point3d(m_goal_x, m_goal_y), obstacle_hmap_))
    {
      R_WARN << "Calculate obstacle heuristic map failed.";
      return false;
    }
    goal_ = goal;
  }

  // clear vector
  path.clear();
  expand.clear();

  unsigned int sx = static_cast<unsigned int>(m_start_x);
  unsigned int sy = static_cast<unsigned int>(m_start_y);
  unsigned int gx = static_cast<unsigned int>(m_goal_x);
  unsigned int gy = static_cast<unsigned int>(m_goal_y);

  if (obstacle_hmap_(sy, sx) > 0)
  {
    path.push_back(start);
    unsigned int curr_mx = sx;
    unsigned int curr_my = sy;

    while (!(curr_mx == gx && curr_my == gy))
    {
      unsigned int min_mx = curr_mx;
      unsigned int min_my = curr_my;
      float curr_cost = obstacle_hmap_(curr_my, curr_mx);

      for (const auto& motion : motions_)
      {
        int new_mx = static_cast<int>(curr_mx) + motion.x();
        int new_my = static_cast<int>(curr_my) + motion.y();

        if (new_mx >= 0 && new_mx < nx_ && new_my >= 0 && new_my < ny_)
        {
          float new_cost = obstacle_hmap_(new_my, new_mx);
          if (new_cost >= 0 && new_cost < curr_cost)
          {
            curr_cost = new_cost;
            min_mx = new_mx;
            min_my = new_my;
          }
        }
      }

      curr_mx = min_mx;
      curr_my = min_my;
      double wx, wy;
      costmap_->mapToWorld(curr_mx, curr_my, wx, wy);
      path.emplace_back(wx, wy);
    }
    return true;
  }
  return false;
}

bool GradientPathPlanner::_calObstacleHeuristic(const Eigen::MatrixXi& grid_map, const Point3d& goal,
                                                Eigen::MatrixXf& obstacle_hmap)
{
  const int goal_x = static_cast<int>(goal.x());
  const int goal_y = static_cast<int>(goal.y());
  const int height = grid_map.rows();
  const int width = grid_map.cols();

  if (grid_map(goal_y, goal_x) != GRIDMAP_FREE_VAL)
  {
    R_ERROR << "Goal not in free space";
    return false;
  }

  // initialize heurisitics
  const auto& mask = (grid_map.array() == GRIDMAP_FREE_VAL).eval();
  obstacle_hmap.setConstant(height, width, std::numeric_limits<float>::infinity());
  obstacle_hmap(goal_y, goal_x) = 0.0f;

  // dijkstra queue
  using QueueElement = std::pair<float, std::pair<int, int>>;
  std::priority_queue<QueueElement, std::vector<QueueElement>, std::greater<>> queue;
  queue.emplace(0.0f, std::make_pair(goal_y, goal_x));

  // main loop
  while (!queue.empty())
  {
    const auto node = queue.top();
    float curr_cost = node.first;
    int y = node.second.first;
    int x = node.second.second;
    queue.pop();

    for (const auto& motion : motions_)
    {
      const int ny = y + motion.y();
      const int nx = x + motion.x();

      if (nx < 0 || nx >= width || ny < 0 || ny >= height)
        continue;

      if (!mask(ny, nx))
        continue;

      const float new_cost = curr_cost + motion.g();

      if (new_cost < obstacle_hmap(ny, nx))
      {
        obstacle_hmap(ny, nx) = new_cost;
        queue.emplace(new_cost, std::make_pair(ny, nx));
      }
    }
  }

  obstacle_hmap = obstacle_hmap.unaryExpr([](float v) { return std::isinf(v) ? -1.0f : v; });
  return true;
}

}  // namespace path_planner
}  // namespace rmp
