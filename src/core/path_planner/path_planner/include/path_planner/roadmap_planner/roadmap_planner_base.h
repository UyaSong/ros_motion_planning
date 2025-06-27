/**
 * *********************************************************
 *
 * @file: roadmap_planner_base.h
 * @brief: Contains the roadmap-based planner top class
 * @author: Yang Haodong
 * @date: 2025-3-7
 * @version: 1.0
 *
 * Copyright (c) 2025, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_PATH_PLANNER_GRAPH_PLANNER_ROADMAP_PLANNER_BASE_H
#define RMP_PATH_PLANNER_GRAPH_PLANNER_ROADMAP_PLANNER_BASE_H

#include "common/geometry/vec2d.h"
#include "common/geometry/point.h"
#include "common/geometry/collision_checker.h"

#include "path_planner/path_planner.h"

namespace rmp
{
namespace path_planner
{
/**
 * @brief Class for objects that plan using the roadmap-based algorithm
 */
class RoadmapPathPlannerBase : public PathPlanner
{
public:
  /**
   * @brief Construct a new the roadmap-based planner top class
   * @param costmap   the environment for path planning
   * @param obstacle_factor obstacle factor(greater means obstacles)
   */
  RoadmapPathPlannerBase(costmap_2d::Costmap2DROS* costmap_ros, double obstacle_factor)
    : PathPlanner(costmap_ros, obstacle_factor){};

  /**
   * @brief Pure virtual function that is overloadde by planner implementations
   * @param start          start node
   * @param goal           goal node
   * @param path           optimal path consists of Node
   * @param expand         containing the node been search during the process
   * @return true if path found, else false
   */
  virtual bool plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand) = 0;

protected:
  using RoadMap = std::vector<std::vector<std::pair<int, double>>>;
  using Edge = std::pair<rmp::common::geometry::Vec2d, rmp::common::geometry::Vec2d>;

protected:
  /**
   * @brief Dijkstra's algorithm implementation for road map search
   * @param road_map The graph structure represented by an adjacency list, where each element is a pair (target node
   * index, edge weight)
   * @param start_idx The index of the starting node in road_map
   * @param goal_idx The index of the goal node in road_map
   * @param path The output sequence of node indices in the path (from start to goal)
   * @return flag true if path found successfully
   */
  bool _searchOnRoadMap(const RoadMap& road_map, int start_idx, int goal_idx, std::vector<int>& path)
  {
    const double INF = std::numeric_limits<double>::infinity();
    int node_count = static_cast<int>(road_map.size());
    std::vector<double> dist(node_count, INF);
    std::vector<int> prev(node_count, -1);
    using QueueNode = std::pair<double, int>;
    std::priority_queue<QueueNode, std::vector<QueueNode>, std::greater<>> OPEN;
    dist[start_idx] = 0;
    OPEN.emplace(0.0, start_idx);

    // dijkstra loop
    while (!OPEN.empty())
    {
      auto node = OPEN.top();
      double curr_dist = node.first;
      int u = node.second;
      OPEN.pop();

      // goal found
      if (u == goal_idx)
      {
        break;
      }

      if (curr_dist > dist[u])
      {
        continue;
      }

      // bfs
      for (const auto& road_node : road_map[u])
      {
        int v = road_node.first;
        double d = road_node.second;
        double new_cost = dist[u] + d;
        if (new_cost < dist[v])
        {
          dist[v] = new_cost;
          prev[v] = u;
          OPEN.emplace(new_cost, v);
        }
      }
    }

    // backtrace
    if (dist[goal_idx] == INF)
      return false;

    path.clear();
    for (int current = goal_idx; current != -1; current = prev[current])
    {
      path.push_back(current);
    }
    std::reverse(path.begin(), path.end());

    return true;
  }

  /**
   * @brief Check if there is any obstacle between two points using Bresenham's line algorithm
   * @param pt1_x x-coordinate of the first point
   * @param pt1_y y-coordinate of the first point
   * @param pt2_x x-coordinate of the second point
   * @param pt2_y y-coordinate of the second point
   * @return true if collision occurs, else false
   */
  bool _isCollision(int pt1_x, int pt1_y, int pt2_x, int pt2_y)
  {
    return rmp::common::geometry::CollisionChecker::BresenhamCollisionDetection(
        rmp::common::geometry::Point2i(pt1_x, pt1_y), rmp::common::geometry::Point2i(pt2_x, pt2_y),
        [&](const rmp::common::geometry::Point2i& pt) {
          return costmap_->getCharMap()[grid2Index(pt.x(), pt.y())] >= costmap_2d::LETHAL_OBSTACLE * obstacle_factor_;
        });
  }
};
}  // namespace path_planner
}  // namespace rmp
#endif