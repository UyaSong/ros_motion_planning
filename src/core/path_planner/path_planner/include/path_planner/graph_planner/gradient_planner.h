/**
 * *********************************************************
 *
 * @file: gradient_planner.h
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
#ifndef RMP_PATH_PLANNER_GRAPH_PLANNER_GRADIENT_H
#define RMP_PATH_PLANNER_GRAPH_PLANNER_GRADIENT_H

#include <Eigen/Dense>

#include "path_planner/path_planner.h"

namespace rmp
{
namespace path_planner
{
/**
 * @brief Class for objects that plan using the Gradient-based algorithm
 */
class GradientPathPlanner : public PathPlanner
{
private:
  using Node = rmp::common::structure::Node<int>;

public:
  /**
   * @brief Construct a new GradientPathPlanner object
   * @param costmap   the environment for path planning
   * @param obstacle_factor obstacle factor(greater means obstacles)
   */
  GradientPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, double obstacle_factor = 1.0);

  /**
   * @brief GradientPathPlanner implementation
   * @param start          start node
   * @param goal           goal node
   * @param path           optimal path consists of Node
   * @param expand         containing the node been search during the process
   * @return true if path found, else false
   */
  bool plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand);

private:
  bool _calObstacleHeuristic(const Eigen::MatrixXi& grid_map, const Point3d& goal, Eigen::MatrixXf& obstacle_hmap);

  //   const std::pair<ShiftParams, ShiftParams>& _getShiftParams(int delta, int dim);

private:
  bool is_grid_map_valid_{ false };
  Eigen::MatrixXi grid_map_;
  Point3d goal_;
  Eigen::MatrixXf obstacle_hmap_;
  static std::vector<Node> motions_;
};
}  // namespace path_planner
}  // namespace rmp
#endif
