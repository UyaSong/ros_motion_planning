/**
 * *********************************************************
 *
 * @file: voronoi_planner.h
 * @brief: Contains the Voronoi-based planner class
 * @author: Yang Haodong
 * @date: 2023-07-21
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_PATH_PLANNER_GRAPH_PLANNER_VORONOI_PLANNER_H_
#define RMP_PATH_PLANNER_GRAPH_PLANNER_VORONOI_PLANNER_H_

#include <vector>

#include "voronoi_layer.h"

#include "path_planner/roadmap_planner/roadmap_planner_base.h"

namespace rmp
{
namespace path_planner
{
/**
 * @brief Class for objects that plan using the Voronoi-based planning algorithm
 */
class VoronoiPathPlanner : public RoadmapPathPlannerBase
{
public:
  /**
   * @brief Construct a new Voronoi-based planning object
   * @param costmap   the environment for path planning
   * @param obstacle_factor obstacle factor(greater means obstacles)
   */
  VoronoiPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, double obstacle_factor = 1.0);

  /**
   * @brief Voronoi-based planning implementation
   * @param start         start node
   * @param goal          goal node
   * @param path          optimal path consists of Node
   * @param expand        containing the node been search during the process
   * @return  true if path found, else false
   */
  bool plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand);

private:
  /**
   * @brief update voronoi object using costmap ROS wrapper
   */
  void _updateVoronoi();

private:
  DynamicVoronoi voronoi_;                              // dynamic voronoi map
  std::vector<rmp::common::geometry::Vec2d> vertices_;  // voronoi nodes
  std::vector<Edge> valid_edges_;                       // valid edge set for voronoi graph
  RoadMap roadmap_;                                     // topological road network structure for voronoi graph

  // allowable motions
  using Node = rmp::common::structure::Node<int>;
  static std::vector<Node> motions_;
};
}  // namespace path_planner
}  // namespace rmp
#endif
