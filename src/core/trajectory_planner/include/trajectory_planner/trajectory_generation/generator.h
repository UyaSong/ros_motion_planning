/***********************************************************
 *
 * @file: generator.h
 * @breif: Trajectory generation base
 * @author: Yang Haodong
 * @update: 2025-1-12
 * @version: 1.0
 *
 * Copyright (c) 2025, Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef RMP_TRAJECTORY_GENERATION_GENERATOR_H_
#define RMP_TRAJECTORY_GENERATION_GENERATOR_H_

#include <costmap_2d/costmap_2d_ros.h>

#include "common/geometry/point.h"
#include "common/geometry/vec2d.h"
#include "common/structure/trajectory.h"
#include "path_planner/path_processor/path_processor.h"

namespace rmp
{
namespace trajectory_generation
{
class Generator
{
protected:
  using Vec2d = rmp::common::geometry::Vec2d;
  using Point2d = rmp::common::geometry::Point2d;
  using Points2d = rmp::common::geometry::Points2d;
  using Point3d = rmp::common::geometry::Point3d;
  using Points3d = rmp::common::geometry::Points3d;
  using Trajectory2d = rmp::common::structure::Trajectory2d;
  using Trajectory3d = rmp::common::structure::Trajectory3d;

public:
  /**
   * @brief Construct a new trajectory generator object
   * @param costmap_ros costmap ROS wrapper
   */
  Generator();
  Generator(costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Destroy the trajectory generator object
   */
  virtual ~Generator() = default;

  /**
   * @brief Running trajectory generation
   * @return true if optimizes successfully, else failed
   */
  virtual bool generation() = 0;

  /**
   * @brief Get the optimized trajectory
   * @param traj the trajectory buffer
   * @return true if optimizes successfully, else failed
   */
  virtual bool getTrajectory(Trajectory3d& traj) = 0;

  /**
   * @brief Set the waypoints from planning path
   * @param waypoints the waypoints
   * @param is_prune if true, prune the original waypoints
   */
  void setWaypoints(const Points3d& waypoints, bool is_prune = true);

protected:
  /**
   * @brief Judge whether the grid(x, y) is inside the map
   * @param x grid coordinate x
   * @param y grid coordinate y
   * @return true if inside the map else false
   */
  bool _insideMap(unsigned int x, unsigned int y);
  /**
   * @brief Judge whether the grid(x, y) is inside the map
   * @param x world coordinate x
   * @param y world coordinate y
   * @return true if inside the map else false
   */
  bool _insideMap(double x, double y);

protected:
  costmap_2d::Costmap2DROS* costmap_ros_;  // costmap ROS wrapper
  unsigned int nx_, ny_, map_size_;        // map size
  Points3d waypoints_;                     // waypoints
  Trajectory3d trajectory_;                // genetated trajectory
  std::unique_ptr<rmp::path_planner::PathProcessor> path_processor_;
};
}  // namespace trajectory_generation
}  // namespace rmp

#endif