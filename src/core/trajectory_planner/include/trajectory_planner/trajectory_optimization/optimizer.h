/***********************************************************
 *
 * @file: optimizer.h
 * @breif: Trajectory optimization
 * @author: Yang Haodong
 * @update: 2023-12-29
 * @version: 1.0
 *
 * Copyright (c) 2023, Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef RMP_TRAJECTORY_OPTIMIZATION_OPTIMIZER_H_
#define RMP_TRAJECTORY_OPTIMIZATION_OPTIMIZER_H_

#include <costmap_2d/costmap_2d_ros.h>

#include "common/geometry/point.h"
#include "common/structure/trajectory.h"

namespace rmp
{
namespace trajectory_optimization
{
class Optimizer
{
protected:
  using Point2d = rmp::common::geometry::Point2d;
  using Points2d = rmp::common::geometry::Points2d;
  using Point3d = rmp::common::geometry::Point3d;
  using Points3d = rmp::common::geometry::Points3d;
  using Trajectory2d = rmp::common::structure::Trajectory2d;
  using Trajectory3d = rmp::common::structure::Trajectory3d;

public:
  /**
   * @brief Construct a new trajectory optimizer object
   * @param costmap_ros costmap ROS wrapper
   */
  Optimizer();
  Optimizer(costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Destroy the trajectory optimizer object
   */
  virtual ~Optimizer() = default;

  /**
   * @brief Running trajectory optimization
   * @param waypoints path points <x, y, theta> before optimization
   * @return true if optimizes successfully, else failed
   */
  virtual bool run(const Points3d& waypoints) = 0;
  virtual bool run(const Trajectory3d& traj) = 0;

  /**
   * @brief Get the optimized trajectory
   * @param traj the trajectory buffer
   * @return true if optimizes successfully, else failed
   */
  virtual bool getTrajectory(Trajectory3d& traj) = 0;

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
};

}  // namespace trajectory_optimization
}  // namespace rmp
#endif