/***********************************************************
 *
 * @file: constrained_optimizer.h
 * @breif: Trajectory optimization using nonlinear least squares based on Ceres
 * @author: Yang Haodong
 * @update: 2025-01-13
 * @version: 1.0
 *
 * Copyright (c) 2025, Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef RMP_TRAJECTORY_OPTIMIZATION_CONSTRAINED_OPTIMIZER_H_
#define RMP_TRAJECTORY_OPTIMIZATION_CONSTRAINED_OPTIMIZER_H_

#include <ceres/ceres.h>
#include <ceres/cubic_interpolation.h>

#include "distance_layer.h"

#include "common/geometry/vec2d.h"
#include "common/math/math_helper.h"
#include "path_planner/path_processor/path_processor.h"
#include "trajectory_planner/trajectory_optimization/optimizer.h"

namespace rmp
{
namespace trajectory_optimization
{
class ConstrainedOptimizer : public Optimizer
{
private:
  using Vec2d = rmp::common::geometry::Vec2d;
  using DistanceField = std::vector<std::vector<double>>;

public:
  /**
   * @brief Construct a new trajectory optimizer object
   * @param costmap_ros costmap ROS wrapper
   * @param max_time the maximum solve time
   * @param alpha learning rate
   * @param obs_dist_max the maximum distance to obstacle (m)
   * @param k_max the maximum curvature
   * @param w_obstacle the weight for obstacle avoidance
   * @param w_smooth the weight for smooth
   * @param w_curvature the weight for curvature
   */
  ConstrainedOptimizer(costmap_2d::Costmap2DROS* costmap_ros, double obs_dist_max, double k_max, double w_obstacle,
                       double w_smooth, double w_distance, double w_curvature, bool debug);
  ~ConstrainedOptimizer() = default;

  /**
   * @brief Running trajectory optimization
   * @param waypoints path points <x, y, theta> before optimization
   * @return true if optimizes successfully, else failed
   */
  bool run(const Points3d& waypoints);
  bool run(const Trajectory3d& traj);

  /**
   * @brief Get the optimized trajectory
   * @param traj the trajectory buffer
   * @return true if optimizes successfully, else failed
   */
  bool getTrajectory(Trajectory3d& traj);

protected:
  /**
   * @brief trajectory optimization executor
   * @param waypoints path points <x, y, theta> before optimization
   * @return true if optimizes successfully, else failed
   */
  bool optimize(const Points3d& waypoints);

private:
  /**
   * @brief Build the nonlinear least squares problem
   * @param waypoints path points <x, y, theta> before optimization
   * @param problem ceres problem object
   * @return true if build successfully, else failed
   */
  bool _buildProblem(const Points3d& waypoints, ceres::Problem& problem);

private:
  double obs_dist_max_;  // the maximum distance to obstacle (m)
  double k_max_;         // the maximum curvature
  double w_obstacle_;    // the weight for obstacle avoidance
  double w_smooth_;      // the weight for smooth
  double w_distance_;    // the weight for distance
  double w_curvature_;   // the weight for curvature
  bool debug_;           // whether show the debug information

  double start_angle_, goal_angle_;                                   // yaw angle of start ang goal
  std::vector<Eigen::Vector3d> path_opt_;                             // optimized path
  std::unique_ptr<rmp::path_planner::PathProcessor> path_processor_;  // path processor
  std::shared_ptr<ceres::Grid2D<double>> costmap_grid_;               // 2d costmap grid
  boost::shared_ptr<costmap_2d::DistanceLayer> distance_layer_;       // distance layer
};

}  // namespace trajectory_optimization
}  // namespace rmp
#endif