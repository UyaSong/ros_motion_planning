/***********************************************************
 *
 * @file: minimumsnap_optimizer.h
 * @breif: Trajectory optimization using minimumsnap methods
 * @author: Yang Haodong
 * @update: 2024-9-20
 * @version: 1.0
 *
 * Copyright (c) 2023, Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef RMP_TRAJECTORY_OPTIMIZATION_MINIMUMSNAP_OPTIMIZER_H_
#define RMP_TRAJECTORY_OPTIMIZATION_MINIMUMSNAP_OPTIMIZER_H_

#include <Eigen/Dense>

#include "common/geometry/polygon2d.h"
#include "common/geometry/curve/quintic_polynomial.h"

#include "trajectory_planner/trajectory_optimization/optimizer.h"

namespace rmp
{
namespace common
{
namespace safety_corridor
{
class ConvexSafetyCorridor;
}
}  // namespace common
}  // namespace rmp

namespace rmp
{
namespace trajectory_optimization
{
class MinimumsnapOptimizer : public Optimizer
{
private:
  using Polygon2d = rmp::common::geometry::Polygon2d;
  using PolyCurve = rmp::common::geometry::QuinticPolynomial;
  using State2d = std::array<double, 6>;  // x, vx, ax, py, vy, ay

public:
  /**
   * @brief Construct a new trajectory optimizer object
   * @param costmap_ros costmap ROS wrapper
   * @param max_iter the maximum iterations for optimization
   * @param vel_max the maximum velocity (m/s)
   * @param acc_max the maximum acceleration (m/s2)
   * @param jerk_max the maximum jerk (m/s3)
   * @param safety_range The safety range to limit polygon space [m]
   */
  MinimumsnapOptimizer(int max_iter, double vel_max, double acc_max, double jerk_max);
  MinimumsnapOptimizer(costmap_2d::Costmap2DROS* costmap_ros, int max_iter, double vel_max, double acc_max,
                       double jerk_max, double safety_range = 0.5);
  MinimumsnapOptimizer(Points3d obstacles, int max_iter, double vel_max, double acc_max, double jerk_max,
                       double safety_range = 0.5);
  ~MinimumsnapOptimizer() = default;

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

  /**
   * @brief Get the convex safety corridor
   * @param polygons the convex safety corridor
   */
  const std::vector<Polygon2d>& getPolygons() const;

private:
  /**
   * @brief trajectory optimization executor
   * @param waypoints path points <x, y, theta> before optimization
   * @return true if optimizes successfully, else failed
   */
  bool optimize(const Points3d& waypoints);

  /**
   * @brief solve the minimumsnap qp problem using osqp
   * @param start_state start point with <px, vx, ax, py, vy, ay>
   * @param target_state target point with <px, vx, ax, py, vy, ay>
   * @param waypoints path points <x, y, theta> before optimization
   * @return true if optimizes successfully, else failed
   */
  bool solve(const State2d& start_state, const State2d& target_state, const Points3d& waypoints);

  /**
   * @brief auxiliary matrix
   * @param dt_i time allocation of segment i
   * @return A^+_i
   */
  Eigen::MatrixXd A_i_plus(double dt_i);

  /**
   * @brief auxiliary matrix
   * @param dt_i time allocation of segment i
   * @return A^-_i
   */
  Eigen::MatrixXd A_i_minus(double dt_i);

private:
  int max_iter_;      // the maximum iterations for optimization
  int k_;             // segment number for piecewise curve
  int dim_;           // dimension
  double vel_max_;    // the maximum velocity (m/s)
  double acc_max_;    // the maximum acceleration (m/s2)
  double jerk_max_;   // the maximum jerk (m/s3)
  const int n_{ 5 };  // polynomial order
  const int r_{ 4 };  // smooth order (4 for minimum snap, 3 for minimum jerk)

  std::vector<Polygon2d> polygons_;
  std::vector<double> time_allocations_;
  std::vector<std::vector<PolyCurve>> trajectories_;
  std::shared_ptr<rmp::common::safety_corridor::ConvexSafetyCorridor> safety_corridor_;
};
}  // namespace trajectory_optimization
}  // namespace rmp

#endif