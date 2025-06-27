/***********************************************************
 *
 * @file: lbfgs_optimizer.h
 * @breif: Trajectory optimization using LBFGS method
 * @author: Yang Haodong
 * @update: 2025-01-13
 * @version: 1.0
 *
 * Copyright (c) 2025, Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef RMP_TRAJECTORY_OPTIMIZATION_LBFGS_OPTIMIZER_H_
#define RMP_TRAJECTORY_OPTIMIZATION_LBFGS_OPTIMIZER_H_

#include "distance_layer.h"

#include "path_planner/path_processor/path_processor.h"
#include "trajectory_planner/trajectory_optimization/optimizer.h"

namespace rmp
{
namespace trajectory_optimization
{
class LBFGSOptimizer : public Optimizer
{
public:
  /**
   * @brief Construct a new trajectory optimizer object
   * @param costmap_ros costmap ROS wrapper
   * @param obs_dist_max the maximum distance to obstacle (m)
   * @param k_max the maximum curvature
   * @param w_obstacle the weight for obstacle avoidance
   * @param w_smooth the weight for smooth
   * @param w_curvature the weight for curvature
   */
  LBFGSOptimizer(costmap_2d::Costmap2DROS* costmap_ros, double obs_dist_max, double k_max, double w_obstacle,
                 double w_smooth, double w_curvature);
  ~LBFGSOptimizer() = default;

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
   * @brief Calculate the total cost and gradient for the given path during optimization.
   * @param ptr Pointer to the LBFGSOptimizer instance.
   * @param x The current optimized variables (waypoints) as a vector.
   * @param g Reference to the gradient vector to be updated.
   * @return The total cost for the given path.
   */
  static double costFunction(void* ptr, const Eigen::VectorXd& x, Eigen::VectorXd& g);

  /**
   * @brief Calculate the obstacle avoidance gradient and cost for the given path.
   * @param instance The instance of LBFGSOptimizer.
   * @param path_opt The optimized waypoints.
   * @param obstacle_cost The total obstacle cost.
   * @return The obstacle avoidance gradient vector for the waypoints.
   */
  static Eigen::Matrix2Xd _calObstacleTerm(const LBFGSOptimizer* instance, const Eigen::Matrix2Xd& path_opt,
                                           double& obstacle_cost);
  /**
   * @brief Calculate the smooth gradient and cost for the given path.
   * @param instance The instance of LBFGSOptimizer.
   * @param path_opt The optimized waypoints.
   * @param obstacle_cost The total smooth cost.
   * @return The smooth gradient vector for the waypoints.
   */
  static Eigen::Matrix2Xd _calSmoothTerm(const LBFGSOptimizer* instance, const Eigen::Matrix2Xd& path_opt,
                                         double& smooth_cost);

  /**
   * @brief Calculate the curvature gradient and cost for the given path.
   * @param instance The instance of LBFGSOptimizer.
   * @param path_opt The optimized waypoints.
   * @param obstacle_cost The total smooth cost.
   * @return The curvature gradient vector for the waypoints.
   */
  static Eigen::Matrix2Xd _calCurvatureTerm(const LBFGSOptimizer* instance, const Eigen::Matrix2Xd& path_opt,
                                            double& curvature_cost);

private:
  double obs_dist_max_;                                               // the maximum distance to obstacle (pixel)
  double k_max_;                                                      // the maximum curvature
  double w_obstacle_;                                                 // the weight for obstacle avoidance
  double w_smooth_;                                                   // the weight for smooth
  double w_curvature_;                                                // the weight for curvature
  std::unique_ptr<rmp::path_planner::PathProcessor> path_processor_;  // path processor
  Points3d path_opt_;                                                 // optimized path
  boost::shared_ptr<costmap_2d::DistanceLayer> distance_layer_;       // ESDF field
  double start_angle_, goal_angle_;                                   // yaw angle of start ang goal
};
}  // namespace trajectory_optimization
}  // namespace rmp

#endif