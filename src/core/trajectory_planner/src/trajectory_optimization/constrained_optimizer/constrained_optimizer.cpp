/***********************************************************
 *
 * @file: constrained_optimizer.cpp
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
#include "common/util/log.h"
#include "common/math/math_helper.h"
#include "path_planner/path_prune/ramer_douglas_peucker.h"
#include "trajectory_planner/trajectory_optimization/constrained_optimizer/cost_functions.h"
#include "trajectory_planner/trajectory_optimization/constrained_optimizer/constrained_optimizer.h"

using namespace rmp::path_planner;

namespace
{
constexpr double kPruneDelta = 0.25;          // [m]
constexpr double kPruneMaxInterval = 0.60;    // [m]
constexpr double kCeresMaxSolverTime = 10.0;  // [s]
constexpr int kCeresMaxIterations = 50;
constexpr double kCeresFunctionTol = 1.0e-5;
constexpr double kCeresGradientTol = 1e-5;
constexpr double kCeresParameterTol = 1.0e-5;
}  // namespace

namespace rmp
{
namespace trajectory_optimization
{
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
ConstrainedOptimizer::ConstrainedOptimizer(costmap_2d::Costmap2DROS* costmap_ros, double obs_dist_max, double k_max,
                                           double w_obstacle, double w_smooth, double w_distance, double w_curvature,
                                           bool debug)
  : Optimizer(costmap_ros)
  , obs_dist_max_(obs_dist_max)
  , k_max_(k_max)
  , w_obstacle_(w_obstacle)
  , w_smooth_(w_smooth)
  , w_distance_(w_distance)
  , w_curvature_(w_curvature)
  , debug_(debug)
  , path_processor_(std::make_unique<RDPPathProcessor>(kPruneDelta, kPruneMaxInterval))
{
}

/**
 * @brief Running trajectory optimization
 * @param waypoints path points <x, y, theta> before optimization
 * @return true if optimizes successfully, else failed
 */
bool ConstrainedOptimizer::run(const Points3d& waypoints)
{
  start_angle_ = waypoints[0].theta();
  goal_angle_ = waypoints.back().theta();
  Points3d prune_waypoints;
  path_processor_->process(waypoints, prune_waypoints);
  return optimize(prune_waypoints);
}
bool ConstrainedOptimizer::run(const Trajectory3d& traj)
{
  return run(traj.position);
}

/**
 * @brief Get the optimized trajectory
 * @param traj the trajectory buffer
 * @return true if optimizes successfully, else failed
 */
bool ConstrainedOptimizer::getTrajectory(Trajectory3d& traj)
{
  size_t traj_size = path_opt_.size();
  if (traj_size == 0)
  {
    R_INFO << "No invalid trajectory found.";
    return false;
  }

  traj.reset(traj_size);

  double t = 0.0;
  const double dt = 0.25;
  for (size_t i = 0; i < traj_size; i++)
  {
    const auto& pt = path_opt_[i];
    traj.time.push_back(t);
    double theta;
    if (i == 0)
    {
      theta = start_angle_;
    }
    else if (i + 1 == traj_size)
    {
      theta = goal_angle_;
    }
    else
    {
      auto last_vec = Vec2d(path_opt_[i - 1][0], path_opt_[i - 1][1]);
      auto curr_vec = Vec2d(path_opt_[i][0], path_opt_[i][1]);
      auto next_vec = Vec2d(path_opt_[i + 1][0], path_opt_[i + 1][1]);
      auto tangent_dir = rmp::common::math::tangentDir(last_vec, curr_vec, next_vec, false);
      auto dir = tangent_dir.innerProd(next_vec - curr_vec) >= 0 ? tangent_dir : -tangent_dir;
      theta = dir.angle();
    }
    traj.position.emplace_back(pt[0], pt[1], theta);

    if (i < traj_size - 1)
    {
      const auto& pt_next = path_opt_[i + 1];
      const double vx = (pt_next[0] - pt[0]) / dt;
      const double vy = (pt_next[1] - pt[1]) / dt;
      traj.velocity.emplace_back(vx, vy);
      if (i < traj_size - 2)
      {
        const auto& pt_next_next = path_opt_[i + 2];
        const double vx_next = (pt_next_next[0] - pt_next[0]) / dt;
        const double vy_next = (pt_next_next[1] - pt_next[1]) / dt;
        traj.acceletation.emplace_back((vx_next - vx) / dt, (vy_next - vy) / dt);
      }
    }
    t += dt;
  }

  return true;
}

/**
 * @brief trajectory optimization executor
 * @param waypoints path points <x, y, theta> before optimization
 * @return true if optimizes successfully, else failed
 */
bool ConstrainedOptimizer::optimize(const Points3d& waypoints)
{
  if (waypoints.size() < 2)
  {
    R_WARN << "Path must have at least 2 points.";
  }

  // solve the problem
  ceres::Problem problem;
  ceres::Solver::Options options;
  options.max_solver_time_in_seconds = kCeresMaxSolverTime;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.max_num_iterations = kCeresMaxIterations;
  options.function_tolerance = kCeresFunctionTol;
  options.gradient_tolerance = kCeresGradientTol;
  options.parameter_tolerance = kCeresParameterTol;
  if (debug_)
  {
    options.minimizer_progress_to_stdout = true;
    options.logging_type = ceres::LoggingType::PER_MINIMIZER_ITERATION;
  }
  else
  {
    options.logging_type = ceres::SILENT;
  }

  if (_buildProblem(waypoints, problem))
  {
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    if (debug_)
    {
      R_INFO << summary.FullReport();
    }
    if (!summary.IsSolutionUsable() || summary.initial_cost - summary.final_cost < 0.0)
    {
      R_WARN << "Solution is not usable";
    }
  }
  else
  {
    R_WARN << "Path too short to optimize";
  }

  return true;
}

/**
 * @brief Build the nonlinear least squares problem
 * @param waypoints path points <x, y, theta> before optimization
 * @param problem ceres problem object
 * @return true if build successfully, else failed
 */
bool ConstrainedOptimizer::_buildProblem(const Points3d& waypoints, ceres::Problem& problem)
{
  // initialize trajectory to optimize
  path_opt_.clear();
  for (const auto& pt : waypoints)
  {
    path_opt_.emplace_back(pt.x(), pt.y(), pt.theta());
  }

  // Create grid (using costmap)
  // costmap_grid_ = std::make_shared<ceres::Grid2D<u_char>>(costmap_ros_->getCostmap()->getCharMap(), 0,
  //                                                         costmap_ros_->getCostmap()->getSizeInCellsY(), 0,
  //                                                         costmap_ros_->getCostmap()->getSizeInCellsX());
  // auto costmap_interpolator = std::make_shared<ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>>(*costmap_grid_);

  // Create grid (using distance field)
  bool is_distance_layer_exist = false;
  for (auto layer = costmap_ros_->getLayeredCostmap()->getPlugins()->begin();
       layer != costmap_ros_->getLayeredCostmap()->getPlugins()->end(); ++layer)
  {
    distance_layer_ = boost::dynamic_pointer_cast<costmap_2d::DistanceLayer>(*layer);
    if (distance_layer_)
    {
      is_distance_layer_exist = true;
      break;
    }
  }
  if (!is_distance_layer_exist)
  {
    R_ERROR << "Failed to get a Distance layer for potentional application.";
  }
  costmap_grid_ = std::make_shared<ceres::Grid2D<double>>(
      distance_layer_->getEDFPtr(), 0, distance_layer_->getSizeInCellsY(), 0, distance_layer_->getSizeInCellsX());
  auto costmap_interpolator = std::make_shared<ceres::BiCubicInterpolator<ceres::Grid2D<double>>>(*costmap_grid_);

  // Create residual blocks
  ceres::LossFunction* loss_function = NULL;
  for (size_t i = 2; i < path_opt_.size(); i++)
  {
    Eigen::Vector3d origin_pose(waypoints[i - 1].x(), waypoints[i - 1].y(), waypoints[i - 1].theta());
    OptimizerCostFunction* cost_function =
        new OptimizerCostFunction(origin_pose, costmap_ros_->getCostmap(), costmap_interpolator, obs_dist_max_, k_max_,
                                  w_obstacle_, w_smooth_, w_distance_, w_curvature_);
    problem.AddResidualBlock(cost_function->AutoDiff(), loss_function, path_opt_[i - 2].data(), path_opt_[i - 1].data(),
                             path_opt_[i].data());
  }

  // first two and last two points are constant (to keep start and end direction)
  int poses_to_optimize = problem.NumParameterBlocks() - 4;
  if (poses_to_optimize <= 0)
  {
    return false;  // nothing to optimize
  }
  problem.SetParameterBlockConstant(path_opt_.front().data());
  problem.SetParameterBlockConstant(path_opt_[1].data());
  problem.SetParameterBlockConstant(path_opt_[path_opt_.size() - 2].data());
  problem.SetParameterBlockConstant(path_opt_.back().data());
  return true;
}
}  // namespace trajectory_optimization
}  // namespace rmp