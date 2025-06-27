/***********************************************************
 *
 * @file: lbfgs_optimizer.cpp
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
#include "common/geometry/vec2d.h"
#include "common/math/math_helper.h"
#include "common/math/solver/lbfgs.hpp"
#include "path_planner/path_prune/ramer_douglas_peucker.h"
#include "trajectory_planner/trajectory_optimization/lbfgs_optimizer/lbfgs_optimizer.h"

using namespace rmp::path_planner;
using namespace rmp::common::math;
using namespace rmp::common::geometry;

namespace rmp
{
namespace trajectory_optimization
{
namespace
{
constexpr double kPruneDelta = 0.25;
constexpr double kPruneMaxInterval = 0.50;
}  // namespace

/**
 * @brief Construct a new trajectory optimizer object
 * @param costmap_ros costmap ROS wrapper
 * @param obs_dist_max the maximum distance to obstacle (m)
 * @param k_max the maximum curvature
 * @param w_obstacle the weight for obstacle avoidance
 * @param w_smooth the weight for smooth
 * @param w_curvature the weight for curvature
 */
LBFGSOptimizer::LBFGSOptimizer(costmap_2d::Costmap2DROS* costmap_ros, double obs_dist_max, double k_max,
                               double w_obstacle, double w_smooth, double w_curvature)
  : Optimizer(costmap_ros)
  , obs_dist_max_(obs_dist_max)
  , k_max_(k_max)
  , w_obstacle_(w_obstacle)
  , w_smooth_(w_smooth)
  , w_curvature_(w_curvature)
  , path_processor_(std::make_unique<RDPPathProcessor>(kPruneDelta, kPruneMaxInterval))
{
}

/**
 * @brief Running trajectory optimization
 * @param waypoints path points <x, y, theta> before optimization
 * @return true if optimizes successfully, else failed
 */
bool LBFGSOptimizer::run(const Points3d& waypoints)
{
  start_angle_ = waypoints[0].theta();
  goal_angle_ = waypoints.back().theta();
  Points3d prune_waypoints;
  path_processor_->process(waypoints, prune_waypoints);
  path_opt_ = prune_waypoints;
  return optimize(prune_waypoints);
}

bool LBFGSOptimizer::run(const Trajectory3d& traj)
{
  return run(traj.position);
}

/**
 * @brief Get the optimized trajectory
 * @param traj the trajectory buffer
 * @return true if optimizes successfully, else failed
 */
bool LBFGSOptimizer::getTrajectory(Trajectory3d& traj)
{
  traj.reset(path_opt_.size());

  double t = 0.0;
  const double dt = 0.25;
  for (int i = 0; i < path_opt_.size(); i++)
  {
    const auto& pt = path_opt_[i];
    traj.time.push_back(t);
    double theta;
    if (i == 0)
    {
      theta = start_angle_;
    }
    else if (i + 1 == path_opt_.size())
    {
      theta = goal_angle_;
    }
    else
    {
      auto last_vec = Vec2d(path_opt_[i - 1].x(), path_opt_[i - 1].y());
      auto curr_vec = Vec2d(path_opt_[i].x(), path_opt_[i].y());
      auto next_vec = Vec2d(path_opt_[i + 1].x(), path_opt_[i + 1].y());
      auto tangent_dir = tangentDir(last_vec, curr_vec, next_vec, false);
      auto dir = tangent_dir.innerProd(next_vec - curr_vec) >= 0 ? tangent_dir : -tangent_dir;
      theta = dir.angle();
    }
    traj.position.emplace_back(pt.x(), pt.y(), theta);

    if (i < path_opt_.size() - 1)
    {
      const auto& pt_next = path_opt_[i + 1];
      const double vx = (pt_next.x() - pt.x()) / dt;
      const double vy = (pt_next.y() - pt.y()) / dt;
      traj.velocity.emplace_back(vx, vy);
      if (i < path_opt_.size() - 2)
      {
        const auto& pt_next_next = path_opt_[i + 2];
        const double vx_next = (pt_next_next.x() - pt_next.x()) / dt;
        const double vy_next = (pt_next_next.y() - pt_next.y()) / dt;
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
bool LBFGSOptimizer::optimize(const Points3d& waypoints)
{
  // distance map update
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

  // first two and last two points are constant (to keep start and end direction)
  int var_num = static_cast<int>(waypoints.size()) - 4;
  Eigen::VectorXd opt_var(2 * var_num);
  for (int i = 2; i < static_cast<int>(waypoints.size()) - 2; ++i)
  {
    opt_var(i - 2) = waypoints[i].x();
    opt_var(i - 2 + var_num) = waypoints[i].y();
  }
  path_opt_ = waypoints;

  // optimizer parameters
  lbfgs::lbfgs_parameter_t lbfgs_params;
  lbfgs_params.mem_size = 256;
  lbfgs_params.past = 3;
  lbfgs_params.min_step = 1.0e-32;
  lbfgs_params.g_epsilon = 0.0;
  lbfgs_params.delta = 0.01;

  // optimization
  double min_cost = 0.0;
  int ret =
      lbfgs::lbfgs_optimize(opt_var, min_cost, &LBFGSOptimizer::costFunction, nullptr, nullptr, this, lbfgs_params);

  // parse solution
  if (ret >= 0)
  {
    for (int i = 0; i < var_num; ++i)
    {
      path_opt_[i + 2].setX(opt_var(i));
      path_opt_[i + 2].setY(opt_var(i + var_num));
    }
  }
  else
  {
    R_WARN << "LBFGS Optimization failed: " << lbfgs::lbfgs_strerror(ret);
    return false;
  }

  return true;
}

/**
 * @brief Calculate the total cost and gradient for the given path during optimization.
 * @param ptr Pointer to the LBFGSOptimizer instance.
 * @param x The current optimized variables (waypoints) as a vector.
 * @param g Reference to the gradient vector to be updated.
 * @return The total cost for the given path.
 */
double LBFGSOptimizer::costFunction(void* ptr, const Eigen::VectorXd& x, Eigen::VectorXd& g)
{
  auto instance = reinterpret_cast<LBFGSOptimizer*>(ptr);
  const auto& origin_path = instance->path_opt_;

  int points_num = static_cast<int>(origin_path.size());
  int var_num = points_num - 4;
  double cost = 0.0;
  Eigen::Matrix2Xd grad;
  grad.resize(2, var_num);
  grad.setZero();

  // update opt-path during optimization
  Eigen::Matrix2Xd path_opt;
  path_opt.resize(2, points_num);
  path_opt(0, 0) = origin_path[0].x();
  path_opt(1, 0) = origin_path[0].y();
  path_opt(0, 1) = origin_path[1].x();
  path_opt(1, 1) = origin_path[1].y();
  path_opt.block(0, 2, 1, var_num) = x.head(var_num).transpose();
  path_opt.block(1, 2, 1, var_num) = x.tail(var_num).transpose();
  path_opt(0, points_num - 2) = origin_path[points_num - 2].x();
  path_opt(1, points_num - 2) = origin_path[points_num - 2].y();
  path_opt(0, points_num - 1) = origin_path[points_num - 1].x();
  path_opt(1, points_num - 1) = origin_path[points_num - 1].y();

  // obstacle term
  double obstacle_cost = 0.0;
  const auto& obstacle_grad = _calObstacleTerm(instance, path_opt, obstacle_cost);
  grad += obstacle_grad;
  cost += obstacle_cost;

  // smooth term
  double smooth_cost = 0.0;
  const auto& smooth_grad = _calSmoothTerm(instance, path_opt, smooth_cost);
  grad += smooth_grad;
  cost += smooth_cost;

  // curvature term
  double curvature_cost = 0.0;
  const auto& curvature_grad = _calCurvatureTerm(instance, path_opt, curvature_cost);
  grad += curvature_grad;
  cost += curvature_cost;

  g.setZero();
  g.head(var_num) = grad.row(0).transpose();
  g.tail(var_num) = grad.row(1).transpose();

  return cost;
}

/**
 * @brief Calculate the obstacle avoidance gradient and cost for the given path.
 * @param instance The instance of LBFGSOptimizer.
 * @param path_opt The optimized waypoints.
 * @param obstacle_cost The total obstacle cost.
 * @return The obstacle avoidance gradient vector for the waypoints.
 */
Eigen::Matrix2Xd LBFGSOptimizer::_calObstacleTerm(const LBFGSOptimizer* instance, const Eigen::Matrix2Xd& path_opt,
                                                  double& obstacle_cost)
{
  obstacle_cost = 0.0;
  Eigen::Matrix2Xd obstacle_grad;
  obstacle_grad.resize(2, static_cast<int>(path_opt.cols()) - 4);
  obstacle_grad.setZero();

  for (int i = 0; i < static_cast<int>(path_opt.cols()) - 4; ++i)
  {
    Vec2d gradient;
    unsigned int mx, my;
    instance->costmap_ros_->getCostmap()->worldToMap(path_opt(0, i + 2), path_opt(1, i + 2), mx, my);
    if (mx < instance->nx_ && mx >= 0 && my < instance->ny_ && my >= 0)
    {
      // the distance to the closest obstacle from the current node
      const double resolution = instance->costmap_ros_->getCostmap()->getResolution();
      double obs_dist = instance->distance_layer_->getDistance(mx, instance->ny_ - my - 1) * resolution;
      double dx, dy;
      instance->distance_layer_->getGradient(mx, instance->ny_ - my - 1, dx, dy);
      Vec2d obs_vec(dx, -dy);
      obs_vec.normalize();
      obs_vec *= resolution;
      if (obs_dist < instance->obs_dist_max_ && obs_dist > 0)
      {
        obstacle_cost += instance->w_obstacle_ * std::pow(obs_dist - instance->obs_dist_max_, 2);
        gradient = instance->w_obstacle_ * 2.0 * (obs_dist - instance->obs_dist_max_) * obs_vec / obs_dist;
        obstacle_grad(0, i) = gradient.x();
        obstacle_grad(1, i) = gradient.y();
      }
    }
  }
  return obstacle_grad;
}

/**
 * @brief Calculate the smooth gradient and cost for the given path.
 * @param instance The instance of LBFGSOptimizer.
 * @param path_opt The optimized waypoints.
 * @param obstacle_cost The total smooth cost.
 * @return The smooth gradient vector for the waypoints.
 */
Eigen::Matrix2Xd LBFGSOptimizer::_calSmoothTerm(const LBFGSOptimizer* instance, const Eigen::Matrix2Xd& path_opt,
                                                double& smooth_cost)
{
  smooth_cost = 0.0;
  Eigen::Matrix2Xd smooth_grad;
  smooth_grad.resize(2, static_cast<int>(path_opt.cols()) - 4);
  smooth_grad.setZero();

  for (int i = 2; i < static_cast<int>(path_opt.cols()) - 2; ++i)
  {
    Vec2d xi_c2(path_opt(0, i - 2), path_opt(1, i - 2));
    Vec2d xi_c1(path_opt(0, i - 1), path_opt(1, i - 1));
    Vec2d xi(path_opt(0, i), path_opt(1, i));
    Vec2d xi_p1(path_opt(0, i + 1), path_opt(1, i + 1));
    Vec2d xi_p2(path_opt(0, i + 2), path_opt(1, i + 2));

    Vec2d error = xi_c1 + xi_p1 - 2 * xi;
    smooth_cost += instance->w_smooth_ * error.lengthSquare();
    Vec2d gradient = 2.0 * instance->w_smooth_ * (xi_c2 - 4.0 * xi_c1 + 6.0 * xi - 4.0 * xi_p1 + xi_p2);
    smooth_grad(0, i - 2) = gradient.x();
    smooth_grad(1, i - 2) = gradient.y();
  }

  return smooth_grad;
}

/**
 * @brief Calculate the curvature gradient and cost for the given path.
 * @param instance The instance of LBFGSOptimizer.
 * @param path_opt The optimized waypoints.
 * @param obstacle_cost The total smooth cost.
 * @return The curvature gradient vector for the waypoints.
 */
Eigen::Matrix2Xd LBFGSOptimizer::_calCurvatureTerm(const LBFGSOptimizer* instance, const Eigen::Matrix2Xd& path_opt,
                                                   double& curvature_cost)
{
  curvature_cost = 0.0;
  Eigen::Matrix2Xd curvature_grad;
  curvature_grad.resize(2, static_cast<int>(path_opt.cols()) - 4);
  curvature_grad.setZero();

  auto ort = [](const Vec2d& a, const Vec2d& b) { return a - b * a.innerProd(b) / std::pow(b.length(), 2); };

  for (int i = 2; i < static_cast<int>(path_opt.cols()) - 2; ++i)
  {
    Vec2d xi_c1(path_opt(0, i - 1), path_opt(1, i - 1));
    Vec2d xi(path_opt(0, i), path_opt(1, i));
    Vec2d xi_p1(path_opt(0, i + 1), path_opt(1, i + 1));

    // the curvature needs to be reduced in the world coordinate system to prevent shaking
    double resolution = instance->costmap_ros_->getCostmap()->getResolution();
    Vec2d d_xi = (xi - xi_c1) / (0.1 * resolution);
    Vec2d d_xi_p1 = (xi_p1 - xi) / (0.1 * resolution);

    // orthogonal complements vector
    Vec2d p1, p2;

    // the distance of the vectors
    double abs_dxi = d_xi.length();
    double abs_dxi_p1 = d_xi_p1.length();

    if (abs_dxi > 1e-3 && abs_dxi_p1 > 1e-3)
    {
      // the angular change at the node
      double d_phi = acos(rmp::common::math::clamp(d_xi.innerProd(d_xi_p1) / (abs_dxi * abs_dxi_p1), -1.0, 1.0));
      if (std::fabs(d_phi - 1.0) < rmp::common::math::kMathEpsilon ||
          std::fabs(d_phi + 1.0) < rmp::common::math::kMathEpsilon)
      {
        continue;
      }
      double k = d_phi / abs_dxi;

      // if the curvature is smaller then the maximum do nothing
      if (k > instance->k_max_)
      {
        double u = 1.0 / abs_dxi / std::sqrt(1 - std::pow(std::cos(d_phi), 2));
        // calculate the p1 and p2 terms
        p1 = ort(d_xi, -d_xi_p1) / (abs_dxi * abs_dxi_p1);
        p2 = -ort(d_xi_p1, d_xi) / (abs_dxi * abs_dxi_p1);

        // calculate the last terms
        Vec2d s = d_phi * d_xi / std::pow(abs_dxi, 3);
        Vec2d ki = u * (-p1 - p2) - s;
        Vec2d ki_c1 = u * p2 + s;
        Vec2d ki_p1 = u * p1;

        // calculate the gradient
        Vec2d gradient = instance->w_curvature_ * (0.25 * ki_c1 + 0.5 * ki + 0.25 * ki_p1);
        if (std::isnan(gradient.x()) || std::isnan(gradient.y()))
        {
          continue;
        }
        curvature_grad(0, i - 2) = gradient.x();
        curvature_grad(1, i - 2) = gradient.y();

        curvature_cost += instance->k_max_ * std::pow(k - instance->k_max_, 2);
      }
    }
  }

  return curvature_grad;
}

}  // namespace trajectory_optimization
}  // namespace rmp