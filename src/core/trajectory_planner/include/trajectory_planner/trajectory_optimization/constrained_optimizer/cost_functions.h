/***********************************************************
 *
 * @file: cost_functions.h
 * @breif: Cost functions of constrained_optimizer
 * @author: Yang Haodong
 * @update: 2025-01-13
 * @version: 1.0
 *
 * Copyright (c) 2025, Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef RMP_TRAJECTORY_OPTIMIZATION_CONSTRAINED_OPTIMIZER_COST_FUNCTIONS_H_
#define RMP_TRAJECTORY_OPTIMIZATION_CONSTRAINED_OPTIMIZER_COST_FUNCTIONS_H_

#include <ceres/ceres.h>
#include <ceres/cubic_interpolation.h>

#include <costmap_2d/costmap_2d.h>

#include "distance_layer.h"

namespace rmp
{
namespace trajectory_optimization
{
/**
 * @brief template version for arcCenter
 */
template <typename T>
inline Eigen::Matrix<T, 2, 1> arcCenter(Eigen::Matrix<T, 2, 1> pt_prev, Eigen::Matrix<T, 2, 1> pt,
                                        Eigen::Matrix<T, 2, 1> pt_next, bool is_cusp)
{
  Eigen::Matrix<T, 2, 1> d1 = pt - pt_prev;
  Eigen::Matrix<T, 2, 1> d2 = pt_next - pt;

  if (is_cusp)
  {
    d2 = -d2;
    pt_next = pt + d2;
  }

  T det = d1[0] * d2[1] - d1[1] * d2[0];
  if (ceres::abs(det) < (T)1e-4)
  {  // straight line
    return Eigen::Matrix<T, 2, 1>((T)std::numeric_limits<double>::infinity(),
                                  (T)std::numeric_limits<double>::infinity());
  }

  Eigen::Matrix<T, 2, 1> mid1 = (pt_prev + pt) / (T)2;
  Eigen::Matrix<T, 2, 1> mid2 = (pt + pt_next) / (T)2;
  Eigen::Matrix<T, 2, 1> n1(-d1[1], d1[0]);
  Eigen::Matrix<T, 2, 1> n2(-d2[1], d2[0]);
  T det1 = (mid1[0] + n1[0]) * mid1[1] - (mid1[1] + n1[1]) * mid1[0];
  T det2 = (mid2[0] + n2[0]) * mid2[1] - (mid2[1] + n2[1]) * mid2[0];
  Eigen::Matrix<T, 2, 1> center((det1 * n2[0] - det2 * n1[0]) / det, (det1 * n2[1] - det2 * n1[1]) / det);
  return center;
}

class OptimizerCostFunction
{
public:
  OptimizerCostFunction(const Eigen::Vector3d& origin_pose, costmap_2d::Costmap2D* costmap,
                        const std::shared_ptr<ceres::BiCubicInterpolator<ceres::Grid2D<double>>>& costmap_interpolator,
                        double obs_dist_max_, double k_max, double w_obstacle, double w_smooth, double w_distance,
                        double w_curvature)
    : origin_pose_(origin_pose)
    , costmap_(costmap)
    , costmap_interpolator_(costmap_interpolator)
    , obs_dist_max_(obs_dist_max_)
    , k_max_(k_max)
    , w_obstacle_(w_obstacle)
    , w_smooth_(w_smooth)
    , w_distance_(w_distance)
    , w_curvature_(w_curvature)
  {
  }

  ~OptimizerCostFunction() = default;

  ceres::CostFunction* AutoDiff()
  {
    return new ceres::AutoDiffCostFunction<OptimizerCostFunction, 4, 2, 2, 2>(this);
  }

  /**
   * @brief Optimizer cost function evaluation
   * @param pt_prev Point Xi-1 for calculating Xi's cost
   * @param pt_curr Point Xi for evaluation
   * @param pt_next Point Xi+1 for calculating Xi's cost
   * @param pt_residual array of output residuals (smoothing, curvature, distance, cost)
   * @return if successful in computing values
   */
  template <typename T>
  bool operator()(const T* const pt_prev, const T* const pt_curr, const T* const pt_next, T* pt_residual) const
  {
    Eigen::Map<const Eigen::Matrix<T, 2, 1>> xi_prev(pt_prev);
    Eigen::Map<const Eigen::Matrix<T, 2, 1>> xi_curr(pt_curr);
    Eigen::Map<const Eigen::Matrix<T, 2, 1>> xi_next(pt_next);
    Eigen::Map<Eigen::Matrix<T, 4, 1>> residual(pt_residual);
    residual.setZero();

    // compute cost
    addSmoothingResidual<T>(xi_prev, xi_curr, xi_next, residual[0]);
    addCurvatureResidual<T>(xi_prev, xi_curr, xi_next, residual[1]);
    addDistanceResidual<T>(xi_curr, (origin_pose_.template block<2, 1>(0, 0)).template cast<T>(), residual[2]);
    addCostResidual<T>(xi_curr, residual[3]);

    return true;
  }

protected:
  /**
   * @brief Cost function term for smooth paths
   * @param pt_prev Point Xi-1 for calculating Xi's cost
   * @param pt_curr Point Xi for evaluation
   * @param pt_next Point Xi+1 for calculating Xi's cost
   * @param r Residual (cost) of term
   */
  template <typename T>
  inline void addSmoothingResidual(const Eigen::Matrix<T, 2, 1>& pt_prev, const Eigen::Matrix<T, 2, 1>& pt,
                                   const Eigen::Matrix<T, 2, 1>& pt_next, T& r) const
  {
    Eigen::Matrix<T, 2, 1> d_next = pt_next - pt;
    Eigen::Matrix<T, 2, 1> d_prev = pt - pt_prev;
    T next_to_prev_ratio = d_next.template block<2, 1>(0, 0).norm() / d_prev.template block<2, 1>(0, 0).norm();
    Eigen::Matrix<T, 2, 1> d_diff = next_to_prev_ratio * d_next - d_prev;
    r += (T)w_smooth_ * d_diff.dot(d_diff);
  }

  /**
   * @brief Cost function term for maximum curved paths
   * @param pt_prev Point Xi-1 for calculating Xi's cost
   * @param pt_curr Point Xi for evaluation
   * @param pt_next Point Xi+1 for calculating Xi's cost
   * @param r Residual (cost) of term
   */
  template <typename T>
  inline void addCurvatureResidual(const Eigen::Matrix<T, 2, 1>& pt_prev, const Eigen::Matrix<T, 2, 1>& pt_curr,
                                   const Eigen::Matrix<T, 2, 1>& pt_next, T& r) const
  {
    Eigen::Matrix<T, 2, 1> center = arcCenter(pt_prev, pt_curr, pt_next, false);
    if (ceres::IsInfinite(center[0]))
    {
      return;
    }

    T turning_rad = (pt_curr - center).norm();
    T ki_minus_kmax = (T)1.0 / turning_rad - k_max_;

    if (ki_minus_kmax <= (T)rmp::common::math::kMathEpsilon)
    {
      return;
    }

    r += (T)w_curvature_ * ki_minus_kmax * ki_minus_kmax;  // objective function value
  }

  /**
   * @brief Cost function derivative term for steering away changes in pose
   * @param xi Point Xi for evaluation
   * @param xi_original original point Xi for evaluation
   * @param r Residual (cost) of term
   */
  template <typename T>
  inline void addDistanceResidual(const Eigen::Matrix<T, 2, 1>& xi, const Eigen::Matrix<T, 2, 1>& xi_original,
                                  T& r) const
  {
    r += (T)w_distance_ * (xi - xi_original).squaredNorm();  // objective function value
  }

  /**
   * @brief Cost function term for steering away from costs
   * @param xi Point Xi for evaluation
   * @param r Residual (cost) of term
   */
  template <typename T>
  inline void addCostResidual(const Eigen::Matrix<T, 2, 1>& xi, T& r) const
  {
    T resolution = (T)costmap_->getResolution();
    Eigen::Matrix<T, 2, 1> costmap_origin(costmap_->getOriginX(), costmap_->getOriginY());
    Eigen::Matrix<T, 2, 1> interp_pos = (xi - costmap_origin.template cast<T>()) / resolution;
    T value;
    costmap_interpolator_->Evaluate((T)costmap_->getSizeInCellsY() - interp_pos[1] - (T)0.5, interp_pos[0] - (T)0.5,
                                    &value);
    T cost = value * resolution * resolution;
    cost = cost > (T)(obs_dist_max_ * obs_dist_max_) ? (T)0 : cost;
    r += (T)w_distance_ * cost;  // esdf

    // r += (T)w_distance_ * value * value;  // costmap
  }

private:
  const Eigen::Vector3d origin_pose_;
  costmap_2d::Costmap2D* costmap_;
  std::shared_ptr<ceres::BiCubicInterpolator<ceres::Grid2D<double>>> costmap_interpolator_;
  double obs_dist_max_;  // the maximum distance to obstacle (m)
  double k_max_;         // the maximum curvature
  double w_obstacle_;    // the weight for obstacle avoidance
  double w_smooth_;      // the weight for smooth
  double w_distance_;    // the weight for distance
  double w_curvature_;   // the weight for curvature
};
}  // namespace trajectory_optimization
}  // namespace rmp
#endif