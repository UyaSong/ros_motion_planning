/***********************************************************
 *
 * @file: cubic_bezier_generator.h
 * @breif: Trajectory generation using Cubic Bezier
 * @author: Yang Haodong
 * @update: 2025-1-12
 * @version: 1.0
 *
 * Copyright (c) 2025, Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef RMP_TRAJECTORY_GENERATION_CUBIC_BEZIER_GENERATOR_H_
#define RMP_TRAJECTORY_GENERATION_CUBIC_BEZIER_GENERATOR_H_

#include "common/geometry/curve/bezier_curve.h"
#include "trajectory_planner/trajectory_generation/generator.h"

namespace rmp
{
namespace trajectory_generation
{
class CubicBezierGenerator : public Generator
{
public:
  /**
   * @brief Construct a new trajectory generator object
   * @param costmap_ros costmap ROS wrapper
   * @param sample_ratio Simulation or interpolation size [m]
   * @param offset The offset of control points [m]
   * @param keep_start_orientation whether to keep start pose orientation
   * @param keep_goal_orientation whether to keep goal pose orientation
   */
  CubicBezierGenerator(costmap_2d::Costmap2DROS* costmap_ros, double sample_ratio, double offset,
                       bool keep_start_orientation, bool keep_goal_orientation);
  ~CubicBezierGenerator() = default;

  /**
   * @brief Running trajectory generation
   * @return true if optimizes successfully, else failed
   */
  bool generation();
  /**
   * @brief Get the optimized trajectory
   * @param traj the trajectory buffer
   * @return true if optimizes successfully, else failed
   */
  bool getTrajectory(Trajectory3d& traj);

private:
  double sample_ratio_;
  double offset_;
  bool keep_start_orientation_;
  bool keep_goal_orientation_;
  Trajectory3d traj_gen_;
  std::unique_ptr<rmp::common::geometry::BezierCurve> bezier_gen_;
};
}  // namespace trajectory_generation
}  // namespace rmp

#endif