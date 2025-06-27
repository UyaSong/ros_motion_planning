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
#include "common/util/log.h"
#include "common/math/math_helper.h"
#include "trajectory_planner/trajectory_generation/cubic_bezier_generator.h"

using namespace rmp::common::geometry;

namespace rmp
{
namespace trajectory_generation
{
/**
 * @brief Construct a new trajectory generator object
 * @param costmap_ros costmap ROS wrapper
 * @param sample_ratio Simulation or interpolation size [m]
 * @param offset The offset of control points [m]
 * @param keep_start_orientation whether to keep start pose orientation
 * @param keep_goal_orientation whether to keep goal pose orientation
 */
CubicBezierGenerator::CubicBezierGenerator(costmap_2d::Costmap2DROS* costmap_ros, double sample_ratio, double offset,
                                           bool keep_start_orientation, bool keep_goal_orientation)
  : Generator(costmap_ros)
  , sample_ratio_(sample_ratio)
  , offset_(offset)
  , keep_start_orientation_(keep_start_orientation)
  , keep_goal_orientation_(keep_goal_orientation)
  , bezier_gen_(std::make_unique<BezierCurve>(sample_ratio, offset))
{
}

bool CubicBezierGenerator::generation()
{
  if (waypoints_.empty())
  {
    R_INFO << "Cubic Bezier Generation need to set waypoints first.";
    return false;
  }
  trajectory_.clear();

  if (waypoints_.size() <= 2)
  {
    for (const auto& pt : waypoints_)
    {
      trajectory_.position.emplace_back(pt.x(), pt.y(), pt.theta());
    }
    return true;
  }

  auto prelast_dir = Vec2d(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());

  for (size_t i = 1; i <= waypoints_.size(); i++)
  {
    if ((!std::isinf(prelast_dir.x())) && (!std::isinf(prelast_dir.y())))
    {
      auto last_dir = Vec2d();
      auto prelast_vec = Vec2d(waypoints_[i - 2].x(), waypoints_[i - 2].y());
      auto last_vec = Vec2d(waypoints_[i - 1].x(), waypoints_[i - 1].y());
      // compute orientation of last point
      if (i < waypoints_.size())
      {
        auto current_vec = Vec2d(waypoints_[i].x(), waypoints_[i].y());
        auto tangent_dir = rmp::common::math::tangentDir(prelast_vec, last_vec, current_vec, false);
        last_dir = tangent_dir.innerProd(current_vec - last_vec) >= 0 ? tangent_dir : -tangent_dir;
        last_dir.normalize();
      }
      else if (keep_goal_orientation_)
      {
        last_dir = Vec2d(std::cos(waypoints_.back().theta()), std::sin(waypoints_.back().theta()));
      }
      else
      {
        last_dir = last_vec - prelast_vec;
        last_dir.normalize();
      }

      // Interpolate poses between prelast and last
      auto prelast_pt = Point3d(waypoints_[i - 2].x(), waypoints_[i - 2].y(), prelast_dir.angle());
      auto last_pt = Point3d(waypoints_[i - 1].x(), waypoints_[i - 1].y(), last_dir.angle());
      Points3d interp_pts;
      if (!bezier_gen_->generation(prelast_pt, last_pt, interp_pts))
      {
        return false;
      }
      trajectory_.position.insert(trajectory_.position.end(), interp_pts.begin(), interp_pts.end() - 1);
      if (i == waypoints_.size())
      {
        trajectory_.position.emplace_back(last_pt);
      }
      prelast_dir = last_dir;
    }
    else
    {
      // start pose
      auto dir = keep_start_orientation_ ?
                     Vec2d(std::cos(waypoints_[0].theta()), std::sin(waypoints_[0].theta())) :
                     Vec2d(waypoints_[1].x() - waypoints_[0].x(), waypoints_[1].y() - waypoints_[0].y());
      dir.normalize();
      trajectory_.position.emplace_back(waypoints_[0].x(), waypoints_[0].y(), dir.angle());
      prelast_dir = dir;
    }
  }
  return true;
}

/**
 * @brief Get the optimized trajectory
 * @param traj the trajectory buffer
 * @return true if optimizes successfully, else failed
 */
bool CubicBezierGenerator::getTrajectory(Trajectory3d& traj)
{
  size_t traj_size = trajectory_.position.size();
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
    const auto& pt = trajectory_.position[i];
    traj.time.push_back(t);
    traj.position.emplace_back(pt.x(), pt.y(), pt.theta());

    if (i < traj_size - 1)
    {
      const auto& pt_next = trajectory_.position[i + 1];
      const double vx = (pt_next.x() - pt.x()) / dt;
      const double vy = (pt_next.y() - pt.y()) / dt;
      traj.velocity.emplace_back(vx, vy);
      if (i < traj_size - 2)
      {
        const auto& pt_next_next = trajectory_.position[i + 2];
        const double vx_next = (pt_next_next.x() - pt_next.x()) / dt;
        const double vy_next = (pt_next_next.y() - pt_next.y()) / dt;
        traj.acceletation.emplace_back((vx_next - vx) / dt, (vy_next - vy) / dt);
      }
    }
    t += dt;
  }

  return true;
}

}  // namespace trajectory_generation
}  // namespace rmp
