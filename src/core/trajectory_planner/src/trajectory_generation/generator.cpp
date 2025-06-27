/***********************************************************
 *
 * @file: generator.cpp
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
#include "path_planner/path_prune/ramer_douglas_peucker.h"
#include "trajectory_planner/trajectory_generation/generator.h"

using namespace rmp::path_planner;

namespace rmp
{
namespace trajectory_generation
{
/**
 * @brief Construct a new trajectory generator object
 * @param costmap_ros costmap ROS wrapper
 */
Generator::Generator() : costmap_ros_(nullptr), nx_(0), ny_(0), map_size_(0)
{
}

Generator::Generator(costmap_2d::Costmap2DROS* costmap_ros)
  : costmap_ros_(costmap_ros)
  , nx_(costmap_ros->getCostmap()->getSizeInCellsX())
  , ny_(costmap_ros->getCostmap()->getSizeInCellsY())
  , map_size_(costmap_ros->getCostmap()->getSizeInCellsX() * costmap_ros->getCostmap()->getSizeInCellsY())
  , path_processor_(std::make_unique<RDPPathProcessor>(0.25, 3.5))
{
}

/**
 * @brief Set the waypoints from planning path
 * @param waypoints the waypoints
 */
void Generator::setWaypoints(const Points3d& waypoints, bool is_prune)
{
  Points3d prune_waypoints;
  waypoints_.clear();
  if (is_prune)
  {
    path_processor_->process(waypoints, prune_waypoints);
    waypoints_.reserve(prune_waypoints.size());
    for (const auto& wp : prune_waypoints)
    {
      waypoints_.emplace_back(wp.x(), wp.y(), wp.theta());
    }
  }
  else
  {
    waypoints_.reserve(waypoints.size());
    for (const auto& wp : waypoints)
    {
      waypoints_.emplace_back(wp.x(), wp.y(), wp.theta());
    }
  }
}

/**
 * @brief Judge whether the grid(x, y) is inside the map
 * @param x grid coordinate x
 * @param y grid coordinate y
 * @return true if inside the map else false
 */
bool Generator::_insideMap(unsigned int x, unsigned int y)
{
  return (x < nx_ && x >= 0 && y < ny_ && y >= 0) ? true : false;
}

/**
 * @brief Judge whether the grid(x, y) is inside the map
 * @param x world coordinate x
 * @param y world coordinate y
 * @return true if inside the map else false
 */
bool Generator::_insideMap(double x, double y)
{
  unsigned int ix, iy;
  costmap_ros_->getCostmap()->worldToMap(x, y, ix, iy);
  return (ix < nx_ && ix >= 0 && iy < ny_ && iy >= 0) ? true : false;
}
}  // namespace trajectory_generation
}  // namespace rmp