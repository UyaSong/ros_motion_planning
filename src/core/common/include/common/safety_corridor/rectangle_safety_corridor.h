/**
 * *********************************************************
 *
 * @file: rectangle_safety_corridor.h
 * @brief: rectangle safety corridor class
 * @author: Yang Haodong
 * @paper: Optimization-Based Trajectory Planning for Autonomous Parking With
      Irregularly Placed Obstacles: A Lightweight Iterative Framework
 * @date: 2024-11-13
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_COMMON_SAFETYCORRIDOR_RECTANGLE_SAFETY_CORRIDOR_H_
#define RMP_COMMON_SAFETYCORRIDOR_RECTANGLE_SAFETY_CORRIDOR_H_

#include "common/geometry/polygon2d.h"
#include "common/safety_corridor/safety_corridor.h"

namespace rmp
{
namespace common
{
namespace safety_corridor
{
class RectangleSafetyCorridor : public SafetyCorridor
{
private:
  using Vec2d = rmp::common::geometry::Vec2d;
  using Polygon2d = rmp::common::geometry::Polygon2d;

public:
  /**
   * @brief Empty constructor.
   */
  RectangleSafetyCorridor();

  /**
   * @brief Constructor which takes a vector of obstacle points as its barrier.
   * @param obstacle_pts The obstacle points to construct the safety corridor.
   * @param safety_range The considered safety range.
   * @param max_iteration The maximum iterations for corridor construction
   */
  RectangleSafetyCorridor(const Points3d& obstacle_pts, double safety_range, int max_iteration);
  RectangleSafetyCorridor(costmap_2d::Costmap2DROS* costmap_ros, double safety_range, int max_iteration);

  virtual ~RectangleSafetyCorridor() = default;

  /**
   * @brief Construct a safety corridor composed of polygons based on waypoints and environmental obstacles。
   * @param waypoints The waypoints
   * @param decomp_polygons The decomposed polygons to construct the safety corridor.
   * @param prune_waypoints The overlap between rectangles may result in redundancy. Pruning is applied to retain only
   the minimal set of corridor overlap points as the pruning points.
   * @return true if successful construction.
   * @note direction
   *       			  3
              ________
             |		    |
          0  | (x, y) | 2
             |________|
                  1
   */
  bool decompose(const Points3d& waypoints, std::vector<Polygon2d>& decomp_polygons, Points3d& prune_waypoints) const;

protected:
  /**
   * @brief Determine if a rectangle collides with static obstacles.
   * @param lx/ly/ux/uy rectangle vertice [pixel]
   * @return true if collision occurs, else false
   * @note
   * 			       _________ (ux, uy)
                |	   	   |
      (lx, ly) |_________|
   */
  bool isRectCollision(int lx, int ly, int ux, int uy) const;

  /**
   * @brief Determine if a rectangle collides with static obstacles incrementally.
   * @param edge edge index
   * @param x/y center of initial rectangle [pixel]
   * @param increment incremental array of each edge
   * @return true if collision occurs, else false
   */
  bool isEdgeCollision(int edge, int x, int y, const std::vector<int>& increment) const;

protected:
  double safety_range_;  // The safety range to limit polygon space [m]
  int max_iteration_;    // maximum iteration for corridor construction
};
}  // namespace safety_corridor
}  // namespace common
}  // namespace rmp
#endif