/**
 * *********************************************************
 *
 * @file: rectangle_safety_corridor.cpp
 * @brief: rectangle safety corridor class
 * @author: Yang Haodong
 * @date: 2024-11-14
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include "common/util/log.h"
#include "common/math/math_helper.h"
#include "common/geometry/collision_checker.h"
#include "common/safety_corridor/rectangle_safety_corridor.h"

namespace rmp
{
namespace common
{
namespace safety_corridor
{
namespace
{
// radius of initial approved region [m]
constexpr double radius = 0.4;
}  // namespace
/**
 * @brief Empty constructor.
 */
RectangleSafetyCorridor::RectangleSafetyCorridor() : SafetyCorridor(){};

/**
 * @brief Constructor which takes a vector of obstacle points as its barrier.
 * @param obstacle_pts The obstacle points to construct the safety corridor.
 * @param safety_range The considered safety range.
 * @param max_iteration The maximum iterations for corridor construction
 */
RectangleSafetyCorridor::RectangleSafetyCorridor(const Points3d& obstacle_pts, double safety_range, int max_iteration)
  : SafetyCorridor(obstacle_pts), safety_range_(safety_range), max_iteration_(max_iteration){};

RectangleSafetyCorridor::RectangleSafetyCorridor(costmap_2d::Costmap2DROS* costmap_ros, double safety_range,
                                                 int max_iteration)
  : SafetyCorridor(costmap_ros), safety_range_(safety_range), max_iteration_(max_iteration){};

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
bool RectangleSafetyCorridor::decompose(const Points3d& waypoints, std::vector<Polygon2d>& decomp_polygons,
                                        Points3d& prune_waypoints) const
{
  int step = 1;
  prune_waypoints.clear();
  decomp_polygons.clear();
  int nums = static_cast<int>(waypoints.size());
  double resolution = costmap_ros_->getCostmap()->getResolution();
  int safety_range_in_pixel = static_cast<int>(safety_range_ / resolution);
  for (int i = 0; i < nums - 1; i++)
  {
    Vec2d wp(waypoints[i].x(), waypoints[i].y());
    if (decomp_polygons.size() > 0 && decomp_polygons.back().isPointIn(wp))
    {
      continue;
    }

    // adjustment for initial (x, y)
    unsigned int mx, my;
    costmap_ros_->getCostmap()->worldToMap(wp.x(), wp.y(), mx, my);
    int x = static_cast<int>(mx), y = static_cast<int>(my);
    int init_step = static_cast<int>(radius / resolution);
    if (isRectCollision(x - init_step, y - init_step, x + init_step, y + init_step))
    {
      int inc = 4;
      int real_x, real_y;
      while (inc < max_iteration_)
      {
        int it = inc / 4;
        int edge = inc % 4;
        inc += 1;
        real_x = x, real_y = y;
        if (edge == 0)
        {
          real_x = int(x - it * step);
        }
        else if (edge == 1)
        {
          real_y = int(y - it * step);
        }
        else if (edge == 2)
        {
          real_x = int(x + it * step);
        }
        else
        {
          real_y = int(y + it * step);
        }

        // collision detection
        if (!isRectCollision(real_x - init_step, real_y - init_step, real_x + init_step + 1, real_y + init_step + 1))
        {
          break;
        }
      }
      if (inc >= max_iteration_)
      {
        return false;
      }
      else
      {
        x = real_x;
        y = real_y;
      }
    }

    // main loop for construction in (x, y)
    int inc = 4;
    std::vector<int> block(4, 0), increment(4, 0);
    while (inc < max_iteration_ && (std::accumulate(block.begin(), block.end(), 0) != 4))
    {
      int it = inc / 4;
      int edge = inc % 4;
      inc += 1;

      if (block[edge])
      {
        continue;
      }

      increment[edge] = static_cast<int>(it * step);

      if (increment[edge] >= safety_range_in_pixel || isEdgeCollision(edge, x, y, increment))
      {
        // increment[edge] -= step
        block[edge] = 1;
      }
    }

    if (inc >= max_iteration_)
    {
      return false;
    }
    else
    {  // save
      prune_waypoints.emplace_back(wp.x(), wp.y());
      Polygon2d polygon({ { wp.x() + increment[2] * resolution, wp.y() + increment[3] * resolution },
                          { wp.x() - increment[0] * resolution, wp.y() + increment[3] * resolution },
                          { wp.x() - increment[0] * resolution, wp.y() - increment[1] * resolution },
                          { wp.x() + increment[2] * resolution, wp.y() - increment[1] * resolution } });
      decomp_polygons.emplace_back(std::move(polygon));
    }
  }
  return true;
}

/**
 * @brief Determine if a rectangle collides with static obstacles.
 * @param lx/ly/ux/uy rectangle vertice
 * @return true if collision occurs, else false
 * @note
 * 			       _________ (ux, uy)
              |	   	   |
    (lx, ly) |_________|
 */
bool RectangleSafetyCorridor::isRectCollision(int lx, int ly, int ux, int uy) const
{
  for (int x = lx; x < ux; x++)
  {
    for (int y = ly; y < uy; y++)
    {
      Point2i pt(x, y);
      if (isObstacleInMap(pt))
      {
        return true;
      }
    }
  }
  return false;
}

/**
 * @brief Determine if a rectangle collides with static obstacles incrementally.
 * @param edge edge index
 * @param x/y center of initial rectangle [pixel]
 * @param increment incremental array of each edge
 * @return true if collision occurs, else false
 */
bool RectangleSafetyCorridor::isEdgeCollision(int edge, int x, int y, const std::vector<int>& increment) const
{
  if (edge == 0 || edge == 2)
  {
    int new_x = edge == 0 ? x - increment[edge] : x + increment[edge];
    for (int iy = y - increment[1]; iy <= y + increment[3]; iy++)
    {
      Point2i pt(new_x, iy);
      if (isObstacleInMap(pt))
      {
        return true;
      }
    }
    return false;
  }
  else if (edge == 1 || edge == 3)
  {
    int new_y = edge == 1 ? y - increment[edge] : y + increment[edge];
    for (int ix = x - increment[0]; ix <= x + increment[2]; ix++)
    {
      Point2i pt(ix, new_y);
      if (isObstacleInMap(pt))
      {
        return true;
      }
    }
    return false;
  }
  else
  {
    R_ERROR << "Edge index must be 0, 1, 2, 3.";
    return false;
  }
}
}  // namespace safety_corridor
}  // namespace common
}  // namespace rmp