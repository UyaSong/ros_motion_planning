/***********************************************************
 *
 * @file: time_allocation.h
 * @breif: time allocation for trajectory generation
 * @author: Yang Haodong
 * @update: 2024-9-30
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef RMP_TRAJECTORY_OPTIMIZATION_TIME_ALLOCATION_H_
#define RMP_TRAJECTORY_OPTIMIZATION_TIME_ALLOCATION_H_

#include <vector>

namespace rmp
{
namespace trajectory_optimization
{
class TimeAllocator
{
public:
  TimeAllocator() = default;
  ~TimeAllocator() = default;

  template <typename Point>
  static void normalAllocation(const std::vector<Point>& waypoints, double v_max, std::vector<double>& time_allocations)
  {
    time_allocations.clear();
    for (int i = 0; i < static_cast<int>(waypoints.size()) - 1; i++)
    {
      const auto& wp_1 = waypoints[i];
      const auto& wp_2 = waypoints[i + 1];
      time_allocations.push_back(std::hypot(wp_1.x() - wp_2.x(), wp_1.y() - wp_2.y()) / v_max);
    }
  }

  template <typename Point>
  static void trapezoidalAllocation(const std::vector<Point>& waypoints, double v_max, double a_max,
                                    std::vector<double>& time_allocations)
  {
    time_allocations.clear();
    for (int i = 0; i < static_cast<int>(waypoints.size()) - 1; i++)
    {
      const auto& wp_1 = waypoints[i];
      const auto& wp_2 = waypoints[i + 1];
      const double dist = std::hypot(wp_1.x() - wp_2.x(), wp_1.y() - wp_2.y());
      double dt = dist > v_max * v_max / a_max ? v_max / a_max + dist / v_max : 2 * std::sqrt(dist / a_max);
      time_allocations.push_back(dt);
    }
  }
};
}  // namespace trajectory_optimization
}  // namespace rmp
#endif