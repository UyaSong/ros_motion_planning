/**
 * *********************************************************
 *
 * @file: trajectory_planner_factory.cpp
 * @brief: Create the trajectory planner with specifical parameters
 * @author: Yang Haodong
 * @date: 2025-02-16
 * @version: 1.0
 *
 * Copyright (c) 2025, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include "path_planner/utils/trajectory_planner_factory.h"

using namespace rmp::trajectory_optimization;

namespace rmp
{
namespace path_planner
{
/**
 * @brief Create and configure planner
 * @param nh ROS node handler
 * @param costmap_ros costmap ROS wrapper
 * @param planner_props planner property
 * @return bool true if create successful, else false
 */
bool TrajectoryPlannerFactory::createPlanner(ros::NodeHandle& nh, costmap_2d::Costmap2DROS* costmap_ros,
                                             PlannerProps& planner_props)
{
  double obstacle_factor;
  std::string optimizer_name;
  nh.param("optimizer_name", optimizer_name, (std::string) "");
  if (optimizer_name == "")
  {
    R_INFO << "No optimizer selected.";
    planner_props.traj_optimizer_ptr = nullptr;
    return true;
  }
  else
  {
    if (optimizer_name == "conjugate_optimizer")
    {
      int max_iter;
      double alpha, obs_dist_max, k_max;
      double w_obstacle, w_smooth, w_curvature;
      nh.param("/move_base/Optimizer/max_iter", max_iter, 100);  // the maximum iterations for optimization
      nh.param("/move_base/Optimizer/alpha", alpha, 0.1);        // learning rate
      nh.param("/move_base/Optimizer/obs_dist_max", obs_dist_max,
               0.3);                                                    // the maximum distance to obstacle (m)
      nh.param("/move_base/Optimizer/k_max", k_max, 0.15);              // the maximum curvature
      nh.param("/move_base/Optimizer/w_obstacle", w_obstacle, 0.50);    // the weight for obstacle avoidance
      nh.param("/move_base/Optimizer/w_smooth", w_smooth, 0.25);        // the weight for smooth
      nh.param("/move_base/Optimizer/w_curvature", w_curvature, 0.25);  // the weight for curvature
      planner_props.traj_optimizer_ptr = std::make_shared<CGOptimizer>(costmap_ros, max_iter, alpha, obs_dist_max,
                                                                       k_max, w_obstacle, w_smooth, w_curvature);
    }
    else if (optimizer_name == "minimumsnap_optimizer")
    {
      int max_iter;
      double vel_max, acc_max, jerk_max;
      nh.param("/move_base/Optimizer/max_iter", max_iter, 100);  // the maximum iterations for optimization
      nh.param("/move_base/Optimizer/vel_max", vel_max, 1.0);    // the maximum velocity (m/s)
      nh.param("/move_base/Optimizer/acc_max", acc_max, 2.0);    // the maximum acceleration (m/s2)
      nh.param("/move_base/Optimizer/jerk_max", jerk_max, 4.0);  // the maximum jerk (m/s3)
      planner_props.traj_optimizer_ptr =
          std::make_shared<MinimumsnapOptimizer>(costmap_ros, max_iter, vel_max, acc_max, jerk_max);
    }
    else if (optimizer_name == "constrained_optimizer")
    {
      bool debug;
      double obs_dist_max, k_max;
      double w_obstacle, w_smooth, w_distance, w_curvature;
      nh.param("/move_base/Optimizer/obs_dist_max", obs_dist_max,
               0.5);                                                    // the maximum distance to obstacle (m)
      nh.param("/move_base/Optimizer/k_max", k_max, 2.5);               // the maximum curvature
      nh.param("/move_base/Optimizer/w_obstacle", w_obstacle, 0.015);   // the weight for obstacle avoidance
      nh.param("/move_base/Optimizer/w_smooth", w_smooth, 2.0e6);       // the weight for smooth
      nh.param("/move_base/Optimizer/w_distance", w_distance, 30.0);    // the weight for distance
      nh.param("/move_base/Optimizer/w_curvature", w_curvature, 30.0);  // the weight for curvature
      nh.param("/move_base/Optimizer/debug", debug, false);             // whether to show debug information
      planner_props.traj_optimizer_ptr = std::make_shared<ConstrainedOptimizer>(
          costmap_ros, obs_dist_max, k_max, w_obstacle, w_smooth, w_distance, w_curvature, debug);
    }
    else if (optimizer_name == "lbfgs_optimizer")
    {
      double obs_dist_max, k_max;
      double w_obstacle, w_smooth, w_curvature;
      nh.param("/move_base/Optimizer/obs_dist_max", obs_dist_max,
               0.3);                                                    // the maximum distance to obstacle (m)
      nh.param("/move_base/Optimizer/k_max", k_max, 0.15);              // the maximum curvature
      nh.param("/move_base/Optimizer/w_obstacle", w_obstacle, 0.50);    // the weight for obstacle avoidance
      nh.param("/move_base/Optimizer/w_smooth", w_smooth, 0.25);        // the weight for smooth
      nh.param("/move_base/Optimizer/w_curvature", w_curvature, 0.25);  // the weight for curvature
      planner_props.traj_optimizer_ptr =
          std::make_shared<LBFGSOptimizer>(costmap_ros, obs_dist_max, k_max, w_obstacle, w_smooth, w_curvature);
    }
    else
    {
      R_ERROR << "Unknown optimizer name: " << optimizer_name;
      return false;
    }

    R_INFO << "Using optimizer: " << optimizer_name;
    planner_props.optimizer_name = optimizer_name;
    return true;
  }
}
}  // namespace path_planner
}  // namespace rmp