/**
 * *********************************************************
 *
 * @file: voronoi_planner.cpp
 * @brief: Contains the Voronoi-based planner class
 * @author: Yang Haodong
 * @date: 2023-07-21
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include "path_planner/roadmap_planner/voronoi_planner.h"

namespace rmp
{
namespace path_planner
{
using namespace rmp::common::geometry;

std::vector<VoronoiPathPlanner::Node> VoronoiPathPlanner::motions_ = {
  { 0, 1, 1.0 },          { 1, 0, 1.0 },           { 0, -1, 1.0 },          { -1, 0, 1.0 },
  { 1, 1, std::sqrt(2) }, { 1, -1, std::sqrt(2) }, { -1, 1, std::sqrt(2) }, { -1, -1, std::sqrt(2) },
};

/**
 * @brief Construct a new Voronoi-based planning object
 * @param costmap   the environment for path planning
 * @param obstacle_factor obstacle factor(greater means obstacles)
 */
VoronoiPathPlanner::VoronoiPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, double obstacle_factor)
  : RoadmapPathPlannerBase(costmap_ros, obstacle_factor)
{
}

/**
 * @brief Voronoi-based planning implementation
 * @param start         start node
 * @param goal          goal node
 * @param path          optimal path consists of Node
 * @param expand        containing the node been search during the process
 * @return  true if path found, else false
 */
bool VoronoiPathPlanner::plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand)
{
  double m_start_x, m_start_y, m_goal_x, m_goal_y;
  if ((!validityCheck(start.x(), start.y(), m_start_x, m_start_y)) ||
      (!validityCheck(goal.x(), goal.y(), m_goal_x, m_goal_y)))
  {
    return false;
  }

  // update voronoi diagram
  _updateVoronoi();

  const Vec2d start_vec(m_start_x, m_start_y);
  const Vec2d goal_vec(m_goal_x, m_goal_y);
  int start_idx = 0, goal_idx = 0;
  double dist2start = std::numeric_limits<double>::infinity();
  double dist2goal = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < vertices_.size(); ++i)
  {
    const auto& vertex = vertices_[i];
    if (!_isCollision(m_start_x, m_start_y, vertex.x(), vertex.y()))
    {
      const double dist = (start_vec - vertex).length();
      if (dist < dist2start)
      {
        dist2start = dist;
        start_idx = grid2Index(vertex.x(), vertex.y());
      }
    }
    if (!_isCollision(m_goal_x, m_goal_y, vertex.x(), vertex.y()))
    {
      const double dist = (goal_vec - vertex).length();
      if (dist < dist2goal)
      {
        dist2goal = dist;
        goal_idx = grid2Index(vertex.x(), vertex.y());
      }
    }
  }

  // search on voronoi map
  std::vector<int> v_path;
  if (_searchOnRoadMap(roadmap_, start_idx, goal_idx, v_path))
  {
    path.clear();
    path.emplace_back(start);
    if (v_path.size() > 2)
    {
      for (size_t i = 1; i < v_path.size() - 1; ++i)
      {
        // convert to world frame
        int mx, my;
        double wx, wy;
        index2Grid(v_path[i], mx, my);
        costmap_->mapToWorld(mx, my, wx, wy);
        path.emplace_back(wx, wy);
      }
    }
    path.emplace_back(goal);

    expand.clear();
    for (const auto& edge : valid_edges_)
    {
      const auto& pt1 = edge.first;
      const auto& pt2 = edge.second;
      int pid = grid2Index(pt2.x(), pt2.y());
      expand.emplace_back(pt1.x(), pt1.y(), pid);
    }

    return true;
  }
  else
  {
    return false;
  }
}

/**
 * @brief update voronoi object using costmap ROS wrapper
 */
void VoronoiPathPlanner::_updateVoronoi()
{
  bool voronoi_layer_exist = false;
  for (auto layer = costmap_ros_->getLayeredCostmap()->getPlugins()->begin();
       layer != costmap_ros_->getLayeredCostmap()->getPlugins()->end(); ++layer)
  {
    boost::shared_ptr<costmap_2d::VoronoiLayer> voronoi_layer =
        boost::dynamic_pointer_cast<costmap_2d::VoronoiLayer>(*layer);
    if (voronoi_layer)
    {
      voronoi_layer_exist = true;
      boost::unique_lock<boost::mutex> lock(voronoi_layer->getMutex());
      voronoi_ = voronoi_layer->getVoronoi();
      break;
    }
  }
  if (!voronoi_layer_exist)
  {
    R_ERROR << "Failed to get a Voronoi layer for potentional application.";
  }
  // update roadmap
  roadmap_.clear();
  vertices_.clear();
  valid_edges_.clear();
  vertices_.reserve(map_size_);
  roadmap_.resize(map_size_);
  valid_edges_.reserve(map_size_);
  for (int j = 0; j < ny_; j++)
  {
    for (int i = 0; i < nx_; i++)
    {
      if (voronoi_.isVoronoi(i, j))
      {
        vertices_.emplace_back(i, j);
        const int cur_index = grid2Index(i, j);
        for (const auto& motion : motions_)
        {
          const int new_x = i + motion.x();
          const int new_y = j + motion.y();
          if (new_x >= 0 && new_x < nx_ && new_y >= 0 && new_y < ny_ && voronoi_.isVoronoi(new_x, new_y))
          {
            valid_edges_.emplace_back(Vec2d(new_x, new_y), Vec2d(i, j));
            roadmap_[cur_index].emplace_back(grid2Index(new_x, new_y), motion.g());
          }
        }
      }
    }
  }
}

}  // namespace path_planner
}  // namespace rmp