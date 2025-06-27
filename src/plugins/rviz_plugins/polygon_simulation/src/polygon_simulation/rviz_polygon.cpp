/**
 * *********************************************************
 *
 * @file: rviz_polygon.cpp
 * @brief: Polygon object for rviz plugin
 * @author: Yang Haodong
 * @date: 2024-8-20
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <rviz/ogre_helpers/movable_text.h>

#include "polygon_simulation/rviz_polygon.h"

namespace rmp
{
namespace polygon_simulation
{
/**
 * @brief Construct a new Visual polygon object
 * @param idx        polygon index
 */
VPolygon::VPolygon() : VPolygonNode()
{
  mode_ = POLYGON_MODE_GENERAL;
}

VPolygon::VPolygon(unsigned int idx) : VPolygonNode(idx)
{
  mode_ = POLYGON_MODE_GENERAL;
}

/**
 * @brief Destroy the Visual polygon object
 */
VPolygon::~VPolygon()
{
}

/**
 * @brief Set parameters for the polygon
 * @param params        parameters map <key, val>
 */
void VPolygon::setPolygonParams(std::unordered_map<std::string, float>& params)
{
  auto find = [&](std::string key) -> bool { return params.find(key) != params.end(); };
  lasso_mode_ = find("POLYGON_LASSO_MODE") ? static_cast<bool>(params["POLYGON_LASSO_MODE"]) : lasso_mode_;
  is_convex_ = find("POLYGON_CONVEX") ? static_cast<bool>(params["POLYGON_CONVEX"]) : lasso_mode_;
}

/**
 * @brief Trigger polygon rendering
 */
void VPolygon::render()
{
  Ogre::ManualObject* points = dynamic_cast<Ogre::ManualObject*>(point_node_->getAttachedObject(point_idx_));
  Ogre::ManualObject* lines = dynamic_cast<Ogre::ManualObject*>(line_node_->getAttachedObject(line_idx_));

  if (!lasso_mode_)
  {
    drawPoints(points, points_);
  }

  drawPoints(points, { center_ }, true);

  if (size() > 1)
  {
    drawLines(lines, points_);
  }
}

bool VPolygon::is_convex() const
{
  return is_convex_;
}

/**
 * @brief Collision detection
 * @param other   other polygon instance
 * @return flag   collision occurs (true) or not (false)
 */
bool VPolygon::isCollisionWith(const std::shared_ptr<VPolygonNode>& other)
{
  if (other->empty() || !other->valid())
    return false;

  if (other->mode() == POLYGON_MODE_GENERAL)
  {
    auto other_poly = std::dynamic_pointer_cast<VPolygon>(other);
    if (is_convex_ && other_poly->is_convex())
    {
      return _SAT(other_poly);
    }
    return false;
  }
  else
  {
    return false;
  }
}

bool VPolygon::_SAT(const std::shared_ptr<VPolygon>& other)
{
  // 0) useful functions
  auto project = [](const std::vector<Ogre::Vector3>& vertices, const Ogre::Vector3& axis, double& proj_min,
                    double& proj_max) {
    proj_min = std::numeric_limits<double>::max();
    proj_max = std::numeric_limits<double>::min();
    for (const auto& pt : vertices)
    {
      double res = pt.dotProduct(axis);
      proj_max = std::max(res, proj_max);
      proj_min = std::min(res, proj_min);
    }
  };

  // 1) get separating axes
  std::vector<Ogre::Vector3> axes;
  for (int i = 0; i < size(); ++i)
  {
    const auto& pt1 = points_[i];
    const auto& pt2 = points_[next(i)];
    const auto& edge = pt2 - pt1;
    Ogre::Vector3 nor(edge.y, -edge.x, 0.0);
    nor.normalise();
    axes.emplace_back(std::move(nor));
  }

  for (int i = 0; i < other->size(); ++i)
  {
    const auto& pt1 = other->points()[i];
    const auto& pt2 = other->points()[other->next(i)];
    const auto& edge = pt2 - pt1;
    Ogre::Vector3 nor(edge.y, -edge.x, 0.0);
    nor.normalise();
    axes.emplace_back(std::move(nor));
  }

  // 2) projection
  for (const auto& axis : axes)
  {
    double proj_1_min, proj_1_max;
    double proj_2_min, proj_2_max;
    project(points_, axis, proj_1_min, proj_1_max);
    project(other->points(), axis, proj_2_min, proj_2_max);
    // overlap
    if (!(proj_1_min <= proj_2_max && proj_2_min <= proj_1_max))
    {
      return false;
    }
  }
  return true;
}

}  // namespace polygon_simulation
}  // namespace rmp