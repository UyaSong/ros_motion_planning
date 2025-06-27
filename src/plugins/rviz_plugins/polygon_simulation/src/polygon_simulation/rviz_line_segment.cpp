/**
 * *********************************************************
 *
 * @file: rviz_line_segement.cpp
 * @brief: Line segment object for rviz plugin
 * @author: Yang Haodong
 * @date: 2024-9-15
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <ros/ros.h>
#include "polygon_simulation_utils/polygon_simulation_utils.h"
#include "polygon_simulation/rviz_line_segment.h"
#include "polygon_simulation/rviz_circle.h"
#include "polygon_simulation/rviz_rectangle.h"
#include "polygon_simulation/rviz_discs_model.h"

namespace rmp
{
namespace polygon_simulation
{
/**
 * @brief Construct a new Visual circle object
 * @param idx        polygon index
 */
VLineSegment::VLineSegment() : VPolygonNode()
{
  mode_ = POLYGON_MODE_LINESEGEMENT;
}

VLineSegment::VLineSegment(unsigned int idx) : VPolygonNode(idx)
{
  mode_ = POLYGON_MODE_LINESEGEMENT;
}

/**
 * @brief Destroy the Visual circle object
 */
VLineSegment::~VLineSegment()
{
}

/**
 * @brief Get the start of line segment
 */
const Ogre::Vector3& VLineSegment::start() const
{
  return start_;
}

/**
 * @brief Get the end of line segment
 */
const Ogre::Vector3& VLineSegment::end() const
{
  return end_;
}

/**
 * @brief Get the length of line segment
 */
float VLineSegment::length() const
{
  return length_;
}

/**
 * @brief Set parameters for the polygon
 * @param params        parameters map <key, val>
 */
void VLineSegment::setPolygonParams(std::unordered_map<std::string, float>& params)
{
  auto find = [&](std::string key) -> bool { return params.find(key) != params.end(); };
  start_.x = find("LINESEGMENT_START_X") ? params["LINESEGMENT_START_X"] : start_.x;
  start_.y = find("LINESEGMENT_START_Y") ? params["LINESEGMENT_START_Y"] : start_.y;
  end_.x = find("LINESEGMENT_END_X") ? params["LINESEGMENT_END_X"] : end_.x;
  end_.y = find("LINESEGMENT_END_Y") ? params["LINESEGMENT_END_Y"] : end_.y;

  end_.z = 0;
  start_.z = 0;
  center_ = (start_ + end_) / 2.0;
  const float dx = end_.x - start_.x;
  const float dy = end_.y - start_.y;
  length_ = std::hypot(dx, dy);
}

/**
 * @brief Trigger polygon rendering
 */
void VLineSegment::render()
{
  if (length_ < kMathEpsilon)
    return;

  Ogre::ManualObject* points = dynamic_cast<Ogre::ManualObject*>(point_node_->getAttachedObject(point_idx_));
  Ogre::ManualObject* lines = dynamic_cast<Ogre::ManualObject*>(line_node_->getAttachedObject(line_idx_));
  points_.clear();
  points_.emplace_back(start_);
  points_.emplace_back(end_);
  drawPoints(points, points_);
  drawLines(lines, points_);
}

/**
 * @brief Collision detection
 * @return flag   collision occurs (true) or not (false)
 */
bool VLineSegment::isCollisionWith(const std::shared_ptr<VPolygonNode>& other)
{
  if (other->empty() || !other->valid())
    return false;

  if (other->mode() == POLYGON_MODE_GENERAL)
  {
    return false;
  }
  else if (other->mode() == POLYGON_MODE_RECTANGLE)
  {
    const auto other_rect = std::dynamic_pointer_cast<VRectangle>(other);
    const float cos_angle = other_rect->cos_angle();
    const float sin_angle = other_rect->sin_angle();
    const float cx = other_rect->center().x, cy = other_rect->center().y;
    const float half_l = 0.5f * std::fabs(other_rect->length()), half_w = 0.5f * std::fabs(other_rect->width());

    auto isPointInRect = [&](const Ogre::Vector3& pt) -> bool {
      const float pt2rect_x = pt.x - cx;
      const float pt2rect_y = pt.y - cy;
      const float rx = pt2rect_x * cos_angle + pt2rect_y * sin_angle + cx;
      const float ry = -pt2rect_x * sin_angle + pt2rect_y * cos_angle + cy;
      return (rx <= cx + half_l && rx >= cx - half_l && ry <= cy + half_w && ry >= cy - half_w);
    };
    if (isPointInRect(start_) || isPointInRect(end_))
      return true;

    const float theta = -2 * std::atan(half_w / half_l);
    const auto lt_pt = other_rect->left_top_point();
    const auto rb_pt = 2 * other_rect->center() - lt_pt;
    const auto lt2center = lt_pt - other_rect->center();
    const Ogre::Vector3 lb_pt(lt2center.x * std::cos(theta) - lt2center.y * std::sin(theta) + cx,
                              lt2center.x * std::sin(theta) + lt2center.y * std::cos(theta) + cy, 0.0f);
    const auto rt_pt = 2 * other_rect->center() - lb_pt;

    if (isLineIntersaction(lt_pt, rb_pt) || isLineIntersaction(rt_pt, lb_pt))
      return true;

    return false;
  }
  else if ((other->mode() == POLYGON_MODE_CIRCLE) || other->mode() == POLYGON_MODE_DISCSMODEL)
  {
    auto isCollisionWithCircle = [&](const Ogre::Vector3& center, const float radius) -> bool {
      if (std::hypot(center.x - start_.x, center.y - start_.y) < radius)
        return true;
      if (std::hypot(center.x - end_.x, center.y - end_.y) < radius)
        return true;
      const float a = innerProduct(end_ - start_, end_ - start_);
      const float b = 2 * innerProduct(end_ - start_, start_ - center);
      const float c = innerProduct(start_ - center, start_ - center) - radius * radius;
      const float delta = b * b - 4 * a * c;
      if (delta < kMathEpsilon)
        return false;
      const float t1 = (-b + std::sqrt(delta)) / (2 * a);
      const float t2 = (-b - std::sqrt(delta)) / (2 * a);
      if (isWithin(t1, 0.0f, 1.0f) || isWithin(t2, 0.0f, 1.0f))
        return true;
      return false;
    };

    if (other->mode() == POLYGON_MODE_CIRCLE)
    {
      auto other_circle = std::dynamic_pointer_cast<VCircle>(other);
      const auto& other_circle_center = other_circle->center();
      const auto& other_circle_radius = other_circle->radius();
      return isCollisionWithCircle(other_circle_center, other_circle_radius);
    }
    else
    {
      auto other_disc_model = std::dynamic_pointer_cast<VDiscsModel>(other);
      for (const auto& disc : other_disc_model->discs())
      {
        if (isCollisionWithCircle(disc.first, disc.second))
        {
          return true;
        }
      }
      return false;
    }
  }
  else if (other->mode() == POLYGON_MODE_LINESEGEMENT)
  {
    auto other_line = std::dynamic_pointer_cast<VLineSegment>(other);
    return isLineIntersaction(other_line);
  }
  else
  {
    return false;
  }
}

/**
 * @brief Line segment intersaction detection
 * @param other   other line segment
 * @return flag   collision occurs (true) or not (false)
 */
bool VLineSegment::isLineIntersaction(const std::shared_ptr<VLineSegment>& other) const
{
  return isLineIntersaction(other->start(), other->end());
}

bool VLineSegment::isLineIntersaction(const Ogre::Vector3& start, const Ogre::Vector3& end) const
{
  if (length_ <= kMathEpsilon || std::hypot(start.x - end.x, start.y - end.y) <= kMathEpsilon)
    return false;
  if (_endPointTest(start, end))
    return true;
  if (!_rapidTest(start, end))
    return false;
  if (_straddleTest(start, end))
    return true;
  return false;
}

/**
 * @brief End points test for line segment intersaction
 * @param start   the start point of other line segment instance
 * @param end     the end point of other line segment instance
 * @return flag   start/end is equal to other's start/end (true) or not (false)
 */
bool VLineSegment::_endPointTest(const Ogre::Vector3& start, const Ogre::Vector3& end) const
{
  return (_isPointIn(start) || _isPointIn(end)) ? true : false;
}

/**
 * @brief Rapid test for line segment intersaction
 * @param start   the start point of other line segment instance
 * @param end     the end point of other line segment instance
 * @return flag   intersaction may occurs (true) or not (false)
 */
bool VLineSegment::_rapidTest(const Ogre::Vector3& start, const Ogre::Vector3& end) const
{
  return std::min(start_.x, end_.x) < std::max(start.x, end.x) &&
         std::min(start.x, end.x) < std::max(start_.x, end_.x) &&
         std::min(start_.y, end_.y) < std::max(start.y, end.y) && std::min(start.y, end.y) < std::max(start_.y, end_.y);
}

/**
 * @brief Straddle test for line segment intersaction
 * @param start   the start point of other line segment instance
 * @param end     the end point of other line segment instance
 * @return flag   collision occurs (true) or not (false)
 */
bool VLineSegment::_straddleTest(const Ogre::Vector3& start, const Ogre::Vector3& end) const
{
  const float cc1 = crossProduct(start_, end_, start);
  const float cc2 = crossProduct(start_, end_, end);
  if (cc1 * cc2 >= -kMathEpsilon)
  {
    return false;
  }
  const float cc3 = crossProduct(start, end, start_);
  const float cc4 = crossProduct(start, end, end_);
  if (cc3 * cc4 >= -kMathEpsilon)
  {
    return false;
  }
  return true;
}

/**
 * @brief Check if a point is within the line segment.
 * @param point The point to check if it is within the line segment.
 * @return Whether the input point is within the line segment or not.
 */
bool VLineSegment::_isPointIn(const Ogre::Vector3& point) const
{
  if (length_ <= kMathEpsilon)
  {
    return std::abs(point.x - start_.x) <= kMathEpsilon && std::abs(point.y - start_.y) <= kMathEpsilon;
  }
  const float prod = crossProduct(point, start_, end_);
  if (std::abs(prod) > kMathEpsilon)
  {
    return false;
  }
  return isWithin(point.x, start_.x, end_.x) && isWithin(point.y, start_.y, end_.y);
}
}  // namespace polygon_simulation
}  // namespace rmp