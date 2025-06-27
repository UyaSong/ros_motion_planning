/**
 * *********************************************************
 *
 * @file: rviz_circle.cpp
 * @brief: Circle object for rviz plugin
 * @author: Yang Haodong
 * @date: 2024-8-22
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include "polygon_simulation_utils/polygon_simulation_utils.h"
#include "polygon_simulation/rviz_circle.h"
#include "polygon_simulation/rviz_rectangle.h"
#include "polygon_simulation/rviz_discs_model.h"
#include "polygon_simulation/rviz_line_segment.h"

namespace rmp
{
namespace polygon_simulation
{
namespace
{
constexpr int circle_sample_ratio = static_cast<int>(2 * M_PI / 0.1f);
}

/**
 * @brief Construct a new Visual circle object
 * @param idx        polygon index
 */
VCircle::VCircle() : VPolygonNode()
{
  mode_ = POLYGON_MODE_CIRCLE;
}

VCircle::VCircle(unsigned int idx) : VPolygonNode(idx)
{
  mode_ = POLYGON_MODE_CIRCLE;
}

/**
 * @brief Destroy the Visual circle object
 */
VCircle::~VCircle()
{
}

/**
 * @brief Get the radius of circle
 */
float VCircle::radius() const
{
  return radius_;
}

/**
 * @brief Set parameters for the polygon
 * @param params        parameters map <key, val>
 */
void VCircle::setPolygonParams(std::unordered_map<std::string, float>& params)
{
  auto find = [&](std::string key) -> bool { return params.find(key) != params.end(); };
  radius_ = find("CIRCLE_RADIUS") ? params["CIRCLE_RADIUS"] : radius_;
  center_.x = find("CIRCLE_CENTER_X") ? params["CIRCLE_CENTER_X"] : center_.x;
  center_.y = find("CIRCLE_CENTER_Y") ? params["CIRCLE_CENTER_Y"] : center_.y;
}

/**
 * @brief Trigger polygon rendering
 */
void VCircle::render()
{
  if (radius_ == 0.0f)
    return;

  Ogre::ManualObject* points = dynamic_cast<Ogre::ManualObject*>(point_node_->getAttachedObject(point_idx_));
  Ogre::ManualObject* lines = dynamic_cast<Ogre::ManualObject*>(line_node_->getAttachedObject(line_idx_));
  points_.clear();
  int sample_num = circle_sample_ratio * radius_;
  for (int i = 0; i < sample_num; i++)
  {
    float sin_a = std::sin(i * 2 * M_PI / sample_num);
    float cos_a = std::cos(i * 2 * M_PI / sample_num);
    points_.emplace_back(center_.x + radius_ * cos_a, center_.y + radius_ * sin_a, center_.z);
  }
  if (size() > 2)
  {
    drawPoints(points, { center_ });
    drawLines(lines, points_);
  }
}

/**
 * @brief Collision detection
 * @return flag   collision occurs (true) or not (false)
 */
bool VCircle::isCollisionWith(const std::shared_ptr<VPolygonNode>& other)
{
  if (other->empty() || !other->valid())
    return false;

  if (other->mode() == POLYGON_MODE_GENERAL)
  {
    return false;
  }
  else if (other->mode() == POLYGON_MODE_RECTANGLE)
  {
    auto other_rect = std::dynamic_pointer_cast<VRectangle>(other);
    auto v = center_ - other_rect->center();

    // rotate ang project first quadrant
    float theta = -other_rect->angle();
    float rotate_vx = std::fabs(v.x * std::cos(theta) - v.y * std::sin(theta));
    float rotate_vy = std::fabs(v.x * std::sin(theta) + v.y * std::cos(theta));

    // right-top point of rectangle
    float h_x = std::fabs(other_rect->length()) / 2.0f;
    float h_y = std::fabs(other_rect->width()) / 2.0f;

    // closest vector
    float u_x = std::max(0.0f, rotate_vx - h_x);
    float u_y = std::max(0.0f, rotate_vy - h_y);
    if (std::hypot(u_x, u_y) < radius_)
      return true;
    return false;
  }
  else if (other->mode() == POLYGON_MODE_CIRCLE)
  {
    auto other_circle = std::dynamic_pointer_cast<VCircle>(other);
    const auto& other_circle_center = other_circle->center();
    const auto& other_circle_radius = other_circle->radius();
    if (std::hypot(other_circle_center.x - center_.x, other_circle_center.y - center_.y) <=
        radius_ + other_circle_radius)
      return true;
    else
      return false;
  }
  else if (other->mode() == POLYGON_MODE_DISCSMODEL)
  {
    auto other_disc_model = std::dynamic_pointer_cast<VDiscsModel>(other);

    for (const auto& disc : other_disc_model->discs())
    {
      if (std::hypot(disc.first.x - center_.x, disc.first.y - center_.y) <= radius_ + disc.second)
        return true;
    }
    return false;
  }
  else if (other->mode() == POLYGON_MODE_LINESEGEMENT)
  {
    auto other_line = std::dynamic_pointer_cast<VLineSegment>(other);
    const auto start = other_line->start();
    const auto end = other_line->end();
    if (std::hypot(center_.x - start.x, center_.y - start.y) < radius_)
      return true;
    if (std::hypot(center_.x - end.x, center_.y - end.y) < radius_)
      return true;
    const float a = innerProduct(end - start, end - start);
    const float b = 2 * innerProduct(end - start, start - center_);
    const float c = innerProduct(start - center_, start - center_) - radius_ * radius_;
    const float delta = b * b - 4 * a * c;
    if (delta < kMathEpsilon)
      return false;
    const float t1 = (-b + std::sqrt(delta)) / (2 * a);
    const float t2 = (-b - std::sqrt(delta)) / (2 * a);
    if (isWithin(t1, 0.0f, 1.0f) || isWithin(t2, 0.0f, 1.0f))
      return true;
    return false;
  }
  else
  {
    return false;
  }
}

}  // namespace polygon_simulation
}  // namespace rmp