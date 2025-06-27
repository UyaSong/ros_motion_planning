/**
 * *********************************************************
 *
 * @file: rviz_rectangle.cpp
 * @brief: Rectangle object for rviz plugin
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
#include <ros/ros.h>

#include "polygon_simulation/rviz_circle.h"
#include "polygon_simulation/rviz_rectangle.h"
#include "polygon_simulation/rviz_discs_model.h"
#include "polygon_simulation/rviz_line_segment.h"

namespace rmp
{
namespace polygon_simulation
{
/**
 * @brief Construct a new Visual rectangle object
 * @param idx        polygon index
 */
VRectangle::VRectangle() : VPolygonNode()
{
  mode_ = POLYGON_MODE_RECTANGLE;
  debug_info_.emplace_back("l", std::make_pair<float, float>(0.0f, 0.0f));
  debug_info_.emplace_back("w", std::make_pair<float, float>(0.0f, 0.0f));
  debug_info_.emplace_back("left_top", std::make_pair<float, float>(0.0f, 0.0f));
}

VRectangle::VRectangle(unsigned int idx) : VPolygonNode(idx)
{
  mode_ = POLYGON_MODE_RECTANGLE;
  debug_info_.emplace_back("l", std::make_pair<float, float>(0.0f, 0.0f));
  debug_info_.emplace_back("w", std::make_pair<float, float>(0.0f, 0.0f));
  debug_info_.emplace_back("left_top", std::make_pair<float, float>(0.0f, 0.0f));
}

/**
 * @brief Destroy the Visual rectangle object
 */
VRectangle::~VRectangle()
{
}

/**
 * @brief Get the left-top point of rectangle
 */
const Ogre::Vector3& VRectangle::left_top_point() const
{
  return lt_pt_;
}

/**
 * @brief Get the length of rectangle
 */
float VRectangle::length() const
{
  return l_;
}

/**
 * @brief Get the width of rectangle
 */
float VRectangle::width() const
{
  return w_;
}

/**
 * @brief Get the angle of rectangle
 */
float VRectangle::angle() const
{
  return angle_;
}

/**
 * @brief Get the cos(angle) of rectangle
 */
float VRectangle::cos_angle() const
{
  return cos_angle_;
}

/**
 * @brief Get the sin(angle) of rectangle
 */
float VRectangle::sin_angle() const
{
  return sin_angle_;
}

/**
 * @brief Set parameters for the polygon
 * @param params        parameters map <key, val>
 */
void VRectangle::setPolygonParams(std::unordered_map<std::string, float>& params)
{
  auto find = [&](std::string key) -> bool { return params.find(key) != params.end(); };
  w_ = find("RECTANGLE_WIDTH") ? params["RECTANGLE_WIDTH"] : w_;
  l_ = find("RECTANGLE_LENGTH") ? params["RECTANGLE_LENGTH"] : l_;
  angle_ = find("RECTANGLE_ANGLE") ? params["RECTANGLE_ANGLE"] : angle_;
  anchor_pt_.x = find("RECTANGLE_ANCHOR_POINT_X") ? params["RECTANGLE_ANCHOR_POINT_X"] : anchor_pt_.x;
  anchor_pt_.y = find("RECTANGLE_ANCHOR_POINT_Y") ? params["RECTANGLE_ANCHOR_POINT_Y"] : anchor_pt_.y;

  cos_angle_ = std::cos(angle_);
  sin_angle_ = std::sin(angle_);
}

/**
 * @brief Trigger polygon rendering
 */
void VRectangle::render()
{
  if (l_ == 0.0f || w_ == 0.0f)
    return;

  Ogre::ManualObject* points = dynamic_cast<Ogre::ManualObject*>(point_node_->getAttachedObject(point_idx_));
  Ogre::ManualObject* lines = dynamic_cast<Ogre::ManualObject*>(line_node_->getAttachedObject(line_idx_));
  points_.clear();
  points_.emplace_back(anchor_pt_.x, anchor_pt_.y, 0.0f);
  points_.emplace_back(anchor_pt_.x - sin_angle_ * w_, anchor_pt_.y + cos_angle_ * w_, 0.0f);
  points_.emplace_back(anchor_pt_.x + cos_angle_ * l_ - sin_angle_ * w_,
                       anchor_pt_.y + sin_angle_ * l_ + cos_angle_ * w_, 0.0f);
  points_.emplace_back(anchor_pt_.x + cos_angle_ * l_, anchor_pt_.y + sin_angle_ * l_, 0.0f);
  center_.x = anchor_pt_.x + 0.5f * cos_angle_ * l_ - 0.5f * sin_angle_ * w_;
  center_.y = anchor_pt_.y + 0.5f * cos_angle_ * w_ + 0.5f * sin_angle_ * l_;

  float lt_pt_x = 0.5f * l_ - 0.5f * std::fabs(l_);
  float lt_pt_y = 0.5f * w_ - 0.5f * std::fabs(w_);
  lt_pt_.x = anchor_pt_.x + cos_angle_ * lt_pt_x - sin_angle_ * lt_pt_y;
  lt_pt_.y = anchor_pt_.y + sin_angle_ * lt_pt_x + cos_angle_ * lt_pt_y;

  // debug
  float debug_l_x = 0.5f * l_;
  float debug_l_y = 0.5f * w_ - 0.5f * std::fabs(w_);
  debug_info_[0].first = "l " + numberToString(std::fabs(l_), 2);
  debug_info_[0].second.first = anchor_pt_.x + cos_angle_ * debug_l_x - sin_angle_ * debug_l_y;
  debug_info_[0].second.second = anchor_pt_.y + sin_angle_ * debug_l_x + cos_angle_ * debug_l_y;

  float debug_w_x = 0.5f * l_ - 0.5f * std::fabs(l_);
  float debug_w_y = 0.5f * w_;
  debug_info_[1].first = "w " + numberToString(std::fabs(w_), 2);
  debug_info_[1].second.first = anchor_pt_.x + cos_angle_ * debug_w_x - sin_angle_ * debug_w_y;
  debug_info_[1].second.second = anchor_pt_.y + sin_angle_ * debug_w_x + cos_angle_ * debug_w_y;

  debug_info_[2].first = "(" + numberToString(lt_pt_.x, 2) + ", " + numberToString(lt_pt_.y, 2) + ")";
  debug_info_[2].second.first = lt_pt_.x;
  debug_info_[2].second.second = lt_pt_.y;

  drawPoints(points, points_);
  drawPoints(points, { center_ }, true);
  drawLines(lines, points_);
}

/**
 * @brief Collision detection
 * @param other   other polygon instance
 * @return flag   collision occurs (true) or not (false)
 */
bool VRectangle::isCollisionWith(const std::shared_ptr<VPolygonNode>& other)
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
    if (std::fabs(angle_) <= 1e-6 && std::fabs(other_rect->angle()) <= 1e-6)
    {
      return _AABBCollisionDetection(other_rect);
    }
    else
    {
      return _OBBCollisionDetection(other_rect);
    }
  }
  else if (other->mode() == POLYGON_MODE_CIRCLE)
  {
    auto other_circle = std::dynamic_pointer_cast<VCircle>(other);
    auto v = other_circle->center() - center_;

    // rotate ang project first quadrant
    float theta = -angle_;
    float rotate_vx = std::fabs(v.x * std::cos(theta) - v.y * std::sin(theta));
    float rotate_vy = std::fabs(v.x * std::sin(theta) + v.y * std::cos(theta));

    // right-top point of rectangle
    float h_x = std::fabs(l_) / 2.0f;
    float h_y = std::fabs(w_) / 2.0f;

    // closest vector
    float u_x = std::max(0.0f, rotate_vx - h_x);
    float u_y = std::max(0.0f, rotate_vy - h_y);
    if (std::hypot(u_x, u_y) < other_circle->radius())
      return true;
    return false;
  }
  else if (other->mode() == POLYGON_MODE_DISCSMODEL)
  {
    auto other_disc_model = std::dynamic_pointer_cast<VDiscsModel>(other);

    for (const auto& disc : other_disc_model->discs())
    {
      auto v = disc.first - center_;

      // rotate ang project first quadrant
      float theta = -angle_;
      float rotate_vx = std::fabs(v.x * std::cos(theta) - v.y * std::sin(theta));
      float rotate_vy = std::fabs(v.x * std::sin(theta) + v.y * std::cos(theta));

      // right-top point of rectangle
      float h_x = std::fabs(l_) / 2.0f;
      float h_y = std::fabs(w_) / 2.0f;

      // closest vector
      float u_x = std::max(0.0f, rotate_vx - h_x);
      float u_y = std::max(0.0f, rotate_vy - h_y);
      if (std::hypot(u_x, u_y) < disc.second)
        return true;
    }
    return false;
  }
  else if (other->mode() == POLYGON_MODE_LINESEGEMENT)
  {
    const auto other_line = std::dynamic_pointer_cast<VLineSegment>(other);
    const float cx = center_.x, cy = center_.y;
    const float half_l = 0.5f * std::fabs(length()), half_w = 0.5f * std::fabs(width());

    auto isPointInRect = [&](const Ogre::Vector3& pt) -> bool {
      const float pt2rect_x = pt.x - cx;
      const float pt2rect_y = pt.y - cy;
      const float theta = -angle();
      const float rx = pt2rect_x * std::cos(theta) - pt2rect_y * std::sin(theta) + cx;
      const float ry = pt2rect_x * std::sin(theta) + pt2rect_y * std::cos(theta) + cy;
      return (rx <= cx + half_l && rx >= cx - half_l && ry <= cy + half_w && ry >= cy - half_w);
    };
    if (isPointInRect(other_line->start()) || isPointInRect(other_line->end()))
      return true;

    const float theta = -2 * std::atan(half_w / half_l);
    const auto rb_pt = 2 * center_ - lt_pt_;
    const auto lt2center = lt_pt_ - center_;
    const Ogre::Vector3 lb_pt(lt2center.x * std::cos(theta) - lt2center.y * std::sin(theta) + cx,
                              lt2center.x * std::sin(theta) + lt2center.y * std::cos(theta) + cy, 0.0f);
    const auto rt_pt = 2 * center_ - lb_pt;

    if (other_line->isLineIntersaction(lt_pt_, rb_pt) || other_line->isLineIntersaction(rt_pt, lb_pt))
      return true;

    return false;
  }
  else
  {
    return false;
  }
}

/**
 * @brief Axis-aligned Bounding Box Collision detection
 * @param other   other rectangle instance
 * @return flag   collision occurs (true) or not (false)
 */
bool VRectangle::_AABBCollisionDetection(const std::shared_ptr<VRectangle>& other)
{
  return lt_pt_.x < other->left_top_point().x + std::fabs(other->length()) &&
         other->left_top_point().x < lt_pt_.x + std::fabs(l_) &&
         lt_pt_.y < other->left_top_point().y + std::fabs(other->width()) &&
         other->left_top_point().y < lt_pt_.y + std::fabs(w_);
}

/**
 * @brief Oriented Bounding Box Collision detection
 * @param other   other rectangle instance
 * @return flag   collision occurs (true) or not (false)
 */
bool VRectangle::_OBBCollisionDetection(const std::shared_ptr<VRectangle>& other)
{
  const float shift_x = other->center().x - center_.x;
  const float shift_y = other->center().y - center_.y;

  const float half_length = 0.5f * std::fabs(l_);
  const float half_width = 0.5f * std::fabs(w_);
  const float other_half_length = 0.5f * std::fabs(other->length());
  const float other_half_width = 0.5f * std::fabs(other->width());
  const float dx1 = cos_angle_ * half_length;
  const float dy1 = sin_angle_ * half_length;
  const float dx2 = sin_angle_ * half_width;
  const float dy2 = -cos_angle_ * half_width;
  const float dx3 = other->cos_angle() * other_half_length;
  const float dy3 = other->sin_angle() * other_half_length;
  const float dx4 = other->sin_angle() * other_half_width;
  const float dy4 = -other->cos_angle() * other_half_width;

  return std::abs(shift_x * cos_angle_ + shift_y * sin_angle_) <= std::abs(dx3 * cos_angle_ + dy3 * sin_angle_) +
                                                                      std::abs(dx4 * cos_angle_ + dy4 * sin_angle_) +
                                                                      half_length &&
         std::abs(shift_x * sin_angle_ - shift_y * cos_angle_) <= std::abs(dx3 * sin_angle_ - dy3 * cos_angle_) +
                                                                      std::abs(dx4 * sin_angle_ - dy4 * cos_angle_) +
                                                                      half_width &&
         std::abs(shift_x * other->cos_angle() + shift_y * other->sin_angle()) <=
             std::abs(dx1 * other->cos_angle() + dy1 * other->sin_angle()) +
                 std::abs(dx2 * other->cos_angle() + dy2 * other->sin_angle()) + other_half_length &&
         std::abs(shift_x * other->sin_angle() - shift_y * other->cos_angle()) <=
             std::abs(dx1 * other->sin_angle() - dy1 * other->cos_angle()) +
                 std::abs(dx2 * other->sin_angle() - dy2 * other->cos_angle()) + other_half_width;
}
}  // namespace polygon_simulation
}  // namespace rmp