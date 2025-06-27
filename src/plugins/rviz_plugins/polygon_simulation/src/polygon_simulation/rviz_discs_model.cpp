/**
 * *********************************************************
 *
 * @file: rviz_discs_model.cpp
 * @brief: Discs model object for rviz plugin
 * @author: Yang Haodong
 * @date: 2024-9-8
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
#include "polygon_simulation/rviz_discs_model.h"
#include "polygon_simulation/rviz_circle.h"
#include "polygon_simulation/rviz_rectangle.h"
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
 * @brief Construct a new Visual discs model object
 * @param idx        polygon index
 */
VDiscsModel::VDiscsModel() : VRectangle()
{
  mode_ = POLYGON_MODE_DISCSMODEL;
  // debug_info_.emplace_back("l", std::make_pair<float, float>(0.0f, 0.0f));
  // debug_info_.emplace_back("w", std::make_pair<float, float>(0.0f, 0.0f));
  // debug_info_.emplace_back("left_top", std::make_pair<float, float>(0.0f, 0.0f));
}

VDiscsModel::VDiscsModel(unsigned int idx) : VRectangle(idx)
{
  mode_ = POLYGON_MODE_DISCSMODEL;
  // debug_info_.emplace_back("l", std::make_pair<float, float>(0.0f, 0.0f));
  // debug_info_.emplace_back("w", std::make_pair<float, float>(0.0f, 0.0f));
  // debug_info_.emplace_back("left_top", std::make_pair<float, float>(0.0f, 0.0f));
}

/**
 * @brief Destroy the Visual discs model object
 */
VDiscsModel::~VDiscsModel()
{
}

const std::vector<std::pair<Ogre::Vector3, float>>& VDiscsModel::discs() const
{
  return discs_;
}

/**
 * @brief Set parameters for the polygon
 * @param params        parameters map <key, val>
 */
void VDiscsModel::setPolygonParams(std::unordered_map<std::string, float>& params)
{
  auto find = [&](std::string key) -> bool { return params.find(key) != params.end(); };
  w_ = find("DISCMODEL_WIDTH") ? params["DISCMODEL_WIDTH"] : w_;
  l_ = find("DISCMODEL_LENGTH") ? params["DISCMODEL_LENGTH"] : l_;
  angle_ = find("DISCMODEL_ANGLE") ? params["DISCMODEL_ANGLE"] : angle_;
  anchor_pt_.x = find("DISCMODEL_ANCHOR_POINT_X") ? params["DISCMODEL_ANCHOR_POINT_X"] : anchor_pt_.x;
  anchor_pt_.y = find("DISCMODEL_ANCHOR_POINT_Y") ? params["DISCMODEL_ANCHOR_POINT_Y"] : anchor_pt_.y;
  cos_angle_ = std::cos(angle_);
  sin_angle_ = std::sin(angle_);
  disc_nums_ = find("DISCMODEL_DISC_NUMBER") ? params["DISCMODEL_DISC_NUMBER"] : disc_nums_;
  dynamic_mode_ = find("DISCMODEL_DYNAMIC_MODE") ? params["DISCMODEL_DYNAMIC_MODE"] : dynamic_mode_;
  safety_buffer_ = find("DISCMODEL_SAFETY_BUFFER") ? params["DISCMODEL_SAFETY_BUFFER"] : safety_buffer_;
  safety_buffer_ = std::max(safety_buffer_, 0.25f);
}

/**
 * @brief Trigger polygon rendering
 */
void VDiscsModel::render()
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

  drawPoints(points, points_);
  drawLines(lines, points_);

  discs_.clear();

  const float half_length = 0.5 * std::fabs(l_);
  const float half_width = 0.5 * std::fabs(w_);
  int disc_nums = !dynamic_mode_ ?
                      disc_nums_ :
                      std::round(half_length / (std::sqrt((half_width * 2.0f + safety_buffer_) * safety_buffer_)));

  if (disc_nums < 1)
  {
    float radius = std::hypot(half_length, half_width);
    discs_.emplace_back(center_, radius);
  }
  else
  {
    float radius = std::hypot(half_length / disc_nums, half_width);
    for (int i = 1; i <= disc_nums; ++i)
    {
      float tmp_l = (2 * i - 1) * half_length / disc_nums - half_length;
      discs_.emplace_back(center_ + Ogre::Vector3(tmp_l * cos_angle_, tmp_l * sin_angle_, 0.0), radius);
    }
  }
  for (const auto& disc : discs_)
  {
    int sample_num = circle_sample_ratio * disc.second;
    std::vector<Ogre::Vector3> disc_points;
    for (int i = 0; i < sample_num; i++)
    {
      float sin_a = std::sin(i * 2 * M_PI / sample_num);
      float cos_a = std::cos(i * 2 * M_PI / sample_num);
      disc_points.emplace_back(disc.first.x + disc.second * cos_a, disc.first.y + disc.second * sin_a, 0.0);
    }
    if (disc_points.size() > 2)
    {
      drawPoints(points, { disc.first }, true);
      drawLines(lines, disc_points, true);
    }
  }
}

/**
 * @brief Collision detection
 * @param other   other polygon instance
 * @return flag   collision occurs (true) or not (false)
 */
bool VDiscsModel::isCollisionWith(const std::shared_ptr<VPolygonNode>& other)
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

    for (const auto& disc : discs_)
    {
      auto v = disc.first - other_rect->center();

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
      if (std::hypot(u_x, u_y) < disc.second)
        return true;
    }
    return false;
  }
  else if (other->mode() == POLYGON_MODE_CIRCLE)
  {
    auto other_circle = std::dynamic_pointer_cast<VCircle>(other);
    const auto& other_circle_center = other_circle->center();
    const auto& other_circle_radius = other_circle->radius();

    for (const auto& disc : discs_)
    {
      if (std::hypot(other_circle_center.x - disc.first.x, other_circle_center.y - disc.first.y) <=
          disc.second + other_circle_radius)
        return true;
    }
    return false;
  }
  else if (other->mode() == POLYGON_MODE_DISCSMODEL)
  {
    auto other_disc_model = std::dynamic_pointer_cast<VDiscsModel>(other);
    const auto& other_discs = other_disc_model->discs();
    for (const auto& disc : discs_)
    {
      for (const auto& other_disc : other_discs)
      {
        if (std::hypot(other_disc.first.x - disc.first.x, other_disc.first.y - disc.first.y) <=
            disc.second + other_disc.second)
          return true;
      }
    }
    return false;
  }
  else if (other->mode() == POLYGON_MODE_LINESEGEMENT)
  {
    auto other_line = std::dynamic_pointer_cast<VLineSegment>(other);
    const auto start = other_line->start();
    const auto end = other_line->end();
    for (const auto& disc : discs_)
    {
      if (std::hypot(disc.first.x - start.x, disc.first.y - start.y) < disc.second)
        return true;
      if (std::hypot(disc.first.x - end.x, disc.first.y - end.y) < disc.second)
        return true;
      const float a = innerProduct(end - start, end - start);
      const float b = 2 * innerProduct(end - start, start - disc.first);
      const float c = innerProduct(start - disc.first, start - disc.first) - disc.second * disc.second;
      const float delta = b * b - 4 * a * c;
      if (delta < kMathEpsilon)
        continue;
      const float t1 = (-b + std::sqrt(delta)) / (2 * a);
      const float t2 = (-b - std::sqrt(delta)) / (2 * a);
      if (isWithin(t1, 0.0f, 1.0f) || isWithin(t2, 0.0f, 1.0f))
        return true;
    }
    return false;
  }
  else
  {
    return false;
  }
}

}  // namespace polygon_simulation
}  // namespace rmp