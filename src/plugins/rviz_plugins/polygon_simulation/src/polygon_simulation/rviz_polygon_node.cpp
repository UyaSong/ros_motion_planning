/**
 * *********************************************************
 *
 * @file: rviz_polygon_node.cpp
 * @brief: Polygon node object for rviz plugin
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
#include <ros/ros.h>
#include <rviz/ogre_helpers/movable_text.h>

#include "polygon_simulation/rviz_polygon_node.h"
#include "polygon_simulation_utils/polygon_simulation_utils.h"

namespace rmp
{
namespace polygon_simulation
{
/**
 * @brief Construct a new Visual polygon object
 * @param idx        polygon index
 */
VPolygonNode::VPolygonNode()
{
  idx_ = 0U;
  mode_ = POLYGON_MODE_NONE;
  point_idx_ = "points_0";
  line_idx_ = "lines_0";
  point_material_ = Ogre::MaterialManager::getSingleton().create(
      "points_0_material", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  line_material_ = Ogre::MaterialManager::getSingleton().create(
      "lines_0_material", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
}

VPolygonNode::VPolygonNode(unsigned int idx)
{
  idx_ = idx;
  mode_ = POLYGON_MODE_NONE;
  point_idx_ = "points_" + std::to_string(idx);
  line_idx_ = "lines_" + std::to_string(idx);
  point_material_ = Ogre::MaterialManager::getSingleton().create(
      point_idx_ + "_material", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  line_material_ = Ogre::MaterialManager::getSingleton().create(
      line_idx_ + "_material", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
}

/**
 * @brief Destroy the Visual polygon object
 */
VPolygonNode::~VPolygonNode()
{
  destory();
}

/**
 * @brief Get polygon index
 */
unsigned int VPolygonNode::id() const
{
  return idx_;
}

/**
 * @brief Get polygon points size
 */
int VPolygonNode::size() const
{
  return static_cast<int>(points_.size());
}

/**
 * @brief Judge whether there are points inside object
 */
bool VPolygonNode::empty() const
{
  return size() <= 0;
}

/**
 * @brief Judge whether the polygon is valid. Polygons that have not been created are invalid
 */
bool VPolygonNode::valid() const
{
  return valid_;
}

/**
 * @brief Judge whether the polygon collides with others
 */
bool VPolygonNode::collide() const
{
  return collid_;
}

/**
 * @brief Get polygon mode (polygon, rectangle or circle)
 */
PolygonMode VPolygonNode::mode() const
{
  return mode_;
}

/**
 * @brief Get center point of polygon
 */
const Ogre::Vector3& VPolygonNode::center() const
{
  return center_;
}

/**
 * @brief Get convex property of polygon
 */
bool VPolygonNode::is_convex() const
{
  return is_convex_;
}

/**
 * @brief Get polygons of polygon
 */
const std::vector<Ogre::Vector3>& VPolygonNode::points() const
{
  return points_;
}

bool VPolygonNode::operator==(VPolygonNode& other) const
{
  return idx_ == other.id();
}

bool VPolygonNode::operator!=(VPolygonNode& other) const
{
  return idx_ != other.id();
}

/**
 * @brief Add point into polygon
 * @param point      the point to add
 */
void VPolygonNode::add(Ogre::Vector3 point)
{
  if (empty())
  {
    center_ = point;
  }
  else
  {
    center_.x = (size() * center_.x + point.x) / (size() + 1);
    center_.y = (size() * center_.y + point.y) / (size() + 1);
  }
  points_.push_back(point);

  area_ = 0.0;
  bool is_ccw = true;
  if (size() > 2)
  {
    for (int i = 1; i < size(); ++i)
    {
      area_ += crossProduct(points_[0], points_[i - 1], points_[i]);
    }
  }
  if (area_ < 0.0)
  {
    area_ = -area_;
    is_ccw = false;
  }
  area_ /= 2.0;

  is_convex_ = true;
  // Make sure the points are in ccw order
  if (is_ccw)
  {
    for (int i = 0; i < size(); ++i)
    {
      if (crossProduct(points_[prev(i)], points_[i], points_[next(i)]) <= -kMathEpsilon)
      {
        is_convex_ = false;
        break;
      }
    }
  }
  else
  {
    for (int i = size() - 1; i > 0; --i)
    {
      if (crossProduct(points_[next(i)], points_[i], points_[prev(i)]) <= -kMathEpsilon)
      {
        is_convex_ = false;
        break;
      }
    }
  }
}

/**
 * @brief Get the next index of given index.
 * @param at the given index
 * @return next index
 */
int VPolygonNode::next(int at) const
{
  return at >= size() - 1 ? 0 : at + 1;
}

/**
 * @brief Get the previous index of given index.
 * @param at the given index
 * @return previous index
 */
int VPolygonNode::prev(int at) const
{
  return at == 0 ? size() - 1 : at - 1;
}

/**
 * @brief Clear the created points/lines/text
 */
void VPolygonNode::clear()
{
  if (empty())
    return;

  // Clear points
  points_.clear();

  // Clear the points visualization
  auto* points = dynamic_cast<Ogre::ManualObject*>(point_node_->getAttachedObject(point_idx_));
  if (points)
    points->clear();

  // Clear the lines visualization
  auto* lines = dynamic_cast<Ogre::ManualObject*>(line_node_->getAttachedObject(line_idx_));
  if (lines)
    lines->clear();

  // Make the text object invisible
  auto* text = dynamic_cast<rviz::MovableText*>(text_node_->getAttachedObject(0));
  if (text)
    text->setVisible(false);

  for (auto& debug_text_node : debug_text_nodes_)
  {
    auto* text = dynamic_cast<rviz::MovableText*>(debug_text_node->getAttachedObject(0));
    if (text)
      text->setVisible(false);
  }
}

/**
 * @brief Clear the created points/lines/text and destory the scene nodes
 */
void VPolygonNode::destory()
{
  clear();

  scene_manager_->destroyManualObject(point_idx_);
  scene_manager_->destroyManualObject(line_idx_);
  scene_manager_->destroySceneNode(point_node_);
  scene_manager_->destroySceneNode(line_node_);
  scene_manager_->destroySceneNode(text_node_);
  for (auto& debug_text_node : debug_text_nodes_)
    scene_manager_->destroySceneNode(debug_text_node);
  Ogre::MaterialManager::getSingleton().remove(point_material_->getName());
  Ogre::MaterialManager::getSingleton().remove(line_material_->getName());
}

/**
 * @brief Set the polygon valid
 */
void VPolygonNode::done()
{
  valid_ = true;
}

/**
 * @brief Set the collision flag for the polygon
 * @param collision_flag      collision flag
 */
void VPolygonNode::setCollision(bool collision_flag)
{
  collid_ = collision_flag;
}

/**
 * @brief Set the parent scene manager meanwhile create point/line/text scene node
 * @param scene_manager      parent scene manager
 */
void VPolygonNode::setSceneManager(Ogre::SceneManager* scene_manager)
{
  scene_manager_ = scene_manager;
  point_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode(point_idx_);
  Ogre::ManualObject* points = scene_manager_->createManualObject(point_idx_);
  point_node_->attachObject(points);

  line_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode(line_idx_);
  Ogre::ManualObject* lines = scene_manager_->createManualObject(line_idx_);
  line_node_->attachObject(lines);

  std::string caption = "#" + std::to_string(idx_);
  text_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode(caption);
  auto* text = new rviz::MovableText(caption, "Liberation Sans", 0.5, Ogre::ColourValue::Blue);
  text->setVisible(false);
  text->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_ABOVE);
  text_node_->attachObject(text);

  int i = 0;
  for (const auto& kv : debug_info_)
  {
    auto debug_text_node =
        scene_manager_->getRootSceneNode()->createChildSceneNode(std::to_string(idx_) + "_debug_" + std::to_string(i));
    auto* text = new rviz::MovableText(kv.first, "Liberation Sans", 0.5, Ogre::ColourValue::Blue);
    text->setVisible(false);
    text->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_ABOVE);
    debug_text_node->attachObject(text);
    debug_text_nodes_.push_back(std::move(debug_text_node));
    i++;
  }
}

/**
 * @brief Set the parent scene manager meanwhile create point/line/text scene node
 * @param text_size      text size
 * @param is_visible     whether the text is visible
 * @param show_debug     whether show the debug information
 */
void VPolygonNode::updateText(float text_size, bool is_visible, bool show_debug)
{
  if (empty())
    return;

  auto* text = dynamic_cast<rviz::MovableText*>(text_node_->getAttachedObject(0));
  if (text == nullptr)
    return;

  if (!is_visible)
  {
    text->setVisible(false);
  }
  else
  {
    // Set the position of the text node as the center of the points
    text->setRenderingMinPixelSize(5);
    text->setCharacterHeight(text_size);
    text->setVisible(true);
    text_node_->setPosition(center_);
  }

  // set debug information
  int i = 0;
  for (const auto& info : debug_info_)
  {
    auto debug_text_node = debug_text_nodes_[i];
    auto* text = dynamic_cast<rviz::MovableText*>(debug_text_node->getAttachedObject(0));
    if (text == nullptr)
      continue;
    if (!show_debug)
    {
      text->setVisible(false);
    }
    else
    {
      text->setCaption(info.first);
      text->setRenderingMinPixelSize(5);
      text->setCharacterHeight(text_size);
      text->setVisible(true);
      debug_text_node->setPosition(info.second.first, info.second.second, 0.0f);
    }
    i++;
  }
}

/**
 * @brief Set visual properties for the line
 * @param line_color     line color
 */
void VPolygonNode::setPointVisualProperty(QColor point_color, float point_size)
{
  point_material_->setPointSize(point_size);
  updateMaterialColor(point_material_, point_color);
}

/**
 * @brief Set visual properties for the points
 * @param point_color     pointpoint color
 * @param point_size      point size
 */
void VPolygonNode::setLineVisualProperty(QColor line_color)
{
  updateMaterialColor(line_material_, line_color);
}

/**
 * @brief draw lines in the figure
 * @param lines      lines object
 * @param draw_points    points to draw
 * @param draw_continue continue drawing (true) or not (false)
 */
void VPolygonNode::drawLines(Ogre::ManualObject* lines, const std::vector<Ogre::Vector3>& draw_points,
                             bool draw_continue)
{
  if (!draw_continue)
    lines->clear();

  lines->begin(line_material_->getName(), Ogre::RenderOperation::OT_LINE_STRIP);
  for (std::size_t i = 0; i < draw_points.size() - 1; ++i)
  {
    lines->position(draw_points[i]);
    lines->position(draw_points[i + 1]);
  }

  // Close the polygon
  if (draw_points.size() > 2)
  {
    lines->position(draw_points.back());
    lines->position(draw_points.front());
  }

  lines->end();
}

/**
 * @brief draw points in the figure
 * @param points        points object
 * @param draw_points    points to draw
 * @param draw_continue continue drawing (true) or not (false)
 */
void VPolygonNode::drawPoints(Ogre::ManualObject* points, const std::vector<Ogre::Vector3>& draw_points,
                              bool draw_continue)
{
  if (!draw_continue)
    points->clear();

  points->begin(point_material_->getName(), Ogre::RenderOperation::OT_POINT_LIST);
  for (std::size_t i = 0; i < draw_points.size(); ++i)
  {
    points->position(draw_points[i]);
  }

  points->end();
}

}  // namespace polygon_simulation
}  // namespace rmp