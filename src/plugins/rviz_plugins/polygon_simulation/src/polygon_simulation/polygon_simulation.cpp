/**
 * *********************************************************
 *
 * @file: polygon_simulation.cpp
 * @brief: Polygon selection (rviz plugin)
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
#include <QKeyEvent>
#include <rviz/load_resource.h>
#include <rviz/display_context.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/ogre_helpers/movable_text.h>
#include <rviz/selection/selection_manager.h>
#include <pluginlib/class_list_macros.h>

#include "polygon_simulation/rviz_circle.h"
#include "polygon_simulation/rviz_polygon.h"
#include "polygon_simulation/rviz_rectangle.h"
#include "polygon_simulation/rviz_discs_model.h"
#include "polygon_simulation/rviz_line_segment.h"
#include "polygon_simulation/polygon_simulation.h"
#include "polygon_simulation/PolygonStamped.h"
#include "polygon_simulation_utils/polygon_simulation_utils.h"

PLUGINLIB_EXPORT_CLASS(rmp::polygon_simulation::PolygonSimulation, rviz::Tool)

namespace rmp
{
namespace polygon_simulation
{
PolygonSimulation::PolygonSimulation() : rviz::Tool()
{
  shortcut_key_ = 'p';
  setIcon(rviz::loadPixmap("package://polygon_simulation/icons/classes/PolygonSimulation.svg"));
}

PolygonSimulation::~PolygonSimulation()
{
  polygons_.clear();
}

void PolygonSimulation::onInitialize()
{
  server_ = nh_.advertiseService("get_selection", &PolygonSimulation::_callback, this);

  // general properties
  polygon_mode_prop_ = new rviz::EnumProperty("Polygon mode", "Polygon", "How to create the polygon",
                                              getPropertyContainer(), SLOT(updatePanel()), this);
  polygon_mode_prop_->addOption("Polygon", POLYGON_MODE_GENERAL);
  polygon_mode_prop_->addOption("Rectangle", POLYGON_MODE_RECTANGLE);
  polygon_mode_prop_->addOption("Circle", POLYGON_MODE_CIRCLE);
  polygon_mode_prop_->addOption("Discs Model", POLYGON_MODE_DISCSMODEL);
  polygon_mode_prop_->addOption("Line Segment", POLYGON_MODE_LINESEGEMENT);

  collision_detection_prop_ =
      new rviz::BoolProperty("Collision detection", false, "Collision detection for all polygons",
                             getPropertyContainer(), SLOT(onCollisionDetection()), this);

  // polygon property
  lasso_mode_prop_ = new rviz::BoolProperty("Lasso mode", false, "Toggle between lasso and discrete click mode",
                                            getPropertyContainer(), SLOT(updateView()), this);
  polygon_convex_prop_ = new rviz::StringProperty("Convex", "False", "Whether the polygon is convex or not",
                                                  getPropertyContainer(), SLOT(updateView()), this);

  // rectangle property
  rectangle_pt_x_prop_ = new rviz::FloatProperty("Rectangle X", 0, "X-coordinate of rectangle", getPropertyContainer(),
                                                 SLOT(updateView()), this);
  rectangle_pt_y_prop_ = new rviz::FloatProperty("Rectangle Y", 0, "Y-coordinate of rectangle", getPropertyContainer(),
                                                 SLOT(updateView()), this);
  rectangle_l_prop_ = new rviz::FloatProperty("Rectangle length", 0, "Length of rectangle", getPropertyContainer(),
                                              SLOT(updateView()), this);
  rectangle_w_prop_ = new rviz::FloatProperty("Rectangle width", 0, "Width of rectangle", getPropertyContainer(),
                                              SLOT(updateView()), this);
  rectangle_angle_prop_ = new rviz::FloatProperty("Rectangle angle (rad)", 0, "Angle of rectangle",
                                                  getPropertyContainer(), SLOT(updateView()), this);
  // circle property
  circle_pt_x_prop_ = new rviz::FloatProperty("Circle X", 0, "X-coordinate of circle", getPropertyContainer(),
                                              SLOT(updateView()), this);
  circle_pt_y_prop_ = new rviz::FloatProperty("Circle Y", 0, "Y-coordinate of circle", getPropertyContainer(),
                                              SLOT(updateView()), this);
  circle_r_prop_ =
      new rviz::FloatProperty("Circle radius", 0, "Radius of circle", getPropertyContainer(), SLOT(updateView()), this);

  // disc model property
  dynamic_discs_num_prop_ = new rviz::BoolProperty("Dynamic discs number", false, "Dynamic discs number",
                                                   getPropertyContainer(), SLOT(updatePanel()), this);
  safety_buffer_prop_ = new rviz::FloatProperty("Safety buffer", 0.0, "Safety buffer for disc model",
                                                getPropertyContainer(), SLOT(updateView()), this);
  fixed_discs_num_prop_ =
      new rviz::IntProperty("Discs number", 3, "Discs number", getPropertyContainer(), SLOT(updateView()), this);

  // line segment
  line_segment_s_x_prop_ = new rviz::FloatProperty("Line Start X", 0, "Start x-coordinate of line segment",
                                                   getPropertyContainer(), SLOT(updateView()), this);
  line_segment_s_y_prop_ = new rviz::FloatProperty("Line Start Y", 0, "Start x-coordinate of line segment",
                                                   getPropertyContainer(), SLOT(updateView()), this);
  line_segment_e_x_prop_ = new rviz::FloatProperty("Line End X", 0, "End y-coordinate of line segment",
                                                   getPropertyContainer(), SLOT(updateView()), this);
  line_segment_e_y_prop_ = new rviz::FloatProperty("Line End Y", 0, "End y-coordinate of line segment",
                                                   getPropertyContainer(), SLOT(updateView()), this);

  // visual properties
  visual_group_prop_ = new rviz::StringProperty("Visualization", "", "Visualization properties for lines and points",
                                                getPropertyContainer());
  same_visual_prop_ = new rviz::BoolProperty("Same Property", false, "Toggles the same setting for all polygons",
                                             visual_group_prop_, SLOT(updateViewProperty()), this);
  point_color_prop_ = new rviz::ColorProperty("Point Color", DEFAULT_POINT_QT_COLOR, "Color of the points",
                                              visual_group_prop_, SLOT(updateViewProperty()), this);
  line_color_prop_ = new rviz::ColorProperty("Line Color", DEFAULT_LINE_QT_COLOR, "Color of the line",
                                             visual_group_prop_, SLOT(updateViewProperty()), this);
  point_size_prop_ = new rviz::FloatProperty("Point Size", 8.0, "Size of clicked points", visual_group_prop_,
                                             SLOT(updateViewProperty()), this);
  text_visibility_prop_ = new rviz::BoolProperty("Show Text", true, "Toggles the visibility of the text display",
                                                 visual_group_prop_, SLOT(updateText()), this);
  debug_visibility_prop_ = new rviz::BoolProperty("Show Debug", true, "Toggles the visibility of the debug information",
                                                  visual_group_prop_, SLOT(updateText()), this);
  text_size_prop_ = new rviz::FloatProperty("Text Size", 0.15, "Height of the text display (m)", visual_group_prop_,
                                            SLOT(updateText()), this);
  points_gap_size_prop_ = new rviz::FloatProperty(
      "Point Generation Gap", 0.002, "Separation between adjacent points in a polygon (m)", visual_group_prop_);

  updatePanel();
  newPolygon();
}

void PolygonSimulation::activate()
{
}

void PolygonSimulation::deactivate()
{
}

int PolygonSimulation::processMouseEvent(rviz::ViewportMouseEvent& event)
{
  if (polygon_mode_prop_->getOptionInt() == POLYGON_MODE_GENERAL)
  {
    if (event.leftUp() || (event.left() && lasso_mode_prop_->getBool()))
    {
      auto& polygon = polygons_.back();
      Ogre::Vector3 position;
      if (context_->getSelectionManager()->get3DPoint(event.viewport, event.x, event.y, position))
      {
        if (!polygon->empty())
        {
          const Ogre::Vector3& last_point = polygon->points().back();
          if (last_point.squaredDistance(position) < std::pow(points_gap_size_prop_->getFloat(), 2.0))
            return rviz::Tool::Render;
        }
        polygon->add(position);

        std::unordered_map<std::string, float> params;
        params["POLYGON_CONVEX"] = polygon->is_convex() ? 1.0 : 0.0;

        polygon->setPolygonParams(params);
        polygon_convex_prop_->blockSignals(true);
        polygon_convex_prop_->setValue(polygon->is_convex() ? "True" : "False");
        polygon_convex_prop_->blockSignals(false);
        polygon->render();
        if (collision_detection_prop_->getBool())
          _collisionDetection(polygon);
      }
    }
  }
  else if ((polygon_mode_prop_->getOptionInt() == POLYGON_MODE_RECTANGLE) ||
           (polygon_mode_prop_->getOptionInt() == POLYGON_MODE_DISCSMODEL))
  {
    if (event.left())
    {
      auto& polygon = polygons_.back();
      Ogre::Vector3 position;
      if (context_->getSelectionManager()->get3DPoint(event.viewport, event.x, event.y, position))
      {
        if (polygon->empty())
        {
          polygon->add(position);
        }
        else
        {
          const auto& anchor_pt = polygon->points()[0];
          float l = static_cast<float>(position.x - anchor_pt.x);
          float w = static_cast<float>(position.y - anchor_pt.y);
          float min_dist = std::pow(points_gap_size_prop_->getFloat(), 2.0);
          if (std::fabs(l) < min_dist || std::fabs(w) < min_dist)
            return rviz::Tool::Render;
          std::unordered_map<std::string, float> params;

          if (polygon_mode_prop_->getOptionInt() == POLYGON_MODE_RECTANGLE)
          {
            params["RECTANGLE_WIDTH"] = w;
            params["RECTANGLE_LENGTH"] = l;
            params["RECTANGLE_ANGLE"] = 0.0;
            params["RECTANGLE_ANCHOR_POINT_X"] = anchor_pt.x;
            params["RECTANGLE_ANCHOR_POINT_Y"] = anchor_pt.y;
          }
          else
          {
            params["DISCMODEL_WIDTH"] = w;
            params["DISCMODEL_LENGTH"] = l;
            params["DISCMODEL_ANGLE"] = 0.0;
            params["DISCMODEL_ANCHOR_POINT_X"] = anchor_pt.x;
            params["DISCMODEL_ANCHOR_POINT_Y"] = anchor_pt.y;
            params["DISCMODEL_DISC_NUMBER"] = fixed_discs_num_prop_->getInt();
            params["DISCMODEL_SAFETY_BUFFER"] = safety_buffer_prop_->getFloat();
            params["DISCMODEL_DYNAMIC_MODE"] = dynamic_discs_num_prop_->getBool();
          }
          polygon->setPolygonParams(params);
          rectangle_pt_x_prop_->blockSignals(true);
          rectangle_pt_y_prop_->blockSignals(true);
          rectangle_l_prop_->blockSignals(true);
          rectangle_w_prop_->blockSignals(true);
          rectangle_angle_prop_->blockSignals(true);
          rectangle_pt_x_prop_->setValue(anchor_pt.x);
          rectangle_pt_y_prop_->setValue(anchor_pt.y);
          rectangle_l_prop_->setValue(l);
          rectangle_w_prop_->setValue(w);
          rectangle_angle_prop_->setValue(0.0f);
          rectangle_pt_x_prop_->blockSignals(false);
          rectangle_pt_y_prop_->blockSignals(false);
          rectangle_l_prop_->blockSignals(false);
          rectangle_w_prop_->blockSignals(false);
          rectangle_angle_prop_->blockSignals(false);
          polygon->render();
          if (collision_detection_prop_->getBool())
            _collisionDetection(polygon);
        }
      }
    }
  }
  else if (polygon_mode_prop_->getOptionInt() == POLYGON_MODE_LINESEGEMENT)
  {
    if (event.left())
    {
      auto& polygon = polygons_.back();
      Ogre::Vector3 position;
      if (context_->getSelectionManager()->get3DPoint(event.viewport, event.x, event.y, position))
      {
        if (polygon->empty())
        {
          polygon->add(position);
        }
        else
        {
          std::unordered_map<std::string, float> params;
          const auto& anchor_pt = polygon->points()[0];
          params["LINESEGMENT_START_X"] = anchor_pt.x;
          params["LINESEGMENT_START_Y"] = anchor_pt.y;
          params["LINESEGMENT_END_X"] = position.x;
          params["LINESEGMENT_END_Y"] = position.y;
          polygon->setPolygonParams(params);
          line_segment_s_x_prop_->blockSignals(true);
          line_segment_s_y_prop_->blockSignals(true);
          line_segment_e_x_prop_->blockSignals(true);
          line_segment_e_y_prop_->blockSignals(true);
          line_segment_s_x_prop_->setValue(anchor_pt.x);
          line_segment_s_y_prop_->setValue(anchor_pt.y);
          line_segment_e_x_prop_->setValue(position.x);
          line_segment_e_y_prop_->setValue(position.y);
          line_segment_s_x_prop_->blockSignals(false);
          line_segment_s_y_prop_->blockSignals(false);
          line_segment_e_x_prop_->blockSignals(false);
          line_segment_e_y_prop_->blockSignals(false);
          polygon->render();
          if (collision_detection_prop_->getBool())
            _collisionDetection(polygon);
        }
      }
    }
  }
  else if (polygon_mode_prop_->getOptionInt() == POLYGON_MODE_CIRCLE)
  {
    if (event.left())
    {
      auto& polygon = polygons_.back();
      Ogre::Vector3 position;
      if (context_->getSelectionManager()->get3DPoint(event.viewport, event.x, event.y, position))
      {
        if (polygon->empty())
        {
          polygon->add(position);
          std::unordered_map<std::string, float> params;
          params["CIRCLE_CENTER_X"] = position.x;
          params["CIRCLE_CENTER_Y"] = position.y;
          polygon->setPolygonParams(params);
          circle_pt_x_prop_->blockSignals(true);
          circle_pt_y_prop_->blockSignals(true);
          circle_pt_x_prop_->setValue(position.x);
          circle_pt_y_prop_->setValue(position.y);
          circle_pt_x_prop_->blockSignals(false);
          circle_pt_y_prop_->blockSignals(false);
        }
        else
        {
          const auto& center = polygon->center();
          float r = std::hypot(static_cast<float>(position.x - center.x), static_cast<float>(position.y - center.y));
          float min_dist = std::pow(points_gap_size_prop_->getFloat(), 2.0);
          if (r < min_dist)
            return rviz::Tool::Render;
          std::unordered_map<std::string, float> params;
          params["CIRCLE_RADIUS"] = r;
          polygon->setPolygonParams(params);
          circle_r_prop_->blockSignals(true);
          circle_r_prop_->setValue(r);
          circle_r_prop_->blockSignals(false);
          polygon->render();
          if (collision_detection_prop_->getBool())
            _collisionDetection(polygon);
        }
      }
    }
  }

  if (event.middleUp())
  {
    polygons_.back()->clear();
    // collision detection
    if (collision_detection_prop_->getBool())
    {
      _collisionDetection(polygons_.back());
    }
  }
  else if (event.rightUp())
  {
    if (!polygons_.empty())
    {
      auto& polygon = polygons_.back();
      updateText();
      polygon->done();

      // collision detection
      if (collision_detection_prop_->getBool())
      {
        _collisionDetection(polygon, true);
      }

      newPolygon();
    }
  }

  return rviz::Tool::Render;
}

int PolygonSimulation::processKeyEvent(QKeyEvent* event, rviz::RenderPanel* /*panel*/)
{
  switch (event->key())
  {
    case Qt::Key_Delete:
    case Qt::Key_Backspace:
      polygons_.clear();
      newPolygon();
      break;
    default:
      break;
  }

  return rviz::Tool::Render;
}

/**
 * @brief generate a new polygon object
 */
void PolygonSimulation::newPolygon()
{
  std::size_t index = polygons_.size();

  if (!polygons_.empty() && polygons_.back()->empty())
    return;

  std::shared_ptr<VPolygonNode> polygon;
  if (polygon_mode_prop_->getOptionInt() == POLYGON_MODE_GENERAL)
    polygon = std::shared_ptr<VPolygonNode>(new VPolygon(index));
  else if (polygon_mode_prop_->getOptionInt() == POLYGON_MODE_RECTANGLE)
    polygon = std::shared_ptr<VPolygonNode>(new VRectangle(index));
  else if (polygon_mode_prop_->getOptionInt() == POLYGON_MODE_CIRCLE)
    polygon = std::shared_ptr<VPolygonNode>(new VCircle(index));
  else if (polygon_mode_prop_->getOptionInt() == POLYGON_MODE_DISCSMODEL)
    polygon = std::shared_ptr<VPolygonNode>(new VDiscsModel(index));
  else if (polygon_mode_prop_->getOptionInt() == POLYGON_MODE_LINESEGEMENT)
    polygon = std::shared_ptr<VPolygonNode>(new VLineSegment(index));

  polygon->setSceneManager(scene_manager_);
  auto line_color = collision_detection_prop_->getBool() ? SAFE_LINE_QT_COLOR : line_color_prop_->getColor();
  auto point_color = collision_detection_prop_->getBool() ? SAFE_POINT_QT_COLOR : point_color_prop_->getColor();
  polygon->setLineVisualProperty(line_color);
  polygon->setPointVisualProperty(point_color, point_size_prop_->getFloat());
  _initPolygonParams(polygon);
  polygons_.push_back(std::move(polygon));
}

/**
 * @brief Update pannel display (polygon/rectangle/circle)
 */
void PolygonSimulation::updatePanel()
{
  auto option = polygon_mode_prop_->getOptionInt();
  // common
  line_color_prop_->setHidden(collision_detection_prop_->getBool());
  point_color_prop_->setHidden(collision_detection_prop_->getBool());
  // general polygon
  lasso_mode_prop_->setHidden(option != POLYGON_MODE_GENERAL);
  polygon_convex_prop_->setHidden(option != POLYGON_MODE_GENERAL);
  // rectangle
  rectangle_pt_x_prop_->setHidden((option != POLYGON_MODE_RECTANGLE) && (option != POLYGON_MODE_DISCSMODEL));
  rectangle_pt_y_prop_->setHidden((option != POLYGON_MODE_RECTANGLE) && (option != POLYGON_MODE_DISCSMODEL));
  rectangle_l_prop_->setHidden((option != POLYGON_MODE_RECTANGLE) && (option != POLYGON_MODE_DISCSMODEL));
  rectangle_w_prop_->setHidden((option != POLYGON_MODE_RECTANGLE) && (option != POLYGON_MODE_DISCSMODEL));
  rectangle_angle_prop_->setHidden((option != POLYGON_MODE_RECTANGLE) && (option != POLYGON_MODE_DISCSMODEL));
  // circle
  circle_pt_x_prop_->setHidden(option != POLYGON_MODE_CIRCLE);
  circle_pt_y_prop_->setHidden(option != POLYGON_MODE_CIRCLE);
  circle_r_prop_->setHidden(option != POLYGON_MODE_CIRCLE);
  // disc model
  dynamic_discs_num_prop_->setHidden(option != POLYGON_MODE_DISCSMODEL);
  safety_buffer_prop_->setHidden((option != POLYGON_MODE_DISCSMODEL) || !dynamic_discs_num_prop_->getBool());
  fixed_discs_num_prop_->setHidden((option != POLYGON_MODE_DISCSMODEL) || dynamic_discs_num_prop_->getBool());
  // line segment
  line_segment_s_x_prop_->setHidden(option != POLYGON_MODE_LINESEGEMENT);
  line_segment_s_y_prop_->setHidden(option != POLYGON_MODE_LINESEGEMENT);
  line_segment_e_x_prop_->setHidden(option != POLYGON_MODE_LINESEGEMENT);
  line_segment_e_y_prop_->setHidden(option != POLYGON_MODE_LINESEGEMENT);

  if (!polygons_.empty() && !polygons_.back()->valid())
  {
    polygons_.pop_back();
    newPolygon();
  }

  // collision detection
  if (collision_detection_prop_->getBool())
  {
    for (auto& polygon : polygons_)
    {
      if (polygon->collide())
      {
        polygon->setLineVisualProperty(COLLISION_LINE_QT_COLOR);
        polygon->setPointVisualProperty(COLLISION_POINT_QT_COLOR, point_size_prop_->getFloat());
      }
      else
      {
        polygon->setLineVisualProperty(SAFE_LINE_QT_COLOR);
        polygon->setPointVisualProperty(SAFE_POINT_QT_COLOR, point_size_prop_->getFloat());
      }
    }
  }
}

/**
 * @brief Update polygon view properties
 */
void PolygonSimulation::updateViewProperty()
{
  if (same_visual_prop_->getBool())
  {
    for (auto& polygon : polygons_)
    {
      polygon->setLineVisualProperty(line_color_prop_->getColor());
      polygon->setPointVisualProperty(point_color_prop_->getColor(), point_size_prop_->getFloat());
    }
  }
  else
  {
    auto& polygon = polygons_.back();
    polygon->setLineVisualProperty(line_color_prop_->getColor());
    polygon->setPointVisualProperty(point_color_prop_->getColor(), point_size_prop_->getFloat());
  }
}

/**
 * @brief Collision detection callback
 */
void PolygonSimulation::onCollisionDetection()
{
  updatePanel();

  std::size_t nums = polygons_.size();

  if (!collision_detection_prop_->getBool())
  {
    for (size_t i = 0; i < nums; i++)
    {
      polygons_[i]->setLineVisualProperty(DEFAULT_LINE_QT_COLOR);
    }
    return;
  }

  for (size_t i = 0; i < nums; i++)
  {
    polygons_[i]->setLineVisualProperty(SAFE_LINE_QT_COLOR);
  }

  for (size_t i = 0; i < nums; i++)
  {
    const auto& polygon_a = polygons_[i];
    if (!polygon_a->valid())
      continue;
    for (size_t j = i + 1; j < nums; j++)
    {
      const auto& polygon_b = polygons_[j];
      if (!polygon_b->valid())
        continue;
      if (polygon_a->isCollisionWith(polygon_b))
      {
        polygon_a->setLineVisualProperty(COLLISION_LINE_QT_COLOR);
        polygon_b->setLineVisualProperty(COLLISION_LINE_QT_COLOR);
      }
    }
  }
}

/**
 * @brief Update text display
 */
void PolygonSimulation::updateText()
{
  const float text_size = text_size_prop_->getFloat();
  const bool text_visible = text_visibility_prop_->getBool();
  const bool debug_visible = debug_visibility_prop_->getBool();
  if (text_visible)
    text_size_prop_->setHidden(false);
  else
    text_size_prop_->setHidden(true);

  for (const auto& polygon : polygons_)
    polygon->updateText(text_size, text_visible, debug_visible);
}

/**
 * @brief Update polygon view (points and lines)
 */
void PolygonSimulation::updateView()
{
  auto& polygon = polygons_.back();
  _initPolygonParams(polygon);
  polygon->render();

  if (collision_detection_prop_->getBool())
    _collisionDetection(polygon);
}

/**
 * @brief callback function for ros service
 */
bool PolygonSimulation::_callback(GetSelection::Request& /*req*/, GetSelection::Response& res)
{
  res.selection.reserve(polygons_.size());
  for (const auto& polygon : polygons_)
  {
    // Skip selections with fewer than 3 points
    if (polygon->size() < 3)
      continue;

    PolygonStamped polygon_stamped;
    polygon_stamped.header.frame_id = context_->getFixedFrame().toStdString();
    for (const Ogre::Vector3& pt : polygon->points())
    {
      geometry_msgs::Point32 msg;
      msg.x = pt.x;
      msg.y = pt.y;
      msg.z = pt.z;
      polygon_stamped.polygon.points.push_back(msg);
    }

    res.selection.push_back(polygon_stamped);
  }
  return true;
}

/**
 * @brief refresh polygon parameters
 */
void PolygonSimulation::_initPolygonParams(std::shared_ptr<VPolygonNode>& polygon)
{
  std::unordered_map<std::string, float> params;
  if (polygon_mode_prop_->getOptionInt() == POLYGON_MODE_GENERAL)
  {
    params["POLYGON_LASSO_MODE"] = lasso_mode_prop_->getBool();
    params["POLYGON_CONVEX"] = polygon_convex_prop_->getStdString() == "True" ? 1.0f : 0.0f;
  }
  else if (polygon_mode_prop_->getOptionInt() == POLYGON_MODE_RECTANGLE)
  {
    params["RECTANGLE_WIDTH"] = rectangle_w_prop_->getFloat();
    params["RECTANGLE_LENGTH"] = rectangle_l_prop_->getFloat();
    params["RECTANGLE_ANGLE"] = rectangle_angle_prop_->getFloat();
    params["RECTANGLE_ANCHOR_POINT_X"] = rectangle_pt_x_prop_->getFloat();
    params["RECTANGLE_ANCHOR_POINT_Y"] = rectangle_pt_y_prop_->getFloat();
  }
  else if (polygon_mode_prop_->getOptionInt() == POLYGON_MODE_DISCSMODEL)
  {
    params["DISCMODEL_WIDTH"] = rectangle_w_prop_->getFloat();
    params["DISCMODEL_LENGTH"] = rectangle_l_prop_->getFloat();
    params["DISCMODEL_ANGLE"] = rectangle_angle_prop_->getFloat();
    params["DISCMODEL_ANCHOR_POINT_X"] = rectangle_pt_x_prop_->getFloat();
    params["DISCMODEL_ANCHOR_POINT_Y"] = rectangle_pt_y_prop_->getFloat();
    params["DISCMODEL_DISC_NUMBER"] = fixed_discs_num_prop_->getInt();
    params["DISCMODEL_SAFETY_BUFFER"] = safety_buffer_prop_->getFloat();
    params["DISCMODEL_DYNAMIC_MODE"] = dynamic_discs_num_prop_->getBool();
  }
  else if (polygon_mode_prop_->getOptionInt() == POLYGON_MODE_LINESEGEMENT)
  {
    params["LINESEGMENT_START_X"] = line_segment_s_x_prop_->getFloat();
    params["LINESEGMENT_START_Y"] = line_segment_s_y_prop_->getFloat();
    params["LINESEGMENT_END_X"] = line_segment_e_x_prop_->getFloat();
    params["LINESEGMENT_END_Y"] = line_segment_e_y_prop_->getFloat();
  }
  else if (polygon_mode_prop_->getOptionInt() == POLYGON_MODE_CIRCLE)
  {
    params["CIRCLE_RADIUS"] = circle_r_prop_->getFloat();
    params["CIRCLE_CENTER_X"] = circle_pt_x_prop_->getFloat();
    params["CIRCLE_CENTER_Y"] = circle_pt_y_prop_->getFloat();
  }
  polygon->setPolygonParams(params);
}

/**
 * @brief Collision detection and update visualization
 */
void PolygonSimulation::_collisionDetection(std::shared_ptr<VPolygonNode>& polygon, bool set_collid)
{
  bool self_collision = false;
  for (const auto& other : polygons_)
  {
    if (!other->valid() || other == polygon)
      continue;

    bool other_collision = false;
    if (!polygon->empty() && polygon->isCollisionWith(other))
    {
      self_collision = true;
      other_collision = true;
    }
    if (other_collision || other->collide())
    {
      other->setLineVisualProperty(COLLISION_LINE_QT_COLOR);
      other->setPointVisualProperty(COLLISION_POINT_QT_COLOR, point_size_prop_->getFloat());
      if (set_collid)
        other->setCollision(true);
    }
    else
    {
      other->setLineVisualProperty(SAFE_LINE_QT_COLOR);
      other->setPointVisualProperty(SAFE_POINT_QT_COLOR, point_size_prop_->getFloat());
    }
  }

  if (self_collision)
  {
    polygon->setLineVisualProperty(COLLISION_LINE_QT_COLOR);
    polygon->setPointVisualProperty(COLLISION_POINT_QT_COLOR, point_size_prop_->getFloat());
    if (set_collid)
      polygon->setCollision(true);
  }
  else
  {
    polygon->setLineVisualProperty(SAFE_LINE_QT_COLOR);
    polygon->setPointVisualProperty(SAFE_POINT_QT_COLOR, point_size_prop_->getFloat());
  }
}

}  // namespace polygon_simulation
}  // namespace rmp