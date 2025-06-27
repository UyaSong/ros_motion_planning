/**
 * *********************************************************
 *
 * @file: polygon_simulation.h
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
#include <ros/ros.h>
#include <rviz/tool.h>
#include <rviz/ogre_helpers/billboard_line.h>

#include <OgreMaterial.h>
#include <OgreVector3.h>
#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include "polygon_simulation/rviz_polygon_node.h"
#include "polygon_simulation/GetSelection.h"

#define DEFAULT_LINE_QT_COLOR Qt::blue
#define DEFAULT_POINT_QT_COLOR Qt::red
#define COLLISION_LINE_QT_COLOR Qt::red
#define COLLISION_POINT_QT_COLOR Qt::red
#define SAFE_LINE_QT_COLOR Qt::darkGreen
#define SAFE_POINT_QT_COLOR Qt::darkGreen

namespace rviz
{
class DisplayContext;
class SelectionManager;
class MovableText;

class EnumProperty;
class BoolProperty;
class ColorProperty;
class FloatProperty;
class StringProperty;
class IntProperty;
}  // namespace rviz

using GetSelection = polygon_simulation::GetSelection;
using PolygonStamped = polygon_simulation::PolygonStamped;

namespace rmp
{
namespace polygon_simulation
{
class PolygonSimulation : public rviz::Tool
{
  Q_OBJECT

public:
  PolygonSimulation();
  virtual ~PolygonSimulation();
  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz::ViewportMouseEvent& event) override;
  int processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel) override;

public:
  /**
   * @brief generate a new polygon object
   */
  void newPolygon();

public Q_SLOTS:
  /**
   * @brief Update pannel display (polygon/rectangle/circle)
   */
  void updatePanel();

  /**
   * @brief Update text display
   */
  void updateText();

  /**
   * @brief Update polygon view (points and lines)
   */
  void updateView();

  /**
   * @brief Update polygon view properties
   */
  void updateViewProperty();

  /**
   * @brief Collision detection callback (all polygons)
   */
  void onCollisionDetection();

private:
  /**
   * @brief callback function for ros service
   */
  bool _callback(GetSelection::Request&, GetSelection::Response& res);

  /**
   * @brief refresh polygon parameters
   */
  void _initPolygonParams(std::shared_ptr<VPolygonNode>& polygon);

  /**
   * @brief Collision detection and update visualization (single polygon)
   */
  void _collisionDetection(std::shared_ptr<VPolygonNode>& polygon, bool set_collid = false);

  // visualization
  rviz::EnumProperty* polygon_mode_prop_;
  rviz::ColorProperty* point_color_prop_;
  rviz::ColorProperty* line_color_prop_;
  rviz::FloatProperty* point_size_prop_;
  rviz::BoolProperty* text_visibility_prop_;
  rviz::BoolProperty* debug_visibility_prop_;
  rviz::BoolProperty* same_visual_prop_;
  rviz::FloatProperty* text_size_prop_;
  rviz::FloatProperty* points_gap_size_prop_;
  rviz::StringProperty* visual_group_prop_;

  // common
  rviz::BoolProperty* collision_detection_prop_;

  // general polygon
  rviz::BoolProperty* lasso_mode_prop_;
  rviz::StringProperty* polygon_convex_prop_;

  // rectangle
  rviz::FloatProperty* rectangle_pt_x_prop_;
  rviz::FloatProperty* rectangle_pt_y_prop_;
  rviz::FloatProperty* rectangle_l_prop_;
  rviz::FloatProperty* rectangle_w_prop_;
  rviz::FloatProperty* rectangle_angle_prop_;

  // circle
  rviz::FloatProperty* circle_pt_x_prop_;
  rviz::FloatProperty* circle_pt_y_prop_;
  rviz::FloatProperty* circle_r_prop_;

  // disc model
  rviz::BoolProperty* dynamic_discs_num_prop_;
  rviz::FloatProperty* safety_buffer_prop_;
  rviz::IntProperty* fixed_discs_num_prop_;

  // line segment
  rviz::FloatProperty* line_segment_s_x_prop_;
  rviz::FloatProperty* line_segment_s_y_prop_;
  rviz::FloatProperty* line_segment_e_x_prop_;
  rviz::FloatProperty* line_segment_e_y_prop_;

  // ros
  ros::NodeHandle nh_;
  ros::ServiceServer server_;

  // polygons
  std::vector<std::shared_ptr<VPolygonNode>> polygons_;
};

}  // namespace polygon_simulation
}  // namespace rmp
