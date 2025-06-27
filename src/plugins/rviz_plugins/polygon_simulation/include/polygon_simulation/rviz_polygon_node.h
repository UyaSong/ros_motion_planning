/**
 * *********************************************************
 *
 * @file: rviz_polygon_node.h
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
#ifndef POLYGON_SIMULATION_RVIZ_POLYGON_NODE_H
#define POLYGON_SIMULATION_RVIZ_POLYGON_NODE_H

#include <unordered_map>

#include <QColor>

#include <OgreMaterial.h>
#include <OgreVector3.h>
#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

namespace Ogre
{
class Material;
class Vector3;
class SceneManager;
class SceneNode;
class ManualObject;
}  // namespace Ogre

namespace rviz
{
class MovableText;
}  // namespace rviz

namespace rmp
{
namespace polygon_simulation
{
enum PolygonMode
{
  POLYGON_MODE_NONE,
  POLYGON_MODE_GENERAL,
  POLYGON_MODE_RECTANGLE,
  POLYGON_MODE_CIRCLE,
  POLYGON_MODE_DISCSMODEL,
  POLYGON_MODE_LINESEGEMENT,
};

class VPolygonNode
{
public:
  /**
   * @brief Construct a new Visual polygon node object
   * @param idx        polygon index
   */
  VPolygonNode();
  VPolygonNode(unsigned int idx);

  /**
   * @brief Destroy the Visual polygon node object
   */
  virtual ~VPolygonNode();

public:
  /**
   * @brief Get polygon index
   */
  unsigned int id() const;

  /**
   * @brief Get polygon points size
   */
  int size() const;

  /**
   * @brief Judge whether there are points inside object
   */
  bool empty() const;

  /**
   * @brief Judge whether the polygon is valid. Polygons that have not been created are invalid
   */
  bool valid() const;

  /**
   * @brief Judge whether the polygon collides with others
   */
  bool collide() const;

  /**
   * @brief Get polygon mode (polygon, rectangle or circle)
   */
  PolygonMode mode() const;

  /**
   * @brief Get center point of polygon
   */
  const Ogre::Vector3& center() const;

  /**
   * @brief Get convex property of polygon
   */
  bool is_convex() const;

  /**
   * @brief Get polygons of polygon
   */
  const std::vector<Ogre::Vector3>& points() const;

  bool operator==(VPolygonNode& other) const;
  bool operator!=(VPolygonNode& other) const;

  /**
   * @brief Add point into polygon
   * @param point      the point to add
   */
  void add(Ogre::Vector3 point);

  /**
   * @brief Get the next index of given index.
   * @param at the given index
   * @return next index
   */
  int next(int at) const;

  /**
   * @brief Get the previous index of given index.
   * @param at the given index
   * @return previous index
   */
  int prev(int at) const;

  /**
   * @brief Clear the created points/lines/text
   */
  void clear();

  /**
   * @brief Clear the created points/lines/text and destory the scene nodes
   */
  void destory();

  /**
   * @brief Set the polygon valid
   */
  void done();

  /**
   * @brief Set the collision flag for the polygon
   * @param collision_flag      collision flag
   */
  void setCollision(bool collision_flag);

  /**
   * @brief Set the parent scene manager meanwhile create point/line/text scene node
   * @param scene_manager      parent scene manager
   */
  void setSceneManager(Ogre::SceneManager* scene_manager);

  /**
   * @brief Set the parent scene manager meanwhile create point/line/text scene node
   * @param text_size      text size
   * @param is_visible     whether the text is visible
   * @param show_debug     whether show the debug information
   */
  void updateText(float text_size, bool is_visible = true, bool show_debug = false);

  /**
   * @brief Set visual properties for the lines
   * @param line_color     line color
   */
  void setLineVisualProperty(QColor line_color);

  /**
   * @brief Set visual properties for the points
   * @param point_color     pointpoint color
   * @param point_size      point size
   */
  void setPointVisualProperty(QColor point_color, float point_size);

  /**
   * @brief Set parameters for the polygon
   * @param params        parameters map <key, val>
   */
  virtual void setPolygonParams(std::unordered_map<std::string, float>& params) = 0;

  /**
   * @brief Trigger polygon rendering
   */
  virtual void render() = 0;

  /**
   * @brief Collision detection
   * @param other   other polygon instance
   * @return flag   collision occurs (true) or not (false)
   */
  virtual bool isCollisionWith(const std::shared_ptr<VPolygonNode>& other) = 0;

protected:
  /**
   * @brief update material (point or line) color
   */
  static void updateMaterialColor(Ogre::MaterialPtr material, const QColor& color)
  {
    qreal r, g, b, a;
    color.getRgbF(&r, &g, &b, &a);
    material->setDiffuse(r, g, b, a);
    material->setSpecular(r, g, b, a);
    material->setAmbient(r, g, b);
  }

  /**
   * @brief transform the float number to string with fixed precision
   */
  static std::string numberToString(float value, int precision)
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value;
    return oss.str();
  }

  /**
   * @brief draw lines in the figure
   * @param lines      lines object
   * @param draw_points    points to draw
   * @param draw_continue continue drawing (true) or not (false)
   */
  void drawLines(Ogre::ManualObject* lines, const std::vector<Ogre::Vector3>& draw_points, bool draw_continue = false);

  /**
   * @brief draw points in the figure
   * @param points        points object
   * @param draw_points    points to draw
   * @param draw_continue continue drawing (true) or not (false)
   */
  void drawPoints(Ogre::ManualObject* points, const std::vector<Ogre::Vector3>& draw_points,
                  bool draw_continue = false);

protected:
  // index
  unsigned int idx_;
  std::string point_idx_;
  std::string line_idx_;

  // ogre nodes
  Ogre::SceneManager* scene_manager_;
  Ogre::SceneNode* point_node_;
  Ogre::SceneNode* line_node_;
  Ogre::SceneNode* text_node_;
  Ogre::MaterialPtr point_material_;
  Ogre::MaterialPtr line_material_;
  std::vector<Ogre::SceneNode*> debug_text_nodes_;

  // common
  PolygonMode mode_;
  bool collid_{ false };
  bool valid_{ false };
  Ogre::Vector3 center_;
  std::vector<Ogre::Vector3> points_;
  bool is_convex_{ false };
  double area_{ 0.0 };

  // [value-px-py]
  std::vector<std::pair<std::string, std::pair<float, float>>> debug_info_;
};
}  // namespace polygon_simulation
}  // namespace rmp
#endif