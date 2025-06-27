/**
 * *********************************************************
 *
 * @file: rviz_rectangle.h
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
#ifndef POLYGON_SIMULATION_RVIZ_RECTANGLE_H
#define POLYGON_SIMULATION_RVIZ_RECTANGLE_H

#include "rviz_polygon_node.h"

namespace rmp
{
namespace polygon_simulation
{
class VRectangle : public VPolygonNode
{
public:
  /**
   * @brief Construct a new Visual rectangle object
   * @param idx        polygon index
   */
  VRectangle();
  VRectangle(unsigned int idx);

  /**
   * @brief Destroy the Visual rectangle object
   */
  virtual ~VRectangle();

  /**
   * @brief Get the left-top point of rectangle
   */
  const Ogre::Vector3& left_top_point() const;

  /**
   * @brief Get the length of rectangle
   */
  float length() const;

  /**
   * @brief Get the width of rectangle
   */
  float width() const;

  /**
   * @brief Get the angle of rectangle
   */
  float angle() const;

  /**
   * @brief Get the cos(angle) of rectangle
   */
  float cos_angle() const;

  /**
   * @brief Get the sin(angle) of rectangle
   */
  float sin_angle() const;

  /**
   * @brief Set parameters for the polygon
   * @param params        parameters map <key, val>
   */
  void setPolygonParams(std::unordered_map<std::string, float>& params) override;

  /**
   * @brief Trigger polygon rendering
   */
  void render() override;

  /**
   * @brief Collision detection
   * @param other   other polygon instance
   * @return flag   collision occurs (true) or not (false)
   */
  bool isCollisionWith(const std::shared_ptr<VPolygonNode>& other) override;

protected:
  /**
   * @brief Axis-aligned Bounding Box Collision detection
   * @param other   other rectangle instance
   * @return flag   collision occurs (true) or not (false)
   */
  bool _AABBCollisionDetection(const std::shared_ptr<VRectangle>& other);

  /**
   * @brief Oriented Bounding Box Collision detection
   * @param other   other rectangle instance
   * @return flag   collision occurs (true) or not (false)
   */
  bool _OBBCollisionDetection(const std::shared_ptr<VRectangle>& other);

protected:
  Ogre::Vector3 anchor_pt_;  // anchor point
  Ogre::Vector3 lt_pt_;      // left-top point of rectangle
  float l_{ 0.0f };          // length
  float w_{ 0.0f };          // width
  float angle_{ 0.0f };      // angle
  float cos_angle_{ 1.0f };  // cos(angle)
  float sin_angle_{ 0.0f };  // sin(angle)
};

}  // namespace polygon_simulation
}  // namespace rmp
#endif