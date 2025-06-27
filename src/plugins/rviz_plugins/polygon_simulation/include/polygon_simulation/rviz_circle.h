/**
 * *********************************************************
 *
 * @file: rviz_circle.h
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
#ifndef POLYGON_SIMULATION_RVIZ_CIRCLE_H
#define POLYGON_SIMULATION_RVIZ_CIRCLE_H

#include "rviz_polygon_node.h"

namespace rmp
{
namespace polygon_simulation
{
class VCircle : public VPolygonNode
{
public:
  /**
   * @brief Construct a new Visual circle object
   * @param idx        polygon index
   */
  VCircle();
  VCircle(unsigned int idx);

  /**
   * @brief Destroy the Visual circle object
   */
  virtual ~VCircle();

  /**
   * @brief Get the radius of circle
   */
  float radius() const;

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

private:
  float radius_{ 0.0f };
};

}  // namespace polygon_simulation
}  // namespace rmp
#endif