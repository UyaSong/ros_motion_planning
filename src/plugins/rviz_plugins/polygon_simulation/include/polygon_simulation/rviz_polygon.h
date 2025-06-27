/**
 * *********************************************************
 *
 * @file: rviz_polygon.h
 * @brief: Polygon object for rviz plugin
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
#ifndef POLYGON_SIMULATION_RVIZ_POLYGON_H
#define POLYGON_SIMULATION_RVIZ_POLYGON_H

#include "rviz_polygon_node.h"

namespace rmp
{
namespace polygon_simulation
{
class VPolygon : public VPolygonNode
{
public:
  /**
   * @brief Construct a new Visual polygon object
   * @param idx        polygon index
   */
  VPolygon();
  VPolygon(unsigned int idx);

  /**
   * @brief Destroy the Visual polygon object
   */
  virtual ~VPolygon();

  /**
   * @brief Set parameters for the polygon
   * @param params        parameters map <key, val>
   */
  virtual void setPolygonParams(std::unordered_map<std::string, float>& params) override;

  /**
   * @brief Trigger polygon rendering
   */
  virtual void render() override;

  bool is_convex() const;

  /**
   * @brief Collision detection
   * @param other   other polygon instance
   * @return flag   collision occurs (true) or not (false)
   */
  bool isCollisionWith(const std::shared_ptr<VPolygonNode>& other) override;

private:
  bool _SAT(const std::shared_ptr<VPolygon>& other);

private:
  bool lasso_mode_{ false };
  bool is_convex_{ false };
};
}  // namespace polygon_simulation
}  // namespace rmp
#endif