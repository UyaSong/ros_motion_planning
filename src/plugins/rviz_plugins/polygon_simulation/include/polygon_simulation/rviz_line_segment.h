/**
 * *********************************************************
 *
 * @file: rviz_line_segement.h
 * @brief: Line segment object for rviz plugin
 * @author: Yang Haodong
 * @date: 2024-9-15
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef POLYGON_SIMULATION_RVIZ_LINESEGMENT_H
#define POLYGON_SIMULATION_RVIZ_LINESEGMENT_H

#include "rviz_polygon_node.h"

namespace rmp
{
namespace polygon_simulation
{
class VLineSegment : public VPolygonNode
{
public:
  /**
   * @brief Construct a new Visual line segment object
   * @param idx        polygon index
   */
  VLineSegment();
  VLineSegment(unsigned int idx);

  /**
   * @brief Destroy the Visual line segment object
   */
  virtual ~VLineSegment();

  /**
   * @brief Get the start of line segment
   */
  const Ogre::Vector3& start() const;

  /**
   * @brief Get the end of line segment
   */
  const Ogre::Vector3& end() const;

  /**
   * @brief Get the length of line segment
   */
  float length() const;

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

  /**
   * @brief Line segment intersaction detection
   * @param other   other line segment
   * @return flag   collision occurs (true) or not (false)
   */
  bool isLineIntersaction(const std::shared_ptr<VLineSegment>& other) const;
  bool isLineIntersaction(const Ogre::Vector3& start, const Ogre::Vector3& end) const;

protected:
  /**
   * @brief End points test for line segment intersaction
   * @param start   the start point of other line segment instance
   * @param end     the end point of other line segment instance
   * @return flag   start/end is equal to other's start/end (true) or not (false)
   */
  bool _endPointTest(const Ogre::Vector3& start, const Ogre::Vector3& end) const;
  /**
   * @brief Rapid test for line segment intersaction
   * @param start   the start point of other line segment instance
   * @param end     the end point of other line segment instance
   * @return flag   intersaction may occurs (true) or not (false)
   */
  bool _rapidTest(const Ogre::Vector3& start, const Ogre::Vector3& end) const;
  /**
   * @brief Straddle test for line segment intersaction
   * @param start   the start point of other line segment instance
   * @param end     the end point of other line segment instance
   * @return flag   collision occurs (true) or not (false)
   */
  bool _straddleTest(const Ogre::Vector3& start, const Ogre::Vector3& end) const;

private:
  /**
   * @brief Check if a point is within the line segment.
   * @param point The point to check if it is within the line segment.
   * @return Whether the input point is within the line segment or not.
   */
  bool _isPointIn(const Ogre::Vector3& point) const;

private:
  Ogre::Vector3 start_;
  Ogre::Vector3 end_;
  float length_;
};

}  // namespace polygon_simulation
}  // namespace rmp
#endif