/**
 * *********************************************************
 *
 * @file: rviz_discs_model.h
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
#ifndef POLYGON_SIMULATION_RVIZ_DISCS_MODEL_H
#define POLYGON_SIMULATION_RVIZ_DISCS_MODEL_H

#include "rviz_rectangle.h"

namespace rmp
{
namespace polygon_simulation
{
class VDiscsModel : public VRectangle
{
public:
  /**
   * @brief Construct a new Visual discs model object
   * @param idx        polygon index
   */
  VDiscsModel();
  VDiscsModel(unsigned int idx);

  const std::vector<std::pair<Ogre::Vector3, float>>& discs() const;

  /**
   * @brief Destroy the Visual discs model object
   */
  virtual ~VDiscsModel();

  /**
   * @brief Set parameters for the discs model
   * @param params        parameters map <key, val>
   */
  void setPolygonParams(std::unordered_map<std::string, float>& params) override;

  /**
   * @brief Trigger discs model rendering
   */
  void render() override;

  /**
   * @brief Collision detection
   * @param other   other polygon instance
   * @return flag   collision occurs (true) or not (false)
   */
  bool isCollisionWith(const std::shared_ptr<VPolygonNode>& other) override;

private:
  int disc_nums_;
  bool dynamic_mode_;
  float safety_buffer_;
  std::vector<std::pair<Ogre::Vector3, float>> discs_;
};

}  // namespace polygon_simulation
}  // namespace rmp
#endif