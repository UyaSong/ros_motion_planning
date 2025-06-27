/**
 * *********************************************************
 *
 * @file: polygon_simulation_utils.h
 * @brief: useful function for polygon simulation package
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
#ifndef POLYGON_SIMULATION_UTILS_H
#define POLYGON_SIMULATION_UTILS_H

#include <OgreVector3.h>

namespace rmp
{
namespace polygon_simulation
{
namespace
{
constexpr float kMathEpsilon = 1e-6;
}

/**
 * @brief Cross product between two 2-D vectors from the common start point,
 *        and end at two other points.
 **/
static float crossProduct(const Ogre::Vector3& s, const Ogre::Vector3& e1, const Ogre::Vector3& e2)
{
  const Ogre::Vector3 vec_s_e1 = e1 - s;
  const Ogre::Vector3 vec_s_e2 = e2 - s;
  return vec_s_e1.x * vec_s_e2.y - vec_s_e1.y * vec_s_e2.x;
};

/**
 * @brief Inner product between two 2-D vectors from the common start point,
 *        and end at two other points.
 **/
static float innerProduct(const Ogre::Vector3& v1, const Ogre::Vector3& v2)
{
  return v1.x * v2.x + v1.y * v2.y;
}

static bool isWithin(float val, float bound1, float bound2)
{
  if (bound1 > bound2)
  {
    std::swap(bound1, bound2);
  }
  return val >= bound1 - kMathEpsilon && val <= bound2 + kMathEpsilon;
};
}  // namespace polygon_simulation
}  // namespace rmp

#endif