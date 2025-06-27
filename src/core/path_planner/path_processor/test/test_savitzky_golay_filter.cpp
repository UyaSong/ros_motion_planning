/**
 * *********************************************************
 *
 * @file: test_savitzky_golay_filter.cpp
 * @brief: Savitzky-Golay Filter test file
 * @author: Yang Haodong
 * @date: 2024-11-17
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <random>
#include <gtest/gtest.h>
#include <matplotlibcpp.h>

#include "common/util/log.h"
#include "common/geometry/point.h"
#include "path_planner/path_smooth/savitzky_golay.h"

namespace plt = matplotlibcpp;

TEST(TestSavitzkyGolayFilter, Numerical)
{
  int sample_num = 100;
  std::random_device rd;
  std::mt19937 eng(rd());
  std::uniform_real_distribution<float> p(0, 1);

  std::vector<double> origin_x, origin_y;
  std::vector<double> smooth_x, smooth_y;
  rmp::common::geometry::Points3d origin_pts, smooth_pts;

  // origin noisy data
  for (int i = 0; i < sample_num; i++)
  {
    double x = static_cast<double>(i);
    double y = std::sin(x / sample_num * 2 * M_PI) + 0.5 * p(eng);
    origin_x.push_back(x);
    origin_y.push_back(y);
    origin_pts.emplace_back(x, y);
  }

  // smooth data
  auto sag_filter = std::make_unique<rmp::path_planner::SavitzkyGolayPathProcessor>();
  sag_filter->process(origin_pts, smooth_pts);
  for (const auto& pt: smooth_pts) {
      smooth_x.push_back(pt.x());
      smooth_y.push_back(pt.y());
  }

  plt::plot(origin_x, origin_y, "b-");
  plt::plot(smooth_x, smooth_y, "r-");
  plt::xlabel("x");
  plt::ylabel("y");
  plt::grid(true);
  plt::title("Numerical test using Savitzky-Golay filter");
  plt::show();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}