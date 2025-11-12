/*
 *  Copyright (C) 2025 Seyond Inc.
 *
 *  License: Apache License
 *
 *  Point type definitions for seyond_decoder
 */

#pragma once

#include <pcl/point_types.h>

namespace seyond
{

struct EIGEN_ALIGN16 PointXYZIT
{
  PCL_ADD_POINT4D;
  float intensity;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace seyond

POINT_CLOUD_REGISTER_POINT_STRUCT(
  seyond::PointXYZIT,
  (float, x,
   x)(float, y, y)(float, z, z)(float, intensity, intensity)(double, timestamp, timestamp))