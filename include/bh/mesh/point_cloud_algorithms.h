//==================================================
// point_cloud_algorithms.h
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 28.09.17
//==================================================
#pragma once

#include <type_traits>
#include "point_cloud.h"
#include "../math/geometry.h"

namespace bh {

class PointCloudAlgorithms {
public:

  template<typename FloatT>
  static BoundingBox3D<FloatT> getBoundingBox3D(const PointCloud<FloatT> &pc) {
    BH_USE_FIXED_EIGEN_TYPES(FloatT);
    BoundingBox3D<FloatT> bbox = BoundingBox3D<FloatT>::Invalid();
    for (const Vector3 &point : pc.vertices()) {
      bbox.include(point);
    }
    return bbox;
  }

  template <typename FloatT, typename IndexT = int64_t>
  static PointCloud<FloatT> filterOnVoxelGrid(
          const PointCloud<FloatT>& input,
          const typename bh::EigenTypes<FloatT>::Vector3& voxel_size,
          const IndexT min_points_per_voxel = 1);

  template <typename FloatT, typename IndexT = int64_t>
  static PointCloud<FloatT> filterOnVoxelGrid(
          const PointCloud<FloatT>& input,
          const FloatT voxel_size,
          const IndexT min_points_per_voxel = 1) {
    BH_USE_FIXED_EIGEN_TYPES(FloatT);
    const Vector3 voxel_size_vector(voxel_size, voxel_size, voxel_size);
    return filterOnVoxelGrid<FloatT, IndexT>(input, voxel_size_vector, min_points_per_voxel);
  };

};

}

#include "point_cloud_algorithms_voxel_grid.hxx"
