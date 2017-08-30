//==================================================
// distance_field.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Mar 9, 2017
//==================================================
#pragma once

#include <cstddef>
#include "../eigen.h"
#include "geometry.h"
#include "grid3d.h"

namespace bh {

#if __GNUC__ && !__CUDACC__
  #pragma GCC push_options
  #pragma GCC optimize ("fast-math")
#endif

template <typename FloatT>
class ContinuousGrid3DUtils {
public:
    using FloatType = FloatT;
    USE_FIXED_EIGEN_TYPES(FloatType);

    ContinuousGrid3DUtils(const FloatT grid_increment)
    : grid_increment_(grid_increment) {}

    ContinuousGrid3DUtils(const FloatT grid_increment, const FloatT shift_factor)
    : grid_increment_(grid_increment),
      grid_offset_(shift_factor * grid_increment * Vector3::Ones()) {}

    ContinuousGrid3DUtils(const FloatT grid_increment, const Vector3 grid_offset)
    : grid_increment_(grid_increment), grid_offset_(grid_offset) {}

    FloatT getIncrement() const {
      return grid_increment_;
    }

    Vector3 getOffset() const {
      return grid_offset_;
    }

    Vector3 getStepX() const {
      return Vector3(grid_increment_, 0, 0);
    }

    Vector3 getStepY() const {
      return Vector3(0, grid_increment_, 0);
    }

    Vector3 getStepZ() const {
      return Vector3(0, 0, grid_increment_);
    }

    Vector3 getStepXYZ() const {
      return Vector3(grid_increment_, grid_increment_, grid_increment_);
    }

    Vector3 getMinIntersectingVoxel(const BoundingBox3D<FloatT>& bbox) const {
      Vector3 min_voxel = bbox.getMinimum() + getStepXYZ() / 2;
      using std::floor;
      min_voxel = grid_offset_ + getIncrement() * floor((min_voxel - grid_offset_).array() / getIncrement()).matrix();
      return min_voxel;
    }

    Vector3 getMaxIntersectingVoxel(const BoundingBox3D<FloatT>& bbox) const {
      Vector3 max_voxel = bbox.getMaximum() - getStepXYZ() / 2;
      using std::ceil;
      max_voxel = grid_offset_ + getIncrement() * ceil((max_voxel - grid_offset_).array() / getIncrement()).matrix();
      return max_voxel;
    }

private:
    FloatT grid_increment_;
    Vector3 grid_offset_;
};

using ContinuousGrid3DUtilsf = ContinuousGrid3DUtils<float>;
using ContinuousGrid3DUtilsd = ContinuousGrid3DUtils<double>;

template <typename CoordT, typename ValueT>
class ContinuousGrid3D : public Grid3D<ValueT> {
public:
  using Base = Grid3D<ValueT>;
  using Base::operator();

  using size_t = std::size_t;
  using CoordType = CoordT;
  using ValueType = ValueT;
  using BoundingBoxType = BoundingBox3D<CoordType>;
  using Vector3s = Eigen::Vector3s;
  USE_FIXED_EIGEN_TYPES(CoordType);

  ContinuousGrid3D() {}

  ContinuousGrid3D(const BoundingBoxType& grid_bbox, const size_t dim_x, const size_t dim_y, const size_t dim_z)
  : Base(dim_x, dim_y, dim_z), grid_bbox_(grid_bbox) {
    const Vector3 dim_vec = this->getDimensions().template cast<CoordType>();
    grid_increment_ = grid_bbox_.getExtent().array() / dim_vec.array();
  }

  const BoundingBoxType& getGridBbox() const {
    return grid_bbox_;
  }

  const Vector3& getGridOrigin() const {
    return grid_bbox_.getMinimum();
  }

  const Vector3& getGridIncrement() const {
    return grid_increment_;
  }

  bool isInsideGrid(const Vector3& xyz) const {
    return grid_bbox_.isInside(xyz);
  }

  Vector3s getGridIndices(const Vector3& xyz) const {
    Vector3 indices_float = (xyz - getGridOrigin()).cwiseQuotient(getGridIncrement());
    Vector3s indices(indices_float.array().floor().template cast<size_t>());
    for (int i = 0; i < indices.rows(); ++i) {
      if (indices(i) < 0 && xyz(i) >= grid_bbox_.getMinimum(i)) {
        indices(i) = 0;
      }
      if (indices(i) >= this->getDimensions()(i) && xyz(i) <= grid_bbox_.getMaximum(i)) {
        indices(i) = this->getDimensions()(i) - 1;
      }
    }
    return indices;
  }

  Vector3 getGridPosition(const Vector3s& indices) const {
    return getGridOrigin() + getGridIncrement() * indices.cast<CoordType>();
  }

  Vector3 getGridPosition(const size_t ix, const size_t iy, const size_t iz) const {
    return getGridPosition(Vector3s(ix, iy, iz));
  }

  const ValueType& operator()(const Vector3& xyz) const {
    return Base::operator()(getGridIndices(xyz));
  }

  ValueType& operator()(const Vector3& xyz) {
    return Base::operator()(getGridIndices(xyz));
  }

private:
  friend class boost::serialization::access;

  template <typename Archive>
  void serialize(Archive& ar, unsigned int version) {
    ar & boost::serialization::base_object<Grid3D>(*this);
    ar & const_cast<BoundingBoxType&>(grid_bbox_);
    ar & const_cast<Vector3&>(grid_increment_);
  }

  BoundingBoxType grid_bbox_;
  Vector3 grid_increment_;
};

#if __GNUC__ && !__CUDACC__
  #pragma GCC pop_options
#endif

}
