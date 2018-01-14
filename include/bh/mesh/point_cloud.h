//==================================================
// point_cloud.h.h
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 27.09.17
//==================================================
#pragma once

#include <memory>
#include <vector>
#include <Eigen/Dense>

#include "../memory.h"
#include "../eigen.h"
#include "../color.h"
#include "../math/geometry.h"

namespace bh {

template <typename FloatT>
class PointCloudFactory;

template <typename FloatT>
class PointCloud {
public:
  using FloatType = FloatT;
  using ColorType = Color4<FloatT>;
  USE_FIXED_EIGEN_TYPES(FloatType);

  explicit PointCloud();

  explicit PointCloud(const std::vector<Vector3>& vertices);

  explicit PointCloud(const std::vector<Vector3>& vertices,
                      const std::vector<Vector3>& normals);

  explicit PointCloud(const std::vector<Vector3>& vertices,
                      const std::vector<ColorType>& colors);

  explicit PointCloud(const std::vector<Vector3>& vertices,
                      const std::vector<Vector3>& normals,
                      const std::vector<ColorType>& colors);

  explicit PointCloud(const std::vector<Vector3>& vertices,
                      const std::vector<Vector3>& normals,
                      const std::vector<ColorType>& colors,
                      const std::vector<Vector2>& texture_uvs);

  PointCloud(const PointCloud& other) = default;

  PointCloud(PointCloud& other) = default;

  virtual ~PointCloud();

  PointCloud& operator=(const PointCloud& other) = default;

  PointCloud& operator=(PointCloud&& other) = default;

  const std::vector<Vector3>& vertices() const;

  std::vector<Vector3>& vertices();

  bool hasNormals() const;

  const std::vector<Vector3>& normals() const;

  std::vector<Vector3>& normals();

  bool hasTextureUVs() const;

  const std::vector<Vector2>& textureUVs() const;

  std::vector<Vector2>& textureUVs();

  bool hasColors() const;

  const std::vector<ColorType>& colors() const;

  std::vector<ColorType>& colors();

private:
  friend class PointCloudFactory<FloatT>;

  std::vector<Vector3> vertices_;
  std::vector<Vector3> normals_;
  std::vector<Vector2> texture_uvs_;
  std::vector<ColorType> colors_;
};

template <typename FloatT>
class PointCloudFactory {
public:
  using FloatType = FloatT;
  using ColorType = Color4<FloatT>;
  USE_FIXED_EIGEN_TYPES(FloatType);
  using PointCloudType = PointCloud<FloatType>;

  PointCloudFactory() = delete;
  PointCloudFactory(const PointCloudFactory&) = delete;
  PointCloudFactory& operator=(const PointCloudFactory&) = delete;

  /// Create a point cloud from a range of points of type Vector3
  template <typename PointIterator>
  static PointCloudType createFromPoints(const PointIterator first, const PointIterator last,
                                         const bool remove_duplicates = false);
};

} /* namespace bh */

#include "point_cloud.hxx"
