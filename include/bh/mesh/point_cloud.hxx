//==================================================
// point_cloud.hxx
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 27.09.17
//==================================================

#define _USE_MATH_DEFINES
#include <cmath>
#include <array>
#include <iostream>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <set>

namespace bh {

template <typename FloatT>
PointCloud<FloatT>::PointCloud() {}

template <typename FloatT>
PointCloud<FloatT>::PointCloud(
        const std::vector<Vector3>& vertices)
        : vertices_(vertices) {}

template <typename FloatT>
PointCloud<FloatT>::PointCloud(
        const std::vector<Vector3>& vertices,
        const std::vector<Vector3>& normals)
        : vertices_(vertices),
          normals_(normals) {}

template <typename FloatT>
PointCloud<FloatT>::PointCloud(
        const std::vector<Vector3>& vertices,
        const std::vector<ColorType>& colors)
        : vertices_(vertices),
          colors_(colors) {}

template <typename FloatT>
PointCloud<FloatT>::PointCloud(
        const std::vector<Vector3>& vertices,
        const std::vector<Vector3>& normals,
        const std::vector<ColorType>& colors)
        : vertices_(vertices),
          normals_(normals),
          colors_(colors) {}

template <typename FloatT>
PointCloud<FloatT>::PointCloud(
        const std::vector<Vector3>& vertices,
        const std::vector<Vector3>& normals,
        const std::vector<ColorType>& colors,
        const std::vector<Vector2>& texture_uvs)
        : vertices_(vertices),
          normals_(normals),
          colors_(colors),
          texture_uvs_(texture_uvs) {}

template <typename FloatT>
PointCloud<FloatT>::~PointCloud() {}

template <typename FloatT>
auto PointCloud<FloatT>::vertices() const -> const std::vector<Vector3>& {
  return vertices_;
}

template <typename FloatT>
auto PointCloud<FloatT>::vertices() -> std::vector<Vector3>& {
  return vertices_;
}

template <typename FloatT>
bool PointCloud<FloatT>::hasNormals() const {
  return !normals_.empty();
}

template <typename FloatT>
auto PointCloud<FloatT>::normals() const -> const std::vector<Vector3>& {
  return normals_;
}

template <typename FloatT>
auto PointCloud<FloatT>::normals() -> std::vector<Vector3>& {
  return normals_;
}

template <typename FloatT>
bool PointCloud<FloatT>::hasTextureUVs() const {
  return !texture_uvs_.empty();
}

template <typename FloatT>
auto PointCloud<FloatT>::textureUVs() const -> const std::vector<Vector2>& {
  return texture_uvs_;
}

template <typename FloatT>
auto PointCloud<FloatT>::textureUVs() -> std::vector<Vector2>& {
  return texture_uvs_;
}

template <typename FloatT>
bool PointCloud<FloatT>::hasColors() const {
  return !colors_.empty();
}

template <typename FloatT>
auto PointCloud<FloatT>::colors() const -> const std::vector<ColorType>& {
  return colors_;
}

template <typename FloatT>
auto PointCloud<FloatT>::colors() -> std::vector<ColorType>& {
  return colors_;
}

template <typename FloatT>
template <typename PointIterator>
auto PointCloudFactory<FloatT>::createFromPoints(
        const PointIterator first, const PointIterator last, const bool remove_duplicates) -> PointCloudType {
  PointCloudType mesh;
  if (remove_duplicates) {
    std::unordered_set<Vector3> vertex_index_set;
    for (PointIterator it = first; it != last; ++it) {
      const Vector3& v = *it;
      const auto set_it = vertex_index_set.find(v);
      if (set_it == vertex_index_set.end()) {
        mesh.vertices().push_back(v);
        vertex_index_set.emplace(v);
      }
    }
    mesh.vertices().shrink_to_fit();
  }
  else {
    mesh.vertices().resize(last - first);
    std::copy(first, last, mesh.vertices().begin());
  }
  return mesh;
}

}
