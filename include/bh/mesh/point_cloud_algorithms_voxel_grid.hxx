//==================================================
// point_cloud_algorithms_voxel_grid.hxx
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 28.09.17
//==================================================

// Adapted from Point Cloud Library:
//
//Software License Agreement (BSD License)
//
//Point Cloud Library (PCL) - www.pointclouds.org
//Copyright (c) 2009-2012, Willow Garage, Inc.
//Copyright (c) 2012-, Open Perception, Inc.
//Copyright (c) XXX, respective authors.
//
//All rights reserved.
//
//Redistribution and use in source and binary forms, with or without
//        modification, are permitted provided that the following conditions
//are met:
//
//* Redistributions of source code must retain the above copyright
//        notice, this list of conditions and the following disclaimer.
//* Redistributions in binary form must reproduce the above
//copyright notice, this list of conditions and the following
//        disclaimer in the documentation and/or other materials provided
//        with the distribution.
//* Neither the name of the copyright holder(s) nor the names of its
//        contributors may be used to endorse or promote products derived
//        from this software without specific prior written permission.
//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//        LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//        FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//        COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//        INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//                                                                  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//        CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//        POSSIBILITY OF SUCH DAMAGE.

namespace {

template<typename IndexT>
struct PointCloudIndexIdx {
  IndexT idx;
  IndexT point_cloud_index;

  PointCloudIndexIdx(const IndexT idx, const IndexT point_cloud_index)
          : idx(idx), point_cloud_index(point_cloud_index) {
  }

  bool operator<(const PointCloudIndexIdx &other) const {
    return idx < other.idx;
  }
};

template <typename T, typename IndexIterator>
inline T computeAverage(const std::vector<T>& items, IndexIterator first, IndexIterator last) {
  T average = T::Zero();
  for (IndexIterator it = first; it != last; ++it) {
    average += items[it->point_cloud_index];
  }
  average /= (last - first);
  return average;
};

template <typename VectorT, typename IndexVectorType, typename IndexT = std::size_t>
inline IndexT computeVoxelIndex(const VectorT& point, const VectorT& inverse_leaf_size,
                                const IndexVectorType& min_b, const IndexVectorType& divb_mul) {
  const IndexT ijk0 = static_cast<IndexT>(floor(point(0) * inverse_leaf_size[0]) - static_cast<typename VectorT::Scalar>(min_b[0]));
  const IndexT ijk1 = static_cast<IndexT>(floor(point(1) * inverse_leaf_size[1]) - static_cast<typename VectorT::Scalar>(min_b[1]));
  const IndexT ijk2 = static_cast<IndexT>(floor(point(2) * inverse_leaf_size[2]) - static_cast<typename VectorT::Scalar>(min_b[2]));

  // Compute the centroid leaf index
  const IndexT idx = ijk0 * divb_mul[0] + ijk1 * divb_mul[1] + ijk2 * divb_mul[2];
  return idx;
}

}

namespace bh {

template<typename FloatT, typename IndexT>
PointCloud <FloatT> PointCloudAlgorithms::filterOnVoxelGrid(
        const PointCloud <FloatT> &input,
        const typename bh::EigenTypes<FloatT>::Vector3 &voxel_size,
        const IndexT min_points_per_voxel) {
  static_assert(std::is_signed<IndexT>::value, "Index type for filterOnVoxelGrid has to be signed.");

  BH_USE_FIXED_EIGEN_TYPES(FloatT);
  using PointCloudIndexIdxType = PointCloudIndexIdx<IndexT>;
  using IndexVectorType = Eigen::Matrix<IndexT, 3, 1>;

  Vector3 inverse_leaf_size(1 / voxel_size(0),
                            1 / voxel_size(1),
                            1 / voxel_size(2));

  // Get the minimum and maximum dimensions
  const BoundingBox3D<FloatT> bbox = getBoundingBox3D(input);

  // Check that the leaf size is not too small, given the size of the data
  const IndexT dx = static_cast<IndexT>((bbox.getMaximum()[0] - bbox.getMinimum()[0]) * inverse_leaf_size(0)) + 1;
  const IndexT dy = static_cast<IndexT>((bbox.getMaximum()[1] - bbox.getMinimum()[1]) * inverse_leaf_size(1)) + 1;
  const IndexT dz = static_cast<IndexT>((bbox.getMaximum()[2] - bbox.getMinimum()[2]) * inverse_leaf_size(2)) + 1;

  if ((dx * dy * dz) > std::numeric_limits<IndexT>::max()) {
    throw BH_EXCEPTION("Leaf size is too small for the input dataset. Integer indices would overflow.");
    return input;
  }

  IndexVectorType min_b, max_b, div_b, divb_mul;
  // Compute the minimum and maximum bounding box values
  min_b[0] = static_cast<IndexT>(floor(bbox.getMinimum()[0] * inverse_leaf_size(0)));
  max_b[0] = static_cast<IndexT>(floor(bbox.getMaximum()[0] * inverse_leaf_size(0)));
  min_b[1] = static_cast<IndexT>(floor(bbox.getMinimum()[1] * inverse_leaf_size(1)));
  max_b[1] = static_cast<IndexT>(floor(bbox.getMaximum()[1] * inverse_leaf_size(1)));
  min_b[2] = static_cast<IndexT>(floor(bbox.getMinimum()[2] * inverse_leaf_size(2)));
  max_b[2] = static_cast<IndexT>(floor(bbox.getMaximum()[2] * inverse_leaf_size(2)));

  // Compute the number of divisions needed along all axis
  div_b = max_b - min_b + IndexVectorType::Ones();

  // Set up the division multiplier
  divb_mul = IndexVectorType(1, div_b[0], div_b[0] * div_b[1]);

  // Storage for mapping leaf and pointcloud indexes
  std::vector<PointCloudIndexIdxType> index_vector;
  index_vector.reserve(input.vertices().size());

  // First pass: go over all points and insert them into the index_vector vector
  // with calculated idx. Points with the same idx value will contribute to the
  // same point of resulting CloudPoint
  for (IndexT i = 0; i < input.vertices().size(); ++i) {
    const Vector3 &point = input.vertices()[i];
    const IndexT idx = computeVoxelIndex(point, inverse_leaf_size, min_b, divb_mul);
    index_vector.push_back(PointCloudIndexIdxType(idx, i));
  }

  // Second pass: sort the index_vector vector using value representing target cell as index
  // in effect all points belonging to the same output cell will be next to each other
  std::sort(index_vector.begin(), index_vector.end(), std::less<PointCloudIndexIdxType>());

  // Third pass: count output cells
  // we need to skip all the same, adjacenent idx values
  IndexT total = 0;
  IndexT index = 0;
  // first_and_last_indices_vector[i] represents the index in index_vector of the first point in
  // index_vector belonging to the voxel which corresponds to the i-th output point,
  // and of the first point not belonging to it.
  std::vector<std::pair<IndexT, IndexT>> first_and_last_indices_vector;
  // Worst case size
  first_and_last_indices_vector.reserve(index_vector.size());
  while (index < index_vector.size()) {
    IndexT i = index + 1;
    while (i < index_vector.size() && index_vector[i].idx == index_vector[index].idx) {
      ++i;
    }
    if (i - index >= min_points_per_voxel) {
      ++total;
      first_and_last_indices_vector.push_back(std::pair<IndexT, IndexT>(index, i));
    }
    index = i;
  }

  // Fourth pass: compute centroids, insert them into their final position
  PointCloud<FloatT> output;
  output.vertices().resize(total);
  if (input.hasColors()) {
    output.colors().resize(total);
  }
  if (input.hasNormals()) {
    output.normals().resize(total);
  }

  index = 0;
  for (IndexT cp = 0; cp < first_and_last_indices_vector.size(); ++cp) {
    // calculate centroid - sum values from all input points, that have the same idx value in index_vector array
    const IndexT first_index = first_and_last_indices_vector[cp].first;
    const IndexT last_index = first_and_last_indices_vector[cp].second;

    const Vector3 centroid = computeAverage<Vector3>(
            input.vertices(), index_vector.begin() + first_index, index_vector.begin() + last_index);
    output.vertices()[index] = centroid;

    if (input.hasColors()) {
      const Color4<FloatT> color = computeAverage<Color4<FloatT>>(
              input.colors(), index_vector.begin() + first_index, index_vector.begin() + last_index);
      output.colors()[index] = color;
    }

    if (input.hasNormals()) {
      const Vector3 normal = computeAverage<Vector3>(
              input.normals(), index_vector.begin() + first_index, index_vector.begin() + last_index);
      output.normals()[index] = normal;
    }

    ++index;
  }

  return output;
};

}
