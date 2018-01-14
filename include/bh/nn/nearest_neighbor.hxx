//==================================================
// nearest_neighbor.hxx
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 27.09.17
//==================================================

namespace bh {

template <typename FloatT, std::size_t dimension, typename DistanceT, typename IndexT>
NearestNeighbor<FloatT, dimension, DistanceT, IndexT>::NearestNeighbor(const std::size_t max_leaf_size)
        : index_(dimension, *this, nanoflann::KDTreeSingleIndexAdaptorParams(max_leaf_size)),
          last_rebuild_size_(0) {
}

template <typename FloatT, std::size_t dimension, typename DistanceT, typename IndexT>
template <typename Iterator>
NearestNeighbor<FloatT, dimension, DistanceT, IndexT>::NearestNeighbor(Iterator first, Iterator last, const std::size_t max_leaf_size)
        : points_(first, last),
          index_(dimension, *this, nanoflann::KDTreeSingleIndexAdaptorParams(max_leaf_size)),
          last_rebuild_size_(0) {
  rebuildIndex();
}

template <typename FloatT, std::size_t dimension, typename DistanceT, typename IndexT>
NearestNeighbor<FloatT, dimension, DistanceT, IndexT>::~NearestNeighbor() {
};

template <typename FloatT, std::size_t dimension, typename DistanceT, typename IndexT>
bool NearestNeighbor<FloatT, dimension, DistanceT, IndexT>::empty() const {
  return points_.empty();
};

template <typename FloatT, std::size_t dimension, typename DistanceT, typename IndexT>
auto NearestNeighbor<FloatT, dimension, DistanceT, IndexT>::numPoints() const -> IndexType {
  return points_.size();
};

template <typename FloatT, std::size_t dimension, typename DistanceT, typename IndexT>
void NearestNeighbor<FloatT, dimension, DistanceT, IndexT>::clear() {
  points_.clear();
  rebuildIndex();
};

template <typename FloatT, std::size_t dimension, typename DistanceT, typename IndexT>
void NearestNeighbor<FloatT, dimension, DistanceT, IndexT>::rebuildIndex() {
  index_.buildIndex();
  last_rebuild_size_ = numPoints();
};

template <typename FloatT, std::size_t dimension, typename DistanceT, typename IndexT>
template <typename Iterator>
void NearestNeighbor<FloatT, dimension, DistanceT, IndexT>::addPoints(Iterator begin, Iterator end,
                                                                      const FloatType rebuild_threshold) {
  std::copy(begin, end, std::back_inserter(points_));
  if (points_.size() / FloatType(last_rebuild_size_) >= rebuild_threshold) {
    rebuildIndex();
  }
}

template <typename FloatT, std::size_t dimension, typename DistanceT, typename IndexT>
void NearestNeighbor<FloatT, dimension, DistanceT, IndexT>::addPoint(const Point& point,
                                                                     const FloatType rebuild_threshold) {
  points_.push_back(point);
  if (points_.size() / FloatType(last_rebuild_size_) >= rebuild_threshold) {
    rebuildIndex();
  }
}

template <typename FloatT, std::size_t dimension, typename DistanceT, typename IndexT>
auto NearestNeighbor<FloatT, dimension, DistanceT, IndexT>::getPoint(IndexType point_id) const -> const Point& {
  BH_ASSERT(point_id >= 0 && point_id < points_.size());
  return points_[point_id];
}

template <typename FloatT, std::size_t dimension, typename DistanceT, typename IndexT>
auto NearestNeighbor<FloatT, dimension, DistanceT, IndexT>::radiusSearchMaxResults(
        const Point& query, const FloatType radius, const IndexType max_results,
        const bool sorted, const FloatType eps) -> RadiusMaxNumResultSet {
  RadiusMaxNumResultSet result_set(radius, max_results);
  nanoflann::SearchParams search_params;
  search_params.eps = eps;
  search_params.sorted = sorted;
  index_.radiusSearchCustomCallback(query.data(), result_set, search_params);
  return result_set;
};

//
// Adapter interface for nanoflann
//

template <typename FloatT, std::size_t dimension, typename DistanceT, typename IndexT>
auto NearestNeighbor<FloatT, dimension, DistanceT, IndexT>::derived() const -> const SelfType& {
  return *this;
}

template <typename FloatT, std::size_t dimension, typename DistanceT, typename IndexT>
auto NearestNeighbor<FloatT, dimension, DistanceT, IndexT>::derived() -> SelfType& {
  return *this;
}

// Must return the number of data points
template <typename FloatT, std::size_t dimension, typename DistanceT, typename IndexT>
auto NearestNeighbor<FloatT, dimension, DistanceT, IndexT>::kdtree_get_point_count() const -> IndexType {
  return points_.size();
}

// Returns the dim'th component of the idx'th point in the class:
template <typename FloatT, std::size_t dimension, typename DistanceT, typename IndexT>
auto NearestNeighbor<FloatT, dimension, DistanceT, IndexT>::kdtree_get_pt(const IndexType idx, int dim) const
-> FloatType{
  return points_[idx][dim];
}

// Optional bounding-box computation: return false to default to a standard bbox computation loop.
//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
template <typename FloatT, std::size_t dimension, typename DistanceT, typename IndexT>
template <class BBOX>
bool NearestNeighbor<FloatT, dimension, DistanceT, IndexT>::kdtree_get_bbox(BBOX & /*bb*/) const {
  return false;
}

}
