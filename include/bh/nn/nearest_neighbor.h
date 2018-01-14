//==================================================
// nearest_neighbor.h.h
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 27.09.17
//==================================================
#pragma once

#include <algorithm>
#include <forward_list>
#include "../contrib/nanoflann/nanoflann.hpp"
#include "../common.h"
#include "../eigen.h"

namespace bh {

template <typename FloatT, std::size_t dimension, typename DistanceT = nanoflann::metric_L2,
        typename IndexT = std::size_t>
class NearestNeighbor {
public:
  using FloatType = FloatT;
  using IndexType = IndexT;
  using DistanceType = DistanceT;
  const std::size_t DIMENSION = dimension;

  using Point = Eigen::Matrix<FloatType, dimension, 1>;

  using SelfType = NearestNeighbor<FloatT, dimension, DistanceT, IndexT>;
  using MetricType = typename DistanceType::template traits<FloatType, SelfType>::distance_t;
  using NanoflannType = nanoflann::KDTreeSingleIndexAdaptor<MetricType, SelfType, dimension, IndexType>;

  class RadiusMaxNumResultSet;

  NearestNeighbor(const std::size_t max_leaf_size = 10);

  template <typename Iterator>
  NearestNeighbor(Iterator first, Iterator last, const std::size_t max_leaf_size = 10);

  ~NearestNeighbor();

  bool empty() const;

  IndexType numPoints() const;

  void clear();

  void rebuildIndex();

  template <typename Iterator>
  void addPoints(Iterator begin, Iterator end, const FloatType rebuild_threshold = 2);

  void addPoint(const Point& point, const FloatType rebuild_threshold = 2);

  const Point& getPoint(IndexType point_id) const;

  RadiusMaxNumResultSet radiusSearchMaxResults(
          const Point& query, const FloatType radius, const IndexType max_results,
          const bool sorted = false, const FloatType eps = 0);

  //
  // Adapter interface for nanoflann
  //

  const SelfType & derived() const;

  SelfType & derived();

  // Must return the number of data points
  inline IndexType kdtree_get_point_count() const;

  // Returns the dim'th component of the idx'th point in the class:
  inline FloatType kdtree_get_pt(const IndexType idx, int dim) const;

  // Optional bounding-box computation: return false to default to a standard bbox computation loop.
  //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
  //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
  template <class BBOX>
  bool kdtree_get_bbox(BBOX& /*bb*/) const;

  class RadiusMaxNumResultSet {
  public:
    const FloatType radius;
    const IndexType max_results;

    std::vector<std::pair<IndexType, FloatType>> indices_dists_;

    RadiusMaxNumResultSet(FloatType radius_, IndexType max_results_)
            : radius(radius_), max_results(max_results_) {
      init();
    }

    RadiusMaxNumResultSet(FloatType radius_, IndexType max_results_,
                          std::vector<std::pair<IndexType,DistanceType> > &indices_dists)
            : radius(radius_), max_results(max_results_), indices_dists_(indices_dists) {
      init();
    }

    void init() {
      clear();
    }

    void clear() {
      indices_dists_.clear();
    }

    size_t size() const {
      return indices_dists_.size();
    }

    bool full() const {
      return true;
    }

    /**
     * Called during search to add an element matching the criteria.
     * @return true if the search should be continued, false if the results are sufficient
     */
    bool addPoint(FloatType dist, IndexType index)  {
      if (dist < radius) {
        indices_dists_.push_back(std::make_pair(index, dist));
        if (indices_dists_.size() >= max_results) {
          return false;
        }
      }
      return true;
    }

    FloatType worstDist() const {
      return radius;
    }

    /**
     * Find the worst result (furtherest neighbor) without copying or sorting
     * Pre-conditions: size() > 0
     */
    std::pair<IndexType, FloatType> worstItem() const {
      if (indices_dists_.empty()) {
        throw std::runtime_error("Cannot invoke RadiusResultSet::worst_item() on an empty list of results.");
      }
      typedef typename std::vector<std::pair<IndexType, FloatType> >::const_iterator DistIt;
      DistIt it = std::max_element(indices_dists_.begin(), indices_dists_.end(), nanoflann::IndexDist_Sorter());
      return *it;
    }

    std::pair<IndexType, FloatType> getItem(const IndexType index) const {
      return indices_dists_[index];
    };

    IndexType getItemIndex(const IndexType index) const {
      return indices_dists_[index].first;
    };

    FloatType getItemDistance(const IndexType index) const {
      return indices_dists_[index].second;
    };
  };

private:
  // The order is important. points_ needs to be initialized in the constructor before index_.
  std::vector<Point> points_;
  NanoflannType index_;
  size_t last_rebuild_size_;
};

}

#include "nearest_neighbor.hxx"
