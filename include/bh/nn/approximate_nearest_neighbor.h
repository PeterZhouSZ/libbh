/*
 * approximate_nearest_neighbor.h
 *
 *  Created on: Dec 29, 2016
 *      Author: bhepp
 */
#pragma once

#include <algorithm>
#include <forward_list>
#include <flann/flann.hpp>
#include "../common.h"
#include "../eigen.h"

namespace bh {

template <typename FloatT, std::size_t dimension, typename NormT = flann::L2<FloatT>>
class ApproximateNearestNeighbor {
public:
  using FloatType = FloatT;
  using Point = Eigen::Matrix<FloatType, dimension, 1>;
  using PointNotAligned = Eigen::Matrix<FloatType, dimension, 1, Eigen::DontAlign>;
  using IndexType = std::size_t;
  using DistanceType = typename NormT::ResultType;
  using FlannElementType = typename NormT::ElementType;
  using FlannPointType = std::array<FloatT, dimension>;
  using FlannMatrix = flann::Matrix<FlannElementType>;
  using FlannIndexMatrix = flann::Matrix<std::size_t>;
  using FlannDistanceMatrix = flann::Matrix<DistanceType>;
  using EigenMatrix = Eigen::Matrix<FloatType, Eigen::Dynamic, Eigen::Dynamic>;

  ApproximateNearestNeighbor(std::size_t num_trees = 4, NormT norm = NormT());

  ApproximateNearestNeighbor(const flann::IndexParams& index_params, NormT norm = NormT());

  ~ApproximateNearestNeighbor();

  static flann::SearchParams getDefaultSearchParams();

  void clear();

  void rebuildIndex();

  void setSearchParams(const flann::SearchParams& params);

  flann::SearchParams& searchParams() { return search_params_; }

  const flann::SearchParams& searchParams() const { return search_params_; }

  void initIndexNxD(const EigenMatrix& points);

  void initIndexDxN(const EigenMatrix& points);

  // Initialize from a container of Eigen column vectors
  template <typename Iterator>
  void initIndex(Iterator begin, Iterator end);

  void addPointsNxD(const EigenMatrix& points, FloatType rebuild_threshold = 2);

  void addPointsDxN(const EigenMatrix& points, FloatType rebuild_threshold = 2);

  template <typename Iterator>
  void addPoints(Iterator begin, Iterator end, FloatType rebuild_threshold = 2);

  void addPoint(const Point& point, FloatType rebuild_threshold = 2);

  Point getPoint(std::size_t point_id) const;

  struct SingleResult {
    std::vector<IndexType> indices;
    std::vector<DistanceType> distances;
  };

  struct Result {
    std::vector<std::vector<IndexType>> indices;
    std::vector<std::vector<DistanceType>> distances;
  };

  void knnSearch(const Point& point, std::size_t knn,
                 std::vector<IndexType>* indices, std::vector<DistanceType>* distances) const;

  SingleResult knnSearch(const Point& point, std::size_t knn) const;

  // Expects a points matrix with dimension NxD.
  Result knnSearchNxD(const EigenMatrix& points, std::size_t knn) const;

  // Expects a points matrix with dimension DxN.
  Result knnSearchDxN(const EigenMatrix& points, std::size_t knn) const;

  template <typename Iterator>
  Result knnSearch(Iterator begin, Iterator end, std::size_t knn) const;

  void radiusSearch(const Point& point, FloatType radius,
                    std::vector<IndexType>* indices, std::vector<DistanceType>* distances,
                    std::size_t max_results=(std::size_t)-1) const;

  SingleResult radiusSearch(const Point& point, FloatType radius, std::size_t max_results=(std::size_t)-1) const;

  std::size_t radiusSearchCount(const Point& point, FloatType radius) const;

  // Expects a points matrix with dimension NxD.
  Result radiusSearchNxD(const EigenMatrix& points, FloatType radius, std::size_t max_results=(std::size_t)-1) const;

  // Expects a points matrix with dimension DxN.
  Result radiusSearchDxN(const EigenMatrix& points, FloatType radius, std::size_t max_results=(std::size_t)-1) const;

  template <typename Iterator>
  Result radiusSearch(Iterator begin, Iterator end, FloatType radius, std::size_t max_results=(std::size_t)-1) const;

  /// Only for testing against knnSearch
  void knnSearchExact(const Point& point, std::size_t knn,
                      std::vector<IndexType>* indices, std::vector<DistanceType>* distances) const;

  /// Only for testing against radiusSearch
  void radiusSearchExact(const Point& point, FloatType radius,
                         std::vector<IndexType>* indices, std::vector<DistanceType>* distances,
                         std::size_t max_results=(std::size_t)-1) const;

  bool empty() const;

  std::size_t numPoints() const;

  const flann::Index<NormT>& getFlannIndex() const;

  flann::Index<NormT>& getFlannIndex();

private:
  FlannPointType& addPointInternal(const Point& point);

  template <typename Iterator>
  FlannPointType& addPointsInternal(Iterator begin, Iterator end);

  std::size_t numPointsInternal() const;

  bool initialized_;
  flann::IndexParams index_params_;
  NormT norm_;
  flann::Index<NormT> index_;
  mutable flann::SearchParams search_params_;

  // TODO: Reorganize with better data structures (i.e. using Eigen vectors)
  // Need two arrays here because points for initialization need to be dense
  std::vector<FlannPointType> init_points_;
  std::forward_list<FlannPointType> points_;
  std::forward_list<std::vector<FlannPointType>> point_arrays_;
  size_t points_size_;
};

}

#include "approximate_nearest_neighbor.hxx"
