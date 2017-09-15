//==================================================
// approximate_nearest_neighbor.hxx.cpp
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 21.03.17

namespace bh {

//#pragma GCC push_options
//#pragma GCC optimize("O0")

template<typename FloatT, std::size_t dimension, typename NormT>
ApproximateNearestNeighbor<FloatT, dimension, NormT>::ApproximateNearestNeighbor(
        std::size_t num_trees, NormT norm)
        : initialized_(false), index_params_(flann::KDTreeIndexParams(num_trees)),
          norm_(norm), index_(index_params_, norm_),
          points_size_(0) {
  setSearchParams(getDefaultSearchParams());
}

template<typename FloatT, std::size_t dimension, typename NormT>
ApproximateNearestNeighbor<FloatT, dimension, NormT>::ApproximateNearestNeighbor(
        const flann::IndexParams &index_params, NormT norm)
        : initialized_(false), index_params_(index_params),
          norm_(norm), index_(index_params, norm),
          points_size_(0) {
  setSearchParams(getDefaultSearchParams());
}

template<typename FloatT, std::size_t dimension, typename NormT>
ApproximateNearestNeighbor<FloatT, dimension, NormT>::~ApproximateNearestNeighbor() {
  clear();
}

template<typename FloatT, std::size_t dimension, typename NormT>
auto ApproximateNearestNeighbor<FloatT, dimension, NormT>::getDefaultSearchParams()
-> flann::SearchParams {
  flann::SearchParams params;
  params.checks = 128;
  params.eps = 0;
  params.sorted = true;
  params.max_neighbors = -1;
  params.use_heap = flann::FLANN_Undefined;
  params.cores = 1;
  params.matrices_in_gpu_ram = false;
  return params;
}

template<typename FloatT, std::size_t dimension, typename NormT>
void ApproximateNearestNeighbor<FloatT, dimension, NormT>::clear() {
  initialized_ = false;
  index_ = flann::Index<NormT>(index_params_, norm_);
  init_points_.clear();
//  for (FlannElementType *point_ptr : points_) {
//    delete[] point_ptr;
//  }
  points_.clear();
  points_size_ = 0;
}

template<typename FloatT, std::size_t dimension, typename NormT>
void ApproximateNearestNeighbor<FloatT, dimension, NormT>::rebuildIndex() {
  index_.buildIndex();
}

template<typename FloatT, std::size_t dimension, typename NormT>
void ApproximateNearestNeighbor<FloatT, dimension, NormT>::setSearchParams(const flann::SearchParams &params) {
  search_params_ = params;
}

template<typename FloatT, std::size_t dimension, typename NormT>
void ApproximateNearestNeighbor<FloatT, dimension, NormT>::initIndexNxD(
        const EigenMatrix &points) {
  const EigenMatrix points_transposed = points.transpose();
  initIndexDxN(points_transposed);
}

template<typename FloatT, std::size_t dimension, typename NormT>
void ApproximateNearestNeighbor<FloatT, dimension, NormT>::initIndexDxN(
        const EigenMatrix &points) {
  BH_ASSERT(points.rows() == dimension);
  // FLANN is not supposed to modify the values
  init_points_.reserve(points.cols());
  for (int i = 0; i < points.cols(); ++i) {
    FlannPointType flann_point;
    for (std::size_t j = 0; j < dimension; ++j) {
      flann_point[j] = points(j, i);
    }
    init_points_.push_back(flann_point);
  }
  FlannMatrix flann_points(&(init_points_.front())[0], init_points_.size(), dimension);
  index_.buildIndex(flann_points);
  initialized_ = true;
}

// Initialize from a container of Eigen column vectors
template<typename FloatT, std::size_t dimension, typename NormT>
template<typename Iterator>
void ApproximateNearestNeighbor<FloatT, dimension, NormT>::initIndex(Iterator begin, Iterator end) {
//  init_points_.reserve((end - begin) * dimension);
//  for (Iterator it = begin; it != end; ++it) {
//    FlannPointType flann_point;
//    for (std::size_t col = 0; col < dimension; ++col) {
//      flann_point[col] = (*it)(col);
//    }
//    init_points_.push_back(flann_point);
////    for (std::size_t col = 0; col < dimension; ++col) {
////      init_points_.push_back((*it)(col));
////    }
//  }
//  FlannMatrix flann_points(&(init_points_.front())[0], init_points_.size(), dimension);
//  index_.buildIndex(flann_points);
//  initialized_ = true;
  std::size_t num_points = end - begin;
  EigenMatrix points(dimension, num_points);
  for (Iterator it = begin; it != end; ++it) {
    points.col(it - begin) = it->transpose();
  }
  initIndexDxN(points);
}

template<typename FloatT, std::size_t dimension, typename NormT>
void ApproximateNearestNeighbor<FloatT, dimension, NormT>::addPointsNxD(
        const EigenMatrix &points, const FloatType rebuild_threshold) {
  if (!initialized_) {
    initIndexNxD(points);
    return;
  }
  const EigenMatrix points_transposed = points.transpose();
  addPointsDxN(points_transposed, rebuild_threshold);
}

template<typename FloatT, std::size_t dimension, typename NormT>
void ApproximateNearestNeighbor<FloatT, dimension, NormT>::addPointsDxN(
        const EigenMatrix &points, const FloatType rebuild_threshold) {
  BH_ASSERT(points.rows() == dimension);
  if (!initialized_) {
    initIndexDxN(points);
    return;
  }

  std::vector<Point> points_vector;
  points_vector.reserve(points.cols());
  for (int i = 0; i < points.cols(); ++i) {
    points_vector.push_back(points.col(i));
  }
  FlannPointType& flann_point = addPointsInternal(points_vector.begin(), points_vector.end());
  FlannMatrix flann_mat(&flann_point[0], points.cols(), dimension);
  index_.addPoints(flann_mat, (float)rebuild_threshold);
  points_size_ += points.cols();
}

template<typename FloatT, std::size_t dimension, typename NormT>
template<typename Iterator>
void ApproximateNearestNeighbor<FloatT, dimension, NormT>::addPoints(
        Iterator begin, Iterator end, const FloatType rebuild_threshold) {
  if (!initialized_) {
    initIndex(begin, end);
    return;
  }
  std::size_t num_points = end - begin;
  EigenMatrix points(dimension, num_points);
  for (Iterator it = begin; it != end; ++it) {
    points.col(it - begin) = it->transpose();
  }
  addPointsDxN(points, rebuild_threshold);
}

//template<typename FloatT, std::size_t dimension, typename NormT>
//template<typename Iterator>
//void ApproximateNearestNeighbor<FloatT, dimension, NormT>::addPoints(
//        Iterator begin, Iterator end, FloatType rebuild_threshold) {
//  for (Iterator it = begin; it != end; ++it) {
//    addPoint(*it, rebuild_threshold);
//  }
//}

template<typename FloatT, std::size_t dimension, typename NormT>
void ApproximateNearestNeighbor<FloatT, dimension, NormT>::addPoint(
        const Point &point, FloatType rebuild_threshold) {
  if (!initialized_) {
    initIndex(&point, &point + 1);
    return;
  }
  FlannPointType& flann_point = addPointInternal(point);
  FlannMatrix flann_mat(&flann_point[0], 1, dimension);
  index_.addPoints(flann_mat, (float)rebuild_threshold);
  ++points_size_;
}

template<typename FloatT, std::size_t dimension, typename NormT>
auto ApproximateNearestNeighbor<FloatT, dimension, NormT>::getPoint(std::size_t point_id) const -> Point {
  // FLANN is not supposed to modify anything
  flann::Index <NormT> &index_const = const_cast<flann::Index <NormT> &>(index_);
  const FlannElementType *flann_point = index_const.getPoint(point_id);
  Point point;
  for (std::size_t i = 0; i < dimension; ++i) {
    point(i) = static_cast<FloatType>(flann_point[i]);
  }
  return point;
}

template<typename FloatT, std::size_t dimension, typename NormT>
void ApproximateNearestNeighbor<FloatT, dimension, NormT>::knnSearch(
        const Point &point, std::size_t knn,
        std::vector <IndexType> *indices, std::vector <DistanceType> *distances) const {
  if (empty()) {
    indices->resize(0);
    distances->resize(0);
    return;
  }
  // FLANN is not supposed to modify the values
  Point &point_nonconst = const_cast<Point &>(point);
  FlannMatrix flann_query(point_nonconst.data(), 1, point_nonconst.rows());
  BH_ASSERT(indices->size() == knn);
  BH_ASSERT(distances->size() == knn);
  FlannIndexMatrix flann_indices(indices->data(), 1, knn);
  FlannMatrix flann_distances(distances->data(), 1, knn);
  int count = index_.knnSearch(flann_query, flann_indices, flann_distances, knn, search_params_);
  BH_ASSERT((std::size_t) count <= knn);
  indices->resize(count);
  distances->resize(count);
}

template<typename FloatT, std::size_t dimension, typename NormT>
auto ApproximateNearestNeighbor<FloatT, dimension, NormT>::knnSearch(
        const Point &point, std::size_t knn) const -> SingleResult {
  if (empty()) {
    return SingleResult();
  }
  // FLANN is not supposed to modify the values
  Point &point_nonconst = const_cast<Point &>(point);
  Result result;
  FlannMatrix flann_query(point_nonconst.data(), 1, point_nonconst.rows());
  // FLANN is calling a const function (error in interface)
  flann::Index<NormT>* index_nonconst = const_cast<flann::Index<NormT>*>(&index_);
  int count = index_nonconst->knnSearch(flann_query, result.indices, result.distances, knn, search_params_);
  BH_ASSERT((std::size_t) count == result.indices[0].size());
  SingleResult single_result;
  single_result.indices = std::move(result.indices.front());
  single_result.distances = std::move(result.distances.front());
  return single_result;
}

template<typename FloatT, std::size_t dimension, typename NormT>
auto ApproximateNearestNeighbor<FloatT, dimension, NormT>::knnSearchNxD(
        const EigenMatrix &points, std::size_t knn) const -> Result {
  if (empty()) {
    return Result();
  }
  const EigenMatrix points_transposed = points.transpose();
  return knnSearchDxN(points_transposed, knn);
}

template<typename FloatT, std::size_t dimension, typename NormT>
auto ApproximateNearestNeighbor<FloatT, dimension, NormT>::knnSearchDxN(
        const EigenMatrix &points, std::size_t knn) const -> Result {
  BH_ASSERT(points.rows() == dimension);
  if (empty()) {
    return Result();
  }
  // FLANN is not supposed to modify the values
  EigenMatrix &points_nonconst = const_cast<EigenMatrix &>(points);
  FlannMatrix flann_queries(points_nonconst.data(), points_nonconst.cols(), points_nonconst.rows());
  Result result;
  // FLANN is calling a const function (error in interface)
  flann::Index<NormT>* index_nonconst = const_cast<flann::Index<NormT>*>(&index_);
  index_nonconst->knnSearch(flann_queries, result.indices, result.distances, knn, search_params_);
  return result;
}

template<typename FloatT, std::size_t dimension, typename NormT>
template<typename Iterator>
auto ApproximateNearestNeighbor<FloatT, dimension, NormT>::knnSearch(
        Iterator begin, Iterator end, std::size_t knn) const -> Result {
  std::size_t num_points = end - begin;
  EigenMatrix points(dimension, num_points);
  for (Iterator it = begin; it != end; ++it) {
    points.col(it - begin) = it->transpose();
  }
  return knnSearchDxN(points, knn);
}

template<typename FloatT, std::size_t dimension, typename NormT>
void ApproximateNearestNeighbor<FloatT, dimension, NormT>::radiusSearch(
        const Point &point, FloatType radius,
        std::vector <IndexType> *indices, std::vector <DistanceType> *distances,
        std::size_t max_results) const {
  if (empty()) {
    indices->resize(0);
    distances->resize(0);
    return;
  }
  const std::size_t prev_max_neighbors = search_params_.max_neighbors;
  search_params_.max_neighbors = (int)max_results;
  // FLANN is not supposed to modify the values
  Point &point_nonconst = const_cast<Point &>(point);
  FlannMatrix flann_query(point_nonconst.data(), 1, point_nonconst.rows());
//  indices->resize(max_results);
//  distances->resize(max_results);
//  FlannIndexMatrix flann_indices(indices->data(), 1, max_results);
//  FlannMatrix flann_distances(distances->data(), 1, max_results);
//  const int count = index_.radiusSearch(flann_query, flann_indices, flann_distances, static_cast<float>(radius),
//                                  search_params_);
//  indices->resize(count);
//  distances->resize(count);

  Result result;
  index_.radiusSearch(flann_query, result.indices, result.distances, static_cast<float>(radius),
                                        search_params_);
  search_params_.max_neighbors = prev_max_neighbors;
  *indices = std::move(result.indices.front());
  *distances = std::move(result.distances.front());
}

template<typename FloatT, std::size_t dimension, typename NormT>
auto ApproximateNearestNeighbor<FloatT, dimension, NormT>::radiusSearch(
        const Point &point, FloatType radius, std::size_t max_results) const -> SingleResult {
  if (empty()) {
    return SingleResult();
  }
  const std::size_t prev_max_neighbors = search_params_.max_neighbors;
  search_params_.max_neighbors = (int)max_results;
  // FLANN is not supposed to modify the values
  Point &point_nonconst = const_cast<Point &>(point);
  Result result;
  FlannMatrix flann_query(point_nonconst.data(), 1, point_nonconst.rows());
  const int count = index_.radiusSearch(flann_query, result.indices, result.distances, static_cast<float>(radius),
                                  search_params_);
  BH_ASSERT((std::size_t) count == result.indices[0].size());
  SingleResult single_result;
  single_result.indices = std::move(result.indices.front());
  single_result.distances = std::move(result.distances.front());
  search_params_.max_neighbors = prev_max_neighbors;
  return single_result;
}

template<typename FloatT, std::size_t dimension, typename NormT>
std::size_t ApproximateNearestNeighbor<FloatT, dimension, NormT>::radiusSearchCount(
        const Point &point, FloatType radius) const {
  if (empty()) {
    return 0;
  }
  const std::size_t prev_max_neighbors = search_params_.max_neighbors;
  search_params_.max_neighbors = 0;
  // FLANN is not supposed to modify the values
  Point &point_nonconst = const_cast<Point &>(point);
  Result result;
  FlannMatrix flann_query(point_nonconst.data(), 1, point_nonconst.rows());
  const int count = index_.radiusSearch(flann_query, result.indices, result.distances, static_cast<float>(radius),
                                  search_params_);
  search_params_.max_neighbors = prev_max_neighbors;
  return count;
}

template<typename FloatT, std::size_t dimension, typename NormT>
auto ApproximateNearestNeighbor<FloatT, dimension, NormT>::radiusSearchNxD(
        const EigenMatrix &points, FloatType radius, std::size_t max_results) const -> Result {
  if (empty()) {
    return Result();
  }
  const EigenMatrix points_transposed = points.transpose();
  return radiusSearchDxN(points_transposed, radius, max_results);
}

template<typename FloatT, std::size_t dimension, typename NormT>
auto ApproximateNearestNeighbor<FloatT, dimension, NormT>::radiusSearchDxN(
        const EigenMatrix &points, FloatType radius, std::size_t max_results) const -> Result {
  BH_ASSERT(points.rows() == dimension);
  if (empty()) {
    return Result();
  }
  const std::size_t prev_max_neighbors = search_params_.max_neighbors;
  search_params_.max_neighbors = (int)max_results;
  // FLANN is not supposed to modify the values
  EigenMatrix &points_nonconst = const_cast<EigenMatrix &>(points);
  FlannMatrix flann_queries(points_nonconst.data(), points_nonconst.cols(), points_nonconst.rows());
  Result result;
  // FLANN is calling a const function (error in interface)
  flann::Index<NormT>* index_nonconst = const_cast<flann::Index<NormT>*>(&index_);
  index_nonconst->radiusSearch(flann_queries, result.indices, result.distances, radius, search_params_);
  search_params_.max_neighbors = prev_max_neighbors;
  return result;
}

template<typename FloatT, std::size_t dimension, typename NormT>
template<typename Iterator>
auto ApproximateNearestNeighbor<FloatT, dimension, NormT>::radiusSearch(
        Iterator begin, Iterator end, FloatType radius, std::size_t max_results) const -> Result {
  std::size_t num_points = end - begin;
  EigenMatrix points(dimension, num_points);
  for (Iterator it = begin; it != end; ++it) {
    points.col(it - begin) = it->transpose();
  }
  return radiusSearchDxN(points, radius, max_results);
}

/// Only for testing against knnSearch
template<typename FloatT, std::size_t dimension, typename NormT>
void ApproximateNearestNeighbor<FloatT, dimension, NormT>::knnSearchExact(
        const Point &point, std::size_t knn,
        std::vector <IndexType> *indices, std::vector <DistanceType> *distances) const {
  using std::swap;
  knn = std::min(knn, numPoints());
  indices->resize(knn);
  distances->resize(knn);
  for (std::size_t i = 0; i < knn; ++i) {
    (*indices)[i] = static_cast<IndexType>(-1);
    (*distances)[i] = std::numeric_limits<DistanceType>::max();
  }

//  for (std::size_t i = 0; i < init_points_.size(); ++i) {
//    const FlannPointType& flann_point = init_points_[i];
//    FloatType dist_square = 0;
//    for (std::size_t col = 0; col < dimension; ++col) {
//      FloatType d = point(col) - static_cast<FloatType>(flann_point[col]);
//      dist_square += d * d;
//    }
//    if (dist_square < (*distances)[distances->size() - 1]) {
//      (*distances)[distances->size() - 1] = dist_square;
//      (*indices)[distances->size() - 1] = i;
//    }
//    // Fix ordering of nearest neighbors
//    for (std::size_t j = distances->size() - 1; j > 0; --j) {
//      if ((*distances)[j - 1] > (*distances)[j]) {
//        swap((*distances)[j - 1], (*distances)[j]);
//        swap((*indices)[j - 1], (*indices)[j]);
//      }
//    }
//  }
//  size_t i = init_points_.size();
//  for (auto it = std::begin(points_); it != std::end(points_); ++it, ++i) {
//    const FlannPointType& flann_point = *it;
//    FloatType dist_square = 0;
//    for (std::size_t col = 0; col < dimension; ++col) {
//      FloatType d = point(col) - static_cast<FloatType>(flann_point[col]);
//      dist_square += d * d;
//    }
//    if (dist_square < (*distances)[distances->size() - 1]) {
//      (*distances)[distances->size() - 1] = dist_square;
//      (*indices)[distances->size() - 1] = i;
//    }
//    // Fix ordering of nearest neighbors
//    for (std::size_t j = distances->size() - 1; j > 0; --j) {
//      if ((*distances)[j - 1] > (*distances)[j]) {
//        swap((*distances)[j - 1], (*distances)[j]);
//        swap((*indices)[j - 1], (*indices)[j]);
//      }
//    }
//  }

  for (size_t i = 0; i < numPoints(); ++i) {
    const Point flann_point = getPoint(i);
    const FloatType dist_square = (point - flann_point).squaredNorm();
    if (dist_square < (*distances)[distances->size() - 1]) {
      (*distances)[distances->size() - 1] = dist_square;
      (*indices)[distances->size() - 1] = i;
    }
    // Fix ordering of nearest neighbors
    for (std::size_t j = distances->size() - 1; j > 0; --j) {
      if ((*distances)[j - 1] > (*distances)[j]) {
        swap((*distances)[j - 1], (*distances)[j]);
        swap((*indices)[j - 1], (*indices)[j]);
      }
    }
  }
}

/// Only for testing against radiusSearch
template<typename FloatT, std::size_t dimension, typename NormT>
void ApproximateNearestNeighbor<FloatT, dimension, NormT>::radiusSearchExact(
        const Point &point, FloatType radius,
        std::vector <IndexType> *indices, std::vector <DistanceType> *distances,
        std::size_t max_results) const {
  indices->resize(0);
  distances->resize(0);
//  for (std::size_t i = 0; i < init_points_.size(); ++i) {
//    const FlannPointType& flann_point = init_points_[i];
//    FloatType dist_square = 0;
//    for (std::size_t col = 0; col < dimension; ++col) {
//      FloatType d = point(col) - static_cast<FloatType>(flann_point[col]);
//      dist_square += d * d;
//    }
//    if (dist_square <= radius) {
//      indices->push_back(i);
//      distances->push_back(dist_square);
//      if (max_results != (size_t)-1 && indices->size() >= max_results) {
//        return;
//      }
//    }
//  }
//  size_t i = init_points_.size();
//  for (auto it = std::begin(points_); it != std::end(points_); ++it) {
//    const FlannPointType& flann_point = *it;
//    FloatType dist_square = 0;
//    for (std::size_t col = 0; col < dimension; ++col) {
//      FloatType d = point(col) - static_cast<FloatType>(flann_point[col]);
//      dist_square += d * d;
//    }
//    if (dist_square <= radius) {
//      indices->push_back(i);
//      distances->push_back(dist_square);
//      if (max_results != (size_t)-1 && indices->size() >= max_results) {
//        return;
//      }
//    }
//  }

  for (size_t i = 0; i < numPoints(); ++i) {
    const Point flann_point = getPoint(i);
    const FloatType dist_square = (point - flann_point).squaredNorm();
    if (dist_square <= radius) {
      indices->push_back(i);
      distances->push_back(dist_square);
      if (max_results != (size_t)-1 && indices->size() >= max_results) {
        break;
      }
    }
  }

  if (search_params_.sorted && !indices->empty()) {
    std::vector<size_t> idx_array;
    for (size_t i = 0; i < indices->size(); ++i) {
      idx_array.push_back(i);
    }
    std::sort(idx_array.begin(), idx_array.end(), [&](const size_t a, const size_t b) {
      return (*distances)[a] < (*distances)[b];
    });
    std::vector<IndexType> indices_copy = *indices;
    std::vector<DistanceType> distances_copy = *distances;
    for (size_t i = 0; i < indices->size(); ++i) {
      (*indices)[i] = indices_copy[idx_array[i]];
      (*distances)[i] = distances_copy[idx_array[i]];
    }
  }
}

template<typename FloatT, std::size_t dimension, typename NormT>
bool ApproximateNearestNeighbor<FloatT, dimension, NormT>::empty() const {
  return numPoints() == 0;
}

template<typename FloatT, std::size_t dimension, typename NormT>
std::size_t ApproximateNearestNeighbor<FloatT, dimension, NormT>::numPoints() const {
//  return numPointsInternal();
  return index_.size();
}

template<typename FloatT, std::size_t dimension, typename NormT>
auto ApproximateNearestNeighbor<FloatT, dimension, NormT>::addPointInternal(const Point &point) -> FlannPointType& {
  FlannPointType flann_point;
  for (std::size_t col = 0; col < dimension; ++col) {
    flann_point[col] = point(col);
  }
  points_.push_front(flann_point);
  return points_.front();
//  FlannElementType *point_ptr = new FlannElementType[dimension];
//  for (std::size_t col = 0; col < dimension; ++col) {
//    point_ptr[col] = point(col);
//  }
//  points_.push_front();
//  return point_ptr;
}

template<typename FloatT, std::size_t dimension, typename NormT>
template<typename Iterator>
auto ApproximateNearestNeighbor<FloatT, dimension, NormT>::addPointsInternal(Iterator begin, Iterator end) -> FlannPointType& {
  std::vector<FlannPointType> flann_points;
  flann_points.reserve(end - begin);
  for (Iterator it = begin; it != end; ++it) {
    FlannPointType flann_point;
    for (std::size_t col = 0; col < dimension; ++col) {
      flann_point[col] = (*it)(col);
    }
    flann_points.push_back(flann_point);
  }
  flann_points.shrink_to_fit();
  point_arrays_.push_front(std::move(flann_points));
  return point_arrays_.front()[0];
}

template<typename FloatT, std::size_t dimension, typename NormT>
std::size_t ApproximateNearestNeighbor<FloatT, dimension, NormT>::numPointsInternal() const {
  return init_points_.size() + points_size_;
}

template<typename FloatT, std::size_t dimension, typename NormT>
auto ApproximateNearestNeighbor<FloatT, dimension, NormT>::getFlannIndex() const -> const flann::Index<NormT>& {
  return index_;
};

template<typename FloatT, std::size_t dimension, typename NormT>
auto ApproximateNearestNeighbor<FloatT, dimension, NormT>::getFlannIndex() -> flann::Index<NormT>& {
  return index_;
};

}

//#pragma GCC pop_options

