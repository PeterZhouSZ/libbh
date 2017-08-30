/*
 * random.h
 *
 *  Created on: Dec 27, 2016
 *      Author: bhepp
 */

#include <random>
#include "eigen.h"

namespace bh {

template <typename FloatT, typename IntT, typename RandomNumberGeneratorT = std::mt19937_64>
class Random {
public:
  using FloatType = FloatT;
  using IntType = IntT;
  using RandomNumberGenerator = RandomNumberGeneratorT;
  using Vector3 = Eigen::Matrix<FloatType, 3, 1>;

  static constexpr FloatType kSphereSquaredNormThreshold = FloatType { 0.001 };

  Random()
  : normal_dist_(0, 1), uniform_dist_(0, 1),
    uniform_int_dist_(0, std::numeric_limits<IntType>::max()) {}

  Random(std::size_t seed)
  : rng_(seed), normal_dist_(0, 1), uniform_dist_(0, 1),
    uniform_int_dist_(0, std::numeric_limits<IntType>::max()) {}

  void setSeed(std::size_t seed) {
    rng_.seed(seed);
  }

  RandomNumberGeneratorT& rng() {
    return rng_;
  }

  const RandomNumberGeneratorT& rng() const {
    return rng_;
  }

  bool sampleBernoulli(const FloatType true_probability) const {
    return sampleUniform() <= true_probability;
  }

  FloatType sampleNormal() const {
    return normal_dist_(rng_);
  }

  FloatType sampleNormal(const FloatType mean, const FloatType sigma) const {
    return mean + sigma * sampleNormal();
  }

  /// Sample uniform number from interval [0, 1)
  FloatType sampleUniform() const {
    return uniform_dist_(rng_);
  }

  /// Sample uniform number from interval [min, max) (excluding max)
  FloatType sampleUniform(const FloatType min, const FloatType max) const {
    return min + (max - min) * sampleUniform();
  }

  /// Sample uniform integer from interval [0, std::numeric_limits<IntType>::max()] (including max)
  IntType sampleUniformInt() const {
    return uniform_int_dist_(rng_);
  }

  /// Sample uniform integer from interval [min, max] (including max)
  IntType sampleUniformInt(const IntType min, const IntType max) const {
    const IntType random_max = std::numeric_limits<IntType>::max();
    const IntType num_values = max - min + 1;
    const IntType unbiased_sample_range = random_max - random_max % num_values;
    IntType sample;
    do {
      sample = sampleUniformInt();
    }
    while (sample >= unbiased_sample_range);
    return sample % num_values;
  }

  /// Sample uniform integer from interval [min, max) (excluding max)
  IntType sampleUniformIntExclusive(const IntType min, const IntType max) const {
    return sampleUniformInt(min, max - 1);
  }

  /// Sample uniform integer from interval [0, max) (excluding max)
  IntType sampleUniformIntExclusive(const IntType max) const {
    return sampleUniformIntExclusive(0, max);
  }

  Vector3 sampleSphericalShell(FloatType min_radius, FloatType max_radius) const {
    Vector3 vec;
    sampleSphericalShell(&vec);
    return vec;
  }

  template <typename Vector3T>
  void sampleSphericalShell(const FloatType min_radius, const FloatType max_radius, Vector3T* vec) const {
    sampleUnitSphere(vec);
    std::uniform_real_distribution<FloatType> radius_dist(min_radius, max_radius);
    (*vec) *= sampleUniform(min_radius, max_radius);
  }

  Vector3 sampleUnitSphere() const {
    Vector3 vec;
    sampleUnitSphere(&vec);
    return vec;
  }

  template <typename Vector3T>
  void sampleUnitSphere(Vector3T* vec) const {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector3T, 3);
    do {
      (*vec)(0) = sampleNormal();
      (*vec)(1) = sampleNormal();
      (*vec)(2) = sampleNormal();
    }
    while (vec->squaredNorm() < kSphereSquaredNormThreshold);
    vec->normalize();
  }

  /// Sample from a uniform discrete distribution
  template <typename Iterator>
  Iterator sampleDiscrete(const Iterator first, const Iterator last) {
    const size_t index = sampleUniformIntExclusive(last - first);
    const Iterator sample_it = first + index;
    return sample_it;
  }

  /// Sample from a discrete distribution with probabilities proportional to the given (positive) weights
  template <typename Iterator>
  Iterator sampleDiscreteWeighted(const Iterator first, const Iterator last) {
    FloatType total_weight = 0;
    for (Iterator it = first; it != last; ++it) {
      total_weight += *it;
    }
    const FloatType normalizing_factor = 1 / total_weight;
    const FloatType u = sampleUniform();
    FloatType cumulative_probability = 0;
    for (Iterator it = first; it != last; ++it) {
      cumulative_probability += *it * normalizing_factor;
      if (u <= cumulative_probability) {
        return it;
      }
    }
    return last;
  }

  /// Sample from a discrete distribution with probabilities proportional to the given (positive) weights
  template <typename Iterator, typename Evaluator>
  Iterator sampleDiscreteWeighted(const Iterator first, const Iterator last, Evaluator eval) {
    FloatType total_weight = 0;
    for (Iterator it = first; it != last; ++it) {
      total_weight += eval(*it);
    }
    const FloatType normalizing_factor = 1 / total_weight;
    const FloatType u = sampleUniform();
    FloatType cumulative_probability = 0;
    for (Iterator it = first; it != last; ++it) {
      cumulative_probability += eval(*it) * normalizing_factor;
      if (u <= cumulative_probability) {
        return it;
      }
    }
    return last;
  }

  /// Sample from a discrete distribution with given cumulative probabilities (sorted in ascending order)
  template <typename Iterator>
  Iterator sampleDiscreteCumulative(const Iterator first, const Iterator last) {
    FloatType u = sampleUniform();
    Iterator it = std::upper_bound(first, last, u);
    if (it == last) {
      return last - 1;
    }
    return it;
  }

  /// Sample from a discrete distribution with given cumulative probabilities (sorted in ascending order)
  template <typename Iterator, typename Compare>
  Iterator sampleDiscreteCumulative(const Iterator first, const Iterator last, Compare comp) {
    FloatType u = sampleUniform();
    Iterator it = std::upper_bound(first, last, u, comp);
    if (it == last) {
      return last - 1;
    }
    return it;
  }

private:
  mutable RandomNumberGenerator rng_;

  mutable std::normal_distribution<FloatType> normal_dist_;
  mutable std::uniform_real_distribution<FloatType> uniform_dist_;
  mutable std::uniform_int_distribution<IntType> uniform_int_dist_;
};

}
