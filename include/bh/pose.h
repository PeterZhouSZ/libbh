//==================================================
// pose.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 9, 2016
//==================================================
#pragma once

#include <iostream>
#include <bh/eigen.h>
#include <bh/eigen_utils.h>
#include <bh/eigen_serialization.h>
#include <boost/serialization/access.hpp>

namespace bh {

// A pose gives the transformation from image to world coordinate system
template <typename T>
struct Pose {
  using FloatType = T;
  USE_FIXED_EIGEN_TYPES(FloatType)

  Pose()
  : translation_(0, 0, 0), quaternion_(1, 0, 0, 0) {}

  Pose(const Vector3& translation, const Quaternion& quaternion)
  : translation_(translation), quaternion_(quaternion) {}

  Pose(const Vector3& translation, const Matrix3x3& rotation)
  : translation_(translation), quaternion_(rotation) {}

  Pose(const Matrix4x4& matrix)
  : translation_(matrix.col(3).template topRows<3>()),
    quaternion_(matrix.template topLeftCorner<3, 3>()) {}

  static Pose createFromImageToWorldTransformation(
      const Vector3& translation, const Matrix3x3& rotation) {
    return Pose(translation, rotation);
  }

  static Pose createFromImageToWorldTransformation(
      const Vector3& translation, const Quaternion& quaternion) {
    return Pose(translation, quaternion);
  }

  static Pose createFromWorldToImageTransformation(
      const Vector3& translation, const Matrix3x3& rotation) {
    return Pose(translation, rotation).inverse();
  }

  static Pose createFromWorldToImageTransformation(
      const Vector3& translation, const Quaternion& quaternion) {
    return Pose(translation, quaternion).inverse();
  }

  bool isValid() const {
    return translation_.allFinite() && quaternion_.coeffs().allFinite();
  }

  bool operator==(const Pose& other) const {
    return translation_ == other.translation_ &&
            (quaternion_.coeffs() == other.quaternion_.coeffs() || quaternion_.coeffs() == -other.quaternion_.coeffs());
  }

  bool operator!=(const Pose& other) const {
    return !(*this == other);
  }

  bool isApprox(const Pose& other, const FloatType prec = Eigen::NumTraits<FloatType>::dummy_precision()) const {
    return translation_.isApprox(other.translation_, prec) && quaternion_.isApprox(other.quaternion_, prec);
  }

  Vector3& translation() {
    return translation_;
  }

  const Vector3& translation() const {
    return translation_;
  }

  Quaternion& quaternion() {
    return quaternion_;
  }

  const Quaternion& quaternion() const {
    return quaternion_;
  }

  FloatType getPositionDistanceTo(const Pose& other) const {
    const FloatType pos_distance = (translation_ - other.translation_).norm();
    return pos_distance;
  }

  FloatType getAngularDistanceTo(const Pose& other) const {
    const FloatType angular_distance = quaternion_.angularDistance(other.quaternion_);
    return angular_distance;
  }

  FloatType getDistanceTo(const Pose& other, FloatType rotation_factor = 1) const {
    const FloatType pos_distance = getPositionDistanceTo(other);
    const FloatType angular_distance = getAngularDistanceTo(other);
    return pos_distance + rotation_factor * angular_distance;
  }

  const Vector3& getWorldPosition() const {
    return translation_;
//    return - (quaternion_.inverse() * translation_);
  }

  Pose inverse() const {
    Pose inv_pose;
    inv_pose.quaternion() = quaternion_.inverse();
    inv_pose.translation() = - (inv_pose.quaternion() * translation_);
    return inv_pose;
  }

  Matrix3x3 rotation() const {
    return quaternion_.toRotationMatrix();
  }

  Matrix3x4 getTransformationImageToWorld() const {
    Matrix3x4 transformation;
    transformation.template leftCols<3>() = quaternion_.toRotationMatrix();
    transformation.template rightCols<1>() = translation_;
    return transformation;
  }

  Matrix3x4 getTransformationWorldToImage() const {
    Matrix3x4 transformation;
    transformation.template leftCols<3>() = quaternion_.inverse().toRotationMatrix();
    transformation.template rightCols<1>() = - transformation.template leftCols<3>() * translation_;
    return transformation;
  }

  Matrix4x4 getTransformationImageToWorld4x4() const {
    Matrix4x4 transformation;
    transformation.template topRows<3>() = getTransformationImageToWorld();
    transformation.row(3) = Vector4(0, 0, 0, 1);
    return transformation;
  }

  Matrix4x4 getTransformationWorldToImage4x4() const {
    Matrix4x4 transformation;
    transformation.template topRows<3>() = getTransformationWorldToImage();
    transformation.row(3) = Vector4(0, 0, 0, 1);
    return transformation;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  // Boost serialization
  friend class boost::serialization::access;

  template <typename Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & translation_;
    ar & quaternion_;
  }

  Vector3 translation_;
  Quaternion quaternion_;
};

template <typename _CharT, typename FloatType>
std::basic_ostream<_CharT>& operator<<(std::basic_ostream<_CharT>& out, const Pose<FloatType>& pose) {
  out << "t: (" << pose.translation().transpose() << "), q: (" << pose.quaternion() << ")";
  return out;
}

}
