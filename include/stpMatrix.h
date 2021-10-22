//
// Created by dvrk-1804 on 2021-09-21.
//

#ifndef DVRK_TWO_CONSOLE_LIBS_STPMATRIX_H_
#define DVRK_TWO_CONSOLE_LIBS_STPMATRIX_H_

#include <ros/topic.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/convert.h>

#define timeOut ros::Duration(0.002)

class stpMatrix {
 public:
  stpMatrix():
  transform_stamped_(),
  matrix_4_d_()
  {}

  stpMatrix(geometry_msgs::TransformStamped transform_stamped) {
    transform_stamped_ = transform_stamped;
    matrix_4_d_ = tf2::transformToEigen(transform_stamped);
  }
  stpMatrix(Eigen::Isometry3d matrix_4_d) {
    matrix_4_d_ = matrix_4_d;
    transform_stamped_ = tf2::eigenToTransform(matrix_4_d);
  }

  inline Eigen::Matrix3d GetMatrixRotation() {
    return matrix_4_d_.matrix().block(0,0,3,3);
  }

  inline Eigen::Matrix3Xd GetMatrixTranslation() {
    return matrix_4_d_.matrix().block(0,3,3,1);
  }

  inline geometry_msgs::Quaternion GetTransformRotation() {
    return transform_stamped_.transform.rotation;
  }

  inline geometry_msgs::Vector3 GetTransformTranslation() {
    return transform_stamped_.transform.translation;
  }

  inline Eigen::Matrix3d Inverse() {
    return this->GetMatrixRotation().inverse();
  }

  inline void SetRotation(Eigen::Matrix3d rotation) {
    matrix_4_d_.matrix().block(0,0,3,3) = rotation;
    UpdateTransformStamped();
  }

  inline void SetRotation(geometry_msgs::TransformStamped rotation) {
    transform_stamped_.transform.rotation = rotation.transform.rotation;
    UpdateEigenMatrix();
  }


  inline void SetTranslation(Eigen::Matrix3Xd translation) {
    matrix_4_d_.matrix().block(0,3,3,1) = translation;
    UpdateTransformStamped();
  }

  inline void SetTranslation(geometry_msgs::TransformStamped transform_stamped) {
    transform_stamped_.transform.translation = transform_stamped.transform.translation;
    UpdateEigenMatrix();
  }

  inline void FromNormalized(Eigen::Matrix3d rotation) {
    matrix_4_d_.matrix().block(0,0,3,3) = rotation.normalized();
    UpdateTransformStamped();
  }

  inline void UpdateTransformStamped() {
    transform_stamped_ = tf2::eigenToTransform(matrix_4_d_);
  }

  inline void UpdateEigenMatrix() {
    matrix_4_d_ = tf2::transformToEigen(transform_stamped_);
  }

  inline void GetROSMessage(const geometry_msgs::TransformStamped& newMessage) {
    transform_stamped_ = newMessage;
    UpdateEigenMatrix();
  }

  inline double AlignedWith(Eigen::Matrix3d comparison_matrix) {
    Eigen::Quaterniond comparison_quaternion(comparison_matrix);
    Eigen::Quaterniond this_quaternion(matrix_4_d_.rotation());
    return comparison_quaternion.angularDistance(this_quaternion);
  }

  inline bool isValid(std::string topicName) {
    geometry_msgs::TransformStampedConstPtr executionResult;
    executionResult = ros::topic::waitForMessage<geometry_msgs::TransformStamped>(topicName, timeOut);

    if (executionResult == nullptr) {
      return false;
    }
    return true;
  }

  inline bool isValid(stpMatrix matrix) {
    int rows = matrix.matrix_4_d_.rows(), cols = matrix.matrix_4_d_.cols();
    if (rows == 4 && cols == 4) {
      return true;
    }
    return false;
  }

  Eigen::Isometry3d matrix_4_d_;
  geometry_msgs::TransformStamped transform_stamped_;

};


#endif //DVRK_TWO_CONSOLE_LIBS_STPMATRIX_H_
