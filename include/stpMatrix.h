//
// Created by dvrk-1804 on 2021-09-21.
//

#ifndef DVRK_TWO_CONSOLE_LIBS_STPMATRIX_H_
#define DVRK_TWO_CONSOLE_LIBS_STPMATRIX_H_

#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/convert.h>

class stpMatrix {
 public:

 private:
  Eigen::Matrix4d matrix_4_d_;
  geometry_msgs::TransformStamped transform_stamped_;
};


#endif //DVRK_TWO_CONSOLE_LIBS_STPMATRIX_H_
