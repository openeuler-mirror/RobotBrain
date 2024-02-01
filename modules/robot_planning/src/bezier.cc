/********************************************************************
 * @file bezier.cc
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-01-24
 *
 * @copyright Copyright (c) 2024 ROSC
 *
 ********************************************************************/

#include <kdl/utilities/utility.h>
#include <robot_brain/robot_planning/bezier.h>
#include <math.h>
#include <cmath>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <robot_brain/core.hpp>

namespace rosc {

Bezier::Bezier() {}
Bezier::~Bezier() {}

error_t Bezier::AddPoint(KDL::Vector Point) {
  this->point_.push_back(Point);
  this->order_ = this->point_.size() - 1;
  return 0;
}

/**
 * @brief 递归求解贝塞尔曲线
 *
 * @param i
 * @param k
 * @param t
 * @return KDL::Vector
 */
KDL::Vector Bezier::deCasteljau(int i, int k, double t) {
  if (k == 0) {
    return this->point_[i];
  }
  KDL::Vector res, pos1, pos2;
  pos1 = deCasteljau(i, k - 1, t);
  pos2 = deCasteljau(i + 1, k - 1, t);
  for (int i = 0; i < 3; i++) {
    res.data[i] = (1 - t) * pos1.data[i] + t * pos2.data[i];
  }
  return res;
}

/**
 * @brief 返回点的位置
 *
 * @param t
 * @param pos
 * @return error_t
 */
error_t Bezier::Pos(double t, KDL::Vector *pos) {
  if (t < 0 || t > 1) {
    return -1;
  }
  KDL::Vector res = this->deCasteljau(0, this->order_, t);
  for (int i = 0; i < 3; i++) {
    pos->data[i] = res.data[i];
  }
  return 0;
}

/**
 * @brief
 *
 * @param p1
 * @param p2
 * @param p3
 * @return double
 */
double Bezier::CalculateLength(KDL::Vector p1, KDL::Vector p2, KDL::Vector p3) {
  double ax = p1.x() - 2 * p2.x() + p3.x();
  double ay = p1.y() - 2 * p2.y() + p3.y();
  double bx = 2 * p2.x() - 2 * p1.x();
  double by = 2 * p2.y() - 2 * p1.y();
  double A = 4 * (ax * ax + ay * ay);
  double B = 4 * (ax * bx + ay * by);
  double C = bx * bx + by * by;
  double t = 1;
  double temp1 = sqrt(C + t * (B + A * t));

  double temp2 = (2 * A * t * temp1 + B * (temp1 - sqrt(C)));

  double temp3 = log(B + 2 * sqrt(A) * sqrt(C));

  double temp4 = log(B + 2 * A * t + 2 * sqrt(A) * temp1);

  double temp5 = 2 * sqrt(A) * temp2;

  double temp6 = (B * B - 4 * A * C) * (temp3 - temp4);

  return (temp5 + temp6) / (8 * pow(A, 1.5));
}
/**
 * @brief 清空队列
 *
 */
void Bezier::clear() {
  this->order_ = 0;
  this->point_.clear();
}
}  // namespace rosc
