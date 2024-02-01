/********************************************************************
 * @file bezier.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-01-24
 *
 * @copyright Copyright (c) 2024 ROSC
 *
 ********************************************************************/
#ifndef MODULES_ROBOT_PLANNING_INCLUDE_ROBOT_BRAIN_ROBOT_PLANNING_BEZIER_H_
#define MODULES_ROBOT_PLANNING_INCLUDE_ROBOT_BRAIN_ROBOT_PLANNING_BEZIER_H_

#include <iostream>
#include <ostream>
#include <vector>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include "robot_brain/config.h"

namespace rosc {

class Bezier {
 public:
  Bezier();
  ~Bezier();

 private:
  int order_;
  std::vector<KDL::Vector> point_;

 private:
  KDL::Vector deCasteljau(int i, int k, double t);

 public:
  void clear();
  error_t AddPoint(KDL::Vector Point);
  error_t Pos(double t, KDL::Vector *pos);
  double CalculateLength(KDL::Vector p1, KDL::Vector p2, KDL::Vector p3);
};
}  // namespace rosc
#endif  // MODULES_ROBOT_PLANNING_INCLUDE_ROBOT_BRAIN_ROBOT_PLANNING_BEZIER_H_
