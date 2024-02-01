/********************************************************************
 * @file double_hump.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-01-24
 *
 * @copyright Copyright (c) 2024 ROSC
 *
 ********************************************************************/
#ifndef MODULES_ROBOT_PLANNING_INCLUDE_ROBOT_BRAIN_ROBOT_PLANNING_DOUBLE_HUMP_H_
#define MODULES_ROBOT_PLANNING_INCLUDE_ROBOT_BRAIN_ROBOT_PLANNING_DOUBLE_HUMP_H_

#include <cerrno>
#include <iostream>
#include <ostream>
#include <vector>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include "robot_brain/config.h"
#include "robot_brain/robot_planning/interpolate.h"

namespace rosc {

class DoubleHump {
 public:
  DoubleHump();
  ~DoubleHump();

 private:
  double P0_;
  double Pm0_;
  double Pmend_;
  double Pend_;
  double v_max_;  // 匀速阶段最大速度
  double v_mid_;
  double v_start_;   // 起始速度
  double v_end_;     // 终点速度
  double acc_max_;   // 最大加速度
  double dcc_max_;   // 最大减速度
  double jerk_;      // 加加速度
  double jerk_dcc_;  // 减减速度

  double T[4];
  double S[4];
  double Tf_;
  double Sf_;
  TrajType traj_type_;  // 轨迹曲线类型

 private:
  S_VelocityProfile *s_vp_[2];
  T_VelocityProfile *t_vp_[2];

 public:
  error_t DoubleHumpVelocityProfile_S(double P0, double Pend, double Vmax,
                                      double Amax, double Jmax, double Vmid);
  error_t DoubleHumpVelocityProfile_S(double P0, double Pend, double Vmax,
                                      double Amax, double Jmax, double Vmid,
                                      double Sm);
  error_t DoubleHumpVelocityProfile_S(double P0, double Pend, double Vmax,
                                      double Amax, double Jmax, double Vmid,
                                      double Pm0, double Pmend);
  error_t DoubleHumpVelocityProfile_T(double P0, double Pend, double Vmax,
                                      double Amax, double Vmid);
  error_t DoubleHumpVelocityProfile_T(double P0, double Pend, double Vmax,
                                      double Amax, double Vmid, double Sm);
  error_t DoubleHumpVelocityProfile_T(double P0, double Pend, double Vmax,
                                      double Amax, double Vmid, double Pm0,
                                      double Pmend);
  void GetT(double *time);
  double Duration();
  double Pos(double t);
};
}  // namespace rosc
#endif  // MODULES_ROBOT_PLANNING_INCLUDE_ROBOT_BRAIN_ROBOT_PLANNING_DOUBLE_HUMP_H_
