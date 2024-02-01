/********************************************************************
 * @file double_hump.cc
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-01-24
 *
 * @copyright Copyright (c) 2024 ROSC
 *
 ********************************************************************/
#include "robot_brain/robot_planning/interpolate.h"
#include <yaml-cpp/yaml.h>
#include <robot_brain/robot_planning/double_hump.h>
#include <robot_brain/config.h>
#include <kdl/utilities/utility.h>
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/node/parse.h>
#include <math.h>
#include <cstring>
#include <iostream>
#include <cstddef>
#include <cmath>
#include <memory>
#include <cerrno>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <robot_brain/core.hpp>
namespace rosc {
DoubleHump::DoubleHump() {
  for (int i = 0; i < 2; ++i) {
    this->s_vp_[i] = new S_VelocityProfile();
    this->t_vp_[i] = new T_VelocityProfile();
  }
}

DoubleHump::~DoubleHump() {
  for (int i = 0; i < 2; ++i) {
    delete this->s_vp_[i];
    delete this->t_vp_[i];
  }
}

/********************************************************************
 * @brief 基于S型速度规划实现双峰对称速度规划，可设置zone区域的速度
 *
 * @param P0
 * @param Pend
 * @param Vmax
 * @param Amax
 * @param Jmax
 * @param Vmid
 * @return error_t
 ********************************************************************/
error_t DoubleHump::DoubleHumpVelocityProfile_S(double P0, double Pend,
                                                double Vmax, double Amax,
                                                double Jmax, double Vmid) {
  this->traj_type_ = S_VELOCITY_PROFILE;
  this->P0_ = P0;
  this->Pend_ = Pend;
  this->v_mid_ = Vmid;
  this->v_max_ = Vmax;
  this->acc_max_ = Amax;
  this->jerk_ = Jmax;
  double S1, S3;
  S1 = S3 = fabs(Pend - P0) / 2;
  this->S[1] = S1;
  this->S[2] = 0;
  this->S[3] = S3;
  error_t e1 =
      this->s_vp_[0]->SetVelocityProfile(Vmax, Amax, Jmax, S[1], 0, Vmid);
  error_t e2 =
      this->s_vp_[1]->SetVelocityProfile(Vmax, Amax, Jmax, S[3], Vmid, 0);
  this->T[1] = this->s_vp_[0]->Duration();
  this->T[2] = 0;
  this->T[3] = this->s_vp_[1]->Duration();
  this->Tf_ = T[1] + T[2] + T[3];
  this->Sf_ = S[1] + S[2] + S[3];
  if (e1 == 0 && e2 == 0) {
    return 0;
  }
  return -1;
}
error_t DoubleHump::DoubleHumpVelocityProfile_T(double P0, double Pend,
                                                double Vmax, double Amax,
                                                double Vmid) {
  this->traj_type_ = T_VELOCITY_PROFILE;
  this->P0_ = P0;
  this->Pend_ = Pend;
  this->v_mid_ = Vmid;
  this->v_max_ = Vmax;
  this->acc_max_ = Amax;
  this->jerk_ = 0;
  double S1, S3;
  S1 = S3 = fabs(Pend - P0) / 2;
  this->S[1] = S1;
  this->S[2] = 0;
  this->S[3] = S3;
  error_t e1 = this->t_vp_[0]->SetVelocityProfile(Vmax, Amax, S[1], 0, Vmid);
  error_t e2 = this->t_vp_[1]->SetVelocityProfile(Vmax, Amax, S[3], Vmid, 0);
  this->T[1] = this->t_vp_[0]->Duration();
  this->T[2] = 0;
  this->T[3] = this->t_vp_[1]->Duration();
  this->Tf_ = T[1] + T[2] + T[3];
  this->Sf_ = S[1] + S[2] + S[3];
  if (e1 == 0 && e2 == 0) {
    return 0;
  }
  return -1;
}
/********************************************************************
 * @brief 基于S型速度规划实现双峰对称速度规划，可以设置zone的速度和距离
 *
 * @param P0
 * @param Pend
 * @param Vmax
 * @param Amax
 * @param Jmax
 * @param Vmid
 * @param Sm
 * @return error_t
 ********************************************************************/
error_t DoubleHump::DoubleHumpVelocityProfile_S(double P0, double Pend,
                                                double Vmax, double Amax,
                                                double Jmax, double Vmid,
                                                double Sm) {
  this->traj_type_ = S_VELOCITY_PROFILE;
  this->P0_ = P0;
  this->Pend_ = Pend;
  this->v_mid_ = Vmid;
  this->v_max_ = Vmax;
  this->acc_max_ = Amax;
  this->jerk_ = Jmax;
  double S1, S3;
  S1 = S3 = fabs(Pend - P0 - Sm) / 2;
  this->S[1] = S1;
  this->S[2] = Sm;
  this->S[3] = S3;
  error_t e1 =
      this->s_vp_[0]->SetVelocityProfile(Vmax, Amax, Jmax, S[1], 0, Vmid);
  error_t e2 =
      this->s_vp_[1]->SetVelocityProfile(Vmax, Amax, Jmax, S[3], Vmid, 0);
  this->T[1] = this->s_vp_[0]->Duration();
  this->T[2] = Sm / v_mid_;
  this->T[3] = this->s_vp_[1]->Duration();
  this->Tf_ = T[1] + T[2] + T[3];
  this->Sf_ = S[1] + S[2] + S[3];
  if (e1 == 0 && e2 == 0) {
    return 0;
  }
  return -1;
}

error_t DoubleHump::DoubleHumpVelocityProfile_T(double P0, double Pend,
                                                double Vmax, double Amax,
                                                double Vmid, double Sm) {
  this->traj_type_ = T_VELOCITY_PROFILE;
  this->P0_ = P0;
  this->Pend_ = Pend;
  this->v_mid_ = Vmid;
  this->v_max_ = Vmax;
  this->acc_max_ = Amax;
  this->jerk_ = 0;
  double S1, S3;
  S1 = S3 = fabs(Pend - P0 - Sm) / 2;
  this->S[1] = S1;
  this->S[2] = Sm;
  this->S[3] = S3;
  error_t e1 = this->t_vp_[0]->SetVelocityProfile(Vmax, Amax, S[1], 0, Vmid);
  error_t e2 = this->t_vp_[1]->SetVelocityProfile(Vmax, Amax, S[3], Vmid, 0);
  this->T[1] = this->t_vp_[0]->Duration();
  this->T[2] = Sm / v_mid_;
  this->T[3] = this->t_vp_[1]->Duration();
  this->Tf_ = T[1] + T[2] + T[3];
  this->Sf_ = S[1] + S[2] + S[3];
  if (e1 == 0 && e2 == 0) {
    return 0;
  }
  return -1;
}

/********************************************************************
 * @brief 基于S型速度规划实现双峰非对称速度规划，设置zone区域的起点和终点
 *
 * @param P0
 * @param Pend
 * @param Vmax
 * @param Amax
 * @param Jmax
 * @param Vmid
 * @param Pm0
 * @param Pmend
 * @return error_t
 ********************************************************************/
error_t DoubleHump::DoubleHumpVelocityProfile_S(double P0, double Pend,
                                                double Vmax, double Amax,
                                                double Jmax, double Vmid,
                                                double Pm0, double Pmend) {
  this->traj_type_ = S_VELOCITY_PROFILE;
  this->P0_ = P0;
  this->Pend_ = Pend;
  this->Pm0_ = Pm0;
  this->Pmend_ = Pmend;
  this->v_mid_ = Vmid;
  this->v_max_ = Vmax;
  this->acc_max_ = Amax;
  this->jerk_ = Jmax;
  double S1, S2, S3;
  S1 = fabs(Pm0 - P0);
  S2 = fabs(Pmend - Pm0);
  S3 = fabs(Pend - Pmend);

  this->S[1] = S1;
  this->S[2] = S2;
  this->S[3] = S3;
  error_t e1 =
      this->s_vp_[0]->SetVelocityProfile(Vmax, Amax, Jmax, S[1], 0, Vmid);
  error_t e2 =
      this->s_vp_[1]->SetVelocityProfile(Vmax, Amax, Jmax, S[3], Vmid, 0);
  this->T[1] = this->s_vp_[0]->Duration();
  this->T[2] = S[2] / v_mid_;
  this->T[3] = this->s_vp_[1]->Duration();
  this->Tf_ = T[1] + T[2] + T[3];
  this->Sf_ = S[1] + S[2] + S[3];
  if (e1 == 0 && e2 == 0) {
    return 0;
  }
  return -1;
}

error_t DoubleHump::DoubleHumpVelocityProfile_T(double P0, double Pend,
                                                double Vmax, double Amax,
                                                double Vmid, double Pm0,
                                                double Pmend) {
  this->traj_type_ = T_VELOCITY_PROFILE;
  this->P0_ = P0;
  this->Pend_ = Pend;
  this->Pm0_ = Pm0;
  this->Pmend_ = Pmend;
  this->v_mid_ = Vmid;
  this->v_max_ = Vmax;
  this->acc_max_ = Amax;
  this->jerk_ = 0;
  double S1, S2, S3;
  S1 = fabs(Pm0 - P0);
  S2 = fabs(Pmend - Pm0);
  S3 = fabs(Pend - Pmend);

  this->S[1] = S1;
  this->S[2] = S2;
  this->S[3] = S3;
  error_t e1 = this->t_vp_[0]->SetVelocityProfile(Vmax, Amax, S[1], 0, Vmid);
  error_t e2 = this->t_vp_[1]->SetVelocityProfile(Vmax, Amax, S[3], Vmid, 0);
  this->T[1] = this->t_vp_[0]->Duration();
  this->T[2] = S[2] / v_mid_;
  this->T[3] = this->t_vp_[1]->Duration();
  this->Tf_ = T[1] + T[2] + T[3];
  this->Sf_ = S[1] + S[2] + S[3];
  if (e1 == 0 && e2 == 0) {
    return 0;
  }
  return -1;
}

/********************************************************************
 * @brief
 *
 * @param t
 * @return double
 ********************************************************************/
double DoubleHump::Pos(double t) {
  if (t > this->Tf_) {
    return this->Pend_;
  }
  double t_stage;
  double fabs_pos;
  if (this->traj_type_ == S_VELOCITY_PROFILE) {
    if (t <= T[1]) {
      t_stage = t;
      fabs_pos = this->s_vp_[0]->Pos(t);
    } else if (t > T[1] && t <= T[1] + T[2]) {
      t_stage = t - T[1];
      fabs_pos = this->S[1] + t_stage * v_mid_;
    } else if (t > (T[1] + T[2]) && t <= (T[1] + T[2] + T[3])) {
      t_stage = t - T[1] - T[2];
      fabs_pos = S[1] + S[2] + this->s_vp_[1]->Pos(t_stage);
    }
  } else if (this->traj_type_ == T_VELOCITY_PROFILE) {  // T型规划
    if (t <= T[1]) {
      t_stage = t;
      fabs_pos = this->t_vp_[0]->Pos(t);
    } else if (t > T[1] && t <= T[1] + T[2]) {
      t_stage = t - T[1];
      fabs_pos = this->S[1] + t_stage * v_mid_;
    } else if (t > (T[1] + T[2]) && t <= (T[1] + T[2] + T[3])) {
      t_stage = t - T[1] - T[2];
      fabs_pos = S[1] + S[2] + this->t_vp_[1]->Pos(t_stage);
    }
  }
  if (this->P0_ < this->Pend_) {
    return this->P0_ + fabs_pos;
  } else {
    return this->P0_ - fabs_pos;
  }
  return 0;
}

double DoubleHump::Duration() { return this->Tf_; }
}  // namespace rosc
