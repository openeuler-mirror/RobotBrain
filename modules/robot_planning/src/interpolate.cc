/**
 * @file interpolate.cc
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2021-10-07
 *
 * @copyright Copyright (c) 2021 ROSC
 *
 */

#include "robot_brain/command_types.hpp"
#include <yaml-cpp/yaml.h>
#include <robot_brain/robot_planning/interpolate.h>
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
// const char *const kSorterConfigDir = "/robot/config/sorter_config.yml";
void S_VelocityProfile::PrintPara() {
  std::cout << "七个阶段位移：" << std::endl;
  double S = 0;
  for (int i = 1; i < 8; ++i) {
    std::cout << this->S[i] << " ";
    S += this->S[i];
  }
  std::cout << std::endl << "总位移：" << S << std::endl;

  if (S < 1e-2) {
    return;
  }

  std::cout << std::endl << "七个阶段速度：" << std::endl;
  for (int i = 1; i < 8; ++i) {
    std::cout << this->V[i] << " ";
  }
  std::cout << std::endl;
  std::cout << std::endl << "七个阶段时间：" << std::endl;
  for (int i = 1; i < 8; ++i) {
    std::cout << this->T[i] << " ";
  }
  std::cout << std::endl << "总时间：" << this->Tf_ << std::endl;

  std::cout << std::endl << "七个阶段加速度：" << std::endl;
  for (int i = 1; i < 8; ++i) {
    std::cout << this->A[i] << " ";
  }
  std::cout << std::endl;
  std::cout << std::endl << "七个阶段加加速度：" << std::endl;
  for (int i = 1; i < 8; ++i) {
    std::cout << this->J[i] << " ";
  }
  std::cout << std::endl << std::endl << std::endl;
}

void S_VelocityProfile::ReplanStop(double current_time) {
  double cur_pos = this->Pos(current_time);
  double cur_vel = this->Vel(current_time);
  // 100ms停止速度
  double dcc_time = 0.1;
  this->jerk_dcc_ = cur_vel * 4 / pow(dcc_time, 2);

  this->T[1] = 0;
  this->T[2] = 0;
  this->T[3] = 0;
  this->T[4] = current_time;
  this->T[5] = dcc_time / 2;
  this->T[6] = 0;
  this->T[7] = T[5];

  this->J[1] = 0;
  this->J[2] = 0;
  this->J[3] = 0;
  this->J[4] = 0;
  this->J[5] = this->jerk_dcc_;
  this->J[6] = 0;
  this->J[7] = J[5];

  this->A[1] = 0;
  this->A[2] = 0;
  this->A[3] = 0;
  this->A[4] = 0;
  this->A[5] = 0;
  this->A[6] = this->A[5] - this->J[5] * this->T[5];
  this->A[7] = 0;

  this->V[1] = cur_vel;
  this->V[2] = cur_vel;
  this->V[3] = cur_vel;
  this->V[4] = cur_vel;
  this->V[5] = cur_vel;
  this->V[6] = V[5] - J[5] * T[5] * T[5] / 2;
  this->V[7] = V[6];

  this->S[1] = 0;
  this->S[2] = 0;
  this->S[3] = 0;
  this->S[4] = cur_pos;
  this->S[5] = V[5] * T[5] - J[5] * T[5] * T[5] * T[5] / 6;
  this->S[6] = 0;
  this->S[7] = J[7] * T[7] * T[7] * T[7] / 6;

  this->Tf_ = current_time + dcc_time;
  this->Sf_ = cur_pos + S[5] + S[7];
}

// 求两个圆的交点
typedef struct {
  float x;
  float y;
} Point2f;  // 类似opencv

typedef struct {
  Point2f c;
  float r;
} CIRCLE;

void IntersectionOf2Circles(CIRCLE c1, CIRCLE c2, Point2f *P1, Point2f *P2) {
  float a1, b1, R1, a2, b2, R2;
  a1 = c1.c.x;
  b1 = c1.c.y;
  R1 = c1.r;

  a2 = c2.c.x;
  b2 = c2.c.y;
  R2 = c2.r;

  //
  float R1R1 = R1 * R1;
  float a1a1 = a1 * a1;
  float b1b1 = b1 * b1;

  float a2a2 = a2 * a2;
  float b2b2 = b2 * b2;
  float R2R2 = R2 * R2;

  float subs1 = a1a1 - 2 * a1 * a2 + a2a2 + b1b1 - 2 * b1 * b2 + b2b2;
  float subs2 = -R1R1 * a1 + R1R1 * a2 + R2R2 * a1 - R2R2 * a2 + a1a1 * a1 -
                a1a1 * a2 - a1 * a2a2 + a1 * b1b1 - 2 * a1 * b1 * b2 +
                a1 * b2b2 + a2a2 * a2 + a2 * b1b1 - 2 * a2 * b1 * b2 +
                a2 * b2b2;
  float subs3 = -R1R1 * b1 + R1R1 * b2 + R2R2 * b1 - R2R2 * b2 + a1a1 * b1 +
                a1a1 * b2 - 2 * a1 * a2 * b1 - 2 * a1 * a2 * b2 + a2a2 * b1 +
                a2a2 * b2 + b1b1 * b1 - b1b1 * b2 - b1 * b2b2 + b2b2 * b2;
  float sigma = sqrt((R1R1 + 2 * R1 * R2 + R2R2 - a1a1 + 2 * a1 * a2 - a2a2 -
                      b1b1 + 2 * b1 * b2 - b2b2) *
                     (-R1R1 + 2 * R1 * R2 - R2R2 + subs1));
  if (abs(subs1) > 0.0000001) {
    P1->x = (subs2 - sigma * b1 + sigma * b2) / (2 * subs1);
    P2->x = (subs2 + sigma * b1 - sigma * b2) / (2 * subs1);

    P1->y = (subs3 + sigma * a1 - sigma * a2) / (2 * subs1);
    P2->y = (subs3 - sigma * a1 + sigma * a2) / (2 * subs1);
  }
}

static S_Vel_Para V_CalculateS(double v_start, double v_end, double acc,
                               double jerk = 0.0);

static S_Vel_Para S_CalculateV(double v_start, double v_end, double acc,
                               double jerk, double S);
/**
 * @brief
 *已知速度求路程S，求解S速度规划的加速阶段（3个阶段）或减速阶段（3个阶段）
 *
 * @param v_start
 * @param v_end
 * @param acc
 * @param jerk
 * @return S_Vel_Para
 */
static S_Vel_Para V_CalculateS(double v_start, double v_end, double acc,
                               double jerk) {
  double S, s1, s2, s3;  // 总路程，第一段，第二段，第三段
  double v0, v1, v2, v3;  // 第一段末速度， 第二段末速度，第三段末速度
  double t1, t2, t3;  // 第一段时间，第二段时间，第三段时间
  double acc_ = acc;
  S_Vel_Para res;

  t1 = jerk > 0.0 ? (acc_ / jerk) : 0;
  t3 = t1;
  double delta_v = fabs(v_start - v_end);

  // 判断是否存在匀加速阶段
  if (pow(acc_, 2) / jerk > delta_v) {
    // 需要重新计算最大加速度，没有达到最大加速度
    acc_ = std::sqrt(delta_v * jerk);
    t1 = acc_ / jerk;
    t2 = 0;
    t3 = t1;

  } else {
    // 存在匀加速阶段
    t2 = (delta_v - pow(acc_, 2) / jerk) / acc_;
  }
  res.acc[1] = 0;  // 每个阶段的起始加速度
  res.acc[2] = acc_;
  res.acc[3] = acc_;

  res.jerk[1] = jerk;
  res.jerk[2] = 0;
  res.jerk[3] = jerk;

  // 计算s
  if (v_start < v_end) {
    // 加速
    v0 = v_start;
    v1 = v0 + jerk * t1 * t1 / 2.0;
    v2 = v1 + acc_ * t2;
    v3 = v2 + acc_ * t3 - jerk * t3 * t3 / 2.0;

    s1 = v0 * t1 + jerk * pow(t1, 3) / 6.0;
    s2 = v1 * t2 + acc_ * t2 * t2 / 2.0;
    s3 = v2 * t3 + acc_ * t3 * t3 / 2.0 - jerk * t3 * t3 * t3 / 6.0;
    S = s1 + s2 + s3;

  } else {
    // 减速
    v0 = v_start;
    v1 = v0 - jerk * t1 * t1 / 2;
    v2 = v1 - acc_ * t2;
    v3 = v2 - acc_ * t3 + jerk * t3 * t3 / 2.0;

    s1 = v0 * t1 - jerk * pow(t1, 3) / 6.0;
    s2 = v1 * t2 - acc_ * t2 * t2 / 2.0;
    s3 = v2 * t3 - acc_ * t3 * t3 / 2.0 + jerk * pow(t3, 3) / 6.0;
    S = s1 + s2 + s3;
  }

  res.S = S;
  res.t[1] = t1;
  res.t[2] = t2;
  res.t[3] = t3;

  res.v[0] = v0;  // 第0阶段的末速度
  res.v[1] = v1;  // 第一阶段的末速度
  res.v[2] = v2;
  res.v[3] = v3;

  res.s[1] = s1;
  res.s[2] = s2;
  res.s[3] = s3;
  return res;
}

/**
 * @brief 已知路程的情况下，求速度
 *  针对double_acc 和 double_dcc 的曲线情况，将其作为一段加速或减速进行规划
 *  已知路程和速度，调整加速度或加加速度以满足情况
 * @param v_start
 * @param v_end
 * @param acc
 * @param jerk
 * @param S
 * @return S_Vel_Para
 */
static S_Vel_Para S_CalculateV(double v_start, double v_end, double acc,
                               double jerk, double S) {
  S_Vel_Para res, temp_para;

  if (fabs(S) < 1e-6) {  // 总路程为0
    res.S = S;
    for (int i = 1; i < 4; i++) {
      res.s[i] = 0;
      res.t[i] = 0;
      res.v[i] = v_start;
    }
    res.acc[1] = 0;
    res.acc[2] = acc;
    res.acc[3] = acc;
    res.jerk[1] = jerk;
    res.jerk[2] = 0;
    res.jerk[3] = 0;
    return res;
  }
  double delta_v = fabs(v_start - v_end);
  double temp_S;
  double temp_jerk = 0;

  temp_para = V_CalculateS(v_start, v_end, acc, jerk);
  temp_S = temp_para.S;

  if (temp_S > S) {
    // 提高加加速度，缩短时间
    // 判断是否需要提高加速度，如果按照最大加速度计算，是否满足路程要求
    // 满足始末速度的最小运动距离
    if ((v_start + v_end) / 2.0 * delta_v / acc > S) {
      // todo(maqun@buaa.edu.cn) 运动距离太小，加速度太小，速度变化太大
      // 速度规划异常，加速度超过最大加速度限制，始末速度设置不合理
      memset(res.acc, 0, sizeof(res.acc));
      memset(res.jerk, 0, sizeof(res.jerk));
      memset(res.t, 0, sizeof(res.t));
      memset(res.s, 0, sizeof(res.s));
      if (v_start < v_end) {  // 先加速
        res.s[1] = S;
        res.t[1] = S / ((v_start + v_end) / 2.0);
        res.acc[1] = delta_v / res.t[1];
        res.S = S;
        res.v[3] = res.v[2] = v_end;
        res.v[0] = res.v[1] = v_start;
      } else {  // 先减速
        res.s[2] = S;
        res.t[2] = S / ((v_start + v_end) / 2.0);
        res.acc[2] = delta_v / res.t[2];
        res.S = S;
        res.v[3] = res.v[2] = v_end;
        res.v[0] = res.v[1] = v_start;
      }

      return res;
    }
    // 迭代加加速度，求得一个比较合适的值
    temp_jerk = jerk;
    while (temp_S > S) {
      temp_jerk = 1.5 * temp_jerk;
      temp_para = V_CalculateS(v_start, v_end, acc, temp_jerk);
      temp_S = temp_para.S;
    }
    // 修改为一段加速-匀速或匀速-减速曲线
    if (v_start < v_end) {  // 先加速后匀速
      res.s[3] = S - temp_S;
      res.v[3] = v_end;
      res.t[3] = res.s[3] / v_end;
      res.acc[3] = 0;
      res.jerk[3] = 0;
      res.jerk[0] = temp_jerk;
      res.jerk[1] = 0;
      res.jerk[2] = temp_jerk;
      for (int i = 0; i < 3; ++i) {
        res.s[i] = temp_para.s[i + 1];
        // res.v[i] = temp_para.v[i + 1];
        res.t[i] = temp_para.t[i + 1];
        res.acc[i] = temp_para.acc[i + 1];
        res.S = S;
      }
      res.v[0] = v_start;
      res.v[1] = temp_para.v[1];
      res.v[2] = temp_para.v[2];
      return res;
    } else {  // 先匀速再减速
      res.s[0] = S - temp_S;
      res.v[0] = v_start;
      res.t[0] = res.s[0] / v_start;
      res.acc[0] = 0;
      res.jerk[0] = 0;
      res.jerk[1] = temp_jerk;
      res.jerk[2] = 0;
      res.jerk[3] = temp_jerk;
      for (int i = 1; i < 4; ++i) {
        res.s[i] = temp_para.s[i];
        res.v[i] = temp_para.v[i];
        res.t[i] = temp_para.t[i];
        res.acc[i] = temp_para.acc[i];
        res.S = S;
      }
      return res;
    }
  } else if (temp_S < S) {
    // 先匀速，再加速(或减速)，低速运行或高速运行
    res.s[0] = S - temp_S;
    res.t[0] = res.s[0] / v_start;
    res.v[0] = v_start;
  } else {
    res.s[0] = 0;
    res.t[0] = 0;
    res.v[0] = v_start;
  }
  // 可能会修改加加速度
  res.jerk[1] = temp_jerk;
  res.jerk[2] = 0;
  res.jerk[3] = temp_jerk;
  res.S = res.s[0];
  for (int i = 1; i < 4; ++i) {
    res.v[i] = temp_para.v[i];
    res.s[i] = temp_para.s[i];
    res.t[i] = temp_para.t[i];
    res.acc[i] = temp_para.acc[i];
    res.S += res.s[i];
  }
  return res;
}

T_VelocityProfile::T_VelocityProfile() {}

T_VelocityProfile::~T_VelocityProfile() {}

/**
 * @brief T型速度规划构造函数，输入相关参数并进行速度规划。
 * 默认情况下起始速度与终点速度都为0，减速度和加速度相同，也可以输入想要的参数
 *
 * @param v_max 最大速度
 * @param acc  加速度
 * @param Sf 总行程
 * @param v_start 起始速度
 * @param v_end 终点速度
 * @param dcc 减速度
 */
T_VelocityProfile::T_VelocityProfile(double v_max, double acc, double Sf,
                                     double v_start, double v_end, double dcc) {
  this->v_start_ = v_start;
  this->v_end_ = v_end;
  this->v_max_ = v_max;
  this->acc_ = acc;
  if (fabs(dcc - 0.0) < 1e-10) {
    this->dcc_ = dcc;
  } else {
    this->dcc_ = this->acc_;
  }
  this->Sf_ = Sf;
  this->VelocityProfile();
}

/**
 * @brief
 * 设置T型速度规划参数，共分为三个阶段，
 * acc为第一阶段加速度的绝对值，dcc为第三阶段加速度的绝对值，
 * 真实运动的加速或减速由初速度和末速度决定
 * @param v_max 最大速度
 * @param acc  加速度，取正值
 * @param Sf  最大行程
 * @param v_start  起始速度，默认为0
 * @param v_end  终点速度，默认为0
 * @param dcc   减速度，默认为加速度大小，取正值
 * @return error_t  返回值：0 正常, -1 错误
 */
error_t T_VelocityProfile::SetVelocityProfile(double v_max, double acc,
                                              double Sf, double v_start,
                                              double v_end, double dcc) {
  this->v_start_ = v_start;
  this->v_end_ = v_end;
  this->v_max_ = v_max;
  this->acc_ = acc;
  if (fabs(dcc - 0.0) < 1e-10) {
    this->dcc_ = acc;
  } else {
    this->dcc_ = dcc;
  }
  this->Sf_ = Sf;
  this->acc_direc_ = v_max - v_start > 0 ? 1 : -1;
  this->dcc_direc_ = v_max - v_end > 0 ? -1 : 1;
  return this->VelocityProfile();
}

/**
 * @brief 梯形速度规划
 * 设置起始速度和末速度，加速度和减速度不同
 *
 */
error_t T_VelocityProfile::VelocityProfile() {
  // 判断是否到达最大速度
  this->T[1] = fabs(this->v_max_ - this->v_start_) / this->acc_;
  this->T[3] = fabs(this->v_max_ - this->v_end_) / this->dcc_;
  this->S[1] = fabs(this->v_start_ + this->v_max_) * this->T[1] / 2.0;
  this->S[3] = fabs(this->v_end_ + this->v_max_) * this->T[3] / 2.0;
  if (S[1] + S[3] > this->Sf_) {
    // 加速和减速阶段的理论位移已经大于总位移，说明没有到达最大速度
    // 重新计算最大速度
    if (this->acc_direc_ > 0 && this->dcc_direc_ < 0) {
      // * 1. 先加速再减速
      this->v_max_ = sqrt((2 * this->Sf_ * this->acc_ * this->dcc_ +
                           this->dcc_ * pow(this->v_start_, 2) +
                           this->acc_ * pow(this->v_end_, 2)) /
                          (this->acc_ + this->dcc_));
      if (std::isnan(this->v_max_)) {
        // 异常处理：无法求解正确的T型速度规划
        return -1;
      }
      this->T[1] = fabs(this->v_max_ - this->v_start_) / this->acc_;
      this->T[2] = 0;
      this->T[3] = fabs(this->v_max_ - this->v_end_) / this->dcc_;
      this->S[1] = fabs(this->v_start_ + this->v_max_) * this->T[1] / 2.0;
      this->S[2] = 0;
      this->S[3] = this->Sf_ - this->S[1];
    } else if (this->acc_direc_ > 0 && this->dcc_direc_ > 0) {
      // * 2. 先加速再加速, 重新计算中间速度
      if (fabs(this->acc_ - this->dcc_) < 1e-10) {
        // 加速度和减速度相等
        this->v_max_ = (this->v_start_ + this->v_end_) / 2.0;
        // 重新计算加速度
        this->acc_ = this->dcc_ =
            (pow(this->v_end_, 2) - pow(this->v_start_, 2)) / this->Sf_ / 2.0;
      } else {
        this->v_max_ = sqrt((2 * this->Sf_ * this->acc_ * this->dcc_ -
                             this->acc_ * pow(this->v_end_, 2) +
                             this->dcc_ * pow(this->v_start_, 2)) /
                            (this->dcc_ - this->acc_));
        if (std::isnan(this->v_max_)) {
          // 异常处理：无法求解正确的T型速度规划
          return -1;
        }
      }

      this->T[1] = fabs(this->v_max_ - this->v_start_) / this->acc_;
      this->T[2] = 0;
      this->T[3] = fabs(this->v_max_ - this->v_end_) / this->dcc_;
      this->S[1] = fabs(this->v_start_ + this->v_max_) * this->T[1] / 2.0;
      this->S[2] = 0;
      this->S[3] = this->Sf_ - this->S[1];
    } else if (this->acc_direc_ < 0 && this->dcc_direc_ < 0) {
      // * 3. 先减速再减速
      if (fabs(this->acc_ - this->dcc_) < 1e-10) {
        // 加速度和减速度相等，重新计算加速度
        this->v_max_ = (this->v_start_ + this->v_end_) / 2.0;
        this->acc_ = this->dcc_ =
            (pow(this->v_start_, 2) - pow(this->v_end_, 2)) / this->Sf_ / 2.0;
      } else {
        this->v_max_ = sqrt((2 * this->Sf_ * this->acc_ * this->dcc_ +
                             this->acc_ * pow(this->v_end_, 2) -
                             this->dcc_ * pow(this->v_start_, 2)) /
                            (this->acc_ - this->dcc_));
        if (std::isnan(this->v_max_)) {
          // 异常处理：无法求解正确的T型速度规划
          return -1;
        }
      }
      this->T[1] = fabs(this->v_max_ - this->v_start_) / this->acc_;
      this->T[2] = 0;
      this->T[3] = fabs(this->v_max_ - this->v_end_) / this->dcc_;
      this->S[1] = fabs(this->v_start_ + this->v_max_) * this->T[1] / 2.0;
      this->S[2] = 0;
      this->S[3] = this->Sf_ - this->S[1];
    } else if (this->acc_direc_ < 0 && this->dcc_direc_ > 0) {
      // * 4. 先减速再加速
      this->v_max_ = sqrt((2 * this->Sf_ * this->acc_ * this->dcc_ -
                           this->acc_ * pow(this->v_end_, 2) -
                           this->dcc_ * pow(this->v_start_, 2)) /
                          (-this->acc_ - this->dcc_));
      if (std::isnan(this->v_max_)) {
        //  异常处理：无法求解正确的T型速度规划
        return -1;
      } else if (!(this->v_max_ < this->v_start_ &&
                   this->v_max_ < this->v_end_)) {
        // *异常处理：无法求解目标T型速度规划，修改为其他类型
        this->acc_ = this->dcc_ =
            fabs(pow(this->v_start_, 2) - pow(this->v_end_, 2)) / this->Sf_ /
            2.0;
        this->v_max_ = (this->v_start_ + this->v_end_) / 2.0;
        this->acc_direc_ = this->v_max_ - this->v_start_ > 0 ? 1 : -1;
        this->dcc_direc_ = this->v_max_ - this->v_end_ > 0 ? -1 : 1;
      }
      this->T[1] = fabs(this->v_max_ - this->v_start_) / this->acc_;
      this->T[2] = 0;
      this->T[3] = fabs(this->v_max_ - this->v_end_) / this->dcc_;
      this->S[1] = fabs(this->v_start_ + this->v_max_) * this->T[1] / 2.0;
      this->S[2] = 0;
      this->S[3] = this->Sf_ - this->S[1];
    }

  } else {
    // 到达中间速度，计算每一阶段的时间
    this->S[2] = this->Sf_ - this->S[1] - this->S[3];
    this->T[2] = this->S[2] / this->v_max_;
  }
  this->Tf_ = T[1] + T[2] + T[3];
  return 0;
}

/**
 * @brief 返回当前时间点的位置
 *
 * @param time
 * @return double
 */
double T_VelocityProfile::Pos(double time) {
  double p, t;
  if ((time > 0 && time < this->T[1]) || fabs(time - this->T[1]) < 1e-10 ||
      fabs(time - 0.0) < 1e-10) {
    // 处于第一阶段
    t = time;
    p = this->v_start_ * t + this->acc_ * t * t / 2.0 * this->acc_direc_;
  } else if ((time > this->T[1] && time < (this->T[1] + this->T[2])) ||
             fabs(time - this->T[1] - this->T[2]) < 1e-10) {
    // 处于第二阶段
    t = time - this->T[1];
    p = this->S[1] + this->v_max_ * t;
  } else if (time > (this->T[1] + this->T[2]) && time < this->Duration()) {
    // 处于第三阶段
    t = time - this->T[1] - this->T[2];
    p = this->S[1] + this->S[2] + this->v_max_ * t +
        this->dcc_ * t * t / 2.0 * this->dcc_direc_;
  } else if (time > this->Tf_ || fabs(time - this->Tf_) < 1e-10) {
    // 大于或等于运动总时长
    p = this->Sf_;
  } else {
    p = 0.0;
  }
  return p;
}

/**
 * @brief 返回当前时间点下的速度
 *
 * @param time
 * @return double
 */
double T_VelocityProfile::Vel(double time) {
  double v, t;
  if ((time > 0 && time < this->T[1]) || fabs(time - this->T[1]) < 1e-10 ||
      fabs(time - 0.0) < 1e-10) {
    // 处于第一阶段, 0 <= time <= T[1]
    t = time;
    v = this->v_start_ + this->acc_ * t;
  } else if ((time > this->T[1] && time < (this->T[1] + this->T[2])) ||
             fabs(time - this->T[1] - this->T[2]) < 1e-10) {
    // 处于第二阶段, T[1] < time <= T[1]+T[2]
    t = time - this->T[1];
    v = this->v_max_;
  } else if (time > (this->T[1] + this->T[2]) && time < this->Duration()) {
    // 处于第三阶段, T[1]+T[2] < time < T[1]+T[2]+T[3]
    t = time - this->T[1] - this->T[2];
    v = this->v_max_ - this->dcc_ * t;
  } else if (time > this->Tf_ || fabs(time - this->Tf_) < 1e-10) {
    // 大于或等于运动总时长, time >= duration
    v = this->v_end_;
  } else {
    v = this->v_start_;
  }
  return v;
}

/**
 * @brief 返回当前时间点下的加速度
 *
 * @param time
 * @return double
 */
double T_VelocityProfile::Acc(double time) {
  double a, t;
  if ((time > 0 && time < this->T[1]) || fabs(time - this->T[1]) < 1e-10 ||
      fabs(time - 0.0) < 1e-10) {
    // 处于第一阶段
    t = time;
    a = this->acc_;
  } else if ((time > this->T[1] && time < (this->T[1] + this->T[2])) ||
             fabs(time - this->T[1] - this->T[2]) < 1e-10) {
    // 处于第二阶段
    t = time - this->T[1];
    a = 0.0;
  } else if (time > (this->T[1] + this->T[2]) && time < this->Duration()) {
    // 处于第三阶段
    t = time - this->T[1] - this->T[2];
    a = this->dcc_;
  } else if (time > this->Tf_ || fabs(time - this->Tf_) < 1e-10) {
    // 大于或等于运动总时长
    a = this->dcc_;
  } else {
    a = this->acc_;
  }
  return a;
}

/**
 * @brief 返回当前轨迹运行总时间
 *
 * @return double
 */
double T_VelocityProfile::Duration() { return this->Tf_; }
/**
 * @brief 默认构造函数
 *
 */
S_VelocityProfile::S_VelocityProfile() {}

/**
 * @brief Construct a new s velocityprofile::s velocityprofile object
 *
 * @param v_max 最大速度
 * @param acc_max 最大加速度
 * @param jerk 加加速度
 * @param Sf 轨迹总长度
 * @param v_start 初速度
 * @param v_end 末速度
 * @param dcc_max 最大减速度
 * @param jerk_dcc 减减速度
 */
S_VelocityProfile::S_VelocityProfile(double v_max, double acc_max, double jerk,
                                     double Sf, double v_start, double v_end,
                                     double dcc_max, double jerk_dcc) {
  this->v_max_ = v_max;
  this->acc_max_ = acc_max;
  this->jerk_ = jerk;
  this->Sf_ = Sf;
  this->v_start_ = v_start;
  this->v_end_ = v_end;
  this->dcc_max_ = fabs(dcc_max) < 1e-6 ? acc_max : dcc_max;
  this->jerk_dcc_ = fabs(jerk_dcc) < 1e-6 ? jerk : jerk_dcc;
  // 判断曲线类型
  if (v_start < v_max && v_end < v_max) {
    this->traj_type_ = S_Vel_Type::Convexity_S;
  } else if (v_start > v_max && v_end > v_max) {
    this->traj_type_ = S_Vel_Type::Concavity_S;
  } else if (v_start >= v_max && v_max >= v_end) {
    this->traj_type_ = S_Vel_Type::Double_Dcc;
  } else if (v_start <= v_max && v_max <= v_end) {
    this->traj_type_ = S_Vel_Type::Double_Acc;
  }
  this->VelocityProfile(true);
}

/**
 * @brief Destroy the s velocityprofile::s velocityprofile object
 *
 */
S_VelocityProfile::~S_VelocityProfile() {}

/**
 * @brief  使用S型速度规划一段新的路径
 *
 * @param v_max 最大速度
 * @param acc_max 最大加速度
 * @param jerk 加加速度
 * @param Sf 轨迹总长度
 * @param v_start
 * @param v_end
 * @param dcc_max
 * @param jerk_dcc
 * @return error_t
 */
error_t S_VelocityProfile::SetVelocityProfile(double v_max, double acc_max,
                                              double jerk, double Sf,
                                              double v_start, double v_end,
                                              double dcc_max, double jerk_dcc) {
  this->v_max_ = v_max;
  this->acc_max_ = acc_max;
  this->jerk_ = jerk;
  this->Sf_ = Sf;
  this->v_start_ = v_start;
  this->v_end_ = v_end;
  this->dcc_max_ = fabs(dcc_max) < 1e-6 ? acc_max : dcc_max;
  this->jerk_dcc_ = fabs(jerk_dcc) < 1e-6 ? jerk : jerk_dcc;
  // 判断曲线类型
  if (v_start < v_max && v_end < v_max) {
    this->traj_type_ = S_Vel_Type::Convexity_S;
  } else if (v_start > v_max && v_end > v_max) {
    this->traj_type_ = S_Vel_Type::Concavity_S;
  } else if (v_start >= v_max && v_max >= v_end) {
    this->traj_type_ = S_Vel_Type::Double_Dcc;
  } else if (v_start <= v_max && v_max <= v_end) {
    this->traj_type_ = S_Vel_Type::Double_Acc;
  }
  error_t res = this->VelocityProfile(true);
  // PrintPara();
  return res;
}

/**
 * @brief s型速度规划计算
 * 默认加速阶段和减速阶段是对称的，初速度和末速度都是0
 */
error_t S_VelocityProfile::VelocityProfile() {
  // 判断是否到达了最大速度
  this->T[1] = this->acc_max_ / this->jerk_;

  this->T[2] = this->v_max_ / this->acc_max_ - T[1];
  this->T[3] = this->T[1];
  this->V[1] = 0;
  this->V[2] = jerk_ * T[1] * T[1] / 2;
  // 如果没有匀加速阶段，也没有匀速阶段，即速度没有达到最大，加速度也没达到最大
  if (this->jerk_ * pow(T[1], 3) * 2 >= this->Sf_) {
    double t = pow(this->Sf_ / this->jerk_ / 2.0, 1.0 / 3.0);
    // 计算时间
    this->T[1] = t;
    this->T[2] = 0;
    this->T[3] = t;
    this->T[4] = 0;
    this->T[5] = t;
    this->T[6] = 0;
    this->T[7] = t;
    this->Tf_ = 0;
    for (int i = 1; i < 8; i++) {
      this->Tf_ += T[i];
    }
    // 计算最大加速度
    // 计算速度
    this->V[1] = 0;
    this->V[2] = this->jerk_ * t * t / 2;
    this->V[3] = this->V[2];
    this->V[4] = this->jerk_ * t * t;
    this->V[5] = V[4];
    this->V[6] = V[3];
    this->V[7] = V[6];
    // 计算位移
    this->S[1] = this->jerk_ * pow(t, 3) / 6.0;
    this->S[2] = 0;
    this->S[3] = this->jerk_ * pow(t, 3) / 6.0 * 5.0;
    this->S[4] = 0;
    this->S[5] = S[3];
    this->S[6] = 0;
    this->S[7] = S[1];
    return 0;
  }

  if (V[2] * 2 > v_max_) {
    /* && 2 * jerk_ * pow(T[1], 3) < Sf_
     * 没有匀加速阶段，没有达到最大加速度
     * T[2]阶段时间为0
     * 加加速度太大导致没有匀加速阶段，便到达了最大速度v_max_
     * 没有达到最大加速度，acc_max_改变
     */
    T[1] = std::sqrt(v_max_ / jerk_);
    T[2] = 0;
    T[3] = T[1];
    acc_max_ = T[1] * jerk_;
    this->V[2] = jerk_ * T[1] * T[1] / 2;
    this->V[3] = V[2];
    this->V[4] = v_max_;
  } else {
    /*
     * 有匀加速阶段，达到最大加速度
     * 三个阶段，加加速度，匀加速， 减加速
     * 最大加速度和最大速度不变
     */
    this->V[3] = V[2] + acc_max_ * T[2];
    this->V[4] = v_max_;
  }

  // 第一段位移 s1 = J * t1 * t1 * t1 / 6
  this->S[1] = this->jerk_ * pow(T[1], 3) / 6;
  // 第二段位移
  this->S[2] = V[2] * T[2] + acc_max_ * T[2] * T[2] / 2;
  // 第三段位移
  this->S[3] =
      V[3] * T[3] + acc_max_ * T[3] * T[3] / 2 - jerk_ * pow(T[3], 3) / 6;

  if (2 * (S[1] + S[2] + S[3]) < Sf_) {
    // 存在匀速阶段，到达了最大速度
    // 如果不存在匀加速和匀减速阶段也没关系
    T[4] = (Sf_ - 2 * (S[1] + S[2] + S[3])) / v_max_;
    T[5] = T[3];
    T[6] = T[2];
    T[7] = T[1];
    this->Tf_ = 0;
    for (int i = 1; i < 8; i++) {
      this->Tf_ += T[i];
    }
    S[4] = v_max_ * T[4];
    S[5] = S[3];
    S[6] = S[2];
    S[7] = S[1];
    V[4] = v_max_;
    V[5] = v_max_;
    V[6] = V[3];
    V[7] = V[2];

  } else {  //! 不存在匀速阶段，没有到达最大速度，加速阶段结束之后就是减速阶段
    // 先判断前三个阶段中，是否到达了最大加速度，即存在匀加速阶段
    this->T[1] = acc_max_ / jerk_;
    this->T[3] = T[1];
    //? 能够达到的最大速度是多少呢，关键是求出T2
    // 求一元二次方程的解 s1 + s2 + s3 = Sf/2
    double a = acc_max_;
    double b = 3 * acc_max_ * T[1];
    double c = 2 * acc_max_ * T[1] * T[1] - Sf_;
    double deta = b * b - 4 * a * c;
    if (c <= 0 && deta >= 0) {
      /*
       * 方程有根且有一个大于0的正根
       * 存在匀加速阶段，即到达了最大加速度
       * 不存在T4阶段，存在T1、T2、T3、T5、T6、T7阶段
       */
      T[2] = (-b + sqrt(deta)) / 2 / a;
      T[4] = 0;
      T[5] = T[3];
      T[6] = T[2];
      T[7] = T[1];
      this->Tf_ = 0;
      for (int i = 1; i < 8; i++) {
        this->Tf_ += T[i];
      }

      S[1] = jerk_ * pow(T[1], 3) / 6;
      V[1] = 0;
      V[2] = jerk_ * T[1] * T[1] / 2;
      S[2] = V[2] * T[2] + acc_max_ * T[2] * T[2] / 2;
      V[3] = V[2] + acc_max_ * T[2];
      S[3] =
          V[3] * T[3] + acc_max_ * T[3] * T[3] / 2 - jerk_ * pow(T[3], 3) / 6;
      V[4] = acc_max_ * (T[1] + T[2]);
      V[5] = V[4];
      V[6] = V[3];
      V[7] = V[2];
      S[4] = 0;
      S[5] = S[3];
      S[6] = S[2];
      S[7] = S[1];

    } else {
      // 方程无根或只有负根，即不存在T2阶段，没有到达最大加速度，也没有达到最大速度
      // LOG(INFO) << "Velocity Porfile Error!";
      T[1] = pow(Sf_ / 2 / jerk_, 1.0 / 3);
      T[2] = 0;
      T[3] = T[1];
      T[4] = 0;
      T[5] = T[3];
      T[6] = T[2];
      T[7] = T[1];
      this->Tf_ = 0;
      for (int i = 1; i < 8; i++) {
        this->Tf_ += T[i];
      }
      acc_max_ = T[1] * jerk_;
      V[1] = 0;
      V[2] = jerk_ * pow(T[1], 2) / 2;
      V[3] = V[2];
      V[4] = jerk_ * pow(T[1], 2);
      V[5] = V[4];
      V[6] = V[3];
      V[7] = V[3];
    }
  }
  return 0;
}

/**
 * @brief 在初始速度和终点速度不为0的情况下进行S型速度规划
 *
 * @param s
 * @return error_t 0 规划成功，-1 失败
 */
error_t S_VelocityProfile::VelocityProfile(bool s) {
  // 起始速度不为0的s型速度规划
  if (this->traj_type_ == S_Vel_Type::Convexity_S) {
    // ****凸型S曲线
    S_Vel_Para Acc_period =
        V_CalculateS(this->v_start_, this->v_max_, this->acc_max_, this->jerk_);
    S_Vel_Para Dcc_period = V_CalculateS(this->v_max_, this->v_end_,
                                         this->dcc_max_, this->jerk_dcc_);
    double S_acc = Acc_period.S;
    double S_dcc = Dcc_period.S;
    if (S_acc + S_dcc > this->Sf_) {
      // ** 需要降低匀速运动阶段的最大速度
      // 加速和减速阶段运动的路程已经超过总路程，没有匀速阶段且需要调整加减速阶段
      double v_low =
          this->v_start_ > this->v_end_ ? this->v_start_ : this->v_end_;
      double v_high = v_max_;
      // **迭代求合适的匀速运动阶段的最大速度
      while (S_acc + S_dcc > this->Sf_) {
        v_high = v_high * 0.95;

        Acc_period =
            V_CalculateS(this->v_start_, v_high, this->acc_max_, this->jerk_);
        Dcc_period =
            V_CalculateS(v_high, this->v_end_, this->dcc_max_, this->jerk_dcc_);
        S_acc = Acc_period.S;
        S_dcc = Dcc_period.S;
        // todo 当路程较小，达不到最大速度，会出现规划不一致问题
        if (v_high < v_low) {
          break;
        }
      }
      this->v_max_ = v_high;
      if (v_high < v_low) {  // 由先加速，在减速的曲线蜕变为一段加速曲线
        if (this->v_start_ < this->v_end_) {
          Acc_period = V_CalculateS(this->v_start_, this->v_end_,
                                    this->acc_max_, this->jerk_);
          Dcc_period = V_CalculateS(this->v_end_, this->v_end_, this->dcc_max_,
                                    this->jerk_dcc_);
        } else {
          Dcc_period = V_CalculateS(this->v_start_, this->v_end_,
                                    this->acc_max_, this->jerk_);
          Acc_period = V_CalculateS(this->v_start_, this->v_start_,
                                    this->dcc_max_, this->jerk_dcc_);
        }
        S_acc = Acc_period.S;
        S_dcc = Dcc_period.S;
        this->v_max_ =
            this->v_start_ > this->v_end_ ? this->v_start_ : this->v_end_;
        // if (S_acc + S_dcc > Sf_) {
        //   return -1;  // 无法实现指定速度的规划
        // }
      }
    }
    // 存在匀速运动过程
    this->S[1] = Acc_period.s[1];
    this->S[2] = Acc_period.s[2];
    this->S[3] = Acc_period.s[3];

    this->S[4] = this->Sf_ - S_acc - S_dcc;
    this->T[4] = this->S[4] / this->v_max_;

    this->S[5] = Dcc_period.s[1];
    this->S[6] = Dcc_period.s[2];
    this->S[7] = Dcc_period.s[3];

    this->T[1] = Acc_period.t[1];
    this->T[2] = Acc_period.t[2];
    this->T[3] = Acc_period.t[3];

    this->T[5] = Dcc_period.t[1];
    this->T[6] = Dcc_period.t[2];
    this->T[7] = Dcc_period.t[3];

    this->V[1] = Acc_period.v[0];  // 第一阶段的起始速度
    this->V[2] = Acc_period.v[1];
    this->V[3] = Acc_period.v[2];
    this->V[4] = this->v_max_;
    this->V[5] = Dcc_period.v[0];
    this->V[6] = Dcc_period.v[1];
    this->V[7] = Dcc_period.v[2];

    this->A[1] = Acc_period.acc[1];
    this->A[2] = Acc_period.acc[2];
    this->A[3] = Acc_period.acc[3];
    this->A[4] = 0;
    this->A[5] = Dcc_period.acc[1];
    this->A[6] = Dcc_period.acc[2];
    this->A[7] = Dcc_period.acc[3];

    this->J[1] = Acc_period.jerk[1];
    this->J[2] = 0;
    this->J[3] = Acc_period.jerk[3];
    this->J[4] = 0;
    this->J[5] = Dcc_period.jerk[1];
    this->J[6] = 0;
    this->J[7] = Dcc_period.jerk[3];
  } else if (this->traj_type_ == S_Vel_Type::Double_Acc ||
             this->traj_type_ == S_Vel_Type::Double_Dcc) {
    // ****两段加速曲线 或者 两段减速曲线
    S_Vel_Para Acc_period =
        V_CalculateS(this->v_start_, this->v_max_, this->acc_max_, this->jerk_);
    S_Vel_Para Dcc_period =
        V_CalculateS(this->v_max_, this->v_end_, this->acc_max_, this->jerk_);
    double S_acc = Acc_period.S;  // 第一段变速轨迹长度
    double S_dcc = Dcc_period.S;  // 第二段变速轨迹长度
    if (S_acc + S_dcc > this->Sf_) {
      // 不存在匀速阶段，按照一段加速或一段减速轨迹去模拟
      S_Vel_Para temp_S = S_CalculateV(this->v_start_, this->v_end_,
                                       this->acc_max_, this->jerk_, this->Sf_);
      // 7个阶段变为4个阶段，可能存在匀速阶段，也可能不存在
      if (this->v_start_ < this->v_end_) {  // 先加速，在匀速
        for (int i = 0; i < 4; i++) {
          this->S[i + 1] = temp_S.s[i];
          this->T[i + 1] = temp_S.t[i];
          this->A[i + 1] = temp_S.acc[i];
          this->J[i + 1] = temp_S.jerk[i];
          this->V[i + 1] = temp_S.v[i];
        }
        this->V[1] = this->v_start_;
        this->S[5] = this->S[6] = this->S[7] = 0;
        this->T[5] = this->T[6] = this->T[7] = 0;
        this->V[5] = this->V[6] = this->V[7] = this->v_end_;
      } else {
        this->S[1] = this->S[2] = this->S[3] = 0;
        this->T[1] = this->T[2] = this->T[3] = 0;
        this->V[1] = this->V[2] = this->V[3] = this->v_start_;

        this->S[4] = temp_S.s[0];
        this->S[5] = temp_S.s[1];
        this->S[6] = temp_S.s[2];
        this->S[7] = temp_S.s[3];

        this->T[4] = temp_S.t[0];
        this->T[5] = temp_S.t[1];
        this->T[6] = temp_S.t[2];
        this->T[7] = temp_S.t[3];

        this->V[4] = this->v_start_;
        this->V[5] = temp_S.v[0];
        this->V[6] = temp_S.v[1];
        this->V[7] = temp_S.v[2];

        this->A[1] = temp_S.acc[1];
        this->A[2] = temp_S.acc[2];
        this->A[3] = temp_S.acc[3];
        this->A[4] = 0;
        this->A[5] = temp_S.acc[1];
        this->A[6] = temp_S.acc[2];
        this->A[7] = temp_S.acc[3];

        this->J[1] = temp_S.jerk[1];
        this->J[2] = 0;
        this->J[3] = temp_S.jerk[3];
        this->J[4] = 0;
        this->J[5] = temp_S.jerk[1];
        this->J[6] = 0;
        this->J[7] = temp_S.jerk[3];
      }

    } else {
      // 存在匀速阶段
      this->S[1] = Acc_period.s[1];
      this->S[2] = Acc_period.s[2];
      this->S[3] = Acc_period.s[3];

      this->S[4] = this->Sf_ - S_acc - S_dcc;
      this->T[4] = this->S[4] / this->v_max_;

      this->S[5] = Dcc_period.s[1];
      this->S[6] = Dcc_period.s[2];
      this->S[7] = Dcc_period.s[3];

      this->T[1] = Acc_period.t[1];
      this->T[2] = Acc_period.t[2];
      this->T[3] = Acc_period.t[3];

      this->T[5] = Dcc_period.t[1];
      this->T[6] = Dcc_period.t[2];
      this->T[7] = Dcc_period.t[3];

      this->V[1] = Acc_period.v[0];
      this->V[2] = Acc_period.v[1];
      this->V[3] = Acc_period.v[2];
      this->V[4] = this->v_max_;
      this->V[5] = Dcc_period.v[0];
      this->V[6] = Dcc_period.v[1];
      this->V[7] = Dcc_period.v[2];

      this->A[1] = Acc_period.acc[1];
      this->A[2] = Acc_period.acc[2];
      this->A[3] = Acc_period.acc[3];
      this->A[4] = 0;
      this->A[5] = Dcc_period.acc[1];
      this->A[6] = Dcc_period.acc[2];
      this->A[7] = Dcc_period.acc[3];

      this->J[1] = Acc_period.jerk[1];
      this->J[2] = 0;
      this->J[3] = Acc_period.jerk[3];
      this->J[4] = 0;
      this->J[5] = Dcc_period.jerk[1];
      this->J[6] = 0;
      this->J[7] = Dcc_period.jerk[3];
    }
  } else if (this->traj_type_ == S_Vel_Type::Concavity_S) {
    // ************凹S型曲线，匀速阶段速度均小于初速度和末速度
    S_Vel_Para Acc_period =
        V_CalculateS(this->v_start_, this->v_max_, this->acc_max_, this->jerk_);
    S_Vel_Para Dcc_period =
        V_CalculateS(this->v_max_, this->v_end_, this->acc_max_, this->jerk_);
    double S_acc = Acc_period.S;  // 先减速
    double S_dcc = Dcc_period.S;  // 再加速

    // 需要判断一下速度规划是否合理，如果无法满足凹形曲线，那就忽略中间速度
    if (V_CalculateS(v_start_, v_end_, acc_max_, jerk_).S > Sf_) {
      // 单向的加减速都比要求的速度大，那么不满足速度要求
      // todo(maqun@buaa.eud.cn) 异常处理
      std::cout << "不满足曲线规划要求" << std::endl;
      return -1;
      // 直接修改为单程加减速来拟合始末速度和位移距离
      if (v_start_ >= v_end_) {
        Acc_period = S_CalculateV(v_start_, v_end_, acc_max_, jerk_, Sf_);
        S_acc = Acc_period.S;
        Dcc_period = S_CalculateV(v_start_, v_end_, acc_max_, jerk_, 0.0);
        S_dcc = Dcc_period.S;
      } else {
        Dcc_period = S_CalculateV(v_start_, v_end_, acc_max_, jerk_, Sf_);
        S_dcc = Dcc_period.S;
        Acc_period = S_CalculateV(v_start_, v_end_, acc_max_, jerk_, 0.0);
        S_acc = Acc_period.S;
      }
    }

    if (S_acc + S_dcc > this->Sf_) {
      // 加速和减速阶段运动的路程已经超过总路程，没有匀速阶段且需要调整加减速阶段
      double v_high =
          this->v_start_ > this->v_end_ ? this->v_start_ : this->v_end_;
      double v_low = v_max_;
      // todo(maqun@buaa.edu.cn) 使用二分法求合适的匀速运动阶段的最大速度
      while (S_acc + S_dcc > this->Sf_) {
        v_low = (v_low + v_high) / 2;

        Acc_period =
            V_CalculateS(this->v_start_, v_low, this->acc_max_, this->jerk_);
        S_acc = Acc_period.S;
        Dcc_period =
            V_CalculateS(v_low, this->v_end_, this->acc_max_, this->jerk_);
        S_dcc = Dcc_period.S;
      }
      this->v_max_ = v_low;
    }
    // 存在匀速运动过程
    this->S[1] = Acc_period.s[1];
    this->S[2] = Acc_period.s[2];
    this->S[3] = Acc_period.s[3];

    this->S[4] = this->Sf_ - S_acc - S_dcc;
    this->T[4] = this->S[4] / this->v_max_;

    this->S[5] = Dcc_period.s[1];
    this->S[6] = Dcc_period.s[2];
    this->S[7] = Dcc_period.s[3];

    this->T[1] = Acc_period.t[1];
    this->T[2] = Acc_period.t[2];
    this->T[3] = Acc_period.t[3];

    this->T[5] = Dcc_period.t[1];
    this->T[6] = Dcc_period.t[2];
    this->T[7] = Dcc_period.t[3];

    this->V[1] = Acc_period.v[0];  // 第一阶段的起始速度
    this->V[2] = Acc_period.v[1];
    this->V[3] = Acc_period.v[2];
    this->V[4] = this->v_max_;
    this->V[5] = Dcc_period.v[0];
    this->V[6] = Dcc_period.v[1];
    this->V[7] = Dcc_period.v[2];

    this->A[1] = Acc_period.acc[1];
    this->A[2] = Acc_period.acc[2];
    this->A[3] = Acc_period.acc[3];
    this->A[4] = 0;
    this->A[5] = Dcc_period.acc[1];
    this->A[6] = Dcc_period.acc[2];
    this->A[7] = Dcc_period.acc[3];

    this->J[1] = Acc_period.jerk[1];
    this->J[2] = 0;
    this->J[3] = Acc_period.jerk[3];
    this->J[4] = 0;
    this->J[5] = Dcc_period.jerk[1];
    this->J[6] = 0;
    this->J[7] = Dcc_period.jerk[3];
  }
  this->Tf_ = 0;
  for (int i = 1; i < 8; ++i) {
    this->Tf_ += this->T[i];
  }
  return 0;
}

/**
 * @brief 根据时间获取当前轨迹的位置
 *
 * @param time
 * @return double
 */
double S_VelocityProfile::Pos(double time) {
  if (time < 0)
    return -1;
  double position = 0.0;
  double t[8];  // t[i]是T[i]阶段结束的时间
  t[0] = 0;
  for (int i = 1; i < 8; ++i) {
    t[i] = T[i] + t[i - 1];
  }
  if (time >= 0 && time < t[1]) {  // 处于第一阶段，加加速阶段
    // *根据曲线类型判断第一阶段是加速还是减速
    if (this->traj_type_ == S_Vel_Type::Convexity_S ||
        this->traj_type_ == S_Vel_Type::Double_Acc) {
      position = this->V[1] * time + this->J[1] * pow(time, 3) / 6.0;
    } else if (this->traj_type_ == S_Vel_Type::Concavity_S ||
               this->traj_type_ == S_Vel_Type::Double_Dcc) {
      position = this->V[1] * time - this->J[1] * pow(time, 3) / 6.0;
    }
    // position = this->jerk_ * pow(time, 3) / 6;
    return position;
  } else if (time >= t[1] && time < t[2]) {  // 处于第二阶段，匀加速阶段
    time = time - t[1];
    // *根据曲线类型判断第二阶段是加速还是减速
    if (this->traj_type_ == S_Vel_Type::Convexity_S ||
        this->traj_type_ == S_Vel_Type::Double_Acc) {
      // 加速
      position = S[1] + V[2] * time + A[2] * pow(time, 2) / 2.0;
    } else if (this->traj_type_ == S_Vel_Type::Concavity_S ||
               this->traj_type_ == S_Vel_Type::Double_Dcc) {
      // 减速
      position = S[1] + V[2] * time - A[2] * pow(time, 2) / 2.0;
    }

    return position;
  } else if (time >= t[2] && time < t[3]) {  // 处于第三个阶段，减加速阶段
    time = time - t[2];
    if (this->traj_type_ == S_Vel_Type::Convexity_S ||
        this->traj_type_ == S_Vel_Type::Double_Acc) {
      // 加速 s = v3*t + a*t*t/2 - j*t*t*t/6
      position = S[1] + S[2] + V[3] * time + A[3] * pow(time, 2) / 2.0 -
                 J[3] * pow(time, 3) / 6.0;
    } else if (this->traj_type_ == S_Vel_Type::Concavity_S ||
               this->traj_type_ == S_Vel_Type::Double_Dcc) {
      // 减速
      position = S[1] + S[2] + V[3] * time - A[3] * pow(time, 2) / 2.0 +
                 J[3] * pow(time, 3) / 6.0;
    }

    return position;
  } else if (time >= t[3] && time < t[4]) {  // 处于第四个阶段，匀速阶段
    time = time - t[3];

    position = this->S[1] + S[2] + S[3] + this->V[4] * time;
    return position;
  } else if (time >= t[4] && time < t[5]) {  // 处于第五个阶段，加减速阶段
    time = time - t[4];
    if (this->traj_type_ == S_Vel_Type::Concavity_S ||
        this->traj_type_ == S_Vel_Type::Double_Acc) {
      // 加速
      position =
          S[1] + S[2] + S[3] + S[4] + V[5] * time + J[5] * pow(time, 3) / 6.0;
    } else if (this->traj_type_ == S_Vel_Type::Convexity_S ||
               this->traj_type_ == S_Vel_Type::Double_Dcc) {
      // 减速
      position =
          S[1] + S[2] + S[3] + S[4] + V[5] * time - J[5] * pow(time, 3) / 6.0;
    }
    // position = S[1] + S[2] + S[3] + S[4] + V[5] * time -
    //            this->jerk_ * pow(time, 3) / 6;
    return position;
  } else if (time >= t[5] && time < t[6]) {  // 处于第六个阶段，匀减速阶段
    time = time - t[5];
    if (this->traj_type_ == S_Vel_Type::Concavity_S ||
        this->traj_type_ == S_Vel_Type::Double_Acc) {
      // 加速
      position = S[1] + S[2] + S[3] + S[4] + S[5] + V[6] * time +
                 A[6] * pow(time, 2) / 2.0;
    } else if (this->traj_type_ == S_Vel_Type::Convexity_S ||
               this->traj_type_ == S_Vel_Type::Double_Dcc) {
      // 减速
      position = S[1] + S[2] + S[3] + S[4] + S[5] + V[6] * time -
                 A[6] * pow(time, 2) / 2.0;
    }
    // position = S[1] + S[2] + S[3] + S[4] + S[5] + V[6] * time -
    //            this->jerk_ * T[1] * pow(time, 2) / 2;
    return position;
  } else if (time >= t[6] && time < t[7]) {  // 处于第七个阶段，减减速阶段
    time = time - t[6];
    if (this->traj_type_ == S_Vel_Type::Concavity_S ||
        this->traj_type_ == S_Vel_Type::Double_Acc) {
      // 加速
      position = S[1] + S[2] + S[3] + S[4] + S[5] + S[6] + V[7] * time +
                 A[7] * pow(time, 2) / 2.0 - J[7] * pow(time, 3) / 6.0;
    } else if (this->traj_type_ == S_Vel_Type::Convexity_S ||
               this->traj_type_ == S_Vel_Type::Double_Dcc) {
      // 减速
      position = S[1] + S[2] + S[3] + S[4] + S[5] + S[6] + V[7] * time -
                 A[7] * pow(time, 2) / 2.0 + J[7] * pow(time, 3) / 6.0;
    }
    return position;
  } else if (time >= t[7]) {
    position = this->Sf_;
    return position;
  }
  return position;
}

/**
 * @brief 根据时间获取当前轨迹的加速度
 *
 * @param time
 * @return double
 */
double S_VelocityProfile::Acc(double time) {
  double acceleration = 0;
  if (time < 0) {
    return -1;
  }
  double t[8];  // t[i]是T[i]阶段结束的时间
  t[0] = 0;
  for (int i = 1; i < 8; ++i) {
    t[i] = T[i] + t[i - 1];
  }
  if (time >= 0 && time < t[1]) {  // 加加速阶段，第一阶段
    if (traj_type_ == S_Vel_Type::Convexity_S ||
        traj_type_ == S_Vel_Type::Double_Acc) {
      // 加速
      acceleration = J[1] * time;
    } else if (traj_type_ == S_Vel_Type::Concavity_S ||
               traj_type_ == S_Vel_Type::Double_Dcc) {
      // 减速
      acceleration = -J[1] * time;
    }
  } else if (time >= t[1] && time < t[2]) {  // 匀加速阶段，加速度不变，第二阶段
    if (traj_type_ == S_Vel_Type::Convexity_S ||
        traj_type_ == S_Vel_Type::Double_Acc) {
      // 加速
      acceleration = A[2];
    } else if (traj_type_ == S_Vel_Type::Concavity_S ||
               traj_type_ == S_Vel_Type::Double_Dcc) {
      // 减速
      acceleration = -A[2];
    }
  } else if (time >= t[2] && time < t[3]) {  // 减加速阶段，第三阶段
    time = time - t[2];
    if (traj_type_ == S_Vel_Type::Convexity_S ||
        traj_type_ == S_Vel_Type::Double_Acc) {
      // 加速
      acceleration = A[3] - J[3] * time;
    } else if (traj_type_ == S_Vel_Type::Concavity_S ||
               traj_type_ == S_Vel_Type::Double_Dcc) {
      // 减速
      acceleration = -A[3] + J[3] * time;
    }
    acceleration = jerk_ * (T[1] - time);
  } else if (time >= t[3] && time < t[4]) {  // 匀速阶段，第四阶段
    time = time - t[3];
    acceleration = 0.0;
  } else if (time >= t[4] && time < t[5]) {  // 加减速阶段， 第五阶段
    time = time - t[4];
    if (traj_type_ == S_Vel_Type::Convexity_S ||
        traj_type_ == S_Vel_Type::Double_Acc) {
      // 加速
      acceleration = A[5] + J[5] * time;
    } else if (traj_type_ == S_Vel_Type::Concavity_S ||
               traj_type_ == S_Vel_Type::Double_Dcc) {
      // 减速
      acceleration = A[5] - J[5] * time;
    }
  } else if (time >= t[5] && time < t[6]) {  // 匀减速阶段，第六阶段
    time = time - t[5];
    if (traj_type_ == S_Vel_Type::Convexity_S ||
        traj_type_ == S_Vel_Type::Double_Acc) {
      // 加速
      acceleration = A[6];
    } else if (traj_type_ == S_Vel_Type::Concavity_S ||
               traj_type_ == S_Vel_Type::Double_Dcc) {
      // 减速
      acceleration = -A[6];
    }
  } else if (time >= t[6] && time < t[7]) {  // 减减速阶段，第七阶段
    time = time - t[6];
    if (traj_type_ == S_Vel_Type::Convexity_S ||
        traj_type_ == S_Vel_Type::Double_Acc) {
      // 加速
      acceleration = A[7] - J[7] * time;
    } else if (traj_type_ == S_Vel_Type::Concavity_S ||
               traj_type_ == S_Vel_Type::Double_Dcc) {
      // 减速
      acceleration = -A[7] + J[7] * time;
    }
  } else if (time >= t[7]) {
    acceleration = 0.0;
  }

  return acceleration;
}

/**
 * @brief 获取轨迹当前时下间的速度
 *
 * @param time
 * @return double
 */
double S_VelocityProfile::Vel(double time) {
  double velocity = 0;
  if (time < 0) {
    return -1;
  }
  double t[8];  // t[i]是T[i]阶段结束的时间
  t[0] = 0;
  for (int i = 1; i < 8; ++i) {
    t[i] = T[i] + t[i - 1];
  }

  if (time >= 0 && time < t[1]) {  // 加加速阶段 1
    if (traj_type_ == S_Vel_Type::Convexity_S ||
        traj_type_ == S_Vel_Type::Double_Acc) {  // 加速曲线
      velocity = V[1] + J[1] * pow(time, 2) / 2.0;
    } else if (traj_type_ == S_Vel_Type::Concavity_S ||
               traj_type_ == S_Vel_Type::Double_Dcc) {  // 减速曲线
      velocity = V[1] - J[1] * pow(time, 2) / 2.0;
    }
  } else if (time >= t[1] && time < t[2]) {  // 匀加速阶段，加速度不变 2
    time = time - t[1];
    if (traj_type_ == S_Vel_Type::Convexity_S ||
        traj_type_ == S_Vel_Type::Double_Acc) {  // 加速曲线
      velocity = V[2] + A[2] * time;
    } else if (traj_type_ == S_Vel_Type::Concavity_S ||
               traj_type_ == S_Vel_Type::Double_Dcc) {  // 减速曲线
      velocity = V[2] - A[2] * time;
    }
  } else if (time >= t[2] && time < t[3]) {  // 减加速阶段 3
    time = time - t[2];
    if (traj_type_ == S_Vel_Type::Convexity_S ||
        traj_type_ == S_Vel_Type::Double_Acc) {  // 加速曲线
      velocity = V[3] + A[3] * time - J[3] * pow(time, 2) / 2.0;
    } else if (traj_type_ == S_Vel_Type::Concavity_S ||
               traj_type_ == S_Vel_Type::Double_Dcc) {  // 减速曲线
      velocity = V[3] - A[3] * time + J[3] * pow(time, 2) / 2.0;
    }
  } else if (time >= t[3] && time < t[4]) {  // 匀速阶段 4
    time = time - t[3];
    velocity = V[4];
  } else if (time >= t[4] && time < t[5]) {  // 加减速阶段 5
    time = time - t[4];
    if (traj_type_ == S_Vel_Type::Concavity_S ||
        traj_type_ == S_Vel_Type::Double_Acc) {
      // 加速
      velocity = V[5] + J[5] * pow(time, 2) / 2.0;
    } else if (traj_type_ == S_Vel_Type::Convexity_S ||
               traj_type_ == S_Vel_Type::Double_Dcc) {
      // 减速
      velocity = V[5] - J[5] * pow(time, 2) / 2.0;
    }
  } else if (time >= t[5] && time < t[6]) {  // 匀减速阶段6
    time = time - t[5];
    if (traj_type_ == S_Vel_Type::Concavity_S ||
        traj_type_ == S_Vel_Type::Double_Acc) {
      // 加速
      velocity = V[6] + A[6] * time;
    } else if (traj_type_ == S_Vel_Type::Convexity_S ||
               traj_type_ == S_Vel_Type::Double_Dcc) {
      // 减速
      velocity = V[6] - A[6] * time;
    }
  } else if (time >= t[6] && time < t[7]) {  // 减减速阶段 7
    time = time - t[6];
    if (traj_type_ == S_Vel_Type::Concavity_S ||
        traj_type_ == S_Vel_Type::Double_Acc) {
      // 加速
      velocity = V[7] + A[7] * time - J[7] * pow(time, 2) / 2.0;
    } else if (traj_type_ == S_Vel_Type::Convexity_S ||
               traj_type_ == S_Vel_Type::Double_Dcc) {
      // 减速
      velocity = V[7] - A[7] * time + J[7] * pow(time, 2) / 2.0;
    }
  } else if (time >= t[7]) {
    velocity = v_end_;
  }

  return velocity;
}

/**
 * @brief 根据时间计算加加速度
 *
 * @param time
 * @return double
 */
double S_VelocityProfile::Jerk(double time) {
  double jerk = 0;
  if (time < 0) {
    return -1;
  }
  double t[8];  // t[i]是T[i]阶段结束的时间
  t[0] = 0;
  for (int i = 1; i < 8; ++i) {
    t[i] = T[i] + t[i - 1];
  }

  if (time >= 0 && time < t[1]) {  // 加加速阶段 1
    jerk = this->jerk_;
  } else if (time >= t[1] && time < t[2]) {  // 匀加速阶段，加速度不变 2
    jerk = 0.0;
  } else if (time >= t[2] && time < t[3]) {  // 减加速阶段 3
    jerk = -this->jerk_;
  } else if (time >= t[3] && time < t[4]) {  // 匀速阶段 4
    jerk = 0.0;
  } else if (time >= t[4] && time < t[5]) {  // 加减速阶段 5
    jerk = -this->jerk_;
  } else if (time >= t[5] && time < t[6]) {  // 匀减速阶段6
    jerk = 0.0;
  } else if (time >= t[6] && time < t[7]) {  // 减减速阶段 7
    jerk = this->jerk_;
  } else if (time >= t[7]) {
    jerk = 0.0;
  }
  return jerk;
}
/**
 * @brief 获取当前轨迹运行的总时间
 *
 * @return double
 */
double S_VelocityProfile::Duration() { return this->Tf_; }

TrajectoryRobot::TrajectoryRobot(int dof) {
  this->dof_ = dof;
  for (int i = 0; i < dof_; ++i) {
    this->s_vp_[i] = new S_VelocityProfile();
    this->t_vp_[i] = new T_VelocityProfile();
  }
  this->bzer_ = new Bezier();
  this->jnt_start_ = KDL::JntArray(dof_);
  this->jnt_end_ = KDL::JntArray(dof_);
  this->last_jnt_for_inverse_ = KDL::JntArray(dof_);
}
/**
 * @brief 轨迹构造函数，对指针数组初始化
 *
 */
TrajectoryRobot::TrajectoryRobot() {
  this->dof_ = 6;
  for (int i = 0; i < dof_; ++i) {
    this->s_vp_[i] = new S_VelocityProfile();
    this->t_vp_[i] = new T_VelocityProfile();
  }
  this->bzer_ = new Bezier();
  this->jnt_start_ = KDL::JntArray(dof_);
  this->jnt_end_ = KDL::JntArray(dof_);
  this->last_jnt_for_inverse_ = KDL::JntArray(dof_);
}

/**
 * @brief 轨迹析构函数，判断指针数组是否为null，析构
 *
 */
TrajectoryRobot::~TrajectoryRobot() {
  for (int i = 0; i < dof_; ++i) {
    delete this->s_vp_[i];
    delete this->t_vp_[i];
  }
  delete this->bzer_;
}

/**
 * @brief 设置轨迹参数并进行s型轨迹规划
 *
 * @param start 轨迹起点
 * @param end   轨迹终点
 * @param v_max 轨迹运行最大速度
 * @param acc_max 轨迹运行最大及速度
 * @param jerk   轨迹运行加加速度
 * @return error_t 返回值：0 正常，-1 第一个关节规划失败， -2
 *第二个关节规划失败。。。
 */
error_t TrajectoryRobot::SetTrajectory(double start[], double end[],
                                       double v_max[], double acc_max[],
                                       double jerk[], double dcc_max[],
                                       double dcc_jerk[]) {
  this->is_motion_line_ = true;
  std::memcpy(this->start_, start, sizeof(double) * dof_);
  std::memcpy(this->end_, end, sizeof(double) * dof_);
  double time = 0.0;
  for (int i = 0; i < dof_; i++) {
    S[i] = fabs(end[i] - start[i]);
  }

  // 先计算每个关节运动的最大用时
  for (int i = 0; i < dof_; ++i) {
    double err = this->s_vp_[i]->SetVelocityProfile(
        v_max[i], acc_max[i], jerk[i], S[i], 0, 0, dcc_max[i], dcc_jerk[i]);
    if (err < 0) {
      return -(i + 1);
    }
    T[i] = this->s_vp_[i]->Duration();
    if (T[i] > time) {
      time = T[i];
    }
  }
  this->Tf_ = time;
  return 0;
}

/**
 * @brief 设置一段轴运动轨迹，关节角度 moveABSJ()
 *
 * @param start 起点
 * @param end 终点
 * @param v_max 最大速度
 * @param acc_max 最大加速度
 * @param jerk 加加速度
 * @return error_t 返回值：0 正常, -1 第一个关节错误，-2 第二个关节错误...
 */
error_t TrajectoryRobot::SetTrajectoryAngle(const KDL::JntArray &start,
                                            const KDL::JntArray &end,
                                            double v_max[], double acc_max[],
                                            double jerk[]) {
  this->jnt_start_ = start;
  this->jnt_end_ = end;
  double T[kDofMax];
  double time = 0.0;
  double S[kDofMax];
  for (int i = 0; i < dof_; ++i) {
    S[i] = fabs(end(i) - start(i));
  }

  for (int i = 0; i < dof_; ++i) {
    double err =
        this->s_vp_[i]->SetVelocityProfile(v_max[i], acc_max[i], jerk[i], S[i]);
    if (err < 0) {
      return -(i + 1);
    }
    T[i] = this->s_vp_[i]->Duration();
    if (T[i] > time) {
      time = T[i];
    }
  }
  this->Tf_ = time;
  return 0;
}

/**
 * @brief 关节笛卡尔运动规划，起点终点为关节坐标，以直线相连
 *
 * @param start
 * @param end
 * @param v_max
 * @param acc_max
 * @param jerk
 * @return error_t
 */
error_t TrajectoryRobot::SetTrajectoryCartesianAxis(KDL::Frame start,
                                                    KDL::Frame end,
                                                    double v_max,
                                                    double acc_max,
                                                    double jerk) {
  KDL::Vector start_vec, end_vec;

  start_vec = start.p;
  end_vec = end.p;
  double r_start, p_start, y_start;
  double r_end, p_end, y_end;
  start.M.GetRPY(r_start, p_start, y_start);
  end.M.GetRPY(r_end, p_end, y_end);
  // 总行程
  double S = sqrt(pow(start_vec.x() - end_vec.x(), 2) +
                  pow(start_vec.y() - end_vec.y(), 2) +
                  pow(start_vec.z() - end_vec.z(), 2));
  double angle = sqrt(pow(r_start - r_end, 2) + pow(p_start - p_end, 2) +
                      pow(y_start - y_end, 2));
  this->s_vp_[0]->SetVelocityProfile(v_max, acc_max, jerk, S);
  this->s_vp_[1]->SetVelocityProfile(v_max, acc_max, jerk, angle);

  this->Sf_ = S;
  this->Tf_ = this->s_vp_[0]->Duration();
  this->start_vec_ = start_vec;
  this->end_vec_ = end_vec;
  this->start_cart_ = start;
  this->end_cart_ = end;

  return 0;
}

/********************************************************************
 * @brief 重新规划路径，减速到停止
 *
 ********************************************************************/
void TrajectoryRobot::ReplanTrajecotryAixsStop(double cur_traj_time) {
  // 针对每个关节都进行replan
  this->Tf_ = 0;
  // double t_full = this->Duration();
  for (int i = 0; i < dof_; ++i) {
    if (this->s_vp_[i]->Duration() > 0.001) {
      // replan
      this->s_vp_[i]->ReplanStop(cur_traj_time);
      if (this->Tf_ < this->s_vp_[i]->Duration()) {
        this->Tf_ = this->s_vp_[i]->Duration();
      }
      this->T[i] = this->s_vp_[i]->Duration();
    }
  }
}

/**
 * @brief
 *
 * @param time
 * @return RobotPos
 */
void TrajectoryRobot::Pos_cartesian(double time, KDL::Frame *pos) {
  if (Sf_ < 1e-6) {
    // pos->p = start_vec_;
    *pos = start_cart_;
    return;
  }
  if (time < 0) {
    pos->p = this->start_vec_;
  } else if (time > this->Tf_) {
    pos->p = this->end_vec_;
  }
  KDL::Vector cur_vec;
  double d = this->s_vp_[0]->Pos(time);
  if ((int)(time * 1000) % 10 == 0) {
    std::cout << time << " " << d << std::endl;
  }
  cur_vec.data[0] = d * (end_vec_.x() - start_vec_.x()) / Sf_ + start_vec_.x();
  cur_vec.data[1] = d * (end_vec_.y() - start_vec_.y()) / Sf_ + start_vec_.y();
  cur_vec.data[2] = d * (end_vec_.z() - start_vec_.z()) / Sf_ + start_vec_.z();
  // todo 笛卡尔的旋转运动
  pos->p = cur_vec;
  pos->M = start_cart_.M;
}

/**
 * @brief 返回当前时间下的位置
 *
 * @param time
 * @return RobotPos
 */
RobotPos TrajectoryRobot::Pos(double time) { return RobotPos(); }

/**
 * @brief
 *
 * @param time
 * @param next_pos
 **/
void TrajectoryRobot::Pos_Axis(double time, double *next_pos) {
  if (this->Duration() < 1e-6) {
    for (int i = 0; i < dof_; ++i) {
      next_pos[i] = start_[i];
    }
    return;
  }
  double t_full = this->Duration();
  for (int i = 0; i < dof_; ++i) {
    if (this->end_[i] < this->start_[i]) {
      next_pos[i] = start_[i] - this->s_vp_[i]->Pos(time / t_full *
                                                    this->s_vp_[i]->Duration());
    } else {
      next_pos[i] = start_[i] + this->s_vp_[i]->Pos(time / t_full *
                                                    this->s_vp_[i]->Duration());
    }
  }
}

/**
 * @brief 返回当前时间轨迹的轴角度
 *
 * @param time
 * @return KDL::JntArray
 */
KDL::JntArray TrajectoryRobot::Pos_J(double time) {
  KDL::JntArray pos(dof_);
  for (int i = 0; i < dof_; ++i) {
    if (this->jnt_end_(i) < this->jnt_start_(i)) {
      pos(i) = jnt_start_(i) - this->s_vp_[i]->Pos(time);

    } else {
      pos(i) = jnt_start_(i) + this->s_vp_[i]->Pos(time);
    }
  }

  return pos;
}

/**
 * @brief
 *
 * @param time
 * @return RobotPos
 */

/**
 * @brief 返回当前时间轨迹的速度
 *
 * @param time
 * @return RobotPos
 */
RobotPos TrajectoryRobot::Vel(double time) { return RobotPos(); }

/**
 * @brief 返回轨迹当前时间下的轴速度
 *
 * @param time
 * @return KDL::JntArray
 */
KDL::JntArray TrajectoryRobot::Vel_J(double time) {
  KDL::JntArray vel(dof_);
  for (int i = 0; i < dof_; ++i) {
    if (this->jnt_end_(i) < this->jnt_start_(i)) {
      vel(i) = this->jnt_start_(i) - this->s_vp_[i]->Vel(time);
    } else {
      vel(i) = this->jnt_start_(i) + this->s_vp_[i]->Vel(time);
    }
  }
  return vel;
}
/**
 * @brief 返回当前时间轨迹的加速度
 *
 * @param time
 * @return RobotPos
 */
RobotPos TrajectoryRobot::Acc(double time) { return RobotPos(); }

/**
 * @brief 返回当前时间轨迹的加速度
 *
 * @param time
 * @return KDL::JntArray
 */
KDL::JntArray TrajectoryRobot::Acc_J(double time) {
  KDL::JntArray acc(dof_);
  for (int i = 0; i < dof_; ++i) {
    if (this->jnt_end_(i) < this->jnt_start_(i)) {
      acc(i) = -this->s_vp_[i]->Acc(time);
    } else {
      acc(i) = this->s_vp_[i]->Acc(time);
    }
  }
  return acc;
}
/**
 * @brief 返回轨迹运行的总时间
 *
 * @return double
 */
double TrajectoryRobot::Duration() { return this->Tf_; }
}  // namespace rosc
