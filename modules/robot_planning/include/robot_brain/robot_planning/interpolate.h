/**
 * @file interpolate.h
 * @author maqun@buaa.edu.cn (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2021-10-07
 *
 * @copyright Copyright (c) 2021 ROSC
 *
 */
#ifndef MODULES_ROBOT_PLANNING_INCLUDE_ROBOT_BRAIN_ROBOT_PLANNING_INTERPOLATE_H_
#define MODULES_ROBOT_PLANNING_INCLUDE_ROBOT_BRAIN_ROBOT_PLANNING_INTERPOLATE_H_

#include <robot_brain/config.h>
#include <cerrno>
#include "./bezier.h"
#include "robot_brain/command_types.hpp"
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
namespace rosc {

typedef enum TrajType { S_VELOCITY_PROFILE, T_VELOCITY_PROFILE } TrajType;

typedef enum S_Vel_Type {
  Convexity_S,  // 凸性S曲线
  Concavity_S,  // 凹形S曲线
  Double_Acc,   // 双加速S曲线
  Double_Dcc    // 双减速S曲线
} S_Vel_Type;

typedef struct S_Vel_Para {
  double v[4];
  double t[4];
  double s[4];
  double S;
  double acc[4];
  double jerk[4];
} S_Vel_Para;

class S_VelocityProfile {
 public:
  S_VelocityProfile();
  S_VelocityProfile(double v_max, double acc_max, double jerk, double Sf,
                    double v_start = 0.0, double v_end = 0.0,
                    double dcc_max = 0.0, double jerk_dcc = 0.0);
  ~S_VelocityProfile();

 private:
  double v_max_;          // 匀速阶段最大速度
  double v_start_;        // 起始速度
  double v_end_;          // 终点速度
  double acc_max_;        // 最大加速度
  double dcc_max_;        // 最大减速度
  double jerk_;           // 加加速度
  double jerk_dcc_;       // 减减速度
  double T[8];            // 7个阶段的运行时间
  double S[8];            // 7个阶段的运行距离
  double V[8];            // 7个阶段的起始速度
  double A[8];            // 7个阶段起始加速度
  double J[8];            // 7个阶段的起始加加速度
  double Tf_;             // 运行总时长
  double Sf_;             // 运行总长度
  S_Vel_Type traj_type_;  // 轨迹曲线类型

 private:
  error_t VelocityProfile();
  error_t VelocityProfile(bool s);

 public:
  error_t SetVelocityProfile(double v_max, double acc_max, double jerk,
                             double Sf, double v_start = 0.0,
                             double v_end = 0.0, double dcc_max = 0.0,
                             double jerk_dcc = 0.0);
  double Pos(double time);
  double Vel(double time);
  double Acc(double time);
  double Jerk(double time);
  double Duration();
  void PrintPara();
  void ReplanStop(double current_time);
};

class T_VelocityProfile {
 public:
  T_VelocityProfile();
  T_VelocityProfile(double v_max, double acc, double Sf, double v_start = 0.0,
                    double v_end = 0.0, double dcc = 0.0);
  ~T_VelocityProfile();

 private:
  double v_max_;    // 轨迹最大速度
  double v_start_;  // 轨迹起始速度
  double v_end_;    // 轨迹终点速度
  double acc_;      // 轨迹加速度
  double dcc_;      // 估计减速度
  double Tf_;       // 轨迹运行总时间
  double Sf_;       // 轨迹总行程
  double T[4];      // 三个阶段开始的时间
  double S[4];      // 三个阶段的位移
  double V[4];      // 三个阶段的起始速度
  int acc_direc_;   // 第一个阶段方向
  int dcc_direc_;   // 第二个阶段方向

 private:
  error_t VelocityProfile();

 public:
  error_t SetVelocityProfile(double v_max, double acc, double Sf,
                             double v_start = 0.0, double v_end = 0.0,
                             double dcc = 0.0);
  double Pos(double time);
  double Vel(double time);
  double Acc(double time);
  double Jerk(double time);
  double Duration();
};

/**
 * @brief 机器人轨迹规划类
 *
 */
class TrajectoryRobot {
 public:
  TrajectoryRobot();
  explicit TrajectoryRobot(int dof);
  ~TrajectoryRobot();

 private:
  int dof_;

 protected:
  S_VelocityProfile *s_vp_[kDofMax];  // s型速度规划器，四个关节
  T_VelocityProfile *t_vp_[kDofMax];  // t型速度规划器
  Bezier *bzer_;                      // 贝塞尔曲线规划器
  double v_max_;                      // 速度最大值
  double acc_max_;                    // 加速度最大值
  double jerk_;                       // 加加速度
  double S[kDofMax];                  // 轨迹中四个关节的行程
  double T[kDofMax];                  // 轨迹中四个关节的运行时间
  double Tf_;                         // 轨迹运行需要的最大时间
  double Sf_;                         // 轨迹最大行程（空间行程）
  double start_[kDofMax];             // 起始位置
  double end_[kDofMax];               // 终点位置
  double mid_[kDofMax];               // 中间点位置
  RobotPos start_coord_;              // 起始位置坐标
  RobotPos end_coord_;                // 终点位置坐标
  RobotPos mid_coord_;                // 中间点位置坐标
  KDL::Vector start_vec_;             // *轨迹起始点位置向量
  KDL::Vector mid_vec_;               // *轨迹中间点位置向量
  KDL::Vector end_vec_;               // *轨迹终点位置向量
  KDL::Frame start_cart_;
  KDL::Frame mid_cart_;
  KDL::Frame end_cart_;
  KDL::Vector bzer_end_vec_;
  KDL::JntArray jnt_start_;  // 初始关节角度
  KDL::JntArray jnt_end_;    // 末端关节角度
  char fix_blade_;

  double d1_;       // 第一段直线轨迹长度
  double d2_;       // 第二段直线轨迹长度
  double c_;        // 中间圆弧长度
  double a_;        // 转弯区对应的长度
  double radius_;   // 圆弧半径
  char which_arm_;  // 运动臂（L: 左臂，R：右臂，A：同时运动）
  KDL::Vector arc_start_;   // 圆弧的起点坐标
  KDL::Vector arc_end_;     // 圆弧的终点坐标
  KDL::Vector arc_center_;  // 圆弧的圆心
  char arc_direction_;      // 圆弧运动方向（'+' | '-'）
  double theta_;            // 两条线段的夹角
  double t_[4];  // 三点运动中，直线、弧线、直线运动时间
  bool is_motion_line_;  // 运动模式是二点直线还是三点插补
  bool is_auto_intermediate_point_;  // 圆弧插补是否自动计算中间点位置

 public:
  KDL::JntArray
      last_jnt_for_inverse_;  // 记录数值法求笛卡尔逆解时上一个关节位置

 public:
  error_t SetTrajectory(double start[], double end[], double v_max[],
                        double acc_max[], double jerk[], double dcc_max[],
                        double dcc_jerk[]);
  error_t SetTrajectoryAngle(const KDL::JntArray &start,
                             const KDL::JntArray &end, double v_max[],
                             double acc_max[], double jerk[]);

  virtual error_t SetTrajectoryCartesianAxis(KDL::Frame start, KDL::Frame end,
                                             double v_max, double acc_max,
                                             double jerk);
  virtual RobotPos Pos(double time);
  virtual RobotPos Vel(double time);
  virtual RobotPos Acc(double time);

  void Pos_Axis(double time, double *next_pos);

  void Pos_cartesian(double time, KDL::Frame *pos);

  KDL::JntArray Pos_J(double time);
  KDL::JntArray Vel_J(double time);
  KDL::JntArray Acc_J(double time);

  virtual double Duration();

 public:
  void ReplanTrajecotryAixsStop(double cur_traj_time);
};

}  // namespace rosc
#endif  // MODULES_ROBOT_PLANNING_INCLUDE_ROBOT_BRAIN_ROBOT_PLANNING_INTERPOLATE_H_
