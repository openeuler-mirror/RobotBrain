/**
 * @file robot_model.h
 * @author maqun (maqun@buaa.edu.cn)
 * @brief 机器人模型
 * @version 0.1
 * @date 2021-08-17
 *
 * @copyright Copyright (c) 2021 ROSC
 *
 */
#ifndef MODULES_CORE_INCLUDE_ROBOT_BRAIN_CORE_ROBOT_MODEL_H_
#define MODULES_CORE_INCLUDE_ROBOT_BRAIN_CORE_ROBOT_MODEL_H_

#include <bits/stdint-uintn.h>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarray.hpp>
#include <robot_brain/core/config.h>
#include <cerrno>
#include <cstddef>
#include <vector>
#include <memory>
#include <string>
#include <kdl/segment.hpp>
#include <kdl/joint.hpp>
#include <kdl/frames.hpp>
#include <kdl/chain.hpp>
#include <kdl/rigidbodyinertia.hpp>
#include <robot_brain/ethercat_frame_types.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolver.hpp>
#include "robot_brain/config.h"
#include "robot_brain/command_types.hpp"

namespace rosc {
typedef struct seg {
  KDL::Joint joint;               // 关节
  KDL::Frame dh;                  // dh参数
  KDL::RigidBodyInertia inertia;  // 刚体惯量
} seg;                            // 机器人段

typedef enum InverseKinematicsType {
  LMA,  // 列文伯格-马夸尔特法,一种使用最广泛的最小二乘算法
  NR,         // 牛顿拉普森法
  ANALYTICAL  // 解析法
} InverseKinematicsType;

typedef enum OperationMode { Host, Teaching } OperationMode;
typedef struct JointParam {
  double resolution;  ///< 每圈的分辨率
} ServoParam;

/**
 * @brief 机器人模型类
 *
 */
class RobotModel {
 public:
  std::string name_;  /// 机器人名称
  ArmType arm_type_;

 protected:
  int dof_;  ///< 机器人自由度
  OperationMode mode_;
  double tool_length_;

 public:
  int slave_num_;
  SlaveType *slave_list_;

 public:
  int speed_level_;               // 当前速度等级
  uint32_t speed_persent_;        // 速度百分比
  double speed_type_;             // 速度类型

  double jog_speed_tp_[3][kDofMax];
  double inching_speed_[kDofMax];

  double vel_max_[kDofMax];  // 每个轴的最大速度
  double acc_max_[kDofMax];  // 每个轴的最大加速度
  double jerk_[kDofMax];     // 每个轴的加加速度
  double dcc_max_[kDofMax];
  double dcc_jerk_[kDofMax];
  double motion_range_start_[kDofMax];  // 每个轴的冲程
  double motion_range_end_[kDofMax];
  double encoder_[kDofMax];  // 编码器
  double ratio_[kDofMax];    // 减速比
  double jog_step_max_[kDofMax];
  // io 序号
  int io_slave_seq_;  // IO从站位置

 public:
  RobotModel();
  ~RobotModel();

 public:
  virtual int GetDof();
  virtual int32_t Trans2Encoder(double axis_out, int seq, double axis_zero);
  virtual double Encoder2Position(int32_t cur_encoder, int32_t zero_encoder,
                                  int seq);
  virtual bool CheckServoErrorState(uint16_t *statuscode);
  virtual bool CheckServoEMCState(uint16_t *statuscode);
  virtual bool CheckServoPowerState(uint16_t *statuscode);
  virtual void SetSpeedLevel(int speed_level);
  virtual int GetSpeedLevel();

  // 运动学
  virtual KDL::JntArray ForwordKinematic(const KDL::JntArray &joint);
  virtual void InverseKinematic(const double *pos, double *q_out);

  virtual error_t ForwordKinematicsPos(const KDL::JntArray &joint,
                                       KDL::Frame *cart_pos, int segment = -1);
  virtual error_t InverseKinematicsPos(const KDL::JntArray &q_init,
                                       const KDL::Frame &target_T,
                                       KDL::JntArray *q_out);
  virtual double GetArmLength();
  virtual double GetForearmLength();
  virtual double GetFingerLength();
  virtual void SetOperationMode(OperationMode mode);
  virtual OperationMode GetOperationMode();
  virtual void UpdateConfig(const YAML::Node &config);
  uint32_t GetMotionSpeedPersent();
  void SetMotionSpeedPersent(uint32_t persent);
};

/**
 * @brief 6轴协作
 *
 */
class RobotModelCobot : public RobotModel {
 public:
  explicit RobotModelCobot(const YAML::Node &config);
  RobotModelCobot();
  ~RobotModelCobot();

 private:
  std::string config_path_;

 public:
  double elevation_r_;         // 升降轴旋转半径
  // double tool_length_;  // 当伸展轴为0时，末端在x方向的坐标

  double minimum_radius_rotation_;  // 最小回旋半径
  double calibration_pos_;  //机械臂标定位置
  double arm_length_;       // 大臂长度

 private:
  KDL::Chain robot_chain_;
  KDL::ChainFkSolverPos_recursive *fk_sovler_;      // 正运动学位置求解
  KDL::ChainIkSolverPos_LMA *ik_solver_lma_;        // 逆运动学位置求解
  KDL::ChainIkSolverPos_NR *ik_solver_nr_;          // 逆运动学位置求解
  KDL::ChainFkSolverVel_recursive *fk_sovler_vel_;  // 正运动学速度求解
  KDL::ChainIkSolverVel_pinv *ik_sovler_vel_;       // 逆运动学速度求解

 public:
  virtual int GetDof();

  virtual int32_t Trans2Encoder(double axis_out, int seq, double axis_zero);
  virtual double Encoder2Position(int32_t cur_encoder, int32_t zero_encoder,
                                  int seq);
  virtual bool CheckServoErrorState(uint16_t *statuscode);
  virtual bool CheckServoEMCState(uint16_t *statuscode);
  virtual bool CheckServoPowerState(uint16_t *statuscode);
  virtual void SetSpeedLevel(int speed_level);
  virtual int GetSpeedLevel();

  // 运动学
  virtual KDL::JntArray ForwordKinematic(const KDL::JntArray &joint);
  virtual void InverseKinematic(const double *pos, double *q_out);

  virtual error_t ForwordKinematicsPos(const KDL::JntArray &joint,
                                       KDL::Frame *cart_pos, int segment = -1);
  virtual error_t InverseKinematicsPos(const KDL::JntArray &q_init,
                                       const KDL::Frame &target_T,
                                       KDL::JntArray *q_out);

  virtual void UpdateConfig(const YAML::Node &config);
};

class RobotModelScara : public RobotModel {
 public:
  RobotModelScara();
  explicit RobotModelScara(const YAML::Node &config);
  ~RobotModelScara();

 private:
  double arm_length_;      // 大臂长度
  double forearm_length_;  // 小臂长度
  double finger_length_;   // 手指长度

  double elevation_r_;

 private:
  KDL::Chain robot_chain_;
  KDL::ChainFkSolverPos_recursive *fk_sovler_;      // 正运动学位置求解
  KDL::ChainIkSolverPos_LMA *ik_solver_lma_;        // 逆运动学位置求解
  KDL::ChainIkSolverPos_NR *ik_solver_nr_;          // 逆运动学位置求解
  KDL::ChainFkSolverVel_recursive *fk_sovler_vel_;  // 正运动学速度求解
  KDL::ChainIkSolverVel_pinv *ik_sovler_vel_;       // 逆运动学速度求解

 public:
  virtual int GetDof();

  virtual int32_t Trans2Encoder(double axis_out, int seq, double axis_zero);
  virtual double Encoder2Position(int32_t cur_encoder, int32_t zero_encoder,
                                  int seq);
  virtual bool CheckServoErrorState(uint16_t *statuscode);
  virtual bool CheckServoEMCState(uint16_t *statuscode);
  virtual bool CheckServoPowerState(uint16_t *statuscode);
  virtual void SetSpeedLevel(int speed_level);
  virtual int GetSpeedLevel();

  virtual KDL::JntArray ForwordKinematic(const KDL::JntArray &joint);
  virtual void InverseKinematic(const double *pos, double *q_out);

  virtual error_t ForwordKinematicsPos(const KDL::JntArray &joint,
                                       KDL::Frame *cart_pos, int segment = -1);
  virtual error_t InverseKinematicsPos(const KDL::JntArray &q_init,
                                       const KDL::Frame &target_T,
                                       KDL::JntArray *q_out);
  error_t InverseKinematicsPos_analytical(const KDL::JntArray &q_init,
                                          const KDL::Frame &target_T,
                                          double *q_out);  // NOLINT

  virtual double GetArmLength();
  virtual double GetForearmLength();
  virtual double GetFingerLength();

  virtual void UpdateConfig(const YAML::Node &config);

 private:
  int InverseKinematicsPosLMA(const KDL::JntArray &q_init,
                              const KDL::Frame &target_T, KDL::JntArray *q_out);
  int InverseKinematicsPosNR(const KDL::JntArray &q_init,
                             const KDL::Frame &target_T, KDL::JntArray *q_out);
};

}  // namespace rosc

#endif  // MODULES_CORE_INCLUDE_ROBOT_BRAIN_CORE_ROBOT_MODEL_H_
