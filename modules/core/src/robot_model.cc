/**
 * @file robot_model.cc
 * @author maqun (maqun@buaa.edu.cn)
 * @brief
 * @version 0.1
 * @date 2021-08-17
 *
 * @copyright Copyright (c) 2021 ROSC
 *
 */

#include <bits/stdint-intn.h>
#include <bits/stdint-uintn.h>
#include <kdl/utilities/utility.h>
#include <math.h>
#include <robot_brain/core/robot_model.h>
#include <glog/logging.h>
#include <yaml-cpp/node/node.h>
#include <cerrno>
#include <cmath>
#include <iostream>
#include <memory>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/joint.hpp>
#include <kdl/chain.hpp>
#include <kdl/segment.hpp>
#include <robot_brain/core.hpp>
#include <robot_brain/command_types.hpp>
#include "robot_brain/config.h"
#include "robot_brain/core/default_values.h"

namespace rosc {
RobotModel::RobotModel() {}
/**
 * @brief RobotModel 析构函数
 *
 */
RobotModel::~RobotModel() {}

int RobotModel::GetDof() { return this->dof_; }

int32_t RobotModel::Trans2Encoder(double axis_out, int seq, double axis_zero) {
  return 0;
}

double RobotModel::Encoder2Position(int32_t cur_encoder, int32_t zero_encoder,
                                    int seq) {
  return 0;
}

bool RobotModel::CheckServoEMCState(uint16_t *statuscode) { return false; }

bool RobotModel::CheckServoErrorState(uint16_t *statuscode) { return false; }

bool RobotModel::CheckServoPowerState(uint16_t *statuscode) { return false; }

void RobotModel::SetSpeedLevel(int speed_level) {}

int RobotModel::GetSpeedLevel() { return -1; }

KDL::JntArray RobotModel::ForwordKinematic(const KDL::JntArray &joint) {
  return KDL::JntArray();
}
void RobotModel::InverseKinematic(const double *pos, double *q_out) {}

error_t RobotModel::ForwordKinematicsPos(const KDL::JntArray &joint,
                                         KDL::Frame *cart_pos,
                                         int segment) {
  return -1;
}

error_t RobotModel::InverseKinematicsPos(const KDL::JntArray &q_init,
                                         const KDL::Frame &target_T,
                                         KDL::JntArray *q_out) {
  return -1;
}
double RobotModel::GetArmLength() { return 0; }
double RobotModel::GetForearmLength() { return 0; }
double RobotModel::GetFingerLength() { return 0; }
void RobotModel::SetOperationMode(OperationMode mode) { this->mode_ = mode; }

OperationMode RobotModel::GetOperationMode() { return this->mode_; }

void RobotModel::UpdateConfig(const YAML::Node &config) {}

void RobotModel::SetMotionSpeedPersent(uint32_t persent) {
  this->speed_persent_ = persent;
}

uint32_t RobotModel::GetMotionSpeedPersent() { return this->speed_level_; }

/**
 * @brief Construct a new Robot Model 6-axis
 *
 */
RobotModelCobot::RobotModelCobot() {  // 构造第一个chain，计算从joint
                                      // 到cartesian
  double d1 = 143.5, a1 = 0, alpha1 = 0;
  double d2 = 0, a2 = 0, alpha2 = -KDL::PI_2;
  double d3 = 0, a3 = 243.5, alpha3 = 0;
  double d4 = 86, a4 = 211, alpha4 = 0;
  double d5 = 86, a5 = 0, alpha5 = KDL::PI_2;
  double d6 = 0, a6 = 0, alpha6 = -KDL::PI_2;
  double offset2 = -KDL::PI_2;
  double offset4 = KDL::PI_2;

  // 升降关节
  this->robot_chain_.addSegment(KDL::Segment(
      "J1", KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(a1, alpha1, d1, 0)));
  this->robot_chain_.addSegment(
      KDL::Segment("J2", KDL::Joint(KDL::Joint::RotZ),
                   KDL::Frame::DH(a2, alpha2, d2, offset2)));
  this->robot_chain_.addSegment(KDL::Segment(
      "J3", KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(a3, alpha3, d3, 0)));
  this->robot_chain_.addSegment(
      KDL::Segment("J4", KDL::Joint(KDL::Joint::RotZ),
                   KDL::Frame::DH(a4, alpha4, d4, offset4)));
  this->robot_chain_.addSegment(KDL::Segment(
      "J5", KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(a5, alpha5, d5, 0)));
  this->robot_chain_.addSegment(KDL::Segment(
      "J6", KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(a6, alpha6, d6, 0)));
  // 运动学
  this->fk_sovler_ = new KDL::ChainFkSolverPos_recursive(this->robot_chain_);
  this->ik_solver_lma_ =
      new KDL::ChainIkSolverPos_LMA(this->robot_chain_, 1E-8);
  this->fk_sovler_vel_ =
      new KDL::ChainFkSolverVel_recursive(this->robot_chain_);
  this->ik_sovler_vel_ = new KDL::ChainIkSolverVel_pinv(this->robot_chain_);
  this->ik_solver_nr_ = new KDL::ChainIkSolverPos_NR(
      this->robot_chain_, *this->fk_sovler_, *this->ik_sovler_vel_);
  this->dof_ = 6;
}

RobotModelCobot::RobotModelCobot(const YAML::Node &config) {
  // 构造第一个chain，计算从joint 到cartesian
  double d1 = 143.5, a1 = 0, alpha1 = KDL::PI_2;
  double d2 = 0, a2 = -243.5, alpha2 = 0;
  double d3 = 0, a3 = -211.0, alpha3 = 0;
  double d4 = 86.5, a4 = 0, alpha4 = KDL::PI_2;
  double d5 = 86, a5 = 0, alpha5 = -KDL::PI_2;
  double d6 = 72.5, a6 = 0, alpha6 = 0;

  // 升降关节
  this->robot_chain_.addSegment(KDL::Segment(
      "J1", KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(a1, alpha1, d1, 0)));
  this->robot_chain_.addSegment(KDL::Segment(
      "J2", KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(a2, alpha2, d2, 0)));
  this->robot_chain_.addSegment(KDL::Segment(
      "J3", KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(a3, alpha3, d3, 0)));
  this->robot_chain_.addSegment(KDL::Segment(
      "J4", KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(a4, alpha4, d4, 0)));
  this->robot_chain_.addSegment(KDL::Segment(
      "J5", KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(a5, alpha5, d5, 0)));
  this->robot_chain_.addSegment(KDL::Segment(
      "J6", KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(a6, alpha6, d6, 0)));
  // 运动学
  this->fk_sovler_ = new KDL::ChainFkSolverPos_recursive(this->robot_chain_);
  this->ik_solver_lma_ =
      new KDL::ChainIkSolverPos_LMA(this->robot_chain_, 1E-8);
  this->fk_sovler_vel_ =
      new KDL::ChainFkSolverVel_recursive(this->robot_chain_);
  this->ik_sovler_vel_ = new KDL::ChainIkSolverVel_pinv(this->robot_chain_);
  this->ik_solver_nr_ = new KDL::ChainIkSolverPos_NR(
      this->robot_chain_, *this->fk_sovler_, *this->ik_sovler_vel_);

  // first transfer speed, speed_level 1
  this->dof_ = config["zero_device_config"]["robot"]["dof"].as<int>();
  this->slave_num_ =
      config["zero_device_config"]["robot"]["slave_num"].as<int>();
  this->slave_list_ = new SlaveType[this->slave_num_]();
  for (int i = 0; i < this->slave_num_; ++i) {
    int tp =
        config["zero_device_config"]["robot"]["slave_type_list"][i].as<int>();
    switch (tp) {
    case 0:
      this->slave_list_[i] = MOTOR_SERVO;
      break;
    case 1:
      this->slave_list_[i] = IO_SERVO;
      break;
    case 2:
      this->slave_list_[i] = IO_SERVO_IN;
      break;
    case 3:
      this->slave_list_[i] = IO_SERVO_OUT;
      break;
    default:
      break;
    }
  }

  // 设置每个轴运行的最大速度、加速度、加加速度
  this->io_slave_seq_ =
      config["zero_device_config"]["robot"]["IO_slave_seq"].as<int>();

  for (int i = 0; i < this->dof_; ++i) {
    this->vel_max_[i] =
        config["zero_device_config"]["robot"]["joints"]["vel_max"][i]
            .as<double>();
    this->acc_max_[i] =
        config["zero_device_config"]["robot"]["joints"]["acc_max"][i]
            .as<double>();
    this->jerk_[i] =
        config["zero_device_config"]["robot"]["joints"]["jerk"][i].as<double>();
    this->dcc_max_[i] =
        config["zero_device_config"]["robot"]["joints"]["dcc_max"][i]
            .as<double>();
    this->dcc_jerk_[i] =
        config["zero_device_config"]["robot"]["joints"]["dcc_jerk"][i]
            .as<double>();
    this->motion_range_start_[i] =
        config["zero_device_config"]["robot"]["joints"]["MotionRangeStart"][i]
            .as<double>();
    this->motion_range_end_[i] =
        config["zero_device_config"]["robot"]["joints"]["MotionRangeEnd"][i]
            .as<double>();
    this->encoder_[i] =
        config["zero_device_config"]["robot"]["joints"]["encoder"][i].as<int>();
    this->ratio_[i] =
        config["zero_device_config"]["robot"]["joints"]["ratio"][i]
            .as<double>();
  }
  this->speed_level_ = 2;  // 默认速度等级是低速
  this->mode_ = OperationMode::Host;
}  // NOLINT

/********************************************************************
 * @brief 在线更新robot对象中的参数
 *
 * @param config
 ********************************************************************/
void RobotModelCobot::UpdateConfig(const YAML::Node &config) {
  // first transfer speed, speed_level 1

  this->elevation_r_ = 10.0;
  this->tool_length_ =
      config["zero_device_config"]["robot"]["tool_length"].as<double>();

  // 设置每个轴运行的最大速度、加速度、加加速度
  this->io_slave_seq_ =
      config["zero_device_config"]["robot"]["IO_slave_seq"].as<int>();

  for (int i = 0; i < this->dof_; ++i) {
    this->vel_max_[i] =
        config["zero_device_config"]["robot"]["joints"]["vel_max"][i]
            .as<double>();
    this->acc_max_[i] =
        config["zero_device_config"]["robot"]["joints"]["acc_max"][i]
            .as<double>();
    this->jerk_[i] =
        config["zero_device_config"]["robot"]["joints"]["jerk"][i].as<double>();
    this->dcc_max_[i] =
        config["zero_device_config"]["robot"]["joints"]["dcc_max"][i]
            .as<double>();
    this->dcc_jerk_[i] =
        config["zero_device_config"]["robot"]["joints"]["dcc_jerk"][i]
            .as<double>();
    this->motion_range_start_[i] =
        config["zero_device_config"]["robot"]["joints"]["MotionRangeStart"][i]
            .as<double>();
    this->motion_range_end_[i] =
        config["zero_device_config"]["robot"]["joints"]["MotionRangeEnd"][i]
            .as<double>();
    this->encoder_[i] =
        config["zero_device_config"]["robot"]["joints"]["encoder"][i].as<int>();
    this->ratio_[i] =
        config["zero_device_config"]["robot"]["joints"]["ratio"][i]
            .as<double>();
  }

  this->speed_persent_ = 1000;
}  // NOLINT
/**
 * @brief 析构函数
 *
 */
RobotModelCobot::~RobotModelCobot() {
  delete[] this->slave_list_;
  delete this->fk_sovler_;
  delete this->fk_sovler_vel_;
  delete this->ik_sovler_vel_;
  delete this->ik_solver_lma_;
  delete this->ik_solver_nr_;
  LOG(INFO) << "Robot Model Destructor.";
}

/**
 * @brief 获取机器人自由度
 *
 * @return int
 */
int RobotModelCobot::GetDof() { return this->dof_; }

/********************************************************************
 * @brief
 *
 * @param joint 电机转的角度
 * @return KDL::JntArray
 ********************************************************************/
KDL::JntArray RobotModelCobot::ForwordKinematic(const KDL::JntArray &joint) {
  KDL::JntArray robot_pos(this->dof_);

  return robot_pos;
}

/********************************************************************
 * @brief 将pos转成关节的角度值
 *
 * @param pos
 * @return KDL::JntArray
 ********************************************************************/
void RobotModelCobot::InverseKinematic(const double *pos, double *q_out) {

}

/********************************************************************
 * @brief
 *
 * @param pos
 * @return KDL::Vector
 ********************************************************************/
error_t RobotModelCobot::ForwordKinematicsPos(const KDL::JntArray &joint,
                                              KDL::Frame *cart_pos,
                                              int segment) {
  this->fk_sovler_->JntToCart(joint, *cart_pos, segment);
  return 0;
}

/********************************************************************
 * @brief
 *
 * @param vec
 * @return KDL::JntArray
 ********************************************************************/
error_t RobotModelCobot::InverseKinematicsPos(const KDL::JntArray &q_init,
                                              const KDL::Frame &target_T,
                                              KDL::JntArray *q_out
                                              ) {
  this->ik_solver_nr_->CartToJnt(q_init, target_T, *q_out);
  return 0;
}

/**
 * @brief 将插补动作的每一步转换为编码器的值
 *
 * @param delta_q
 * @param seq
 * @return int32_t
 */
int32_t RobotModelCobot::Trans2Encoder(double axis_out, int seq,
                                       double axis_zero) {
  double delta_q;
  delta_q = axis_out - axis_zero;
  return -delta_q * this->encoder_[seq] * this->ratio_[seq] / 360.0;
}

/**
 * @brief 由伺服编码器的值计算该轴的pos
 *
 * @param cur_encoder 当前编码器的值
 * @param zero_encoder 零位编码器的值
 * @param seq 伺服序号
 * @return double 该伺服的pos
 */
double RobotModelCobot::Encoder2Position(int32_t cur_encoder,
                                         int32_t zero_encoder, int seq) {
  double joint = 0 + (-cur_encoder + zero_encoder) * 360.0 /
                         this->encoder_[seq] / this->ratio_[seq];
  return joint;
}

/**
 * @brief 检测伺服是否处于错误状态
 *
 * @param statuscode
 * @return true 伺服没有处于错误状态
 * @return false 伺服处于错误状态
 */
bool RobotModelCobot::CheckServoErrorState(uint16_t *statuscode) {
  for (int i = 0; i < this->slave_num_; ++i) {
    if (this->slave_list_[i] != SlaveType::MOTOR_SERVO) {
      continue;
    }
    if (((statuscode[i] >> 3) & 0x001) == 0x001) {
      return false;
    }
  }
  return true;
}

/**
 * @brief 检测伺服是否处于急停状态
 *
 * @param statuscode
 * @return true 没有处于急停状态
 * @return false 处于急停状态
 */
bool RobotModelCobot::CheckServoEMCState(uint16_t *statuscode) {
  for (int i = 0; i < this->slave_num_; ++i) {
    if (this->slave_list_[i] != SlaveType::MOTOR_SERVO) {
      continue;
    }
    // LOG(INFO) << "伺服状态字: " << statuscode[i];
    if (((statuscode[i] >> 5) & 0x001) == 0x000) {
      return false;
    }
  }
  return true;
}

/**
 * @brief 检测伺服是否处于上电状态
 *
 * @param statuscode
 * @return true  处于上电状态
 * @return false 未处于上电状态
 */
bool RobotModelCobot::CheckServoPowerState(uint16_t *statuscode) {
  for (int i = 0; i < this->slave_num_; ++i) {
    if (this->slave_list_[i] != SlaveType::MOTOR_SERVO) {
      continue;
    }
    // LOG(INFO) << "伺服状态字: " << statuscode[i];
    if ((statuscode[i] & 0x006f) != 0x0027) {
      return false;
    }
  }
  return true;
}

/**
 * @brief
 *
 * @param speed_level
 */
void RobotModelCobot::SetSpeedLevel(int speed_level) {
  this->speed_level_ = speed_level;
}

int RobotModelCobot::GetSpeedLevel() { return this->speed_level_; }

/*********************** M124 机型 *************************************/

/********************************************************************
 * @brief Construct a new scara Robot Model
 *
 ********************************************************************/
RobotModelScara::RobotModelScara() : RobotModel() {
  this->arm_length_ = 440;
  this->forearm_length_ = 440;
  this->finger_length_ = 345;
  double a3 = finger_length_;
  double d1 = 0;

  // 构造第一个chain，计算从joint 到cartesian
  // 升降关节
  this->robot_chain_.addSegment(KDL::Segment("Elevation",
                                             KDL::Joint(KDL::Joint::TransZ),
                                             KDL::Frame::DH(0, 0, d1, 0)));
  // 旋转关节
  this->robot_chain_.addSegment(
      KDL::Segment("Rotation", KDL::Joint(KDL::Joint::RotZ),
                   KDL::Frame::DH(0, 0, 0, KDL::PI_2)));
  // 伸展关节
  this->robot_chain_.addSegment(KDL::Segment(
      "Extension", KDL::Joint(KDL::Joint::TransX), KDL::Frame::DH(0, 0, 0, 0)));
  // blade
  this->robot_chain_.addSegment(KDL::Segment(
      "Forearm", KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(a3, 0, 0, 0)));
  this->fk_sovler_ = new KDL::ChainFkSolverPos_recursive(this->robot_chain_);
  this->ik_solver_lma_ =
      new KDL::ChainIkSolverPos_LMA(this->robot_chain_, 1E-8);
  this->fk_sovler_vel_ =
      new KDL::ChainFkSolverVel_recursive(this->robot_chain_);
  this->ik_sovler_vel_ = new KDL::ChainIkSolverVel_pinv(this->robot_chain_);
  this->ik_solver_nr_ = new KDL::ChainIkSolverPos_NR(
      this->robot_chain_, *this->fk_sovler_, *this->ik_sovler_vel_);
}

RobotModelScara::RobotModelScara(const YAML::Node &config) {
  this->arm_length_ = 440;
  this->forearm_length_ = 440;
  this->finger_length_ = 345;
  double a3 = finger_length_;
  double d1 = 0;

  // 构造第一个chain，计算从joint 到cartesian
  // 升降关节
  this->robot_chain_.addSegment(KDL::Segment("Elevation",
                                             KDL::Joint(KDL::Joint::TransZ),
                                             KDL::Frame::DH(0, 0, d1, 0)));
  // 旋转关节
  this->robot_chain_.addSegment(KDL::Segment("Rotation",
                                             KDL::Joint(KDL::Joint::RotZ),
                                             KDL::Frame::DH(0, 0, 0, KDL::PI)));
  // 伸展关节
  this->robot_chain_.addSegment(KDL::Segment(
      "Arm", KDL::Joint(KDL::Joint::TransX), KDL::Frame::DH(0, 0, 0, 0)));
  // 小臂末端
  this->robot_chain_.addSegment(KDL::Segment(
      "Forearm", KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(a3, 0, 0, 0)));

  this->fk_sovler_ = new KDL::ChainFkSolverPos_recursive(this->robot_chain_);
  this->ik_solver_lma_ = new KDL::ChainIkSolverPos_LMA(this->robot_chain_);
  this->fk_sovler_vel_ =
      new KDL::ChainFkSolverVel_recursive(this->robot_chain_);
  this->ik_sovler_vel_ = new KDL::ChainIkSolverVel_pinv(this->robot_chain_);
  this->ik_solver_nr_ = new KDL::ChainIkSolverPos_NR(
      this->robot_chain_, *this->fk_sovler_, *this->ik_sovler_vel_);

  // 读取参数
  this->dof_ = config["scara_device_config"]["robot"]["dof"].as<int>();
  this->slave_num_ =
      config["scara_device_config"]["robot"]["slave_num"].as<int>();
  this->slave_list_ = new SlaveType[this->slave_num_]();
  for (int i = 0; i < this->slave_num_; ++i) {
    int tp =
        config["scara_device_config"]["robot"]["slave_type_list"][i].as<int>();
    switch (tp) {
    case 0:
      this->slave_list_[i] = MOTOR_SERVO;
      break;
    case 1:
      this->slave_list_[i] = IO_SERVO;
      break;
    case 2:
      this->slave_list_[i] = IO_SERVO_IN;
      break;
    case 3:
      this->slave_list_[i] = IO_SERVO_OUT;
      break;
    default:
      break;
    }
  }

  // 设置每个轴运行的最大速度、加速度、加加速度
  for (int i = 0; i < this->dof_; ++i) {
    this->vel_max_[i] =
        config["zero_device_config"]["robot"]["joints"]["vel_max"][i]
            .as<double>();
    this->acc_max_[i] =
        config["zero_device_config"]["robot"]["joints"]["acc_max"][i]
            .as<double>();
    this->jerk_[i] =
        config["zero_device_config"]["robot"]["joints"]["jerk"][i].as<double>();
    this->dcc_max_[i] =
        config["zero_device_config"]["robot"]["joints"]["dcc_max"][i]
            .as<double>();
    this->dcc_jerk_[i] =
        config["zero_device_config"]["robot"]["joints"]["dcc_jerk"][i]
            .as<double>();
    this->motion_range_start_[i] =
        config["zero_device_config"]["robot"]["joints"]["MotionRangeStart"][i]
            .as<double>();
    this->motion_range_end_[i] =
        config["zero_device_config"]["robot"]["joints"]["MotionRangeEnd"][i]
            .as<double>();
    this->encoder_[AxisNum::Elevation] =
        config["zero_device_config"]["robot"]["joints"]["encoder"][i].as<int>();
    this->ratio_[i] =
        config["zero_device_config"]["robot"]["joints"]["ratio"][i]
            .as<double>();
  }
  this->speed_level_ = 2;
  this->mode_ = OperationMode::Host;
}  // NOLINT

/********************************************************************
 * @brief 在线更新robot对象的参数
 *
 * @param config
 ********************************************************************/
void RobotModelScara::UpdateConfig(const YAML::Node &config) {
  // 设置每个轴运行的最大速度、加速度、加加速度
  for (int i = 0; i < this->dof_; ++i) {
    this->vel_max_[i] =
        config["zero_device_config"]["robot"]["joints"]["vel_max"][i]
            .as<double>();
    this->acc_max_[i] =
        config["zero_device_config"]["robot"]["joints"]["acc_max"][i]
            .as<double>();
    this->jerk_[i] =
        config["zero_device_config"]["robot"]["joints"]["jerk"][i].as<double>();
    this->dcc_max_[i] =
        config["zero_device_config"]["robot"]["joints"]["dcc_max"][i]
            .as<double>();
    this->dcc_jerk_[i] =
        config["zero_device_config"]["robot"]["joints"]["dcc_jerk"][i]
            .as<double>();
    this->motion_range_start_[i] =
        config["zero_device_config"]["robot"]["joints"]["MotionRangeStart"][i]
            .as<double>();
    this->motion_range_end_[i] =
        config["zero_device_config"]["robot"]["joints"]["MotionRangeEnd"][i]
            .as<double>();
    this->encoder_[AxisNum::Elevation] =
        config["zero_device_config"]["robot"]["joints"]["encoder"][i].as<int>();
    this->ratio_[i] =
        config["zero_device_config"]["robot"]["joints"]["ratio"][i]
            .as<double>();
  }
}  // NOLINT
RobotModelScara::~RobotModelScara() {
  delete this->fk_sovler_;
  delete this->fk_sovler_vel_;
  delete this->ik_sovler_vel_;
  delete this->ik_solver_lma_;
  delete this->ik_solver_nr_;
  delete[] this->slave_list_;
}

int RobotModelScara::GetDof() { return this->dof_; }

/********************************************************************
 * @brief
 *
 * @param axis_out
 * @param seq
 * @param axis_zero
 * @return int32_t
 ********************************************************************/
int32_t RobotModelScara::Trans2Encoder(double axis_out, int seq,
                                       double axis_zero) {
  // int32_t encoder;
  // encoder =
  //     (axis_out - axis_zero) / 360.0 * this->encoder_[seq] *
  //     this->ratio_[seq];
  // return encoder;
  double delta_q;
  if (seq == 0) {  // 0
    delta_q = axis_out - axis_zero;
    return delta_q / this->ratio_[seq] * this->encoder_[seq];
  } else if (seq == 3 || seq == 1) {
    delta_q = -axis_out + axis_zero;
    return delta_q * this->encoder_[seq] * this->ratio_[seq] / 360.0;
  } else {  // 旋转 1
    delta_q = axis_out - axis_zero;
    return delta_q * this->encoder_[seq] * this->ratio_[seq] / 360.0;
  }
}

/********************************************************************
 * @brief
 *
 * @param cur_encoder
 * @param zero_encoder
 * @param seq
 * @return double
 ********************************************************************/
double RobotModelScara::Encoder2Position(int32_t cur_encoder,
                                         int32_t zero_encoder, int seq) {
  // double axis;
  // axis = (cur_encoder - zero_encoder)
  // * 360.0 / this->ratio_[seq] / this->encoder_[seq];
  // return axis;

  if (seq == 0) {
    double joint = 0 + (cur_encoder - zero_encoder) / this->encoder_[seq];
    return joint * this->ratio_[seq];
  } else if (seq == 3 || seq == 1) {
    double joint = (-cur_encoder + zero_encoder) * 360.0 / this->encoder_[seq] /
                   this->ratio_[seq];
    return joint;
  } else {
    double joint = (cur_encoder - zero_encoder) * 360.0 / this->encoder_[seq] /
                   this->ratio_[seq];
    return joint;
  }
}

bool RobotModelScara::CheckServoErrorState(uint16_t *statuscode) {
  for (int i = 0; i < this->slave_num_; ++i) {
    if (this->slave_list_[i] != SlaveType::MOTOR_SERVO) {
      continue;
    }
    if (((statuscode[i] >> 3) & 0x001) == 0x001) {
      return false;
    }
  }
  return true;
}

bool RobotModelScara::CheckServoEMCState(uint16_t *statuscode) {
  for (int i = 0; i < this->slave_num_; ++i) {
    if (this->slave_list_[i] != SlaveType::MOTOR_SERVO) {
      continue;
    }
    // LOG(INFO) << "伺服状态字: " << statuscode[i];
    if (((statuscode[i] >> 5) & 0x001) == 0x000) {
      return false;
    }
  }
  return true;
}

bool RobotModelScara::CheckServoPowerState(uint16_t *statuscode) {
  for (int i = 0; i < this->slave_num_; ++i) {
    if (this->slave_list_[i] != SlaveType::MOTOR_SERVO) {
      continue;
    }
    // LOG(INFO) << "伺服状态字: " << statuscode[i];
    if ((statuscode[i] & 0x006f) != 0x0027) {
      return false;
    }
  }
  return true;
}

void RobotModelScara::SetSpeedLevel(int speed_level) {
  this->speed_level_ = speed_level;
}

int RobotModelScara::GetSpeedLevel() { return this->speed_level_; }

/********************************************************************
 * @brief 从关节旋转的角度到线性轴
 *
 * @param joint
 * @return KDL::JntArray
 ********************************************************************/
KDL::JntArray RobotModelScara::ForwordKinematic(const KDL::JntArray &joint) {
  KDL::JntArray jnt(this->dof_);
  jnt(0) = joint(0);
  if (joint(2) < 0) {  // extension 为负，elbow在右侧
    jnt(1) = joint(1) + joint(2) / 2;                             // rotation
    jnt(2) = 2 * arm_length_ * sin(joint(2) / 2 * KDL::deg2rad);  // extension
    jnt(3) = joint(2) / 2 + 90 + joint(3);                        // wrist 1
    jnt(4) = joint(2) / 2 + 90 + joint(4);                        // wrist 2
  } else {  // extension 为正，elbow在左侧
    jnt(1) = joint(1) + joint(2) / 2;
    jnt(2) = 2 * arm_length_ * sin(joint(2) / 2 * KDL::deg2rad);
    jnt(3) = joint(2) / 2 + 90 + joint(3);
    jnt(4) = joint(2) / 2 + 90 + joint(4);
  }

  return jnt;
}

/********************************************************************
 * @brief 从线性轴转到旋转轴
 *
 * @param pos
 * @param q_out
 ********************************************************************/
void RobotModelScara::InverseKinematic(const double *pos, double *q_out) {
  q_out[0] = pos[0];
  if (pos[2] < 0) {  // extension 为负
    q_out[2] = asin(pos[2] / 2 / arm_length_) * 2 * KDL::rad2deg;
    q_out[1] = pos[1] - q_out[2] / 2;
    q_out[3] = pos[3] - q_out[2] / 2 - 90;
    q_out[4] = pos[4] - q_out[2] / 2 - 90;
  } else {  // extension 为正
    q_out[2] = asin(pos[2] / 2 / arm_length_) * 2 * KDL::rad2deg;
    q_out[1] = pos[1] - q_out[2] / 2;
    q_out[3] = pos[3] - q_out[2] / 2 - 90;
    q_out[4] = pos[4] - q_out[2] / 2 - 90;
  }
}

/********************************************************************
 * @brief
 *
 * @param joint
 * @param cart_pos
 * @param segmentNr
 * @return error_t
 ********************************************************************/
error_t RobotModelScara::ForwordKinematicsPos(const KDL::JntArray &joint,
                                              KDL::Frame *cart_pos,
                                              int segment) {
  // 采用kdl库计算运动学
  KDL::JntArray q_in(this->dof_ - 1);
  for (int i = 0; i < this->dof_ - 1; ++i) {
    q_in(i) = joint(i);
  }
  q_in(1) = q_in(1) * KDL::deg2rad;
  q_in(3) = q_in(3) * KDL::deg2rad;
  int state = this->fk_sovler_->JntToCart(q_in, *cart_pos, segment);
  return state;
}

/********************************************************************
 * @brief
 *
 * @param q_init
 * @param target_T
 * @param q_out
 * @return int
 ********************************************************************/
int RobotModelScara::InverseKinematicsPosLMA(const KDL::JntArray &q_init,
                                             const KDL::Frame &target_T,
                                             KDL::JntArray *q_out) {
  KDL::JntArray joint_out(q_init.rows());
  int state = this->ik_solver_lma_->CartToJnt(q_init, target_T, joint_out);
  if (state < 0) {
    if (state == KDL::ChainIkSolverPos_LMA::E_GRADIENT_JOINTS_TOO_SMALL) {
      std::cout << "E_GRADIENT_JOINTS_TOO_SMALL the gradient of towards the "
                   "joints is to small "
                << std::endl;
    } else if (state ==
               KDL::ChainIkSolverPos_LMA::E_INCREMENT_JOINTS_TOO_SMALL) {
      std::cout << "if joint position increments are to small," << std::endl;
    } else if (state == KDL::ChainIkSolverPos_LMA::E_MAX_ITERATIONS_EXCEEDED) {
      std::cout << "* if number of iterations is exceeded.return state;"
                << std::endl;
    }
  } else {
    for (unsigned int i = 0; i < q_init.rows(); ++i) {
      q_out->data[i] = joint_out.data[i];
    }
  }
  return state;
}

int RobotModelScara::InverseKinematicsPosNR(const KDL::JntArray &q_init,
                                            const KDL::Frame &target_T,
                                            KDL::JntArray *q_out) {
  KDL::JntArray joint_out(q_init.rows());
  int state = this->ik_solver_nr_->CartToJnt(q_init, target_T, joint_out);
  if (state < 0) {
    return state;
  } else {
    for (unsigned int i = 0; i < q_init.rows(); ++i) {
      q_out->data[i] = joint_out(i);
    }
  }
  return state;
}

/********************************************************************
 * @brief M124机型解析解
 *
 * @param q_init 上一个值
 * @param target_T 目标笛卡尔位置
 * @param q_out 输出值
 * @return error_t
 ********************************************************************/
error_t RobotModelScara::InverseKinematicsPos_analytical(
    const KDL::JntArray &q_init, const KDL::Frame &target_T, double *q_out) {
  double blade = this->finger_length_;
  double r, p, yaw;
  double wrist_x, wrist_y;
  double end_x, end_y;
  double extension;
  double rotation, wrist;
  target_T.M.GetRPY(r, p, yaw);
  end_x = target_T.p.x();
  end_y = target_T.p.y();
  // 计算腕部数值
  wrist_x = end_x - blade * cos(yaw);
  wrist_y = end_y - blade * sin(yaw);
  extension = sqrt(wrist_x * wrist_x + wrist_y * wrist_y);
  // 分情况计算，extension有正负
  double dot, cross;
  // extension > 0
  dot = (wrist_x) * (-1) + (wrist_y) * (0);
  cross = (-1) * (wrist_y) - (wrist_x) * (0);
  double r1 = rotation = std::atan2(cross, dot);
  dot = (wrist_x) * (end_x - wrist_x) + (wrist_y) * (end_y - wrist_y);
  cross = (wrist_x) * (end_y - wrist_y) - (end_x - wrist_x) * (wrist_y);
  double w1 = wrist = std::atan2(cross, dot);
  double e1 = extension;

  double err_1 = fabs(extension - q_init(2)) + fabs(rotation - q_init(1)) +
                 fabs(wrist - q_init(3));
  // extension < 0
  double e2 = extension = -extension;
  dot = (-wrist_x) * (-1) + 0;
  cross = (-1) * (-wrist_y) - 0;
  double r2 = rotation = std::atan2(cross, dot);
  dot = (-wrist_x) * (end_x - wrist_x) + (-wrist_y) * (end_y - wrist_y);
  cross = (-wrist_x) * (end_y - wrist_y) - (end_x - wrist_x) * (-wrist_y);
  double w2 = wrist = std::atan2(cross, dot);
  double err_2 = fabs(extension - q_init(2)) + fabs(rotation - q_init(1)) +
                 fabs(wrist - q_init(3));

  if (err_1 < err_2) {
    q_out[1] = r1;
    q_out[2] = e1;
    q_out[3] = w1;
  } else {
    q_out[1] = r2;
    q_out[2] = e2;
    q_out[3] = w2;
  }
  q_out[0] = target_T.p.z();
  // 判断求解结果与初始值相差是否过大,差距为2pi
  if (fabs(q_init(1) - q_out[1]) > KDL::PI) {
    q_out[1] += q_init(1) < q_out[1] ? -2 * KDL::PI : 2 * KDL::PI;
  }
  if (fabs(q_init(3) - q_out[3]) > KDL::PI) {
    q_out[3] += q_init(3) < q_out[3] ? -2 * KDL::PI : 2 * KDL::PI;
  }
  return 0;
}
/********************************************************************
 * @brief
 *
 * @param q_init
 * @param target_T
 * @param q_out
 * @param type
 * @return error_t
 ********************************************************************/
error_t RobotModelScara::InverseKinematicsPos(const KDL::JntArray &q_init,
                                              const KDL::Frame &target_T,
                                              KDL::JntArray *q_out
                                              ) {
  InverseKinematicsType type = InverseKinematicsType::ANALYTICAL;
  // 如果目标坐标超出了最大伸展长度，返回前一个关节数值
  if (sqrt(pow(target_T.p.x(), 2) + pow(target_T.p.y(), 2)) >
      (this->arm_length_ + this->forearm_length_ + this->finger_length_)) {
    for (int i = 0; i < this->dof_; ++i) {
      q_out->data[i] = q_init(i);
    }
    return -1;
  }
  // 机器人有两个末端，两个末端构型相同，按照一个末端的运动学来计算
  KDL::JntArray last_q(this->dof_ - 1);

  double out_q[kDofMax];
  for (int i = 0; i < this->dof_ - 1; ++i) {
    last_q(i) = q_init(i);
  }
  last_q(1) = last_q(1) * KDL::deg2rad;
  int ret;
  switch (type) {
  case InverseKinematicsType::LMA:
    // ret = this->InverseKinematicsPosLMA(last_q, target_T, &out_q);
    break;
  case InverseKinematicsType::NR:
    // ret = this->InverseKinematicsPosNR(last_q, target_T, &out_q);
    break;
  case InverseKinematicsType::ANALYTICAL:
    ret = this->InverseKinematicsPos_analytical(last_q, target_T, out_q);
    break;
  }
  for (int i = 0; i < this->dof_ - 1; ++i) {
    q_out->data[i] = out_q[i];
  }
  q_out->data[1] = out_q[1] * KDL::rad2deg;
  return ret;
}
double RobotModelScara::GetArmLength() { return this->arm_length_; }
double RobotModelScara::GetForearmLength() { return this->forearm_length_; }
double RobotModelScara::GetFingerLength() { return this->finger_length_; }

}  // namespace rosc
