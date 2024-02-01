/**
 * @file trajectory_component.cc
 * @author maqun (maqun@buaa.edu.cn)
 * @brief
 * @version 0.1
 * @date 2022-01-18
 *
 * @copyright Copyright (c) 2022 ROSC
 *
 */
#include <bits/stdint-intn.h>
#include <bits/stdint-uintn.h>
#include <unistd.h>
#include <kdl/utilities/utility.h>
#include <glog/logging.h>
#include <link.h>
#include <math.h>
#include <pthread.h>
#include <sstream>
#include <chrono>  // NOLINT
#include <algorithm>
#include <thread>  // NOLINT
#include <cmath>
#include <ctime>
#include <iostream>
#include <vector>
#include <string>
#include <cstdio>
#include <cstring>
#include <ostream>
#include <memory>
#include <istream>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <rtt/ConnPolicy.hpp>
#include <kdl/joint.hpp>
#include <rtt/os/Time.hpp>
#include <rtt/os/threads.hpp>
#include <rtt/FlowStatus.hpp>
#include <robot_brain/core.hpp>
#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Property.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/OperationCaller.hpp>
#include <rtt/Port.hpp>
#include <rtt/Activity.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>
#include <robot_brain/robot_teach.hpp>
#include "../include/trajectory_component.h"
#include "robot_brain/config.h"
#include <robot_brain/ethercat_frame_types.hpp>
#include "robot_brain/robot_exception/robot_error.h"
#include "robot_brain/robot_exception/robot_status.h"
#include "robot_brain/teach_point.h"
#include "robot_brain/command_types.hpp"
#include <robot_brain/robot_planning.hpp>

#define BLOCK_TIME 3

namespace rosc {
/**
 * @brief Destroy the Trajectory Component:: Trajectory Component object
 *
 */
TrajectoryComponent::~TrajectoryComponent() {}
/**
 * @brief Construct a new Trajectory Component:: Trajectory Component object
 *
 */
TrajectoryComponent::TrajectoryComponent()
    : TrajectoryComponent(std::string("trajectory")) {
  this->rob_model_ = Application::GetContext()->GetRobotModel();
  LOG(INFO) << "Robot Model 共享指针第 " << rob_model_.use_count()
            << " 次初始化";
}

/**
 * @brief Construct a new Trajectory Component:: Trajectory Component object
 *
 * @param name
 */
TrajectoryComponent::TrajectoryComponent(const std::string &name)
    : RTT::TaskContext(name, PreOperational),
      out_to_robot_port_("encoder_frame_port"),
      input_from_robot_port_("robot_to_service_port",
                             Orocos::ConnPolicy::data()),
      in_from_service_port_("service_to_traj_port", Orocos::ConnPolicy::data()),
      out_to_exception_port_("exception_status_port") {
  LOG(INFO) << "Trajectory component init.";
  this->rob_model_ = Application::GetContext()->GetRobotModel();
  LOG(INFO) << "Robot Model 共享指针第 " << rob_model_.use_count()
            << " 次初始化";
  this->dof_ = rob_model_->GetDof();
  // 设置运行周期
  this->setActivity(
      new Orocos::Activity(ORO_SCHED_OTHER, 0, 0, 8, 0, "trajectory_activity"));
  this->ports()->addPort(out_to_robot_port_);
  this->ports()->addPort(input_from_robot_port_);
  this->ports()->addEventPort(in_from_service_port_);

  this->ports()->addPort(out_to_exception_port_);

  EncoderFrame encoder_frame;
  out_to_robot_port_.setDataSample(encoder_frame);

  StatusCode exception_frame;
  out_to_exception_port_.setDataSample(exception_frame);

  auto config = *Application::GetContext()->GetConfig();
  int arm_tp = config["init_global_config"]["robot_type"].as<int>();
  switch (arm_tp) {
  case 0:
    this->arm_type_ = ZERO_6;
    break;
  case 1:
    this->arm_type_ = SCARA;
    break;
    break;
  default:
    this->arm_type_ = XB4S;
    break;
  }
  LOG(INFO) << "Trajectory init done.";
}

/**
 * @brief 组件配置函数
 *
 * @return true
 * @return false
 */
bool TrajectoryComponent::configureHook() {
  if (!input_from_robot_port_.connected()) {
    LOG(INFO) << "traj_input_port not connected.";
    throw rosc::RobotException(StatusCode::kTrajectoryPortNotConnected_in_rob);
    return false;
  }
  if (!out_to_robot_port_.connected()) {
    LOG(INFO) << "traj_output_port not connected.";
    throw rosc::RobotException(StatusCode::kTrajectoryPortNotConnected_out_rob);
    return false;
  }

  if (!in_from_service_port_.connected()) {
    LOG(INFO) << "traj_in_from_service_port not connected.";
    throw rosc::RobotException(
        StatusCode::kTrajectoryPortNotConnected_in_service);
    return false;
  }
  this->addOperation("EmergencyStop", &TrajectoryComponent::EmergencyStop, this)
      .doc("EmergencyStop");

  RTT::TaskContext *robot_com = this->getPeer("robot");
  this->GetPlanner = robot_com->getOperation("GetPlanner");
  this->trajectory_planner_ = this->GetPlanner();

  LOG(INFO) << "trajectory planner pointer: "
            << this->trajectory_planner_.use_count();
  return true;
}

/**
 * @brief
 *
 * @return true
 * @return false
 */
bool TrajectoryComponent::startHook() {
  rosc::License licenseTool;
  bool isMatch = licenseTool.checkLicense();
  if (!isMatch) {
    LOG(ERROR) << "license mismatch";
  }
  return isMatch;
}

void TrajectoryComponent::exceptionHook(const std::exception &e) {
  // 将异常发送到异常处理组件
  try {
    rosc::RobotException &re =
        dynamic_cast<rosc::RobotException &>(const_cast<std::exception &>(e));
    switch (re.GetStatusCode()) {
    case StatusCode::kRobotPortNotConnected_in_ec:
      this->out_to_exception_port_.write(re.GetStatusCode());
      break;
    default:
      this->out_to_exception_port_.write(re.GetStatusCode());
      break;
    }
    // 低等异常自己处理

    // 高等级异常抛出，调用
  } catch (std::bad_cast &b) {
    // 转化失败
  }
}
void memcpyFrame(KDL::Frame *cartesian, double *pos) {
  cartesian->p.x(pos[0]);
  cartesian->p.y(pos[1]);
  cartesian->p.z(pos[2]);
  cartesian->M = KDL::Rotation::RPY(pos[3], pos[4], pos[5]);
}
/**
 * @brief
 *
 */
void TrajectoryComponent::updateHook() {
  RobotPos cur_pos;
  KDL::Frame start_cart, end_cart;
  if (this->in_from_service_port_.read(input_command_) == RTT::NewData) {
    switch (input_command_.op_type) {
    case CommandType::MOTION:  // 下发移动指令
      LOG(INFO) << "Trajectory receive motion command.";
      // 两点运动
      this->next_motion_iszone_ = false;
      this->MoveOperation(input_command_.speed, input_command_.start,
                          input_command_.end);
      break;
    case CommandType::MOVETO:
      LOG(INFO) << "Trajectory receive move to single point command.";
      this->next_motion_iszone_ = false;
      this->GetCurrentPosition(&cur_pos);
      this->MoveOperation(input_command_.speed, cur_pos, input_command_.end);
      break;
    case CommandType::MOVEL:
      LOG(INFO) << "Trajectory receive move line command.";
      this->next_motion_iszone_ = false;
      this->MoveLine(input_command_.speed, input_command_.start,
                     input_command_.end);
      break;
    case CommandType::JOG_START:  // 开始点动
      LOG(INFO) << "Trajectory receive Jog command.";
      this->which_axis_ = input_command_.whichAxis;
      this->direction_ = input_command_.direction;
      this->jog_step_ = input_command_.jogstep;
      this->StartMoveJog();
      break;
    case CommandType::POINT_JOG_START:
      LOG(INFO) << "Trajectory receive jog to point command.";
      this->StartMoveToPoint(TransferSpeedType::JOG_SPEED, input_command_.end);
      break;
    case CommandType::JOG_STOP:

      break;
    case CommandType::NOP:
      std::this_thread::sleep_for(std::chrono::seconds(1));
      break;
    default:
      LOG(INFO) << "Trajectory 组件收到未定义的指令类型";
      this->out_to_exception_port_.write(
          StatusCode::kTrajectoryUndefinedCommand);
      break;
    }
  }
}

void TrajectoryComponent::stopHook() {}
void TrajectoryComponent::cleanupHook() {}

double CompareCartesianPos(const KDL::Frame &pos1, const KDL::Frame &pos2) {
  double err = fabs(pos1.p.x() - pos2.p.x()) + fabs(pos1.p.y() - pos2.p.y()) +
               fabs(pos1.p.z() - pos2.p.z());
  return err;
}

/********************************************************************
 * @brief
 *
 * @param sp
 * @param start
 * @param end
 * @param tp_mode
 * @return ExecStatus
 ********************************************************************/
ExecStatus TrajectoryComponent::MoveOperation(const TransferSpeedType &sp,
                                              const RobotPos &start,
                                              const RobotPos &end,
                                              int tp_mode) {
  int speed_level = this->rob_model_->GetSpeedLevel();
  double vel_max[kDofMax], acc_max[kDofMax], jerk[kDofMax], dcc_max[kDofMax],
      dcc_jerk[kDofMax];
  auto config = *Application::GetContext()->GetConfig();
  for (int i = 0; i < this->dof_; ++i) {
    vel_max[i] = this->rob_model_->vel_max_[i];
    if (speed_level == 0) {
      acc_max[i] = this->rob_model_->acc_max_[i];
      jerk[i] = this->rob_model_->jerk_[i];
      dcc_max[i] = this->rob_model_->dcc_max_[i];
      dcc_jerk[i] = this->rob_model_->dcc_jerk_[i];
    } else if (speed_level == 1) {
      acc_max[i] = this->rob_model_->acc_max_[i] * 0.5;
      jerk[i] = this->rob_model_->jerk_[i] * 0.5;
      dcc_max[i] = this->rob_model_->dcc_max_[i] * 0.5;
      dcc_jerk[i] = this->rob_model_->dcc_jerk_[i] * 0.5;
    } else {
      acc_max[i] = this->rob_model_->acc_max_[i] * 0.1;
      jerk[i] = this->rob_model_->jerk_[i] * 0.1;
      dcc_max[i] = this->rob_model_->dcc_max_[i] * 0.1;
      dcc_jerk[i] = this->rob_model_->dcc_jerk_[i] * 0.1;
    }
  }
  if (sp == TransferSpeedType::LOW_SPEED ||
      sp == TransferSpeedType::SPEED_IN_LOW_SPEED_AREA) {
    acc_max[0] = acc_max[0] * 0.3;
    jerk[0] = jerk[0] * 0.3;
    dcc_max[0] = dcc_max[0] * 0.3;
    dcc_jerk[0] = dcc_jerk[0] * 0.3;
  }

  // 路径规划插补
  // *两点运动
  double joint_start[kDofMax], joint_end[kDofMax];
  for (int i = 0; i < this->dof_; ++i) {
    joint_start[i] = start.joint(i);
    joint_end[i] = end.joint(i);
  }
  double err;
  err = this->trajectory_planner_->SetTrajectory(
      joint_start, joint_end, vel_max, acc_max, jerk, acc_max, jerk);

  if (err < 0) {
    this->out_to_exception_port_.write(StatusCode::kTrajectoryMovePlanFailed);
  }

  double duration = this->trajectory_planner_->Duration();
  if (duration < 0 || std::isnan(duration) || std::isinf(duration)) {
    this->out_to_exception_port_.write(StatusCode::kTrajectoryMovePlanFailed);
    return ExecStatus::FAILED;
  }
  LOG(INFO) << "the duration is " << duration;
  EncoderFrame frame;
  frame.mode = MotionMode::MOVE_SEND;
  frame.time_stamp = this->input_command_.timestamp;
  out_to_robot_port_.write(frame);
  return ExecStatus::SUCCESS;
}

void TrajectoryComponent::memcpyJointArray(KDL::JntArray *joint,
                                           const double *pos) {
  for (int i = 0; i < this->dof_; i++) {
    joint->data[i] = pos[i];
  }
}

/**
 * @brief 两点之间笛卡尔运动
 *
 * @param sp
 * @param pos1
 * @param pos2
 * @return ExecStatus
 */
ExecStatus TrajectoryComponent::MoveLineAxis(const TransferSpeedType &sp,
                                             const RobotPos &start,
                                             const RobotPos &end) {
  double vel_max, acc_max, jerk;
  auto config = *Application::GetContext()->GetConfig();
  vel_max = config["zero_device_config"]["zero_device_config"]["robot"]
                  ["joints"]["linear-motion"]["vel_max"]
                      .as<double>();
  acc_max = config["zero_device_config"]["zero_device_config"]["robot"]
                  ["joints"]["linear-motion"]["acc_max"]
                      .as<double>();
  jerk = config["zero_device_config"]["zero_device_config"]["robot"]["joints"]
               ["linear-motion"]["jerk"]
                   .as<double>();
  // 路径规划插补
  //* 笛卡尔运动
  KDL::JntArray start_jnt(this->dof_), end_jnt(this->dof_);
  KDL::Frame cart_start, cart_end;
  this->rob_model_->ForwordKinematicsPos(start.joint, &cart_start);
  this->rob_model_->ForwordKinematicsPos(end.joint, &cart_end);
  double err = this->trajectory_planner_->SetTrajectoryCartesianAxis(
      cart_start, cart_end, vel_max, acc_max, jerk);

  if (err < 0) {
    this->out_to_exception_port_.write(StatusCode::kTrajectoryMovePlanFailed);
    return ExecStatus::FAILED;
  }

  double duration = this->trajectory_planner_->Duration();
  if (duration < 0 || std::isnan(duration) || std::isinf(duration)) {
    this->out_to_exception_port_.write(StatusCode::kTrajectoryMovePlanFailed);
    return ExecStatus::FAILED;
  }
  LOG(INFO) << "the duration is " << duration;
  EncoderFrame frame;
  frame.mode = MotionMode::MOVELINE_SEND;
  frame.time_stamp = std::chrono::time_point_cast<std::chrono::microseconds>(
                         std::chrono::system_clock::now())
                         .time_since_epoch()
                         .count();

  out_to_robot_port_.write(frame);
  // 阻塞，直到指令执行结束
  std::this_thread::sleep_for(
      std::chrono::milliseconds(static_cast<int>(duration * 1000) + 1));
  // 通信同步，阻塞，直到收到下游组件执行完毕
  return ExecStatus::SUCCESS;
}

/********************************************************************
 * @brief
 *
 * @param sp
 * @param start
 * @param end
 * @return ExecStatus
 ********************************************************************/
ExecStatus TrajectoryComponent::MoveLine(const TransferSpeedType &sp,
                                         const RobotPos &start,
                                         const RobotPos &end) {
  // 先判断当前伺服状态是否可执行
  double vel_max, acc_max, jerk;
  auto config = *Application::GetContext()->GetConfig();

  vel_max = config["zero_device_config"]["zero_device_config"]["robot"]
                  ["joints"]["linear-motion"]["vel_max"]
                      .as<double>();
  acc_max = config["zero_device_config"]["zero_device_config"]["robot"]
                  ["joints"]["linear-motion"]["acc_max"]
                      .as<double>();
  jerk = config["zero_device_config"]["zero_device_config"]["robot"]["joints"]
               ["linear-motion"]["jerk"]
                   .as<double>();

  if (this->rob_model_->GetSpeedLevel() == 1) {
    acc_max = acc_max * 0.5;
    jerk = jerk * 0.5;
  } else if (this->rob_model_->GetSpeedLevel() == 2) {
    acc_max = acc_max * 0.1;
    jerk = jerk * 0.1;
  }
  LOG(INFO) << "speed level: " << this->rob_model_->GetSpeedLevel();
  LOG(INFO) << "笛卡尔运动速度：" << vel_max;
  LOG(INFO) << "笛卡尔运动加速度：" << acc_max;
  LOG(INFO) << "笛卡尔运动加加速度：" << jerk;

  for (int i = 0; i < this->dof_; ++i) {
    this->trajectory_planner_->start_[i] = start.joint(i);
    this->trajectory_planner_->end_[i] = end.joint(i);
  }
  this->trajectory_planner_->last_jnt_for_inverse_ = start.joint;
  double err = this->trajectory_planner_->SetTrajectoryCartesianAxis(
      start.cartesian, end.cartesian, vel_max, acc_max, jerk);
  this->trajectory_planner_->s_vp_[0]->PrintPara();
  if (err < 0) {
    this->out_to_exception_port_.write(StatusCode::kTrajectoryMovePlanFailed);
    return ExecStatus::FAILED;
  }

  double duration = this->trajectory_planner_->Duration();
  if (duration < 0 || std::isnan(duration) || std::isinf(duration)) {
    this->out_to_exception_port_.write(StatusCode::kTrajectoryMovePlanFailed);
    return ExecStatus::FAILED;
  }
  LOG(INFO) << "the duration is " << duration;
  EncoderFrame frame;
  frame.mode = MotionMode::MOVELINE_SEND;
  frame.time_stamp = std::chrono::time_point_cast<std::chrono::microseconds>(
                         std::chrono::system_clock::now())
                         .time_since_epoch()
                         .count();
  out_to_robot_port_.write(frame);
  // 阻塞，直到指令执行结束
  std::this_thread::sleep_for(
      std::chrono::milliseconds(static_cast<int>(duration * 1000) + 1));
  // 通信同步，阻塞，直到收到下游组件执行完毕
  return ExecStatus::SUCCESS;
}

/********************************************************************
 * @brief
 *
 * @return ExecStatus
 ********************************************************************/
ExecStatus TrajectoryComponent::StartMoveJog() {
  ExecStatus res;
  rosc::Config config = *rosc::Application::GetContext()->GetConfig();
  RobotPos cur_pos, target_pos;
  this->GetCurrentPosition(&cur_pos);
  target_pos = cur_pos;
  LOG(INFO) << "jog axis: " << this->which_axis_;
  target_pos.joint(which_axis_) =
      this->direction_ > 0 ? this->rob_model_->motion_range_end_[which_axis_]
                           : this->rob_model_->motion_range_start_[which_axis_];
  res = this->StartMoveToPoint(TransferSpeedType::JOG_SPEED, target_pos);
  return res;
}

/**
 * @brief 开启jog移动到指定点位线程
 *
 * @return true
 * @return false
 */
ExecStatus TrajectoryComponent::StartMoveToPoint(const TransferSpeedType &sp,
                                                 const RobotPos &target_pos) {
  RobotPos current_pos;
  double start[kDofMax];
  double end[kDofMax];
  this->GetCurrentPosition(&current_pos);

  double vel_max[kDofMax], acc_max[kDofMax], jerk[kDofMax];

  int speed_level = this->rob_model_->GetSpeedLevel();

  for (int i = 0; i < this->dof_; ++i) {
    vel_max[i] = this->rob_model_->vel_max_[i];
    if (speed_level == 0) {
      acc_max[i] = this->rob_model_->acc_max_[i];
      jerk[i] = this->rob_model_->jerk_[i];
    } else if (speed_level == 1) {
      acc_max[i] = this->rob_model_->acc_max_[i] * 0.5;
      jerk[i] = this->rob_model_->jerk_[i] * 0.5;
    } else {
      acc_max[i] = this->rob_model_->acc_max_[i] * 0.1;
      jerk[i] = this->rob_model_->jerk_[i] * 0.1;
    }
  }
  LOG(INFO) << "速度：" << vel_max[0] << " " << vel_max[1] << " " << vel_max[2]
            << " " << vel_max[3];
  LOG(INFO) << "加速度：" << acc_max[0] << " " << acc_max[1] << " "
            << acc_max[2] << " " << acc_max[3];
  LOG(INFO) << "加加速度：" << jerk[0] << " " << jerk[1] << " " << jerk[2]
            << " " << jerk[3];
  if (sp == TransferSpeedType::LOW_SPEED ||
      sp == TransferSpeedType::SPEED_IN_LOW_SPEED_AREA) {
    acc_max[0] = acc_max[0] * 0.3;
    jerk[0] = jerk[0] * 0.3;
  }
  for (int i = 0; i < this->dof_; ++i) {
    start[i] = current_pos.joint(i);
    end[i] = target_pos.joint(i);
  }

  // 路径规划插补
  // *两点运动
  double err = this->trajectory_planner_->SetTrajectory(
      start, end, vel_max, acc_max, jerk, acc_max, jerk);
  if (err < 0) {
    this->out_to_exception_port_.write(StatusCode::kTrajectoryMovePlanFailed);
  }

  double duration = this->trajectory_planner_->Duration();
  if (duration < 0 || std::isnan(duration) || std::isinf(duration)) {
    this->out_to_exception_port_.write(StatusCode::kTrajectoryMovePlanFailed);
    return ExecStatus::FAILED;
  }
  LOG(INFO) << "the duration is " << duration;
  EncoderFrame frame;
  frame.mode = MotionMode::JOG_ANGLE;
  frame.time_stamp = std::chrono::time_point_cast<std::chrono::microseconds>(
                         std::chrono::system_clock::now())
                         .time_since_epoch()
                         .count();

  out_to_robot_port_.write(frame);
  return ExecStatus::SUCCESS;
}

/**
 * @brief 获取指定点的位置
 *
 * @param point_name
 * @return Position
 */
void TrajectoryComponent::GetPosition(const std::string &point_name,
                                      double *position) {
  std::shared_ptr<Teach> robteach = std::make_shared<Teach>();
  Point_Position pos = robteach->GetPoint(point_name);
  for (int i = 0; i < this->dof_; ++i) {
    position[i] = pos.pos.joint(i);
  }
}

/**
 * @brief 获取当前点位
 *
 * @return RobotPos
 */
RobotPos TrajectoryComponent::GetCurrentPosition(RobotPos *cur_pos) {
  EncoderFrame statusFrame;
  this->input_from_robot_port_.read(statusFrame);
  for (int i = 0; i < this->dof_; ++i) {
    cur_pos->joint(i) = statusFrame.current_position[i];
    std::cout << cur_pos->joint(i) << std::endl;
  }
  this->rob_model_->ForwordKinematicsPos(cur_pos->joint, &cur_pos->cartesian);
  return RobotPos();
}

/**
 * @brief
 *
 */
void TrajectoryComponent::EmergencyStop() {
  this->out_to_robot_port_.clear();
  EncoderFrame frame;
  frame.time_stamp = this->time_stamp_emcy_;
  frame.mode = MotionMode::MOVEFINISH_SEND;
  this->out_to_robot_port_.write(frame);
}

ORO_CREATE_COMPONENT(rosc::TrajectoryComponent)

}  // namespace rosc
