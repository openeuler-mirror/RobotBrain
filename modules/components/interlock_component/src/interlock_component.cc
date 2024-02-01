/********************************************************************
 * @file interlock_component.cc
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-10-10
 *
 * @copyright Copyright (c) 2023 ROSC
 *
 ********************************************************************/
#include <interlock_component.h>
#include "robot_brain/config.h"
#include "robot_brain/core/application.h"
#include "robot_brain/core/license.h"
#include "robot_brain/robot_exception/robot_error.h"
#include "robot_brain/robot_exception/robot_status.h"
#include "robot_brain/teach_point.h"
#include "robot_brain/command_types.hpp"
#include <algorithm>
#include <bits/stdint-intn.h>
#include <bits/stdint-uintn.h>
#include <bitset>
#include <chrono> // NOLINT
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <glog/logging.h>
#include <iostream>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/joint.hpp>
#include <kdl/utilities/utility.h>
#include <link.h>
#include <math.h>
#include <memory>
#include <ostream>
#include <robot_brain/core.hpp>
#include <robot_brain/ethercat_frame_types.hpp>
#include <robot_brain/robot_teach.hpp>
#include <rtt/Activity.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Component.hpp>
#include <rtt/ConnPolicy.hpp>
#include <rtt/FlowStatus.hpp>
#include <rtt/Logger.hpp>
#include <rtt/OperationCaller.hpp>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Time.hpp>
#include <rtt/os/Time.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/os/threads.hpp>
#include <string>
#include <thread> // NOLINT
#include <unistd.h>
#include <vector>
#include <yaml-cpp/node/node.h>

namespace rosc {
InterlockComponent::~InterlockComponent() {
  LOG(INFO) << "Robot component destructor.";
  LOG(INFO) << "Robot model ptr :" << this->rob_model_.use_count();
}

InterlockComponent::InterlockComponent()
    : InterlockComponent(std::string("interlock")) {
  this->rob_model_ = Application::GetContext()->GetRobotModel();
}

InterlockComponent::InterlockComponent(const std::string &name)
    : RTT::TaskContext(name, PreOperational),
      out_to_ethercat_port_("io_ctl_frame_port"),
      input_from_ethercat_port_("status_frame_port",
                                Orocos::ConnPolicy::data()),
      out_to_service_port_("interlock_to_service_port"),
      input_from_service_port_(
          "service_to_interlock_port",
          Orocos::ConnPolicy::buffer(32, Orocos::ConnPolicy::LOCK_FREE, true)),
      out_to_exception_port_("exception_status_port") {
  LOG(INFO) << "Interlock component init.";
  // robot_model指针
  this->rob_model_ = rosc::Application::GetContext()->GetRobotModel();
  // 从文件中读取周期
  rosc::Config config = *rosc::Application::GetContext()->GetConfig();
  seconds = RTT::nsecs_to_Seconds(
      config["bus_config"]["communication_cycle"].as<RTT::nsecs>());
  bool is_sync_to_slave =
      config["bus_config"]["reference_slave_clock"].as<bool>();
  if (is_sync_to_slave) {
    this->setActivity(new Orocos::Activity(
        ORO_SCHED_RT, RTT::os::HighestPriority, 0, 1, 0, "robot_activity"));
    this->ports()->addEventPort(input_from_ethercat_port_);
  } else {
    this->setActivity(new Orocos::Activity(ORO_SCHED_RT,
                                           RTT::os::HighestPriority, seconds, 1,
                                           0, "robot_activity"));
    this->ports()->addPort(input_from_ethercat_port_);
  }
  this->ports()->addPort(out_to_ethercat_port_);
  this->ports()->addPort(out_to_service_port_);
  this->ports()->addPort(input_from_service_port_);
  this->ports()->addPort(out_to_exception_port_);

  rosc::SingleCommand command;
  rosc::DigitIOFrame io_frame;
  this->out_to_ethercat_port_.setDataSample(io_frame);
  rosc::StatusCode exceptionframe;
  this->out_to_exception_port_.setDataSample(exceptionframe);
  rosc::CommandExecResult feed_frame;
  this->out_to_service_port_.setDataSample(feed_frame);

  std::shared_ptr<Teach> robteach = std::make_shared<Teach>();

  this->user_mode_ = 0;
  this->err_io_seq_ = 0;
  int arm_tp = config["init_global_config"]["robot_type"].as<int>();
  YAML::Node slave_list;
  switch (arm_tp) {
  case 0:
    this->arm_type_ = ZERO_6;
    this->slave_num_ =
        config["zero_device_config"]["zero_device_config"]["robot"]["slave_num"]
            .as<int>();
    this->dof_ =
        config["zero_device_config"]["zero_device_config"]["robot"]["dof"]
            .as<int>();
    slave_list = config["zero_device_config"]["zero_device_config"]["robot"]
                       ["slave_type_list"];
    break;
  case 1:
    this->arm_type_ = SCARA;
    this->slave_num_ = config["scara_device_config"]["scara_device_config"]
                             ["robot"]["slave_num"]
                                 .as<int>();
    this->dof_ =
        config["scara_device_config"]["scara_device_config"]["robot"]["dof"]
            .as<int>();
    slave_list = config["scara_device_config"]["scara_device_config"]["robot"]
                       ["slave_type_list"];
    break;
  default:
    this->arm_type_ = XB4S;
    break;
  }
  this->slave_list_ = new SlaveType[this->slave_num_]();
  for (int i = 0; i < this->slave_num_; ++i) {
    int tp = slave_list[i].as<int>();
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
}

bool InterlockComponent::configureHook() {
  if (!input_from_service_port_.connected()) {
    LOG(INFO) << "input from service port not connected.";
    throw rosc::RobotException(StatusCode::kRobotPortNotConnected_in_traj);
    return false;
  }
  if (!this->out_to_ethercat_port_.connected()) {
    LOG(INFO) << "out_to_ethercat_port_ is not connected.";
    throw rosc::RobotException(StatusCode::kRobotPortNotConnected_out_ec);
    return false;
  }
  if (!this->input_from_ethercat_port_.connected()) {
    LOG(INFO) << "input_from_ethercat_port_ is not connected.";
    throw rosc::RobotException(StatusCode::kRobotPortNotConnected_in_ec);
    return false; 
  }

  RTT::TaskContext *robot_com = this->getPeer("robot");
  this->robot_EmergencyRecover = robot_com->getOperation("EmergencyRecover");
  this->CallClearError = robot_com->getOperation("ClearError");
  this->CallPowerOn = robot_com->getOperation("PowerOn");
  this->CallPowerOff = robot_com->getOperation("PowerOff");
  return true;
}

bool InterlockComponent::startHook() { return true; }

void InterlockComponent::exceptionHook(std::exception const &e) {
  // 将异常发送到异常处理组件
  try {
    rosc::RobotException &re =
        dynamic_cast<rosc::RobotException &>(const_cast<std::exception &>(e));
    switch (re.GetStatusCode()) {
    case StatusCode::kRobotPortNotConnected_in_ec:
      this->out_to_exception_port_.write(re.GetStatusCode());
      break;
    case StatusCode::kRobotPortNotConnected:
      this->out_to_exception_port_.write(re.GetStatusCode());
      break;
    // case StatusCode::kRobotPortNotConnected_in_traj:
    //   this->out_to_exception_port_.write(re.GetStatusCode());
    //   break;
    case StatusCode::kRobotPortNotConnected_out_ec:
      this->out_to_exception_port_.write(re.GetStatusCode());
      break;
    case StatusCode::kRobotPortNotConnected_out_traj:
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

void InterlockComponent::updateHook() {
  if (this->input_from_service_port_.read(this->io_command_) == RTT::NewData) {
    // 收到新的io指令
    LOG(INFO) << "interlock 收到新的io指令";
    this->IO_Operation(io_command_.whichIO, io_command_.io_op);
  }
  // 更新io状态
  this->input_from_ethercat_port_.read(this->statusframe_);  // 读取
  double current_pos[kDofMax];
  this->GetCurrentPosition(this->statusframe_, current_pos);

  // 判断是否处于ready状态，在非运动且可以接受指令的情况下为ready
  ServoState state = ServoState::OP;
  if (statusframe_.ethercat_frame_send_status == IDLE) {
    state = ServoState::READY;
    if (!this->rob_model_->CheckServoEMCState(this->statusframe_.status_word)) {
      state = ServoState::EMCY;
    }
    if (!this->rob_model_->CheckServoErrorState(
            this->statusframe_.status_word)) {
      state = ServoState::ERROR;
    }
  }
  // 每个周期都发送一次io数据
  // this->out_to_ethercat_port_.write(next_send_io_frame_);
}

/********************************************************************
 * @brief IO输出操作
 *
 * @param whichio
 * @param op
 * @return true
 * @return false
 ********************************************************************/
bool InterlockComponent::IO_Operation(int whichio, bool op) {
  if (op) {
    this->next_send_io_frame_.digit_io_out |= 0x01 << whichio;
  } else {
    this->next_send_io_frame_.digit_io_out &= ~(0x01 << whichio);
  }
  LOG(INFO) << (std::bitset<32>)(next_send_io_frame_.digit_io_out);
  return true;
}

void InterlockComponent::GetCurrentPosition(EthercatStatusFrame statusFrame,
                                            double *q_out) {
  // 计算当前的joint
  for (int i = 0; i < this->dof_; ++i) {
    q_out[i] = this->rob_model_->Encoder2Position(
        statusFrame.current_position[i], pos_zero_.encoder[i], i);
  }
}

void InterlockComponent::stopHook() {}

void InterlockComponent::cleanupHook() {}

ORO_CREATE_COMPONENT(rosc::InterlockComponent)
}  // namespace rosc
