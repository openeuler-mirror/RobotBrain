/**
 * @file robot_service_component.cc
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief
 * @version 0.1
 * @date 2022-01-09
 *
 * @copyright Copyright (c) 2022 ROSC
 *
 */
#include <robot_service_component.h>
#include <robot_brain/config.h>
#include <robot_brain/robot_exception/robot_status.h>
#include <robot_brain/teach_point.h>
#include <bits/stdint-intn.h>
#include <glog/logging.h>
#include <kdl/utilities/utility.h>
#include <unistd.h>
#include <string>
#include <thread>  // NOLINT
#include <vector>
#include <bitset>
#include <chrono>  // NOLINT
#include <cmath>
#include <cstring>
#include <memory>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/joint.hpp>
#include <robot_brain/core.hpp>
#include <robot_brain/ethercat_frame_types.hpp>
#include <robot_brain/command_types.hpp>
#include <rtt/Activity.hpp>
#include <rtt/ConnPolicy.hpp>
#include <rtt/FlowStatus.hpp>
#include <rtt/OperationCaller.hpp>
#include <rtt/RTT.hpp>
#include <rtt/TaskContext.hpp>
#include <robot_brain/robot_planning.hpp>

namespace rosc {

namespace state {

static void StateChangeLog(std::string state) {
  LOG(INFO) << "|*****************状态转移至：" << state
            << "*****************|";
}

void OffState::entryGuard(FullControl &control) noexcept {
  LOG(INFO) << "In offState state";
}

/***********************针对不同的状态进行响应*******************************/
template <typename Event>
void BaseState::react(const Event &,
                      FullControl &control) noexcept {  // NOLINT
  LOG(WARNING) << "[unsupported state transition]";
  rosc::RobotServiceComponent *p_robot_service =
      control.context().robot_service;
  p_robot_service->PortWriteError();
}

void OffState::react(const event::InitEvent &, FullControl &control) noexcept {
  if (control.context().ready) {
    LOG(INFO) << "change to inital state";
    control.changeTo<InitialState>();
    auto p_robot_service = control.context().robot_service;
    p_robot_service->mechine_state_ = ControllerState::Init;
  } else {
    LOG(ERROR) << "init event error!";
  }
}
// 上电
void InitialState::react(const event::TurnPowerOnEvent &,
                         FullControl &control) noexcept {  // NOLINT
  auto p_robot_service = control.context().robot_service;
  if (p_robot_service->PowerOn()) {
    LOG(INFO) << "Power on, change to the status: IdleState.";
    control.changeTo<IdleState>();
    StateChangeLog("IdleState");
    p_robot_service->mechine_state_ = ControllerState::Idle;
  } else {
    LOG(INFO) << "Power on failed, change to the status: FaultState.";
    control.changeTo<FaultState>();
    StateChangeLog("FaultState");
    p_robot_service->mechine_state_ = ControllerState::Error;
  }
}
void IdleState::react(const event::TurnPowerOnEvent &,
                      FullControl &control) noexcept {  // NOLINT
  auto p_robot_service = control.context().robot_service;
  p_robot_service->PortWriteError();
  LOG(INFO) << "Already power on, IdleState.";
}
// 初始状态，清错
void InitialState::react(const event::RobotClearEvent &,
                         FullControl &control) noexcept {  // NOLINT
  auto p_robot_service = control.context().robot_service;
  int res = p_robot_service->ClearError();
  if (res == 0) {
    LOG(INFO) << "clear error, change to the status: Initialstate.";
    control.changeTo<InitialState>();
    StateChangeLog("InitialState");
    p_robot_service->mechine_state_ = ControllerState::Init;
  } else if (res == 1) {
    LOG(INFO) << "clear error, change to the status: Idlestate.";
    control.changeTo<IdleState>();
    StateChangeLog("IdleState");
    p_robot_service->mechine_state_ = ControllerState::Idle;
  } else if (res == -1) {
    LOG(INFO) << "clear error, change to the status: Errorstate.";
    control.changeTo<FaultState>();
    StateChangeLog("FaultState");
    p_robot_service->mechine_state_ = ControllerState::Error;
  }
  // LOG(INFO) << "clear error, change to the status: Initialstate.";
  // control.changeTo<InitialState>();
  // StateChangeLog("InitialState");
  // p_robot_service->mechine_state_ = ControllerState::Init;
}

void InitialState::react(const event::MoveBetweenPointsEvent &,
                         FullControl &control) noexcept {  // NOLINT
  LOG(INFO) << "未上电状态下无法执行运动指令。";
  auto p_robot_service = control.context().robot_service;
  p_robot_service->PortWriteError();
}

void InitialState::react(const event::StartMoveEvent &,
                         FullControl &Control) noexcept {  // NOLINT
  LOG(INFO) << "未上电状态下无法执行运动指令。";
  auto p_robot_service = Control.context().robot_service;
  p_robot_service->PortWriteError();
}
void InitialState::react(const event::DoneMoveEvent &,
                         FullControl &Control) noexcept {  // NOLINT
  LOG(INFO) << "未上电状态下无法执行运动指令。";
  auto p_robot_service = Control.context().robot_service;
  p_robot_service->PortWriteError();
}

void InitialState::react(const event::TurnPowerOffEvent &,
                         FullControl &Control) noexcept {  // NOLINT
  LOG(INFO) << "未上电状态下无法执行下电指令。";
  auto p_robot_service = Control.context().robot_service;
  p_robot_service->PortWriteError();
}

// 下电
void IdleState::react(const event::TurnPowerOffEvent &,
                      FullControl &control) noexcept {  // NOLINT
  // 实现下电操作
  auto p_robot_service = control.context().robot_service;
  if (p_robot_service->PowerOff()) {
    LOG(INFO) << "Power off, change to the status: IdleState.";
    control.changeTo<InitialState>();
    StateChangeLog("InitialState");
    p_robot_service->mechine_state_ = ControllerState::Init;
  } else {
    control.changeTo<FaultState>();
    StateChangeLog("FaultState");
    p_robot_service->mechine_state_ = ControllerState::Error;
  }
}
/**
 * @brief 点位运动
 *
 * @param control
 */
void IdleState::react(const event::MoveBetweenPointsEvent &,
                      FullControl &control) noexcept {  // NOLINT
  control.changeTo<MoveState>();
  StateChangeLog("MoveState");
  auto p_robot_service = control.context().robot_service;
  p_robot_service->mechine_state_ = ControllerState::Op;
  if (!p_robot_service->Move()) {
    control.changeTo<FaultState>();
    StateChangeLog("FaultState");
    p_robot_service->mechine_state_ = ControllerState::Error;
  }
  // control.changeTo<IdleState>();
  // StateChangeLog("IdleState");
  // p_robot_service->mechine_state_ = ControllerState::Idle;
}

/**
 * @brief
 *
 * @param control
 */
void IdleState::react(const event::MoveStepEvent &,
                      FullControl &control) noexcept {
  control.changeTo<MoveState>();
  StateChangeLog("MoveState");
  auto p_robot_service = control.context().robot_service;
  p_robot_service->mechine_state_ = ControllerState::Op;
  if (p_robot_service->MoveStep()) {
    control.changeTo<IdleState>();
    StateChangeLog("IdleState");
    p_robot_service->mechine_state_ = ControllerState::Idle;
  } else {
    control.changeTo<FaultState>();
    StateChangeLog("FaultState");
    p_robot_service->mechine_state_ = ControllerState::Error;
  }
}

// 运动到指定点位
void IdleState::react(const event::MoveToPointEvent &,
                      FullControl &control) noexcept {  // NOLINT
  control.changeTo<MoveState>();
  StateChangeLog("MoveState");
  auto p_robot_service = control.context().robot_service;
  p_robot_service->mechine_state_ = ControllerState::Op;
  if (!p_robot_service->MoveTo()) {
    control.changeTo<FaultState>();
    StateChangeLog("FaultState");
    p_robot_service->mechine_state_ = ControllerState::Error;
  }
}

void IdleState::react(const event::MoveToCartesianPointEvent &,
                      FullControl &control) noexcept {  // NOLINT
  control.changeTo<MoveState>();
  StateChangeLog("MoveState");
  auto p_robot_service = control.context().robot_service;
  p_robot_service->mechine_state_ = ControllerState::Op;
  if (p_robot_service->MoveToCart()) {
    control.changeTo<IdleState>();
    StateChangeLog("IdleState");
    p_robot_service->mechine_state_ = ControllerState::Idle;
  } else {
    control.changeTo<FaultState>();
    StateChangeLog("FaultState");
    p_robot_service->mechine_state_ = ControllerState::Error;
  }
}

void IdleState::react(const event::MoveLineEvent &,
                      FullControl &control) noexcept {  // NOLINT
  control.changeTo<MoveState>();
  StateChangeLog("MoveState");
  auto p_robot_service = control.context().robot_service;
  p_robot_service->mechine_state_ = ControllerState::Op;
  if (p_robot_service->MoveLine()) {
    control.changeTo<IdleState>();
    StateChangeLog("IdleState");
    p_robot_service->mechine_state_ = ControllerState::Idle;
  } else {
    control.changeTo<FaultState>();
    StateChangeLog("FaultState");
    p_robot_service->mechine_state_ = ControllerState::Error;
  }
}

// JOG开始运动
void IdleState::react(const event::StartMoveEvent &,
                      FullControl &control) noexcept {  // NOLINT.
  // 开始运动，状态机由空闲转换为运动状态
  control.changeTo<MoveState>();
  StateChangeLog("MoveState");
  auto p_robot_service = control.context().robot_service;
  p_robot_service->mechine_state_ = ControllerState::Op;
  bool res = p_robot_service->MoveJogStart();
  if (res == false) {
    control.changeTo<FaultState>();
    StateChangeLog("FaultState");
    p_robot_service->mechine_state_ = ControllerState::Error;
  }
}

void MoveState::react(const event::StartMoveEvent &,
                      FullControl &control) noexcept {
  // 运动状态收到jog指令
  control.changeTo<MoveState>();
  StateChangeLog("MoveState");
  auto p_robot_service = control.context().robot_service;
  p_robot_service->mechine_state_ = ControllerState::Op;
  bool res = p_robot_service->MoveJogStart();
  if (res == false) {
    control.changeTo<FaultState>();
    StateChangeLog("FaultState");
    p_robot_service->mechine_state_ = ControllerState::Error;
  }
}

void IdleState::react(const event::StartMovePointEvent &,
                      FullControl &control) noexcept {  // NOLINT.
  // 开始运动，状态机由空闲转换为运动状态
  control.changeTo<MoveState>();
  StateChangeLog("MoveState");
  auto p_robot_service = control.context().robot_service;
  p_robot_service->mechine_state_ = ControllerState::Op;
  bool res = p_robot_service->PointJogStart();
  if (res == false) {
    control.changeTo<FaultState>();
    StateChangeLog("FaultState");
    p_robot_service->mechine_state_ = ControllerState::Error;
  }
}

void MoveState::react(const event::StartMovePointEvent &,
                      FullControl &control) noexcept {  // NOLINT.
  // 运动状态收到指令
  control.changeTo<MoveState>();
  StateChangeLog("MoveState");
  auto p_robot_service = control.context().robot_service;
  p_robot_service->mechine_state_ = ControllerState::Op;
  bool res = p_robot_service->PointJogStart();
  if (res == false) {
    control.changeTo<FaultState>();
    StateChangeLog("FaultState");
    p_robot_service->mechine_state_ = ControllerState::Error;
  }
}

void IdleState::react(const event::RobotClearEvent &,
                      FullControl &control) noexcept {  // NOLINT
  auto p_robot_service = control.context().robot_service;
  int res = p_robot_service->ClearError();
  if (res == 0) {
    LOG(INFO) << "clear error, change to the status: Initialstate.";
    control.changeTo<InitialState>();
    StateChangeLog("InitialState");
    p_robot_service->mechine_state_ = ControllerState::Init;
  } else if (res == 1) {
    LOG(INFO) << "clear error, change to the status: idlestate.";
    control.changeTo<IdleState>();
    StateChangeLog("IdleState");
    p_robot_service->mechine_state_ = ControllerState::Idle;
  } else if (res == -1) {
    LOG(INFO) << "clear error, change to the status: Errorstate.";
    control.changeTo<FaultState>();
    StateChangeLog("FaultState");
    p_robot_service->mechine_state_ = ControllerState::Error;
  }
  // control.changeTo<InitialState>();
  // StateChangeLog("InitialState");
  //
  // p_robot_service->ClearError();
  // p_robot_service->mechine_state_ = ControllerState::Init;
}

// JOG停止运动
void MoveState::react(const event::DoneMoveEvent &,
                      FullControl &control) noexcept {
  // 停止运动，状态机由运动转换为空闲
  auto p_robot_service = control.context().robot_service;
  bool res = p_robot_service->MoveJogStop();
  if (res == false) {
    control.changeTo<FaultState>();
    StateChangeLog("FaultState");
    p_robot_service->mechine_state_ = ControllerState::Error;
  } else {
    control.changeTo<IdleState>();
    StateChangeLog("IdleState");
    p_robot_service->mechine_state_ = ControllerState::Idle;
  }
}

void MoveState::react(const event::RobotClearEvent &,
                      FullControl &control) noexcept {
  auto p_robot_service = control.context().robot_service;
  int res = p_robot_service->ClearError();
  if (res == 0) {
    LOG(INFO) << "clear error, change to the status: Initialstate.";
    control.changeTo<InitialState>();
    StateChangeLog("InitialState");
    p_robot_service->mechine_state_ = ControllerState::Init;
  } else if (res == 1) {
    LOG(INFO) << "clear error, change to the status: idlestate.";
    control.changeTo<IdleState>();
    StateChangeLog("IdleState");
    p_robot_service->mechine_state_ = ControllerState::Idle;
  } else if (res == -1) {
    LOG(INFO) << "clear error, change to the status: Errorstate.";
    control.changeTo<FaultState>();
    StateChangeLog("FaultState");
    p_robot_service->mechine_state_ = ControllerState::Error;
  }
  // p_robot_service->ClearError();
  // p_robot_service->mechine_state_ = ControllerState::Init;
  // control.changeTo<InitialState>();
}

void MoveState::react(const event::MoveSuccessEvent &,
                      FullControl &control) noexcept {
  auto p_robot_service = control.context().robot_service;
  control.changeTo<IdleState>();
  StateChangeLog("IdleState");
  p_robot_service->mechine_state_ = ControllerState::Idle;
}

void MoveState::react(const event::MoveFailedEvent &,
                      FullControl &control) noexcept {
  auto p_robot_service = control.context().robot_service;
  control.changeTo<FaultState>();
  StateChangeLog("FaultState");
  p_robot_service->mechine_state_ = ControllerState::Error;
}

/**
 * @brief 错误事件
 *
 * @param control
 */
void IdleState::react(const event::RobotFaultEvent &,
                      FullControl &control) noexcept {  // NOLINT
  control.changeTo<FaultState>();
}

void IdleState::react(const event::PointEvent &,
                      FullControl &control) noexcept {  // NOLINT
  auto p_robot_service = control.context().robot_service;
  p_robot_service->SavePoint();
}
// void MoveState::enter(PlanControl &control) noexcept {
//   auto plan = control.plan();
//   plan.change<MoveReadyState, MoveLineState>();
//   plan.change<MoveLineState, MoveReadyState>();
// }

// void MoveState::planSucceeded(FullControl &control) noexcept {  // NOLINT.
//   control.changeTo<IdleState>();
// }

// void MoveState::planFailed(FullControl &control) noexcept {  // NOLINT.
//   control.changeTo<FaultState>();
// }

void MoveReadyState::update(FullControl &control) noexcept {
  control.changeTo<MoveLineState>();
}

void MoveLineState::update(FullControl &control) noexcept {  // NOLINT
  control.changeTo<MoveReadyState>();
}

void FaultState::react(const event::RobotClearEvent &,
                       FullControl &control) noexcept {  // NOLINT
  // control.changeTo<InitialState>();
  // StateChangeLog("InitialState");
  auto p_robot_service = control.context().robot_service;
  int res = p_robot_service->ClearError();
  if (res == 0) {
    LOG(INFO) << "clear error, change to the status: Initialstate.";
    control.changeTo<InitialState>();
    StateChangeLog("InitialState");
    p_robot_service->mechine_state_ = ControllerState::Init;
  } else if (res == 1) {
    LOG(INFO) << "clear error, change to the status: idlestate.";
    control.changeTo<IdleState>();
    StateChangeLog("IdleState");
    p_robot_service->mechine_state_ = ControllerState::Idle;
  } else if (res == -1) {
    LOG(INFO) << "clear error, change to the status: Errorstate.";
    control.changeTo<FaultState>();
    StateChangeLog("FaultState");
    p_robot_service->mechine_state_ = ControllerState::Error;
  }
  // p_robot_service->ClearError();
  // p_robot_service->mechine_state_ = ControllerState::Init;
}

};  // namespace state

void HexToString(StatusCode code, char *str) {
  double err[6];
  for (int i = 0; i < 6; ++i) {
    uint32_t t = (code >> ((5 - i) * 4)) & 0x0F;
    switch (t) {
    case 0x00:
      err[i] = '0';
      break;
    case 0x01:
      err[i] = '1';
      break;
    case 0x02:
      err[i] = '2';
      break;
    case 0x03:
      err[i] = '3';
      break;
    case 0x04:
      err[i] = '4';
      break;
    case 0x05:
      err[i] = '5';
      break;
    case 0x06:
      err[i] = '6';
      break;
    case 0x07:
      err[i] = '7';
      break;
    case 0x08:
      err[i] = '8';
      break;
    case 0x09:
      err[i] = '9';
      break;
    case 0x0A:
      err[i] = 'A';
      break;
    case 0x0B:
      err[i] = 'B';
      break;
    case 0x0C:
      err[i] = 'C';
      break;
    case 0x0D:
      err[i] = 'D';
      break;
    case 0x0E:
      err[i] = 'E';
      break;
    case 0x0F:
      err[i] = 'F';
      break;
    default:
      break;
    }
  }
  str[0] = err[0];
  str[1] = err[3];
  str[2] = err[4];
  str[3] = err[5];
}
/**
 * @brief
 * 20230906，用于传感器状态转换成string
 * @return string
 * @return
 */
std::string MyHexToString(int _data) {
  std::string str;
  switch (_data) {
  case 0x00:
    str = '0';
    break;
  case 0x01:
    str = '1';
    break;
  case 0x02:
    str = '2';
    break;
  case 0x03:
    str = '3';
    break;
  case 0x04:
    str = '4';
    break;
  case 0x05:
    str = '5';
    break;
  case 0x06:
    str = '6';
    break;
  case 0x07:
    str = '7';
    break;
  case 0x08:
    str = '8';
    break;
  case 0x09:
    str = '9';
    break;
  case 0x0A:
    str = 'A';
    break;
  case 0x0B:
    str = 'B';
    break;
  case 0x0C:
    str = 'C';
    break;
  case 0x0D:
    str = 'D';
    break;
  case 0x0E:
    str = 'E';
    break;
  case 0x0F:
    str = 'F';
    break;
  default:
    break;
  }
  return str;
}

RobotServiceComponent::~RobotServiceComponent() {}
RobotServiceComponent::RobotServiceComponent()
    : RobotServiceComponent(std::string("robot_service")) {}

RobotServiceComponent::RobotServiceComponent(const std::string &name)
    : RTT::TaskContext(name, PreOperational),
      state_machine_(state_machine_context_),
      command_input_port_(
          "command_input_port",
          Orocos::ConnPolicy::buffer(64, Orocos::ConnPolicy::LOCKED)),
      command_exec_result_port_("command_exec_result_port"),
      out_to_traj_port_("service_to_traj_port"),
      in_from_robot_port_("robot_to_service_port", Orocos::ConnPolicy::data()),
      in_from_ethercat_port_("status_frame_port", Orocos::ConnPolicy::data()),
      out_to_interlock_port_("service_to_interlock_port"),
      out_to_exception_port_("exception_status_port"),
      in_from_exception_port_("exception_output_port",
                              Orocos::ConnPolicy::data()) {
  this->setActivity(new Orocos::Activity(ORO_SCHED_OTHER, 0, 0, 4, 0,
                                         "robot_service_activity"));
  // 上游组件端口
  this->ports()->addEventPort(command_input_port_);
  this->ports()->addPort(command_exec_result_port_);
  rosc::CommandExecResult sample;
  command_exec_result_port_.setDataSample(sample);

  // 下游组件端口
  this->ports()->addPort(out_to_traj_port_);
  rosc::SingleCommand singlecommand;
  out_to_traj_port_.setDataSample(singlecommand);

  // 来自robot组件
  this->ports()->addEventPort(in_from_robot_port_);

  // 下游interlock组件
  this->ports()->addPort(out_to_interlock_port_);
  out_to_interlock_port_.setDataSample(singlecommand);

  // 下游ethercat组件端口
  this->ports()->addPort(in_from_ethercat_port_);

  this->ports()->addPort(out_to_exception_port_);
  this->ports()->addPort(in_from_exception_port_);
  rosc::StatusCode exception_frame;
  out_to_exception_port_.setDataSample(exception_frame);

  this->rob_model_ = Application::GetContext()->GetRobotModel();
  this->dof_ = this->rob_model_->GetDof();
  LOG(INFO) << "RobotService: Robot Model 共享指针第 " << rob_model_.use_count()
            << " 次初始化";
  std::shared_ptr<Teach> teach = std::make_shared<Teach>();

  this->pos_zero_ = teach->GetPoint("ZERO");

  this->command_track_ = std::make_shared<CommandTrack>(20);

  LOG(INFO) << "============R Service Component Init Done====================";
}
/**
 * @brief
 *
 * @return true
 * @return false
 */
bool RobotServiceComponent::configureHook() {
  state_machine_context_.robot_service = this;
  state_machine_context_.ready = true;
  this->addOperation("GetCurrentPosition",
                     &RobotServiceComponent::GetCurrentPosition, this)
      .doc("GetCurrentPosition");
  this->addOperation("EmergencyStop", &RobotServiceComponent::Emergency_stop,
                     this)
      .doc("EmergencyStop");
  this->addOperation("EmergencyRecover",
                     &RobotServiceComponent::Emergency_recover, this)
      .doc("EmergencyRecover");

  state_machine_.react(rosc::event::InitEvent{});
  if (!this->out_to_exception_port_.connected()) {
    LOG(WARNING) << "out to exception port is not connected.";
    return false;
  }
  if (!this->in_from_robot_port_.connected()) {
    LOG(WARNING) << "command in from robot port is not connected.";
    this->out_to_exception_port_.write(
        StatusCode::kRobotServicePortNotConnected_out_ec);
    return false;
  }
  if (!this->in_from_ethercat_port_.connected()) {
    LOG(WARNING) << "command input from ethercat port is not connected.";
    return false;
  }
  if (!this->command_input_port_.connected()) {
    LOG(WARNING) << "command input port is not connected.";
    // return false;
  }
  if (!this->command_exec_result_port_.connected()) {
    LOG(WARNING) << "command execute result port is not connected.";
    // return false;
  }

  RTT::TaskContext *robot_com = this->getPeer("robot");
  this->robot_EmergencyStop = robot_com->getOperation("EmergencyStop");
  this->robot_EmergencyRecover = robot_com->getOperation("EmergencyRecover");
  // this->IOOperation = robot_com->getOperation("IO_Operation");
  this->SavePoint_Operation = robot_com->getOperation("SavePoint_Operation");
  this->CallClearError = robot_com->getOperation("ClearError");
  this->CallPowerOn = robot_com->getOperation("PowerOn");
  this->CallPowerOff = robot_com->getOperation("PowerOff");

  RTT::TaskContext *traj_com = this->getPeer("trajectory");
  this->traj_EmergencyStop = traj_com->getOperation("EmergencyStop");

  RTT::TaskContext *exception_com = this->getPeer("exception");
  this->GetCurrentExceptionCode =
      exception_com->getOperation("GetCurrentExceptionCode");
  return true;
}

bool RobotServiceComponent::startHook() {
  rosc::License licenseTool;
  bool isMatch = licenseTool.checkLicense();
  if (!isMatch) {
    LOG(ERROR) << "license mismatch";
  }
  return isMatch;
  return true;
}

void RobotServiceComponent::updateHook() {
  // 触发updateHook：旧指令执行结束触发，新指令发送触发
  EncoderFrame frame;
  RTT::FlowStatus feed_status = in_from_robot_port_.read(frame);
  if (feed_status == RTT::NewData) {
    // 收到指令执行结束数据
    LOG(INFO) << "robot service 收到指令执行结束的feedback";
    if (frame.mode == FAILURE_FEEDBACK ||
        frame.servo_state == ServoState::EMCY ||
        frame.servo_state == ServoState::ERROR) {
      // 执行失败
      this->feedback_command_.status = ExecStatus::FAILED;
      this->state_machine_.react(event::MoveFailedEvent{});
    } else {
      this->feedback_command_.status = ExecStatus::SUCCESS;
      this->state_machine_.react(event::MoveSuccessEvent{});
    }
    this->feedback_command_.tp = command_.op_type;
    this->feedback_command_.timestamp = command_.timestamp;
    this->command_exec_result_port_.write(this->feedback_command_);
  }
  RTT::FlowStatus status = command_input_port_.read(command_);
  if (status == RTT::NewData) {
    LOG(INFO) << "---------------------- A new command is excuting in "
                 "robot_service_component "
                 "-----------------------------";
    // 可以根据指令内容，增加更多其他的指令处理
    switch (command_.op_type) {
    case CommandType::POWER_ON:
      LOG(INFO) << "power on command.";
      state_machine_.react(rosc::event::TurnPowerOnEvent{});
      break;
    case CommandType::POWER_OFF:
      LOG(INFO) << "power off command.";
      state_machine_.react(rosc::event::TurnPowerOffEvent{});
      break;
    case CommandType::MHOM:
      LOG(INFO) << "MHOM command.";
      this->command_track_->push(command_);
      break;
    case CommandType::MOTION:
      LOG(INFO) << "Move between points command.";
      this->command_track_->push(command_);
      state_machine_.react(rosc::event::MoveBetweenPointsEvent{});
      break;
    case CommandType::MOVEZONE:
      LOG(INFO) << "Move with mid point command.";
      this->command_track_->push(command_);
      state_machine_.react(rosc::event::MoveBetweenPointsEvent{});
      break;
    case CommandType::MOVETO:
      LOG(INFO) << "Move to the target position command.";
      this->command_track_->push(command_);
      state_machine_.react(rosc::event::MoveToPointEvent{});
      break;
    case CommandType::MOVETOCART:
      LOG(INFO) << "Move to the cartesian position command.";
      this->command_track_->push(command_);
      state_machine_.react(rosc::event::MoveToCartesianPointEvent{});
      break;
    case CommandType::MOVEL:
      LOG(INFO) << "Move Line command.";
      this->command_track_->push(command_);
      state_machine_.react(rosc::event::MoveLineEvent{});
      break;
    case CommandType::MOVE_STEP:
      LOG(INFO) << "Move step command.";
      this->command_track_->push(command_);
      state_machine_.react(rosc::event::MoveStepEvent{});
      break;
    case CommandType::IO_IN:
      LOG(INFO) << "Check digit io in command.";
      this->IO_In_Operation();
      break;
    case CommandType::IO_OUT:
      // IO 操作和状态机关系
      LOG(INFO) << "Digit IO out command.";
      //
      this->IO_Operation();
      break;
    case CommandType::CLEAR:
      LOG(INFO) << "clear command.";
      state_machine_.react(rosc::event::RobotClearEvent{});
      break;
    case CommandType::JOG_START:
      LOG(INFO) << "Jog start command.";
      this->command_track_->push(command_);
      state_machine_.react(rosc::event::StartMoveEvent{});
      break;
    case CommandType::POINT_JOG_START:
      LOG(INFO) << "Point Jog start command.";
      this->command_track_->push(command_);
      state_machine_.react(rosc::event::StartMovePointEvent{});
      break;
    case CommandType::JOG_STOP:
      LOG(INFO) << "Jog stop command.";
      state_machine_.react(rosc::event::DoneMoveEvent{});
      // this->MoveJogStop();
      break;
    case CommandType::POINT:
      LOG(INFO) << "Save Point pos";
      state_machine_.react(rosc::event::PointEvent{});
      // this->SavePoint();
      break;
    case CommandType::SSPD:
      LOG(INFO) << "Set motion Speed";
      this->SetMotionSpeed();
      break;
    case CommandType::SSLV:
      LOG(INFO) << "Set speed level";
      this->SetSpeedLevel();
      break;
    case CommandType::NOP:
      LOG(INFO) << "Nop command.";
      this->NopCommand(command_.milliseconds);
      break;
    case CommandType::FAULT:
      LOG(INFO) << "Fault command.";
      this->FaultCommand();
      break;
    default:
      LOG(WARNING) << "收到未处理的指令类型：" << command_.op_type;
      break;
      }

    this->trigger();
    LOG(INFO) << "-·-·-·-·-·-·-·-·-·-·-·-·-·-·-·-·-·-·-·-·-·-·-·-·-·-·-·-·-·-·-"
                 "·-·-·-·-·-·-·-·-·-·-·-·-·-·-";
  }
}
void RobotServiceComponent::stopHook() {}
void RobotServiceComponent::cleanupHook() {}

/**
 * @brief 机器人上电
 *
 */
bool RobotServiceComponent::PowerOn() {
  // 具体实现内容
  LOG(INFO) << "robot service send command power on.";
  // 先执行io电磁阀检测
  bool res = this->CallPowerOn();
  feedback_command_.timestamp = command_.timestamp;
  LOG(INFO) << "PowerOn time stamp: " << std::hex << command_.timestamp;
  feedback_command_.tp = CommandType::POWER_ON;
  feedback_command_.status = res ? SUCCESS : FAILED;
  if (feedback_command_.status == ExecStatus::SUCCESS) {
    memset(feedback_command_.errcode, '0', sizeof(feedback_command_.errcode));
  } else {
    HexToString(this->GetCurrentExceptionCode(), feedback_command_.errcode);
  }
  command_exec_result_port_.write(feedback_command_);
  if (feedback_command_.status == ExecStatus::SUCCESS) {
    return true;
  } else {
    return false;
  }
}

/**
 * @brief 机器人下电
 *
 */
bool RobotServiceComponent::PowerOff() {
  LOG(INFO) << "robot service send command power off.";
  bool res = this->CallPowerOff();
  feedback_command_.status = res ? SUCCESS : FAILED;
  feedback_command_.timestamp = command_.timestamp;
  feedback_command_.tp = CommandType::POWER_OFF;
  if (feedback_command_.status == ExecStatus::SUCCESS) {
    memset(feedback_command_.errcode, '0', sizeof(feedback_command_.errcode));
    this->GetJoint(this->mem_pos_);
  } else {
    HexToString(this->GetCurrentExceptionCode(), feedback_command_.errcode);
  }
  command_exec_result_port_.write(feedback_command_);
  if (feedback_command_.status == ExecStatus::SUCCESS) {
    return true;
  } else {
    return false;
  }
}

bool RobotServiceComponent::CheckJointLimit(const RobotPos &pos) {
  for (int i = 0; i < this->dof_; ++i) {
    if (pos.joint(i) < this->rob_model_->motion_range_start_[i] ||
        pos.joint(i) > this->rob_model_->motion_range_end_[i]) {
      LOG(WARNING) << "第" << i << "个关节超出运动限制: ";
      LOG(WARNING) << pos.joint(i) << " ["
                   << this->rob_model_->motion_range_start_[i] << ", "
                   << this->rob_model_->motion_range_end_[i] << "]";
      switch (i) {
      case 0:
        this->out_to_exception_port_.write(
            StatusCode::kTrajectoryMoveBeyondLimits_1);
        break;
      case 1:
        this->out_to_exception_port_.write(
            StatusCode::kTrajectoryMoveBeyondLimits_2);
        break;
      case 2:
        this->out_to_exception_port_.write(
            StatusCode::kTrajectoryMoveBeyondLimits_3);
        break;
      case 3:
        this->out_to_exception_port_.write(
            StatusCode::kTrajectoryMoveBeyondLimits_4);
        break;
      default:
        break;
      }
      return false;
    }
  }
  return true;
}
double RobotServiceComponent::ComparePosition(const RobotPos &pos1,
                                              const RobotPos &pos2) {
  double res = 0;
  for (int i = 0; i < this->dof_; i++) {
    res += fabs(pos1.joint(i) - pos2.joint(i));
  }
  return res;
}
bool RobotServiceComponent::ParamValidityCheck(const RobotPos &start,
                                               const RobotPos &end) {
  double robot_pos[kDofMax];
  this->GetJoint(robot_pos);
  RobotPos current_pos;
  for (int i = 0; i < this->dof_; ++i) {
    current_pos.joint(i) = robot_pos[i];
  }
  // 先检测一下运动范围是否符合
  if (!this->CheckJointLimit(current_pos)) {
    LOG(INFO) << "当前位置不在运动范围内";
    this->out_to_exception_port_.write(
        StatusCode::kTrajectoryMoveBeyondLimits_current);
    return false;
  }
  if (!this->CheckJointLimit(start)) {
    this->out_to_exception_port_.write(
        StatusCode::kTrajectoryMoveBeyondLimits_start);
    return false;
  }
  if (!this->CheckJointLimit(end)) {
    this->out_to_exception_port_.write(
        StatusCode::kTrajectoryMoveBeyondLimits_end);
    return false;
  }

  // 判断 pos1 和 current_pos是否一致，如果差距过大需要调整
  std::cout << "[";
  for (int i = 0; i < this->dof_; i++) {
    std::cout << start.joint(i) << " ";
  }
  std::cout << "] - [";
  for (int i = 0; i < this->dof_; i++) {
    std::cout << end.joint(i) << " ";
  }
  std::cout << "]" << std::endl;

  double error = ComparePosition(current_pos, start);
  if (error > 0.5) {
    LOG(WARNING) << "起点位置累积误差：" << error;
    this->out_to_exception_port_.write(
        StatusCode::kTrajectoryMoveStartMismatch);
    return false;
  }
  return true;
}

/**
 * @brief 机器人移动操作
 *
 */
bool RobotServiceComponent::Move() {
  // 进行参数检查
  if (!ParamValidityCheck(command_.start, command_.end)) {
    feedback_command_.tp = command_.op_type;
    feedback_command_.status = ExecStatus::FAILED;
    feedback_command_.timestamp = command_.timestamp;
    this->command_exec_result_port_.write(feedback_command_);
    return false;
  }
  this->out_to_traj_port_.write(command_);
  return true;
}

/**
 * @brief 机器人运动到指定点位
 *
 * @return true
 * @return false
 */
bool RobotServiceComponent::MoveTo() {
  double cur_pos[kDofMax];
  this->GetJoint(cur_pos);
  for (int i = 0; i < this->dof_; ++i) {
    command_.start.joint(i) = cur_pos[i];
  }
  command_.op_type = MOTION;
  return this->Move();
}

bool RobotServiceComponent::MoveToCart() {
  this->out_to_traj_port_.write(command_);
  return true;
}

bool RobotServiceComponent::MoveLine() {
  this->out_to_traj_port_.write(command_);
  return true;
}

/**
 * @brief 步长运动
 *
 * @return true
 * @return false
 */
bool RobotServiceComponent::MoveStep() {
  this->out_to_traj_port_.write(command_);
  return true;
}

/**
 * @brief 开始点动
 *
 */
bool RobotServiceComponent::MoveJogStart() {
  // 检查收到的指令格式是否正确
  if (!(command_.op_type == CommandType::JOG_START)) {
    // 异常处理
    LOG(INFO) << "不匹配的指令类型。";
    return false;
  }
  this->out_to_traj_port_.write(command_);
  return true;
}

/**
 * @brief
 *
 * @return true
 * @return false
 */
bool RobotServiceComponent::PointJogStart() {
  // 检查收到的指令格式是否正确
  if (!(command_.op_type == CommandType::POINT_JOG_START)) {
    // 异常处理
    LOG(INFO) << "不匹配的指令类型。";
    return false;
  }
  this->out_to_traj_port_.write(command_);

  return true;
}

/**
 * @brief 结束点动
 *
 */
bool RobotServiceComponent::MoveJogStop() {
  // 检查收到的指令格式是否正确
  if (!(command_.op_type == CommandType::JOG_STOP)) {
    // 异常处理
    LOG(INFO) << "不匹配的指令类型。";
    return false;
  }
  this->out_to_traj_port_.write(command_);

  return true;
}
/**
 * @brief 下发清除错误指令
 *
 */
int RobotServiceComponent::ClearError() {
  command_.op_type = CommandType::CLEAR;
  bool res = this->CallClearError();
  feedback_command_.status = res ? ExecStatus::SUCCESS : ExecStatus::FAILED;
  feedback_command_.tp = CommandType::CLEAR;
  feedback_command_.timestamp = command_.timestamp;
  command_exec_result_port_.write(feedback_command_);
  EthercatStatusFrame statusFrame;
  this->in_from_ethercat_port_.read(statusFrame);
  if (rob_model_->CheckServoPowerState(statusFrame.status_word)) {
    // LOG(INFO) << "clear error return 1.";
    return 1;
  }
  if (!rob_model_->CheckServoErrorState(statusFrame.status_word)) {
    // LOG(INFO) << "clear error return -1.";
    return -1;
  }
  if (!rob_model_->CheckServoEMCState(statusFrame.status_word)) {
    // LOG(INFO) << "clear error return -2.";
    return -2;
  }
  return 0;
}

void RobotServiceComponent::IOSend(int whichio, bool op) {
  SingleCommand command;
  command.whichIO = whichio;
  command.io_op = op;
  this->out_to_interlock_port_.write(command);
}
/**
 * @brief
 *
 */
bool RobotServiceComponent::IO_Operation() {
  if (!(command_.op_type == CommandType::IO_OUT)) {
    LOG(INFO) << "指令类型不匹配。";
    return false;
  }
  LOG(INFO) << "Command io out: which io: " << command_.whichIO
            << " op: " << command_.io_op;
  // bool res = this->IOOperation(command_.whichIO, command_.io_op);
  bool res = true;
  IOSend(command_.whichIO, command_.io_op);
  feedback_command_.status = res ? ExecStatus::SUCCESS : ExecStatus::FAILED;
  feedback_command_.tp = CommandType::IO_OUT;
  feedback_command_.timestamp = command_.timestamp;
  LOG(INFO) << std::hex << feedback_command_.timestamp << "IO 指令执行结果： "
            << res;
  // 将执行结果转发到上游组件
  if (feedback_command_.status == ExecStatus::SUCCESS) {
    memset(feedback_command_.errcode, '0', sizeof(feedback_command_.errcode));
  } else {
    HexToString(this->GetCurrentExceptionCode(), feedback_command_.errcode);
  }
  command_exec_result_port_.write(feedback_command_);
  if (feedback_command_.status == ExecStatus::SUCCESS) {
    return true;
  } else {
    return false;
  }
}

/**
 * @brief
 *
 * @return true
 * @return false
 */
bool RobotServiceComponent::IO_In_Operation() {
  if (!(command_.op_type == CommandType::IO_IN)) {
    LOG(INFO) << "指令类型不匹配。";
    return false;
  }
  LOG(INFO) << "IO signal check: IO seq: " << command_.whichIO;
  LOG(INFO) << "IO op-type: " << command_.io_op;
  if (command_.whichIO >= 24) {
    feedback_command_.status = ExecStatus::SUCCESS;
    feedback_command_.tp = CommandType::IO_IN;
    feedback_command_.timestamp = command_.timestamp;
    memset(feedback_command_.errcode, '0', sizeof(feedback_command_.errcode));
    this->command_exec_result_port_.write(feedback_command_);
    return true;
  }
  if (command_.whichIO < 0) {
    feedback_command_.status = ExecStatus::FAILED;
    feedback_command_.tp = CommandType::IO_IN;
    feedback_command_.timestamp = command_.timestamp;
    this->out_to_exception_port_.write(KInvalidSignal);
    HexToString(KInvalidSignal, feedback_command_.errcode);
    this->command_exec_result_port_.write(feedback_command_);
    return true;
  }
  RobotCurrentState cur_state;
  bool res;
  uint32_t digit_io_in = 0;
  int i = this->io_in_check_timeout_ * 10;
  while (i > 0) {
    this->GetCurrentPosition(&cur_state);
    digit_io_in = cur_state.digit_io_in;
    if (command_.io_op) {
      // true 是释放
      res = ((digit_io_in >> command_.whichIO) & 0x01) == 1;
      } else {
        // false 是夹持
        res = ((digit_io_in >> command_.whichIO) & 0x01) == 0;
      }
      if (res) {
        break;
      }
      // 延迟判断
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      i--;
  }
  LOG(INFO) << "Cur sensor value: "
            << ((digit_io_in >> command_.whichIO) & 0x01);
  StatusCode errcode;
  if (!res) {
    HexToString(errcode, feedback_command_.errcode);
    this->out_to_exception_port_.write(errcode);
  } else {
    memset(feedback_command_.errcode, '0', sizeof(feedback_command_.errcode));
  }
  feedback_command_.tp = CommandType::IO_IN;
  feedback_command_.status = res ? ExecStatus::SUCCESS : ExecStatus::FAILED;
  feedback_command_.timestamp = command_.timestamp;
  this->command_exec_result_port_.write(feedback_command_);
  return res;
}

/**
 * @brief 获取当前位置，伺服信息和状态
 *
 * @param pos
 */
void RobotServiceComponent::GetCurrentPosition(RobotCurrentState *const pos) {
  EthercatStatusFrame statusFrame;
  this->in_from_ethercat_port_.read(statusFrame);
  Point_Position cur_pos(statusFrame.current_position);
  // 计算当前位置
  for (int i = 0; i < kDof; i++) {
    cur_pos.pos.joint(i) = rob_model_->Encoder2Position(
        statusFrame.current_position[i], pos_zero_.encoder[i], i);
  }
  // 计算当前速度
  pos->state = ControllerState::Init;
  if (rob_model_->CheckServoPowerState(statusFrame.status_word)) {
    pos->state = ControllerState::Idle;
  }
  if (!rob_model_->CheckServoEMCState(statusFrame.status_word)) {
    pos->state = ControllerState::EMC_STOP;
  }
  if (!rob_model_->CheckServoErrorState(statusFrame.status_word)) {
    pos->state = ControllerState::Error;
  }
  if (this->mechine_state_ == ControllerState::Op) {
    pos->state = ControllerState::Op;
  }
  if (this->mechine_state_ == ControllerState::Error) {
    pos->state = ControllerState::Error;
  }

  for (int i = 0; i < kDof; i++) {
    pos->joint[i] = cur_pos.pos.joint(i);
  }
  pos->digit_io_out = statusFrame.digit_io_out;
  pos->digit_io_in = statusFrame.digit_io_in;

  KDL::Frame target_T;
  this->rob_model_->ForwordKinematicsPos(cur_pos.pos.joint, &target_T, 'L');
  pos->cartesian = target_T;
  pos->current_speed_level = rob_model_->speed_level_;
}

/********************************************************************
 * @brief
 *
 * @param position
 ********************************************************************/
void RobotServiceComponent::GetJoint(double *position) {
  EthercatStatusFrame statusFrame;
  this->in_from_ethercat_port_.read(statusFrame);
  Point_Position cur_pos(statusFrame.current_position);
  // 计算当前位置
  for (int i = 0; i < this->dof_; i++) {
    position[i] = rob_model_->Encoder2Position(statusFrame.current_position[i],
                                                pos_zero_.encoder[i], i);
  }
}
/**
 * @brief 伺服急停
 *
 */
void RobotServiceComponent::Emergency_stop() {
  this->robot_EmergencyStop();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  this->traj_EmergencyStop();
  this->mechine_state_ = ControllerState::EMC_STOP;
}

/**
 * @brief 伺服急停恢复
 *
 */
void RobotServiceComponent::Emergency_recover() {
  this->robot_EmergencyRecover();
  this->mechine_state_ = ControllerState::Init;
}

/**
 * @brief 点位操作
 *
 */
bool RobotServiceComponent::SavePoint() {
  bool res = this->SavePoint_Operation(command_.point_name);
  feedback_command_.tp = command_.op_type;
  feedback_command_.timestamp = command_.timestamp;
  feedback_command_.status = res ? ExecStatus::SUCCESS : ExecStatus::FAILED;
  if (feedback_command_.status == ExecStatus::SUCCESS) {
    memset(feedback_command_.errcode, '0', sizeof(feedback_command_.errcode));
  } else {
    HexToString(this->GetCurrentExceptionCode(), feedback_command_.errcode);
  }
  command_exec_result_port_.write(feedback_command_);
  LOG(INFO) << "RobotService组件发送\"保存点位\"指令执行反馈。";
  // 保存点位之后更新一下零点位置信息
  std::shared_ptr<Teach> teach = std::make_shared<Teach>();
  this->pos_zero_ = teach->GetPoint("ZERO");

  RobotCurrentState *pos = new RobotCurrentState();
  this->GetCurrentPosition(pos);
  LOG(INFO) << "robot service 组件更新零点位置";
  delete pos;
  if (feedback_command_.status == ExecStatus::SUCCESS) {
    return true;
  } else if (feedback_command_.status == ExecStatus::FAILED) {
    return false;
  } else {
    return false;
  }
}

/**
 * @brief 不支持在当前状态下执行，指令不报错
 *
 */
void RobotServiceComponent::PortWriteError() {
  CommandExecResult result;
  result.timestamp = this->command_.timestamp;
  result.tp = this->command_.op_type;
  result.status = ExecStatus::SUCCESS;
  if (feedback_command_.status == ExecStatus::SUCCESS) {
    memset(feedback_command_.errcode, '0', sizeof(feedback_command_.errcode));
  } else {
    HexToString(this->GetCurrentExceptionCode(), feedback_command_.errcode);
  }
  this->command_exec_result_port_.write(result);
}

/********************************************************************
 * @brief 跳过执行指令，直接返回true
 *
 ********************************************************************/
void RobotServiceComponent::SkipCommand() {
  CommandExecResult result;
  result.timestamp = this->command_.timestamp;
  result.tp = this->command_.op_type;
  result.status = ExecStatus::SUCCESS;
  if (feedback_command_.status == ExecStatus::SUCCESS) {
    memset(feedback_command_.errcode, '0', sizeof(feedback_command_.errcode));
  } else {
    HexToString(this->GetCurrentExceptionCode(), feedback_command_.errcode);
  }
  this->command_exec_result_port_.write(result);
}

/**
 * @brief 设置速度等级
 *
 * @return true
 * @return false
 */
bool RobotServiceComponent::SetSpeedLevel() {
  if (this->command_.op_type != CommandType::SSLV) {
    return false;
  }
  LOG(INFO) << "SSLV: set speed level " << command_.speedLevel;
  // 修改robot model对象中的成员变量
  this->rob_model_->speed_level_ = command_.speedLevel;
  feedback_command_.tp = SSLV;
  feedback_command_.status = ExecStatus::SUCCESS;
  feedback_command_.timestamp = command_.timestamp;
  if (feedback_command_.status == ExecStatus::SUCCESS) {
    memset(feedback_command_.errcode, '0', sizeof(feedback_command_.errcode));
  } else {
    HexToString(this->GetCurrentExceptionCode(), feedback_command_.errcode);
  }
  command_exec_result_port_.write(feedback_command_);
  if (feedback_command_.status == ExecStatus::SUCCESS) {
    return true;
  } else {
    return false;
  }
}

/**
 * @brief
 *
 * @return true
 * @return false
 */
bool RobotServiceComponent::SetMotionSpeed() {
  if (this->command_.op_type != CommandType::SSPD) {
    return false;
  }
  LOG(INFO) << "reveive set motion speed command.";

  feedback_command_.tp = CommandType::SSPD;
  feedback_command_.status = SUCCESS;
  feedback_command_.timestamp = command_.timestamp;
  // 将执行结果转发到上游组件
  if (feedback_command_.status == ExecStatus::SUCCESS) {
    memset(feedback_command_.errcode, '0', sizeof(feedback_command_.errcode));
  } else {
    HexToString(this->GetCurrentExceptionCode(), feedback_command_.errcode);
  }
  command_exec_result_port_.write(feedback_command_);
  if (feedback_command_.status == ExecStatus::SUCCESS) {
    return true;
  } else {
    return false;
  }
}

/**
 * @brief 设置转移速度等级 SSLV
 *
 * @return true
 * @return false
 */
bool RobotServiceComponent::SetTransferSpeedLevel() {
  if (this->command_.op_type != CommandType::SSLV) {
    return false;
  }
  LOG(INFO) << "SSLV";
  this->rob_model_->SetSpeedLevel(command_.speedLevel);
  LOG(INFO) << "Set speed level: " << command_.speedLevel;
  feedback_command_.tp = CommandType::SSLV;
  feedback_command_.status = ExecStatus::SUCCESS;

  // 将执行结果转发到上游组件
  if (feedback_command_.status == ExecStatus::SUCCESS) {
    memset(feedback_command_.errcode, '0', sizeof(feedback_command_.errcode));
  } else {
    HexToString(this->GetCurrentExceptionCode(), feedback_command_.errcode);
  }
  command_exec_result_port_.write(feedback_command_);
  if (feedback_command_.status == ExecStatus::SUCCESS) {
    return true;
  } else {
    return false;
  }
}
/**
 * @brief 空指令，暂停1s
 *
 * @return true
 * @return false
 */
bool RobotServiceComponent::NopCommand(int milliseconds) {
  if (this->command_.op_type != CommandType::NOP) {
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
  // 将执行结果转发到上游组件
  feedback_command_.timestamp = command_.timestamp;
  memset(feedback_command_.errcode, '0', sizeof(feedback_command_.errcode));
  feedback_command_.status = ExecStatus::SUCCESS;
  feedback_command_.tp = CommandType::NOP;
  command_exec_result_port_.write(feedback_command_);
  if (feedback_command_.status == ExecStatus::SUCCESS) {
    return true;
  } else {
    return false;
  }
}

/********************************************************************
 * @brief
 *
 * @param code
 * @return true
 * @return false
 ********************************************************************/
bool RobotServiceComponent::FaultCommand(StatusCode code) {
  feedback_command_.tp = CommandType::FAULT;
  if (this->command_.op_type != CommandType::FAULT) {
    feedback_command_.tp = this->command_.op_type;
  }
  this->out_to_exception_port_.write(code);
  feedback_command_.timestamp = command_.timestamp;
  feedback_command_.status = ExecStatus::FAILED;

  if (feedback_command_.status == ExecStatus::SUCCESS) {
    memset(feedback_command_.errcode, '0', sizeof(feedback_command_.errcode));
  } else {
    HexToString(this->GetCurrentExceptionCode(), feedback_command_.errcode);
  }
  command_exec_result_port_.write(feedback_command_);
  if (feedback_command_.status == ExecStatus::SUCCESS) {
    return true;
  } else {
    return false;
  }
}

/********************************************************************
 * @brief 从文件中读取点位
 *
 * @param point_name
 * @param position
 ********************************************************************/
void RobotServiceComponent::GetPosition(const std::string &point_name,
                                        double *position) {
  std::shared_ptr<Teach> robteach = std::make_shared<Teach>();
  Point_Position pos = robteach->GetPoint(point_name);
  for (int i = 0; i < kDof; ++i) {
    position[i] = pos.pos.joint(i);
  }
}

ORO_CREATE_COMPONENT(rosc::RobotServiceComponent)
}  // namespace rosc
