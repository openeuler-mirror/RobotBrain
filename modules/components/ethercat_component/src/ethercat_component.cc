/**
 * @file ethercat_component.cc
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief 用于测试和演示搭建的框架的orocos组件
 * @version 0.1
 * @date 2021-08-31
 *
 * @copyright Copyright (c) 2021 ROSC
 *
 */
#include <ethercat_component.h>
#include <bits/stdint-uintn.h>
#include <ethercat_device.h>
#include <glog/logging.h>
#include <ios>
#include <rtt/os/main.h>

#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <pthread.h>

#include <rtt/os/Time.hpp>
#include <robot_brain/core.hpp>
#include <rtt/ConnPolicy.hpp>
#include <rtt/FlowStatus.hpp>
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
#include "robot_brain/core/license.h"
#include "robot_brain/ethercat_frame_types.hpp"
namespace rosc {

EthercatComponent::EthercatComponent()
    : EthercatComponent(std::string("Ethercat")) {}

EthercatComponent::~EthercatComponent() {}

EthercatComponent::EthercatComponent(const std::string &name)
    : RTT::TaskContext(name, PreOperational), is_simulation_(false),
      status_frame_port_("status_frame_port"),
      ctl_frame_buffer_port_(
          "ctl_frame_buffer_port",
          Orocos::ConnPolicy::buffer(ETHERCAT_CTL_FRAME_QUEUE_BUFFER_SIZE,
                                     Orocos::ConnPolicy::LOCK_FREE, true)),
      io_frame_port_("io_ctl_frame_port", Orocos::ConnPolicy::data()) {
  // 从文件中读取周期
  rosc::Config config = *rosc::Application::GetContext()->GetConfig();
  // 加载是否是仿真模式
  bool is_simulation =
      config["init_global_config"]["simulation"]["enabled"].as<bool>();
  this->is_simulation_ = is_simulation;
  LOG(INFO) << "EthercatComponent, start simulation: " << is_simulation;

  period_ns_ = config["bus_config"]["communication_cycle"].as<RTT::nsecs>();
  RTT::Seconds seconds = RTT::nsecs_to_Seconds(period_ns_);
  LOG(INFO) << "EthercatComponent, get period：" << this->getPeriod();
  this->reference_slave_clock_ =
      config["bus_config"]["reference_slave_clock"].as<bool>();
  // 如果是从站模式，只能用从站时钟，时钟不再由这个组件的周期控制
  // 如果是模拟，或者主站模式，可以由这个组件的周期控制
  // 设置操作引擎，每一个周期调用一次
  if (is_simulation || (!reference_slave_clock_)) {
    this->setActivity(new Orocos::Activity(ORO_SCHED_RT,
                                           RTT::os::HighestPriority, seconds, 2,
                                           0, "ethercat_activity"));
  }
  this->ports()->addPort(ctl_frame_buffer_port_);
  this->ports()->addPort(status_frame_port_);
  this->ports()->addPort(io_frame_port_);
  this->addOperation("IsSimulation", &EthercatComponent::IsSimulation, this)
      .doc("Whether the simulation status has been turned on.");
  rosc::EthercatStatusFrame statusSampleFrame;
  status_frame_port_.setDataSample(statusSampleFrame);

  int arm_tp = config["init_global_config"]["robot_type"].as<int>();
  YAML::Node slave_list;
  switch (arm_tp) {
  case 0:
    this->slave_num_ =
        config["zero_device_config"]["zero_device_config"]["robot"]["slave_num"]
            .as<int>();
    slave_list = config["zero_device_config"]["zero_device_config"]["robot"]
                       ["slave_type_list"];
    break;
  default:
    this->slave_num_ = 0;
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
  LOG(INFO) << "SlaveNumber: " << this->slave_num_;
}

/**
 * @brief 当前是否在仿真模式
 *
 * @return true 在仿真模式下
 * @return false 不在仿真模式下
 */
bool EthercatComponent::IsSimulation() { return is_simulation_; }

void *EthercatComponent::IghCyclicTask(void *arg) {
  // 注意，当仿真开启的时候，这个线程不会被创建
  // 仿真模式下，没有从站模式和主站模式的区别
  EthercatComponent *component = reinterpret_cast<EthercatComponent *>(arg);
  pthread_setname_np(pthread_self(), "IghCyclicTask");

  LOG(INFO) << "period nescs :" << component->period_ns_;

  component->Init();

  // set first wake time in a few cycles
  component->p_ec_device_->SetWaitUpTime(
      component->p_ec_device_->SystemTimeNs() + 10 * component->period_ns_);

  while (true) {
    EthercatStatusFrame status_frame;
    EthercatCtlFrame ctl_frame;
    DigitIOFrame io_frame;
    component->p_ec_device_->WaitPeriod();

    component->p_ec_device_->ReadEthercatStatusFrame(
        &status_frame);  // 从igh读取伺服状态

    // 保存编码器上一个周期状态，用于计算速度
    // FIXME: ReadEthercatStatusFrame方法读到了速度，为什么对速度进行了覆盖
    for (int i = 0; i < component->slave_num_; ++i) {
      status_frame.last_position[i] = component->last_position_[i];
      component->last_position_[i] = status_frame.current_position[i];
    }
    // 从端口处读取伺服控制帧
    RTT::FlowStatus status = component->ctl_frame_buffer_port_.read(ctl_frame);
    component->io_frame_port_.read(io_frame);
    ctl_frame.digit_io_out = io_frame.digit_io_out;

    if (status == RTT::NewData) {
      status_frame.ethercat_frame_send_status = EthercatFrameSendStatus::BUSY;
      status_frame.digit_io_out = ctl_frame.digit_io_out;
      component->status_frame_port_.write(status_frame);
      component->p_ec_device_->WriteEthercatCtlFrame(ctl_frame);
    } else if (status == RTT::OldData) {
      status_frame.ethercat_frame_send_status = EthercatFrameSendStatus::IDLE;
      status_frame.digit_io_out = ctl_frame.digit_io_out;
      component->status_frame_port_.write(status_frame);
      component->p_ec_device_->WriteEthercatCtlFrame(ctl_frame);
    } else {  // RTT::NoData
      status_frame.ethercat_frame_send_status = EthercatFrameSendStatus::IDLE;
      component->status_frame_port_.write(status_frame);
      component->p_ec_device_->WriteEthercatCtlFrame();
    }
  }
}

/**
 * @brief Ethercat初始化
 *
 * @return true
 * @return false
 */
bool EthercatComponent::Init() {
  // 初始化主站，从站以及相关配置
  LOG(INFO) << "Start Master initialization.";
  // TODO(maqun): 返回错误false的情况，需要处理异常和错误码的情况
  if (!this->p_ec_device_->CreateMaster(k_master_id_)) {
    LOG(ERROR) << "Create Master Error";
    return false;
  }
  if (!this->p_ec_device_->AddDomain()) {
    LOG(ERROR) << "Add Domain Error";
    return false;
  }
  if (!this->p_ec_device_->ConfigSlaves()) {
    LOG(ERROR) << "Config Slaves Error";
    return false;
  }

  if (this->reference_slave_clock_) {
    if (!this->p_ec_device_->SelectReferenceClock()) {
      LOG(ERROR) << "Select Reference Clock Error";
      return false;
    }
  }

  if (!this->p_ec_device_->DomainRegisterList()) {
    LOG(ERROR) << "Domain Register List Errror";
    return false;
  }

  if (!this->p_ec_device_->MasterActive()) {
    LOG(ERROR) << "Master Active Errror";
    return false;
  }
  if (!this->p_ec_device_->DomainData()) {
    LOG(ERROR) << "Domain Data Error!";
    return false;
  }
  return true;
}

/**
 * @brief 释放ethercat设备
 *
 */
void EthercatComponent::Release() { this->p_ec_device_->MasterDeactive(); }

bool EthercatComponent::configureHook() {
  p_ec_device_ =
      std::unique_ptr<EthercatDevice>(new EthercatDevice(period_ns_));
  if (!this->ctl_frame_buffer_port_.connected()) {
    LOG(INFO) << "ethercat port connect error.";
    return false;
  } else {
    LOG(INFO) << "ctl_frame_buffer_port connected.";
  }
  return true;
}

/**
 * @brief 组件启动的回调函数
 *
 * @return true
 * @return false
 */
bool EthercatComponent::startHook() {
  rosc::License licenseTool;
  bool isMatch = licenseTool.checkLicense();
  if (!isMatch) {
    LOG(ERROR) << "license mismatch";
    return isMatch;
  }

  // ethercat总线的初始化
  if (this->is_simulation_) {
    return true;
  }
  // ethercat总线的初始化
  if (!this->reference_slave_clock_) {
    return Init();
  }

  /* Create cyclic RT-thread */
  struct sched_param param;
  pthread_attr_t attr;
  pthread_t thread;
  int ret;
  ret = pthread_attr_init(&attr);
  if (ret) {
    LOG(ERROR) << "init pthread attributes failed";
    goto out;
  }

  // /* Use scheduling parameters of attr */
  // ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
  // if (ret) {
  //   LOG(ERROR) << "pthread setinheritsched failed";
  //   goto out;
  // }

  /* Set scheduler policy and priority of pthread */
  ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
  if (ret) {
    LOG(ERROR) << "pthread setschedpolicy failed";
    goto out;
  }
  param.sched_priority = sched_get_priority_max(SCHED_FIFO);
  ret = pthread_attr_setschedparam(&attr, &param);
  if (ret) {
    LOG(ERROR) << "pthread setschedparam failed";
    goto out;
  }

  /* Use scheduling parameters of attr */
  ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
  if (ret) {
    LOG(ERROR) << "pthread setinheritsched failed";
    goto out;
  }

  // 设置内核亲和性，绑定到第二个内核上
  cpu_set_t cpu_info;
  CPU_ZERO(&cpu_info);
  CPU_SET(1, &cpu_info);
  if (0 != pthread_attr_setaffinity_np(&attr, sizeof(cpu_set_t), &cpu_info)) {
    LOG(ERROR) << "set affinity failed";
    return -1;
  }

  /* Create a pthread with specified attributes */
  usleep(1000000 / 125);
  ret = pthread_create(&thread, &attr, IghCyclicTask, this);
  if (ret) {
    LOG(ERROR) << "create pthread failed: " << ret << "; "
               << (ret == -1 ? strerror(errno) : strerror(ret));
    goto out;
  }

  pthread_detach(thread);
  return true;
out:
  LOG(ERROR) << "End of Program";
  this->p_ec_device_->MasterDeactive();
  return false;
}

/**
 * @brief 在仿真模式中，根据ctlFrame的值更新simulation_status_
 *
 * @param ctl_frame
 */
void EthercatComponent::SimulateUpdateStatusFrame(
    const EthercatCtlFrame &ctl_frame) {
  // 此处逻辑用于处理，当ctl指令发送power on何power off指令时的对应响应内容。
  // 保障在仿真状态下，robot相关的组件可以正常的power on和power off
  for (int i = 0; i < this->slave_num_; i++) {
    switch (ctl_frame.ctrl_word[i]) {
    case 0x0006:
      // power on status: value & 0x006f = 0x0021
      // power off status: value & 0x006f = 0x0021
      simulation_status_.status_word[i] = 0x2221;
      break;
    case 0x0007:
      // power on status: value & 0x006f = 0x0023
      // power off status: value & 0x006f = 0x0023
      simulation_status_.status_word[i] = 0x2223;
      break;
    case 0x000f:
      // value & 0x006f = 0x0027
      simulation_status_.status_word[i] = 0x2227;
      break;
    case 0x0000:
      // power on status: value & 0x006f = 0x0060
      // power off status: value & 0x006f = 0x0060
      simulation_status_.status_word[i] = 0x2260;
      break;
    }
  }
  memcpy(simulation_status_.operation_mode, ctl_frame.operation_mode,
         sizeof(operation_mode_t) * this->slave_num_);
  memcpy(simulation_status_.ctrl_word, ctl_frame.ctrl_word,
         sizeof(ctrl_word_t) * this->slave_num_);
  memcpy(simulation_status_.target_position, ctl_frame.target_position,
         sizeof(driver_position_t) * this->slave_num_);
  memcpy(simulation_status_.current_position, ctl_frame.target_position,
         sizeof(driver_position_t) * this->slave_num_);
  memset(simulation_status_.modes_operation_display, 0,
         sizeof(modes_operation_display_t) * this->slave_num_);
  simulation_status_.digit_io_out = ctl_frame.digit_io_out;
  simulation_status_.ethercat_frame_send_status = EthercatFrameSendStatus::BUSY;
}

/**
 * @brief 在开启仿真状态下的循环操作
 *
 */
void EthercatComponent::SimulationUpdateHook() {
  EthercatStatusFrame status_frame;
  EthercatCtlFrame ctl_frame;
  DigitIOFrame io_frame;
  status_frame = simulation_status_;

  for (int i = 0; i < this->slave_num_; ++i) {
    status_frame.last_position[i] = this->last_position_[i];
    this->last_position_[i] = status_frame.current_position[i];
  }

  RTT::FlowStatus status = ctl_frame_buffer_port_.read(ctl_frame);
  io_frame_port_.read(io_frame);
  ctl_frame.digit_io_out = io_frame.digit_io_out;

  if (status == RTT::NewData) {
    status_frame.ethercat_frame_send_status = EthercatFrameSendStatus::BUSY;
    status_frame.digit_io_out = ctl_frame.digit_io_out;
    status_frame.data_flag = 1;
    status_frame_port_.write(status_frame);
    SimulateUpdateStatusFrame(ctl_frame);
  } else if (status == RTT::OldData) {
    status_frame.ethercat_frame_send_status = EthercatFrameSendStatus::IDLE;
    status_frame.digit_io_out = ctl_frame.digit_io_out;
    status_frame.data_flag = 0;
    status_frame_port_.write(status_frame);
    SimulateUpdateStatusFrame(ctl_frame);
  } else {  // RTT::NoData
    status_frame.ethercat_frame_send_status = EthercatFrameSendStatus::IDLE;
    status_frame_port_.write(status_frame);
  }
}

void EthercatComponent::updateHook() {
  if (reference_slave_clock_) {
    return;
  }
  // 读取当前状态数据
  if (is_simulation_) {
    this->SimulationUpdateHook();
  } else {
    // 获取系统的纳秒，并同步
    EthercatStatusFrame status_frame;
    EthercatCtlFrame ctl_frame;
    DigitIOFrame io_frame;
    if (wakeup_ == 0) {
      wakeup_ = RTT::os::TimeService::Instance()->getNSecs() +
                Orocos::Seconds_to_nsecs(this->getActivity()->getPeriod());
    } else {
      wakeup_ =
          wakeup_ + Orocos::Seconds_to_nsecs(this->getActivity()->getPeriod());
    }
    p_ec_device_->MasterApplicationTime(wakeup_);
    p_ec_device_->ReadEthercatStatusFrame(&status_frame);  // 从igh读取伺服状态
    // 保存编码器上一个周期状态，用于计算速度
    // FIXME: ReadEthercatStatusFrame方法读到了速度，为什么对速度进行了覆盖
    for (int i = 0; i < this->slave_num_; ++i) {
      status_frame.last_position[i] = this->last_position_[i];
      this->last_position_[i] = status_frame.current_position[i];
    }
    // 从端口处读取伺服控制帧
    RTT::FlowStatus status = ctl_frame_buffer_port_.read(ctl_frame);
    io_frame_port_.read(io_frame);  // IO字段来自interlock的端口
    ctl_frame.digit_io_out = io_frame.digit_io_out;

    if (status == RTT::NewData) {
      status_frame.ethercat_frame_send_status = EthercatFrameSendStatus::BUSY;
      status_frame.digit_io_out = ctl_frame.digit_io_out;
      status_frame_port_.write(status_frame);
      p_ec_device_->WriteEthercatCtlFrame(ctl_frame);
    } else if (status == RTT::OldData) {
      status_frame.ethercat_frame_send_status = EthercatFrameSendStatus::IDLE;
      status_frame.digit_io_out = ctl_frame.digit_io_out;
      status_frame_port_.write(status_frame);
      p_ec_device_->WriteEthercatCtlFrame(ctl_frame);
    } else {  // RTT::NoData
      status_frame.ethercat_frame_send_status = EthercatFrameSendStatus::IDLE;
      status_frame_port_.write(status_frame);
      p_ec_device_->WriteEthercatCtlFrame();
    }
  }
}

void EthercatComponent::stopHook() {
  if (!this->is_simulation_) {
    this->Release();
  }
}

void EthercatComponent::cleanupHook() {}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(HelloWorld)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(rosc::EthercatComponent)
}  // namespace rosc
