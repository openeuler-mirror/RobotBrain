/**
 * @file ethercat_component.h
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief
 * @version 0.1
 * @date 2021-09-08
 *
 * @copyright Copyright (c) 2021 ROSC
 *
 */
#ifndef MODULES_COMPONENTS_ETHERCAT_COMPONENT_INCLUDE_ETHERCAT_COMPONENT_H_
#define MODULES_COMPONENTS_ETHERCAT_COMPONENT_INCLUDE_ETHERCAT_COMPONENT_H_

#include <bits/stdint-uintn.h>
#include <ethercat_device.h>
#include <rtt/InputPort.hpp>
#include <sys/types.h>
#include <string>
#include <thread>
#include <iostream>
#include <memory>
#include <rtt/RTT.hpp>
#include <robot_brain/core.hpp>
#include "robot_brain/ethercat_frame_types.hpp"

#define ETHERCAT_CTL_FRAME_QUEUE_BUFFER_SIZE                                   \
  512  // 组件输入端口ctl frame buffer size

#define READY_IO_SEQ 12

namespace rosc {
class EthercatComponent : public RTT::TaskContext {
 private:
  std::unique_ptr<EthercatDevice> p_ec_device_;  ///< EtherCAT设备
  static const int k_master_id_ = 0;             ///< master主站序号
  bool is_simulation_;                           ///< 是否开启模拟
  EthercatStatusFrame simulation_status_;  ///< 模拟开启时候的伺服当前状态情况
  Orocos::nsecs wakeup_ = 0;
  bool reference_slave_clock_;  // 从站时钟同步
  RTT::nsecs period_ns_;

  driver_position_t last_position_[kSLAVENUMMAX];

  int slave_num_;
  SlaveType *slave_list_;

 protected:
  RTT::OutputPort<rosc::EthercatStatusFrame> status_frame_port_;
  RTT::InputPort<rosc::EthercatCtlFrame> ctl_frame_buffer_port_;
  RTT::InputPort<rosc::DigitIOFrame> io_frame_port_;

 public:
  explicit EthercatComponent(const std::string &name);
  EthercatComponent();
  ~EthercatComponent();
  bool configureHook();
  bool startHook();
  void updateHook();
  void stopHook();
  void cleanupHook();
  bool IsSimulation();

 private:
  bool Init();
  void Release();
  void SimulateUpdateStatusFrame(const EthercatCtlFrame &ctl_frame);
  void SimulationUpdateHook();
  static void *IghCyclicTask(void *arg);
};
};  // namespace rosc

#endif  // MODULES_COMPONENTS_ETHERCAT_COMPONENT_INCLUDE_ETHERCAT_COMPONENT_H_
