/********************************************************************
 * @file interlock_component.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-10-10
 *
 * @copyright Copyright (c) 2023 ROSC
 *
 ********************************************************************/

#ifndef MODULES_COMPONENTS_INTERLOCK_COMPONENT_INCLUDE_INTERLOCK_COMPONENT_H_
#define MODULES_COMPONENTS_INTERLOCK_COMPONENT_INCLUDE_INTERLOCK_COMPONENT_H_
#include "robot_brain/config.h"
#include "robot_brain/core/application.h"
#include "robot_brain/teach_point.h"
#include "robot_brain/command_types.hpp"
#include <bits/stdint-uintn.h>
#include <fstream>
#include <robot_brain/robot_exception.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <memory>
#include <queue>
#include <robot_brain/core.hpp>
#include <robot_brain/ethercat_frame_types.hpp>
#include <robot_brain/robot_exception.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <rtt/RTT.hpp>
#include <rtt/os/Time.hpp>
#include <string>

#define ROBOT_CTL_FRAME_QUEUE_BUFFER_SIZE 8192
namespace rosc {
class InterlockComponent : public RTT::TaskContext {
 protected:
  RTT::OutputPort<DigitIOFrame> out_to_ethercat_port_;
  RTT::InputPort<EthercatStatusFrame> input_from_ethercat_port_;

  RTT::OutputPort<CommandExecResult> out_to_service_port_;
  RTT::InputPort<SingleCommand> input_from_service_port_;
  RTT::OutputPort<StatusCode> out_to_exception_port_;
  RTT::Seconds seconds;

 private:
  RTT::OperationCaller<void(void)>
      robot_EmergencyRecover;  // Robot组件功能，机器人急停
  RTT::OperationCaller<bool(void)> CallClearError;

  RTT::OperationCaller<bool(void)> CallPowerOn;
  RTT::OperationCaller<bool(void)> CallPowerOff;
  SingleCommand io_command_;
  EthercatStatusFrame statusframe_;
  DigitIOFrame next_send_io_frame_;
  std::shared_ptr<RobotModel> rob_model_;
  Point_Position pos_zero_;
  bool isEmergencyStop_;
  int ready_signal_io_;
  int err_io_seq_;    ///< 伺服发生错误时候io信号
  int user_mode_;     ///< 用户模式|主机模式
  bool host_mode_;    ///< 1 host, 0 teach
  int switch_on_io_;  ///< 三段开关按钮对应的输入io
  int mode_io_;       ///< 机器人运行模式对应的输出io

  ArmType arm_type_;
  int dof_;
  int slave_num_;
  SlaveType *slave_list_;

 public:
  InterlockComponent();
  explicit InterlockComponent(const std::string &name);
  ~InterlockComponent();

  bool configureHook();
  bool startHook();
  void updateHook();
  void exceptionHook(std::exception const &e);
  void stopHook();
  void cleanupHook();

  bool IO_Operation(int whichio, bool op);

  bool CheckServoStatusCode(EthercatStatusFrame frame);
  void GetCurrentPosition(EthercatStatusFrame statusFrame, double *q_out);
};
};     // namespace rosc
#endif  // MODULES_COMPONENTS_INTERLOCK_COMPONENT_INCLUDE_INTERLOCK_COMPONENT_H_
