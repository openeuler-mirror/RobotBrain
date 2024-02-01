/**
 * @file robot_component.h
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief
 * @version 0.1
 * @date 2021-10-19
 *
 * @copyright Copyright (c) 2021 ROSC
 *
 */
#ifndef MODULES_COMPONENTS_ROBOT_COMPONENT_INCLUDE_ROBOT_COMPONENT_H_
#define MODULES_COMPONENTS_ROBOT_COMPONENT_INCLUDE_ROBOT_COMPONENT_H_
#include <bits/stdint-uintn.h>
#include <fstream>
#include <memory>
#include <queue>
#include <rtt/os/Time.hpp>
#include <string>
#include <rtt/OutputPort.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/RTT.hpp>
#include <robot_brain/core.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <robot_brain/robot_planning.hpp>
#include "robot_brain/config.h"
#include "robot_brain/core/application.h"
#include <robot_brain/ethercat_frame_types.hpp>
#include <robot_brain/robot_exception.hpp>
#include "robot_brain/teach_point.h"

#define ROBOT_CTL_FRAME_QUEUE_BUFFER_SIZE 8192
namespace rosc {
class RobotComponent : public RTT::TaskContext {
 protected:
  RTT::OutputPort<EthercatCtlFrame> out_to_ethercat_port_;
  RTT::InputPort<EthercatStatusFrame> input_from_ethercat_port_;

  RTT::OutputPort<EncoderFrame> out_to_service_port_;
  RTT::InputPort<EncoderFrame> input_from_traj_port_;
  RTT::OutputPort<StatusCode> out_to_exception_port_;
  RTT::Seconds seconds;

 private:
  EncoderFrame frame_, feedback_frame_;
  uint8_t moveflag_;
  EthercatStatusFrame statusframe_;
  EthercatCtlFrame next_send_frame_;
  std::shared_ptr<RobotModel> rob_model_;
  std::shared_ptr<TrajectoryRobot> trajectory_planner_;
  Point_Position pos_zero_;
  bool isEmergencyStop_;
  int move_finish_counter_;

 private:
  std::queue<MotionMode> command_queue_;  // 待执行命令的队列
  double current_traj_time_;
  ArmType arm_type_;
  int dof_;
  int flip_num_;
  int slave_num_;
  SlaveType *slave_list_;

 public:
  RobotComponent();
  explicit RobotComponent(const std::string &name);
  ~RobotComponent();

  bool configureHook();
  bool startHook();
  void updateHook();
  void exceptionHook(std::exception const &e);
  void stopHook();
  void cleanupHook();
  bool PowerOn();
  bool PowerOff();

  bool ClearError();
  void ServoStop();
  bool Zero_Calibration(const std::string &point_name, int which_axis);
  bool SavePoint_Operation(const std::string &point_name);
  bool EmergencyStop();
  bool EmergencyRecover();
  bool MoveOperation();
  bool MoveLineOperation();
  bool MoveJogAngle();

  bool CheckServoStatusCode(EthercatStatusFrame frame);
  void GetCurrentPosition(EthercatStatusFrame statusFrame,
                          KDL::JntArray *q_out);
  std::shared_ptr<TrajectoryRobot> GetTrajectoryPlanner();
};
};      // namespace rosc
#endif  // MODULES_COMPONENTS_ROBOT_COMPONENT_INCLUDE_ROBOT_COMPONENT_H_
