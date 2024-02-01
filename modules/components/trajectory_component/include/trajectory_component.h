/**
 * @file trajectory_component.h
 * @author maqun@buaa.edu.cn (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-01-18
 *
 * @copyright Copyright (c) 2022 ROSC
 *
 */
#ifndef MODULES_COMPONENTS_TRAJECTORY_COMPONENT_INCLUDE_TRAJECTORY_COMPONENT_H_
#define MODULES_COMPONENTS_TRAJECTORY_COMPONENT_INCLUDE_TRAJECTORY_COMPONENT_H_
#include <pthread.h>
#include <bits/stdint-uintn.h>
#include <exception>
#include <cmath>
#include <memory>
#include <string>
#include <deque>
#include <rtt/OperationCaller.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/RTT.hpp>
#include <robot_brain/core.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <robot_brain/command_types.hpp>
#include <robot_brain/robot_planning.hpp>
#include "robot_brain/config.h"
#include "robot_brain/core/application.h"
#include <robot_brain/ethercat_frame_types.hpp>
#include "robot_brain/core/robot_model.h"
#include "robot_brain/robot_exception/robot_error.h"

namespace rosc {
using rosc::SingleCommand;

class TrajectoryComponent : public RTT::TaskContext {
 protected:
  RTT::OutputPort<EncoderFrame> out_to_robot_port_;
  RTT::InputPort<EncoderFrame> input_from_robot_port_;
  RTT::InputPort<SingleCommand> in_from_service_port_;
  RTT::OutputPort<StatusCode> out_to_exception_port_;
  RTT::OperationCaller<std::shared_ptr<TrajectoryRobot>()> GetPlanner;

 private:
  std::shared_ptr<RobotModel> rob_model_;
  std::shared_ptr<TrajectoryRobot> trajectory_planner_;

  SingleCommand input_command_;

  AxisNum which_axis_;
  int direction_;
  double speed_per_;
  double jog_step_;
  ExecStatus move_jog_status_;
  double robot_global_speed_;
  bool next_motion_iszone_;
  uint64_t time_stamp_emcy_;
  int stage_num_;
  ArmType arm_type_;

  int dof_;

 public:
  TrajectoryComponent();
  explicit TrajectoryComponent(const std::string &name);
  ~TrajectoryComponent();
  bool configureHook();
  bool startHook();
  void updateHook();
  void stopHook();
  void cleanupHook();
  void exceptionHook(const std::exception &e);

  // bool ParamValidityCheck(const RobotPos &start, const RobotPos &end);

  ExecStatus MoveOperation(const TransferSpeedType &sp, const RobotPos &start,
                           const RobotPos &end, int tp_mode = 0);

  ExecStatus MoveLineAxis(const TransferSpeedType &sp, const RobotPos &start,
                          const RobotPos &end);
  ExecStatus MoveLine(const TransferSpeedType &sp, const RobotPos &start,
                      const RobotPos &end);

  ExecStatus StartMoveJog();
  ExecStatus StartMoveToPoint(const TransferSpeedType &sp,
                              const RobotPos &target_pos);
  void GetPosition(const std::string &point_name, double *position);
  RobotPos GetCurrentPosition(RobotPos *cur_pos);
  bool CheckJointLimit(const RobotPos &pos);
  void EmergencyStop();
  double CompareRobotPos(const RobotPos &pos1, const RobotPos &pos2);
  void memcpyJointArray(KDL::JntArray *joint, const double *pos);
};
};  // namespace rosc

#endif  // MODULES_COMPONENTS_TRAJECTORY_COMPONENT_INCLUDE_TRAJECTORY_COMPONENT_H_
