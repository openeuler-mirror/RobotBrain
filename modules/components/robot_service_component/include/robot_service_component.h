/**
 * @file robot_service_component.h
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief
 * @version 0.1
 * @date 2022-01-09
 *
 * @copyright Copyright (c) 2022 ROSC
 *
 */
#ifndef MODULES_COMPONENTS_ROBOT_SERVICE_COMPONENT_INCLUDE_ROBOT_SERVICE_COMPONENT_H_
#define MODULES_COMPONENTS_ROBOT_SERVICE_COMPONENT_INCLUDE_ROBOT_SERVICE_COMPONENT_H_
#include <bits/stdint-uintn.h>
#include <deque>
#include <string>
#include <memory>
#include <map>
#include <vector>
#include <rtt/InputPort.hpp>
#include <robot_brain/command_types.hpp>
#include <robot_brain/ethercat_frame_types.hpp>
#include <rtt/OutputPort.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <kdl/frames.hpp>
#include "robot_brain/config.h"
#include "robot_brain/robot_exception/robot_status.h"
#include "robot_brain/teach_point.h"
#include <robot_brain/robot_exception.hpp>
#include <robot_brain/robot_planning.hpp>
// Configure optional HFSM2 functionality using #defines
// (in this case we're using Plans to make transition cycle more
// straightforward):
#define HFSM2_ENABLE_PLANS

// Include HFSM2
#include <hfsm2/machine.hpp>

namespace rosc {

typedef enum ControllerState {
  ON,
  OFF,
  Error,
  Init,
  Op,
  Idle,
  EMC_STOP
} ControllerState;
typedef struct RobotCurrentState {
  double joint[kDofMax];
  uint32_t digit_io_out;
  uint32_t digit_io_in;
  ControllerState state;
  double torque[kDofMax];
  double velocity[kDofMax];
  uint16_t error_code[kDofMax];
  uint16_t current_speed_level;
  double tachometer[kDofMax];  // 电机转速
  KDL::Vector A;
  KDL::Vector B;
  KDL::Vector C;
  KDL::Vector D;
  KDL::Frame cartesian;
} RobotCurrentState;

typedef struct coordinate {
  double x;
  double y;
  double angle;
  std::string type;
} coordinate;

// 声明
class RobotServiceComponent;

namespace event {

// ----------------------事件------------------------
struct Event {};

/**
 * @brief 事件声明
 *
 */
struct RobotEvent : Event {};

/**
 * @brief 初始化事件，进入初始状态
 */
struct InitEvent : RobotEvent {};

/**
 * @brief 打开上电开关
 */
struct TurnPowerOnEvent : RobotEvent {};

/**
 * @brief 关闭上电开关
 */
struct TurnPowerOffEvent : RobotEvent {};

/**
 * @brief 两点之间移动
 *
 */
struct MoveBetweenPointsEvent : RobotEvent {};

struct MoveToPointEvent : RobotEvent {};

struct MoveToCartesianPointEvent : RobotEvent {};

struct MoveLineEvent : RobotEvent {};

struct MoveStepEvent : RobotEvent {};

struct MoveSuccessEvent : RobotEvent {};

struct MoveFailedEvent : RobotEvent {};

/**
 * @brief 开始移动指令
 */
struct StartMoveEvent : RobotEvent {};
struct StartMovePointEvent : RobotEvent {};

/**
 * @brief 结束移动动作
 */
struct DoneMoveEvent : RobotEvent {};

/**
 * @brief 机器人报错
 */
struct RobotFaultEvent : RobotEvent {};

/**
 * @brief 机器人清错
 */
struct RobotClearEvent : RobotEvent {};

struct PointEvent : RobotEvent {};

}  // namespace event

/**
 * @brief 定义一个接口类（用于状态机和host之间的数据交互）
 * Define interface class between the state machine and its host
 * (also ok to use the host object itself):
 */
struct RobotServiceContext {
  bool ready;                                  ///< 是否准备好
  rosc::RobotServiceComponent *robot_service;  ///< robotService
};

// (Optional) Define type config:
using RobotServiceConfig = hfsm2::Config::ContextT<RobotServiceContext &>;

// (Optional, recommended) Definehfsm2::Machine for convenience:
using M = hfsm2::MachineT<RobotServiceConfig>;

// ----------------------状态-----------------------
namespace state {
//------------------------------------------------------------------------------
// Declare state machine structure.
// States need to be forward declared, e.g. with a magic macro:
#define S(s) struct s

// ---------------------状态机------------------------
using FSM = M::PeerRoot<S(OffState), S(InitialState), S(MoveState),
                        S(IdleState), S(FaultState)>;
#undef S

struct BaseState : FSM::State {
  template <typename Event>
  void react(const Event &, FullControl &control) noexcept;  // NOLINT
};

// Define states and override required state methods:
struct OffState : BaseState {
  void entryGuard(FullControl &control) noexcept;  // NOLINT;
  void react(const event::InitEvent &,
             FullControl &control) noexcept;  // NOLINT
  // 忽略其他的
  using BaseState::react;
};

struct InitialState : BaseState {
  void react(const event::TurnPowerOnEvent &,
             FullControl &control) noexcept;  // NOLINT

  void react(const event::MoveBetweenPointsEvent &,
             FullControl &control) noexcept;  // NOLINT
  void react(const event::StartMoveEvent &,
             FullControl &control) noexcept;  // NOLINT
  void react(const event::DoneMoveEvent &,
             FullControl &control) noexcept;  // NOLINT
  void react(const event::TurnPowerOffEvent &,
             FullControl &control) noexcept;  // NOLINT
  void react(const event::RobotClearEvent &,
             FullControl &control) noexcept;  // NOLINT
  // 忽略其他的
  using BaseState::react;
};

struct IdleState : BaseState {
  void react(const event::StartMoveEvent &,
             FullControl &control) noexcept;  // NOLINT
  void react(const event::TurnPowerOnEvent &,
             FullControl &control) noexcept;  // NOLINT
  void react(const event::TurnPowerOffEvent &,
             FullControl &control) noexcept;  // NOLINT
  void react(const event::RobotFaultEvent &,
             FullControl &control) noexcept;  // NOLINT
  void react(const event::PointEvent &,
             FullControl &control) noexcept;  // NOLINT
  void react(const event::MoveBetweenPointsEvent &,
             FullControl &control) noexcept;  // NOLINT
  void react(const event::MoveToPointEvent &,
             FullControl &control) noexcept;  // NOLINT
  void react(const event::MoveToCartesianPointEvent &,
             FullControl &control) noexcept;  // NOLINT
  void react(const event::StartMovePointEvent &,
             FullControl &control) noexcept;  // NOLINT
  void react(const event::MoveLineEvent &,
             FullControl &control) noexcept;  // NOLINT
  void react(const event::RobotClearEvent &,
             FullControl &control) noexcept;  // NOLINT
  void react(const event::MoveStepEvent &,
             FullControl &control) noexcept;  // NOLINT

  using BaseState::react;
};

struct MoveState : BaseState {
  // void enter(PlanControl &control) noexcept;  // NOLINT.

  // void planSucceeded(FullControl &control) noexcept;  // NOLINT.

  // void planFailed(FullControl &control) noexcept;  // NOLINT.

  void react(const event::DoneMoveEvent &,
             FullControl &control) noexcept;  // NOLINT
  void react(const event::RobotClearEvent &,
             FullControl &control) noexcept;  // NOLINT
  void react(const event::StartMoveEvent &,
             FullControl &control) noexcept;  // NOLINT
  void react(const event::StartMovePointEvent &,
             FullControl &control) noexcept;  // NOLINT
  void react(const event::MoveSuccessEvent &,
             FullControl &control) noexcept;  // NOLINT
  void react(const event::MoveFailedEvent &,
             FullControl &control) noexcept;  // NOLINT

  using BaseState::react;
};

struct MoveReadyState : BaseState {
  void update(FullControl &control) noexcept;  // NOLINT
};

struct MoveLineState : BaseState {
  void update(FullControl &control) noexcept;  // NOLINT
};

struct FaultState : BaseState {
  void react(const event::RobotClearEvent &,
             FullControl &control) noexcept;  // NOLINT
  using BaseState::react;
};

};  // namespace state

typedef struct CommandTrack {
  std::deque<SingleCommand> commandlist;
  uint size;
  CommandTrack(int s) {
    size = s;
    commandlist = std::deque<SingleCommand>(size);
  }
  void push(SingleCommand command) {
    // 超出队列size，丢掉尾部的指令
    if (commandlist.size() == size) {
      commandlist.pop_back();
    }
    // 向队列头部插入
    commandlist.push_front(command);
  }
  SingleCommand pop() {
    SingleCommand command = commandlist.front();
    commandlist.pop_front();
    return command;
  }
  SingleCommand top() { return commandlist.front(); }
} CommandTrack;

class RobotServiceComponent : public RTT::TaskContext {
 private:
  state::FSM::Instance state_machine_;         ///< 机器人状态机
  RobotServiceContext state_machine_context_;  ///< 机器人状态机上下文
  RTT::InputPort<rosc::SingleCommand> command_input_port_;  ///< 接受的指令序列
  RTT::OutputPort<rosc::CommandExecResult>
      command_exec_result_port_;  ///< 对外表达指令的接受及完成结果

  RTT::OutputPort<rosc::SingleCommand> out_to_traj_port_;  ///< 下发到traj组件

  RTT::InputPort<rosc::EncoderFrame>
      in_from_robot_port_;  ///< 接受来自robot组件的反馈

  RTT::InputPort<EthercatStatusFrame>
      in_from_ethercat_port_;  ///< 接收来自ethercat模块的数据帧

  RTT::OutputPort<SingleCommand> out_to_interlock_port_;

  RTT::OutputPort<StatusCode> out_to_exception_port_;  ///< 与exception组件通信
  RTT::InputPort<StatusCode> in_from_exception_port_;  ///< exception输入

  RTT::OperationCaller<void(void)>
      robot_EmergencyStop;  // Robot组件功能，机器人急停

  RTT::OperationCaller<void(void)>
      robot_EmergencyRecover;  // Robot组件功能，机器人急停

  RTT::OperationCaller<void(void)>
      traj_EmergencyStop;  // Trajectory组件功能，机器人急停信号

  RTT::OperationCaller<StatusCode(void)> GetCurrentExceptionCode;

  RTT::OperationCaller<bool(int, bool)> IOOperation;

  RTT::OperationCaller<bool(const std::string)> SavePoint_Operation;

  RTT::OperationCaller<bool(void)> CallClearError;

  RTT::OperationCaller<bool(void)> CallPowerOn;

  RTT::OperationCaller<bool(void)> CallPowerOff;

 private:
  std::shared_ptr<RobotModel> rob_model_;
  SingleCommand command_;
  CommandExecResult feedback_command_;
  Point_Position pos_zero_;
  double io_in_check_timeout_;
  bool is_retry_mode_;
  std::shared_ptr<CommandTrack> command_track_;
  double mem_pos_[kDof];  // 在下电时记录关节位置

 private:
  int dof_;

 public:
  ControllerState mechine_state_;

 public:
  RobotServiceComponent();
  explicit RobotServiceComponent(const std::string &name);
  ~RobotServiceComponent();
  bool configureHook();
  bool startHook();
  void updateHook();
  void stopHook();
  void cleanupHook();
  // 相关功能接口
  bool PowerOn();
  bool PowerOff();
  bool Move();
  bool MoveTo();
  bool MoveToCart();
  bool MoveLine();
  bool MoveStep();
  int ClearError();
  bool IO_Operation();
  bool IO_In_Operation();
  bool MoveJogStart();
  bool PointJogStart();
  bool MoveJogStop();
  bool SavePoint();
  void GetCurrentPosition(RobotCurrentState *const pos);
  void Emergency_stop();
  void Emergency_recover();
  void PortWriteError();
  void SkipCommand();

  bool SetSpeedLevel();
  bool SetMotionSpeed();
  bool SetTransferSpeedLevel();

  bool NopCommand(int milliseconds);
  bool FaultCommand(StatusCode code = StatusCode::kInvalidParaNXC);
  void GetPosition(const std::string &point_name, double *position);
  void GetJoint(double *position);
  void IOSend(int which_io, bool op);
  bool ParamValidityCheck(const RobotPos &start, const RobotPos &end);
  bool CheckJointLimit(const RobotPos &pos);
  double ComparePosition(const RobotPos &pos1, const RobotPos &pos2);
};
};  // namespace rosc

#endif  // MODULES_COMPONENTS_ROBOT_SERVICE_COMPONENT_INCLUDE_ROBOT_SERVICE_COMPONENT_H_
