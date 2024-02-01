/**
 * @file robot_component.cc
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief
 * @version 0.1
 * @date 2021-10-19
 *
 * @copyright Copyright (c) 2021 ROSC
 *
 */
#include <robot_brain/config.h>
#include <bits/stdint-intn.h>
#include <bits/stdint-uintn.h>
#include <yaml-cpp/node/node.h>
#include <robot_brain/core/application.h>
#include <robot_brain/robot_planning.hpp>
#include <robot_brain/core/license.h>
#include <robot_brain/teach_point.h>
#include <robot_component.h>
#include <kdl/utilities/utility.h>
#include <glog/logging.h>
#include <link.h>
#include <math.h>
#include <unistd.h>
#include <bitset>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <thread>  // NOLINT
#include <chrono>  // NOLINT
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <ostream>
#include <robot_brain/robot_exception.hpp>
#include <kdl/frames.hpp>
#include <rtt/ConnPolicy.hpp>
#include <kdl/jntarray.hpp>
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
#include <robot_brain/ethercat_frame_types.hpp>

namespace rosc {
RobotComponent::~RobotComponent() {
  delete[] this->slave_list_;
  LOG(INFO) << "Robot component destructor.";
  LOG(INFO) << "Robot model ptr :" << this->rob_model_.use_count();
}
RobotComponent::RobotComponent() : RobotComponent(std::string("robot")) {
  this->rob_model_ = Application::GetContext()->GetRobotModel();
}

/**
 * @brief Construct a new Robot Component:: Robot Component object
 *
 * @param name
 */
RobotComponent::RobotComponent(const std::string &name)
    : RTT::TaskContext(name, PreOperational),
      out_to_ethercat_port_("ctl_frame_buffer_port"),
      input_from_ethercat_port_("status_frame_port",
                                Orocos::ConnPolicy::data()),
      out_to_service_port_("robot_to_service_port"),
      input_from_traj_port_(
          "encoder_frame_port",
          Orocos::ConnPolicy::buffer(ROBOT_CTL_FRAME_QUEUE_BUFFER_SIZE,
                                     Orocos::ConnPolicy::LOCK_FREE, true)),
      out_to_exception_port_("exception_status_port") {
  // robot_model指针
  this->rob_model_ = rosc::Application::GetContext()->GetRobotModel();
  this->dof_ = rob_model_->GetDof();
  this->trajectory_planner_ = std::make_shared<TrajectoryRobot>(this->dof_);
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
  this->ports()->addPort(input_from_traj_port_);
  this->ports()->addPort(out_to_exception_port_);

  rosc::EthercatCtlFrame ctlSampleFrame;
  out_to_ethercat_port_.setDataSample(ctlSampleFrame);

  EncoderFrame encoderSampleFrame;
  out_to_service_port_.setDataSample(encoderSampleFrame);

  StatusCode exceptionframe;
  out_to_exception_port_.setDataSample(exceptionframe);

  this->moveflag_ = 0;
  this->isEmergencyStop_ = false;

  int arm_tp = config["init_global_config"]["robot_type"].as<int>();
  YAML::Node slave_list;
  switch (arm_tp) {
  case 0:
    this->arm_type_ = ZERO_6;
    this->slave_num_ =
        config["zero_device_config"]["zero_device_config"]["robot"]["slave_num"]
            .as<int>();
    slave_list = config["zero_device_config"]["zero_device_config"]["robot"]
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
  // *获取零点位置编码器值
  std::shared_ptr<Teach> robteach = std::make_shared<Teach>();
  this->pos_zero_ = robteach->GetPoint("ZERO");

  LOG(INFO) << "this->dof_: " << this->dof_;
  LOG(INFO) << "============Robot Component Init Done====================";
}

/**
 * @brief
 *
 * @return true
 * @return false
 */
bool RobotComponent::configureHook() {
  if (!input_from_traj_port_.connected()) {
    LOG(INFO) << "traj_input_port not connected.";
    throw rosc::RobotException(StatusCode::kRobotPortNotConnected_in_traj);
    return false;
  }
  if (!out_to_service_port_.connected()) {
    LOG(INFO) << "out_to_service_port not connected.";
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
  this->addOperation("EmergencyStop", &RobotComponent::EmergencyStop, this)
      .doc("EmergencyStop");
  this->addOperation("EmergencyRecover", &RobotComponent::EmergencyRecover,
                     this)
      .doc("EmergencyRecover");
  this->addOperation("GetPlanner", &RobotComponent::GetTrajectoryPlanner, this)
      .doc("GetPlanner");
  this->addOperation("SavePoint_Operation",
                     &RobotComponent::SavePoint_Operation, this)
      .doc("SavePoint_Operation");
  this->addOperation("ClearError", &RobotComponent::ClearError, this)
      .doc("ClearError");
  this->addOperation("PowerOn", &RobotComponent::PowerOn, this).doc("PowerOn");
  this->addOperation("PowerOff", &RobotComponent::PowerOff, this)
      .doc("PowerOff");
  return true;
}

/**
 * @brief
 *
 * @return true
 * @return false
 */
bool RobotComponent::startHook() {
  rosc::License licenseTool;
  bool isMatch = licenseTool.checkLicense();
  if (!isMatch) {
    LOG(ERROR) << "license mismatch";
    return isMatch;
  }
  next_send_frame_.digit_io_out = 0;
  for (int i = 0; i < this->slave_num_; ++i) {
    if (this->slave_list_[i] != MOTOR_SERVO) {
      continue;
    }
    next_send_frame_.ctrl_word[i] = 0x06;
    next_send_frame_.operation_mode[i] = 8;
  }
  this->out_to_ethercat_port_.write(next_send_frame_);
  return true;
}

/**
 * @brief 异常处理流程
 *
 * @param e
 */
void RobotComponent::exceptionHook(std::exception const &e) {
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

/**
 * @brief robot 组件周期性调用updatehook，周期1ms
 * 1. 先读取ethercat 端口的编码器数据
 * 2. 判断状态字是否出错
 * 3. 将编码器值转换成 pos
 * 4. 处理上游组件发来的指令
 * 5. 记录上游指令的处理结果，发送反馈数据
 */
void RobotComponent::updateHook() {
  bool is_command_pending;
  if (this->command_queue_.empty()) {
    if (this->input_from_traj_port_.read(frame_) == RTT::NewData) {
      is_command_pending = true;
    } else {
      is_command_pending = false;
    }
  } else {
    is_command_pending = true;
    frame_.mode = this->command_queue_.front();
    this->command_queue_.pop();
  }
  bool on_success;                  // 上下电判定是否成功的标志
  EthercatStatusFrame statusFrame;  // 下游ethercat组件的状态帧
  this->input_from_ethercat_port_.read(statusFrame);  // 每一个hook读取一次

  // 计算当前位置
  KDL::JntArray cur_joint(this->dof_);
  this->GetCurrentPosition(statusFrame, &cur_joint);
  for (int i = 0; i < this->dof_; ++i) {
    feedback_frame_.current_position[i] = cur_joint(i);
  }
  // 读取指令下发
  if (is_command_pending) {
    feedback_frame_.time_stamp = frame_.time_stamp;
    // 伺服处于错误状态或急停状态，不接收其他指令
    if (!this->rob_model_->CheckServoErrorState(statusFrame.status_word) ||
        !this->rob_model_->CheckServoEMCState(statusFrame.status_word) ||
        isEmergencyStop_) {
      feedback_frame_.mode = MotionMode::FAILURE_FEEDBACK;
      switch (frame_.mode) {
      case MotionMode::CLEARERROR_SEND:
        // * 在错误状态下只能执行清错指令
        LOG(INFO) << "Operation clear servo error in error state.";
        // this->ServoStop();
        if (this->ClearError()) {  // 如果清错指令执行成功的话
          feedback_frame_.mode = MotionMode::CLEARERROR_FEEDBACK;
        } else {
          feedback_frame_.mode = MotionMode::FAILURE_FEEDBACK;
          this->out_to_exception_port_.write(StatusCode::kClearErrorFaild);
        }
        break;
      case MotionMode::MOVEFINISH_SEND:
        LOG(INFO) << "运动结束。";
        feedback_frame_.mode = MotionMode::FAILURE_FEEDBACK;
        this->moveflag_ = 0;
        break;
      case MotionMode::MOVE_SEND:
        LOG(INFO) << "伺服出现错误，运动结束。";
        feedback_frame_.mode = MotionMode::FAILURE_FEEDBACK;
        this->moveflag_ = 0;
        break;
      case MotionMode::MOVECIRCLE_SEND:
        LOG(INFO) << "伺服出现错误，运动结束。";
        feedback_frame_.mode = MotionMode::FAILURE_FEEDBACK;
        this->moveflag_ = 0;
        break;
      case MotionMode::MOVELINE_SEND:
        LOG(INFO) << "伺服出现错误，运动结束。";
        feedback_frame_.mode = MotionMode::FAILURE_FEEDBACK;
        this->moveflag_ = 0;
        break;
      default:
        LOG(INFO) << "伺服停止，无法执行指令。";
        this->out_to_exception_port_.write(
            StatusCode::kRobotExceptionalCommand);
        break;
      }
    } else {  // *状态字没有错误的情况
      switch (frame_.mode) {
      case MotionMode::CLEARERROR_SEND:  // * 清除错误
        LOG(INFO) << "operation clear error.";
        // this->ServoStop();
        if (this->ClearError()) {
          feedback_frame_.mode = MotionMode::CLEARERROR_FEEDBACK;
        } else {
          feedback_frame_.mode = MotionMode::FAILURE_FEEDBACK;
          this->out_to_exception_port_.write(StatusCode::kClearErrorFaild);
        }
        break;
      case MotionMode::POWERON_SEND:  // *上电
        LOG(INFO) << "operation power on.";
        on_success = this->PowerOn();
        if (on_success) {
          feedback_frame_.mode = MotionMode::POWERON_FEEDBACK;
        } else {
          LOG(WARNING) << "上电失败。";
          feedback_frame_.mode = MotionMode::FAILURE_FEEDBACK;
          this->out_to_exception_port_.write(StatusCode::kSecondaryPowerOn);
        }
        break;
      case MotionMode::POWEROFF_SEND:  // * 下电
        LOG(INFO) << "operation power off.";
        on_success = this->PowerOff();
        if (on_success) {
          feedback_frame_.mode = MotionMode::POWEROFF_FEEDBACK;
        } else {
          LOG(WARNING) << "下电失败";
          feedback_frame_.mode = MotionMode::FAILURE_FEEDBACK;
          this->out_to_exception_port_.write(StatusCode::kSecondaryPowerOff);
        }
        break;
      case MotionMode::MOVE_SEND:    // *运动
        if (this->moveflag_ == 0) {  // 标志位为0表示当前机器人处于空闲状态
          // 从静止开始执行move动作
          this->moveflag_ = 1;  // 标志位为1表示机器人进入运动状态
          this->current_traj_time_ = 0;
          // 开始运动的时候调用三次，向缓冲区内写三组数据
          this->MoveOperation();
          this->MoveOperation();
        }
        this->MoveOperation();
        break;
      case MotionMode::MOVELINE_SEND:
        if (this->moveflag_ == 0) {
          this->moveflag_ = 1;
          this->current_traj_time_ = 0;
          this->MoveLineOperation();
          this->MoveLineOperation();
        }
        this->MoveLineOperation();
        break;
      case MotionMode::JOG_ANGLE:  // jog指令，关节运动
        if (this->moveflag_ == 0) {
          this->moveflag_ = 1;
          this->current_traj_time_ = 0;
          this->MoveJogAngle();
        }
        this->MoveJogAngle();
        break;
      case MotionMode::MOVEFINISH_SEND:  // *运动指令最后一条，确定是否执行完毕
        if (statusFrame.ethercat_frame_send_status ==
            EthercatFrameSendStatus::IDLE) {
          this->moveflag_ = 0;
          feedback_frame_.mode = MotionMode::COMPLETED_FEEDBACK;
        } else {
          this->command_queue_.push(MotionMode::MOVEFINISH_SEND);
        }
        break;
      case MotionMode::SAVEPOINT_SEND:  // *保存点位
        LOG(INFO) << "teach save point.";
        if (this->SavePoint_Operation(frame_.point_name)) {
          feedback_frame_.mode = MotionMode::SAVEPOINT_FEEDBACK;
        } else {
          LOG(WARNING) << "保存点位操作失败";
          feedback_frame_.mode = MotionMode::FAILURE_FEEDBACK;
          this->out_to_exception_port_.write(
              StatusCode::kRobotPointCommandFiled);
        }
        break;
      default:
        // LOG(INFO) << "异常指令类型。" << frame_.mode;
        this->out_to_exception_port_.write(
            StatusCode::kRobotExceptionalCommand);
        break;
      }
    }
    if (!this->rob_model_->CheckServoEMCState(statusFrame.status_word) ||
        this->isEmergencyStop_) {
      feedback_frame_.servo_state = ServoState::EMCY;
    } else if (!this->rob_model_->CheckServoErrorState(
                   statusFrame.status_word)) {
      feedback_frame_.servo_state = ServoState::ERROR;
    } else {
      feedback_frame_.servo_state = ServoState::OP;
    }
  } else {  // 没有接收到指令
    ServoState last_servo_state = feedback_frame_.servo_state;
    if (!this->rob_model_->CheckServoEMCState(statusFrame.status_word) ||
        this->isEmergencyStop_) {
      feedback_frame_.servo_state = ServoState::EMCY;
    } else if (!this->rob_model_->CheckServoErrorState(
                   statusFrame.status_word)) {
      feedback_frame_.servo_state = ServoState::ERROR;
    } else {
      feedback_frame_.servo_state = ServoState::READY;
    }
    if (last_servo_state != feedback_frame_.servo_state) {
      std::cout << last_servo_state << " " << feedback_frame_.servo_state
                << std::endl;
      LOG(INFO) << "servo 状态改变，向上游组件发送数据帧";
      this->out_to_ethercat_port_.write(next_send_frame_);
      // 在空闲时向上游组件发送反馈数据
      // 向trajectory组件写反馈数据
      this->out_to_service_port_.write(feedback_frame_);
    }
  }
  feedback_frame_.time_stamp = frame_.time_stamp;
  feedback_frame_.status = statusFrame.ethercat_frame_send_status;
  feedback_frame_.digit_io_in = statusFrame.digit_io_in;
  feedback_frame_.al_state = statusFrame.al_state;
}  // NOLINT

void RobotComponent::stopHook() {}
void RobotComponent::cleanupHook() {}

std::shared_ptr<TrajectoryRobot> RobotComponent::GetTrajectoryPlanner() {
  return this->trajectory_planner_;
}

/**
 * @brief 机器人上电使能
 *
 * @return true
 * @return false
 */
bool RobotComponent::PowerOn() {
  // 该操作非实时操作
  // 上电逻辑主要是依据CIA402协议
  RTT::TaskContext *eth_task_ptr = getPeer("ethercat");
  if (eth_task_ptr == nullptr) {
    LOG(ERROR) << "getPeer error in power on!";
    return false;
  }
  if (!eth_task_ptr->isActive()) {
    LOG(ERROR) << eth_task_ptr->getName() << " component is not active";
    this->out_to_exception_port_.write(StatusCode::kPowerOnEthercatNotActive);
    return false;
  }
  // 重试间隔时间，因为数据据更新之后，直接查询状态数据可能并没有更新完成，所以，设置一个间隔查询时间
  rosc::Config config = *rosc::Application::GetContext()->GetConfig();
  RTT::Seconds interval_time_sec = RTT::nsecs_to_Seconds(
      config["bus_config"]["communication_cycle"].as<RTT::nsecs>());
  RTT::Seconds max_wait = 3.0;  // 最长等待约为3秒
  LOG(INFO) << "----------------Power On-------------------------";
  std::unique_ptr<EthercatCtlFrame> ctl_frame(new EthercatCtlFrame());
  std::unique_ptr<EthercatStatusFrame> status_frame(new EthercatStatusFrame());
  if (!input_from_ethercat_port_.connected()) {
    LOG(ERROR) << "Robot Component - input_from_ethercat_port_ not connnected!";
    this->out_to_exception_port_.write(
        StatusCode::kPowerOnEthercatPortNotConnect);
    return false;
  }
  RTT::FlowStatus status = input_from_ethercat_port_.read(*status_frame.get());
  while (status == RTT::NoData) {
    usleep(interval_time_sec * 1000000);
    if (!input_from_ethercat_port_.connected()) {
      LOG(ERROR) << "status_frame_port connect error!";
      this->out_to_exception_port_.write(
          StatusCode::kPowerOnEthercatPortNotConnect);
      return false;
    }
    status = input_from_ethercat_port_.read(*status_frame.get());
    // LOG(INFO) << "read status frame: " << *status_frame;
    // LOG(INFO) << "wait capture the status frame data!";
  }

  // 此处，主要是为了将位置对齐到原本的位置，防止速度超限
  RTT::Seconds timer = 0;
  bool reach_flag = true;
  while (true) {
    status = input_from_ethercat_port_.read(*status_frame.get());
    if (timer > max_wait) {
      break;
    }
    // 上电前先把目标位置设置的与当前位置相同，这样避免电机在运动的时候直接过速抱死
    for (int i = 0; i < this->slave_num_; ++i) {
      if (this->slave_list_[i] != SlaveType::MOTOR_SERVO)  // 跳过IO 从站
        continue;
      ctl_frame->target_position[i] = status_frame->current_position[i];
      // LOG(INFO) << "before power, [axios " << i
      //           << "] current positon: " << status_frame->current_position[i]
      //           << ", target position: " << status_frame->target_position[i];
      ctl_frame->operation_mode[i] = 8;
    }
    // LOG(INFO)
    // << "try to modify the target position to match the current location";
    out_to_ethercat_port_.write(*ctl_frame);
    usleep(interval_time_sec * 1000000);
    timer += interval_time_sec;
    // 获取状态并观察是否已经变更
    status = input_from_ethercat_port_.read(*status_frame.get());
    // LOG(INFO) << "robot read status " << status << std::endl
    //           << *status_frame << std::endl;
    reach_flag = true;
    for (int i = 0; i < this->slave_num_; i++) {
      if (this->slave_list_[i] != SlaveType::MOTOR_SERVO)  // 跳过IO 从站
        continue;
      if (status_frame->current_position[i] !=
          status_frame->target_position[i]) {
        reach_flag = false;
        break;
      } else {
        // LOG(INFO) << "[axios " << i
        //           << "] current positon: " <<
        //           status_frame->current_position[i]
        //           << ", target position: " <<
        //           status_frame->target_position[i];
      }
    }
    if (reach_flag) {
      break;
    }
  }
  if (!reach_flag) {
    LOG(ERROR) << "The target_position is not reach the same position with the "
                  "current_position.";
    LOG(ERROR) << "------------------Power on failed.--------------------";
    this->out_to_exception_port_.write(StatusCode::kPowerOnPositionNotSync);
    return false;
  }

  timer = 0;
  std::vector<bool> power_on_status(this->slave_num_, false);
  while (true) {
    status = input_from_ethercat_port_.read(*status_frame.get());
    if (timer > max_wait) {
      LOG(ERROR) << "power off wait for long time (" << max_wait << "s).";
      this->out_to_exception_port_.write(StatusCode::kPowerOnTimeout);
      break;
    }
    for (int seq = 0; seq < this->slave_num_; ++seq) {
      if (this->slave_list_[seq] != SlaveType::MOTOR_SERVO) {
        power_on_status[seq] = true;  // IO从站无上电状态
      }
    }

    // 如果已经全部上电，跳出
    if (!std::any_of(power_on_status.begin(), power_on_status.end(),
                     [](bool elem) { return elem == false; })) {
      break;
    }

    // 根据每个关节的当前状态，构建发送帧
    for (int i = 0; i < this->slave_num_; i++) {
      if (this->slave_list_[i] != SlaveType::MOTOR_SERVO)  // 跳过IO 从站
        continue;
      switch (status_frame->status_word[i] & 0x6f) {
      case 0x0040:
      case 0x0060:
        // 状态 Switch on Disabled
        // 判断switch on disabled位是否为1
        // 计划发送6
        ctl_frame->ctrl_word[i] = 0x0006;
        break;
      case 0x0021:  // 1231
        // 状态 Ready to Switch on
        // [!switch on disabled] + [quick stop] + xx + [!fault] + [!operation
        // enabled] +
        // [!switched on] + [Ready to switch on]
        // 计划发送7
        ctl_frame->ctrl_word[i] = 0x0007;
        break;
      case 0x0023:  // 1233
        // 状态 Switched on
        // [!switch on disabled] + [quick stop] + xx +  [!fault] +
        //     [!operation
        // enabled] + [switched on] + [Ready to switch on]
        ctl_frame->ctrl_word[i] = 0x000f;
        break;
      case 0x0027:
        // 状态 Operation enabled
        // [!switch on disabled] + [quick stop] + xx +  [!fault] + [operation
        // enabled] + [switched on] + [Ready to switch on]
        // 该轴已经上电成功
        if (!power_on_status[i]) {
          power_on_status[i] = true;
          LOG(INFO) << "axios " << i << " power on success!";
        }
        break;
      }
    }
    out_to_ethercat_port_.write(*ctl_frame);
    usleep(interval_time_sec * 1000000);
    timer += interval_time_sec;
  }
  // 判断是否所有的轴都上电成功
  if (!std::any_of(power_on_status.begin(), power_on_status.end(),
                   [](bool elem) { return elem == false; })) {
    LOG(INFO) << "------------------Power on success.--------------------";
    this->next_send_frame_ = *ctl_frame;
    return true;
  } else {
    for (int i = 0; i < this->slave_num_; i++) {
      if (this->slave_list_[i] != SlaveType::MOTOR_SERVO)  // 跳过IO 从站
        continue;
      if (!power_on_status[i]) {
        LOG(ERROR) << "axios " << i << " power on faild!";
      }
      /*如果存在某一个伺服上电未成功的话，将所有的伺服控制字写0x00，
       *使所有伺服处于未上电的安全状态
       */
      ctl_frame->ctrl_word[i] = 0x00;
    }
    out_to_ethercat_port_.write(*ctl_frame);
    this->next_send_frame_ = *ctl_frame;
    LOG(ERROR) << "------------------Power on failed.--------------------";
    this->out_to_exception_port_.write(StatusCode::kPowerOnFailed);
    return false;
  }
}

/**
 * @brief 机器人下电使能
 *
 * @return true
 * @return false
 */
bool RobotComponent::PowerOff() {
  // 该操作非实时操作，下电逻辑主要是依据CIA402协议
  RTT::TaskContext *eth_task_ptr = getPeer("ethercat");
  if (!eth_task_ptr || !eth_task_ptr->isActive()) {
    LOG(ERROR) << "ethercat component is null or not active";
    this->out_to_exception_port_.write(StatusCode::kPowerOffEthercatNotActive);
    return false;
  }
  // 重试间隔时间，因为数据据更新之后，直接查询状态数据可能并没有更新完成，所以，设置一个间隔查询时间
  rosc::Config config = *rosc::Application::GetContext()->GetConfig();
  RTT::Seconds interval_time_sec = RTT::nsecs_to_Seconds(
      config["bus_config"]["communication_cycle"].as<RTT::nsecs>());
  RTT::Seconds max_wait = 3.0;  // 最长等待约为3秒
  LOG(INFO) << "----------------------PowerOff--------------------------";
  std::unique_ptr<EthercatCtlFrame> ctl_frame(new EthercatCtlFrame());
  ctl_frame->digit_io_out = next_send_frame_.digit_io_out;
  std::unique_ptr<EthercatStatusFrame> status_frame(new EthercatStatusFrame);
  RTT::Seconds timer = 0;
  std::vector<bool> power_off_status(this->slave_num_, false);
  std::vector<status_word_t> curr_status(this->slave_num_, 0);
  out_to_ethercat_port_.clear();
  while (true) {
    if (timer > max_wait) {
      LOG(ERROR) << "power off wait for long time (" << max_wait << "s).";
      this->out_to_exception_port_.write(StatusCode::kPowerOffTimeout);
      break;
    }
    input_from_ethercat_port_.read(*status_frame.get());
    // 如果已经全部下电，跳出
    for (int seq = 0; seq < this->slave_num_; ++seq) {
      if (this->slave_list_[seq] != SlaveType::MOTOR_SERVO) {
        power_off_status[seq] = true;  // IO从站无上电状态
      }
    }
    if (!std::any_of(power_off_status.begin(), power_off_status.end(),
                     [](bool elem) { return elem == false; })) {
      break;
    }
    // 防止发送下电指令的时候，位置或者速度超限
    for (int i = 0; i < this->slave_num_; ++i) {
      if (this->slave_list_[i] != SlaveType::MOTOR_SERVO)  // 跳过IO 从站
        continue;
      ctl_frame->target_position[i] = status_frame->current_position[i];
      ctl_frame->operation_mode[i] = 8;
    }
    // 根据每个关节的当前状态，构建发送帧
    for (int i = 0; i < this->slave_num_; i++) {
      if (this->slave_list_[i] != SlaveType::MOTOR_SERVO)  // 跳过IO 从站
        continue;
      switch (status_frame->status_word[i] & 0x6f) {
      //
      case 0x0040:
      case 0x0060:
        // 状态 Switch on Disabled
        // 判断switch on disabled位是否为1
        if (!power_off_status[i]) {
          power_off_status[i] = true;
          LOG(INFO) << "axios " << i << " power off success!";
        }
        break;
      case 0x0021:
        // 状态 Ready to Switch on
        // [!switch on disabled] + [quick stop] + xx + [!fault] + [!operation
        // enabled] +
        // [!switched on] + [Ready to switch on]
        // 计划发送0
        ctl_frame->ctrl_word[i] = 0x0000;
        break;
      case 0x0023:
        // 状态 Switched on
        // [!switch on disabled] + [quick stop] + xx +  [!fault] +
        //     [!operation
        // enabled] + [switched on] + [Ready to switch on]
        // 计划发送6
        ctl_frame->ctrl_word[i] = 0x0006;
        break;
      case 0x0027:
        // 状态 Operation enabled
        // [!switch on disabled] + [quick stop] + xx +  [!fault] + [operation
        // enabled] + [switched on] + [Ready to switch on]
        // 计划发送7
        ctl_frame->ctrl_word[i] = 0x0007;
        break;
      }
    }
    out_to_ethercat_port_.write(*ctl_frame);
    usleep(interval_time_sec * 1000000);
    timer += interval_time_sec;
  }
  // 等待超时跳出
  if (!std::any_of(power_off_status.begin(), power_off_status.end(),
                   [](bool elem) { return elem == false; })) {
    LOG(INFO) << "------------------Power off success.--------------------";
    this->next_send_frame_ = *ctl_frame;
    return true;
  } else {
    for (int i = 0; i < this->slave_num_; i++) {
      if (this->slave_list_[i] != SlaveType::MOTOR_SERVO)  // 跳过IO 从站
        continue;
      if (!power_off_status[i]) {
        LOG(ERROR) << "axios " << i << " power off faild!";
      }
    }
    LOG(ERROR) << "------------------Power off failed.--------------------";
    this->out_to_exception_port_.write(StatusCode::kPowerOffFailed);
    this->next_send_frame_ = *ctl_frame;
    return false;
  }
}

/**
 * @brief 角度运动计算下发
 *
 * @return true
 * @return false
 */
bool RobotComponent::MoveOperation() {
  double next_pos[kDofMax];
  KDL::JntArray q_out(this->dof_);
  if (this->current_traj_time_ <
      this->trajectory_planner_->Duration() + seconds) {
    this->trajectory_planner_->Pos_Axis(current_traj_time_, next_pos);
    for (int i = 0; i < this->dof_; ++i) {
      next_send_frame_.target_position[i] =
          this->pos_zero_.encoder[i] +
          this->rob_model_->Trans2Encoder(next_pos[i], i,
                                          pos_zero_.pos.joint(i));
    }
    out_to_ethercat_port_.write(next_send_frame_);
    this->current_traj_time_ += seconds;
    this->command_queue_.push(MotionMode::MOVE_SEND);
  } else {
    this->move_finish_counter_ = 0;
    this->command_queue_.push(MotionMode::MOVEFINISH_SEND);
  }
  return true;
}

/********************************************************************
 * @brief 关节jog运动下发
 *
 * @return true
 * @return false
 ********************************************************************/
bool RobotComponent::MoveJogAngle() {
  double next_pos[kDofMax];
  KDL::JntArray q_out(this->dof_);
  if (this->current_traj_time_ <
      this->trajectory_planner_->Duration() + seconds) {
    this->trajectory_planner_->Pos_Axis(current_traj_time_, next_pos);
    for (int i = 0; i < this->dof_; ++i) {
      next_send_frame_.target_position[i] =
          this->pos_zero_.encoder[i] +
          this->rob_model_->Trans2Encoder(next_pos[i], i,
                                          pos_zero_.pos.joint(i));
    }
    out_to_ethercat_port_.write(next_send_frame_);
    this->current_traj_time_ += seconds;
    this->command_queue_.push(MotionMode::JOG_ANGLE);
    // LOG(INFO) << "cur time: " << this->current_traj_time_;
  } else {
    this->move_finish_counter_ = 0;
    this->command_queue_.push(MotionMode::MOVEFINISH_SEND);
  }
  return true;
}

/**
 * @brief
 *
 * @return true
 * @return false
 */
bool RobotComponent::MoveLineOperation() {
  KDL::JntArray q_out(this->dof_);
  KDL::JntArray q_init(this->dof_);
  KDL::Frame target_T;
  // 从文件中读取周期
  double duration = this->trajectory_planner_->Duration();
  if (this->current_traj_time_ < duration + seconds) {
    this->trajectory_planner_->Pos_cartesian(this->current_traj_time_,
                                             &target_T);

    this->rob_model_->InverseKinematicsPos(
        this->trajectory_planner_->last_jnt_for_inverse_, target_T, &q_out);

    this->trajectory_planner_->last_jnt_for_inverse_ = q_out;
    for (int i = 0; i < this->dof_; ++i) {
      next_send_frame_.target_position[i] =
          this->pos_zero_.encoder[i] +
          this->rob_model_->Trans2Encoder(q_out(i), i,
                                          this->pos_zero_.pos.joint(i));
    }
    frame_.mode = MotionMode::MOVELINE_SEND;
    this->out_to_ethercat_port_.write(next_send_frame_);
    this->current_traj_time_ += seconds;
    this->command_queue_.push(MotionMode::MOVELINE_SEND);
  } else {
    this->move_finish_counter_ = 0;
    this->command_queue_.push(MotionMode::MOVEFINISH_SEND);
  }
  return true;
}

/********************************************************************
 * @brief 标定手臂的零点
 *
 * @param point_name
 * @param which_arm
 * @return true
 * @return false
 ********************************************************************/
bool RobotComponent::Zero_Calibration(const std::string &point_name,
                                      int which_axis) {
  // 读取当前编码器的值
  EthercatStatusFrame statusFrame;
  input_from_ethercat_port_.read(statusFrame);
  std::shared_ptr<Teach> robteach = std::make_shared<Teach>();
  // *零点标定，按照关节设置逐一标定
  Point_Position zero;
  std::string zero_name;
  zero_name = "ZERO";
  zero = robteach->GetPoint(zero_name);
  KDL::SetToZero(zero.pos.joint);
  memcpy(zero.encoder, statusFrame.current_position, sizeof(zero.encoder));

  robteach->SavePoint(zero_name, zero);

  // *把当前保存的零点位置更新一下
  this->pos_zero_ = zero;
  return true;
}
/**
 * @brief 保存示教点位操作
 *
 */
bool RobotComponent::SavePoint_Operation(const std::string &point_name) {
  if (strstr(point_name.c_str(), "ZERO") != nullptr) {
    // 保存零点
    return this->Zero_Calibration(point_name, 0);
  } else {
    // 读取当前编码器的值
    EthercatStatusFrame statusFrame;
    input_from_ethercat_port_.read(statusFrame);
    std::shared_ptr<Teach> robteach = std::make_shared<Teach>();
    // 计算当前的joint
    KDL::JntArray joint(this->dof_);
    Point_Position cur_pos(statusFrame.current_position);
    this->GetCurrentPosition(statusFrame, &cur_pos.pos.joint);

    // 计算左右臂笛卡尔坐标
    KDL::Frame cart_pos;
    this->rob_model_->ForwordKinematicsPos(joint, &cart_pos, 'L');
    // 保存普通的点位

    cur_pos.pos.cartesian = cart_pos;

    LOG(INFO) << "robot component save point: " << point_name;
    robteach->SavePoint(std::string(point_name), cur_pos);
  }
  return true;
}

/**
 * @brief 清除伺服错误指令
 *
 * @return true 清除错误成功
 * @return false 清除错误失败
 */
bool RobotComponent::ClearError() {
  LOG(INFO) << "------------------Clear Error.--------------------";
  EthercatStatusFrame statusFrame;
  this->input_from_ethercat_port_.read(statusFrame);
  if (this->rob_model_->CheckServoErrorState(statusFrame.status_word) == true) {
    return true;
  }
  // 伺服有错误的时候再下电
  for (int i = 0; i < this->slave_num_; ++i) {
    if (this->slave_list_[i] != SlaveType::MOTOR_SERVO) {
      // 跳过IO 从站
      continue;
    } else if (this->slave_list_[i] == SlaveType::MOTOR_SERVO) {
      next_send_frame_.ctrl_word[i] = 0x86;
    }
  }
  this->out_to_ethercat_port_.write(next_send_frame_);
  LOG(INFO) << "Servo Clear Error.";
  std::this_thread::sleep_for(
      std::chrono::milliseconds(100));  // 0.1秒后检查是否清错成功
  this->input_from_ethercat_port_.read(statusFrame);
  return this->rob_model_->CheckServoErrorState(statusFrame.status_word);
}

/**
 * @brief 向伺服发0，开启刹车
 *
 */
void RobotComponent::ServoStop() {
  LOG(WARNING) << "------------------Servo Stop.--------------------";
  for (int i = 0; i < this->slave_num_; ++i) {
    if (this->slave_list_[i] != SlaveType::MOTOR_SERVO) {
      // 跳过IO 从站
      continue;
    } else if (this->slave_list_[i] == SlaveType::MOTOR_SERVO) {
      next_send_frame_.ctrl_word[i] = 0x00;
    }
  }
  this->out_to_ethercat_port_.write(next_send_frame_);
}

/**
 * @brief 检查伺服的状态字是否处于错误状态
 *
 * @param frame
 * @return true
 * @return false
 */
bool RobotComponent::CheckServoStatusCode(EthercatStatusFrame frame) {
  for (int i = 0; i < this->slave_num_; ++i) {
    if (this->slave_list_[i] != SlaveType::MOTOR_SERVO) {  // IO伺服跳过
      continue;
    }
    status_word_t status_word = frame.status_word[i];
    if (((status_word >> 3) & 0x001) == 0x001) {
      // 出现错误状态，发送异常，伺服状态码错误
      StatusCode code = StatusCode::kServoStateError;
      code = (StatusCode)(code | (uint32_t)i);
      this->out_to_exception_port_.write(code);
      for (int i = 0; i < this->dof_; i++) {
        uint32_t seq = (0x00 | i) << 8;
        uint32_t reverse = !(0xF00);
        if ((frame.error_code[i] & 0xFF) == 0x50) {  // 80.错误
          code = StatusCode::kServoEtherCATSyncError_1;
          code = (StatusCode)((code & reverse) | seq);
          this->out_to_exception_port_.write(code);
        } else if ((frame.error_code[i] & 0xFF) == 0x58) {  // 88.错误
          code = StatusCode::kServoESMError_1;
          code = (StatusCode)((code & reverse) | seq);
          this->out_to_exception_port_.write(code);
        } else if ((frame.error_code[i] & 0xFF) == 0x18) {  // 24.错误
          code = StatusCode::kServoExcessivePositionMode_1;
          code = (StatusCode)((code & reverse) | seq);
          this->out_to_exception_port_.write(code);
        } else if ((frame.error_code[i] & 0xFF) == 0x10) {  // 16.错误
          code = StatusCode::kServoTorqueOverloadError_1;
          code = (StatusCode)((code & reverse) | seq);
          this->out_to_exception_port_.write(code);
        }
      }
      return false;
    }
  }
  return true;
}

/**
 * @brief 根据编码器的值计算当前的位置
 *
 * @param statusFrame
 * @return
 */
void RobotComponent::GetCurrentPosition(EthercatStatusFrame statusFrame,
                                        KDL::JntArray *q_out) {
  // 计算当前的joint
  // Point_Position cur_pos(statusFrame.current_position);
  for (int i = 0; i < this->dof_; ++i) {
    q_out->data[i] = this->rob_model_->Encoder2Position(
        statusFrame.current_position[i], pos_zero_.encoder[i], i);
  }
}

/**
 * @brief 在伺服状态下使电机停止
 *
 * @return true
 * @return false
 */
bool RobotComponent::EmergencyStop() {
  LOG(WARNING)
      << "--------------------Emergency Stop--------------------------";
  this->isEmergencyStop_ = true;
  EthercatStatusFrame statusFrame;
  this->out_to_ethercat_port_.clear();
  // this->input_from_traj_port_.clear();

  this->input_from_ethercat_port_.read(statusFrame);
  int32_t *delta_encoder = new int32_t[this->slave_num_];
  for (int i = 0; i < this->slave_num_; ++i) {
    delta_encoder[i] =
        statusFrame.current_position[i] - statusFrame.last_position[i];
  }

  // 使伺服减速停止
  Config config = *Application::GetContext()->GetConfig();
  int duration =
      config["bus_config"]["emergency_deceleration_duration"].as<int>();
  LOG(INFO) << "Emergency deceleration duration is: " << duration;
  for (int k = 0; k <= duration; k++) {
    for (int i = 0; i < this->slave_num_; ++i) {
      if (this->slave_list_[i] != MOTOR_SERVO) {
        continue;
      }
      next_send_frame_.target_position[i] =
          next_send_frame_.target_position[i] +
          delta_encoder[i] * (duration - k) / duration;
    }
    this->out_to_ethercat_port_.write(next_send_frame_);
  }
  // 先减速到0后，然后发送quick stop控制帧
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  for (int i = 0; i < this->slave_num_; ++i) {
    if (this->slave_list_[i] != MOTOR_SERVO) {
      continue;
    }
    next_send_frame_.ctrl_word[i] = 0x0b;  // 0000 1011
    next_send_frame_.operation_mode[i] = 0x00;
  }
  for (int i = 0; i < 10; i++) {
    this->out_to_ethercat_port_.write(next_send_frame_);
  }
  this->out_to_exception_port_.write(StatusCode::kServoEmergencyState);
  delete[] delta_encoder;
  return true;
}

/**
 * @brief 急停恢复
 *
 * @return true
 * @return false
 */
bool RobotComponent::EmergencyRecover() {
  LOG(WARNING)
      << "--------------------Emergency Recover--------------------------";
  // this->out_to_ethercat_port_.getLastWrittenValue();
  for (int i = 0; i < this->slave_num_; ++i) {
    if (this->slave_list_[i] != MOTOR_SERVO) {
      continue;
    }
    next_send_frame_.ctrl_word[i] = 0x06;
  }
  for (int i = 0; i < 10; i++) {
    this->out_to_ethercat_port_.write(next_send_frame_);
  }
  this->input_from_traj_port_.clear();
  this->isEmergencyStop_ = false;
  return true;
}

ORO_CREATE_COMPONENT(rosc::RobotComponent)
}  // namespace rosc
