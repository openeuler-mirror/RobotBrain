/**
 * @file grpc_service_component.cc
 * @author DouYuanbo (douyuanbo@buaa.edu.cn)
 * @brief
 * @version 0.1
 * @date 2022-01-07
 *
 * @copyright Copyright (c) 2022 ROSC
 *
 */
#include "proto/robot_grpc_service.pb.h"
#include "robot_brain/config.h"
#include "robot_brain/core/application.h"
#include "robot_brain/core/robot_model.h"
#include "robot_brain/ethercat_frame_types.hpp"
#include "robot_brain/command_types.hpp"
#include "robot_service_component.h"
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <iostream>
#include <chrono>
#include <future>
#include <grpc_service_component.h>
#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpcpp/impl/codegen/status.h>
#include <rtt/FlowStatus.hpp>
#include <thread>
#include <unistd.h>
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/yaml.h>
#include <robot_brain/core.hpp>
#include <string>
#include <fstream>
#include <map>
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/ConnPolicy.hpp>
#include <boost/filesystem.hpp>

namespace rosc {

GrpcServiceComponent::GrpcServiceComponent()
    : GrpcServiceComponent(std::string("grpc_service")) {}

GrpcServiceComponent::GrpcServiceComponent(const std::string &name)
    : RTT::TaskContext(name, PreOperational),
      out_to_service_port_("command_input_port"),
      in_from_service_port_("command_exec_result_port",
                            Orocos::ConnPolicy::buffer(64)),
      in_from_exception_port_("exception_output_port",
                              Orocos::ConnPolicy::buffer(64)),
      jogspeed_(0.3), start_get_stream_joints_(false), start_io_input_(false) {
  this->setActivity(new Orocos::Activity(ORO_SCHED_OTHER, 0, 0, 4, 0,
                                         "grpc_service_activity"));
  this->ports()->addPort(out_to_service_port_);
  this->ports()->addPort(in_from_service_port_);
  this->ports()->addPort(in_from_exception_port_);
  auto config = *Application::GetContext()->GetConfig();
  int arm_tp = config["init_global_config"]["robot_type"].as<int>();

  enable_jog_.store(false, std::memory_order_relaxed);

  switch (arm_tp) {
  case 0:
    this->arm_type_ = ZERO_6;
    this->dof_ =
        config["zero_device_config"]["zero_device_config"]["robot"]["dof"]
            .as<int>();
    break;
  default:
    this->arm_type_ = XB4S;
    break;
  }
  LOG(INFO) << "grpc sevice init.";
}

GrpcServiceComponent::~GrpcServiceComponent() {
  if (server_) {
    server_->Shutdown();
    LOG(INFO) << "GRPC Server shutdown: " << service_addr_;
    server_.reset();
    service_addr_ = "";
  }
  LOG(INFO) << "GrpcServiceComponent deconstructed.";
}

bool GrpcServiceComponent::configureHook() {
  // 配置端口和operation
  if (!in_from_service_port_.connected()) {
    LOG(ERROR) << "GRPCServiceComponent in_from_service_port_ not connected !";
    return false;
  }
  if (!out_to_service_port_.connected()) {
    LOG(ERROR) << "GRPCServiceComponent out_to_service_port_ not connected!";
    return false;
  }
  TaskContext *robot_service = getPeer("robot_service");
  GetCurrentPosition = robot_service->getOperation("GetCurrentPosition");
  ServiceEmergencyStop = robot_service->getOperation("EmergencyStop");
  ServiceEmergencyRecover = robot_service->getOperation("EmergencyRecover");

  TaskContext *exception_handle = getPeer("exception_handle");
  WriteToLog = exception_handle->getOperation("WriteToLog");
  ReadExceptionLog = exception_handle->getOperation("ReadExceptionLog");
  ClearExceptionLog = exception_handle->getOperation("ClearExceptionLog");

  // 配置GRPC
  const rosc::Config config = *rosc::Application::GetContext()->GetConfig();
  if (config["init_global_config"]["rpc_service"]["enabled"]) {
    grpc_enabled_ =
        config["init_global_config"]["rpc_service"]["enabled"].as<bool>();
    if (!grpc_enabled_) {
      LOG(WARNING) << "GRPC Service is disabled in the config file.";
      return false;
    }
  } else {
    LOG(ERROR)
        << "Config file: init_global_config.rpc_service.enabled undefined!";
    return false;
  }
  unsigned int port =
      config["init_global_config"]["rpc_service"]["port"].as<unsigned int>();
  char addr[30];
  snprintf(addr, sizeof(addr), "0.0.0.0:%u", port);
  service_addr_ = std::string(addr);

  return true;
}

bool GrpcServiceComponent::startHook() {
  // new thread for grpc
  std::thread t([this]() {
    // set name
    pthread_setname_np(pthread_self(), "grpc_service");
#ifndef NDEBUG
    grpc::EnableDefaultHealthCheckService(true);
    grpc::reflection::InitProtoReflectionServerBuilderPlugin();
#endif
    ServerBuilder builder;
    builder.AddListeningPort(service_addr_, grpc::InsecureServerCredentials());
    builder.RegisterService(this);
    grpc::ServerBuilder::SyncServerOption min_pollers_option =
        grpc::ServerBuilder::SyncServerOption::MIN_POLLERS;
    builder.SetSyncServerOption(min_pollers_option, 1);
    grpc::ServerBuilder::SyncServerOption max_pollers_option =
        grpc::ServerBuilder::SyncServerOption::MAX_POLLERS;
    builder.SetSyncServerOption(max_pollers_option, 2);
    server_.reset(builder.BuildAndStart().release());
    LOG(INFO) << "GRPC Listen on: " << service_addr_;
    server_->Wait();
    LOG(INFO) << "GRPC Server is break: " << service_addr_;
  });
  t.detach();

  return true;
}

void GrpcServiceComponent::updateHook() {}

void GrpcServiceComponent::stopHook() {}

void GrpcServiceComponent::cleanupHook() {}

grpc::Status
GrpcServiceComponent::Ping(ServerContext *context,
                           const ::google::protobuf::Empty *request,
                           rosc::PingResponse *reply) {
  LOG(INFO) << "ping...";
  reply->set_reply("Pong");
  reply->set_code(0);
  return grpc::Status::OK;
}

inline ExecStatus
GrpcServiceComponent::ExeCommandAndGetResult(rosc::SingleCommand command) {
  out_to_service_port_lock_.lock();
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>
      tp = std::chrono::time_point_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now());  // 获取当前时间点
  // 计算距离1970-1-1,00:00的时间长度
  command.timestamp = tp.time_since_epoch().count();
  command.timestamp |= 0xFF00000000000000;  // FF用于标识grpc发送的时间戳
  this->out_to_service_port_.write(command);
  out_to_service_port_lock_.unlock();
  CommandExecResult feedback_command;
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    in_from_service_port_lock_.lock();
    RTT::FlowStatus status = this->in_from_service_port_.read(feedback_command);
    in_from_service_port_lock_.unlock();
    if (status == RTT::NewData &&
        ((feedback_command.timestamp & 0xFF00000000000000) ==
         0xFF00000000000000)) {
      timestamps_buffer_lock_.lock();
      timestamps_buffer_.push(feedback_command);
      timestamps_buffer_lock_.unlock();
    }
    if (timestamps_buffer_.front().timestamp == command.timestamp) {
      timestamps_buffer_lock_.lock();
      CommandExecResult current_command_feedback = timestamps_buffer_.front();
      timestamps_buffer_.pop();
      timestamps_buffer_lock_.unlock();
      LOG(INFO) << "-------------------GRPC EXEC END------------------------";
      return current_command_feedback.status;
    }
  }
}

grpc::Status
GrpcServiceComponent::PowerOn(ServerContext *context,
                              const ::google::protobuf::Empty *request,
                              rosc::PowerOnResponse *reply) {
  SingleCommand command;
  LOG(INFO) << "Start Power On";
  command.op_type = rosc::CommandType::POWER_ON;
  ExecStatus status = ExeCommandAndGetResult(command);
  if (status == ExecStatus::SUCCESS) {
    reply->set_code(0);
    return grpc::Status::OK;
  } else {
    reply->set_code(1);
    reply->set_reply("上电失败");
    return grpc::Status::OK;
  }
}

grpc::Status
GrpcServiceComponent::PowerOff(ServerContext *context,
                               const ::google::protobuf::Empty *request,
                               rosc::PowerOffResponse *reply) {
  SingleCommand command;
  LOG(INFO) << "Start Power Off";
  command.op_type = rosc::CommandType::POWER_OFF;
  ExecStatus status = ExeCommandAndGetResult(command);
  if (status == ExecStatus::SUCCESS) {
    reply->set_code(0);
    return grpc::Status::OK;
  } else {
    reply->set_code(1);
    reply->set_reply("下电失败");
    return grpc::Status::OK;
  }
}

grpc::Status
GrpcServiceComponent::ClearError(ServerContext *context,
                                 const ::google::protobuf::Empty *request,
                                 rosc::ClearErrorResponse *reply) {
  LOG(INFO) << "Start Clear Error";
  SingleCommand command;
  command.op_type = rosc::CommandType::CLEAR;
  ExeCommandAndGetResult(command);
  LOG(INFO) << "清除错误执行完成";
  reply->set_code(0);
  return grpc::Status::OK;
}

/**
 * @brief 急停
 *
 * @param context
 * @param request
 * @param reply
 * @return Status
 */
grpc::Status
GrpcServiceComponent::EmergencyStop(ServerContext *context,
                                    const rosc::EmergencyStopRequest *request,
                                    rosc::EmergencyStopResponse *reply) {
  LOG(INFO) << "Start Emergency Stop";
  ServiceEmergencyStop();
  reply->set_code(0);
  LOG(INFO) << "Emergency Stop Cpmplete";
  return grpc::Status::OK;
}

grpc::Status GrpcServiceComponent::EmergencyRecover(
    ServerContext *context, const rosc::EmergencyRecoverRequest *request,
    rosc::EmergencyRecoverResponse *reply) {
  LOG(INFO) << "Start Emergency Recover";
  ServiceEmergencyRecover();
  reply->set_code(0);
  LOG(INFO) << "Emergency Recover Complete";
  return grpc::Status::OK;
}

grpc::Status
GrpcServiceComponent::TeachMoveStep(ServerContext *context,
                                    const rosc::TeachMoveStepRequest *request,
                                    rosc::TeachMoveStepResponse *reply) {
  SingleCommand command;
  GRPCAxisNum axis = request->axis();
  LOG(INFO) << "Jog step start, axis: " << axis;
  AxisNum which_axis;
  switch (axis) {
  case GRPCAxisNum::ELEVATION:  // 'z'
    which_axis = rosc::AxisNum::Elevation;
    break;
  case GRPCAxisNum::ROTATION:  // 'r'
    which_axis = rosc::AxisNum::Rotation;
    break;
  case GRPCAxisNum::EXTENSION_L:  // 'e'
    which_axis = rosc::AxisNum::Extension_L;
    break;
  case GRPCAxisNum::EXTENSION_R:
    which_axis = rosc::AxisNum::Extension_R;
    break;
  case GRPCAxisNum::EXTENSION:
    which_axis = rosc::AxisNum::Extension;
    break;
  case GRPCAxisNum::WRIST_R:
    which_axis = rosc::AxisNum::Wrist_R;
    break;
  case GRPCAxisNum::WRIST_L:
    which_axis = rosc::AxisNum::Wrist_L;
    break;
  case GRPCAxisNum::CARTESIAN_X_L:
    which_axis = rosc::AxisNum::Cartesian_X_L;
    break;
  case GRPCAxisNum::CARTESIAN_X_R:
    which_axis = rosc::AxisNum::Cartesian_X_R;
    break;
  case GRPCAxisNum::CARTESIAN_Y_L:
    which_axis = rosc::AxisNum::Cartesian_Y_L;
    break;
  case GRPCAxisNum::CARTESIAN_Y_R:
    which_axis = rosc::AxisNum::Cartesian_Y_R;
    break;
  case GRPCAxisNum::FLIP_L:
    which_axis = rosc::AxisNum::Flip_L;
    break;
  case GRPCAxisNum::FLIP_R:
    which_axis = rosc::AxisNum::Flip_R;
    break;
  default:
    LOG(ERROR) << "Jog dof Error!";
    break;
  }

  command.op_type = rosc::CommandType::MOVE_STEP;
  command.whichAxis = which_axis;
  command.jogstep = request->jog_step();
  command.direction = request->direction();
  ExecStatus status = ExeCommandAndGetResult(command);
  if (status == ExecStatus::SUCCESS) {
    reply->set_code(0);
    return grpc::Status::OK;
  } else {
    reply->set_code(1);
    reply->set_reply("点动失败");
    return grpc::Status::OK;
  }
}

grpc::Status
GrpcServiceComponent::TeachJogStart(ServerContext *context,
                                    const rosc::TeachJogStartRequest *request,
                                    rosc::TeachJogStartResponse *reply) {
  LOG(INFO) << "Jog start";
  SingleCommand command;
  int axis = request->axis();
  AxisNum which_axis;
  switch (axis) {
  case GRPCAxisNum::ELEVATION:  // 'z'
    which_axis = rosc::AxisNum::Elevation;
    break;
  case GRPCAxisNum::ROTATION:  // 'r'
    which_axis = rosc::AxisNum::Rotation;
    break;
  case GRPCAxisNum::EXTENSION_L:  // 'e'
    which_axis = rosc::AxisNum::Extension_L;
    break;
  case GRPCAxisNum::EXTENSION_R:
    which_axis = rosc::AxisNum::Extension_R;
    break;
  case GRPCAxisNum::EXTENSION:
    which_axis = rosc::AxisNum::Extension;
    break;
  case GRPCAxisNum::WRIST_R:
    which_axis = rosc::AxisNum::Wrist_R;
    break;
  case GRPCAxisNum::WRIST_L:
    which_axis = rosc::AxisNum::Wrist_L;
    break;
  case GRPCAxisNum::CARTESIAN_X_L:
    which_axis = rosc::AxisNum::Cartesian_X_L;
    break;
  case GRPCAxisNum::CARTESIAN_X_R:
    which_axis = rosc::AxisNum::Cartesian_X_R;
    break;
  case GRPCAxisNum::CARTESIAN_Y_L:
    which_axis = rosc::AxisNum::Cartesian_Y_L;
    break;
  case GRPCAxisNum::CARTESIAN_Y_R:
    which_axis = rosc::AxisNum::Cartesian_Y_R;
    break;
  case GRPCAxisNum::FLIP_L:
    which_axis = rosc::AxisNum::Flip_L;
    break;
  case GRPCAxisNum::FLIP_R:
    which_axis = rosc::AxisNum::Flip_R;
    break;
  default:
    LOG(ERROR) << "Jog dof Error!";
    break;
  }
  command.op_type = rosc::CommandType::JOG_START;
  command.whichAxis = which_axis;
  command.direction = request->direction();
  enable_jog_.store(true, std::memory_order_relaxed);
  while (enable_jog_.load(std::memory_order_relaxed)) {
    ExecStatus status = ExeCommandAndGetResult(command);
    if (status != ExecStatus::SUCCESS) {
      reply->set_code(1);
      reply->set_reply("点动开启失败");
      return grpc::Status::OK;
    }
    // usleep(10000);
  }
  reply->set_code(0);
  return grpc::Status::OK;
}

grpc::Status
GrpcServiceComponent::TeachJogStop(ServerContext *context,
                                   const rosc::TeachJogStopRequest *request,
                                   rosc::TeachJogStopResponse *reply) {
  LOG(INFO) << "Jog stop";
  enable_jog_.store(false, std::memory_order_relaxed);
  SingleCommand command;
  command.op_type = rosc::CommandType::JOG_STOP;
  ExecStatus status = ExeCommandAndGetResult(command);
  if (status == ExecStatus::SUCCESS) {
    reply->set_code(0);
    return grpc::Status::OK;
  } else {
    reply->set_code(1);
    return grpc::Status::OK;
  }
}

grpc::Status
GrpcServiceComponent::TeachMove(ServerContext *context,
                                const rosc::TeachMoveRequest *request,
                                rosc::TeachMoveResponse *reply) {
  LOG(INFO) << "Start Teach Move Home";
  SingleCommand command;
  std::string point_name = request->name();
  rosc::Config config = *rosc::Application::GetContext()->GetConfig();
  boost::filesystem::path point_file(
      config["init_global_config"]["data"]["root_path"].as<std::string>());
  point_file.append(
      config["init_global_config"]["data"]["files_path"]["points_file"]
          .as<std::string>());
  if (!boost::filesystem::is_regular_file(point_file)) {
    LOG(ERROR) << "load file " << point_file << "error!";
    throw rosc::RobotException(StatusCode::kConfigFileNotExist);
  }
  YAML::Node points = YAML::LoadFile(point_file.string());
  YAML::Node end_point = points[point_name]["joint"];
  if (!end_point.IsDefined()) {
    reply->set_code(1);
    reply->set_reply("数据不存在！");
    return grpc::Status::OK;
  }
  auto it = end_point.begin();
  int axis_index = 0;
  while (it != end_point.end() && axis_index < this->dof_) {
    command.end.joint(axis_index) = (it->second).as<double>();
    ++it;
    ++axis_index;
  }
  RobotCurrentState state;
  this->GetCurrentPosition(&state);
  for (int i = 0; i < this->dof_; ++i) {
    command.start.joint(i) = state.joint[i];
  }
  command.speed = rosc::TransferSpeedType::LOW_SPEED;
  command.op_type = rosc::CommandType::MOTION;
  ExecStatus status = ExeCommandAndGetResult(command);
  if (status == ExecStatus::SUCCESS) {
    reply->set_code(0);
    return grpc::Status::OK;
  } else {
    reply->set_code(1);
    reply->set_reply("运动指令执行失败");
    return grpc::Status::OK;
  }
}

grpc::Status
GrpcServiceComponent::RobotSetting(ServerContext *context,
                                   const rosc::RobotSettingRequest *request,
                                   rosc::RobotSettingResponse *reply) {
  const rosc::Config config = *rosc::Application::GetContext()->GetConfig();
  LOG(INFO) << "获取配置数据，配置类型为：" << request->setting_type();
  if (request->setting_type() == "device") {
    rosc::DeviceSetting *device_setting = new rosc::DeviceSetting();
    YAML::Node config_node;
    config_node = config["zero_device_config"]["M124_device_config"]["robot"];

    int dof = config_node["dof"].as<int>();
    device_setting->set_dof(dof);
    int io = config_node["IO_slave_seq"].as<int>();
    device_setting->set_io(io);

    reply->set_setting_type(request->setting_type());
    reply->set_allocated_device_setting(device_setting);
    reply->set_code(0);
  } else if (request->setting_type() == "joints") {
    YAML::Node joints_node;

    joints_node =
        config["zero_device_config"]["zero_device_config"]["robot"]["joints"];

    rosc::JointSetting *joint_setting = new rosc::JointSetting();
    for (auto it = joints_node.begin(); it != joints_node.end(); ++it) {
      joint_setting->add_joint_name(it->first.as<std::string>());
      joint_setting->add_vel_max(it->second["vel_max"].as<double>());
      joint_setting->add_acc_max(it->second["acc_max"].as<double>());
      joint_setting->add_jerk(it->second["jerk"].as<double>());

      if (it->first.as<std::string>() != "linear-motion") {
        joint_setting->add_motionrangestart(
            it->second["MotionRangeStart"].as<double>());
        joint_setting->add_motionrangeend(
            it->second["MotionRangeEnd"].as<double>());
        joint_setting->add_dcc_max(it->second["dcc_max"].as<double>());
        joint_setting->add_dcc_jerk(it->second["dcc_jerk"].as<double>());
      }
    }

    reply->set_allocated_joint_setting(joint_setting);
    reply->set_setting_type(request->setting_type());
    reply->set_code(0);
  } else {
    reply->set_code(1);
    reply->set_reply("配置数据不存在");
  }
  LOG(INFO) << "获取配置数据方法return";
  return grpc::Status::OK;
}

grpc::Status GrpcServiceComponent::SaveJointSetting(
    ServerContext *context, const rosc::SaveJointSettingRequest *request,
    rosc::SaveJointSettingResponse *reply) {
  rosc::Config config = *rosc::Application::GetContext()->GetConfig();
  auto rob_model = rosc::Application::GetContext()->GetRobotModel();
  std::string node_name;

  node_name = "zero_device_config";

  YAML::Node device_node = config[node_name];
  int data_size = request->data_size();
  for (int i = 0; i < data_size; ++i) {
    rosc::SaveJointSettingItem item = request->data(i);
    std::string axis_name = item.axis_name();
    std::string attribute = item.attribute();
    double value = item.value();
    device_node[node_name]["robot"]["joints"][axis_name][attribute] = value;
  }
  config.UpdateConfig(node_name, device_node);
  rob_model->UpdateConfig(device_node);
  reply->set_code(0);
  return grpc::Status::OK;
}

grpc::Status GrpcServiceComponent::TeachSavePosition(
    ServerContext *context, const rosc::TeachSavePositionRequest *request,
    rosc::TeachSavePositionResponse *reply) {
  std::string point_name = request->name();
  auto data_manager = rosc::Application::GetContext()->GetDataManager();
  if (data_manager->IsExist(point_name) && request->code() == 0) {
    reply->set_code(2);
    return grpc::Status::OK;
  }
  SingleCommand command;
  snprintf(command.point_name, sizeof(command.point_name), "%s",
           point_name.c_str());
  std::string describe = request->describe();
  snprintf(command.mesg, sizeof(command.mesg), "%s", describe.c_str());
  std::string arm = request->arm();

  LOG(INFO) << "执行保存点位，点名称为: " << command.point_name;
  command.op_type = rosc::CommandType::POINT;
  ExecStatus status = ExeCommandAndGetResult(command);
  if (status == ExecStatus::SUCCESS) {
    if (point_name.find("cassette") == point_name.npos &&
        point_name.find("PA") == point_name.npos &&
        point_name.find("transfer") == point_name.npos) {
      reply->set_code(0);
      return grpc::Status::OK;
    }
    std::size_t npos = point_name.find_last_of("_");

    int index = std::stoi(point_name.substr(npos + 1,
                                            point_name.length() - (npos + 1))) -
                1;
    std::string point_type = point_name.substr(0, npos);
    LOG(INFO) << "point_type: " << point_type;
    rosc::Config config = *rosc::Application::GetContext()->GetConfig();
    int has_mid = request->has_mid();
    if (point_type == "cassette_stage") {
      config["sorter_config"]["cassette_offset"]["intermediate_point"][index] =
          has_mid;
      config.UpdateConfig("sorter_config", config["sorter_config"]);
    } else if (point_type == "transfer_stage") {
      config["sorter_config"]["stage_offset"]["intermediate_point"][index] =
          has_mid;
      config.UpdateConfig("sorter_config", config["sorter_config"]);
    } else if (point_type == "PA") {
      config["sorter_config"]["PA_offset"]["intermediate_point"][index] =
          has_mid;
      config.UpdateConfig("sorter_config", config["sorter_config"]);
    } else if (point_type == "cassette_stage_mid_point") {
      config["sorter_config"]["cassette_offset"]["intermediate_point"][index] =
          1;
      config.UpdateConfig("sorter_config", config["sorter_config"]);
    }
    usleep(100000);
    RobotCurrentState state;
    this->GetCurrentPosition(&state);
    reply->clear_joints();
    for (int i = 0; i < this->dof_; ++i) {
      reply->add_joints(state.joint[i]);
    }
    reply->set_code(0);
    return grpc::Status::OK;
  } else {
    reply->set_code(1);
    if (point_name == "ZERO") {
      reply->set_reply("零点标定失败");
    } else {
      std::string reply_msg = "点位：" + point_name + "保存失败";
      reply->set_reply(reply_msg);
    }
    return grpc::Status::OK;
  }
}

grpc::Status GrpcServiceComponent::TeachSetVelocity(
    ServerContext *context, const rosc::TeachSetVelocityRequest *request,
    rosc::TeachSetVelocityResponse *reply) {
  int speed_level = request->velocity();
  SingleCommand command;
  command.op_type = CommandType::SSLV;
  command.speedLevel = speed_level;
  ExecStatus status = ExeCommandAndGetResult(command);
  if (status == ExecStatus::SUCCESS) {
    reply->set_code(0);
  } else {
    reply->set_code(1);
    reply->set_reply("设置速度失败");
  }

  LOG(INFO) << "执行Set Velocity level: " << speed_level;
  return grpc::Status::OK;
}

grpc::Status GrpcServiceComponent::TeachGetVelocity(
    ServerContext *context, const rosc::TeachGetVelocityRequest *request,
    rosc::TeachGetVelocityResponse *reply) {
  RobotCurrentState state;
  this->GetCurrentPosition(&state);
  int speed_level = static_cast<int>(state.current_speed_level);
  reply->set_velocity(speed_level);
  reply->set_code(0);
  return grpc::Status::OK;
}

grpc::Status
GrpcServiceComponent::DeletePoint(ServerContext *context,
                                  const rosc::DeletePointRequest *request,
                                  rosc::DeletePointResponse *reply) {
  std::string point_name = request->name();
  int type = request->code();
  std::string arm = request->message();
  LOG(INFO) << "修改数据：" << point_name << " type: " << type;
  if (type == 0) {
    auto data_manager = Application::GetContext()->GetDataManager();
    data_manager->DeletePoint(point_name);
    data_manager->Save();
  } else if (type == 1) {
    auto data_manager = Application::GetContext()->GetDataManager();
    data_manager->EmptyPoint(point_name);
    data_manager->Save();
  } else if (type == 2) {
    auto data_manager = Application::GetContext()->GetDataManager();
    YAML::Node points =
        Application::GetContext()->GetDataManager()->GetOriginPointData();
    for (auto it = points.begin(); it != points.end(); ++it) {
      std::string point_name = it->first.as<std::string>();
      if (point_name.find("ZERO") == point_name.npos) {
        data_manager->EmptyPoint(point_name);
      }
    }
    data_manager->Save();
  }

  reply->set_code(0);
  return grpc::Status::OK;
}

grpc::Status GrpcServiceComponent::GetAllPointNames(
    ServerContext *context, const rosc::GetAllPointNamesRequest *request,
    rosc::GetAllPointNamesResponse *reply) {
  YAML::Node points =
      Application::GetContext()->GetDataManager()->GetOriginPointData();
  for (auto it = points.begin(); it != points.end(); ++it) {
    reply->add_point_names(it->first.as<std::string>());
  }
  reply->set_code(0);
  return grpc::Status::OK;
}

grpc::Status GrpcServiceComponent::DownloadPointFile(
    ServerContext *context, const rosc::DownloadPointFileRequest *request,
    grpc::ServerWriter<rosc::DownloadPointFileResponse> *stream) {
  std::ifstream point_file;
  // 从工控机服务端下载文件到示教器端
  auto data_manager = Application::GetContext()->GetDataManager();
  std::string point_file_name = data_manager->GetPointDataFilePath();

  point_file.open(point_file_name, std::ifstream::in | std::ifstream::binary);
  const int kDataSize = 1024 * 1024;
  char point_data[kDataSize];
  rosc::DownloadPointFileResponse response;
  while (!point_file.eof()) {
    point_file.read(point_data, kDataSize);
    response.set_data(point_data, point_file.gcount());
    if (!stream->Write(response)) {
      // Broken stream
      break;
    }
  }
  point_file.close();
  return grpc::Status::OK;
}

grpc::Status
GrpcServiceComponent::GoTeachJoint(ServerContext *context,
                                   const rosc::GoTeachJointRequest *request,
                                   rosc::GoTeachJointResponse *reply) {
  std::string point_name = request->point_name();
  if (point_name == "HOME") {
    SingleCommand command;
    command.op_type = rosc::CommandType::MHOM;
    ExecStatus status = ExeCommandAndGetResult(command);
    if (status == ExecStatus::SUCCESS) {
      reply->set_code(0);
      return grpc::Status::OK;
    } else {
      reply->set_code(1);
      std::string reply_msg = "机械臂回零失败";
      reply->set_reply(reply_msg);
      return grpc::Status::OK;
    }
  } else {
    auto data_manager = rosc::Application::GetContext()->GetDataManager();
    // LOG(INFO) << data_manager->GetOriginPointData();
    YAML::Node point = data_manager->GetPointByName(point_name);
    if (!point) {
      reply->set_code(1);
      reply->set_reply("数据不存在！");
      return grpc::Status::OK;
    }
    SingleCommand command;
    for (int i = 0; i < this->dof_; ++i) {
      command.end.joint(i) = point["joint"][i].as<double>();
    }

    command.op_type = rosc::CommandType::POINT_JOG_START;
    ExecStatus status = ExeCommandAndGetResult(command);
    if (status == ExecStatus::SUCCESS) {
      reply->set_code(0);
      return grpc::Status::OK;
    } else {
      reply->set_code(1);
      std::string reply_msg = "点动失败";
      reply->set_reply(reply_msg);
      return grpc::Status::OK;
    }
  }
}

grpc::Status GrpcServiceComponent::SetIOOutputState(
    ServerContext *context, const rosc::SetIOOutputStateRequest *request,
    rosc::SetIOOutputStateResponse *reply) {
  IOCommand command;
  int which_io = request->serial();
  if (which_io < 0 || which_io > 23) {
    LOG(ERROR) << "IO 编号错误！";
  }
  command.whichIO = which_io;
  command.io_op = request->state();
  command.SetParam(which_io, request->state());
  LOG(INFO) << "io out control: " << command.whichIO
            << ", state: " << command.io_op;

  ExecStatus status = ExeCommandAndGetResult(command);
  if (status == ExecStatus::SUCCESS) {
    reply->set_code(0);
    return grpc::Status::OK;
  } else {
    reply->set_code(1);
    std::string reply_msg = "IO设置失败";
    reply->set_reply(reply_msg);
    return grpc::Status::OK;
  }
}

grpc::Status GrpcServiceComponent::GetIOOutputState(
    ServerContext *context, const rosc::GetIOOutputStateRequest *request,
    rosc::GetIOOutputStateResponse *reply) {
  RobotCurrentState state;
  this->GetCurrentPosition(&state);
  LOG(INFO) << "GRPC Get IO State " << state.digit_io_out;

  uint16_t digit_io_out = state.digit_io_out;
  reply->clear_states();
  for (uint i = 0; i < sizeof(digit_io_out) * 8; ++i) {
    bool s = digit_io_out & (1 << i);
    reply->add_states(s);
  }
  reply->set_code(0);
  return grpc::Status::OK;
}

grpc::Status GrpcServiceComponent::GetIOInputState(
    ServerContext *context, const rosc::GetIOInputStateRequest *request,
    grpc::ServerWriter<rosc::GetIOInputStateResponse> *reply) {
  start_io_input_ = true;
  RobotCurrentState state;
  while (start_io_input_) {
    this->GetCurrentPosition(&state);
    rosc::GetIOInputStateResponse io_state;
    uint16_t digit_io_in = state.digit_io_in;
    for (uint i = 0; i < sizeof(digit_io_in) * 8; ++i) {
      bool s = digit_io_in & (1 << i);
      io_state.add_states(s);
    }
    reply->Write(io_state);
    usleep(100000);
  }
  return grpc::Status::OK;
}

grpc::Status GrpcServiceComponent::StopGetIOInputState(
    ServerContext *context, const rosc::StopGetIOInputStateRequest *request,
    rosc::StopGetIOInputStateResponse *reply) {
  start_io_input_ = false;
  reply->set_code(0);
  return grpc::Status::OK;
}

grpc::Status GrpcServiceComponent::TeachInstruction(
    ServerContext *context, const rosc::TeachInstructionRequest *request,
    rosc::TeachInstructionResponse *reply) {
  LOG(INFO) << "指令操作";
  if (request->op_type() == "NOP") {
    int sleep_time = request->sleep_time();
    SingleCommand nop_command;
    nop_command.op_type = rosc::CommandType::NOP;
    nop_command.milliseconds = sleep_time;
    ExecStatus status = ExeCommandAndGetResult(nop_command);
    if (status == ExecStatus::SUCCESS) {
      reply->set_code(0);
      return grpc::Status::OK;
    } else {
      reply->set_code(1);
      return grpc::Status::OK;
    }
  }
  std::string start_point = request->start_point();
  auto data_manager = rosc::Application::GetContext()->GetDataManager();
  YAML::Node start = data_manager->GetPointByName(start_point);
  SingleCommand pre_command;
  int speed = request->speed();
  pre_command.speed = rosc::TransferSpeedType::FIRST_TRANSFER_SPEED;
  switch (speed) {
  case 1:
    pre_command.speedLevel = 0;
    break;
  case 2:
    pre_command.speedLevel = 1;
    break;
  case 3:
    pre_command.speedLevel = 2;
    break;
  }
  for (int i = 0; i < this->dof_; ++i) {
    pre_command.end.joint(i) = start["joint"][i].as<double>();
  }
  pre_command.op_type = rosc::CommandType::MOVETO;
  ExecStatus status = ExeCommandAndGetResult(pre_command);
  if (status != ExecStatus::SUCCESS) {
    reply->set_code(1);
    return grpc::Status::OK;
  }

  // usleep(1000);
  std::string end_point = request->end_point();
  YAML::Node end = data_manager->GetPointByName(end_point);
  SingleCommand command;

  status = ExeCommandAndGetResult(command);
  if (status == ExecStatus::SUCCESS) {
    reply->set_code(0);
    return grpc::Status::OK;
  } else {
    reply->set_code(1);
    return grpc::Status::OK;
  }
}

grpc::Status GrpcServiceComponent::TeachStreamStartGetJoints(
    ServerContext *context, const ::google::protobuf::Empty *request,
    ::grpc::ServerWriter<::rosc::TeachStreamStartGetJointsResponse> *writer) {
  // 如果已经开启，把原有线程关闭
  rosc::TeachStreamStartGetJointsResponse response;
  RobotCurrentState state;
  // 加入已经加锁，释放一个信号，将原有线程关闭
  while (!teach_stream_get_joints_lock_.try_lock()) {
    LOG(WARNING) << "TeachStreamStartGetJoints have started, Try to stop the "
                    "previous thread.";
    this->start_get_stream_joints_ = false;
    usleep(100000);
  }
  this->start_get_stream_joints_ = true;
  LOG(INFO) << "TeachStreamStartGetJoints start";
  // 考虑并行，start_get_stream_joints_
  // 可能会被TeachStreamStopGetJoints方法修改为false
  while (this->start_get_stream_joints_) {
    // 如果客户端已经关闭，直接退出
    if (context->IsCancelled()) {
      LOG(INFO) << "TeachStreamStartGetJoints context->IsCancelled()";
      break;
    }
    this->GetCurrentPosition(&state);
    switch (state.state) {
    case ControllerState::ON:
      response.set_servo_state("ON");
      break;
    case ControllerState::OFF:
      response.set_servo_state("OFF");
      break;
    case ControllerState::Error:
      response.set_servo_state("ERROR");
      break;
    case ControllerState::Init:
      response.set_servo_state("Init");
      break;
    case ControllerState::Op:
      response.set_servo_state("Op");
      break;
    case ControllerState::Idle:
      response.set_servo_state("Idle");
      break;
    case ControllerState::EMC_STOP:
      response.set_servo_state("EMC_STOP");
      break;
    default:
      break;
    }
    response.clear_joints();
    for (int i = 0; i < this->dof_; ++i) {
      response.add_joints(state.joint[i]);
    }
    response.clear_velocitys();
    for (int i = 0; i < this->dof_; ++i) {
      response.add_velocitys(state.velocity[i]);
    }

    response.set_code(0);
    auto func =
        [](::grpc::ServerWriter<::rosc::TeachStreamStartGetJointsResponse>
               *writer,
           const rosc::TeachStreamStartGetJointsResponse &response) {
          writer->Write(response);
        };
    auto future = std::async(
        std::launch::async,
        std::bind(
            static_cast<void(*)(
                ::grpc::ServerWriter<::rosc::TeachStreamStartGetJointsResponse>
                    *,
                const rosc::TeachStreamStartGetJointsResponse &)>(func),
            std::ref(writer), std::ref(response)));
    if (future.wait_for(std::chrono::seconds(1)) ==
        std::future_status::timeout) {
      LOG(INFO) << "TeachStreamStartGetJoints ServerWrite timeout";
      break;
    }
    // writer->Write(response);
    usleep(100000);
  }
  teach_stream_get_joints_lock_.unlock();
  LOG(INFO) << "TeachStreamStartGetJoints end";
  return grpc::Status::OK;
}

grpc::Status GrpcServiceComponent::TeachStreamStopGetJoints(
    grpc::ServerContext *context, const ::google::protobuf::Empty *request,
    rosc::TeachStreamStopGetJointsResponse *response) {
  this->start_get_stream_joints_ = false;
  response->set_code(0);
  response->set_reply("");
  return grpc::Status::OK;
}

void UintToStr(uint32_t code, char *const str) {
  const char kHexes[] = {'0', '1', '2', '3', '4', '5', '6', '7',
                         '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
  uint32_t mask = 0x0000000Fu;
  const int kLength = sizeof(code) * 2;
  for (int i = kLength - 1; i >= 0; --i) {
    int index = code & mask;
    str[i] = kHexes[index];
    code = code >> 4;
  }
}

grpc::Status GrpcServiceComponent::GetStreamException(
    ServerContext *context,
    grpc::ServerReaderWriter<rosc::GetStreamExceptionResponse,
                             rosc::GetStreamExceptionRequest> *stream) {
  LOG(INFO) << "GRPC transfer exception message start";
  rosc::GetStreamExceptionRequest request;
  rosc::GetStreamExceptionResponse response;
  while (true) {
    response.clear_code();
    response.clear_status_code();
    response.clear_reply();
    response.clear_level();
    response.clear_time();
    response.clear_message();
    bool read_res = stream->Read(&request);
    if (!read_res) {
      return grpc::Status::OK;
    }
    if (request.message() == "STOP") {
      response.set_code(0);
      response.set_level(0);
      stream->Write(response);
      return grpc::Status::OK;
    }
    rosc::StatusCode status_code;
    RTT::FlowStatus status = this->in_from_exception_port_.read(status_code);
    if (status == RTT::NewData) {
      response.set_code(0);
      char code_str[16];
      UintToStr(std::move(status_code), code_str);
      response.set_status_code(static_cast<std::string>(code_str));

      time_t current_time;
      time(&current_time);
      char current_time_str[64];
      strftime(current_time_str, sizeof(current_time_str), "%Y-%m-%d %H:%M:%S",
               localtime(&current_time));
      response.set_time(static_cast<std::string>(current_time_str));

      const uint32_t kLevelMask = 0x0000000Fu;
      int level = (static_cast<uint32_t>(status_code) >> 16) & kLevelMask;
      response.set_level(level);
      std::string log_msg = std::string(current_time_str) + " " +
                            std::to_string(status_code) + " " +
                            std::to_string(level) + " " +
                            rosc::Status::GetInfobyCode(status_code) + "\n";
      response.set_message(rosc::Status::GetInfobyCode(status_code));
      WriteToLog(status_code);
    } else {
      response.set_code(0);
      response.set_level(0);
      response.set_message(
          rosc::Status::GetInfobyCode(rosc::StatusCode::kSuccess));
    }
    stream->Write(response);
    usleep(1000000);
  }
  return grpc::Status::OK;
}

grpc::Status
GrpcServiceComponent::GetException(ServerContext *context,
                                   const rosc::GetExceptionRequest *request,
                                   rosc::GetExceptionResponse *response) {
  LOG(INFO) << "get exception!";
  std::vector<std::string> exceptions = ReadExceptionLog();
  std::vector<std::string> infos;
  std::string info;
  for (auto exception : exceptions) {
    std::stringstream ss(exception);
    while (std::getline(ss, info, '|')) {
      infos.emplace_back(info);
    }
    response->add_time(infos[0]);
    response->add_status_code(infos[1]);
    response->add_level(std::stoi(infos[2]));
    response->add_message(infos[3]);
    infos.clear();
  }
  return grpc::Status::OK;
}

grpc::Status GrpcServiceComponent::DeleteException(
    ServerContext *context, const rosc::DeleteExceptionRequest *request,
    rosc::DeleteExceptionResponse *response) {
  LOG(INFO) << "delete exception!";
  ClearExceptionLog();
  return grpc::Status::OK;
}

void UintToStr(uint16_t code, char *const str) {
  const char kHexes[] = {'0', '1', '2', '3', '4', '5', '6', '7',
                         '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
  uint16_t mask = 0x0000000Fu;
  const int kLength = sizeof(code) * 2;
  for (int i = kLength - 1; i >= 0; --i) {
    int index = code & mask;
    str[i] = kHexes[index];
    code = code >> 4;
  }
}

grpc::Status GrpcServiceComponent::SlaveState(
    ServerContext *context,
    grpc::ServerReaderWriter<rosc::SlaveStateResponse, rosc::SlaveStateRequest>
        *stream) {
  LOG(INFO) << "GRPC transfer slave state start";
  RobotCurrentState currentState;
  rosc::SlaveStateRequest request;
  rosc::SlaveStateResponse response;
  while (true) {
    this->GetCurrentPosition(&currentState);
    stream->Read(&request);
    if (request.message() != "get slave state") {
      break;
    }
    response.clear_torque();
    response.clear_error_code();
    response.clear_velocity();
    for (int i = 0; i < this->dof_; ++i) {
      response.add_torque(currentState.torque[i]);
      response.add_velocity(currentState.velocity[i]);
      char code_str[32];
      UintToStr(currentState.error_code[i], code_str);
      response.add_error_code(code_str);
    }
    response.set_code(0);
    stream->Write(response);
    usleep(1000);
  }
  return grpc::Status::OK;
}

grpc::Status
GrpcServiceComponent::GetSpeedConfig(ServerContext *context,
                                     const rosc::GetSpeedConfigRequest *request,
                                     rosc::GetSpeedConfigResponse *response) {
  std::string speed_type = request->speed_type();
  std::string speed_level = request->speed_level();
  std::string joint = request->joint();
  LOG(INFO) << "speed_type" << speed_type << " speed_level" << speed_level
            << " joint" << joint;
  const rosc::Config config = *rosc::Application::GetContext()->GetConfig();
  YAML::Node speed_node;
  speed_node = config["zero_device_config"]["speed_config"][speed_type];

  std::string speed_config = "";
  for (auto it = speed_node.begin(); it != speed_node.end(); ++it) {
    std::string attribute = it->first.as<std::string>();
    LOG(INFO) << attribute;
    if (attribute.find(speed_level) != attribute.npos &&
        attribute.find(joint) != attribute.npos) {
      double value = it->second.as<double>();
      speed_config += attribute;
      speed_config += ":";
      speed_config += std::to_string(value);
      speed_config += ",";
    }
  }
  response->set_speed_config(speed_config);
  response->set_code(0);
  return grpc::Status::OK;
}

/**
 * @brief保存速度配置值
 *
 * @param context
 * @param request
 * @param reply
 * @return Status
 */
grpc::Status GrpcServiceComponent::SaveSpeedConfig(
    ServerContext *context, const rosc::SaveSpeedConfigRequest *request,
    rosc::SaveSpeedConfigResponse *response) {
  rosc::Config config = *rosc::Application::GetContext()->GetConfig();
  YAML::Node device_node;
  device_node = config["zero_device_config"];

  int data_size = request->data_size();
  std::string speed_type = request->speed_type();
  for (int i = 0; i < data_size; ++i) {
    rosc::SpeedConfig item = request->data(i);
    std::string attribute = item.attribute();
    double value = item.value();
    device_node["speed_config"][speed_type][attribute] = value;
  }

  config.UpdateConfig("zero_device_config", device_node);

  auto rob_model = rosc::Application::GetContext()->GetRobotModel();
  rob_model->UpdateConfig(device_node);
  response->set_code(0);
  return grpc::Status::OK;
}

grpc::Status GrpcServiceComponent::RedifyControllerIP(
    ServerContext *context, const rosc::RedifyControllerIPRequest *request,
    rosc::RedifyControllerIPResponse *response) {
  std::string network_config_file = "/etc/netplan/00-installer-config.yaml";
  YAML::Node net_config = YAML::LoadFile(network_config_file);
  std::string addresses = request->addresses();
  std::string gateway4 = request->gateway4();
  net_config["network"]["ethernets"]["enp3s0"]["addresses"][0] = addresses;
  net_config["network"]["ethernets"]["enp3s0"]["gateway4"] = gateway4;
  fs::path net_config_path(network_config_file);
  std::ofstream net_file(net_config_path.string());
  net_file << net_config;
  net_file.close();
  std::string netplan_apply = "netplan apply";
  int result = system(netplan_apply.c_str());
  if (result != 0) {
    response->set_code(1);
  }
  return grpc::Status::OK;
}

grpc::Status
GrpcServiceComponent::GetServoEncode(ServerContext *context,
                                     const rosc::GetServoEncodeRequest *request,
                                     rosc::GetServoEncodeResponse *response) {
  std::string zero_name;
  zero_name = "ZERO";
  auto data_manager = rosc::Application::GetContext()->GetDataManager();
  YAML::Node zero = data_manager->GetPointByName(zero_name);
  for (uint i = 0; i < zero["encoder"].size(); ++i) {
    response->add_encode(zero["encoder"][i].as<int>());
  }
  response->set_code(0);
  return grpc::Status::OK;
}

grpc::Status GrpcServiceComponent::UpdatePointFile(
    ServerContext *context,
    grpc::ServerReaderWriter<rosc::UpdatePointFileResponse,
                             rosc::UpdatePointFileRequest> *stream) {
  LOG(INFO) << "保存备份数据到工控机";
  rosc::UpdatePointFileRequest request;
  std::string data_str;
  while (stream->Read(&request)) {
    data_str.append(request.data());
  }
  YAML::Node backup_data = YAML::Load(data_str);
  auto data_manager = rosc::Application::GetContext()->GetDataManager();
  YAML::Node point = data_manager->GetOriginPointData();
  for (const auto &item : backup_data) {
    point[item.first.as<std::string>()] = item.second;
  }
  data_manager->UpdatePoint(point);
  data_manager->Save();
  rosc::UpdatePointFileResponse response;
  response.set_code(0);
  stream->Write(response);
  return grpc::Status::OK;
}

ORO_CREATE_COMPONENT(rosc::GrpcServiceComponent)
}  // namespace rosc
