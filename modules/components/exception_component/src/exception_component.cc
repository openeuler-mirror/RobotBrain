/**
 * @file exception_component.cc
 * @author DouYuanbo (douyuanbo@buaa.edu.cn)
 * @brief 机器人异常处理组件
 * @version 0.1
 * @date 2022-02-21
 *
 * @copyright Copyright (c) 2022 ROSC
 *
 */
#include "robot_brain/robot_exception/robot_status.h"
#include <robot_brain/core.hpp>
#include <exception_component.h>

#include <glog/logging.h>
#include <rtt/FlowStatus.hpp>
#include <time.h>

#include <iostream>
#include <fstream>

#include <rtt/Component.hpp>

namespace rosc {
struct ExceptionHandle::Data {
  StatusCode status_code;
  std::string msg;

  std::string ToString() {
    return "Code: " + std::to_string(status_code) + " Msg: " + msg;
  }
};

ExceptionHandle::ExceptionHandle(const std::string &name)
    : RTT::TaskContext(name, PreOperational),
      input_from_other_port_(
          "exception_status_port",
          Orocos::ConnPolicy::buffer(128, Orocos::ConnPolicy::LOCKED)),
      output_to_grpc_port_("exception_output_port") {
  this->setActivity(
      new Orocos::Activity(ORO_SCHED_OTHER, 0, 0, 4, 0, "exception_activity"));
  data_ = std::make_shared<Data>();
  this->ports()->addEventPort(input_from_other_port_);
  this->ports()->addPort(output_to_grpc_port_);
  this->history_code_ = StatusCode::kSuccess;
}

ExceptionHandle::ExceptionHandle() : ExceptionHandle("exception_handle") {}

bool ExceptionHandle::configureHook() {
  if (!input_from_other_port_.connected()) {
    LOG(ERROR)
        << "Exception Handle Component input_from_other_port_ not connected !";
    return false;
  }
  if (!output_to_grpc_port_.connected()) {
    LOG(ERROR)
        << "Exception Handle Component output_to_grpc_port_ not connected!";
    return false;
  }
  this->addOperation("WriteToLog", &ExceptionHandle::WriteToLog, this)
      .doc("WriteToLog");
  this->addOperation("GetCurrentExceptionCode",
                     &ExceptionHandle::GetCurrentExceptionCode, this)
      .doc("GetCurrentExceptionCode");
  this->addOperation("ResetExceptionCode", &ExceptionHandle::ResetExceptionCode,
                     this)
      .doc("ResetExceptionCode");
  this->addOperation("ReadExceptionLog", &ExceptionHandle::ReadExceptionLog,
                     this)
      .doc("ReadExceptionLog");
  this->addOperation("ClearExceptionLog", &ExceptionHandle::ClearExceptionLog,
                     this)
      .doc("ClearExceptionLog");
  return true;
}

bool ExceptionHandle::startHook() { return true; }

void ExceptionHandle::updateHook() {
  StatusCode code = StatusCode::kSuccess;
  RTT::FlowStatus status = input_from_other_port_.read(code);
  if (status == RTT::NewData) {  // 收到新数据
    if (this->history_code_ == StatusCode::kSuccess) {
      data_->status_code = static_cast<StatusCode>(code);
      LOG(WARNING) << "触发异常: " << std::hex << data_->status_code;
      output_to_grpc_port_.write(data_->status_code);
      this->history_code_ = data_->status_code;
      WriteToLog(data_->status_code);
    } else {
      data_->status_code = static_cast<StatusCode>(code);
      LOG(WARNING) << "触发异常: " << std::hex << data_->status_code;
      if (this->history_code_ != data_->status_code) {
        output_to_grpc_port_.write(data_->status_code);
        this->history_code_ = data_->status_code;
        WriteToLog(data_->status_code);
      }
    }
    this->trigger();
  } else if (status == RTT::OldData) {
    // this->history_code_ = StatusCode::kSuccess;
  }
}

void ExceptionHandle::stopHook() {}

void ExceptionHandle::cleanupHook() {}

void UintToStr(uint32_t code, char *const str) {
  const char kHexes[] = {'0', '1', '2', '3', '4', '5', '6', '7',
                         '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
  uint32_t mask = 0x0000000Fu;
  const int kLength = sizeof(code) * 2;
  str[kLength] = '\0';
  for (int i = kLength - 1; i >= 0; --i) {
    int index = code & mask;
    str[i] = kHexes[index];
    code = code >> 4;
  }
}

void ExceptionHandle::WriteToLog(rosc::StatusCode status_code) {
  std::ofstream log_file;
  rosc::Config config = *rosc::Application::GetContext()->GetConfig();
  std::string exception_log_path =
      config["init_global_config"]["exception_log_path"].as<std::string>();
  operate_log_lock_.try_lock();
  log_file.open(exception_log_path.c_str(), std::ios::app);
  if (!log_file.is_open()) {
    log_file.open(exception_log_path.c_str(), std::fstream::out);
  }
  time_t current_time;
  time(&current_time);
  char current_time_str[64];
  strftime(current_time_str, sizeof(current_time_str), "%Y-%m-%dT%H:%M:%S",
           localtime(&current_time));
  char code_str[16];
  UintToStr(std::move(status_code), code_str);
  std::string code = code_str;

  const uint32_t kLevelMask = 0x0000000Fu;
  int level = (static_cast<uint32_t>(status_code) >> 20) & kLevelMask;
  std::string log_msg = std::string(current_time_str) + "|" + code + "|" +
                        std::to_string(level) + "|" +
                        rosc::Status::GetInfobyCode(status_code) + "\n";
  log_file << log_msg;
  log_file.close();
  operate_log_lock_.unlock();
}

std::vector<std::string> ExceptionHandle::ReadExceptionLog() {
  rosc::Config config = *rosc::Application::GetContext()->GetConfig();
  std::string exception_log_path =
      config["init_global_config"]["exception_log_path"].as<std::string>();
  operate_log_lock_.try_lock();
  std::ifstream exception_file(exception_log_path.c_str());
  std::vector<std::string> exceptions;
  std::string exception;
  while (std::getline(exception_file, exception)) {
    exceptions.emplace_back(exception);
  }
  exception_file.close();
  operate_log_lock_.unlock();
  return exceptions;
}

void ExceptionHandle::ClearExceptionLog() {
  rosc::Config config = *rosc::Application::GetContext()->GetConfig();
  std::string exception_log_path =
      config["init_global_config"]["exception_log_path"].as<std::string>();
  operate_log_lock_.try_lock();
  std::ofstream exception_file(exception_log_path.c_str(),
                               std::ios::out | std::ios::trunc);
  exception_file.close();
  operate_log_lock_.unlock();
}

StatusCode ExceptionHandle::GetCurrentExceptionCode() {
  return this->history_code_;
}

void ExceptionHandle::ResetExceptionCode() {
  this->history_code_ = StatusCode::kSuccess;
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(ExceptionHandle)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(rosc::ExceptionHandle)
}  // namespace rosc
