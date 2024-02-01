/**
 * @file exception_component.h
 * @author YuanboDou (douyuanbo@buaa.edu.cn)
 * @brief 异常处理组件
 * @version 0.1
 * @date 2022-02-21
 *
 * @copyright Copyright (c) 2022 ROSC
 *
 */
#ifndef MODULES_COMPONENTS_EXCEPTION_COMPONENT_INCLUDE_EXCEPTION_COMPONENT_H_
#define MODULES_COMPONENTS_EXCEPTION_COMPONENT_INCLUDE_EXCEPTION_COMPONENT_H_

#include "robot_brain/robot_exception/robot_status.h"
#include <robot_brain/robot_exception.hpp>

#include <string>
#include <fstream>
#include <memory>
#include <mutex>
#include <vector>

#include <rtt/RTT.hpp>

namespace rosc {
class ExceptionHandle : public RTT::TaskContext {
 protected:
  RTT::InputPort<rosc::StatusCode> input_from_other_port_;
  RTT::OutputPort<rosc::StatusCode> output_to_grpc_port_;

 public:
  ExceptionHandle();
  explicit ExceptionHandle(const std::string &name);
  ~ExceptionHandle() = default;
  bool configureHook() override;
  bool startHook() override;
  void updateHook() override;
  void stopHook() override;
  void cleanupHook() override;

 private:
  void WriteToLog(rosc::StatusCode status_code);
  void ClearExceptionLog();
  std::vector<std::string> ReadExceptionLog();
  StatusCode GetCurrentExceptionCode();
  void ResetExceptionCode();

  struct Data;
  std::shared_ptr<Data> data_;
  std::string log_path_;
  StatusCode history_code_;
  std::mutex operate_log_lock_;
};
}  // namespace rosc
#endif  // MODULES_COMPONENTS_EXCEPTION_COMPONENT_INCLUDE_EXCEPTION_COMPONENT_H_
