/**
 * @file config_file_exception.h
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief 配置文件相关的异常
 * @version 0.1
 * @date 2021-06-22
 *
 * @copyright Copyright (c) 2021 ROSC
 *
 */
#ifndef MODULES_ROBOT_EXCEPTION_INCLUDE_ROBOT_BRAIN_ROBOT_EXCEPTION_ROBOT_ERROR_H_
#define MODULES_ROBOT_EXCEPTION_INCLUDE_ROBOT_BRAIN_ROBOT_EXCEPTION_ROBOT_ERROR_H_

#include <robot_brain/robot_exception/robot_status.h>
#include <string>
#include <stdexcept>
namespace rosc {

class RobotException : public std::runtime_error {
 public:
  explicit RobotException(StatusCode code)
      : std::runtime_error(code_info_map[code]), code_(code) {}

  ~RobotException() = default;

  StatusCode GetStatusCode() { return code_; }

 private:
  StatusCode code_;
};

}  // namespace rosc

#endif  // MODULES_ROBOT_EXCEPTION_INCLUDE_ROBOT_BRAIN_ROBOT_EXCEPTION_ROBOT_ERROR_H_
