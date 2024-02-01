/**
 * @file command_frame_types.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-02-22
 *
 * @copyright Copyright (c) 2022 ROSC
 *
 */
#include <sstream>
#include <vector>
#include <string>
#include <robot_brain/command_types.hpp>

#ifndef MODULES_CORE_INCLUDE_ROBOT_BRAIN_CORE_COMMAND_FRAME_TYPES_HPP_
#define MODULES_CORE_INCLUDE_ROBOT_BRAIN_CORE_COMMAND_FRAME_TYPES_HPP_

namespace rosc {
struct CompleteCommandEvent {
  std::string msg;
  std::vector<std::string> responseParams;
  char errcode[EXEC_RESULT_ERROR_CODE_LEN];  ///< 错误码
  CompleteCommandEvent &
  operator=(const CompleteCommandEvent &completeCommandEvent) {
    this->msg = completeCommandEvent.msg;
    this->responseParams.assign(completeCommandEvent.responseParams.begin(),
                                completeCommandEvent.responseParams.end());
    memcpy(errcode, completeCommandEvent.errcode, EXEC_RESULT_ERROR_CODE_LEN);
    return *this;
  }
};

struct ReceiveMessage {
  std::vector<char> msg;
  char uno;
  std::string cmd;
  std::vector<std::string> params;
  ReceiveMessage &operator=(const ReceiveMessage &receiveMessage) {
    this->msg = receiveMessage.msg;
    this->uno = receiveMessage.uno;
    this->cmd = receiveMessage.cmd;
    this->params.assign(receiveMessage.params.begin(),
                        receiveMessage.params.end());
    return *this;
  }
};
}  // namespace rosc

#endif  // MODULES_CORE_INCLUDE_ROBOT_BRAIN_CORE_COMMAND_FRAME_TYPES_HPP_
