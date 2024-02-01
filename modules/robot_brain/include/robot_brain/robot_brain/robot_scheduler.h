/**
 * @file robot_scheduler.h
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief 机器人
 * @version 0.1
 * @date 2021-06-24
 *
 * @copyright Copyright (c) 2021 ROSC
 *
 */
#ifndef MODULES_ROBOT_BRAIN_INCLUDE_ROBOT_BRAIN_ROBOT_BRAIN_ROBOT_SCHEDULER_H_
#define MODULES_ROBOT_BRAIN_INCLUDE_ROBOT_BRAIN_ROBOT_BRAIN_ROBOT_SCHEDULER_H_
#include <glog/logging.h>
#include <map>
#include <iostream>
#include <memory>
#include <string>
#include <rtt/TaskContext.hpp>

namespace rosc {

// ----------------------机器人调度 RobotScheduler------------------------

/**
 * @brief 机器人调度器
 * 主要负责状态维护，底层基于RTT的模块图连接和构建
 */
class RobotScheduler {
 public:
  RobotScheduler();
  ~RobotScheduler();

  void Init();
};
}  // namespace rosc
#endif  // MODULES_ROBOT_BRAIN_INCLUDE_ROBOT_BRAIN_ROBOT_BRAIN_ROBOT_SCHEDULER_H_
