/**
 * @file robot_server_main.cc
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief 作为常规的机器人控制器服务器，提供服务．
 * @version 0.1
 * @date 2021-05-25
 *
 * @copyright Copyright (c) 2021
 *
 */
#include <rtt/os/main.h>
#include <glog/logging.h>
#include <robot_brain/robot_brain/robot_scheduler.h>
#include <iostream>
#include <memory>
#include <string>
#include <robot_brain/core.hpp>

int ORO_main(int argc, char **argv) {
  // 初始化google log
  google::InitGoogleLogging("serverLog");
#ifdef NDEBUG
  // 设置级别高于 google::FATAL 的日志同时输出到屏幕
  google::SetStderrLogging(google::GLOG_WARNING);
#else
  // 设置级别高于 google::INFO 的日志同时输出到屏幕
  google::SetStderrLogging(google::GLOG_INFO);
#endif
  // 初始化机器人系统
  rosc::Application::Init(argc, argv);

  rosc::RobotScheduler scherduler;
  scherduler.Init();
  while (true) {
  }
  return 0;
}
