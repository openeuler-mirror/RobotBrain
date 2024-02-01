/**
 * @file robot_scheduler.cc
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief
 * @version 0.1
 * @date 2021-09-09
 *
 * @copyright Copyright (c) 2021 ROSC
 * 机器人调度器
 */
#include <robot_component.h>
#include <robot_brain/robot_brain/robot_scheduler.h>
#include <glog/logging.h>
#include <ethercat_component.h>
#include <memory>
#include <iostream>
#include <rtt/TaskContext.hpp>
#include <rtt/os/Time.hpp>
#include <rtt/os/threads.hpp>
#include <robot_brain/core.hpp>
#include <robot_brain/ethercat_frame_types.hpp>

namespace rosc {

RobotScheduler::RobotScheduler() {}
RobotScheduler::~RobotScheduler() {}
/**
 * @brief 机器人初始化
 *
 */
void RobotScheduler::Init() {
  LOG(INFO) << "RobotScheduler::Init";
  // 构建图
  std::shared_ptr<rosc::EthercatComponent> p_ethercat =
      std::make_shared<rosc::EthercatComponent>("ethercat");
  std::shared_ptr<rosc::RobotComponent> p_robot =
      std::make_shared<rosc::RobotComponent>("robot");
  // std::shared_ptr<rosc::RobotServiceComponent> p_service =
  //     std::make_shared<rosc::RobotServiceComponent>("robot_service");
  // std::shared_ptr<rosc::GrpcServiceComponent> p_grpc_service =
  //     std::make_shared<rosc::GrpcServiceComponent>("grpc_service");

  // RTT::connectPorts(p_ethercat.get(), p_robot.get());
  // RTT::connectPeers(p_ethercat.get(), p_robot.get());

  // p_ethercat->configure();
  // p_robot->configure();
  // p_service->configure();
  // p_grpc_service->configure();

  // rtt_coms_[p_ethercat->getName()] = p_ethercat;
  // rtt_coms_[p_robot->getName()] = p_robot;
  // rtt_coms_[p_grpc_service->getName()] = p_grpc_service;

  // p_ethercat->start();
  // p_robot->start();
  // p_service->start();
  // p_grpc_service->start();
}

}  // namespace rosc
