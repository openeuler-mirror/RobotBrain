/**
 * @file traj_test.cc
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-02-17
 *
 * @copyright Copyright (c) 2022 ROSC
 *
 */
#include <chrono>
#include <gtest/gtest.h>
#include <memory>
#include <thread>
#include <unistd.h>
#include <glog/logging.h>
#include <robot_component.h>
#include <ethercat_component.h>
#include <iostream>
#include <rtt/os/Time.hpp>
#include <rtt/FlowStatus.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <rtt/TaskContext.hpp>
#include "../include/trajectory_component.h"
#include "robot_brain/kinematics.h"
#include "robot_brain/teach_point.h"
#include "robot_brain/command_types.hpp"

TEST(ServiceComponentTest, testReceiveMessage) {
  rosc::TrajectoryComponent trajectory("trajectory");
  rosc::RobotComponent robot("robot");

  RTT::connectPeers(&trajectory, &robot);

  trajectory.configureHook();
  robot.configureHook();
  // trajectory.Move_Operation(rosc::FIRST_TRANSFER_SPEED, pos1, pos2, pos3);

  // trajectory.Move_Operation(rosc::LOW_SPEED, robhome, robstart);

  // for (int i = 1; i < 16; i++) {
  //   trajectory.IO_Operation(i, true, "std::string mesg");
  //   std::this_thread::sleep_for(std::chrono::milliseconds(500));
  // }
  // for (int i = 1; i < 16; i++) {
  //   trajectory.IO_Operation(i, false, "std::string mesg");
  //   std::this_thread::sleep_for(std::chrono::milliseconds(500));
  // }

  // trajectory.GoToZero();
}
