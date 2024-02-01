/**
 * @file robot_component_test.cc
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief
 * @version 0.1
 * @date 2021-10-21
 *
 * @copyright Copyright (c) 2021 ROSC
 *
 */

#include <kdl/utilities/utility.h>
#include <math.h>
#include <robot_component.h>
#include <ethercat_component.h>
#include <bits/stdint-uintn.h>
#include <dirent.h>
#include <gtest/gtest.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>
#include <glog/logging.h>
#include <random>
#include <sstream>
#include <iostream>
#include <string>
#include <memory>
#include <iterator>
#include <rtt/os/Time.hpp>
#include <rtt/os/threads.hpp>
#include <rtt/FlowStatus.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <rtt/TaskContext.hpp>
#include <robot_brain/core.hpp>
#include <robot_brain/ethercat_frame_types.hpp>
#include "robot_brain/core/application.h"
#include "robot_brain/core/robot_model.h"
#include "robot_brain/core/robot_types.hpp"
#include <robot_brain/robot_planning.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/joint.hpp>
#include <robot_brain/robot_teach.hpp>
/**
 * @brief 测试moveabsj功能在
 *
 */
TEST(RobotComponentTest, test_moveabsj_in_simulation) {
  // rosc::EthercatComponent ec("ethercat");
  LOG(INFO) << "start";
  rosc::RobotComponent robot("robot");
}
