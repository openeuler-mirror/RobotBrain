/**
 * @file test_robotmodel.cc
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-03-17
 *
 * @copyright Copyright (c) 2022 ROSC
 *
 */
#include "robot_brain/core/application.h"
#include "robot_brain/core/context.h"
#include "robot_brain/core/robot_model.h"
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <memory>
#include <robot_brain/core.hpp>

TEST(RobotCore, robotmodel) {
  rosc::RobotModel robotmodel =
      *rosc::Application::GetContext()->GetRobotModel();
  LOG(INFO) << robotmodel.GetDof();
}
