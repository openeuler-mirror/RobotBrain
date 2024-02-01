/**
 * @file robot_test.cc
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief 测试robot_brain
 * @version 0.1
 * @date 2021-05-26
 *
 * @copyright Copyright (c) 2021 ROSC
 *
 */
#include "robot_brain/robot_brain/robot_scheduler.h"
#include <robot_brain/robot_brain.hpp>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <string>

class robot_Test : public ::testing::Test {
 protected:
  virtual void SetUp() { LOG(INFO) << "This is robot brain test ==> SetUp!"; }

  virtual void TearDown() {
    LOG(INFO) << "This is robot brain test ==> TearDown";
  }
};
