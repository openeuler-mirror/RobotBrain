/**
 * @file robot_server_test.cpp
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief 测试robot_server文件中的方法和类
 * @version 0.1
 * @date 2021-05-25
 *
 * @copyright Copyright (c) 2021
 *
 */
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <string>

// 引入cpp，是为了对static函数进行测试

class robot_server_Test : public ::testing::Test {
 protected:
  virtual void SetUp() { LOG(INFO) << "This is robot ctl test ==> SetUp!"; }

  virtual void TearDown() {
    LOG(INFO) << "This is robot ctl test ==> TearDown";
  }
};
