/**
 * @file test_main.cc
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief
 * @version 0.1
 * @date 2023-07-23
 *
 * @copyright Copyright (c) 2023 ROSC
 *
 */

#include <glog/logging.h>
#include <gtest/gtest.h>
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging("TEST_LOG");

  // 设置级别高于 google::INFO 的日志同时输出到屏幕
  google::SetStderrLogging(google::GLOG_INFO);
  return RUN_ALL_TESTS();
}
