/**
 * @file test_main.cpp
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief test_main主方法
 * @version 0.1
 * @date 2021-05-29
 *
 * @copyright Copyright (c) 2021 ROSC
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
