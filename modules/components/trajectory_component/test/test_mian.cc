/**
 * @file test_mian.cc
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-01-19
 *
 * @copyright Copyright (c) 2022 ROSC
 *
 */
#include <glog/logging.h>
#include <gtest/gtest.h>
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging("TEST_LOG");
  // 设置级别高于 google::INFO 的日志同时输出到屏幕
  google::SetStderrLogging(google::INFO);
  google::SetLogDestination(google::GLOG_INFO, "./mylog.log");
  int res = RUN_ALL_TESTS();
  LOG(INFO) << "test over..." << res;

  LOG(INFO) << "jieshu.";
}
