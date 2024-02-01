/**
 * @file config_test.cc
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief 配置工具测试
 * @version 0.1
 * @date 2021-07-02
 *
 * @copyright Copyright (c) 2021 ROSC
 *
 */
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <string>
#include <iostream>
#include <robot_brain/core.hpp>

/**
 * @brief 测试从命令行中输入并生成yaml 节点。单划线等情况
 *
 */
TEST(ConfigLoadFromCommandLine_Test, LoadFromCommand) {
  rosc::Config rc;
  std::vector<std::string> argv_vec{"robotService", "--config.test.t=32233",
                                    "--config.test.j = 46",
                                    "-config.test.arr = [3, 4,1] "};
  int argc = argv_vec.size();
  char **argv = new char *[argc];
  for (int i = 0; i < argc; i++) {
    // argv[i] = argv_vec[i].c_str();
    strcpy(argv[i], argv_vec[i].c_str());
  }
  rc.LoadFromCommand(argc, argv);
  ASSERT_TRUE(rc["config"].IsDefined());
  ASSERT_EQ(rc["config"]["test"]["t"].as<int>(), 32233);
  ASSERT_EQ(rc["config"]["test"]["j"].as<int>(), 46);
  ASSERT_EQ(rc["config"]["test"]["arr"][1].as<int>(), 4);
  delete[] argv;
}

/**
 * @brief 测试从命令行中加载配置，空格等情况
 *
 */
TEST(ConfigLoadFromCommandLine_Test, LoadFromCommand2) {
  rosc::Config rc;
  std::vector<std::string> argv_vec{"robotService", "--config.io.num=3",
                                    "-- config.tet.a = [3,2]"};

  int argc = argv_vec.size();
  char **argv = new char *[argc];
  for (int i = 0; i < argc; i++) {
    // argv[i] = argv_vec[i].c_str();
    strcpy(argv[i], argv_vec[i].c_str());
  }
  rc.LoadFromCommand(argc, argv);
  ASSERT_TRUE(rc["config"].IsDefined());
  ASSERT_EQ(rc["config"]["io"]["num"].as<int>(), 3);
  ASSERT_EQ(rc["config"]["tet"]["a"][0].as<int>(), 3);
  delete[] argv;
}

class RobotConfigTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    LOG(INFO) << "This is robot config test ==> SetUp!";
    rosc::Config::ConfigSource source;
    source.config_dir = "../../../../../data/test_data/resources/";
    rc_.Load(source);
  }

  virtual void TearDown() {
    LOG(INFO) << "This is robot config test ==> TearDown";
  }

 protected:
  rosc::Config rc_;
};

/**
 * @brief 测试从文件中直接进行配置
 *
 */
TEST_F(RobotConfigTest, Load) {
  // 测试不同层次的属性文件加载情况
  ASSERT_EQ(rc_["test_2layers"]["test2lConfig"]["test2t"]["a"].as<int>(), 2);
  ASSERT_EQ(rc_["test_2layers"]["test_3layers"]["test1"]["t"].as<int>(), 2);
  // 测试只加载yml后缀文件
  ASSERT_EQ(rc_["test_2layers"]["test_3layers"].size(), 1);
}
