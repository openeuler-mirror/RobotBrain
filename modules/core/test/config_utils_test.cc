/**
 * @file config_tools_test.cpp
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief 测试config_tools
 * @version 0.1
 * @date 2021-05-26
 *
 * @copyright Copyright (c) 2021 ROSC
 *
 */
#include <robot_brain/core/config_utils.h>
#include <robot_brain/core/config.h>
#include <dirent.h>
#include <gtest/gtest.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <random>
#include <sstream>
#include <string>

/**
 * @brief 测试当文件目录不存在的情况下建立文件目录
 */
TEST(CheckAndMkDirTest, dir_not_exist) {
  // 随意建一个文件夹
  std::stringstream ss;
  std::string rand_str;
  ss << std::rand();
  ss >> rand_str;
  std::string test_dir_name =
      "for_test_dir_not_exist_"
      "testxomfdjenxikenxnkdoktxlltlktTxolsdjtxlldnffdslt" +
      rand_str;
  DIR *dp;
  dp = opendir(test_dir_name.c_str());
  if (dp == NULL) {
    bool res = CheckAndMkDir(test_dir_name.c_str());
    ASSERT_EQ(res, true);
    // 如果存在，则删除。
    ASSERT_EQ(rmdir(test_dir_name.c_str()), 0);
  } else {
    closedir(dp);
  }
}

/**
 * @brief 测试当文件目录存在的情况下建立新的文件目录
 */
TEST(CheckAndMkDirTest, dir_exist) {
  std::stringstream ss;
  std::string rand_str;
  ss << std::rand();
  ss >> rand_str;
  // 随意建一个文件夹
  std::string test_dir_name =
      "for_test_dir_exist_testxomfdjenxikenxnkdoktxlltlktTxolsdjtxlldnffdslt" +
      rand_str;
  DIR *dp;
  bool res = CheckAndMkDir(test_dir_name.c_str());
  ASSERT_EQ(res, true);
  dp = opendir(test_dir_name.c_str());
  ASSERT_TRUE(dp != NULL);
  closedir(dp);
  // 删除临时生成的文件
  ASSERT_EQ(rmdir(test_dir_name.c_str()), 0);
}

/**
 * @brief 测试绝对路径，多级路径
 */
TEST(CheckAndMkDirTest, dir_absolute_path) {
  std::stringstream ss;
  std::string rand_str;
  ss << std::rand();
  ss >> rand_str;
  // 获得当前的路径
  char buf[256];
  getcwd(buf, sizeof(buf));
  // 随意建一个文件夹
  std::string test_dir_name =
      std::string(buf) +
      "/for_test_dir_exist_testxomfdjenxikenxnkdoktxlltlktTxolsdjtxlldnffdslt" +
      rand_str;
  DIR *dp;
  bool res = CheckAndMkDir(test_dir_name.c_str());
  ASSERT_EQ(res, true);
  dp = opendir(test_dir_name.c_str());
  ASSERT_TRUE(dp != NULL);
  closedir(dp);
  // 删除临时生成的文件
  ASSERT_EQ(rmdir(test_dir_name.c_str()), 0);
}
