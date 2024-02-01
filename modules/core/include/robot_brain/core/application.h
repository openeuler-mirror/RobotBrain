/**
 * @file appliction_context.h
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief
 * @version 0.1
 * @date 2021-10-25
 *
 * @copyright Copyright (c) 2021 ROSC
 *
 */
#ifndef MODULES_CORE_INCLUDE_ROBOT_BRAIN_CORE_APPLICATION_H_
#define MODULES_CORE_INCLUDE_ROBOT_BRAIN_CORE_APPLICATION_H_
#include "robot_brain/core/config.h"
#include <memory>

namespace rosc {
class Context;
class Config;
/**
 * @brief 应用。管理机器人运行时的应用，包括配置初始化等，全局变量管理等。
 *
 */
class Application {
 private:
  Application();
  ~Application();
  static rosc::Context *context_;

 public:
  static void Init(const char *config_dir);
  static void Init();
  static void Init(int argc, char **argv);
  static void Init(int argc, char **argv, const char *config_dir);
  static void Init(const rosc::Config::ConfigSource &source);
  static Context *GetContext();
};

};  // namespace rosc

#endif  // MODULES_CORE_INCLUDE_ROBOT_BRAIN_CORE_APPLICATION_H_
