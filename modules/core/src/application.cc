/**
 * @file application.cc
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief
 * @version 0.1
 * @date 2021-10-26
 *
 * @copyright Copyright (c) 2021 ROSC
 *
 */
#include <robot_brain/core/context.h>
#include <robot_brain/core/application.h>

namespace rosc {

Context *Application::context_ = nullptr;

Application::Application() {}
Application::~Application() {}

void Application::Init(const Config::ConfigSource &source) {
  if (context_ == nullptr) {
    context_ = new Context();
  } else {
    delete context_;
    context_ = new Context();
  }
  context_->Init(source);
}

void Application::Init() {
  rosc::Config::ConfigSource source;
  Init(source);
}

/**
 * @brief 机器人初始化
 *
 * @param config_dir 配置文件
 */
void Application::Init(const char *config_dir) {
  rosc::Config::ConfigSource source;
  source.config_dir = config_dir;
  Init(source);
}

/**
 * @brief 机器人初始化
 *
 * @param argc 命令行参数个数（包含应用名称）
 * @param argv 命令行参数列表
 */
void Application::Init(int argc, char **argv) {
  rosc::Config::ConfigSource source;
  source.argc = argc;
  source.argv = argv;
  Init(source);
}

/**
 * @brief 机器人初始化
 *
 * @param argc 命令行参数个数（包含应用名称）
 * @param argv 命令行参数列表
 * @param config_dir 配置文件路径
 */
void Application::Init(int argc, char **argv, const char *config_dir) {
  rosc::Config::ConfigSource source;
  source.argc = argc;
  source.argv = argv;
  source.config_dir = config_dir;
  Init(source);
}

Context *Application::GetContext() {
  if (context_ != nullptr) {
    return context_;
  } else {
    // load
    Init();
    return context_;
  }
}

}  // namespace rosc
