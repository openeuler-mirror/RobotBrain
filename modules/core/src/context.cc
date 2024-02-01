/**
 * @file context.cc
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief
 * @version 0.1
 * @date 2021-10-26
 *
 * @copyright Copyright (c) 2021 ROSC
 *
 */
#include "robot_brain/config.h"
#include "robot_brain/core/config.h"
#include "robot_brain/robot_exception/robot_error.h"
#include "robot_brain/robot_exception/robot_status.h"
#include <robot_brain/core/context.h>
#include <robot_brain/core/robot_model.h>
#include <glog/logging.h>
#include <memory>
namespace rosc {
Context::Context() : config_(nullptr) {}

/**
 * @brief 根据配置文件的设置，加载配置并初始化
 *
 * @param source 配置文件的源
 */
void Context::Init(const rosc::Config::ConfigSource &source) {
  // 配置
  config_ = std::make_shared<Config>();
  config_->Load(source);
  // 加载模型
  int arm_tp = (*config_)["init_global_config"]["robot_type"].as<int>();
  switch (arm_tp) {
  case 0:
    this->robot_model_ =
        std::make_shared<RobotModelCobot>((*config_)["zero_device_config"]);
    this->robot_model_->arm_type_ = ZERO_6;
    break;
  case 1:
    this->robot_model_ =
        std::make_shared<RobotModelScara>((*config_)["scara_device_config"]);
    this->robot_model_->arm_type_ = SCARA;
    break;
  default:
    break;
  }

  data_manager_ = std::make_shared<DataManager>();
  data_manager_->Load(*config_);
}

Context::~Context() {}

/**
 * @brief 获得配置
 *
 * @return std::shared_ptr<Config> 配置
 */
std::shared_ptr<Config> Context::GetConfig() { return config_; }

/**
 * @brief 获得机器人模型
 *
 * @return std::shared_ptr<RobotModel> 机器人模型
 */
std::shared_ptr<RobotModel> Context::GetRobotModel() { return robot_model_; }

/**
 * @brief 获得机器人数据管理器
 *
 * @return std::shared_ptr<DataManager>
 */
std::shared_ptr<DataManager> Context::GetDataManager() {
  return this->data_manager_;
}
};  // namespace rosc
