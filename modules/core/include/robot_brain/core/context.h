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
#ifndef MODULES_CORE_INCLUDE_ROBOT_BRAIN_CORE_CONTEXT_H_
#define MODULES_CORE_INCLUDE_ROBOT_BRAIN_CORE_CONTEXT_H_
#include <robot_brain/core/robot_model.h>
#include <robot_brain/core/data_manager.h>
#include <memory>

// #include <robot_brain/robot_kdl.hpp>
namespace rosc {
/**
 * @brief 上下文
 *
 */
class Context {
 public:
  Context();
  Context(Context &&) = default;
  Context(const Context &) = default;
  Context &operator=(Context &&) = default;
  Context &operator=(const Context &) = default;
  void Init(const rosc::Config::ConfigSource &source);
  ~Context();

  std::shared_ptr<Config> GetConfig();
  std::shared_ptr<DataManager> GetDataManager();
  std::shared_ptr<RobotModel> GetRobotModel();

 private:
  std::shared_ptr<Config> config_;
  std::shared_ptr<RobotModel> robot_model_;
  std::shared_ptr<DataManager> data_manager_;
};
}  // namespace rosc
#endif  // MODULES_CORE_INCLUDE_ROBOT_BRAIN_CORE_CONTEXT_H_
