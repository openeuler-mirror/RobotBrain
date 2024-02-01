/**
 * @file config.h
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief 机器人配置
 * @version 0.1
 * @date 2021-05-26
 *
 * @copyright Copyright (c) 2021 ROSC
 *
 */
#ifndef MODULES_CORE_INCLUDE_ROBOT_BRAIN_CORE_CONFIG_H_
#define MODULES_CORE_INCLUDE_ROBOT_BRAIN_CORE_CONFIG_H_
#include "default_values.h"
#include <yaml-cpp/yaml.h>
#include <map>
#include <string>

namespace rosc {
/**
 * @brief 机器人配置文件，辅助完成读取、加载等内容
 * 配置加载会从命令行、配置文件两处，按照顺序进行加载。后加载的内容会覆盖先加载的内容。
 */
class Config {
 public:
  /**
   * @brief 记录配置信息加载的位置和传入的原始信息
   * 支持配置文件和命令行两种参数加载
   */
  struct ConfigSource {
    int argc = 0;                                ///< 命令行参数个数
    char **argv = nullptr;                       ///< 命令行参数列表
    std::string config_dir = kDefaultConfigDir;  ///< 配置文件路径
  };

  Config();
  ~Config();
  void Load(const ConfigSource &source);

  void UpdateConfig(const std::string &config_item,
                    const YAML::Node &config_node);

  template <typename Key> const YAML::Node operator[](const Key &key) const;
  template <typename Key> YAML::Node operator[](const Key &key);

 private:
  void LoadFromCommand(int argc, char **argv);
  void LoadFromFile(const std::string &dir_name);
  // utils
  int CheckNumberEqualDataElementsSize(const YAML::Node &node,
                                       const std::string &number_tag,
                                       const std::string &elements_tag,
                                       const std::string &error_location_tip);

 private:
  YAML::Node config_;  ///< 全部配置文件
  ConfigSource loaded_source_;  ///< 记录配置文件加载的原信息
  bool is_loaded_;              ///< 是否已经加载
};

/**
 * @brief 获取配置具体的条目信息
 *
 * @param key 关键索引值。
 * @return YAML::Node 对应节点
 */
template <typename Key>
const YAML::Node Config::operator[](const Key &key) const {
  return config_[key];
}

/**
 * @brief 获取配置具体的条目信息
 *
 * @param key 关键索引值。
 * @return YAML::Node 对应节点
 */
template <typename Key> YAML::Node Config::operator[](const Key &key) {
  return config_[key];
}

};      // namespace rosc
#endif  // MODULES_CORE_INCLUDE_ROBOT_BRAIN_CORE_CONFIG_H_
