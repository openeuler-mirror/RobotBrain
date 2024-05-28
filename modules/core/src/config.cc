/**
 * @file robot_config.cpp
 * @author your name (you@domain.com)
 * @brief 机器人配置文件相关的工具
 * @version 0.1
 * @date 2021-05-26
 *
 * @copyright Copyright (c) 2021 ROSC
 *
 */
#include <glog/logging.h>
#include <yaml-cpp/node/parse.h>
#include <yaml-cpp/null.h>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/exceptions.h>
#include <yaml-cpp/node/detail/iterator_fwd.h>
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/node/type.h>
#include <algorithm>
#include <cstdlib>
#include <string>
#include <vector>
#include <iostream>
#include <robot_brain/robot_exception.hpp>
#include <boost/filesystem.hpp>
#include <robot_brain/core.hpp>
#include <fstream>
namespace rosc {
namespace fs = boost::filesystem;

/**
 * @brief 递归加载配置文件。遍历整个文件夹，把所有yml文件都加载到配置项中
 *
 * @param curr yaml节点树。加载完之后的结果直接合并到这棵树中
 * @param dir_name 当前路径
 */
static void LoadConfigRecursive(YAML::Node *curr, const fs::path &dir_name) {
  if (!fs::exists(dir_name)) {
    LOG(ERROR) << dir_name << " is not exist.";
    return;
  }
  fs::directory_iterator end_iter;  // 无参数的是终点
  for (fs::directory_iterator iter(dir_name); iter != end_iter; ++iter) {
    if (fs::is_regular_file(iter->status())) {
      const std::string ext = iter->path().extension().string();
      if (ext != ".yml") {
        continue;
      }
      const std::string filename_no_ext = iter->path().stem().string();
      LOG(INFO) << "current path:" << fs::current_path() << ", load file "
                << iter->path();
      (*curr)[filename_no_ext] = YAML::LoadFile(iter->path().string());
    } else if (fs::is_directory(iter->status())) {
      YAML::Node node;
      const std::string dirname = iter->path().filename().string();
      LoadConfigRecursive(&node, iter->path());
      (*curr)[dirname] = node;
    }
  }
}

/**
 * @brief 构造一个对象
 */
Config::Config() : is_loaded_(false) {}

/**
 * @brief 析构一个对象
 *
 */
Config::~Config() {}

void Config::Load(const ConfigSource &source) {
  // 按照配置文件、命令行加载的顺序，加载配置
  if (!source.config_dir.empty() && source.config_dir != "") {
    LoadFromFile(source.config_dir);
  }
  if (source.argc > 1 && source.argv != nullptr) {
    LoadFromCommand(source.argc, source.argv);
  }
  DLOG(INFO) << std::endl
             << "======config content======" << std::endl
             << config_ << std::endl
             << "======config content end=====";
  this->is_loaded_ = true;
  this->loaded_source_ = source;
}

/**
 * @brief 加载配置文件夹中的所有配置
 * @param dir_name 文件夹路径
 */
void Config::LoadFromFile(const std::string &dir_name) {
  // 判断文件夹是否存在
  if (!CheckAndMkDir(dir_name.c_str())) {
    LOG(ERROR) << "dir <" << dir_name << "> not exist and create failed!";
    char str[1024];
    snprintf(str, sizeof(str), "dir <%s> not exist and create failed!",
             dir_name.c_str());
    // throw rosc::ConfigFileError(str);
  }
  // 获得配置文件路径名称，遍历路径下所有的配置文件
  fs::path base_dir(dir_name);
  LoadConfigRecursive(&config_, base_dir);  // 加载相关配置文件
}

/**
 * @brief 加载命令行的所有配置
 * 允许将例如 demo.exe --config.io.num=3 --config.tet.a = [3,2] 加载为
 * config:
 *   io:
 *     num: 3
 *   tet:
 *     a: [3,2]
 * @param argc 命令行参数个数
 * @param argv 命令行的内容详情
 */
void Config::LoadFromCommand(int argc, char **argv) {
  // 解析命令行参数
  int count = argc > 1 ? argc : 1;
  for (int i = 1; i < count; ++i) {
    const char *param_pair = argv[i];
    // 解析键值对, 参数列入--a.b.c=323
    int equ_pos = 0;
    while (param_pair[equ_pos] != '=' && param_pair[equ_pos] != '\0') {
      ++equ_pos;
    }
    if (param_pair[equ_pos] == '\0') {
      // 没有找到'='
      break;
    }
    // 判断是否为'--'或者'-'开头，如果有，则去掉
    int start = 0;
    while (start < 2 && param_pair[start] != '\0' && param_pair[start] == '-') {
      ++start;
    }
    std::string key = std::string(param_pair, start, equ_pos - start);
    // 直接去除key首尾可能存在的空格
    key.erase(0, key.find_first_not_of("\r\t\n "));
    key.erase(key.find_last_not_of("\r\t\n ") + 1);
    std::string value = std::string(&param_pair[equ_pos + 1]);
    // 解析key，并填充配置。
    YAML::Node command_node = YAML::Load(value);
    // 合并
    std::string::size_type last_pos = key.find_first_not_of('.', 0);
    std::string::size_type pos = key.find_first_of(".", last_pos);
    YAML::Node curr;
    curr = config_;
    YAML::Node parent_node;
    std::string tag;
    while (std::string::npos != pos || std::string::npos != last_pos) {
      tag = key.substr(last_pos, pos - last_pos);
      if (!curr[tag]) {
        YAML::Node child_node(YAML::NodeType::Map);
        curr[tag] = child_node;
      }
      parent_node.reset(curr);
      curr.reset(curr[tag]);
      last_pos = key.find_first_not_of('.', pos);
      pos = key.find_first_of('.', last_pos);
    }
    if (parent_node) {
      parent_node[tag] = command_node;
    }
  }
}

/**
 * @brief
 * 校验该node字段中number_tag字段（例如number）字段值与该节点中elements_tag(例如data)元素中的元素个数是否相等。
 * 主要用于一些yaml文件,
 * 这样的模式校验。校验配置信息是否正确。返回对应的number准确数字。注意，在配置文件中，允许省略number字段。
 * number: 2
 * data:
 *  - xxx
 *  - xxx
 * @param node 需要校验的node阶段
 * @param number_tag 数字标识字段开始的tag
 * @param elements_tag 元素所在区域的tag
 * @param error_location_tip
 * 如果发生错误，则错误信息附加提示信息。应该指明错误所在位置。
 * @return
 * number需要的真正值。如果有配置文件端的错误，该函数会直接assert中断程序。
 */
int Config::CheckNumberEqualDataElementsSize(
    const YAML::Node &node, const std::string &number_tag = "number",
    const std::string &elements_tag = "data",
    const std::string &error_location_tip = "") {
  unsigned int number = 0;
  if (node[number_tag].IsDefined()) {
    number = node[number_tag].as<unsigned int>();
    if (node[elements_tag].IsDefined()) {
      LOG_ASSERT(number == node[elements_tag].size())
          << std::endl
          << "Error_location_tip: " << error_location_tip << std::endl
          << number_tag << " should equal to the number of elements of "
          << elements_tag << std::endl
          << "Expect " << node[elements_tag].size() << ", actual " << number
          << ".";
    } else {
      LOG_ASSERT(number == 0)
          << std::endl
          << "Error_location_tip:" << error_location_tip << std::endl
          << " " << number_tag << " should equal to 0, or need the "
          << elements_tag << " section.";
    }
  } else {
    if (node[elements_tag].IsDefined()) {
      number = node[elements_tag].size();
    } else {
      number = 0;
    }
  }
  return number;
}

void Config::UpdateConfig(const std::string &config_item,
                          const YAML::Node &config_node) {
  if (!is_loaded_) {
    LOG(ERROR) << "config not load, can't be update!";
    return;
  }
  this->config_[config_item] = config_node;
  fs::path config_dir(this->loaded_source_.config_dir);
  fs::path file_path((config_item + ".yml").c_str());
  config_dir += file_path;
  std::ofstream file(config_dir.string());
  file << config_node;
  file.close();
}
};  // namespace rosc
