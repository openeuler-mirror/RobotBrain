/**
 * @file data_manager.h
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief 数据管理器
 * @version 0.1
 * @date 2022-06-14
 *
 * @copyright Copyright (c) 2022 ROSC
 *
 */
#ifndef MODULES_CORE_INCLUDE_ROBOT_BRAIN_CORE_DATA_MANAGER_H_
#define MODULES_CORE_INCLUDE_ROBOT_BRAIN_CORE_DATA_MANAGER_H_
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/yaml.h>
#include <robot_brain/core/config.h>
#include <string>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
namespace fs = boost::filesystem;

namespace rosc {

class DataManager {
 public:
  void Load(const Config &config);
  void ReloadFromDisk();
  void DeletePoint(const std::string &point_name);
  void EmptyPoint(const std::string &point_name);
  YAML::Node GetPointByName(const std::string &point_name);
  YAML::Node GetOriginPointData();
  void UpdatePoint(const YAML::Node &point);
  bool IsExist(const std::string &point_name);
  std::string GetPointDataFilePath();

  void Save();
  DataManager();
  ~DataManager();

 private:
  fs::path data_dir_;     ///< 数据所在目录路径
  bool changed_;          ///< 是否更新
  fs::path points_file_;  ///< 点位数据文件路径
  YAML::Node points_;     ///< 点位信息
};
}  // namespace rosc
#endif  // MODULES_CORE_INCLUDE_ROBOT_BRAIN_CORE_DATA_MANAGER_H_
