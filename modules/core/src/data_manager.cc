/**
 * @file data_manager.cc
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief
 * @version 0.1
 * @date 2022-06-14
 *
 * @copyright Copyright (c) 2022 ROSC
 *
 */
#include <robot_brain/core/data_manager.h>
#include <robot_brain/robot_exception/robot_error.h>
#include <robot_brain/robot_exception/robot_status.h>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
namespace rosc {

/**
 * @brief 递归更新点位
 *
 * @param old_node 原有节点
 * @param replace_node 用于更新的节点
 */
static void UpdateNode(YAML::Node *old_node, const YAML::Node &replace_node) {
  for (YAML::const_iterator it = replace_node.begin(); it != replace_node.end();
       ++it) {
    if ((*old_node)[it->first.as<std::string>()].Type() ==
        YAML::NodeType::Null) {
      (*old_node)[it->first.as<std::string>()] = it->second;
    } else if ((*old_node)[it->first.as<std::string>()].Type() !=
                   YAML::NodeType::Map ||
               it->second.Type() != YAML::NodeType::Map) {
      (*old_node)[it->first.as<std::string>()] = it->second;
    } else {
      YAML::Node sub_old_node = (*old_node)[it->first.as<std::string>()];
      YAML::Node sub_replace_node = replace_node[it->first.as<std::string>()];
      UpdateNode(&sub_old_node, sub_replace_node);
    }
  }
}

/**
 * @brief 加载数据
 *
 * @param data_dir 数据加载路径
 */
void DataManager::Load(const Config &config) {
  // 判断文件夹是否存在
  this->data_dir_ = fs::path(
      config["init_global_config"]["data"]["root_path"].as<std::string>());
  if (!fs::is_directory(this->data_dir_)) {
    LOG(ERROR) << "Load Error! <" << this->data_dir_ << "> is not direcotry!";
    throw RobotException(StatusCode::kDataFileLoadError);
  }

  // 加载点位文件
  this->points_file_ =
      this->data_dir_ /
      config["init_global_config"]["data"]["files_path"]["points_file"]
          .as<std::string>();
  if (!boost::filesystem::is_regular_file(this->points_file_)) {
    LOG(ERROR) << "load file " << this->points_file_ << "error!";
    throw RobotException(StatusCode::kDataFileLoadError);
  }
  this->points_ = YAML::LoadFile(this->points_file_.string());
}

/**
 * @brief 从硬盘中重新加载数据文件
 *
 */
void DataManager::ReloadFromDisk() {
  // 判断文件夹是否存在
  if (!fs::is_directory(this->data_dir_)) {
    LOG(ERROR) << "Load Error! <" << this->data_dir_ << "> is not direcotry!";
    throw RobotException(StatusCode::kDataFileLoadError);
  }
  if (!boost::filesystem::is_regular_file(this->points_file_)) {
    LOG(ERROR) << "load file " << this->points_file_ << "error!";
    throw RobotException(StatusCode::kDataFileLoadError);
  }
  this->points_ = YAML::LoadFile(this->points_file_.string());
}

/**
 * @brief 保存数据
 *
 */
void DataManager::Save() {
  if (this->data_dir_.empty()) {
    return;
  }
  if (!this->points_file_.empty()) {
    LOG(INFO) << "Save data for points files. path <" << this->points_file_
              << ">";
    std::ofstream fout(points_file_.string());
    fout << this->points_;
    fout.close();
  }
  this->changed_ = false;
}

DataManager::DataManager() : changed_(false), data_dir_("") {}

DataManager::~DataManager() {
  if (this->changed_) {
    LOG(WARNING) << "Data not saved, have lost. dir: " << this->data_dir_;
    return;
  }
}

/**
 * @brief 删除机器人点位数据
 *
 * @param point_name 点位名称
 */
void DataManager::DeletePoint(const std::string &point_name) {
  this->points_.remove(point_name);
}

/**
 * @brief 点位数据清零
 *
 * @param point_name 点位名称
 */
void DataManager::EmptyPoint(const std::string &point_name) {
  YAML::Node point = this->points_[point_name];
  for (auto item : point) {
    for (auto i = 0; i < point[item.first].size(); ++i) {
      point[item.first][i] = 0.0;
    }
  }
  this->points_[point_name] = point;
}

/**
 * @brief 根据点名获取点位数据
 *
 * @param point_name 点位名称
 */
YAML::Node DataManager::GetPointByName(const std::string &point_name) {
  return this->points_[point_name];
}

/**
 * @brief 获取原始数据
 *
 * @return YAML::Node 点位配置原始数据
 */
YAML::Node DataManager::GetOriginPointData() { return this->points_; }

/**
 * @brief 保存或者更新机器人点位数据
 *
 * @param point yaml节点树。需要保存或者更新的点位数据
 */
void DataManager::UpdatePoint(const YAML::Node &point) {
  UpdateNode(&this->points_, point);
}

/**
 * @brief 判断数据点是否存在
 *
 * @param point_name 点位名称
 */
bool DataManager::IsExist(const std::string &point_name) {
  return points_[point_name].IsDefined();
}

std::string DataManager::GetPointDataFilePath() {
  return points_file_.string();
}

}  // namespace rosc
