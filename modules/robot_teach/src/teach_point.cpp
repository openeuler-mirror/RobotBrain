/**
 * @file teach_point.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2021-12-20
 *
 * @copyright Copyright (c) 2021 ROSC
 *
 */
#include <bits/stdint-intn.h>
#include <kdl/utilities/utility.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
#include <robot_brain/config.h>
#include <robot_brain/teach_point.h>
#include <robot_brain/core/application.h>
#include <robot_brain/robot_planning.hpp>
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/node/parse.h>
#include <cstring>
#include <fstream>
#include <memory>
#include <iostream>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <robot_brain/command_types.hpp>

namespace rosc {

/**
 * @brief Construct a new Teach:: Teach object
 *
 */
Teach::Teach() {
  auto config = *Application::GetContext()->GetConfig();
  int arm_tp = config["init_global_config"]["robot_type"].as<int>();
  switch (arm_tp) {
  case 0:
    this->arm_type_ = ZERO_6;
    this->dof_ =
        config["zero_device_config"]["zero_device_config"]["robot"]["dof"]
            .as<int>();
    break;
  case 1:
    this->arm_type_ = SCARA;
    this->dof_ =
        config["scara_device_config"]["scara_device_config"]["robot"]["dof"]
            .as<int>();
    break;
  default:
    this->arm_type_ = XB4S;
    break;
  }
}

Teach::~Teach() {}

/**
 * @brief 保存目标点
 *
 * @param point_name
 * @param pos
 */
void Teach::SavePoint(const std::string &point_name, Point_Position pos) {
  // YAML::Node point = YAML::LoadFile(this->point_profile_);
  auto data_manager = rosc::Application::GetContext()->GetDataManager();
  YAML::Node point = data_manager->GetOriginPointData();
  double roll, pitch, yaw;
  for (int i = 0; i < this->dof_; ++i) {
    point[point_name]["encoder"][i] = pos.encoder[i];
  }
  for (int i = 0; i < this->dof_; ++i) {
    point[point_name]["joint"][i] = pos.pos.joint(i);
  }

  point[point_name]["cartesian"][0] = pos.pos.cartesian.p.x();
  point[point_name]["cartesian"][1] = pos.pos.cartesian.p.y();
  point[point_name]["cartesian"][2] = pos.pos.cartesian.p.z();

  pos.pos.cartesian.M.GetRPY(roll, pitch, yaw);
  point[point_name]["cartesian"][3] = roll;
  point[point_name]["cartesian"][4] = pitch;
  point[point_name]["cartesian"][5] = yaw;

  data_manager->UpdatePoint(point);
  data_manager->Save();
}

/**
 * @brief 从文件中获取点位
 *
 * @param point_name
 * @return Point_Position
 */
Point_Position Teach::GetPoint(const std::string &point_name) {
  std::cout << "Read point: " << point_name << std::endl;
  YAML::Node point_node =
      rosc::Application::GetContext()->GetDataManager()->GetOriginPointData();
  int32_t encoder[kDofMax];
  if (strstr(point_name.c_str(), "ZERO") != nullptr) {
    for (int i = 0; i < this->dof_; i++) {
      encoder[i] = point_node[point_name]["encoder"][i].as<int32_t>();
    }
  }

  Point_Position res(encoder);
  double roll, pitch, yaw;

  for (int i = 0; i < (this->dof_); ++i) {
    res.pos.joint(i) = point_node[point_name]["joint"][i].as<double>();
  }

  // cartesian
  res.pos.cartesian.p.data[0] =
      point_node[point_name]["cartesian"][0].as<double>();
  res.pos.cartesian.p.data[1] =
      point_node[point_name]["cartesian"][1].as<double>();
  res.pos.cartesian.p.data[2] =
      point_node[point_name]["cartesian"][2].as<double>();
  roll = point_node[point_name]["cartesian"][3].as<double>();
  pitch = point_node[point_name]["cartesian"][4].as<double>();
  yaw = point_node[point_name]["cartesian"][5].as<double>();
  res.pos.cartesian.M = KDL::Rotation::RPY(roll, pitch, yaw);

  return res;
}

}  // namespace rosc
