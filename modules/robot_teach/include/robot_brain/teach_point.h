/**
 * @file teach_point.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2021-12-20
 *
 * @copyright Copyright (c) 2021 ROSC
 *
 */
#ifndef MODULES_ROBOT_TEACH_INCLUDE_ROBOT_BRAIN_TEACH_POINT_H_
#define MODULES_ROBOT_TEACH_INCLUDE_ROBOT_BRAIN_TEACH_POINT_H_

#include "robot_brain/command_types.hpp"
#include <bits/stdint-intn.h>
#include <robot_brain/config.h>
#include <string>
#include <kdl/frames.hpp>
#include <robot_brain/robot_planning.hpp>
#include <robot_brain/core.hpp>

namespace rosc {
#define encoder_val 1048576
typedef struct Point_Position {
  int32_t encoder[kDofMax];    // 四个轴编码器的值
  RobotPos pos;

  Point_Position(int32_t *encoder_pos) {
    for (int i = 0; i < kDofMax; i++) {
      this->encoder[i] = encoder_pos[i];
    }
    pos = RobotPos();
  }
  Point_Position() { pos = RobotPos(); }
} Point_Position;

class Teach {
 public:
  Teach();
  ~Teach();

 private:
  int dof_;
  ArmType arm_type_;

 public:
  void SavePoint(const std::string &point_name, Point_Position pos);
  Point_Position GetPoint(const std::string &point_name);
};
}  // namespace rosc
#endif  // MODULES_ROBOT_TEACH_INCLUDE_ROBOT_BRAIN_TEACH_POINT_H_
