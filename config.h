/**
 * @file config.h
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief robot_brain库的配置相关头文件
 * @details config.h为库编译的相关配置，该文件为自动生成文件，不要手动编辑，如有需要，可以对应的config.h.in文件
 * @version 0.1
 * @date 2021-05-27
 *
 * @copyright Copyright (c) 2021 ROSC
 *
 */
#ifndef ROBOT_BRAIN_CONFIG_H_
#define ROBOT_BRAIN_CONFIG_H_

#define ROSC_ROBOT_TYPE ROBOT_TYPE_ZERO_6

#define ROBOT_TYPE_ZERO_6


typedef enum SlaveType {
  MOTOR_SERVO,
  IO_SERVO,
  IO_SERVO_IN,
  IO_SERVO_OUT
} SlaveType;

typedef enum ArmType { ZERO_6, SCARA, XB4S } ArmType;

const int kDofMax = 6;
const int kSLAVENUMMAX = 9;

#ifdef ROBOT_TYPE_ZERO_6
const int kDof = 6;
#endif

#ifdef ROBOT_TYPE_PANASONIC_2_SLAVE
const int kSLAVENUM = 2;
const int kDof = 2;

#endif

#ifdef ROBOT_TYPE_ROKAE_XB4S
const ArmType ARMTYPE = ArmType::XB4S;
const int kSLAVENUM = 6;
const int kDof = 6;
#endif

// const char *const kDefaultDataPointFilePath =
//     "../../../../../../data/test_data/robot/data/POINT.yml";
// "/robot/config/data/POINT.yml";

#endif  // ROBOT_BRAIN_CONFIG_H_
