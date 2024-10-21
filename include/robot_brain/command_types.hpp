/**
 * @file command_types.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-03-17
 *
 * @copyright Copyright (c) 2022 ROSC
 *
 */
#ifndef ROBOT_BRAIN_COMMAND_TYPES_HPP_
#define ROBOT_BRAIN_COMMAND_TYPES_HPP_

#include <robot_brain/config.h>
#include <bits/stdint-intn.h>
#include <bits/stdint-uintn.h>
#include <string>
#include <vector>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
namespace rosc {

#define COMMANDLIST_MAXLEN 30
#define COMMAND_MESG_MAXLEN 100
#define EXEC_RESULT_ERROR_CODE_LEN 4

typedef struct Arm_pos {
  double Extension;
  double Rotation;
  double Z_axis;
} Arm_pos;

// 坐标结构体
typedef struct RobotPos {
  KDL::JntArray joint;
  KDL::Frame cartesian;
  RobotPos() { joint = KDL::JntArray(kDofMax); }
} RobotPos;

/**
 * @brief 指令类型
 *
 */
typedef enum CommandType {
  MOTION,             ///< 点位运动指令
  MOVE_STEP,          ///< 步长运动
  IO_IN,              ///< IO读操作
  IO_OUT,             ///< IO写操作
  POWER_ON,           ///< 上电指令
  POWER_OFF,          ///< 下电指令
  JOG_START,          ///< 单轴点动开始
  POINT_JOG_START,    ///< jog移动到目标点
  JOG_STOP,           ///< 点动停止
  CLEAR,              ///< 清除错误
  EMERGENCY_STOP,     ///< 急停
  EMERGENCY_RECOVER,  ///< 急停恢复
  POINT,              ///< 点位保存
  MOVEZONE,           ///< 转弯区运动
  MOVETO,             ///< 移动到指定点位，最小旋转半径
  MOVETOCART,         ///< 移动到指定笛卡尔位置
  MOVEL,              ///< 两点之间笛卡尔运动
  SSPD,               ///< 设置运动速度
  SSLV,               ///< 设置速度等级
  MHOM,               ///< 运动到home点
  NOP,                ///< 空指令，停顿
  FAULT               ///< 错误指令
} CommandType;

typedef enum TransferSpeedType {
  FIRST_TRANSFER_SPEED = 0,     // 第一转移速度
  SECOND_TRANSFER_SPEED = 1,    // 第二转移速度
  LOW_SPEED = 2,                // 低速
  HOME_SPEED = 3,               // 返回home速度
  SPEED_IN_LOW_SPEED_AREA = 4,  // 低速区域速度
  JOG_SPEED = 5,
  INCHING_SPEED = 6,
  LINEAR_MOTION_FIRST_SPEED = 7,
  LINEAR_MOTION_SECOND_SPEED = 8,
  LINEAR_MOTION_LOW_SPEED = 9,
  LINEAR_MOTION_HOME_SPEED = 10,
  LINEAR_MOTION_JOG_SPEED = 11,
  LINEAR_MOTION_INCHING_SPEED = 12
} TransferSpeedType;

// 机器人轴序号枚举
typedef enum AxisNum {
  Elevation = 0,
  Rotation = 1,
  Extension_L = 2,
  Extension_R = 3,
  Extension = 4,
  Wrist_L = 5,
  Wrist_R = 6,
  Cartesian_X_L = 7,
  Cartesian_X_R = 8,
  Cartesian_Y_L = 9,
  Cartesian_Y_R = 10,
  Flip_L = 11,
  Flip_R = 12
} AxisNum;

/*
 TODO(QingfengLi): 采用内置联合体对命令参数进行表达,
 重构指令结构体，减少内存占用，同时提高可读性
 */
/**
 * @brief 机械臂控制指令
 */
typedef struct SingleCommand {
  RobotPos start;
  RobotPos end;

  double speedData;     ///< 速度值
  double jogspeed;      ///< 点动速度
  double jogstep;       ///< 点动步长
  int64_t timestamp;    ///< 时间戳
  int whichIO;          ///< IO编号
  int direction;        ///< 点动方向
  int speedLevel = -1;  ///< 速度等级
  uint32_t speedPersent;
  int milliseconds;         ///< nop 时间
  CommandType op_type;      ///< 动作类型
  AxisNum whichAxis;        ///< 点动轴
  TransferSpeedType speed;  ///< sr100指令速度类型
  char speedType;           ///< 速度类型
  char speedAxis;           ///< 哪一个关节的速度
  char mesg[COMMAND_MESG_MAXLEN] = "normal command";  ///< 信息
  char point_name[COMMAND_MESG_MAXLEN];               ///< 点位名称

  bool io_op;                     ///< IO操作
  bool points_3_zone = false;     ///< 是否插补
  bool is_retry_command = false;  ///< 是否为重试指令

  SingleCommand &operator=(const SingleCommand &singleCommand) {
    this->start = singleCommand.start;
    this->end = singleCommand.end;
    this->points_3_zone = singleCommand.points_3_zone;
    this->speed = singleCommand.speed;
    memcpy(this->mesg, singleCommand.mesg, sizeof(singleCommand.mesg));
    memcpy(this->point_name, singleCommand.point_name,
           sizeof(singleCommand.point_name));

    this->op_type = singleCommand.op_type;
    this->whichIO = singleCommand.whichIO;
    this->io_op = singleCommand.io_op;
    // this->io_value = singleCommand.io_value;
    this->whichAxis = singleCommand.whichAxis;
    this->direction = singleCommand.direction;
    this->jogspeed = singleCommand.jogspeed;
    this->timestamp = singleCommand.timestamp;
    this->jogstep = singleCommand.jogstep;
    this->speedData = singleCommand.speedData;
    this->speedLevel = singleCommand.speedLevel;
    this->speedType = singleCommand.speedType;
    this->speedAxis = singleCommand.speedAxis;
    this->milliseconds = singleCommand.milliseconds;
    this->is_retry_command = singleCommand.is_retry_command;
    return *this;
  }
} SingleCommand;

/********************************************************************
 * @brief 错误指令，直接返回错误的情况
 *
 ********************************************************************/
typedef struct FaultCommand : SingleCommand {
  FaultCommand() { op_type = CommandType::FAULT; }
} FaultCommand;

/**
 * @brief Poweron 指令
 *
 */
typedef struct PoweronCommand : SingleCommand {
  PoweronCommand() { op_type = CommandType::POWER_ON; }
} PoweronCommand;

/**
 * @brief Power off 指令
 *
 */
typedef struct PoweroffCommand : SingleCommand {
  PoweroffCommand() { this->op_type = CommandType::POWER_OFF; }
} PoweroffCommand;

typedef struct PointCommand : SingleCommand {
  PointCommand() { this->op_type = CommandType::POINT; }
  void SetParam() {}
} PointCommand;

/**
 * @brief clear error 指令
 *
 */
typedef struct ClearCommand : SingleCommand {
  ClearCommand() { this->op_type = CommandType::CLEAR; }
} ClearCommand;

/**
 * @brief
 *
 */
typedef struct IOCommand : SingleCommand {
  IOCommand() {
    this->op_type = CommandType::IO_OUT;
    // std::memset(&this->io_value, 0, sizeof(this->io_value));
  }
  void SetParam(int whichIO, bool op) {
    this->whichIO = whichIO;
    this->io_op = op;
  }
  void SetMesg(std::string mesg) {
    std::snprintf(this->mesg, sizeof(this->mesg), "%s", mesg.c_str());
  }
} IOCommad;

typedef struct IO_INCommand : SingleCommand {
  IO_INCommand() {
    this->op_type = CommandType::IO_IN;
    // std::memset(&this->io_value, 0, sizeof(this->io_value));
  }
  void SetParam(int whichIO, bool op) {
    this->whichIO = whichIO;
    this->io_op = op;
  }
  void SetMesg(std::string mesg) {
    std::snprintf(this->mesg, sizeof(this->mesg), "%s", mesg.c_str());
  }
} IO_INCommand;

typedef struct NOPCommand : SingleCommand {
  NOPCommand() { this->op_type = CommandType::NOP; }
} NOPCommand;

/**
 * @brief 指令执行的状态
 *
 */
typedef enum ExecStatus {
  // RECEIVED,  ///< 已经收到
  SUCCESS,  ///< 执行成功
  FAILED,   ///< 执行失败
  // ABORT      ///< 中止执行
  TIMEOUT  ///< 执行超时
} ExecStatus;

/**
 * @brief 指令执行的结果状态及内容
 *
 */
typedef struct CommandExecResult {
  ExecStatus status;                         ///< 执行状态
  CommandType tp;                            ///< 回复指令类型
  int64_t timestamp;                         ///< 执行命令时间戳
  std::vector<std::string> responseParams;   ///< 返回执行结果
  char errcode[EXEC_RESULT_ERROR_CODE_LEN];  ///< 错误码
  CommandExecResult() {}
} CommandExecResult;

};  // namespace rosc

#endif  // ROBOT_BRAIN_COMMAND_TYPES_HPP_
