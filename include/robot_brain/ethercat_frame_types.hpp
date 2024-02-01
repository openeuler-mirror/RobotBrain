/**
 * @file ethercat_frame.hpp
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief Ethercat 发送的数据帧
 * @version 0.1
 * @date 2021-09-21
 *
 * @copyright Copyright (c) 2021 ROSC
 *
 */
#ifndef ROBOT_BRAIN_ETHERCAT_FRAME_TYPES_HPP_
#define ROBOT_BRAIN_ETHERCAT_FRAME_TYPES_HPP_
#include <bits/stdint-intn.h>
#include <bits/stdint-uintn.h>
#include <cstring>
#include <robot_brain/config.h>
#include <ostream>
#include <string>
#include <bitset>

#define CHECK_DEVICE_FREQUENCY 1000  // 检查设备的时钟周期间隔数
#define SYNC_CLOCKS_FREQUENCY 1   // 同步时钟的时钟周期间隔数
namespace rosc {
typedef uint8_t operation_mode_t;
typedef uint16_t ctrl_word_t;
typedef uint16_t status_word_t;
typedef int32_t driver_position_t;
typedef uint8_t modes_operation_display_t;
typedef uint8_t ethercat_frame_send_status_t;
typedef uint16_t error_code_t;
typedef int16_t torque_t;
typedef int32_t velocity_t;

enum EthercatFrameSendStatus { IDLE, BUSY };

// 运动模式
enum MotionMode {
  MOVE_SEND = 0,  // 运动指令，下发
  MOVECIRCLE_SEND = 1,
  MOVELINE_SEND = 2,
  POWERON_SEND = 3,     // 上电指令，下发
  POWEROFF_SEND = 4,    // 下电指令，下发
  CLEARERROR_SEND = 5,  // 清错指令，下发
  MOVEFINISH_SEND = 6,  // 运动完毕，下发
  SAVEPOINT_SEND = 7,   // 保存点位，下发
  IO_SEND = 8,          // IO指令，下发
  MOVING_FEEDBACK = 9,  // 运动中，反馈
  JOG_ANGLE = 10,

  POWERON_FEEDBACK = 11,
  POWEROFF_FEEDBACK = 12,
  CLEARERROR_FEEDBACK = 13,
  SAVEPOINT_FEEDBACK = 14,
  IO_FEEDBACK = 15,
  COMPLETED_FEEDBACK = 16,  // 指令执行结束，反馈
  FAILURE_FEEDBACK = 17,    // 指令执行失败，反馈
  PlanFailed = 18           // 指令规划失败
};

#define ENCODER_MESG_MAXLEN 100
typedef enum ServoState { EMCY, ERROR, OP, READY } ServoState;
// 上层组件下发数据格式
struct EncoderFrame {
  double target_position[kSLAVENUMMAX];
  double current_position[kDofMax];
  char mesg[ENCODER_MESG_MAXLEN];
  char point_name[ENCODER_MESG_MAXLEN];
  int64_t time_stamp;
  int al_state;
  uint32_t digit_io_in;
  ethercat_frame_send_status_t status;
  ServoState servo_state;
  MotionMode mode;
};

struct DigitIOFrame {
  uint32_t digit_io_out;
};

/**
 * @brief ethercat控制帧
 *
 */
struct EthercatCtlFrame {
 public:
  operation_mode_t operation_mode[kSLAVENUMMAX];    // 操作模式
  ctrl_word_t ctrl_word[kSLAVENUMMAX];              // 控制字
  driver_position_t target_position[kSLAVENUMMAX];  // 目标位置
  uint32_t digit_io_out;                         // 数字IO输出8位
 protected:
  /**
   * @brief 重载输出操作符
   *
   * @param out
   * @param item
   * @return std::ostream&
   */
  friend std::ostream &operator<<(std::ostream &out,
                                  const EthercatCtlFrame &item);
  EthercatCtlFrame() { memset(target_position, 0, sizeof(target_position)); }
};

/**
 * @brief ethercat状态帧
 *
 */
struct EthercatStatusFrame {
 public:
  operation_mode_t operation_mode[kSLAVENUMMAX];     // 操作模式
  ctrl_word_t ctrl_word[kSLAVENUMMAX];               // 控制字
  driver_position_t target_position[kSLAVENUMMAX];   // 目标位置
  status_word_t status_word[kSLAVENUMMAX];           // 状态字
  driver_position_t current_position[kSLAVENUMMAX];  // 当前位置
  torque_t current_torque[kSLAVENUMMAX];             // 当前力矩
  velocity_t last_position[kSLAVENUMMAX];            // 当前速度
  error_code_t error_code[kSLAVENUMMAX];             // 错误码
  modes_operation_display_t
      modes_operation_display[kSLAVENUMMAX];                // 当前操作字
  uint32_t digit_io_out;                                    // 数字io输入
  uint32_t digit_io_in;                                     // 数字io输出
  ethercat_frame_send_status_t ethercat_frame_send_status;  // 帧发送状态
  int al_state;                                             // igh slave state
  int data_flag;

  EthercatStatusFrame() {
    memset(current_position, 0, sizeof(current_position));
  }
  /**
   * @brief 重载输出操作符
   *
   * @param out
   * @param item
   * @return std::ostream&
   */
  friend std::ostream &operator<<(std::ostream &out,
                                  const EthercatStatusFrame &item);
};

inline std::ostream &operator<<(std::ostream &out,
                                const EthercatCtlFrame &item) {
  out << "ctrl_word:";
  for (int i = 0; i < kSLAVENUMMAX; i++) {
    out << int(item.ctrl_word[i]) << " ";
  }
  out << std::endl;
  out << "operation_mode:";
  for (int i = 0; i < kSLAVENUMMAX; i++) {
    out << int(item.operation_mode[i]) << " ";
  }
  out << std::endl;
  out << "target_position:";
  for (int i = 0; i < kSLAVENUMMAX; i++) {
    out << int(item.target_position[i]) << " ";
  }
  out << std::endl;
  return out;
}

inline std::ostream &operator<<(std::ostream &out,
                                const EthercatStatusFrame &item) {
  out << "operation_mode:";
  for (int i = 0; i < kSLAVENUMMAX; i++) {
    out << int(item.operation_mode[i]) << " ";
  }
  out << std::endl;
  out << "ctrl_word:";
  for (int i = 0; i < kSLAVENUMMAX; i++) {
    out << int(item.ctrl_word[i]) << " ";
  }
  out << std::endl;
  out << "target_position:";
  for (int i = 0; i < kSLAVENUMMAX; i++) {
    out << int(item.target_position[i]) << " ";
  }
  out << std::endl;
  out << "status_word:";
  for (int i = 0; i < kSLAVENUMMAX; i++) {
    out << int(item.status_word[i]) << " ";
  }
  out << std::endl;
  out << "current_position:";
  for (int i = 0; i < kSLAVENUMMAX; i++) {
    out << int(item.current_position[i]) << " ";
  }
  out << std::endl;
  out << "operation_mode:";
  for (int i = 0; i < kSLAVENUMMAX; i++) {
    out << int(item.operation_mode[i]) << " ";
  }
  out << std::endl;
  return out;
}

}  // namespace rosc

#endif  // ROBOT_BRAIN_ETHERCAT_FRAME_TYPES_HPP_
