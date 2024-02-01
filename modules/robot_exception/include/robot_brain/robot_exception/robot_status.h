/**
 * @file robot_status.h
 * @author YuanboDou (douyuanbo@buaa.edu.cn)
 * @brief 机器人异常状态定义组件
 * @version 0.1
 * @date 2022-02-21
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef MODULES_ROBOT_EXCEPTION_INCLUDE_ROBOT_BRAIN_ROBOT_EXCEPTION_ROBOT_STATUS_H_
#define MODULES_ROBOT_EXCEPTION_INCLUDE_ROBOT_BRAIN_ROBOT_EXCEPTION_ROBOT_STATUS_H_
#include <glog/logging.h>
#include <map>
#include <string>

namespace rosc {
/**
 * @brief 异常等级编码
 *
 */
enum ExceptionLevel : uint32_t {
  kFatal = 0x500000u,
  kSeriousError2 = 0x400000u,
  kSeriousError1 = 0x300000u,
  kWarning2 = 0x200000u,
  kWarning1 = 0x100000u
};

enum ExceptionClass : uint32_t {
  kConfig = 0xFF000,
  kData = 0xFF000,
};

/**
 * @brief 异常编码，一共6位，0-2位为子错误码，3-4位为主错误码，5位为错误等级
 *
 */
enum StatusCode : uint32_t {
  kSuccess = 0,
  Normal = 0x00000 | 0x000,
  kSecondaryPowerOff = kWarning2 | 0x02000 | 0x020,
  kSecondaryPowerOn = kWarning2 | 0x02000 | 0x021,
  kHomingNotComplete = kWarning2 | 0x03000 | 0x030,
  kInTeachMode = kWarning2 | 0x04000 | 0x040,

  kUnitInMotion = kWarning2 | 0x05000 | 0x050,
  kUableSetPitch = kWarning2 | 0x05000 | 0x051,
  kUableRestartMotion = kWarning2 | 0x05000 | 0x052,
  kReadyPositionMoveIncomplete1 = kWarning2 | 0x05000 | 0x053,
  kReadyPositionMoveIncomplete2 = kWarning2 | 0x05000 | 0x054,
  kInproperStationType = kWarning2 | 0x05000 | 0x055,
  kRequestedFunctionNotEnabled = kWarning2 | 0x05000 | 0x056,
  kMaintenanceToolInUse = kWarning2 | 0x05000 | 0x057,
  kCommandNotSupported = kWarning2 | 0x05000 | 0x058,
  kInvalidTransferPoint = kWarning2 | 0x05000 | 0x059,
  kLinearMotionFailed = kWarning2 | 0x05000 | 0x05a,
  kUableShuntPassiveWristAxis = kWarning2 | 0x05000 | 0x05b,
  kUnablePerformArmCalibration = kWarning2 | 0x05000 | 0x05d,
  kUnableReadMappingData = kWarning2 | 0x05000 | 0x05e,
  kDataLoadInProgress = kWarning2 | 0x05000 | 0x05f,

  kNotInFlippingPosition = kWarning2 | 0x06000 | 0x060,
  kUnableToHome = kWarning2 | 0x06000 | 0x061,
  kUnablePerformExchangeCommand = kWarning2 | 0x06000 | 0x062,
  kLifterInterferenceError = kWarning2 | 0x06000 | 0x064,
  kInterferenceCheckError1 = kWarning2 | 0x06000 | 0x068,
  kInterferenceCheckError2 = kWarning2 | 0x06000 | 0x069,
  kInterferenceCheckError3 = kWarning2 | 0x06000 | 0x06a,
  kInterferenceCheckError4 = kWarning2 | 0x06000 | 0x06b,

  kBottomSlotpositionRecordIncomplete = kWarning2 | 0x07000 | 0x070,
  kTopSlotposiitonRecordIncomplete = kWarning2 | 0x07000 | 0x071,
  kViaPointOfLinearNotTaught = kWarning2 | 0x07000 | 0x072,
  kUnableGeneratecylindricalLinearPath = kWarning2 | 0x07000 | 0x074,

  kInvalidPosture = kWarning2 | 0x08000 | 0x080,
  kMinSweepPostureGenerateError = kWarning2 | 0x08000 | 0x088,
  kIntermediatePositionW2GenerateError = kWarning2 | 0x08000 | 0x089,
  kIntermediatePositionWGenerateError = kWarning2 | 0x08000 | 0x08a,
  kReadyPositionGenerateError = kWarning2 | 0x08000 | 0x08b,
  kIntermediatePositionSGenerateError = kWarning2 | 0x08000 | 0x08c,
  kTeachPositionGenerateError = kWarning2 | 0x08000 | 0x08d,
  kUnableGeneratecylindricalLinearPosition = kWarning2 | 0x08000 | 0x08e,
  kUnableGenerateMappingPosition = kWarning2 | 0x08000 | 0x08f,

  kInvalidParaMHOM = kWarning2 | 0x09000 | 0x091,
  kInvalidParaINIT = kWarning2 | 0x09000 | 0x092,
  kInvalidParaNXC = kWarning2 | 0x09000 | 0x09A,

  // IO 检测失败
  KInvalidSignal = kWarning2 | 0xAA000 | 0xAA0,
  KInvalidSignalGate = kWarning2 | 0xAA000 | 0xAA1,
  kInvalidSignalSolenoid = kWarning2 | 0xAA000 | 0xAA2,
  kInvalidSignalReady = kWarning2 | 0xAA000 | 0xAA3,
  kInvalidSignalPA = kWarning2 | 0xAA000 | 0xAA4,
  kUnableProcessDueW = kWarning2 | 0xAA000 | 0xAA5,
  // Axis-1 servo error
  kMomentaryOverload_1 = kSeriousError1 | 0x10000 | 0x171,
  kContinuousOverload_1 = kSeriousError1 | 0x10000 | 0x172,
  kDynamicBrakeOverload_1 = kSeriousError1 | 0x10000 | 0x173,
  kSurgeResistorOverload_1 = kSeriousError1 | 0x10000 | 0x174,
  kConverterOverload_1 = kSeriousError1 | 0x10000 | 0x178,
  kHeatsinkOverheat_1 = kSeriousError1 | 0x10000 | 0x17a,
  kAmplifierOverheat_1 = kSeriousError1 | 0x10000 | 0x17b,

  kPositioningTimeout_1 = kSeriousError1 | 0x11000 | 0xE11,

  kExcessiveSpeed_1 = kSeriousError1 | 0x12000 | 0x151,
  kReferenceSpeedError_1 = kSeriousError1 | 0x12000 | 0x152,
  kSpeedReferenceInputLevelError_1 = kSeriousError1 | 0x12000 | 0x153,
  kServoOnError_1 = kSeriousError1 | 0x12000 | 0x154,
  kVibrationAlarm_1 = kSeriousError1 | 0x12000 | 0x1c0,
  // 88.系列错误
  kServoESMError_1 = kSeriousError1 | 0x1a000 | 0x188,
  // 80.系列错误
  kServoEtherCATSyncError_1 = kSeriousError1 | 0x1a000 | 0x180,
  // 16.系列错误
  kServoTorqueOverloadError_1 = kSeriousError1 | 0x1a000 | 0x116,
  // 24.系列错误
  kServoExcessivePositionMode_1 = kSeriousError1 | 0x1a000 | 0x124,

  // Axis-2 servo error
  kMomentaryOverload_2 = kSeriousError1 | 0x20000 | 0x271,
  kContinuousOverload_2 = kSeriousError1 | 0x20000 | 0x272,
  kDynamicBrakeOverload_2 = kSeriousError1 | 0x20000 | 0x273,
  kSurgeResistorOverload_2 = kSeriousError1 | 0x20000 | 0x274,
  kConverterOverload_2 = kSeriousError1 | 0x20000 | 0x278,
  kHeatsinkOverheat_2 = kSeriousError1 | 0x20000 | 0x27a,
  kAmplifierOverheat_2 = kSeriousError1 | 0x20000 | 0x27b,

  kPositioningTimeout_2 = kSeriousError1 | 0x21000 | 0xE11,

  kExcessiveSpeed_2 = kSeriousError1 | 0x22000 | 0x251,
  kReferenceSpeedError_2 = kSeriousError1 | 0x22000 | 0x252,
  kSpeedReferenceInputLevelError_2 = kSeriousError1 | 0x22000 | 0x253,
  kServoOnError_2 = kSeriousError1 | 0x22000 | 0x254,
  kVibrationAlarm_2 = kSeriousError1 | 0x22000 | 0x2c0,
  // 88.系列错误
  kServoESMError_2 = kSeriousError1 | 0x2a000 | 0x288,
  // 80.系列错误
  kServoEtherCATSyncError_2 = kSeriousError1 | 0x2a000 | 0x280,
  // 16.系列错误
  kServoTorqueOverloadError_2 = kSeriousError1 | 0x2a000 | 0x216,
  // 24.系列错误
  kServoExcessivePositionMode_2 = kSeriousError1 | 0x2a000 | 0x224,

  // Axis-3 servo error
  kMomentaryOverload_3 = kSeriousError1 | 0x30000 | 0x371,
  kContinuousOverload_3 = kSeriousError1 | 0x30000 | 0x372,
  kDynamicBrakeOverload_3 = kSeriousError1 | 0x30000 | 0x373,
  kSurgeResistorOverload_3 = kSeriousError1 | 0x30000 | 0x374,
  kConverterOverload_3 = kSeriousError1 | 0x30000 | 0x378,
  kHeatsinkOverheat_3 = kSeriousError1 | 0x30000 | 0x37a,
  kAmplifierOverheat_3 = kSeriousError1 | 0x30000 | 0x37b,

  kPositioningTimeout_3 = kSeriousError1 | 0x31000 | 0xE11,

  kExcessiveSpeed_3 = kSeriousError1 | 0x32000 | 0x351,
  kReferenceSpeedError_3 = kSeriousError1 | 0x32000 | 0x352,
  kSpeedReferenceInputLevelError_3 = kSeriousError1 | 0x32000 | 0x353,
  kServoOnError_3 = kSeriousError1 | 0x32000 | 0x354,
  kVibrationAlarm_3 = kSeriousError1 | 0x32000 | 0x3c0,
  // 88.系列错误
  kServoESMError_3 = kSeriousError1 | 0x3a000 | 0x388,
  // 80.系列错误
  kServoEtherCATSyncError_3 = kSeriousError1 | 0x3a000 | 0x380,
  // 16.系列错误
  kServoTorqueOverloadError_3 = kSeriousError1 | 0x3a000 | 0x316,
  // 24.系列错误
  kServoExcessivePositionMode_3 = kSeriousError1 | 0x3a000 | 0x324,

  // Axis-4 servo error
  kMomentaryOverload_4 = kSeriousError1 | 0x40000 | 0x471,
  kContinuousOverload_4 = kSeriousError1 | 0x40000 | 0x472,
  kDynamicBrakeOverload_4 = kSeriousError1 | 0x40000 | 0x473,
  kSurgeResistorOverload_4 = kSeriousError1 | 0x40000 | 0x474,
  kConverterOverload_4 = kSeriousError1 | 0x40000 | 0x478,
  kHeatsinkOverheat_4 = kSeriousError1 | 0x40000 | 0x47a,
  kAmplifierOverheat_4 = kSeriousError1 | 0x40000 | 0x47b,

  kPositioningTimeout_4 = kSeriousError1 | 0x41000 | 0xE11,

  kExcessiveSpeed_4 = kSeriousError1 | 0x42000 | 0x451,
  kReferenceSpeedError_4 = kSeriousError1 | 0x42000 | 0x452,
  kSpeedReferenceInputLevelError_4 = kSeriousError1 | 0x42000 | 0x453,
  kServoOnError_4 = kSeriousError1 | 0x42000 | 0x454,
  kVibrationAlarm_4 = kSeriousError1 | 0x42000 | 0x4c0,
  // 88.系列错误
  kServoESMError_4 = kSeriousError1 | 0x4a000 | 0x488,
  // 80.系列错误
  kServoEtherCATSyncError_4 = kSeriousError1 | 0x4a000 | 0x480,
  // 16.系列错误
  kServoTorqueOverloadError_4 = kSeriousError1 | 0x4a000 | 0x416,
  // 24.系列错误
  kServoExcessivePositionMode_4 = kSeriousError1 | 0x4a000 | 0x424,

  // Axis-5 servo error
  kMomentaryOverload_5 = kSeriousError1 | 0x50000 | 0x571,
  kContinuousOverload_5 = kSeriousError1 | 0x50000 | 0x572,
  kDynamicBrakeOverload_5 = kSeriousError1 | 0x50000 | 0x573,
  kSurgeResistorOverload_5 = kSeriousError1 | 0x50000 | 0x574,
  kConverterOverload_5 = kSeriousError1 | 0x50000 | 0x578,
  kHeatsinkOverheat_5 = kSeriousError1 | 0x50000 | 0x57a,
  kAmplifierOverheat_5 = kSeriousError1 | 0x50000 | 0x57b,

  kPositioningTimeout_5 = kSeriousError1 | 0x51000 | 0xE11,

  kExcessiveSpeed_5 = kSeriousError1 | 0x52000 | 0x551,
  kReferenceSpeedError_5 = kSeriousError1 | 0x52000 | 0x552,
  kSpeedReferenceInputLevelError_5 = kSeriousError1 | 0x52000 | 0x553,
  kServoOnError_5 = kSeriousError1 | 0x52000 | 0x554,
  kVibrationAlarm_5 = kSeriousError1 | 0x52000 | 0x5c0,
  // 88.系列错误
  kServoESMError_5 = kSeriousError1 | 0x5a000 | 0x588,
  // 80.系列错误
  kServoEtherCATSyncError_5 = kSeriousError1 | 0x5a000 | 0x580,
  // 16.系列错误
  kServoTorqueOverloadError_5 = kSeriousError1 | 0x5a000 | 0x516,
  // 24.系列错误
  kServoExcessivePositionMode_5 = kSeriousError1 | 0x5a000 | 0x524,

  /****************************************************************/
  // Core, range: [1, 100]
  kCoreFailed = kFatal | 1,

  // Robot, range: [101, 200]
  /*** Robot组件错误和异常，主错误码0x100 ***/
  // 下电错误，主错误码：0xA1
  kPowerOffFailed = kWarning2 | 0xA1000 | 0xA11,
  kPowerOffTimeout = kWarning2 | 0xA1000 | 0xA12,
  kPowerOffEthercatNotActive = kWarning2 | 0xA1000 | 0xA13,
  // 上电错误，主错误码：0xA2
  kPowerOnFailed = kSeriousError1 | 0xA2000 | 0xA21,
  kPowerOnTimeout = kWarning2 | 0xA2000 | 0xA22,
  kPowerOnEthercatNotActive = kWarning2 | 0xA2000 | 0xA23,
  kPowerOnPositionNotSync = kSeriousError1 | 0xA2000 | 0xA24,
  kPowerOnEthercatPortNotConnect = kSeriousError1 | 0xA2 | 0xA25,
  // 机器人组件清错指令错误，主错误码 0xA3
  kClearErrorFaild = kWarning2 | 0xA3000 | 0xA30,
  // Move command, 0xA4
  kRobotMoveCommandFailed = kWarning2 | 0xA4000 | 0xA40,
  kRobotMoveCommandTypeMismatch = kWarning2 | 0xA4000 | 0xA41,

  // point command, 0xA5
  kRobotPointCommandFiled = kWarning2 | 0xA5000 | 0xA50,
  kRobotPointCommandTypeMismatch = kWarning2 | 0xA5000 | 0xA51,

  // 机器人组件指令异常，主错误码：0xAf
  kRobotExceptionalCommand =
      kWarning1 | 0xAf000 | 0xAF1,  // robot组件收到异常指令
  kRobotErrorStateCommand =
      kWarning1 | 0xAf000 | 0xAF2,  // 伺服错误状态下robot收到指令

  // 机器人组件端口连接错误，主错误码0xB1
  kRobotPortNotConnected = kSeriousError1 | 0xB1000 | 0xB10,
  kRobotPortNotConnected_in_traj = kSeriousError1 | 0xB1000 | 0xB11,
  kRobotPortNotConnected_out_traj = kSeriousError1 | 0xb1000 | 0xB12,
  kRobotPortNotConnected_out_ec = kSeriousError1 | 0xB1000 | 0xB13,
  kRobotPortNotConnected_in_ec = kSeriousError1 | 0xB1000 | 0xB14,

  // 伺服状态错误，主错误码：0xB2
  kServoStateError = kSeriousError2 | 0xB2000 | 0xB20,
  kServoStateError_1 = kSeriousError2 | 0xB2000 | 0xB21,
  kServoStateError_2 = kSeriousError2 | 0xB2000 | 0xB22,
  kServoStateError_3 = kSeriousError2 | 0xB2000 | 0xB23,
  kServoStateError_4 = kSeriousError2 | 0xB2000 | 0xB24,

  // 伺服急停状态，主错误码0xB0
  kServoEmergencyState = kWarning2 | 0xFF00 | 0xFF1,

  /*** Trajectory 组件错误和异常 主错误码0xc0***/
  // Trajectory组件,指令类异常，主错误码：0xC0
  kTrajectoryPointNameTooLong =
      kWarning2 | 0xC0000 | 0xC01,  // 点位指令名字过长
  kTrajectoryUndefinedCommand = kWarning2 | 0xC0000 | 0xC02,

  // Trajectory 指令执行超时, 主错误码：0xC1
  kTrajectoryClearTimeout = kSeriousError1 | 0xC1000 | 0xC11,

  // Trajectory 运动规划，主错误码：0xC2
  kTrajectoryMoveBeyondLimits = kWarning2 | 0xC2000 | 0xC20,
  kTrajectoryMoveBeyondLimits_start = kWarning2 | 0xC2000 | 0xC2A,
  kTrajectoryMoveBeyondLimits_end = kWarning2 | 0xC2000 | 0xC3B,
  kTrajectoryMoveBeyondLimits_current = kWarning2 | 0xC2000 | 0xC4C,
  kTrajectoryMoveBeyondLimits_1 = kWarning2 | 0xC2000 | 0xC21,
  kTrajectoryMoveBeyondLimits_2 = kWarning2 | 0xC2000 | 0xC22,
  kTrajectoryMoveBeyondLimits_3 = kWarning2 | 0xC2000 | 0xC23,
  kTrajectoryMoveBeyondLimits_4 = kWarning2 | 0xC2000 | 0xC24,
  kTrajectoryMoveStartMismatch = kWarning2 | 0xC2000 | 0xC25,
  kTrajectoryMovePlanFailed = kWarning2 | 0xC2000 | 0xC26,
  kTrajectoryMovePlanFailedZone = kWarning2 | 0xC2000 | 0xC2A,
  kTrajectoryMoveLinePlanFailed = kWarning2 | 0xC2000 | 0xC27,

  kTrajectoryMoveJogThreadGetLockFailed = kWarning2 | 0xC2000 | 0xC2D,
  kTrajectoryMoveJogSpeedBeyond = kWarning2 | 0xC2000 | 0xC2E,
  kTrajectoryMoveLineExceptionalType = kWarning2 | 0xC2000 | 0xC2F,

  // Trajectory组件端口连接错误，主错误码：0xc3
  kTrajectoryPortNotConnected_out_rob = kSeriousError1 | 0xc3000 | 0xC31,
  kTrajectoryPortNotConnected_out_servive = kSeriousError1 | 0xc3000 | 0xC32,
  kTrajectoryPortNotConnected_in_rob = kSeriousError1 | 0xC3000 | 0xC33,
  kTrajectoryPortNotConnected_in_service = kSeriousError1 | 0xC3000 | 0xC34,

  // RobotService组件端口连接错误，主错误码：0xc4
  kRobotServicePortNotConnected_out_ec = kSeriousError1 | 0xC4000 | 0xC41,
  kRobotServicePortNotConnected_in_ec = kSeriousError1 | 0xC4000 | 0xC42,
  kRobotServicePortNotConnected_out_traj = kSeriousError1 | 0xC4000 | 0xC43,
  kRobotServicePortNotConnected_in_traj = kSeriousError1 | 0xC4000 | 0xC44,
  kRobotServicePortNotConnected_out_service = kSeriousError1 | 0xC4000 | 0xC45,
  kRobotServicePortNotConnected_in_service = kSeriousError1 | 0xC4000 | 0xC46,

  // Config

  // 配置文件不存在
  kConfigFileNotExist =
      ExceptionLevel::kFatal | ExceptionClass::kConfig | 0x001,

  // Data
  // 数据文件加载错误
  kDataFileLoadError = ExceptionLevel::kFatal | ExceptionClass::kData | 0x002,
};

/**
 * @brief 异常编码与异常信息
 *
 */
extern std::map<StatusCode, std::string> code_info_map;

class Status {
 public:
  static std::string GetInfobyCode(StatusCode status_code) {
    if (code_info_map.find(status_code) == code_info_map.end()) {
      LOG(WARNING) << "异常码：" << std::hex << status_code
                   << "没有对应的异常描述";
      return "";
    }
    return code_info_map[status_code];
  }
};
}  // namespace rosc
#endif  // MODULES_ROBOT_EXCEPTION_INCLUDE_ROBOT_BRAIN_ROBOT_EXCEPTION_ROBOT_STATUS_H_
