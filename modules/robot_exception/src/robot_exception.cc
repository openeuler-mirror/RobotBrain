/**
 * @file robot_exception.cc
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-06-13
 *
 * @copyright Copyright (c) 2022 ROSC
 *
 */
#include <map>
#include "robot_brain/robot_exception/robot_status.h"
#include <robot_brain/robot_exception.hpp>

namespace rosc {
std::map<StatusCode, std::string> code_info_map = {
    {StatusCode::kSuccess, "success!"},

    {StatusCode::kSecondaryPowerOff,
     "Motion command failed due to servo power off."},
    {StatusCode::kSecondaryPowerOn,
     "Setting command failed due to servo power on. "},
    {StatusCode::kHomingNotComplete, "The motion command requires homing."},
    {StatusCode::kInTeachMode, "Command was rejected because host command was "
                               "sent when the controller was in TEACH mode."},
    {StatusCode::kUableSetPitch,
     "Host attempted to set pitch between slot when pitch generation method "
     "was set to [automatic calculation mode]"},
    {StatusCode::kUableRestartMotion,
     "RESTART command (CRSM) was rejected because there was no motion command "
     "had been paused or the servo power was off "},
    {StatusCode::kReadyPositionMoveIncomplete1,
     "Get/Put (or exchange) command was rejected "
     "because “move to ready position” command(MTRS) "
     "had not been executed before that "},
    {StatusCode::kReadyPositionMoveIncomplete2,
     "Alignment command was rejected because “move to alignment ready "
     "position” command for pr-aligner had not been executed before the "
     "alignment command. "},
    {StatusCode::kInproperStationType,
     "Inaccessible station type is specified."},
    {StatusCode::kRequestedFunctionNotEnabled,
     "Requested action was not executed because the function was not enabled "
     "by a parameter."},
    {StatusCode::kMaintenanceToolInUse,
     "Cannot execute the command since maintenance tool is in use."},
    {StatusCode::kCommandNotSupported,
     "Command was rejected because the command sent is not supported by "
     "specified unit "},
    {StatusCode::kInvalidTransferPoint,
     " Motion between Transfer Points (MPNT) command was rejected because an "
     "invalid point was specified "},
    {StatusCode::kLinearMotionFailed,
     "Cannot move since manipulator arm is in the range(or the posture) where "
     "linear motion cannot be executed "},
    {StatusCode::kUableShuntPassiveWristAxis,
     "Unable to shunt the passive wrist safely from current posture "},
    {StatusCode::kUnablePerformArmCalibration,
     "Arm calibration was rejected because the pre-aligner stage has not been "
     "recorded."},
    {StatusCode::kUnableReadMappingData,
     "The mapping data reference command (RMAP) was rejected because the "
     "mapping has not been executed ever."},
    {StatusCode::kDataLoadInProgress, "Command is rejected because data upload "
                                      "/download command was in progress."},
    {StatusCode::kNotInFlippingPosition,
     "Motion cannot be start due to improper flipping axis posture.(For "
     "manipulators with the flipping axis only.) "},
    {StatusCode::kUnableToHome,
     "Unable to home safely from current manipulator posture.(Possibility of "
     "interference) "},
    {StatusCode::kUnablePerformExchangeCommand,
     "For dual-arm manipulators only. The “exchange motion” command is not "
     "supported at specified station."},
    {StatusCode::kLifterInterferenceError,
     "Lifter collides with P/A or manipulator if it is moved"},
    {StatusCode::kInterferenceCheckError1,
     "It passes through an interference area at the time of single axis "
     "operation other than JOG operation."},
    {StatusCode::kInterferenceCheckError2,
     "Minimum sweep posture is in interference area."},
    {StatusCode::kInterferenceCheckError3,
     "Transportation route passes through interference area."},
    {StatusCode::kInterferenceCheckError4,
     "Minimum sweep posture is in interference area. "},
    {StatusCode::kBottomSlotpositionRecordIncomplete,
     "Command was rejected because the bottom slot position had not been "
     "recorded "},
    {StatusCode::kTopSlotposiitonRecordIncomplete,
     "Command was rejected because the top slot position had not been recorded "
     "when pitch generation method was set to “automatic calculation mode”."},
    {StatusCode::kViaPointOfLinearNotTaught,
     "Via position of cylindrical linear station has not been taught when host "
     "sent motion command to the station "},
    {StatusCode::kUnableGeneratecylindricalLinearPath,
     "When the host sent motion command, NXC was unable to generate the "
     "cylindrical linear motion path."},

    {StatusCode::kInvalidPosture,
     "For manipulators with the flipping axis only. "
     "Attempted to teach the posture different from the preset station"
     "attribute."},
    {StatusCode::kMinSweepPostureGenerateError,
     "Unable to teach because calculated minimum "
     "sweep position exceeds motion range"},
    {StatusCode::kIntermediatePositionW2GenerateError,
     " Unable to teach because calculated intermediate position ‘W2’exceeds "
     "motion range."},
    {StatusCode::kIntermediatePositionWGenerateError,
     "Unable to teach because calculated intermediate position ‘W’ exceeds "
     "motion range."},
    {StatusCode::kReadyPositionGenerateError,
     "Unable to teach because calculated ready position exceeds motion range "},
    {StatusCode::kIntermediatePositionSGenerateError,
     "Unable to teach because calculated intermediate position ‘S’ exceeds "
     "motion range."},
    {StatusCode::kTeachPositionGenerateError,
     "Unable to teach because calculated position exceeds motion range "},
    {StatusCode::kUnableGeneratecylindricalLinearPosition,
     "NXC was unable to generate the positions for cylindrical linear motion "
     "from taught position."},
    {StatusCode::kUnableGenerateMappingPosition,
     "Unable to generate the positions for mapping from taught position."},

    {StatusCode::kInvalidParaINIT, "Invalid parameters for NXC command INIT."},
    {StatusCode::kInvalidParaMHOM, "Invalid parameters for NXC command MHOM."},
    {StatusCode::kInvalidParaNXC, "Invalid parameters for NXC command."},

    // IO
    {StatusCode::KInvalidSignal, "Invalid IO input signal."},
    {StatusCode::KInvalidSignalGate, "Invalid status of IO input signal gate."},
    {StatusCode::kInvalidSignalSolenoid,
     "Invalid status of IO input signal solenoid."},
    {StatusCode::kInvalidSignalReady,
     "Invalid status of IO input signal ready."},
    {StatusCode::kInvalidSignalPA, "Invalid status of IO input signal PA."},
    // Servo 1
    {StatusCode::kMomentaryOverload_1,
     "A torque that exceeds largely the rated value is applied for a several "
     "or several tens seconds."},
    {StatusCode::kContinuousOverload_1,
     "A torque that largely exceeds the rated value is applied continuously."},
    {StatusCode::kDynamicBrakeOverload_1,
     "The rotation energy at DB (dynamic brake) activation exceeds the DB "
     "resistance capacity."},
    {StatusCode::kSurgeResistorOverload_1,
     "Current surge when power is turned ON. "},
    {StatusCode::kConverterOverload_1, "Converter has Overload."},
    {StatusCode::kHeatsinkOverheat_1, "SERVOPACK heat sink has overheated."},
    {StatusCode::kAmplifierOverheat_1, "Amplifier Overheat occurred. "},
    {StatusCode::kPositioningTimeout_1,
     "After command is transmitted, #1 axis positioning cannot be completed."},
    {StatusCode::kExcessiveSpeed_1, "Motor speed is excessively high. "},
    {StatusCode::kReferenceSpeedError_1, "Reference Speed Cable disconnected"},
    {StatusCode::kSpeedReferenceInputLevelError_1,
     "Speed Reference Input Level is improper"},
    {StatusCode::kServoOnError_1, "Error occurs in Servo on "},
    {StatusCode::kVibrationAlarm_1,
     "Vibration at the motor speed was detected."},

    {StatusCode::kServoESMError_1, "Servo 1 occured error 88.*"},
    {StatusCode::kServoEtherCATSyncError_1, "Servo 1 occured error 80.*"},
    {StatusCode::kServoTorqueOverloadError_1, "Servo 1 occured error 16.*"},
    {StatusCode::kServoExcessivePositionMode_1, "Servo 1 occured error 24.*"},
    // servo 2
    {StatusCode::kMomentaryOverload_2,
     "A torque that exceeds largely the rated value is applied for a several "
     "or several tens seconds."},
    {StatusCode::kContinuousOverload_2,
     "A torque that largely exceeds the rated value is applied continuously."},
    {StatusCode::kDynamicBrakeOverload_2,
     "The rotation energy at DB (dynamic brake) activation exceeds the DB "
     "resistance capacity."},
    {StatusCode::kSurgeResistorOverload_2,
     "Current surge when power is turned ON. "},
    {StatusCode::kConverterOverload_2, "Converter has Overload."},
    {StatusCode::kHeatsinkOverheat_2, "SERVOPACK heat sink has overheated."},
    {StatusCode::kAmplifierOverheat_2, "Amplifier Overheat occurred. "},
    {StatusCode::kPositioningTimeout_2,
     "After command is transmitted, #1 axis positioning cannot be completed."},
    {StatusCode::kExcessiveSpeed_2, "Motor speed is excessively high. "},
    {StatusCode::kReferenceSpeedError_2, "Reference Speed Cable disconnected"},
    {StatusCode::kSpeedReferenceInputLevelError_2,
     "Speed Reference Input Level is improper"},
    {StatusCode::kServoOnError_2, "Error occurs in Servo on "},
    {StatusCode::kVibrationAlarm_2,
     "Vibration at the motor speed was detected."},

    {StatusCode::kServoESMError_2, "Servo 2 occured error 88.*"},
    {StatusCode::kServoEtherCATSyncError_2, "Servo 2 occured error 80.*"},
    {StatusCode::kServoTorqueOverloadError_2, "Servo 2 occured error 16.*"},
    {StatusCode::kServoExcessivePositionMode_2, "Servo 2 occured error 24.*"},
    // servo 3
    {StatusCode::kMomentaryOverload_3,
     "A torque that exceeds largely the rated value is applied for a several "
     "or several tens seconds."},
    {StatusCode::kContinuousOverload_3,
     "A torque that largely exceeds the rated value is applied continuously."},
    {StatusCode::kDynamicBrakeOverload_3,
     "The rotation energy at DB (dynamic brake) activation exceeds the DB "
     "resistance capacity."},
    {StatusCode::kSurgeResistorOverload_3,
     "Current surge when power is turned ON. "},
    {StatusCode::kConverterOverload_3, "Converter has Overload."},
    {StatusCode::kHeatsinkOverheat_3, "SERVOPACK heat sink has overheated."},
    {StatusCode::kAmplifierOverheat_3, "Amplifier Overheat occurred. "},
    {StatusCode::kPositioningTimeout_3,
     "After command is transmitted, #1 axis positioning cannot be completed."},
    {StatusCode::kExcessiveSpeed_3, "Motor speed is excessively high. "},
    {StatusCode::kReferenceSpeedError_3, "Reference Speed Cable disconnected"},
    {StatusCode::kSpeedReferenceInputLevelError_3,
     "Speed Reference Input Level is improper"},
    {StatusCode::kServoOnError_3, "Error occurs in Servo on "},
    {StatusCode::kVibrationAlarm_3,
     "Vibration at the motor speed was detected."},

    {StatusCode::kServoESMError_3, "Servo 3 occured error 88.*"},
    {StatusCode::kServoEtherCATSyncError_3, "Servo 3 occured error 80.*"},
    {StatusCode::kServoTorqueOverloadError_3, "Servo 3 occured error 16.*"},
    {StatusCode::kServoExcessivePositionMode_3, "Servo 3 occured error 24.*"},
    // servo 4
    {StatusCode::kMomentaryOverload_4,
     "A torque that exceeds largely the rated value is applied for a several "
     "or several tens seconds."},
    {StatusCode::kContinuousOverload_4,
     "A torque that largely exceeds the rated value is applied continuously."},
    {StatusCode::kDynamicBrakeOverload_4,
     "The rotation energy at DB (dynamic brake) activation exceeds the DB "
     "resistance capacity."},
    {StatusCode::kSurgeResistorOverload_4,
     "Current surge when power is turned ON. "},
    {StatusCode::kConverterOverload_4, "Converter has Overload."},
    {StatusCode::kHeatsinkOverheat_4, "SERVOPACK heat sink has overheated."},
    {StatusCode::kAmplifierOverheat_4, "Amplifier Overheat occurred. "},
    {StatusCode::kPositioningTimeout_4,
     "After command is transmitted, #1 axis positioning cannot be completed."},
    {StatusCode::kExcessiveSpeed_4, "Motor speed is excessively high. "},
    {StatusCode::kReferenceSpeedError_4, "Reference Speed Cable disconnected"},
    {StatusCode::kSpeedReferenceInputLevelError_4,
     "Speed Reference Input Level is improper"},
    {StatusCode::kServoOnError_4, "Error occurs in Servo on "},
    {StatusCode::kVibrationAlarm_4,
     "Vibration at the motor speed was detected."},

    {StatusCode::kServoESMError_4, "Servo 4 occured error 88.*"},
    {StatusCode::kServoEtherCATSyncError_4, "Servo 4 occured error 80.*"},
    {StatusCode::kServoTorqueOverloadError_4, "Servo 4 occured error 16.*"},
    {StatusCode::kServoExcessivePositionMode_4, "Servo 4 occured error 24.*"},
    // servo 5
    {StatusCode::kMomentaryOverload_5,
     "A torque that exceeds largely the rated value is applied for a several "
     "or several tens seconds."},
    {StatusCode::kContinuousOverload_5,
     "A torque that largely exceeds the rated value is applied continuously."},
    {StatusCode::kDynamicBrakeOverload_5,
     "The rotation energy at DB (dynamic brake) activation exceeds the DB "
     "resistance capacity."},
    {StatusCode::kSurgeResistorOverload_5,
     "Current surge when power is turned ON. "},
    {StatusCode::kConverterOverload_5, "Converter has Overload."},
    {StatusCode::kHeatsinkOverheat_5, "SERVOPACK heat sink has overheated."},
    {StatusCode::kAmplifierOverheat_5, "Amplifier Overheat occurred. "},
    {StatusCode::kPositioningTimeout_5,
     "After command is transmitted, #1 axis positioning cannot be completed."},
    {StatusCode::kExcessiveSpeed_5, "Motor speed is excessively high. "},
    {StatusCode::kReferenceSpeedError_5, "Reference Speed Cable disconnected"},
    {StatusCode::kSpeedReferenceInputLevelError_5,
     "Speed Reference Input Level is improper"},
    {StatusCode::kServoOnError_5, "Error occurs in Servo on "},
    {StatusCode::kVibrationAlarm_5,
     "Vibration at the motor speed was detected."},

    {StatusCode::kServoESMError_5, "Servo 5 occured error 88.*"},
    {StatusCode::kServoEtherCATSyncError_5, "Servo 5 occured error 80.*"},
    {StatusCode::kServoTorqueOverloadError_5, "Servo 5 occured error 16.*"},
    {StatusCode::kServoExcessivePositionMode_5, "Servo 5 occured error 24.*"},

    /****************************************************************/
    {StatusCode::kCoreFailed, "Fatal error, Core dumped!"},

    {StatusCode::kPowerOffFailed, "Servo Power Off Failed!"},
    {StatusCode::kPowerOffTimeout, "Servo Power Off time out!"},
    {StatusCode::kPowerOffEthercatNotActive,
     "Servo Power Off Failed, ethercat  Unactived."},

    {StatusCode::kPowerOnFailed, "Servo Power On Failed!"},
    {StatusCode::kPowerOnTimeout, "Servo Power On Time Out!"},
    {StatusCode::kPowerOnEthercatNotActive,
     "Servo Power On Failed, Ethercat Unactived."},
    {StatusCode::kPowerOnPositionNotSync,
     "Servo Power On Failed, Position  Unsynchronized."},
    {StatusCode::kPowerOnEthercatPortNotConnect,
     "Servo Power On Failed, ethercat port not connected."},

    {StatusCode::kClearErrorFaild, "Servo Clear Error Failed."},
    {StatusCode::kRobotMoveCommandFailed,
     "Robot Component Executed Move Command failed."},
    {StatusCode::kRobotMoveCommandTypeMismatch,
     "Robot Component Move Command type mismatched."},
    {StatusCode::kRobotPointCommandFiled,
     "Robot Componnet executed Point command failed."},

    {StatusCode::kRobotExceptionalCommand,
     "Robot Component Received Exceptional Command."},
    {StatusCode::kRobotErrorStateCommand,
     "Robot received command in error state, can not execute."},

    {StatusCode::kRobotPortNotConnected, "Robot Component port not connected."},
    {StatusCode::kRobotPortNotConnected_in_ec, "Robot Port Unconnected In EC"},
    {StatusCode::kRobotPortNotConnected_out_ec,
     "Robot Port Unconnected Out EC"},
    {StatusCode::kRobotPortNotConnected_in_traj,
     "Robot Port Unconnected In Trajectory."},
    {StatusCode::kRobotPortNotConnected_out_traj,
     "Robot Port Unconnected Out Trajectory."},

    {StatusCode::kServoStateError, "Servo State Error."},
    {StatusCode::kServoStateError_1, "1st Servo State Error."},
    {StatusCode::kServoStateError_2, "2nd Servo State Error."},
    {StatusCode::kServoStateError_3, "3rd Servo State Error."},
    {StatusCode::kServoStateError_4, "4th Servo State Error."},

    {StatusCode::kServoEmergencyState, "Servo in Emergency State."},
    {StatusCode::kTrajectoryPointNameTooLong,
     "Trajectory Point Command Name Too Long."},
    {StatusCode::kTrajectoryUndefinedCommand,
     "Trajectory Reveived Undefined Command."},

    {StatusCode::kTrajectoryClearTimeout, "Trajectory clear error time out."},
    {StatusCode::kTrajectoryMoveBeyondLimits,
     "Trajectory Move command beyond axis limits."},
    {StatusCode::kTrajectoryMoveBeyondLimits_start,
     "Trajectory Move command start point beyond axis limits."},
    {StatusCode::kTrajectoryMoveBeyondLimits_end,
     "Trajectory Move command end point beyond axis limits."},
    {StatusCode::kTrajectoryMoveBeyondLimits_current,
     "Trajectory Move command current point beyond axis limits."},
    {StatusCode::kTrajectoryMoveBeyondLimits_1,
     "Trajectory Move command beyond elevation axis limits."},
    {StatusCode::kTrajectoryMoveBeyondLimits_2,
     "Trajectory Move command beyond Rotation axis limits."},
    {StatusCode::kTrajectoryMoveBeyondLimits_3,
     "Trajectory Move command beyond Extension axis limits."},
    {StatusCode::kTrajectoryMoveBeyondLimits_4,
     "Trajectory Move command beyond Extension axis limits."},
    {StatusCode::kTrajectoryMoveStartMismatch,
     "Trajectory Move command start point mismatch current position."},
    {StatusCode::kTrajectoryMovePlanFailed,
     "Trajectory Move command planning failed."},
    {StatusCode::kTrajectoryMovePlanFailedZone,
     "Trajectory Move command interpolation zone planning failed."},
    {StatusCode::kTrajectoryMoveJogThreadGetLockFailed,
     "Trajectory Move jog command thread get lock failed."},
    {StatusCode::kTrajectoryMoveJogSpeedBeyond,
     "Trajectory move jog command over speed."},
    {StatusCode::kTrajectoryMoveLineExceptionalType,
     "Trajectory move line command exceptional type"},
    {StatusCode::kTrajectoryMoveLinePlanFailed,
     "Trajectory move line command planning failed."},

    {StatusCode::kTrajectoryPortNotConnected_in_rob,
     "Trajectory Port Unconnected In Robot."},
    {StatusCode::kTrajectoryPortNotConnected_out_rob,
     "Trajectory Port Unconnected Out Robot."},
    {StatusCode::kTrajectoryPortNotConnected_in_service,
     "Trajectory Port Unconnected In Service."},
    {StatusCode::kTrajectoryPortNotConnected_out_servive,
     "Trajectory Port Unconnected Out Service."},

    {StatusCode::kRobotServicePortNotConnected_in_ec,
     "Robot service port unconnected in ethercat."},
    {StatusCode::kRobotServicePortNotConnected_out_ec,
     "Robot service port unconnected out ethercat."},
    {StatusCode::kRobotServicePortNotConnected_out_traj,
     "Robot Service Port not connected out Trajectory."},
    {StatusCode::kRobotServicePortNotConnected_in_traj,
     "Robot Service Port not connected in Trajectory."},
    {StatusCode::kRobotServicePortNotConnected_out_service,
     "Robot Service Port not connected out Service."},
    {StatusCode::kRobotServicePortNotConnected_in_service,
     "Robot Service Port not connected in Service."},

};

}  // namespace rosc
