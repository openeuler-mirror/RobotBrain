/**
 * @file grpc_service_component.h
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief
 * @version 0.1
 * @date 2022-01-07
 *
 * @copyright Copyright (c) 2022 ROSC
 *
 */
#ifndef MODULES_COMPONENTS_GRPC_SERVICE_COMPONENT_INCLUDE_GRPC_SERVICE_COMPONENT_H_
#define MODULES_COMPONENTS_GRPC_SERVICE_COMPONENT_INCLUDE_GRPC_SERVICE_COMPONENT_H_

#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpcpp/grpcpp.h>
#include <grpcpp/health_check_service_interface.h>
#include <grpcpp/impl/codegen/status.h>
#include <proto/robot_grpc_service.grpc.pb.h>
#include <robot_service_component.h>
#include <glog/logging.h>
#include <atomic>
#include <string>
#include <memory>
#include <map>
#include <queue>
#include <set>
#include <vector>
#include <rtt/RTT.hpp>
#include <robot_brain/command_types.hpp>
#include <robot_brain/robot_exception.hpp>

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::ServerReaderWriter;
using grpc::ServerWriter;
using grpc::Status;
using rosc::RobotGrpcService;

namespace rosc {

class GrpcServiceComponent : public RTT::TaskContext,
                             public RobotGrpcService::Service {
 private:
  int dof_;
  ArmType arm_type_;

 private:
  bool is_grpc_publish_jonits_;  ///< 是否对外通过grpc接口发送节点信息

 protected:
  RTT::OutputPort<rosc::SingleCommand>
      out_to_service_port_;  // 向RobotServiceComponent发送指令的端口
  RTT::InputPort<rosc::CommandExecResult>
      in_from_service_port_;  // 从RobotServiceComponent获取指令执行结果的端口
  RTT::InputPort<rosc::StatusCode>
      in_from_exception_port_;  // 从ExceptionComponent获取异常信息的端口

  RTT::OperationCaller<void(RobotCurrentState *const)>
      GetCurrentPosition;  // RobotService功能，获取当前机器人姿态

  RTT::OperationCaller<void(rosc::StatusCode)> WriteToLog;
  RTT::OperationCaller<std::vector<std::string>(void)> ReadExceptionLog;
  RTT::OperationCaller<void(void)> ClearExceptionLog;

  RTT::OperationCaller<void(void)>
      ServiceEmergencyStop;  // RobotService功能，机器人急停

  RTT::OperationCaller<void(void)>
      ServiceEmergencyRecover;  // RobotService功能，机器人急停

  RTT::OperationCaller<void(RobotCurrentState *const)> GetLayoutRobotPosition;

  RTT::OperationCaller<bool(std::vector<coordinate>)> LayoutSimulation;

 public:
  explicit GrpcServiceComponent(const std::string &name);

  GrpcServiceComponent();

  ~GrpcServiceComponent();

  bool configureHook() override;

  bool startHook() override;

  void updateHook() override;

  void stopHook() override;

  void cleanupHook() override;

  /**
   * @brief 执行命令并获得结果
   *
   * @param command
   * @return ExecStatus
   */
  inline ExecStatus ExeCommandAndGetResult(SingleCommand command);

  /**
   * @brief 测试GRPC是否联通，如果联通，则返回pong
   *
   * @param context
   * @param request
   * @param reply
   * @return Status
   */
  grpc::Status Ping(ServerContext *context,
                    const ::google::protobuf::Empty *request,
                    rosc::PingResponse *reply) override;

  /**
   * @brief 上电
   *
   * @param context
   * @param request
   * @param reply
   * @return Status
   */
  grpc::Status PowerOn(ServerContext *context,
                       const ::google::protobuf::Empty *request,
                       rosc::PowerOnResponse *reply) override;

  /**
   * @brief 下电
   *
   * @param context
   * @param request
   * @param reply
   * @return Status
   */
  grpc::Status PowerOff(ServerContext *context,
                        const ::google::protobuf::Empty *request,
                        rosc::PowerOffResponse *reply) override;

  /**
   * @brief 清理错误
   *
   * @param context
   * @param request
   * @param reply
   * @return Status
   */
  grpc::Status ClearError(ServerContext *context,
                          const ::google::protobuf::Empty *request,
                          rosc::ClearErrorResponse *reply) override;

  /**
   * @brief 急停
   *
   * @param context
   * @param request
   * @param reply
   * @return Status
   */
  grpc::Status EmergencyStop(ServerContext *context,
                             const rosc::EmergencyStopRequest *request,
                             rosc::EmergencyStopResponse *reply) override;

  /**
   * @brief 急停恢复
   *
   * @param context
   * @param request
   * @param reply
   * @return Status
   */
  grpc::Status EmergencyRecover(ServerContext *context,
                                const rosc::EmergencyRecoverRequest *request,
                                rosc::EmergencyRecoverResponse *reply) override;

  /**
   * @brief 启动Jog运动
   *
   * @param context
   * @param request
   * @param reply
   * @return Status
   */
  grpc::Status TeachJogStart(ServerContext *context,
                             const rosc::TeachJogStartRequest *request,
                             rosc::TeachJogStartResponse *reply) override;

  /**
   * @brief 停止Jog运动
   * @deprecated 暂时废弃掉，jog连续运动start存在问题，配套stop也不要使用
   * @param context
   * @param request
   * @param reply
   * @return Status
   */
  grpc::Status TeachJogStop(ServerContext *context,
                            const rosc::TeachJogStopRequest *request,
                            rosc::TeachJogStopResponse *reply) override;

  /**
   * @brief 单步示教动作，点动示教
   *
   * @param context
   * @param request
   * @param reply
   * @return grpc::Status
   */
  grpc::Status TeachMoveStep(ServerContext *context,
                             const rosc::TeachMoveStepRequest *request,
                             rosc::TeachMoveStepResponse *reply) override;
  /**
   * @brief 保存当前点位
   *
   * @param context
   * @param request
   * @param reply
   * @return Status
   */
  grpc::Status
  TeachSavePosition(ServerContext *context,
                    const rosc::TeachSavePositionRequest *request,
                    rosc::TeachSavePositionResponse *reply) override;

  /**
   * @brief 设置示教速度
   *
   * @param context
   * @param request
   * @param reply
   * @return Status
   */
  grpc::Status TeachSetVelocity(ServerContext *context,
                                const rosc::TeachSetVelocityRequest *request,
                                rosc::TeachSetVelocityResponse *reply) override;

  /**
   * @brief 获取示教速度
   *
   * @param context
   * @param request
   * @param reply
   * @return Status
   */
  grpc::Status TeachGetVelocity(ServerContext *context,
                                const rosc::TeachGetVelocityRequest *request,
                                rosc::TeachGetVelocityResponse *reply) override;

  /**
   * @brief 机械臂归零
   *
   * @param context
   * @param request
   * @param reply
   * @return Status
   */
  grpc::Status TeachMove(ServerContext *context,
                         const rosc::TeachMoveRequest *request,
                         rosc::TeachMoveResponse *reply) override;

  /**
   * @brief Jog运动至指定位置，可通过JogStop停止运动
   *
   * @param context
   * @param request
   * @param reply
   * @return Status
   */
  grpc::Status GoTeachJoint(ServerContext *context,
                            const rosc::GoTeachJointRequest *request,
                            rosc::GoTeachJointResponse *reply) override;

  /**
   * @brief 获取工控机配置数据
   *
   * @param context
   * @param request
   * @param reply
   * @return Status
   */
  grpc::Status RobotSetting(ServerContext *context,
                            const rosc::RobotSettingRequest *request,
                            rosc::RobotSettingResponse *reply) override;

  /**
   * @brief 保存工控机配置数据
   *
   * @param context
   * @param request
   * @param reply
   * @return Status
   */
  grpc::Status SaveJointSetting(ServerContext *context,
                                const rosc::SaveJointSettingRequest *request,
                                rosc::SaveJointSettingResponse *reply) override;

  /**
   * @brief 删除点位数据
   *
   * @param context
   * @param request
   * @param reply
   * @return Status
   */
  grpc::Status DeletePoint(ServerContext *context,
                           const rosc::DeletePointRequest *request,
                           rosc::DeletePointResponse *reply) override;

  /**
   * @brief 删除点位数据
   *
   * @param context
   * @param request
   * @param reply
   * @return Status
   */
  grpc::Status GetAllPointNames(ServerContext *context,
                                const rosc::GetAllPointNamesRequest *request,
                                rosc::GetAllPointNamesResponse *reply) override;

  /**
   * @brief 传输点位数据文件到客户端
   *
   * @param context
   * @param request
   * @param stream
   * @return Status
   */
  grpc::Status DownloadPointFile(
      ServerContext *context, const rosc::DownloadPointFileRequest *request,
      grpc::ServerWriter<rosc::DownloadPointFileResponse> *stream) override;

  /**
   * @brief IO设置
   *
   * @param context
   * @param request
   * @param reply
   * @return Status
   */
  grpc::Status SetIOOutputState(ServerContext *context,
                                const rosc::SetIOOutputStateRequest *request,
                                rosc::SetIOOutputStateResponse *reply) override;

  /**
   * @brief 获取当前所有IO状态
   *
   * @param context
   * @param request
   * @param reply
   * @return Status
   */
  grpc::Status GetIOOutputState(ServerContext *context,
                                const rosc::GetIOOutputStateRequest *request,
                                rosc::GetIOOutputStateResponse *reply) override;

  /**
   * @brief 获取当前所有IO状态
   *
   * @param context
   * @param request
   * @param reply
   * @return Status
   */
  grpc::Status GetIOInputState(
      ServerContext *context, const rosc::GetIOInputStateRequest *request,
      grpc::ServerWriter<rosc::GetIOInputStateResponse> *reply) override;

  /**
   * @brief 关闭IO输入状态流
   *
   * @param context
   * @param request
   * @param reply
   * @return Status
   */
  grpc::Status
  StopGetIOInputState(ServerContext *context,
                      const rosc::StopGetIOInputStateRequest *request,
                      rosc::StopGetIOInputStateResponse *reply) override;

  /**
   * @brief 执行单条指令
   *
   * @param context
   * @param request
   * @param reply
   * @return Status
   */
  grpc::Status TeachInstruction(ServerContext *context,
                                const rosc::TeachInstructionRequest *request,
                                rosc::TeachInstructionResponse *reply) override;

  /**
   * @brief 流式发送异常信息
   *
   * @param context
   * @param request
   * @param reply
   * @return Status
   */
  grpc::Status
  GetStreamException(ServerContext *context,
                     grpc::ServerReaderWriter<rosc::GetStreamExceptionResponse,
                                              rosc::GetStreamExceptionRequest>
                         *stream) override;

  /**
   * @brief Get the Exception object
   *
   * @param context
   * @param request
   * @param response
   * @return grpc::Status
   */
  grpc::Status GetException(ServerContext *context,
                            const rosc::GetExceptionRequest *request,
                            rosc::GetExceptionResponse *response) override;

  /**
   * @brief Delete the Exception object
   *
   * @param context
   * @param request
   * @param response
   * @return grpc::Status
   */
  grpc::Status
  DeleteException(ServerContext *context,
                  const rosc::DeleteExceptionRequest *request,
                  rosc::DeleteExceptionResponse *response) override;

  /**
   * @brief 伺服状态
   *
   * @param context
   * @param request
   * @param reply
   * @return Status
   */
  grpc::Status SlaveState(
      ServerContext *context,
      grpc::ServerReaderWriter<rosc::SlaveStateResponse,
                               rosc::SlaveStateRequest> *stream) override;

  /**
   * @brief 示教过程中，开启获取节点信息流。
   *
   * @param context
   * @param request
   * @param writer
   * @return grpc::Status
   */
  grpc::Status TeachStreamStartGetJoints(
      ServerContext *context, const ::google::protobuf::Empty *request,
      ::grpc::ServerWriter<::rosc::TeachStreamStartGetJointsResponse> *writer)
      override;

  /**
   * @brief 示教过程中，停止获取节点信息流。
   *
   * @param context
   * @param request
   * @param response
   * @return ::grpc::Status
   */
  grpc::Status TeachStreamStopGetJoints(
      ServerContext *context, const ::google::protobuf::Empty *request,
      rosc::TeachStreamStopGetJointsResponse *response) override;

  /**
   * @brief 读取速度配置值
   *
   * @param context
   * @param request
   * @param reply
   * @return Status
   */
  grpc::Status GetSpeedConfig(ServerContext *context,
                              const rosc::GetSpeedConfigRequest *request,
                              rosc::GetSpeedConfigResponse *response) override;

  /**
   * @brief 保存速度配置值
   *
   * @param context
   * @param request
   * @param reply
   * @return Status
   */
  grpc::Status
  SaveSpeedConfig(ServerContext *context,
                  const rosc::SaveSpeedConfigRequest *request,
                  rosc::SaveSpeedConfigResponse *response) override;
  grpc::Status
  RedifyControllerIP(ServerContext *context,
                     const rosc::RedifyControllerIPRequest *request,
                     rosc::RedifyControllerIPResponse *response) override;

  grpc::Status GetServoEncode(ServerContext *context,
                              const rosc::GetServoEncodeRequest *request,
                              rosc::GetServoEncodeResponse *response) override;

  /**
   * @brief 传输点位数据文件到客户端
   *
   * @param context
   * @param request
   * @param stream
   * @return Status
   */
  grpc::Status UpdatePointFile(
      ServerContext *context,
      grpc::ServerReaderWriter<rosc::UpdatePointFileResponse,
                               rosc::UpdatePointFileRequest> *stream) override;

 private:
  std::unique_ptr<Server> server_;
  bool grpc_enabled_;
  std::string service_addr_;
  double jogspeed_;
  std::queue<CommandExecResult> timestamps_buffer_;
  std::mutex timestamps_buffer_lock_;
  std::mutex out_to_service_port_lock_;
  std::mutex in_from_service_port_lock_;
  bool start_get_stream_joints_;
  // 保护获得节点信息流的锁，
  // 防止多个客户端同时获取节点信息流，避免控制器负载过高
  std::mutex teach_stream_get_joints_lock_;
  bool start_io_input_;
  std::atomic<bool> enable_jog_;
};
}  // namespace rosc

#endif  // MODULES_COMPONENTS_GRPC_SERVICE_COMPONENT_INCLUDE_GRPC_SERVICE_COMPONENT_H_
