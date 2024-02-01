/**
 * @file ping_client.hpp
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief ping
 * @version 0.1
 * @date 2021-06-01
 *
 * @copyright Copyright (c) 2021 ROSC
 *
 */
#ifndef MODULES_ROBOT_CTL_INCLUDE_ROBOT_RPC_CLIENT_HPP_
#define MODULES_ROBOT_CTL_INCLUDE_ROBOT_RPC_CLIENT_HPP_

#include <grpcpp/grpcpp.h>
#include <robot_grpc_service.grpc.pb.h>
#include <memory>
#include <string>
using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using rosc::PingRequest;
using rosc::PongReply;
using rosc::RobotGrpcService;

namespace rosc {
class RobotRpcClient {
 public:
  explicit RobotRpcClient(std::shared_ptr<Channel> channel)
      : stub_(RobotGrpcService::NewStub(channel)) {}

  // Assembles the client's payload, sends it and presents the response back
  // from the server.
  void Ping() {
    // Data we are sending to the server.
    PingRequest request;
    // Container for the data we expect from the server.
    PongReply reply;
    // Context for the client. It could be used to convey extra information to
    // the server and/or tweak certain RPC behaviors.
    ClientContext context;
    // The actual RPC.
    Status status = stub_->Ping(&context, request, &reply);
    // Act upon its status.
    if (status.ok()) {
      std::cout << reply.message() << std::endl;
    } else {
      std::cout << status.error_code() << ": " << status.error_message()
                << std::endl;
    }
  }

  void PowerOn() {
    PowerOnRequest request;
    PowerOnReply reply;
    ClientContext context;
    Status status = stub_->PowerOn(&context, request, &reply);
    if (status.ok()) {
      std::cout << reply.message() << std::endl;
    } else {
      std::cout << status.error_code() << ": " << status.error_message()
                << std::endl;
    }
  }

  void PowerOff() {
    PowerOffRequest request;
    PowerOffReply reply;
    ClientContext context;
    Status status = stub_->PowerOff(&context, request, &reply);
    if (status.ok()) {
      std::cout << reply.message() << std::endl;
    } else {
      std::cout << status.error_code() << ": " << status.error_message()
                << std::endl;
    }
  }

 private:
  std::unique_ptr<RobotGrpcService::Stub> stub_;
};
}  // namespace rosc

#endif  // MODULES_ROBOT_CTL_INCLUDE_ROBOT_RPC_CLIENT_HPP_
