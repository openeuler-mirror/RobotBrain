// /**
//  * @file exception_service_test.cc
//  * @author your name (you@domain.com)
//  * @brief
//  * @version 0.1
//  * @date 2022-03-08
//  *
//  * @copyright Copyright (c) 2022 ROSC
//  *
//  */

// #include "exception_grpc_service_impl.h"
// #include <glog/logging.h>
// #include <gtest/gtest.h>
// #include <grpcpp/channel.h>
// #include <grpcpp/grpcpp.h>
// #include <grpcpp/impl/codegen/status.h>
// #include <exception_grpc_service.pb.h>
// #include <exception_grpc_service.grpc.pb.h>
// #include <memory>
// #include <string>

// using grpc::Channel;
// using grpc::ClientContext;
// using grpc::ClientReader;
// using grpc::Status;
// using rosc::ExceptionRequest;
// using rosc::ExceptionResponse;
// using rosc::ExceptionService;
// using rosc::PingRequest;
// using rosc::PongResponse;

// class ExceptionClient {
//  public:
//   explicit ExceptionClient(std::shared_ptr<Channel> channel)
//       : stub_(ExceptionService::NewStub(channel)) {}

//   void Ping() {
//     PingRequest request;
//     PongResponse response;
//     ClientContext context;
//     Status status = stub_->Ping(&context, request, &response);
//     if (status.ok()) {
//       std::cout << response.reply() << std::endl;
//     } else {
//       std::cout << status.error_code() << ": " << status.error_message()
//                 << std::endl;
//     }
//   }

//   void GetExceptionMsg() {
//     ClientContext context;
//     ExceptionRequest request;
//     std::unique_ptr<ClientReader<ExceptionResponse>> reader(
//         stub_->ReportException(&context, request));
//     ExceptionResponse exception_msg;
//     while (reader->Read(&exception_msg)) {
//       std::cout << "Get Exception msg: " << exception_msg.reply() <<
//       std::endl;
//     }
//     Status status = reader->Finish();
//     if (status.ok()) {
//       std::cout << "get exception finish!" << std::endl;
//     } else {
//       std::cout << "get exception error!" << std::endl;
//     }
//   }

//  private:
//   std::unique_ptr<ExceptionService::Stub> stub_;
// };

// // 线程的运行函数
// void *start_server(void *arg) {
//   rosc::ExceptionGrpcServiceImpl *robot_manager =
//       (rosc::ExceptionGrpcServiceImpl *)arg;
//   robot_manager->StartService();
//   return 0;
// }

// TEST(ExceptionServer_Test, ReportException) {
//   rosc::ExceptionGrpcServiceImpl server;
//   // server.StartService();
//   pthread_t sid;
//   int ret = pthread_create(&sid, NULL, start_server, &server);
//   sleep(2);
//   ExceptionClient client(grpc::CreateChannel(
//       "127.0.0.1:50051", grpc::InsecureChannelCredentials()));
//   client.Ping();
//   client.GetExceptionMsg();
//   server.Shutdown();
// }

// TEST(ExceptionServer_Test, StartExceptionServer) {
//   rosc::ExceptionGrpcServiceImpl server;
//   server.StartService();
// }
