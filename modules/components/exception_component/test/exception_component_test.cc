// /**
//  * @file exception_component_test.cc
//  * @author your name (you@domain.com)
//  * @brief
//  * @version 0.1
//  * @date 2022-02-21
//  *
//  * @copyright Copyright (c) 2022 ROSC
//  *
//  */
#include <exception_component.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <robot_brain/robot_exception.hpp>

#include <typeinfo>
#include <exception>


#include <rtt/RTT.hpp>
#include <unistd.h>

class ExceptionComponentTest : public RTT::TaskContext {
 protected:
  RTT::OutputPort<rosc::StatusCode> out_port_;

 public:
  ExceptionComponentTest()
      : ExceptionComponentTest(std::string("exception_test")) {}

  explicit ExceptionComponentTest(std::string const &name)
      : RTT::TaskContext(name), out_port_("exception_status_port") {
    this->ports()->addPort(out_port_).doc("输出异常编码");
    // this->setPeriod(1);
  }

  bool configureHook() {
    // 抛出RobotException异常
    throw rosc::RobotException(rosc::StatusCode::kSuccess);
    return true;
  }
  void updateHook() {
    // 抛出RobotException异常
    throw rosc::RobotException(rosc::StatusCode::kSuccess);
  }

  void stopHook() {}
  void cleanupHook() {}

  void exceptionHook(std::exception const &e) {
    // 将异常发送到异常处理组件
    try {
      rosc::RobotException &re =
          dynamic_cast<rosc::RobotException &>(const_cast<std::exception &>(e));
      out_port_.write(re.GetStatusCode());
      // 低等异常自己处理

      // 高等级异常抛出，调用
    } catch (std::bad_cast &b) {
      // 转化失败
    }
  }
};

TEST(ExceptionComponent_Test, WriteLog) {
  rosc::ExceptionHandle exception_handle("exception_handle");
  ExceptionComponentTest exception_server("exception_server");
  RTT::connectPorts(&exception_server, &exception_handle);
  RTT::connectPeers(&exception_server, &exception_handle);
  exception_server.configure();
  exception_handle.configure();
  LOG(INFO) << "config...";
  LOG(INFO) << "ready for log";
  exception_server.start();
  LOG(INFO) << "end";
  exception_handle.start();

  sleep(2);
  exception_handle.stop();
  exception_server.stop();
}
