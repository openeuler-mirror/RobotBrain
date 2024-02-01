/**
 * @file ethercat_device_test.cc
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief
 * @version 0.1
 * @date 2021-10-16
 *
 * @copyright Copyright (c) 2021 ROSC
 *
 */
#include <chrono>
#include <thread>
#include <gtest/gtest.h>
#include <glog/logging.h>
#include <vector>
#include <cstdio>
#include <rtt/FlowStatus.hpp>
#include <rtt/base/rtt-base-fwd.hpp>
#include <robot_brain/core.hpp>
#include <rtt/BufferPolicy.hpp>
#include <rtt/ConnPolicy.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Activity.hpp>

class TestCom2 : public RTT::TaskContext {
 protected:
  RTT::InputPort<int> input_port_;

 public:
  TestCom2() : TestCom2("test_com") {}
  explicit TestCom2(const std::string &name)
      : RTT::TaskContext(name, PreOperational),
        input_port_("test_port", RTT::ConnPolicy::buffer(50)) {
    this->setActivity(
        new RTT::Activity(ORO_SCHED_RT, RTT::os::HighestPriority, 0, 1));
    // 上游组件端口
    this->ports()->addEventPort(input_port_);
  }

  ~TestCom2() {}
  bool configureHook() { return true; }
  bool startHook() { return true; }
  void updateHook() {
    int t;
    auto status = input_port_.read(t);
    if (status == RTT::NewData) {
      LOG(INFO) << "update " << t;
    } else {
      LOG(INFO) << "update old " << t;
    }
    usleep(500000);
    this->trigger();
  }
  void stopHook() {}
  void cleanupHook() {}
};

// class TestCom1 : public RTT::TaskContext {
//  protected:
//   RTT::InputPort<int> input_port_;

//  public:
//   TestCom1() : TestCom1("test_com") {}
//   explicit TestCom1(const std::string &name)
//       : RTT::TaskContext(name, PreOperational),
//         input_port_("test_port", RTT::ConnPolicy::buffer(50)) {
//     this->setActivity(
//         new RTT::Activity(ORO_SCHED_RT, RTT::os::HighestPriority, 1, 1));
//     // 上游组件端口
//     this->ports()->addPort(input_port_);
//   }

//   ~TestCom1() {}
//   bool configureHook() { return true; }
//   bool startHook() { return true; }
//   void updateHook() {}
//   void stopHook() {}
//   void cleanupHook() {}
// };

void func() {
  RTT::OutputPort<int> test_com_out_port("test_port");
  TestCom2 testCom2;
  testCom2.configure();

  ASSERT_TRUE(testCom2.getPort("test_port")->connectTo(&test_com_out_port));
  bool irs = test_com_out_port.connectedTo(testCom2.getPort("test_port"));
  ASSERT_TRUE(irs);

  LOG(INFO) << "START============";
  testCom2.start();
  usleep(1000000);
  LOG(INFO) << "START=====END====";

  int j = 0;

  while (true) {
    test_com_out_port.write(j);
    LOG(INFO) << "write " << j;
    usleep(200000);
    j++;
    if (j > 50) {
      break;
    }
  }
  while (true) {
  }
  LOG(INFO) << "HELLO";
}

TEST(Com_Test, test1) { func(); }