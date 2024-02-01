/**
 * @file ethercat_test.cc
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2021-06-29
 *
 * @copyright Copyright (c) 2021 ROSC
 *
 */
#include "robot_brain/core/application.h"
#include <chrono>
#include <ethercat_component.h>
#include <bits/stdint-uintn.h>
#include <dirent.h>
#include <gtest/gtest.h>
#include <rtt/os/TimeService.hpp>
#include <sys/stat.h>
#include <sys/types.h>
#include <thread>
#include <time.h>
#include <unistd.h>
#include <ethercat_device.h>
#include <glog/logging.h>
#include <random>
#include <sstream>
#include <iostream>
#include <string>
#include <iterator>
#include <rtt/OutputPort.hpp>
#include <rtt/TaskContext.hpp>
#include <robot_brain/core.hpp>

class EthercatComponentTest : public RTT::TaskContext {
 protected:
  RTT::OutputPort<rosc::EthercatCtlFrame> out_port_;
  RTT::InputPort<rosc::EthercatStatusFrame> input_port_;

  int counter = 100;

 public:
  EthercatComponentTest()
      : EthercatComponentTest(std::string("ethercat_test")) {}
  explicit EthercatComponentTest(std::string const &name)
      : RTT::TaskContext(name), out_port_("ctl_frame_buffer_port"),
        input_port_("status_frame_port", Orocos::ConnPolicy::data()) {
    this->setActivity(
        new Orocos::Activity(ORO_SCHED_RT, RTT::os::IncreasePriority, 0.004));
    this->ports()->addPort(out_port_);
    this->ports()->addPort(input_port_);
    rosc::EthercatCtlFrame ctlSampleFrame;
    out_port_.setDataSample(ctlSampleFrame);
  }
  bool configureHook() { return true; }
  bool startHook() { return true; }
  void updateHook() {
    rosc::EthercatStatusFrame status_frame;
    RTT::FlowStatus status = input_port_.read(status_frame);
    if (status == RTT::NoData) {
      LOG(INFO) << "===== no data ====";
    } else {
      LOG(INFO) << status_frame;
    }
  }

  void stopHook() {}
  void cleanupHook() {}
};

/**
 * @brief 测试模拟模式下的通信
 *
 */
TEST(EthercatComponentTest, test_driver_communicate_on_simulation) {
  std::string m_config_dir = "../../../../../../data/test_data/robot/config/";
  rosc::Application::Init(m_config_dir.c_str());
  rosc::EthercatComponent ec("ethercat");
  EthercatComponentTest test_ec;
  RTT::connectPorts(&test_ec, &ec);
  ASSERT_TRUE(ec.configure());
  ASSERT_TRUE(test_ec.configure());
  ASSERT_TRUE(ec.start());
  ASSERT_TRUE(test_ec.start());
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  ASSERT_TRUE(ec.stop());
  ASSERT_TRUE(test_ec.stop());
}

#ifdef PANASONIC_2_SALVE

TEST(EthercatComponentTest, test_component_input_port) {
  rosc::EthercatComponent ec("ethercat");
  EthercatComponentTest test_ec("etheract_test");
  bool connectrs = RTT::connectPorts(&test_ec, &ec);
  test_ec.configure();
  // 发送数据给该组件ec
  ASSERT_TRUE(test_ec.start());
  ASSERT_TRUE(ec.configure());
  ASSERT_TRUE(ec.start());
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  ASSERT_TRUE(ec.stop());
  ASSERT_TRUE(test_ec.stop());
}

#endif  // PANASONIC_2_SALVE
