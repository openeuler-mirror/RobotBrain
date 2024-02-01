/**
 * @file ethercat_component.cc
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief 用于测试和演示搭建的框架的orocos组件
 * @version 0.1
 * @date 2021-08-31
 *
 * @copyright Copyright (c) 2021 ROSC
 *
 */
#include <robot_brain/core/default_values.h>
#include <bits/stdint-uintn.h>
#include <test_com.h>
#include <rtt/os/main.h>
#include <glog/logging.h>
#include <memory>
#include <fstream>
#include <rtt/os/Time.hpp>
#include <robot_brain/core.hpp>
#include <rtt/ConnPolicy.hpp>
#include <rtt/FlowStatus.hpp>
#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Property.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/OperationCaller.hpp>
#include <rtt/Port.hpp>
#include <rtt/Activity.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>

extern "C" {
int add(int x, int y) { return x + y + 2; }
}

namespace rosc {

TestCom::TestCom() : TestCom(std::string("Ethercat")) {}

TestCom::~TestCom() {}

TestCom::TestCom(const std::string &name)
    : RTT::TaskContext(name, PreOperational) {
  rosc::Config config = *rosc::Application::GetContext()->GetConfig();
  // RTT::Seconds seconds = RTT::nsecs_to_Seconds(
  //     config["bus_config"]["communication_cycle"].as<RTT::nsecs>());
  LOG(INFO) << "TestCom, get periodddddddd：" << this->getPeriod();
  std::shared_ptr<int> j = std::make_shared<int>(23);
  j.use_count();
  LOG(INFO) << "value  " << *j.get();
}

bool TestCom::configureHook() {}

/**
 * @brief 组件启动的回调函数
 *
 * @return true
 * @return false
 */
bool TestCom::startHook() { return true; }

void TestCom::updateHook() {}

void TestCom::stopHook() {}

void TestCom::cleanupHook() {}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(HelloWorld)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(rosc::TestCom)
}  // namespace rosc
