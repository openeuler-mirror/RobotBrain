/**
 * @file ethercat_component.h
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief
 * @version 0.1
 * @date 2021-09-08
 *
 * @copyright Copyright (c) 2021 ROSC
 *
 */
#ifndef MODULES_COMPONENTS_TEST_COM_INCLUDE_TEST_COM_H_
#define MODULES_COMPONENTS_TEST_COM_INCLUDE_TEST_COM_H_

#include <memory>
#include <string>

#include <rtt/RTT.hpp>
#include <robot_brain/core.hpp>

#define ETHERCAT_CTL_FRAME_QUEUE_BUFFER_SIZE                                   \
  409600  // 组件输入端口ctl frame buffer size

extern "C" {
int add(int x, int y);
}
namespace rosc {
class TestCom : public RTT::TaskContext {
 public:
  explicit TestCom(const std::string &name);
  TestCom();
  ~TestCom();
  bool configureHook();
  bool startHook();
  void updateHook();
  void stopHook();
  void cleanupHook();
};
}  // namespace rosc

#endif  // MODULES_COMPONENTS_TEST_COM_INCLUDE_TEST_COM_H_
