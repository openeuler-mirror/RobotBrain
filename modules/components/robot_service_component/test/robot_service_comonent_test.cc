/**
 * @file robot_service_comonent_test.cc
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief
 * @version 0.1
 * @date 2022-03-17
 *
 * @copyright Copyright (c) 2022 ROSC
 *
 */

#include <cstdlib>
#include <ethercat_component.h>
#include <kdl/jntarray.hpp>
#include <robot_component.h>
#include <trajectory_component.h>
#include <robot_service_component.h>
#include <exception_component.h>
#include <interlock_component.h>
#include <gtest/gtest.h>
#include <glog/logging.h>
#include <unistd.h>
#include <vector>
#include <thread>  // NOLINT
#include <future>  // NOLINT
#include <cstdio>
#include <chrono>  // NOLINT
#include <kdl/frames.hpp>
#include <rtt/FlowStatus.hpp>
#include <rtt/base/rtt-base-fwd.hpp>
#include <rtt/transports/mqueue/MQTemplateProtocolBase.hpp>
#include <robot_brain/core.hpp>
#include <rtt/BufferPolicy.hpp>
#include <rtt/ConnPolicy.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/transports/mqueue/MQLib.hpp>
#include <rtt/base/InputPortInterface.hpp>
#include <rtt/types/TemplateTypeInfo.hpp>
#include <rtt/transports/mqueue/MQTemplateProtocol.hpp>
#include <rtt/transports/mqueue/MQSerializationProtocol.hpp>
#include <rtt/transports/mqueue/MQSendRecv.hpp>
#include "robot_brain/config.h"
#include "robot_brain/command_types.hpp"
using namespace RTT;          // NOLINT
using namespace RTT::mqueue;  // NOLINT
using namespace RTT::types;   // NOLINT

/**
 * @brief 测试命令下发和结果反馈
 *
 */
TEST(RobotServiceComponentTest, test_robot_servcie_command_send) {}
