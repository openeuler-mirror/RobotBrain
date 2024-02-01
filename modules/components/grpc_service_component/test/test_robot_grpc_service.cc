/**
 * @file exception_service_test.cc
 * @author douyuanbo (douyuanbo@buaa.edu.cn)
 * @brief
 * @version 0.1
 * @date 2022-03-08
 *
 * @copyright Copyright (c) 2022 ROSC
 *
 */
#include <ethercat_component.h>
#include <grpc_service_component.h>
#include <exception_component.h>
#include <trajectory_component.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
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
#include <vector>
#include <future>

using namespace RTT;
using namespace RTT::mqueue;
using namespace RTT::types;

TEST(RobotGrpcService_Test, RobotGrpcService) {}
