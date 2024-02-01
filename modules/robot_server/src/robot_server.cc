/**
 * @file robot_server.cc
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief 机器人运行时环境入口程序
 * @version 0.1
 * @date 2023-07-20
 *
 * @copyright Copyright (c) 2023 ROSC
 *
 */
#include <cli/CLI11.hpp>
#include <ethercat_component.h>
#include <exception_component.h>
#include <interlock_component.h>
#include <robot_brain/core.hpp>
#include <robot_brain/robot_exception.hpp>
#include <robot_brain/version.h>
#include <robot_component.h>
#include <robot_service_component.h>
#include <rtt/TaskContext.hpp>
#include <trajectory_component.h>
int main(int argc, const char **argv) {
  // 设置命令行参数
#ifdef NDEBUG
  std::string m_config_dir = "/robot/config/";
#else
  std::string m_config_dir = "../../../../../data/test_data/robot/config/";
#endif
  // 输出启动版本
  rosc::LogVersion();
  LOG(INFO) << m_config_dir;
  rosc::Application::Init(m_config_dir.c_str());
  auto config = *rosc::Application::GetContext()->GetConfig();
  bool is_sr100 =
      config["init_global_config"]["yaskawa_service"]["is_sr100"].as<bool>();
  LOG(INFO) << "is_sr100 = " << is_sr100;
  // rosc::ServiceComponentSr100 sr100Service("ServiceComponentSr100");
  rosc::EthercatComponent ec("ethercat");
  rosc::RobotComponent robot("robot");
  rosc::TrajectoryComponent trajectory("trajectory");
  rosc::RobotServiceComponent robot_service("robot_service");
  rosc::ExceptionHandle exception("exception_handle");
  // rosc::GrpcServiceComponent grpcServiceComponent("grpc_serviceComponent");
  rosc::InterlockComponent interlock("interlock");

  RTT::connectPeers(&robot_service, &robot);
  RTT::connectPeers(&robot_service, &trajectory);
  RTT::connectPeers(&robot_service, &exception);
  RTT::connectPeers(&trajectory, &robot);
  RTT::connectPeers(&robot, &ec);
  // RTT::connectPeers(&grpcServiceComponent, &robot_service);
  // RTT::connectPeers(&grpcServiceComponent, &exception);
  // RTT::connectPeers(&grpcServiceComponent, &trajectory);
  RTT::connectPeers(&robot, &interlock);

  RTT::connectPorts(&robot, &ec);
  RTT::connectPorts(&trajectory, &robot);
  RTT::connectPorts(&robot_service, &trajectory);
  RTT::connectPorts(&robot_service, &ec);
  RTT::connectPorts(&robot_service, &interlock);
  RTT::connectPorts(&interlock, &ec);
  RTT::connectPorts(&exception, &ec);
  RTT::connectPorts(&exception, &robot);
  RTT::connectPorts(&exception, &trajectory);
  RTT::connectPorts(&exception, &robot_service);
  RTT::connectPorts(&exception, &interlock);
  // RTT::connectPorts(&grpcServiceComponent, &robot_service);
  // RTT::connectPorts(&grpcServiceComponent, &exception);

  if (!ec.configure()) {
    LOG(INFO) << "ec config failed.";
    return 0;
  }

  LOG(INFO) << "before robot config";
  if (!robot.configure()) {
    LOG(INFO) << "Robot config failed.";
    return 0;
  }
  LOG(INFO) << "config...";

  if (!exception.configure()) {
    LOG(INFO) << "exception config failed.";
  }

  if (!trajectory.configure()) {
    LOG(INFO) << "trajectory config failed.";
    return 0;
  }

  if (!robot_service.configure()) {
    LOG(INFO) << "Robot service config failed.";
    return 0;
  }
  if (!interlock.configure()) {
    LOG(INFO) << "interlock config failed.";
    return 0;
  }

  LOG(INFO) << "ready for start";
  ec.start();
  sleep(1);
  robot.start();
  exception.start();
  trajectory.start();
  robot_service.start();
  interlock.start();

  LOG(INFO) << "sr100 service start.";

  // std::string service_name =
  //     is_sr100 ? "ServiceComponentNxc100" : "ServiceComponentNxc100";
  // std::shared_ptr<rosc::ServiceComponentNxc100> serviceComponent = nullptr;
  // if (is_sr100) {
  //   serviceComponent =
  //       std::make_shared<rosc::ServiceComponentSr100>(service_name);
  // } else {
  //   serviceComponent =
  //       std::make_shared<rosc::ServiceComponentNxc100>(service_name);
  // }
  // RTT::connectPeers(serviceComponent.get(), &robot_service);
  // RTT::connectPorts(serviceComponent.get(), &robot_service);
  // RTT::connectPeers(serviceComponent.get(), &trajectory);
  // serviceComponent->configure();
  // serviceComponent->start();
  // grpcServiceComponent.start();
  // 阻塞，server后台运行
  while (true) {
    sleep(1);
  }
  return 0;
}
