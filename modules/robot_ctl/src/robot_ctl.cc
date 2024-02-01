/**
 * @file robot_ctl.cc
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief rctl，机器人命令行工具
 * @version 0.1
 * @date 2021-05-31
 *
 * @copyright Copyright (c) 2021 ROSC
 *
 */
#include <robot_ctl.h>
#include <type_traits>
#include <cli/CLI11.hpp>
#include <robot_rpc_client.hpp>

int main(int argc, char** argv) {
  CLI::App app{"rctl, a command line tools for robot"};
  app.require_subcommand(1);
  std::string target_addr{"127.0.0.1:50051"};
  // app.add_option("-t,--target", target_addr, "指定服务器地址", true);
  // client
  rosc::RobotRpcClient r_client(
      grpc::CreateChannel(target_addr, grpc::InsecureChannelCredentials()));

  auto help_sub = app.add_subcommand("help", "显示帮助文档");
  help_sub->parse_complete_callback([]() { throw CLI::CallForHelp(); });

  // ping subcommand
  CLI::App* ping_sub = app.add_subcommand("ping", "测试服务器是否联通");
  ping_sub->callback([&]() { r_client.Ping(); });

  // power subcommand
  CLI::App* power_sub = app.add_subcommand("power", "电源相关功能");
  power_sub->add_subcommand("on", "上电")->callback([&]() {
    r_client.PowerOn();
  });

  power_sub->add_subcommand("off", "下电")->callback([&]() {
    r_client.PowerOff();
  });

  // add new options/flags here
  CLI11_PARSE(app, argc, argv);
  return 0;
}
