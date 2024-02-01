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
#include <bits/stdint-uintn.h>
#include <dirent.h>
#include <gtest/gtest.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <ethercat_device.h>
#include <glog/logging.h>
#include <random>
#include <sstream>
#include <iostream>
#include <string>
#include <robot_brain/core.hpp>

// TEST(EthercatDevice, power_on) {
//   rosc::EthercatDevice ec_device(4000);
//   int master_id = 0;
//   // 初始化主站，从站以及相关配置
//   LOG(INFO) << "Start Master initialization.";
//   // TODO(maqun): 返回错误false的情况，需要处理异常和错误码的情况
//   if (!ec_device.CreateMaster(master_id)) {
//     LOG(ERROR) << "Create Master Error";
//     FAIL();
//   }
//   if (!ec_device.AddDomain()) {
//     LOG(ERROR) << "Add Domain Error";
//     FAIL();
//   }
//   if (!ec_device.ConfigSlaves()) {
//     LOG(ERROR) << "Config Slaves Error";
//     FAIL();
//   }
//   if (!ec_device.DomainRegisterList()) {
//     LOG(ERROR) << "Domain Register List Errror";
//     FAIL();
//   }
//   if (!ec_device.MasterActive()) {
//     LOG(ERROR) << "Master Active Errror";
//     FAIL();
//   }
//   if (!ec_device.DomainData()) {
//     LOG(ERROR) << "Domain Data Error!";
//     FAIL();
//   }
//   struct sched_param param = {};
//   param.sched_priority = sched_get_priority_max(SCHED_FIFO);

//   printf("Using priority %i.", param.sched_priority);
//   if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
//     perror("sched_setscheduler failed");
//   }
//   printf("Starting cyclic function.\n");
// }
