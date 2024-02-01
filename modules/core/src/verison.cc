/**
 * @file verison.cc
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief
 * @version 0.1
 * @date 2023-08-18
 *
 * @copyright Copyright (c) 2023 ROSC
 *
 */
#include <robot_brain/version.h>
#include <robot_brain/config.h>
#include <glog/logging.h>

namespace rosc {
void LogVersion() {
  LOG(INFO) << "==================================";
  LOG(INFO) << "Version: " << VERSION_MAJOR << "." << VERSION_MINOR << "."
            << VERSION_PATCH;
  LOG(INFO) << "Git Branch: " << GIT_BRANCH;
  LOG(INFO) << "Git Hash: " << GIT_HASH;
  LOG(INFO) << "Build Timestamp: " << BUILD_TIMESTAMP;
#ifndef NDEBUG
  LOG(WARNING) << "Build Type: DEBUG";
#else
  LOG(INFO) << "Build Type: RELEASE";
#endif
  LOG(INFO) << "Robot Type: " << ROSC_ROBOT_TYPE;
  LOG(INFO) << "==================================";
}
}  // namespace rosc
