/**
 * @file config_tools.cpp
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief 配置工具的工具箱
 * @version 0.1
 * @date 2021-05-26
 *
 * @copyright Copyright (c) 2021 ROSC
 *
 */

#include <robot_brain/core/config_utils.h>
#include <glog/logging.h>
#include <iostream>
#include <string>

#ifdef _WIN32
#include <direct.h>
#include <io.h>
#elif __linux__
#include <stdarg.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

#ifdef _WIN32
#define ACCESS _access
#define MKDIR(a) _mkdir((a))
#elif __linux__
#define ACCESS access
#define MKDIR(a) mkdir((a), 0755)
#endif

/**
 * @brief 检查并创建目录
 * @param dir_name 目录名称
 * @return 是否创建成功
 */
bool CheckAndMkDir(const char *dir_name) {
  LOG_ASSERT(dir_name != NULL) << "dir_name is NULL";
  int i = 0;
  int ret;
  int len;
  char *p_dir;
  p_dir = strdup(dir_name);
  len = strlen(p_dir);
  // 创建中间目录
  for (i = 0; i < len; i++) {
    if (p_dir[i] == '\\' || p_dir[i] == '/') {
      // 处理例如"/root/test/"这样的路径
      if (i == 0) {
        p_dir[i] = '/';
        continue;
      }
      p_dir[i] = '\0';
      // 如果不存在，创建
      ret = ACCESS(p_dir, 0);
      if (ret != 0) {
        ret = MKDIR(p_dir);
        if (ret != 0) {
          LOG(ERROR) << "create dir name failed: <" << dir_name
                     << ">. Please check the permission.";
          free(p_dir);
          return false;
        }
      }
      // 支持linux,将所有\换成/
      p_dir[i] = '/';
    }
  }
  // 如果不存在，创建。处理最后一个，兼容最后目录的最后一层加/或者不加/
  ret = ACCESS(p_dir, 0);
  if (ret != 0) {
    ret = MKDIR(p_dir);
    if (ret != 0) {
      LOG(ERROR) << "create dir name failed: <" << dir_name
                 << ">. Please check the permission.";
      free(p_dir);
      return false;
    }
  }
  free(p_dir);
  return true;
}
