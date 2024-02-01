/**
 * @file config_utils.h
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief 配置工具的工具箱
 * @version 0.1
 * @date 2021-05-26
 *
 * @copyright Copyright (c) 2021 ROSC
 *
 */
#ifndef MODULES_CORE_INCLUDE_ROBOT_BRAIN_CORE_CONFIG_UTILS_H_  
#define MODULES_CORE_INCLUDE_ROBOT_BRAIN_CORE_CONFIG_UTILS_H_  

/**
 * @brief 检查并创建目录
 * @param dir_name 目录名称
 * @return 是否创建成功
 */
bool CheckAndMkDir(const char* dir_name);
#endif  // MODULES_CORE_INCLUDE_ROBOT_BRAIN_CORE_CONFIG_UTILS_H_  
