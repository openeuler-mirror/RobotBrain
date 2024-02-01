/**
 * @file tools.hpp
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief
 * @version 0.1
 * @date 2022-11-02
 *
 * @copyright Copyright (c) 2022 ROSC
 *
 */
#ifndef MODULES_CORE_INCLUDE_ROBOT_BRAIN_CORE_TOOLS_HPP_
#define MODULES_CORE_INCLUDE_ROBOT_BRAIN_CORE_TOOLS_HPP_
#include <bits/stdint-intn.h>
/**
 * @brief 返回val的符号
 *
 * @param val 待判断值
 * @return int 当val为负值时返回-1，当val为正值时返回+1，为0时
 */
int sign(int64_t val) { return ((val > 0) - (val < 0)); }

#endif  // MODULES_CORE_INCLUDE_ROBOT_BRAIN_CORE_TOOLS_HPP_
