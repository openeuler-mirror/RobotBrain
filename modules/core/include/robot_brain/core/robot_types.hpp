/**
 * @file robot_types.hpp
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief
 * @version 0.1
 * @date 2021-11-03
 *
 * @copyright Copyright (c) 2021 ROSC
 *
 */
#ifndef MODULES_CORE_INCLUDE_ROBOT_BRAIN_CORE_ROBOT_TYPES_HPP_
#define MODULES_CORE_INCLUDE_ROBOT_BRAIN_CORE_ROBOT_TYPES_HPP_

namespace rosc {
/**
 * @brief 运动模式
 *
 */
typedef enum {
  ABSOLUTE = 0,  ///< 绝对的运动方式
  INCREMENT      ///< 增量的运动方式
} MoveMode;
};  // namespace rosc

#endif  // MODULES_CORE_INCLUDE_ROBOT_BRAIN_CORE_ROBOT_TYPES_HPP_
