/********************************************************************
 * @file velocityprofile_doubleS.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-04-03
 *
 * @copyright Copyright (c) 2023 ROSC
 *
 ********************************************************************/
#ifndef MODULES_ROBOT_INTERPOLATION_INCLUDE_ROBOT_BRAIN_VELOCITYPROFILE_DOUBLES_H_
#define MODULES_ROBOT_INTERPOLATION_INCLUDE_ROBOT_BRAIN_VELOCITYPROFILE_DOUBLES_H_
#include "robot_brain/velocityprofile_base.h"
namespace rosc {
int DoubleS_project(DoubleS_in *inter_in, DoubleS_mid *inter_mid);

/*基础插补数据计算
 * inter_in   输入
 * inter_mid 中间输入
 * inter_out 插补结果输出
 *
 *return 0:满足; 1: 运动
 * */
int DoubleS_computer(DoubleS_in *inter_in, DoubleS_mid *inter_mid,
                     Velocityprofile_out *inter_out);

double DoubleS_computer_distance(DoubleS_in *inter_in, DoubleS_mid *inter_mid,
                                 double t);

double DoubleS_computer_time(DoubleS_in *inter_in, DoubleS_mid *inter_mid,
                             double s);

double DoubleS_computer_prodfile_time(DoubleS_in *inter_in,
                                      Velocityprofile_out *inter_out,
                                      int stopstate);

int DoubleS_sync_prodfile(DoubleS_in *inter_in, Velocityprofile_out *inter_out,
                          double T, double _T, int stopstate);

/*适用于起始和终止速度、加速度为0的情况*/
int DoubleSTimeToVelocity(double s, double time, double vmax, double amax,
                          double jmax, double *velocity);
}  // namespace rosc

#endif  // MODULES_ROBOT_INTERPOLATION_INCLUDE_ROBOT_BRAIN_VELOCITYPROFILE_DOUBLES_H_
