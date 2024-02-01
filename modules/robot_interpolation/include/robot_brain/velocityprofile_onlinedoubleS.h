/********************************************************************
 * @file velocityprofile_onlinedoubleS.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-04-04
 *
 * @copyright Copyright (c) 2023 ROSC
 *
 ********************************************************************/
#ifndef MODULES_ROBOT_INTERPOLATION_INCLUDE_ROBOT_BRAIN_VELOCITYPROFILE_ONLINEDOUBLES_H_
#define MODULES_ROBOT_INTERPOLATION_INCLUDE_ROBOT_BRAIN_VELOCITYPROFILE_ONLINEDOUBLES_H_
#include "robot_brain/velocityprofile_base.h"
namespace rosc {
/*基础插补规划函数
 * inter_in   初始化输入；
 * inter_mid 初始化输出；
 *
 * return 0:正确； -1: 输入错误；
 * */
int OnlineDoubleS_project(OnlineDoubleS_in *inter_in,
                          OnlineDoubleS_mid *inter_mid,
                          Velocityprofile_out *inter_out);

/*基础插补数据计算
 * inter_in   输入
 * inter_mid 中间输入
 * inter_out 插补结果输出
 *
 *return 0:满足; 1: 运动
 * */
int OnlineDoubleS_computer(OnlineDoubleS_in *inter_in,
                           OnlineDoubleS_mid *inter_mid,
                           Velocityprofile_out *inter_out);

int UpdateOnlineDoubleS(OnlineDoubleS_in *inter_in,
                        OnlineDoubleS_mid *inter_mid,
                        Velocityprofile_out *inter_out, double Vmax,
                        double Amax, double Jmax);

int UpdateOnlineDoubleS_Vmax(OnlineDoubleS_in *inter_in,
                             OnlineDoubleS_mid *inter_mid,
                             Velocityprofile_out *inter_out, double Vmax);

int UpdateOnlineDoubleS_Amax(OnlineDoubleS_in *inter_in,
                             OnlineDoubleS_mid *inter_mid,
                             Velocityprofile_out *inter_out, double Amax);

int UpdateOnlineDoubleS_Jmax(OnlineDoubleS_in *inter_in,
                             OnlineDoubleS_mid *inter_mid,
                             Velocityprofile_out *inter_out, double Jmax);

double OnlineDoubleS_computer_prodfile_time(OnlineDoubleS_in *inter_in,
                                            Velocityprofile_out *inter_out,
                                            int stopstate);

int OnlineDoubleS_sync_prodfile(OnlineDoubleS_in *inter_in,
                                Velocityprofile_out *inter_out, double T,
                                double _T, int stopstate);
}  // namespace rosc
#endif  // MODULES_ROBOT_INTERPOLATION_INCLUDE_ROBOT_BRAIN_VELOCITYPROFILE_ONLINEDOUBLES_H_
