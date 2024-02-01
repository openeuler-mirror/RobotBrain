/********************************************************************
 * @file velocityprofile.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-03-22
 *
 * @copyright Copyright (c) 2023 ROSC
 *
 ********************************************************************/
#ifndef MODULES_ROBOT_INTERPOLATION_INCLUDE_ROBOT_BRAIN_VELOCITYPROFILE_H_
#define MODULES_ROBOT_INTERPOLATION_INCLUDE_ROBOT_BRAIN_VELOCITYPROFILE_H_
#include "robot_brain/velocityprofile_base.h"
namespace rosc {
typedef enum VelocityProfileType {
  _doubleS = 0,
  _polynomial1,
  _polynomial5,
  _polynomial7,
  _onlinedoubleS
} VelocityProfileType;

typedef struct VelocityProfile {
  DoubleS_in doubleS_in;
  DoubleS_mid doubleS_mid;

  Polynomial_in polynomial_in;
  Polynomial_mid polynomial_mid;

  OnlineDoubleS_in online_doubleS_in;
  OnlineDoubleS_mid online_doubleS_mid;

  Velocityprofile_out velocityprofile_out;
  VelocityProfileType ProfileType;
  int is_stop_zone;
  double speed_coeff;  // 0~1,最大速度比例系数

  VelocityProfileType GetVelocityProfileType();
} VelocityProfile;

/*速度规划
 * vp:规划数据
 * dt:采样周期(s)
 * S:规划长度
 * vs:起始速度
 * ve:终止速度
 * as:起始加速度
 * ae:终止加速度
 *
 * Vmax:规划最大速度
 * Amax:规划最大加速度
 * Jmax:规划最大加加速度
 * t0:起始时间
 * stopstate:规划的停止方式
 *_velocityprofile_stop(0);_velocityprofile_forcestop(1),
 *_velocityprofile_suspend(2)
 *
 *return 0:right, other: wrong
 * */
int init_VelocityProfile_DoubleS(VelocityProfile *vp, double dt, double S,
                                 double s0, double v_start, double v_end,
                                 double acc_start, double acc_end, double v_max,
                                 double acc_max, double jerk_max, double t0,
                                 VelocityprofileStopState stopstate,
                                 double speed_coeff, int is_stop_zone);

int sync_VelocityProfile_DoubleS(VelocityProfile *vp, double T_src,
                                 double T_dest);

int project_VelocityProfile_DoubleS(VelocityProfile *vp);

int init_VelocityProfile_Polynomial1(VelocityProfile *vp, double dt, double S,
                                     double s0, double t0, double tend,
                                     VelocityprofileStopState stopstate,
                                     double speed_coeff);

int init_VelocityProfile_Polynomial5(VelocityProfile *vp, double dt, double S,
                                     double s0, double vs, double ve, double as,
                                     double ae, double t0, double tend,
                                     VelocityprofileStopState stopstate,
                                     double speed_coeff);

int init_VelocityProfile_Polynomial7(VelocityProfile *vp, double dt, double S,
                                     double s0, double vs, double ve, double as,
                                     double ae, double js, double je, double t0,
                                     double tend,
                                     VelocityprofileStopState stopstate,
                                     double speed_coeff);

/*在线速度规划
 * vp:规划数据
 * dt:采样周期(s)
 * S:规划长度
 * vs:起始速度
 * ve:终止速度
 * as:起始加速度
 * ae:终止加速度
 *
 * Vmax:规划最大速度
 * Amax:规划最大加速度
 * Jmax:规划最大加加速度
 * t0:起始时间
 * stopstate:规划的停止方式_velocityprofile_stop(0);_velocityprofile_forcestop(1),
 *_velocityprofile_suspend(2)
 *
 *return 0:right, other: wrong
 * */
int init_VelocityProfile_OnlineDoubleS(VelocityProfile *vp, double dt, double S,
                                       double s0, double vs, double ve,
                                       double as, double ae, double Vmax,
                                       double Amax, double Jmax, double t0,
                                       VelocityprofileStopState stopstate,
                                       double speed_coeff, int is_stop_zone);

int sync_VelocityProfile_OnlineDoubleS(VelocityProfile *vp, double T_src,
                                       double T_dest);

int project_VelocityProfile_OnlineDoubleS(VelocityProfile *vp);

int updateVelocityProfile_Vmax1(VelocityProfile *vp, double speed_coeff);

int updateVelocityProfile(VelocityProfile *vp, double Vmax, double Amax,
                          double Jmax);

int updateVelocityProfile_Vmax(VelocityProfile *vp, double Vmax);

int updateVelocityProfile_Amax(VelocityProfile *vp, double Amax);

int updateVelocityProfile_Jmax(VelocityProfile *vp, double Jmax);

int renew_VelocityProfileCartesian(VelocityProfile *vp_pos,
                                   VelocityProfile *vp_pose, double pS,
                                   double pvmax, double pve, double qS,
                                   double qvmax, double qve, double speed_coeff,
                                   double ptend, double qtend, int stopstate);

int renew_VelocityProfileJoint(VelocityProfile *vp, double *S, double *vmax,
                               double *ve, double speed_coeff, double *tend,
                               int dof, int stopstate);

Velocityprofile_out *getVelocityProfile(VelocityProfile *vp);

Velocityprofile_out *getVelocityProfile_t(VelocityProfile *vp, double t);

/*获取当前位移
 * */
double getVelocityProfile_displacement(VelocityProfile *vp);

/*获取规划长度*/
double getVelocityProfile_totaldisplacement(VelocityProfile *vp);

/*获取初始规划长度*/
double getVelocityProfile_startdisplacement(VelocityProfile *vp);

void setVelocityProfile_startdisplacement(VelocityProfile *vp, double s0);

/*获取剩余位移
 * */
double getVelocityProfile_remaindisplacement(VelocityProfile *vp);

double getVelocityProfile_sampleperiod(VelocityProfile *vp);

/*获取剩余时间
 * */
double getVelocityProfile_remaintime(VelocityProfile *vp);

double getVelocityProfile_distance(VelocityProfile *vp, double t);

double getVelocityProfile_time(VelocityProfile *vp, double s);

double getVelocityProfileSpeedCoeff(VelocityProfile *vp);

int getVelocityProfileStateType(VelocityProfile *vp);

double getVelocityProfileTime(VelocityProfile *vp);

int syncVelocityProfile(VelocityProfile *vp, double T_src, double T_dest);

}  // namespace rosc
#endif  // MODULES_ROBOT_INTERPOLATION_INCLUDE_ROBOT_BRAIN_VELOCITYPROFILE_H_
