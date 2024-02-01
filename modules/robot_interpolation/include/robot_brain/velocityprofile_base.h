/********************************************************************
 * @file velocityprofile_base.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-03-21
 *
 * @copyright Copyright (c) 2023 ROSC
 *
 ********************************************************************/
#ifndef MODULES_ROBOT_INTERPOLATION_INCLUDE_ROBOT_BRAIN_VELOCITYPROFILE_BASE_H_
#define MODULES_ROBOT_INTERPOLATION_INCLUDE_ROBOT_BRAIN_VELOCITYPROFILE_BASE_H_
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "robot_brain/count.h"
#define VELOCITYPROFILE_STOP_TIME 2  // s  停止时间
#define VELOCITYPROFILE_STOP_DIS 5   // mm  停止距离
namespace rosc {
/********************************************************************
 * @brief 速度规划,停止的形式
 *
 ********************************************************************/
typedef enum VelocityprofileStopState {
  _velocityprofile_stop = 0,               //正常停止的规划
  _velocityprofile_forcestop,              // 强制停止
  _velocityprofile_suspend,                // 暂停
  _velocityprofile_continue,               // 继续
  _velocityprofile_run,                    // 运行
  _velocityprofile_run_constant_velocity,  //匀速运动
  _velocityprofile_constant_stop
} VelocityprofileStopState;

/********************************************************************
 * @brief VelocityProfileSpeedState
 *
 ********************************************************************/
enum {
  _velocity_profile_speed_up = 0,    // 加速
  _velocity_profile_speed_constant,  // 匀速
  _velocity_profile_speed_down,      // 减速
  _velocity_profile_speed_stop       // 停止
};

/********************************************************************
 * @brief
 *
 ********************************************************************/
typedef enum VelocityProfilePlanType {  // todo
  _velocity_profile_full = 0,           // 完整的速度规划
  _velocity_profile_down,
  _velocity_profile_full_add,
  _velocity_profile_full_vmax,
  _velocity_profile_no
} VelocityProfilePlanType;
/********************************************************************
 * @brief
 *
 ********************************************************************/
typedef struct DoubleS_in {
  double dt;         // 采样周期(s)
  double S;          // 位移(mm)或(rad)
  double s0;         // 起始位移
  double t0;         // 起始时间
  double v_start;    // 起始速度(mm/s)或(rad/s)
  double v_end;      // 终止速度(mm/s)或(rad/s)
  double acc_start;  // 起始加速度(mm/s^2)或(rad/s^2)
  double acc_end;    // 终止加速度(mm/s^2)或(rad/s^2)
  double v_max;      // 最大速度(mm/s)或(rad/s)
  double acc_max;    // 最大加速度(mm/s^2)或(rad/s^2)
  double jerk_max;   // 最大加加速度(mm/s^3)或(rad/s^3)
  double v_min;
  double acc_min;
  double jerk_min;

  //----------------------------
  double _S;  // 位移(mm)或(rad)
  double _T;
  double _Vmax;    // 最大速度(mm/s)或(rad/s)
  double _Amax;    // 最大加速度(mm/s^2)或(rad/s^2)
  double _Jmax;    // 最大加加速度(mm/s^3)或(rad/s^3)
  double _zone_S;  // 转弯区大小

  double vs_1;
  double as_1;

  // int StopState;  //停止方式
  VelocityprofileStopState stop_state;
  void Init(double dt, double S, double s0, double v_start, double v_end,
            double acc_start, double acc_end, double v_max, double acc_max,
            double jerk_max, double t0, VelocityprofileStopState stopstate);

  void Scale(double coef);
  void SetVelocityProfileStopState(VelocityprofileStopState state);
} DoubleS_in;

/********************************************************************
 * @brief
 *
 ********************************************************************/
typedef struct DoubleS_mid {
  double T;  // 总规划时间(s)

  double Taa_3;
  double Taa_2;
  double Taa_1;
  double S_1;

  double Taa;       // 加加速时间(s)
  double Tca;       // 匀加速时间(s)
  double Tda;       // 减加速时间(s)
  double Tv;        // 匀速时间(s)
  double Tad;       // 加减速速时间(s)
  double Tcd;       // 匀减速时间(s)
  double Tdd;       // 减减速时间(s)
  double ac_Vmax;   // 实际最大速度(mm/s)或(rad/s)
  double ac_Amaxa;  // 实际最大加速度(mm/s^2)或(rad/s^2)
  double ac_Amaxd;  // 实际最大减速度(mm/s^2)或(rad/s^2)
  double ac_Jmax;   // 实际最大加加速度(mm/s^3)或(rad/s^3)
  // doubleS规划方式，_velocity_profile_full，_velocity_profile_down，_velocity_profile_full_add
  int type;
  VelocityProfilePlanType profile_type;

  // DoubleS_mid();
  void Init();
  // int GetDoubleSType();
  VelocityProfilePlanType GetDoubleSType();
  void Scale(double coef);
} DoubleS_mid;

/********************************************************************
 * @brief 多项式规划
 *
 ********************************************************************/
typedef struct Polynomial_in {
  double S;   // 位移(mm)或(rad)
  double s0;  // 起始位移
  double v_start;
  double v_end;
  double acc_start;
  double acc_end;
  double jerk_start;
  double jerk_end;
  double t0;  //起始时间
  double t_end;
  double dt;
  int n;  //多项式阶数

  // int StopState;
  VelocityprofileStopState stop_state;

  void Init(double dt, double S, double s0, double v_start, double v_end,
            double acc_start, double acc_end, double jerk_start,
            double jerk_end, double t0, double t_end, int n,
            VelocityprofileStopState stopstate);
  void SetS(double s0, double S);
  void SetS_end(double S);
  void SetVelocityProfileStopState(VelocityprofileStopState state);
} Polynomial_in;

typedef struct Polynomial_mid {
  double a[10];
} Polynomial_mid;

/********************************************************************
 * @brief 在线S型速度规划
 *
 ********************************************************************/
typedef struct OnlineDoubleS_in {
  double dt;         // 采样周期(s)
  double S;          // 位移(mm)或(rad)
  double s0;         // 起始位移
  double t0;         // 起始时间
  double v_start;    // 起始速度(mm/s)或(rad/s)
  double v_end;      // 终止速度(mm/s)或(rad/s)
  double acc_start;  // 起始加速度(mm/s^2)或(rad/s^2)
  double acc_end;    // 终止加速度(mm/s^2)或(rad/s^2)
  double v_max;      // 最大速度(mm/s)或(rad/s)
  double acc_max;    // 最大加速度(mm/s^2)或(rad/s^2)
  double jerk_max;   // 最大加加速度(mm/s^3)或(rad/s^3)
  double v_min;
  double acc_min;
  double jerk_min;

  double _S;  // 位移(mm)或(rad)
  double _T;
  double _Vmax;  // 最大速度(mm/s)或(rad/s)
  double _Amax;  // 最大加速度(mm/s^2)或(rad/s^2)
  double _Jmax;  // 最大加加速度(mm/s^3)或(rad/s^3)

  VelocityprofileStopState stop_state;  //停止方式

  double start_moving_v;

  void Init(double dt, double S, double s0, double v_start, double v_end,
            double acc_start, double acc_end, double v_max, double acc_max,
            double jerk_max, double t0, VelocityprofileStopState stopstate);
  void UpdateS(double S);
  void UpdateVMax(double v_max);
  void UpdateAccMax(double acc_max);
  void UpdateJerkMax(double jerk_max);
  // void UpdateStartMovingV(Velocityprofile_out *vp_out);  // todo
  void UpdateStartMovingV(double v);
} OnlineDoubleS_in;

/********************************************************************
 * @brief
 *
 ********************************************************************/
typedef struct OnlineDoubleS_mid {
  double t_stop;
  double _ts;
  int ts_times;
  double Tj2a;
  double Td;
  double Tj2b;
  double h;
  int stop_flag;  // 0:降速停止 ; 1:升速停止

  // OnlineDoubleS_mid();
  void Init();
} OnlineDoubleS_mid;

/********************************************************************
 * @brief 速度规划的输出
 *
 ********************************************************************/
typedef struct Velocityprofile_out {
  double t;  // 插补时间(s)
  double s;  // 插补位置(mm)或(rad)
  double v;  // 插补速度(mm/s)或(rad/s)
  double a;  // 插补加速度(mm/s^2)或(rad/s^2)
  double j;  // 插补加加速度(mm/s^3)或(rad/s^3)

  int state;  // 0:stop; 1:move

  void Init();
  void SetT(double t);
  void InitVelocityProfileS(DoubleS_in *ds_in);
  void InitVelocityProfilePoly(Polynomial_in *poly_in);
  void InitVelocityProfileOnlineS(OnlineDoubleS_in *osin);
  int GetState();
} Velocityprofile_out;

/********************************************************************
 * @brief 初始化双S型速度规划
 *
 * @param ds_in
 * @param dt
 * @param S
 * @param s0
 * @param v_start
 * @param v_end
 * @param acc_start
 * @param acc_end
 * @param v_max
 * @param acc_max
 * @param jerk_max
 * @param t0
 * @param stopstate
 ********************************************************************/
void initDoubleS_in(DoubleS_in *ds_in, double dt, double S, double s0,
                    double v_start, double v_end, double acc_start,
                    double acc_end, double v_max, double acc_max,
                    double jerk_max, double t0, int stopstate);

// todo
int isVelocityprofileStopState(VelocityprofileStopState value);

int getDoubleSVelocityprofileTime(double vs, double *ve, double *vmax,
                                  double amax, double jmax, double S, double *T,
                                  double *_S, int stopstate);

int syncDoubleSVelocityprofile(double vs, double *ve, double *vmax, double amax,
                               double jmax, double T, double S, double _T,
                               double *_S, int stopstate);

double intVelocityprofileTime(double in, double ts);
}  // namespace rosc
#endif  // MODULES_ROBOT_INTERPOLATION_INCLUDE_ROBOT_BRAIN_VELOCITYPROFILE_BASE_H_
