/********************************************************************
 * @file velocityprofile.cc
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-04-03
 *
 * @copyright Copyright (c) 2023 ROSC
 *
 ********************************************************************/
#include <stdio.h>
#include <iostream>
#include <string.h>
#include "robot_brain/config.h"
#include "robot_brain/velocityprofile.h"
#include "robot_brain/velocityprofile_base.h"
#include "robot_brain/velocityprofile_doubleS.h"
#include "robot_brain/velocityprofile_onlinedoubleS.h"
#include "robot_brain/velocityprofile_polynomial.h"
namespace rosc {
VelocityProfileType VelocityProfile::GetVelocityProfileType() {
  return this->ProfileType;
}

/********************************************************************
 * @brief
 *
 * @param vp
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
 * @param speed_coeff
 * @param is_stop_zone
 * @return int
 ********************************************************************/
int init_VelocityProfile_DoubleS(VelocityProfile *vp, double dt, double S,
                                 double s0, double v_start, double v_end,
                                 double acc_start, double acc_end, double v_max,
                                 double acc_max, double jerk_max, double t0,
                                 VelocityprofileStopState stopstate,
                                 double speed_coeff, int is_stop_zone) {
  // 初始化输入
  vp->doubleS_in.Init(dt, S, s0, v_start, v_end, acc_start, acc_end, v_max,
                      acc_max, jerk_max, t0, stopstate);
  // 初始化中段
  vp->doubleS_mid.Init();
  // 初始化输出
  vp->velocityprofile_out.InitVelocityProfileS(&vp->doubleS_in);
  vp->ProfileType = VelocityProfileType::_doubleS;
  //根据速度系数设置实际规划速度
  vp->speed_coeff = speed_coeff;
  vp->doubleS_in.v_max *= speed_coeff;
  vp->doubleS_in.v_min *= speed_coeff;
  vp->doubleS_in.v_end *= speed_coeff;
  //设置曲线结尾是否衔接转弯区
  vp->is_stop_zone = is_stop_zone;

  //获取规划时间,如果规划不满足要求自动修正规划入参数（最大速度和终止速度）
  vp->doubleS_in._T = DoubleS_computer_prodfile_time(
      &(vp->doubleS_in), &(vp->velocityprofile_out), stopstate);
  if (vp->doubleS_in._T < 0) {
    return -1;
  }
  return 0;
}

/********************************************************************
 * @brief
 *
 * @param vp
 * @param T_src
 * @param T_dest
 * @return int
 ********************************************************************/
int syncVelocityProfile(VelocityProfile *vp, double T_src, double T_dest) {
  int err = 0;
  switch (vp->ProfileType) {
  case _doubleS:
    err = DoubleS_sync_prodfile(&(vp->doubleS_in), &(vp->velocityprofile_out),
                                T_src, T_dest, vp->doubleS_in.stop_state);
    break;
  case _polynomial1:
  case _polynomial5:
  case _polynomial7:
    printf("_polynomial invalid!\n");
    break;
  case _onlinedoubleS:
    err = OnlineDoubleS_sync_prodfile(&(vp->online_doubleS_in),
                                      &(vp->velocityprofile_out), T_src, T_dest,
                                      vp->online_doubleS_in.stop_state);
    break;
  default:
    break;
  }
  return err;
}

int sync_VelocityProfile_DoubleS(VelocityProfile *vp, double T_src,
                                 double T_dest) {
  return syncVelocityProfile(vp, T_src, T_dest);
}

int project_VelocityProfile_DoubleS(VelocityProfile *vp) {
  return DoubleS_project(&(vp->doubleS_in), &(vp->doubleS_mid));
}

int init_VelocityProfile_Polynomial1(VelocityProfile *vp, double dt, double S,
                                     double s0, double t0, double tend,
                                     rosc::VelocityprofileStopState stopstate,
                                     double speed_coeff) {
  vp->polynomial_in.Init(dt, S, s0, 0, 0, 0, 0, 0, 0, t0, tend, 1, stopstate);

  vp->velocityprofile_out.InitVelocityProfilePoly(&(vp->polynomial_in));

  vp->ProfileType = _polynomial1;
  vp->speed_coeff = speed_coeff;
  // vp->polynomial_in.tend *=speed_coeff;
  return Polynomial_project(&(vp->polynomial_in), &(vp->polynomial_mid));
}

int init_VelocityProfile_Polynomial5(VelocityProfile *vp, double dt, double S,
                                     double s0, double vs, double ve, double as,
                                     double ae, double t0, double tend,
                                     rosc::VelocityprofileStopState stopstate,
                                     double speed_coeff) {
  vp->polynomial_in.Init(dt, S, s0, vs, ve, as, ae, 0, 0, t0, tend, 5,
                         stopstate);

  vp->velocityprofile_out.InitVelocityProfilePoly(&(vp->polynomial_in));
  vp->ProfileType = _polynomial5;
  vp->speed_coeff = speed_coeff;
  // vp->polynomial_in.tend *=speed_coeff;
  return Polynomial_project(&(vp->polynomial_in), &(vp->polynomial_mid));
}

int init_VelocityProfile_Polynomial7(VelocityProfile *vp, double dt, double S,
                                     double s0, double vs, double ve, double as,
                                     double ae, double js, double je, double t0,
                                     double tend,
                                     VelocityprofileStopState stopstate,
                                     double speed_coeff) {
  vp->polynomial_in.Init(dt, S, s0, vs, ve, as, ae, js, je, t0, tend, 7,
                         stopstate);

  vp->velocityprofile_out.InitVelocityProfilePoly(&(vp->polynomial_in));
  vp->ProfileType = _polynomial7;
  vp->speed_coeff = speed_coeff;
  // vp->polynomial_in.tend *=speed_coeff;
  return Polynomial_project(&(vp->polynomial_in), &(vp->polynomial_mid));
}

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
                                       double speed_coeff, int is_stop_zone) {
  vp->online_doubleS_in.Init(dt, S, s0, vs, ve, as, ae, Vmax, Amax, Jmax, t0,
                             stopstate);
  // initOnlineDoubleS_in(&(vp->online_doubleS_in), dt, S, s0, vs, ve, as, ae,
  //                      Vmax, Amax, Jmax, t0, stopstate);
  vp->online_doubleS_mid.Init();
  // initOnlineDoubleS_mid(&(vp->online_doubleS_mid));
  vp->velocityprofile_out.InitVelocityProfileOnlineS(&vp->online_doubleS_in);
  // initVelocityprofile_out_OnS(&(vp->online_doubleS_in),
  //                             &(vp->velocityprofile_out));
  vp->ProfileType = _onlinedoubleS;
  //根据速度系数设置实际规划速度
  vp->speed_coeff = speed_coeff;
  vp->online_doubleS_in.v_max *= speed_coeff;
  vp->online_doubleS_in.v_min *= speed_coeff;
  vp->online_doubleS_in.v_end *= speed_coeff;
  //设置曲线结尾是否衔接转弯区
  vp->is_stop_zone = is_stop_zone;

  vp->online_doubleS_in._T = OnlineDoubleS_computer_prodfile_time(
      &(vp->online_doubleS_in), &(vp->velocityprofile_out), stopstate);
  if (vp->online_doubleS_in._T < 0) {
    return -1;
  }
  return 0;
}

int sync_VelocityProfile_OnlineDoubleS(VelocityProfile *vp, double T_src,
                                       double T_dest) {
  return syncVelocityProfile(vp, T_src, T_dest);
}

int project_VelocityProfile_OnlineDoubleS(VelocityProfile *vp) {
  return OnlineDoubleS_project(&(vp->online_doubleS_in),
                               &(vp->online_doubleS_mid),
                               &(vp->velocityprofile_out));
}

int updateVelocityProfile_Vmax1(VelocityProfile *vp, double speed_coeff) {
  // int profile_type = getVelocityProfileType(vp);
  int profile_type = vp->GetVelocityProfileType();
  int ret = 0;
  switch (profile_type) {
  case _doubleS:
    printf("_doubleS invalid!\n");
    break;
  case _polynomial5:
    printf("_polynomial5 invalid!\n");
    break;
  case _polynomial7:
    printf("_polynomial7 invalid!\n");
    break;
  case _onlinedoubleS:
    ret = UpdateOnlineDoubleS_Vmax(
        &(vp->online_doubleS_in), &(vp->online_doubleS_mid),
        &(vp->velocityprofile_out), vp->online_doubleS_in._Vmax * speed_coeff);
    break;
  default:
    break;
  }
  return ret;
}

int updateVelocityProfile(VelocityProfile *vp, double Vmax, double Amax,
                          double Jmax) {
  // int profile_type = getVelocityProfileType(vp);
  int profile_type = vp->GetVelocityProfileType();
  int ret = 0;
  switch (profile_type) {
  case _doubleS:
    printf("_doubleS invalid!\n");
    break;
  case _polynomial5:
    printf("_polynomial5 invalid!\n");
    break;
  case _polynomial7:
    printf("_polynomial7 invalid!\n");
    break;
  case _onlinedoubleS:
    ret =
        UpdateOnlineDoubleS(&(vp->online_doubleS_in), &(vp->online_doubleS_mid),
                            &(vp->velocityprofile_out), Vmax, Amax, Jmax);
    break;
  default:
    break;
  }
  return ret;
}

int updateVelocityProfile_Vmax(VelocityProfile *vp, double Vmax) {
  // int profile_type = getVelocityProfileType(vp);
  int profile_type = vp->GetVelocityProfileType();
  int ret = 0;
  switch (profile_type) {
  case _doubleS:
    printf("_doubleS invalid!\n");
    break;
  case _polynomial5:
    printf("_polynomial5 invalid!\n");
    break;
  case _polynomial7:
    printf("_polynomial7 invalid!\n");
    break;
  case _onlinedoubleS:
    ret = UpdateOnlineDoubleS_Vmax(&(vp->online_doubleS_in),
                                   &(vp->online_doubleS_mid),
                                   &(vp->velocityprofile_out), Vmax);
    break;
  default:
    break;
  }
  return ret;
}

int updateVelocityProfile_Amax(VelocityProfile *vp, double Amax) {
  // int profile_type = getVelocityProfileType(vp);
  int profile_type = vp->GetVelocityProfileType();
  int ret = 0;
  switch (profile_type) {
  case _doubleS:
    printf("_doubleS invalid!\n");
    break;
  case _polynomial5:
    printf("_polynomial5 invalid!\n");
    break;
  case _polynomial7:
    printf("_polynomial7 invalid!\n");
    break;
  case _onlinedoubleS:
    ret = UpdateOnlineDoubleS_Amax(&(vp->online_doubleS_in),
                                   &(vp->online_doubleS_mid),
                                   &(vp->velocityprofile_out), Amax);
    break;
  default:
    break;
  }
  return ret;
}

int updateVelocityProfile_Jmax(VelocityProfile *vp, double Jmax) {
  // int profile_type = getVelocityProfileType(vp);
  int profile_type = vp->GetVelocityProfileType();
  int ret = 0;
  switch (profile_type) {
  case _doubleS:
    printf("_doubleS invalid!\n");
    break;
  case _polynomial5:
    printf("_polynomial5 invalid!\n");
    break;
  case _polynomial7:
    printf("_polynomial7 invalid!\n");
    break;
  case _onlinedoubleS:
    ret = UpdateOnlineDoubleS_Jmax(&(vp->online_doubleS_in),
                                   &(vp->online_doubleS_mid),
                                   &(vp->velocityprofile_out), Jmax);
    break;
  default:
    break;
  }
  return ret;
}

/*重新规划
 * vp:规划结果（返回值）
 * vp_before:当前规划（vp可以与vp_before相同）
 *S:  规划位移
 *vmax: 重规划速度（<0:采用之前规划速度）
 *ve: 终止速度
 *ae: 终止加速度
 *tend:规划时间（仅在多项式规划时有效）
 *规划的停止方式_velocityprofile_stop(0);_velocityprofile_forcestop(1),
 *_velocityprofile_suspend(2)
 * */
int renew_VelocityProfileCartesian(VelocityProfile *vp_pos,
                                   VelocityProfile *vp_pose, double pS,
                                   double pvmax, double pve, double qS,
                                   double qvmax, double qve, double speed_coeff,
                                   double ptend, double qtend,
                                   VelocityprofileStopState stopstate) {
  // int profile_type = getVelocityProfileType(vp_pos);
  rosc::VelocityProfileType profile_type = vp_pos->GetVelocityProfileType();
  int err1 = 0;
  int err2 = 0;
  double pos_time;
  double pose_time;
  double max_time;
  switch (profile_type) {
  case _doubleS:
    if (pvmax <= 0) {
      pvmax = vp_pos->doubleS_in._Vmax;
    }
    if (qvmax <= 0) {
      qvmax = vp_pose->doubleS_in._Vmax;
    }
    if (pve < 0) {
      if (0 == vp_pos->is_stop_zone) {
        pve = 0;
      } else {
        pve = pvmax;
      }
    }
    if (qve < 0) {
      if (0 == vp_pose->is_stop_zone) {
        qve = 0;
      } else {
        qve = qvmax;
      }
    }
    if (stopstate < 0) {
      stopstate = vp_pos->doubleS_in.stop_state;
    }
    if (speed_coeff < 0) {
      speed_coeff = vp_pos->speed_coeff;
    }
    err1 = init_VelocityProfile_DoubleS(
        vp_pos, vp_pos->doubleS_in.dt, pS, vp_pos->velocityprofile_out.s,
        vp_pos->velocityprofile_out.v, pve, vp_pos->velocityprofile_out.a, 0,
        pvmax, vp_pos->doubleS_in._Amax, vp_pos->doubleS_in._Jmax,
        vp_pos->velocityprofile_out.t, stopstate, speed_coeff,
        vp_pos->is_stop_zone);
    err2 = init_VelocityProfile_DoubleS(
        vp_pose, vp_pose->doubleS_in.dt, qS, vp_pose->velocityprofile_out.s,
        vp_pose->velocityprofile_out.v, qve, vp_pose->velocityprofile_out.a, 0,
        qvmax, vp_pose->doubleS_in._Amax, vp_pose->doubleS_in._Jmax,
        vp_pose->velocityprofile_out.t, stopstate, speed_coeff,
        vp_pose->is_stop_zone);
    if ((0 != err1) || (0 != err2)) {
      // return ERR_VELOCITYRENEWPLANNING;
      return -1;
    }
    //同步处理
    pos_time = getVelocityProfileTime(vp_pos);
    if (pos_time < 0) {
      // return ERR_VELOCITYRENEWPLANNING;
      return -1;
    }
    pose_time = getVelocityProfileTime(vp_pose);
    if (pose_time < 0) {
      // return ERR_VELOCITYRENEWPLANNING;
      return -1;
    }
    max_time = 0;
    if (pos_time > pose_time) {
      max_time = pos_time;
    } else {
      max_time = pose_time;
    }
    max_time = intVelocityprofileTime(max_time, vp_pos->doubleS_in.dt);
    if (0 != sync_VelocityProfile_DoubleS(vp_pos, pos_time, max_time)) {
      std::cout
          << "Warning: "
          << ("renew_VelocityProfileCartesian: sync_VelocityProfile_DoubleS "
              "failure, Ignore the synchronization\n");
    }
    if (0 != sync_VelocityProfile_DoubleS(vp_pose, pose_time, max_time)) {
      std::cout
          << "Warning: "
          << ("renew_VelocityProfileCartesian: sync_VelocityProfile_DoubleS "
              "failure, Ignore the synchronization\n");
    }
    //规划
    err1 = project_VelocityProfile_DoubleS(vp_pos);
    err2 = project_VelocityProfile_DoubleS(vp_pose);
    if ((0 != err1) || (0 != err2)) {
      // return ERR_VELOCITYRENEWPLANNING;
      return -1;
    }
    break;
  case VelocityProfileType::_polynomial5:
    if (_velocityprofile_forcestop == stopstate) {
      if (ptend < 0) {
        ptend = VELOCITYPROFILE_STOP_TIME;
      }
      if (qtend < 0) {
        qtend = VELOCITYPROFILE_STOP_TIME;
      }
      if (stopstate < 0) {
        stopstate = vp_pos->doubleS_in.stop_state;
      }
      if (speed_coeff < 0) {
        speed_coeff = vp_pos->speed_coeff;
      }
      // err1 = init_VelocityProfile_Ploynomial5(
      //     vp_pos, vp_pos->polynomial_in.dt, pS,
      //     vp_pos->velocityprofile_out.s, vp_pos->velocityprofile_out.v, pve,
      //     vp_pos->velocityprofile_out.a, 0, vp_pos->velocityprofile_out.t,
      //     ptend, stopstate, speed_coeff);
      // err2 = init_VelocityProfile_Ploynomial5(
      //     vp_pose, vp_pose->polynomial_in.dt, qS,
      //     vp_pose->velocityprofile_out.s, vp_pose->velocityprofile_out.v,
      //     qve, vp_pose->velocityprofile_out.a, 0,
      //     vp_pose->velocityprofile_out.t, qtend, stopstate, speed_coeff);
      if ((0 != err1) || (0 != err2)) {
        // return ERR_VELOCITYRENEWPLANNING;
        return -1;
      }
    }
    break;
  case _polynomial7:
    if (_velocityprofile_forcestop == stopstate) {
      if (ptend < 0) {
        ptend = VELOCITYPROFILE_STOP_TIME;
      }
      if (qtend < 0) {
        qtend = VELOCITYPROFILE_STOP_TIME;
      }
      if (stopstate < 0) {
        stopstate = vp_pos->doubleS_in.stop_state;
      }
      if (speed_coeff < 0) {
        speed_coeff = vp_pos->speed_coeff;
      }
      // err1 = init_VelocityProfile_Ploynomial7(
      //     vp_pos, vp_pos->polynomial_in.dt, pS,
      //     vp_pos->velocityprofile_out.s, vp_pos->velocityprofile_out.v, pve,
      //     vp_pos->velocityprofile_out.a, 0, 0, 0,
      //     vp_pos->velocityprofile_out.t, ptend, stopstate, speed_coeff);
      // err2 = init_VelocityProfile_Ploynomial7(
      //     vp_pose, vp_pose->polynomial_in.dt, qS,
      //     vp_pose->velocityprofile_out.s, vp_pose->velocityprofile_out.v,
      //     qve, vp_pose->velocityprofile_out.a, 0, 0, 0,
      //     vp_pose->velocityprofile_out.t, qtend, stopstate, speed_coeff);
      if ((0 != err1) || (0 != err2)) {
        // return ERR_VELOCITYRENEWPLANNING;
        return -1;
      }
    }
    break;
  case _onlinedoubleS:
    if (pvmax <= 0) {
      pvmax = vp_pos->online_doubleS_in._Vmax;
    }
    if (qvmax <= 0) {
      qvmax = vp_pose->online_doubleS_in._Vmax;
    }
    if (pve < 0) {
      if (0 == vp_pos->is_stop_zone) {
        pve = 0;
      } else {
        pve = pvmax;
      }
    }
    if (qve < 0) {
      if (0 == vp_pose->is_stop_zone) {
        qve = 0;
      } else {
        qve = qvmax;
      }
    }
    if (stopstate < 0) {
      stopstate = vp_pos->online_doubleS_in.stop_state;
    }
    if (speed_coeff < 0) {
      speed_coeff = vp_pos->speed_coeff;
    }
    err1 = init_VelocityProfile_OnlineDoubleS(
        vp_pos, vp_pos->online_doubleS_in.dt, pS, vp_pos->velocityprofile_out.s,
        vp_pos->velocityprofile_out.v, pve, vp_pos->velocityprofile_out.a, 0,
        pvmax, vp_pos->online_doubleS_in._Amax, vp_pos->online_doubleS_in._Jmax,
        vp_pos->velocityprofile_out.t, stopstate, speed_coeff,
        vp_pos->is_stop_zone);
    err2 = init_VelocityProfile_OnlineDoubleS(
        vp_pose, vp_pose->online_doubleS_in.dt, qS,
        vp_pose->velocityprofile_out.s, vp_pose->velocityprofile_out.v, qve,
        vp_pose->velocityprofile_out.a, 0, qvmax,
        vp_pose->online_doubleS_in._Amax, vp_pose->online_doubleS_in._Jmax,
        vp_pose->velocityprofile_out.t, stopstate, speed_coeff,
        vp_pose->is_stop_zone);
    if ((0 != err1) || (0 != err2)) {
      // return ERR_VELOCITYRENEWPLANNING;
      return -1;
    }
    if (!((_velocityprofile_forcestop == stopstate) ||
          (_velocityprofile_suspend == stopstate))) {  // 停止不做同步处理
      // 同步处理
      double pos_time = getVelocityProfileTime(vp_pos);
      if (pos_time < 0) {
        // return ERR_VELOCITYRENEWPLANNING;
        return -1;
      }
      double pose_time = getVelocityProfileTime(vp_pose);
      if (pose_time < 0) {
        // return ERR_VELOCITYRENEWPLANNING;
        return -1;
      }

      double max_time = 0;
      if (pos_time > pose_time) {
        max_time = pos_time;
      } else {
        max_time = pose_time;
      }
      max_time = intVelocityprofileTime(max_time, vp_pos->online_doubleS_in.dt);
      if (0 != sync_VelocityProfile_OnlineDoubleS(vp_pos, pos_time, max_time)) {
        std::cout << "Warning: "
                  << ("renew_VelocityProfileCartesian: "
                      "sync_VelocityProfile_OnlineDoubleS failure, Ignore the "
                      "synchronization\n");
      }
      if (0 !=
          sync_VelocityProfile_OnlineDoubleS(vp_pose, pose_time, max_time)) {
        std::cout << "Warning: "
                  << ("renew_VelocityProfileCartesian: "
                      "sync_VelocityProfile_OnlineDoubleS failure, Ignore the "
                      "synchronization\n");
      }
    }

    //规划
    err1 = project_VelocityProfile_OnlineDoubleS(vp_pos);
    err2 = project_VelocityProfile_OnlineDoubleS(vp_pose);
    if ((0 != err1) || (0 != err2)) {
      // return ERR_VELOCITYRENEWPLANNING;
      return -1;
    }
    break;
  default:
    break;
  }

  return 0;
}

int renew_VelocityProfileJoint(VelocityProfile *vp, double *S, double *vmax,
                               double *ve, double speed_coeff, double *tend,
                               int dof, VelocityprofileStopState stopstate) {
  // int profile_type = getVelocityProfileType(&(vp[0]));
  VelocityProfileType profile_type = vp[0].GetVelocityProfileType();
  int i = 0;
  double tmp[kDof];
  memset(tmp, 0, sizeof(tmp));
  double _vmax[kDof];
  memset(_vmax, 0, sizeof(_vmax));
  double _ve[kDof];
  memset(_ve, 0, sizeof(_ve));
  double _tend[kDof];
  memset(_tend, 0, sizeof(_tend));
  double Tmax = 0;
  for (i = 0; i < dof; i++) {
    switch (profile_type) {
    case _doubleS:
      if (NULL == vmax) {
        _vmax[i] = vp[i].doubleS_in._Vmax;
      } else {
        _vmax[i] = vmax[i];
      }
      if (NULL == ve) {
        if (0 == vp[i].is_stop_zone) {
          _ve[i] = 0;
        } else {
          _ve[i] = _vmax[i];
        }

      } else {
        _ve[i] = ve[i];
      }
      if (stopstate < 0) {
        stopstate = vp[i].doubleS_in.stop_state;
      }
      if (speed_coeff < 0) {
        speed_coeff = vp[i].speed_coeff;
      }

      if (0 != init_VelocityProfile_DoubleS(
                   &(vp[i]), vp[i].doubleS_in.dt, S[i],
                   vp[i].velocityprofile_out.s, vp[i].velocityprofile_out.v,
                   _ve[i], vp[i].velocityprofile_out.a, 0, _vmax[i],
                   vp[i].doubleS_in._Amax, vp[i].doubleS_in._Jmax,
                   vp[i].velocityprofile_out.t, stopstate, speed_coeff,
                   vp[i].is_stop_zone)) {
        // return ERR_VELOCITYRENEWPLANNING;
        return -1;
      }

      tmp[i] = getVelocityProfileTime(&(vp[i]));
      if (tmp[i] < 0) {
        // return ERR_VELOCITYRENEWPLANNING;
        return -1;
      }
      if (Tmax < tmp[i]) {
        Tmax = tmp[i];
      }

      break;
    case _polynomial5:
      if (_velocityprofile_forcestop == stopstate) {
        if (NULL == tend) {
          _tend[i] = VELOCITYPROFILE_STOP_TIME;
        } else {
          _tend[i] = tend[i];
        }
        if (stopstate < 0) {
          stopstate = vp[i].doubleS_in.stop_state;
        }
        if (speed_coeff < 0) {
          speed_coeff = vp[i].speed_coeff;
        }
        // if (0 != init_VelocityProfile_Ploynomial5(
        //              &(vp[i]), vp[i].polynomial_in.dt, S[i],
        //              vp[i].velocityprofile_out.s,
        //              vp[i].velocityprofile_out.v, _ve[i],
        //              vp[i].velocityprofile_out.a, 0,
        //              vp[i].velocityprofile_out.t, _tend[i], stopstate,
        //              speed_coeff)) {
        //   // return ERR_VELOCITYRENEWPLANNING;
        //   return -1;
        // }
      }
      break;
    case _polynomial7:
      if (_velocityprofile_forcestop == stopstate) {
        if (NULL == tend) {
          _tend[i] = VELOCITYPROFILE_STOP_TIME;
        } else {
          _tend[i] = tend[i];
        }
        if (stopstate < 0) {
          stopstate = vp[i].doubleS_in.stop_state;
        }
        if (speed_coeff < 0) {
          speed_coeff = vp[i].speed_coeff;
        }
        // if (0 != init_VelocityProfile_Ploynomial7(
        //              &(vp[i]), vp[i].polynomial_in.dt, S[i],
        //              vp[i].velocityprofile_out.s,
        //              vp[i].velocityprofile_out.v, _ve[i],
        //              vp[i].velocityprofile_out.a, 0, 0, 0,
        //              vp[i].velocityprofile_out.t, _tend[i], stopstate,
        //              speed_coeff)) {
        //   // return ERR_VELOCITYRENEWPLANNING;
        //   return -1;
        // }
      }
      break;
    case _onlinedoubleS:
      if (NULL == vmax) {
        _vmax[i] = vp[i].online_doubleS_in._Vmax;
      } else {
        _vmax[i] = vmax[i];
      }
      if (NULL == ve) {
        _ve[i] = vp[i].online_doubleS_in._Vmax;
      } else {
        _ve[i] = ve[i];
      }
      if (stopstate < 0) {
        stopstate = vp[i].online_doubleS_in.stop_state;
      }
      if (speed_coeff < 0) {
        speed_coeff = vp[i].speed_coeff;
      }
      if (0 != init_VelocityProfile_OnlineDoubleS(
                   &(vp[i]), vp[i].online_doubleS_in.dt, S[i],
                   vp[i].velocityprofile_out.s, vp[i].velocityprofile_out.v,
                   _ve[i], vp[i].velocityprofile_out.a, 0, _vmax[i],
                   vp[i].online_doubleS_in._Amax, vp[i].online_doubleS_in._Jmax,
                   0, stopstate, speed_coeff, vp[i].is_stop_zone)) {
        // return ERR_VELOCITYRENEWPLANNING;
        return -1;
      }
      tmp[i] = getVelocityProfileTime(&(vp[i]));
      if (tmp[i] < 0) {
        // return ERR_VELOCITYRENEWPLANNING;
        return -1;
      }
      if (Tmax < tmp[i]) {
        Tmax = tmp[i];
      }
      break;
    default:
      break;
    }
  }
  switch (profile_type) {
  case _doubleS:
    Tmax = intVelocityprofileTime(Tmax, vp[0].doubleS_in.dt);
    break;
  case _onlinedoubleS:
    Tmax = intVelocityprofileTime(Tmax, vp[0].online_doubleS_in.dt);
    break;
  }
  for (i = 0; i < dof; i++) {
    switch (profile_type) {
    case _doubleS:
      if (0 != sync_VelocityProfile_DoubleS(&(vp[i]), tmp[i], Tmax)) {
        std::cout
            << "Warning："
            << ("renew_VelocityProfileJoint: sync_VelocityProfile_DoubleS "
                "failure, Ignore the synchronization\n");
      }

      if (0 != project_VelocityProfile_DoubleS(&(vp[i]))) {
        return -1;
      }
      break;

    case _onlinedoubleS:
      if (0 != sync_VelocityProfile_OnlineDoubleS(&(vp[i]), tmp[i], Tmax)) {
        std::cout << "Warning: "
                  << ("renew_VelocityProfileJoint: "
                      "sync_VelocityProfile_OnlineDoubleS "
                      "failure, Ignore the synchronization\n");
      }
      if (0 != project_VelocityProfile_OnlineDoubleS(&(vp[i]))) {
        // return ERR_VELOCITYRENEWPLANNING;
        return -1;
      }
      break;
    default:
      break;
    }
  }

  return 0;
}

Velocityprofile_out *getVelocityProfile(VelocityProfile *vp) {
  VelocityProfileType profile_type = vp->GetVelocityProfileType();
  switch (profile_type) {
  case _doubleS:
    DoubleS_computer(&(vp->doubleS_in), &(vp->doubleS_mid),
                     &(vp->velocityprofile_out));
    break;
  case _polynomial1:
    Polynomial_computer(&(vp->polynomial_in), &(vp->polynomial_mid),
                        &(vp->velocityprofile_out));
    break;
  case _polynomial5:
    Polynomial_computer(&(vp->polynomial_in), &(vp->polynomial_mid),
                        &(vp->velocityprofile_out));
    break;
  case _polynomial7:
    Polynomial_computer(&(vp->polynomial_in), &(vp->polynomial_mid),
                        &(vp->velocityprofile_out));
    break;
  case _onlinedoubleS:
    OnlineDoubleS_computer(&(vp->online_doubleS_in), &(vp->online_doubleS_mid),
                           &(vp->velocityprofile_out));
    break;
  default:
    break;
  }
  return &(vp->velocityprofile_out);
}

Velocityprofile_out *getVelocityProfile_t(VelocityProfile *vp, double t) {
  VelocityProfileType profile_type = vp->GetVelocityProfileType();
  switch (profile_type) {
  case _doubleS:
    // setVelocityprofile_out_t(&(vp->velocityprofile_out), t -
    // vp->doubleS_in.dt);
    vp->velocityprofile_out.SetT(t - vp->doubleS_in.dt);
    DoubleS_computer(&(vp->doubleS_in), &(vp->doubleS_mid),
                     &(vp->velocityprofile_out));
    break;
  case _polynomial1:
  case _polynomial5:
  case _polynomial7:
    // setVelocityprofile_out_t(&(vp->velocityprofile_out),
    //                          t - vp->polynomial_in.dt);
    // Polynomial_computer(&(vp->polynomial_in), &(vp->polynomial_mid),
    //                     &(vp->velocityprofile_out));
    break;
  case _onlinedoubleS:
    printf("_onlinedoubleS invalid!\n");
    break;
  default:
    break;
  }
  return &(vp->velocityprofile_out);
}
double getVelocityProfile_displacement(VelocityProfile *vp) {
  return vp->velocityprofile_out.s;
}

/*获取规划长度*/
double getVelocityProfile_totaldisplacement(VelocityProfile *vp) {
  double ret = 0;
  switch (vp->ProfileType) {
  case _doubleS:
    ret = vp->doubleS_in.S + vp->doubleS_mid.S_1;
    break;
  case _polynomial1:
  case _polynomial5:
  case _polynomial7:
    ret = vp->polynomial_in.S;
    break;
  case _onlinedoubleS:
    ret = vp->online_doubleS_in.S;
    break;
  default:
    ret = 0;
    break;
  }

  return ret;
}

/*获取初始规划长度*/
double getVelocityProfile_startdisplacement(VelocityProfile *vp) {
  double ret = 0;
  switch (vp->ProfileType) {
  case _doubleS:
    ret = vp->doubleS_in.s0;
    break;
  case _polynomial1:
  case _polynomial5:
  case _polynomial7:
    ret = vp->polynomial_in.s0;
    break;
  case _onlinedoubleS:
    ret = vp->online_doubleS_in.s0;
    break;
  default:
    ret = 0;
    break;
  }

  return ret;
}

void setVelocityProfile_startdisplacement(VelocityProfile *vp, double s0) {
  switch (vp->ProfileType) {
  case _doubleS:
    vp->doubleS_in.s0 = s0;
    vp->velocityprofile_out.s = vp->doubleS_in.s0;
    break;
  case _polynomial1:
  case _polynomial5:
  case _polynomial7:
    vp->polynomial_in.s0 = s0;
    vp->velocityprofile_out.s = vp->polynomial_in.s0;
    break;
  case _onlinedoubleS:
    vp->online_doubleS_in.s0 = s0;
    vp->velocityprofile_out.s = vp->online_doubleS_in.s0;
    break;
  default:
    break;
  }
}

/*获取剩余位移
 * */
double getVelocityProfile_remaindisplacement(VelocityProfile *vp) {
  double ret = 0;
  switch (vp->ProfileType) {
  case _doubleS:
    ret = (vp->doubleS_in.s0 + vp->doubleS_in.S + vp->doubleS_mid.S_1) -
          vp->velocityprofile_out.s;
    break;
  case _polynomial1:
  case _polynomial5:
  case _polynomial7:
    ret = (vp->polynomial_in.s0 + vp->polynomial_in.S) -
          vp->velocityprofile_out.s;
    break;
  case _onlinedoubleS:
    ret = (vp->online_doubleS_in.s0 + vp->online_doubleS_in.S) -
          vp->velocityprofile_out.s;
    break;
  default:
    ret = 0;
    break;
  }
  if (ret < 0) {
    ret = 0;
  }
  return ret;
}

double getVelocityProfile_sampleperiod(VelocityProfile *vp) {
  double dt = 0;
  switch (vp->ProfileType) {
  case _doubleS:
    dt = vp->doubleS_in.dt;
    break;
  case _polynomial1:
  case _polynomial5:
  case _polynomial7:
    dt = vp->polynomial_in.dt;
    break;
  case _onlinedoubleS:
    dt = vp->online_doubleS_in.dt;
    break;
  default:
    break;
  }
  return dt;
}

/*获取剩余时间
 * */
double getVelocityProfile_remaintime(VelocityProfile *vp) {
  double ret = 0;
  switch (vp->ProfileType) {
  case _doubleS:
    ret = (vp->doubleS_in.t0 + vp->doubleS_mid.T + vp->doubleS_mid.Taa_1 +
           vp->doubleS_mid.Taa_2 + vp->doubleS_mid.Taa_3) -
          vp->velocityprofile_out.t;
    break;
  case _polynomial1:
  case _polynomial5:
  case _polynomial7:
    // ret = (vp->polynomial_in.t0 + vp->polynomial_in.tend) -
    //       vp->velocityprofile_out.t;
    // if (ret < 0) {
    //   ret = 0;
    // }
    break;
  case _onlinedoubleS:
    ret = 0;
    printf("_onlinedoubleS invalid!\n");
    break;
  default:
    ret = 0;
    break;
  }

  return ret;
}

double getVelocityProfile_distance(VelocityProfile *vp, double t) {
  double dis = 0;
  switch (vp->ProfileType) {
  case _doubleS:
    dis = DoubleS_computer_distance(&(vp->doubleS_in), &(vp->doubleS_mid), t);
    break;
  case _polynomial1:
  case _polynomial5:
  case _polynomial7:
    // dis = Polynomial_computer_distance(&(vp->polynomial_in),
    //                                    &(vp->polynomial_mid), t);
    break;
  case _onlinedoubleS:
    printf("_onlinedoubleS invalid!\n");
    break;
  default:
    break;
  }
  return dis;
}

double getVelocityProfile_time(VelocityProfile *vp, double s) {
  double t = 0;
  switch (vp->ProfileType) {
  case _doubleS:
    t = DoubleS_computer_time(&(vp->doubleS_in), &(vp->doubleS_mid), s);
    break;
  case _polynomial1:
  case _polynomial5:
  case _polynomial7:
    // t = Polynomial_computer_time(&(vp->polynomial_in), &(vp->polynomial_mid),
    //                              s);
    break;
  case _onlinedoubleS:
    printf("_onlinedoubleS invalid!\n");
    break;
  default:
    break;
  }
  return t;
}

double getVelocityProfileSpeedCoeff(VelocityProfile *vp) {
  return vp->speed_coeff;
}

int getVelocityProfileStateType(VelocityProfile *vp) {
  int type = 0;
  switch (vp->ProfileType) {
  case _doubleS:
    type = vp->doubleS_in.stop_state;
    break;
  case _polynomial1:
  case _polynomial5:
  case _polynomial7:
    type = vp->polynomial_in.stop_state;
    break;
  case _onlinedoubleS:
    type = vp->online_doubleS_in.stop_state;
    break;
  default:
    break;
  }
  return type;
}

double getVelocityProfileTime(VelocityProfile *vp) {
  double time = 0;
  switch (vp->ProfileType) {
  case _doubleS:
    time = vp->doubleS_in._T;
    break;
  case _polynomial1:
  case _polynomial5:
  case _polynomial7:
    printf("_polynomial invalid!\n");
    break;
  case _onlinedoubleS:
    time = vp->online_doubleS_in._T;
    break;
  default:
    break;
  }
  return time;
}

}  // namespace rosc
