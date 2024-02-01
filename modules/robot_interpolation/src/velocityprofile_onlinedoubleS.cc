/********************************************************************
 * @file velocityprofile_onlinedoubleS.cc
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-04-04
 *
 * @copyright Copyright (c) 2023 ROSC
 *
 ********************************************************************/
#include "robot_brain/velocityprofile_onlinedoubleS.h"
namespace rosc {

static double _stop_calculation(OnlineDoubleS_in *inter_in,
                                OnlineDoubleS_mid *inter_mid,
                                Velocityprofile_out *inter_out) {
  double a_max = inter_in->acc_max;
  double a_min = inter_in->acc_min;
  double j_max = inter_in->jerk_max;
  double j_min = inter_in->jerk_min;
  double ve = inter_in->v_end;
  double ae = inter_in->acc_end;
  double v = inter_out->v;
  double a = inter_out->a;

  double Tj2a, Tj2b, Td;
  double h;
  int flag = 0;
  // v>=ve   减速停止
  Tj2a = (a_min - a) / j_min;
  Tj2b = (ae - a_min) / j_max;
  Td = (ve - v) / a_min + Tj2a * (a_min - a) / 2.0 / a_min +
       Tj2b * (a_min - ae) / 2.0 / a_min;

  if (Td < Tj2a + Tj2b) {
    double tmp1 = (j_max - j_min) *
                  (a * a * j_max - j_min * (ae * ae + 2.0 * j_max * (v - ve)));
    if (tmp1 < 0) {
      Tj2a = 0;
      Tj2b = 0;
      Td = 0;
      flag++;
    } else {
      double tmp = sqrt(tmp1);
      Tj2a = -a / j_min + tmp / j_min / (j_min - j_max);
      Tj2b = ae / j_max + tmp / j_max / (j_max - j_min);
      if ((Tj2a < 0) || (Tj2b < 0)) {
        Tj2a = 0;
        Tj2b = 0;
      }
      Td = Tj2a + Tj2b;
    }
  }

  // step2: compute the position displacement produced by the acceleration and
  // velocity profiles
  h = a * Td * Td / 2.0 +
      (j_min * Tj2a * (3.0 * Td * Td - 3.0 * Td * Tj2a + Tj2a * Tj2a) +
       j_max * Tj2b * Tj2b * Tj2b) /
          6.0 +
      Td * v;

  inter_mid->Tj2a = Tj2a;
  inter_mid->Tj2b = Tj2b;
  inter_mid->Td = Td;
  inter_mid->h = h;
  inter_mid->stop_flag = 0;
  // printf("11111111:%f,%f,%f,%f\n",Tj2a,Tj2b,Td,h);

  // v<ve   加速停止
  Tj2a = (a_max - a) / j_max;
  Tj2b = (ae - a_max) / j_min;
  Td = (ve - v) / a_max + Tj2a * (a_max - a) / 2.0 / a_max +
       Tj2b * (a_max - ae) / 2.0 / a_max;

  if (Td < Tj2a + Tj2b) {
    double tmp1 = (j_min - j_max) *
                  (a * a * j_min - j_max * (ae * ae + 2.0 * j_min * (v - ve)));
    if (tmp1 < 0) {
      Tj2a = 0;
      Tj2b = 0;
      Td = 0;
      flag++;
    } else {
      double tmp = sqrt(tmp1);
      Tj2a = -a / j_max + tmp / j_max / (j_max - j_min);
      Tj2b = ae / j_min + tmp / j_min / (j_min - j_max);
      if ((Tj2a < 0) || (Tj2b < 0)) {
        Tj2a = 0;
        Tj2b = 0;
      }
      Td = Tj2a + Tj2b;
    }
  }

  // step2: compute the position displacement produced by the acceleration and
  // velocity profiles
  h = a * Td * Td / 2.0 +
      (j_max * Tj2a * (3.0 * Td * Td - 3.0 * Td * Tj2a + Tj2a * Tj2a) +
       j_min * Tj2b * Tj2b * Tj2b) /
          6.0 +
      Td * v;

  if (h > inter_mid->h) {
    inter_mid->Tj2a = Tj2a;
    inter_mid->Tj2b = Tj2b;
    inter_mid->Td = Td;
    inter_mid->h = h;
    inter_mid->stop_flag = 1;
  }
  if (2 == flag) {
    inter_mid->stop_flag = -1;
    return -1;
  }
  return inter_mid->h;
}

/*基础输入检查
 * inter_in   初始化输入；
 *
 * return 0:正常规划；1减速阶段停止； -1: 输入初始与末端状态错误；-2:
 * 输入最大限制错误
 * */
static int OnlineDoubleS_check(OnlineDoubleS_in *inter_in,
                               OnlineDoubleS_mid *inter_mid,
                               Velocityprofile_out *inter_out) {
  if (inter_in->v_start < 0 || inter_in->v_end < 0 || inter_in->v_max < 0 ||
      inter_in->acc_max <
          max(fabs(inter_in->acc_start), fabs(inter_in->acc_end)) ||
      inter_in->jerk_max < 0) {
    return -1;
  }

  return 0;
}

/*基础插补规划函数
 * inter_in   初始化输入；
 * inter_mid 初始化输出；
 * inter_out 规划结果
 * return 0:正确； -1: 输入错误；
 * */
int OnlineDoubleS_project(OnlineDoubleS_in *inter_in,
                          OnlineDoubleS_mid *inter_mid,
                          Velocityprofile_out *inter_out) {
  int ret = OnlineDoubleS_check(inter_in, inter_mid, inter_out);
  if (0 != ret) {
    return -1;
  }

  inter_mid->t_stop = -1;
  if (inter_in->dt <= inter_mid->_ts) {
    inter_mid->_ts = inter_in->dt;
    inter_mid->ts_times = 1;
  } else {
    double tmp = inter_in->dt / inter_mid->_ts;
    if ((tmp - static_cast<int>(tmp)) < 1e-9) {  // ! int 转换
      inter_mid->ts_times = static_cast<int>(tmp);
    } else {
      return -2;
    }
  }
  double h = _stop_calculation(inter_in, inter_mid, inter_out);
  if ((h < 0) || (h > (inter_in->S - (inter_out->s - inter_in->s0)))) {
    return -3;
  }

  return 0;
}

/*基础插补数据计算
 * inter_in   输入
 * inter_mid 中间输入
 * inter_out 插补结果输出
 *
 *return 0:满足; 1: 运动
 * */
static int _OnlineDoubleS_computer(OnlineDoubleS_in *inter_in,
                                   OnlineDoubleS_mid *inter_mid,
                                   Velocityprofile_out *inter_out) {
  double v_max = inter_in->v_max;
  double a_max = inter_in->acc_max;
  double a_min = inter_in->acc_min;
  double j_max = inter_in->jerk_max;
  double j_min = inter_in->jerk_min;
  double s = inter_out->s - inter_in->s0;
  double v = inter_out->v;
  double a = inter_out->a;
  double j = inter_out->j;
  double dt = inter_mid->_ts;
  double S = inter_in->S;
  double t = inter_out->t + dt;
  double smt = inter_in->start_moving_v;

  inter_out->state = _velocityprofile_run;

  if (inter_in->S < 1e-6) {
    inter_out->t = t;
    inter_out->s = inter_in->s0;
    inter_out->v = 0;
    inter_out->a = 0;
    inter_out->j = 0;
    inter_out->state = inter_in->stop_state;
    return inter_out->state;
  }

  double s_new, v_new, a_new, j_new;

  // compute the time intervals Td, Tj2a and Tj2b
  if (inter_mid->t_stop < 0) {
    _stop_calculation(inter_in, inter_mid, inter_out);
  }

  if (((inter_mid->t_stop < 0) && (inter_mid->h >= (S - s))) ||
      (inter_in->stop_state == _velocityprofile_forcestop) ||
      (inter_in->stop_state == _velocityprofile_suspend)) {
    inter_mid->t_stop = t;
  }

  if (inter_mid->t_stop >= 0) {  // stop moving
    if ((t - inter_mid->t_stop) <= inter_mid->Tj2a) {
      if (0 == inter_mid->stop_flag) {
        j_new = j_min;
      } else {
        j_new = j_max;
      }
    } else if (((t - inter_mid->t_stop) > inter_mid->Tj2a) &&
               ((t - inter_mid->t_stop) <= (inter_mid->Td - inter_mid->Tj2b))) {
      j_new = 0;
    } else if (((t - inter_mid->t_stop) > (inter_mid->Td - inter_mid->Tj2b)) &&
               ((t - inter_mid->t_stop) <= inter_mid->Td)) {
      if (0 == inter_mid->stop_flag) {
        j_new = j_max;
      } else {
        j_new = j_min;
      }
    } else {
      j_new = 0;
      inter_out->state = inter_in->stop_state;
    }
  } else {               // start moving
    if (smt <= v_max) {  //加速
      if (((v - a * a / 2.0 / j_min) >= v_max)) {
        if (a <= 0) {
          j_new = 0;
          a = 0;  //防止加加速度震荡
          inter_out->state = _velocityprofile_run_constant_velocity;
        } else {
          j_new = j_min;
        }
      } else {
        if (a >= a_max) {
          j_new = 0;
          a = a_max;  //防止加加速度震荡
        } else {
          j_new = j_max;
        }
      }

    } else {  //减速
      if ((v - a * a / 2.0 / j_max) <= v_max) {
        if (a >= 0) {
          j_new = 0;
          a = 0;  //防止加加速度震荡
          inter_out->state = _velocityprofile_run_constant_velocity;
        } else {
          j_new = j_max;
        }
      } else {
        if (a <= a_min) {
          j_new = 0;
          a = a_min;  //防止加加速度震荡
        } else {
          j_new = j_min;
        }
      }
    }
  }

  //积分
  if (_velocityprofile_stop == inter_out->state) {
    inter_out->t =
        (floor((t - inter_in->t0) / inter_in->dt) + 1) * inter_in->dt +
        inter_in->t0;
    inter_out->s = inter_in->S + inter_in->s0;
    inter_out->v = inter_in->v_end;
    inter_out->a = inter_in->acc_end;
    inter_out->j = 0;
  } else {
    // step3:
    inter_out->t = t;
    a_new = a + dt / 2.0 * (j + j_new);
    v_new = v + dt / 2.0 * (a + a_new);
    s_new = s + dt / 2.0 * (v + v_new) + inter_in->s0;
    inter_out->s = s_new;
    inter_out->v = v_new;
    inter_out->a = a_new;
    inter_out->j = j_new;
  }

  return inter_out->state;
}

/*基础插补数据计算
 * inter_in   输入
 * inter_mid 中间输入
 * inter_out 插补结果输出
 *
 *return 0:满足; 1: 运动
 * */
int OnlineDoubleS_computer(OnlineDoubleS_in *inter_in,
                           OnlineDoubleS_mid *inter_mid,
                           Velocityprofile_out *inter_out) {
  int ret = 0;
  int i = 0;
  for (i = 0; i < inter_mid->ts_times; i++) {
    ret = _OnlineDoubleS_computer(inter_in, inter_mid, inter_out);
    if ((_velocityprofile_stop == ret) || (_velocityprofile_forcestop == ret)) {
      return ret;
    }
  }
  return ret;
}

int UpdateOnlineDoubleS(OnlineDoubleS_in *inter_in,
                        OnlineDoubleS_mid *inter_mid,
                        Velocityprofile_out *inter_out, double Vmax,
                        double Amax, double Jmax) {
  OnlineDoubleS_in _inter_in = *inter_in;
  OnlineDoubleS_mid _inter_mid = *inter_mid;
  if (inter_mid->t_stop >= 0) {
    return -1;
  }
  if (Vmax < 0 || Amax < max(fabs(inter_out->a), fabs(inter_in->acc_end)) ||
      Jmax < 0) {
    return -2;
  }
  _inter_in.UpdateVMax(Vmax);
  _inter_in.UpdateAccMax(Amax);
  _inter_in.UpdateJerkMax(Jmax);
  _inter_in.UpdateStartMovingV(inter_out->v);
  // updateOnlineDoubleS_Vmax(&_inter_in, Vmax);
  // updateOnlineDoubleS_Amax(&_inter_in, Amax);
  // updateOnlineDoubleS_Jmax(&_inter_in, Jmax);
  // updateOnlineDoubleS_start_moving_v(&_inter_in, inter_out);
  double h = _stop_calculation(&_inter_in, &_inter_mid, inter_out);
  if ((h < 0) || (h > (inter_in->S - (inter_out->s - inter_in->s0)))) {
    return -3;
  }

  inter_in->UpdateVMax(Vmax);
  inter_in->UpdateAccMax(Amax);
  inter_in->UpdateJerkMax(Jmax);
  inter_in->UpdateStartMovingV(inter_out->v);
  // updateOnlineDoubleS_Vmax(inter_in, Vmax);
  // updateOnlineDoubleS_Amax(inter_in, Amax);
  // updateOnlineDoubleS_Jmax(inter_in, Jmax);
  // updateOnlineDoubleS_start_moving_v(inter_in, inter_out);

  return 0;
}

int UpdateOnlineDoubleS_Vmax(OnlineDoubleS_in *inter_in,
                             OnlineDoubleS_mid *inter_mid,
                             Velocityprofile_out *inter_out, double Vmax) {
  return UpdateOnlineDoubleS(inter_in, inter_mid, inter_out, Vmax,
                             inter_in->acc_max, inter_in->jerk_max);
}

int UpdateOnlineDoubleS_Amax(OnlineDoubleS_in *inter_in,
                             OnlineDoubleS_mid *inter_mid,
                             Velocityprofile_out *inter_out, double Amax) {
  return UpdateOnlineDoubleS(inter_in, inter_mid, inter_out, inter_in->v_max,
                             Amax, inter_in->jerk_max);
}

int UpdateOnlineDoubleS_Jmax(OnlineDoubleS_in *inter_in,
                             OnlineDoubleS_mid *inter_mid,
                             Velocityprofile_out *inter_out, double Jmax) {
  return UpdateOnlineDoubleS(inter_in, inter_mid, inter_out, inter_in->v_max,
                             inter_in->acc_max, Jmax);
}

double OnlineDoubleS_computer_prodfile_time(OnlineDoubleS_in *inter_in,
                                            Velocityprofile_out *inter_out,
                                            int stopstate) {
  double T = 0;
  int err = getDoubleSVelocityprofileTime(
      inter_in->v_start, &(inter_in->v_end), &(inter_in->v_max),
      inter_in->acc_max, inter_in->jerk_max,
      inter_in->_S + inter_in->s0 - inter_out->s, &T, NULL, stopstate);
  if (0 != err) {
    return -1;
  }
  return T;
}

int OnlineDoubleS_sync_prodfile(OnlineDoubleS_in *inter_in,
                                Velocityprofile_out *inter_out, double T,
                                double _T, int stopstate) {
  int err = syncDoubleSVelocityprofile(
      inter_in->v_start, &(inter_in->v_end), &(inter_in->v_max),
      inter_in->acc_max, inter_in->jerk_max, T,
      inter_in->_S + inter_in->s0 - inter_out->s, _T, NULL, stopstate);
  if (0 != err) {
    return -1;
  }
  return 0;
}

}  // namespace rosc
