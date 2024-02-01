/********************************************************************
 * @file velocityprofile_doubleS.cc
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-04-04
 *
 * @copyright Copyright (c) 2023 ROSC
 *
 ********************************************************************/
#include "robot_brain/velocityprofile_doubleS.h"
#include "robot_brain/count.h"
namespace rosc {

/*二分法非线性求解
 * fun 非线性方程
 * xmin 搜索下限
 * xmax 搜索上限
 * e0 求解精度
 *
 * return 满足精度的解
 * */
static double dichotomy(double (*fun)(double, DoubleS_in *), double xmin,
                        double xmax, double e0, DoubleS_in *inter_in) {
  double e = 0;
  double x = 0;
  double fmin = 0;
  double fx = 0;
  double ff = 0;
  if (fabs(fun(xmin, inter_in)) <= e0) {
    return xmin;
  } else if (fabs(fun(xmax, inter_in)) <= e0) {
    return xmax;
  } else {
    e = xmax - xmin;
    while (e > e0) {
      x = (xmin + xmax) / 2;
      fmin = fun(xmin, inter_in);
      fx = fun(x, inter_in);
      ff = fmin * fx;
      if (ff < 0) {
        xmax = x;
      } else if (ff > 0) {
        xmin = x;
      } else {
        xmin = x;
        xmax = x;
      }
      e = xmax - xmin;
    }
    x = (xmin + xmax) / 2;
  }
  return x;
}

static double fun1(double Vmax, DoubleS_in *inter_in) {
  double vs = inter_in->v_start;
  double ve = inter_in->v_end;
  double as = inter_in->acc_start;
  double Amax = inter_in->acc_max;
  double Jmax = inter_in->jerk_max;
  double S = inter_in->S;
  return 0.5 * (Vmax + (2 * Jmax * vs - as * as) / (2 * Jmax)) *
             (2 * Amax / Jmax +
              ((Vmax - vs) - 0.5 * (2 * Amax * Amax - as * as) / Jmax) / Amax) -
         vs * as / Jmax + (1.0 / 3.0) * ((as * as * as) / (Jmax * Jmax)) +
         0.5 * (Vmax + ve) * (Amax / Jmax + (Vmax - ve) / Amax) - S;
}

static double fun2(double Vmax, DoubleS_in *inter_in) {
  double vs = inter_in->v_start;
  double ve = inter_in->v_end;
  double as = inter_in->acc_start;
  double Amax = inter_in->acc_max;
  double Jmax = inter_in->jerk_max;
  double S = inter_in->S;
  return 0.5 * (Vmax + (2 * Jmax * vs - as * as) / (2 * Jmax)) *
             (2 * Amax / Jmax +
              ((Vmax - vs) - 0.5 * (2 * Amax * Amax - as * as) / Jmax) / Amax) -
         vs * as / Jmax + (1.0 / 3.0) * ((as * as * as) / (Jmax * Jmax)) +
         (Vmax + ve) * (sqrt((Vmax - ve) / Jmax)) - S;
}

static double fun3(double Vmax, DoubleS_in *inter_in) {
  double vs = inter_in->v_start;
  double ve = inter_in->v_end;
  double as = inter_in->acc_start;
  double Jmax = inter_in->jerk_max;
  double S = inter_in->S;
  return (Vmax + (2 * Jmax * vs - as * as) / (2 * Jmax)) *
             ((sqrt((Vmax - vs) * Jmax + 0.5 * as * as)) / Jmax) -
         vs * as / Jmax + (1.0 / 3.0) * ((as * as * as) / (Jmax * Jmax)) +
         (Vmax + ve) * (sqrt((Vmax - ve) / Jmax)) - S;
}

static double fun4(double Vmax, DoubleS_in *inter_in) {
  double vs = inter_in->v_start;
  double ve = inter_in->v_end;
  double as = inter_in->acc_start;
  double Amax = inter_in->acc_max;
  double Jmax = inter_in->jerk_max;
  double S = inter_in->S;
  return (Vmax + (2 * Jmax * vs - as * as) / (2 * Jmax)) *
             ((sqrt((Vmax - vs) * Jmax + 0.5 * as * as)) / Jmax) -
         vs * as / Jmax + (1.0 / 3.0) * ((as * as * as) / (Jmax * Jmax)) +
         0.5 * (Vmax + ve) * (Amax / Jmax + (Vmax - ve) / Amax) - S;
}

/*基础输入检查
 * inter_in   初始化输入；
 *
 * return 0:正常规划；1减速阶段停止； -1: 输入初始与末端状态错误；-2:
 * 输入最大限制错误
 * */
static int DoubleS_check(DoubleS_in *inter_in) {
  double e0 = 1e-6;
  if (inter_in->v_start < 0 || inter_in->v_end < 0 || inter_in->S < 0) {
    return -1;
  }

  if ((inter_in->v_max + e0) < inter_in->v_end ||
      (inter_in->acc_max + e0) < fabs(inter_in->acc_start) ||
      (inter_in->jerk_max) <= 0) {
    return -2;
  }

  if (inter_in->S < e0) {
    return _velocity_profile_no;
  }

  if ((rosc::VelocityprofileStopState::_velocityprofile_forcestop ==
       inter_in->stop_state) ||
      (rosc::VelocityprofileStopState::_velocityprofile_suspend ==
       inter_in->stop_state)) {
    return _velocity_profile_down;
  }

  if ((inter_in->v_max) <= inter_in->v_start) {
    return _velocity_profile_full_vmax;
  }

  if (inter_in->acc_start < 0) {
    return _velocity_profile_full_add;
  }

  return _velocity_profile_full;
}

/*基础插补规划函数(加速段、匀速段规划)
 * inter_in   初始化输入；
 * inter_mid 初始化输出；
 *
 * return 0:正确； -1: 输入错误；
 * */
static int _DoubleS_project_full(DoubleS_in *inter_in, DoubleS_mid *inter_mid) {
  double e0 = 0.000001;
  double vs = inter_in->v_start;
  double ve = inter_in->v_end;
  double as = inter_in->acc_start;
  double Vmax = inter_in->v_max;
  double Amax = inter_in->acc_max;
  double Jmax = inter_in->jerk_max;
  double S = inter_in->S;
  double T1 = 0, T2 = 0, T3 = 0, T4 = 0, T5 = 0, T6 = 0, T7 = 0;
  if (S < 1e-6) {
    inter_mid->T = 0;
    inter_mid->Taa = 0;
    inter_mid->Tca = 0;
    inter_mid->Tda = 0;
    inter_mid->Tv = 0;
    inter_mid->Tad = 0;
    inter_mid->Tcd = 0;
    inter_mid->Tdd = 0;
    inter_mid->ac_Vmax = 0;
    return 0;
  }

  if (Vmax >= vs + 0.5 * (2 * Amax * Amax - as * as) / Jmax) {
    T1 = (Amax - as) / Jmax;
  } else {
    T1 = (sqrt((Vmax - vs) * Jmax + 0.5 * as * as) - as) / Jmax;
  }

  if (Vmax >= vs + 0.5 * (2 * Amax * Amax - as * as) / Jmax) {
    T2 = ((Vmax - vs) - 0.5 * (2 * Amax * Amax - as * as) / Jmax) / Amax;
  } else {
    T2 = 0;
  }
  if (Vmax >= vs + 0.5 * (2 * Amax * Amax - as * as) / Jmax) {
    T3 = Amax / Jmax;
  } else {
    T3 = (sqrt((Vmax - vs) * Jmax + 0.5 * as * as)) / Jmax;
  }
  if (Vmax >= ve + Amax * Amax / Jmax) {
    T5 = Amax / Jmax;
    T7 = T5;
  } else {
    T5 = sqrt((Vmax - ve) / Jmax);
    T7 = T5;
  }
  if (Vmax >= ve + Amax * Amax / Jmax) {
    T6 = (Vmax - ve) / Amax - Amax / Jmax;
  } else {
    T6 = 0;
  }
  double S1 =
      0.5 * (Vmax + (2 * Jmax * vs - as * as) / (2 * Jmax)) * (2 * T3 + T2) -
      vs * as / Jmax + (1.0 / 3.0) * ((as * as * as) / (Jmax * Jmax)) +
      0.5 * (Vmax + ve) * (2 * T5 + T6);  // 加减速位移
  if (S1 < S) {                           // 存在匀速段
    T4 = (S - S1) / Vmax;
  }
  if (S1 == S) {  //恰好无匀速段
    T4 = 0;
  }
  if (S1 > S) {  //无匀速段，未达到最大速度
    T4 = 0;
    //计算最大速度
    if (vs < ve) {
      double Vmax1 = ve + Amax * Amax / Jmax;  //无匀减速段
      Vmax = Vmax1;
      S1 = 0.5 * (Vmax + (2 * Jmax * vs - as * as) / (2 * Jmax)) *
               (2 * T3 + T2) -
           vs * as / Jmax + (1.0 / 3.0) * ((as * as * as) / (Jmax * Jmax)) +
           0.5 * (Vmax + ve) * (2 * T5 + T6);  //加减速位移
      if (S1 < S) {                            //有匀减速段，有匀加速
        Vmax =
            dichotomy(fun1, ve, inter_in->v_max, e0, inter_in);  //实际最大速度
        T1 = (Amax - as) / Jmax;
        T2 = ((Vmax - vs) - 0.5 * (2 * Amax * Amax - as * as) / Jmax) / Amax;
        T3 = Amax / Jmax;
        T5 = Amax / Jmax;
        T7 = T5;
        T6 = (Vmax - ve) / Amax - Amax / Jmax;
      }
      if (S1 == S) {  //无匀减速段，有匀加速
        Vmax = Vmax1;
        T1 = (Amax - as) / Jmax;
        T2 = ((Vmax - vs) - 0.5 * (2 * Amax * Amax - as * as) / Jmax) / Amax;
        T3 = Amax / Jmax;
        T5 = sqrt((Vmax - ve) / Jmax);
        T7 = T5;
        T6 = 0;
      }
      if (S1 > S) {  //无匀减速段
        double Vmax2 = vs + 0.5 * (2 * Amax * Amax - as * as) / Jmax;
        if (Vmax2 <= ve) {  //有匀加速段
          Vmax = dichotomy(fun2, ve, inter_in->v_max, e0,
                           inter_in);  //实际最大速度
          T1 = (Amax - as) / Jmax;
          T2 = ((Vmax - vs) - 0.5 * (2 * Amax * Amax - as * as) / Jmax) / Amax;
          T3 = Amax / Jmax;
          T5 = sqrt((Vmax - ve) / Jmax);
          T7 = T5;
          T6 = 0;
        } else {
          Vmax = Vmax2;
          double S2 = (Vmax + (2 * Jmax * vs - as * as) / (2 * Jmax)) * T3 -
                      vs * as / Jmax +
                      (1.0 / 3.0) * ((as * as * as) / (Jmax * Jmax)) +
                      (Vmax + ve) * T5;
          if (S2 >= S) {  // 无匀加速段
            Vmax = dichotomy(fun3, ve, inter_in->v_max, e0,
                             inter_in);  //实际最大速度
            T1 = (sqrt((Vmax - vs) * Jmax + 0.5 * as * as) - as) / Jmax;
            T2 = 0;
            T6 = 0;
            T3 = (sqrt((Vmax - vs) * Jmax + 0.5 * as * as)) / Jmax;
            T5 = sqrt((Vmax - ve) / Jmax);
            T7 = T5;
          } else {  //有匀加速段
            Vmax = dichotomy(fun2, ve, inter_in->v_max, e0,
                             inter_in);  //实际最大速度
            T1 = (Amax - as) / Jmax;
            T2 =
                ((Vmax - vs) - 0.5 * (2 * Amax * Amax - as * as) / Jmax) / Amax;
            T3 = Amax / Jmax;
            T5 = sqrt((Vmax - ve) / Jmax);
            T7 = T5;
            T6 = 0;
          }
        }
      }
    } else {  // vs>=ve
      double Vmax1 = vs + 0.5 * (2 * Amax * Amax - as * as) / Jmax;
      Vmax = Vmax1;
      S1 = 0.5 * (Vmax + (2 * Jmax * vs - as * as) / (2 * Jmax)) *
               (2 * T3 + T2) -
           vs * as / Jmax + (1.0 / 3.0) * ((as * as * as) / (Jmax * Jmax)) +
           0.5 * (Vmax + ve) * (2 * T5 + T6);  // 加减速位移
      if (S1 < S) {                            // 有匀加速
        Vmax =
            dichotomy(fun1, vs, inter_in->v_max, e0, inter_in);  // 实际最大速度
        T1 = (Amax - as) / Jmax;
        T2 = ((Vmax - vs) - 0.5 * (2 * Amax * Amax - as * as) / Jmax) / Amax;
        T3 = Amax / Jmax;
        T5 = Amax / Jmax;
        T7 = T5;
        T6 = (Vmax - ve) / Amax - Amax / Jmax;
      }
      if (S1 == S) {  // 无匀加速段,有匀减
        Vmax = Vmax1;
        T1 = (sqrt((Vmax - vs) * Jmax + 0.5 * as * as) - as) / Jmax;
        T2 = 0;
        T3 = (sqrt((Vmax - vs) * Jmax + 0.5 * as * as)) / Jmax;
        T5 = Amax / Jmax;
        T7 = T5;
        T6 = (Vmax - ve) / Amax - Amax / Jmax;
      }
      if (S1 > S) {  // 无匀加速段
        double Vmax2 = ve + (Amax * Amax) / Jmax;
        Vmax = Vmax2;
        double S2 = (Vmax + (2 * Jmax * vs - as * as) / (2 * Jmax)) * T3 -
                    vs * as / Jmax +
                    (1.0 / 3.0) * ((as * as * as) / (Jmax * Jmax)) +
                    (Vmax + ve) * T5;
        if (S2 >= S) {  // 无匀减速段，无匀加速段
          Vmax = dichotomy(fun3, vs, inter_in->v_max, e0,
                           inter_in);  // 实际最大速度
          T1 = (sqrt((Vmax - vs) * Jmax + 0.5 * as * as) - as) / Jmax;
          T2 = 0;
          T3 = (sqrt((Vmax - vs) * Jmax + 0.5 * as * as)) / Jmax;
          T6 = 0;
          T5 = sqrt((Vmax - ve) / Jmax);
          T7 = T5;
        } else {  // 有匀减速，无匀加速段
          Vmax = dichotomy(fun4, vs, inter_in->v_max, e0,
                           inter_in);  //实际最大速度
          T1 = (sqrt((Vmax - vs) * Jmax + 0.5 * as * as) - as) / Jmax;
          T2 = 0;
          T3 = (sqrt((Vmax - vs) * Jmax + 0.5 * as * as)) / Jmax;
          T5 = Amax / Jmax;
          T7 = T5;
          T6 = (Vmax - ve) / Amax - Amax / Jmax;
        }
      }
    }
  }
  inter_mid->T = T1 + T2 + T3 + T4 + T5 + T6 + T7;
  inter_mid->Taa = T1;
  inter_mid->Tca = T2;
  inter_mid->Tda = T3;
  inter_mid->Tv = T4;
  inter_mid->Tad = T5;
  inter_mid->Tcd = T6;
  inter_mid->Tdd = T7;
  inter_mid->ac_Vmax = Vmax;
  //	printf("%f,%f,%f,%f,%f,%f,%f,%f,%f\n",inter_mid->T,inter_mid->Taa,inter_mid->Tca,inter_mid->Tda,inter_mid->Tv,inter_mid->Tad,inter_mid->Tcd,inter_mid->Tdd,inter_mid->ac_Vmax);
  double ce0 = -1e-10;
  if (T1 < ce0 || T2 < ce0 || T3 < ce0 || T4 < ce0 || T5 < ce0 || T6 < ce0 ||
      T7 < ce0) {
    printf("Inter_in set is not reasonable!\n");
    printf("Taa=%f,Tca=%f,Tda=%f,Tv=%f,Tad=%f,Tcd=%f,Tdd=%f\n", T1, T2, T3, T4,
           T5, T6, T7);
    return -2;
  }

  if (T1 < 0)
    T1 = 0;
  if (T2 < 0)
    T2 = 0;
  if (T3 < 0)
    T3 = 0;
  if (T4 < 0)
    T4 = 0;
  if (T5 < 0)
    T5 = 0;
  if (T6 < 0)
    T6 = 0;
  if (T7 < 0)
    T7 = 0;

  return 0;
}

/*基础插补规划函数(加速段、匀速段规划)
 * inter_in   初始化输入；
 * inter_mid 初始化输出；
 *
 * return 0:正确； -1: 输入错误；
 * */
static int DoubleS_project_full(DoubleS_in *inter_in, DoubleS_mid *inter_mid) {
  inter_mid->type = _velocity_profile_full;
  inter_mid->Taa_1 = 0;
  inter_mid->Taa_2 = 0;
  inter_mid->Taa_3 = 0;
  inter_mid->S_1 = 0;
  return _DoubleS_project_full(inter_in, inter_mid);
}

/*基础插补规划函数(减速段规划)
 * inter_in   初始化输入；
 * inter_mid 初始化输出；
 *
 * return 0:正确； -1: 输入错误；
 * */
static int DoubleS_project_down(DoubleS_in *inter_in, DoubleS_mid *inter_mid) {
  inter_mid->type = _velocity_profile_down;
  double vs = inter_in->v_start;
  double as = inter_in->acc_start;
  double Vmax = inter_in->v_max;
  double Amax = inter_in->acc_max;
  double Jmax = inter_in->jerk_max;
  double __t4 = 0;
  if (as > 0) {
    __t4 = 2.0 * as / Jmax;
    as = -as;
  }

  double ac_Amax = 0;
  //-------------仅减减------------
  double _t4 = 0;
  double _t5 = 0;
  double _t6 = 0;
  double _t7 = -as / Jmax;
  double v1 = vs - 0.5 * Jmax * _t7 * _t7;

  //-------------恰好无匀减------------
  _t5 = (Amax + as) / Jmax;
  _t6 = _t5;
  _t7 = _t6 + Amax / Jmax;

  double v2 = (vs - 0.5 * Jmax * (_t5 - _t4) * (_t5 - _t4) + as * (_t5 - _t4)) -
              0.5 * Jmax * (_t6 - _t7) * (_t6 - _t7);

  if (v1 < -1e-6) {  // 无解
    return -1;
  }
  if (v2 < 0) {  // 无匀减
    double _tmp = sqrt((vs + 0.5 * as * as / Jmax) / Jmax);
    _t5 = _tmp + (as / Jmax);
    _t6 = _t5;
    ac_Amax = fabs(-Jmax * (_t5 - _t4) + as);
    _t7 = _t6 + ac_Amax / Jmax;
  } else {  // 有匀减
    _t5 = (Amax + as) / Jmax;
    _t6 = v2 / Amax + _t5;
    _t7 = _t6 + Amax / Jmax;
    ac_Amax = Amax;
  }

  inter_mid->T = _t7 + __t4;
  inter_mid->Taa = 0;
  inter_mid->Tca = 0;
  inter_mid->Tda = __t4 / 2.0;
  inter_mid->Tv = 0;
  inter_mid->Tad = (_t5 - _t4) + __t4 / 2.0;
  inter_mid->Tcd = _t6 - _t5;
  inter_mid->Tdd = _t7 - _t6;
  inter_mid->ac_Vmax = Vmax;
  inter_mid->ac_Amaxa = ac_Amax;

  double ce0 = -1e-10;
  if (inter_mid->Tda < ce0 || inter_mid->Tad < ce0 || inter_mid->Tcd < ce0 ||
      inter_mid->Tdd < ce0) {
    printf("Inter_in set is not reasonable!\n");
    printf("Tda=%.15f,Tad=%.15f,Tcd=%.15f,Tdd=%.15f\n", inter_mid->Tda,
           inter_mid->Tad, inter_mid->Tcd, inter_mid->Tdd);
    return -2;
  }
  if (inter_mid->Tda < 0)
    inter_mid->Tda = 0;
  if (inter_mid->Tad < 0)
    inter_mid->Tad = 0;
  if (inter_mid->Tcd < 0)
    inter_mid->Tcd = 0;
  if (inter_mid->Tdd < 0)
    inter_mid->Tdd = 0;

  return 0;
}

/*基础插补规划函数(加速段、匀速段规划)
 * inter_in   初始化输入；
 * inter_mid 初始化输出；
 *
 * return 0:正确； -1: 输入错误；
 * */
static int DoubleS_project_full_add(DoubleS_in *inter_in,
                                    DoubleS_mid *inter_mid) {
  inter_mid->type = _velocity_profile_full_add;
  double T0 = 0;
  double S_1 = inter_in->S;
  double as = inter_in->acc_start;
  if (inter_in->acc_start < 0) {
    inter_in->acc_start = -inter_in->acc_start;
    T0 = 2.0 * inter_in->acc_start / inter_in->jerk_max;
    inter_in->S =
        inter_in->S -
        (inter_in->v_start * T0 + 0.5 * (-inter_in->acc_start) * T0 * T0 +
         (1.0 / 6.0) * inter_in->jerk_max * T0 * T0 * T0);
    if (inter_in->S <= 1e-6) {
      return -1;
    }
  }
  int ret = _DoubleS_project_full(inter_in, inter_mid);
  inter_in->acc_start = as;
  inter_in->S = S_1;
  inter_mid->Taa = inter_mid->Taa + T0;
  inter_mid->T = inter_mid->T + T0;
  inter_mid->S_1 = 0;
  return ret;
}

/*基础插补规划函数(最大速度小于起始速度，加速度小于零)
 * inter_in   初始化输入；
 * inter_mid 初始化输出；
 *
 * return 0:正确； -1: 输入错误；
 * */
static int DoubleS_project_full_vmax(DoubleS_in *inter_in,
                                     DoubleS_mid *inter_mid) {
  inter_mid->type = _velocity_profile_full_vmax;

  double vs = inter_in->v_start;
  double as = inter_in->acc_start;
  double Vmax = inter_in->v_max;
  double Amax = inter_in->acc_max;
  double Jmax = inter_in->jerk_max;
  double __t4 = 0;
  if (as > 0) {
    __t4 = 2.0 * as / Jmax;
    as = -as;
  }

  double ac_Amax = 0;
  //-------------仅减减------------
  double _t4 = 0;
  double _t5 = 0;
  double _t6 = 0;
  double _t7 = -as / Jmax;
  if (inter_in->acc_start < 0) {
    double v1 = (vs - Vmax) - 0.5 * Jmax * _t7 * _t7;
    if (v1 < 0) {  //无解
      Vmax = vs - 0.5 * Jmax * _t7 * _t7;
    }
  }

  //-------------恰好无匀减------------
  _t5 = (Amax + as) / Jmax;
  _t6 = _t5;
  _t7 = _t6 + Amax / Jmax;

  double v2 = ((vs - Vmax) - 0.5 * Jmax * (_t5 - _t4) * (_t5 - _t4) +
               as * (_t5 - _t4)) -
              0.5 * Jmax * (_t6 - _t7) * (_t6 - _t7);
  if (v2 < 0) {  //无匀减
    double _tmp = sqrt(((vs - Vmax) + 0.5 * as * as / Jmax) / Jmax);
    _t5 = _tmp + (as / Jmax);
    _t6 = _t5;
    ac_Amax = fabs(-Jmax * (_t5 - _t4) + as);
    _t7 = _t6 + ac_Amax / Jmax;
  } else {  //有匀减
    _t5 = (Amax + as) / Jmax;
    _t6 = v2 / Amax + _t5;
    _t7 = _t6 + Amax / Jmax;
    ac_Amax = Amax;
  }

  inter_mid->Taa_3 = __t4 + (_t5 - _t4);
  inter_mid->Taa_2 = _t6 - _t5;
  inter_mid->Taa_1 = _t7 - _t6;

  as = inter_in->acc_start;

  double v5 = vs + as * (inter_mid->Taa_3) -
              0.5 * Jmax * (inter_mid->Taa_3) * (inter_mid->Taa_3);
  double v6 = v5 - Amax * (inter_mid->Taa_2);

  double s5 = vs * (inter_mid->Taa_3) +
              0.5 * as * (inter_mid->Taa_3) * (inter_mid->Taa_3) -
              (1.0 / 6.0) * Jmax * (inter_mid->Taa_3) * (inter_mid->Taa_3) *
                  (inter_mid->Taa_3);
  double s6 = s5 + v5 * (inter_mid->Taa_2) -
              0.5 * Amax * (inter_mid->Taa_2) * (inter_mid->Taa_2);
  inter_mid->S_1 = s6 + v6 * (inter_mid->Taa_1) -
                   0.5 * Jmax * (inter_mid->Taa_1) * (inter_mid->Taa_1) *
                       (inter_mid->Taa_1) +
                   (1.0 / 6.0) * Jmax * (inter_mid->Taa_1) *
                       (inter_mid->Taa_1) * (inter_mid->Taa_1);

  inter_in->S = inter_in->S - inter_mid->S_1;
  if (inter_in->S < 0) {
    if (inter_in->S < -(1e-6)) {
      return -1;
    } else {
      inter_in->S = 0;
    }
  }

  inter_in->acc_start = 0;
  inter_in->v_start = Vmax;

  int ret = _DoubleS_project_full(inter_in, inter_mid);
  return ret;
}

static int DoubleS_project_zero_vmax(DoubleS_in *inter_in,
                                     DoubleS_mid *inter_mid) {
  inter_mid->type = _velocity_profile_no;
  inter_mid->T = 0;
  inter_mid->Taa_3 = 0;
  inter_mid->Taa_2 = 0;
  inter_mid->Taa_1 = 0;
  inter_mid->S_1 = 0;

  inter_mid->Taa = 0;
  inter_mid->Tca = 0;
  inter_mid->Tda = 0;
  inter_mid->Tv = 0;
  inter_mid->Tad = 0;
  inter_mid->Tcd = 0;
  inter_mid->Tdd = 0;
  return 0;
}

/*基础插补规划函数
 * inter_in   初始化输入；
 * inter_mid 初始化输出；
 *
 * return 0:正确； -1: 输入错误；
 * */
int DoubleS_project(DoubleS_in *inter_in, DoubleS_mid *inter_mid) {
  int ret = -1;
  int err = DoubleS_check(inter_in);
  if (err < 0) {
    return -1;
  }

  switch (err) {
  case _velocity_profile_full:
    ret = DoubleS_project_full(inter_in, inter_mid);
    break;
  case _velocity_profile_down:
    ret = DoubleS_project_down(inter_in, inter_mid);
    break;
  case _velocity_profile_full_add:
    ret = DoubleS_project_full_add(inter_in, inter_mid);
    break;
  case _velocity_profile_full_vmax:
    ret = DoubleS_project_full_vmax(inter_in, inter_mid);
    break;
  case _velocity_profile_no:
    ret = DoubleS_project_zero_vmax(inter_in, inter_mid);
    break;
  default:
    ret = -100;
    break;
  }

  return ret;
}

/*基础插补数据计算
 * inter_in   输入
 * inter_mid 中间输入
 * inter_out 插补结果输出
 *
 *return 0:满足; 1: 运动
 * */
static int DoubleS_computer_full(DoubleS_in *inter_in, DoubleS_mid *inter_mid,
                                 Velocityprofile_out *inter_out) {
  double s = 0;
  double v = 0;
  double a = 0;
  double j = 0;
  double t1 = inter_mid->Taa;
  double t2 = t1 + inter_mid->Tca;
  double t3 = t2 + inter_mid->Tda;
  double t4 = t3 + inter_mid->Tv;
  double t5 = t4 + inter_mid->Tad;
  double t6 = t5 + inter_mid->Tcd;
  double t7 = inter_mid->T;

  double vs = inter_in->v_start;
  double as = inter_in->acc_start;
  double Amax = inter_in->acc_max;
  double Jmax = inter_in->jerk_max;

  double v1 = vs + as * t1 + 0.5 * Jmax * t1 * t1;
  double v2 = v1 + Amax * (t2 - t1);
  double v3 = v2 + 0.5 * Jmax * (t2 - t3) * (t2 - t3);
  double v4 = v3;
  double v5 = v4 - 0.5 * Jmax * (t5 - t4) * (t5 - t4);
  double v6 = v5 - Amax * (t6 - t5);

  double s1 = vs * t1 + 0.5 * as * t1 * t1 + (1.0 / 6.0) * Jmax * t1 * t1 * t1;
  double s2 = s1 + v1 * (t2 - t1) + 0.5 * Amax * (t2 - t1) * (t2 - t1);
  double s3 = s2 + v2 * (t3 - t2) +
              0.5 * Jmax * (t2 - t3) * (t2 - t3) * (t3 - t2) +
              (1.0 / 6.0) * Jmax * (t2 - t3) * (t2 - t3) * (t2 - t3);
  double s4 = s3 + v3 * (t4 - t3);
  double s5 = s4 + v4 * (t5 - t4) -
              (1.0 / 6.0) * Jmax * (t5 - t4) * (t5 - t4) * (t5 - t4);
  double s6 = s5 + v5 * (t6 - t5) - 0.5 * Amax * (t6 - t5) * (t6 - t5);

  inter_out->state = _velocityprofile_run;

  double tt = inter_out->t + inter_in->dt;
  double t_ = tt - inter_in->t0;
  double t = 0;
  if ((inter_mid->T < 1e-6) || (inter_in->S < 1e-6)) {
    inter_out->t = tt;
    inter_out->s = inter_in->s0;
    inter_out->v = 0;
    inter_out->a = 0;
    inter_out->j = 0;
    // ! 这两个state的含义以及直接赋值是否正确
    inter_out->state = inter_in->stop_state;
    return inter_out->state;
  }

  if (t_ < 0) {
    t = 0;
    // } else if (t_ >= (inter_mid->T - inter_in->dt / 2.0))
  } else if (t_ >= (inter_mid->T)) {
    t = inter_mid->T;
  } else {
    t = t_;
  }

  if (t >= 0 && t < t1) {
    s = vs * t + 0.5 * as * t * t + (1.0 / 6.0) * Jmax * t * t * t;
    v = vs + as * t + 0.5 * Jmax * t * t;
    a = as + Jmax * t;
    j = Jmax;
  } else if (t >= t1 && t < t2) {
    s = s1 + v1 * (t - t1) + 0.5 * Amax * (t - t1) * (t - t1);
    v = v1 + Amax * (t - t1);
    a = as + Jmax * t1;
    j = 0;
  } else if (t >= t2 && t < t3) {
    s = s2 + v2 * (t - t2) + 0.5 * Jmax * (t2 - t3) * (t2 - t3) * (t - t2) -
        (1.0 / 6.0) * Jmax * (t - t3) * (t - t3) * (t - t3) +
        (1.0 / 6.0) * Jmax * (t2 - t3) * (t2 - t3) * (t2 - t3);
    v = v2 + 0.5 * Jmax * (t2 - t3) * (t2 - t3) -
        0.5 * Jmax * (t - t3) * (t - t3);
    a = as + Jmax * t1 - Jmax * (t - t2);
    j = -Jmax;
  } else if (t >= t3 && t < t4) {
    s = s3 + v3 * (t - t3);
    v = v3;
    a = 0;
    j = 0;
    inter_out->state = _velocityprofile_run_constant_velocity;
  } else if (t >= t4 && t < t5) {
    s = s4 + v4 * (t - t4) -
        (1.0 / 6.0) * Jmax * (t - t4) * (t - t4) * (t - t4);
    v = v4 - 0.5 * Jmax * (t - t4) * (t - t4);
    a = -Jmax * (t - t4);
    j = -Jmax;
  } else if (t >= t5 && t < t6) {
    s = s5 + v5 * (t - t5) - 0.5 * Amax * (t - t5) * (t - t5);
    v = v5 - Amax * (t - t5);
    a = -Jmax * (t5 - t4);
    j = 0;
  } else if (t >= t6 && t <= t7) {
    s = s6 + v6 * (t - t6) - 0.5 * Jmax * (t6 - t7) * (t6 - t7) * (t - t6) +
        (1.0 / 6.0) * Jmax * (t - t7) * (t - t7) * (t - t7) -
        (1.0 / 6.0) * Jmax * (t6 - t7) * (t6 - t7) * (t6 - t7);
    v = v6 - 0.5 * Jmax * (t6 - t7) * (t6 - t7) +
        0.5 * Jmax * (t - t7) * (t - t7);
    a = Jmax * (t - t7);
    j = Jmax;
  }
  inter_out->t = tt;
  inter_out->s = s + inter_in->s0;
  inter_out->v = v;
  inter_out->a = a;
  inter_out->j = j;

  //  if (t>=(inter_mid->T-inter_in->dt/2.0))
  if (t >= (inter_mid->T - inter_in->dt)) {
    // 确保精确结束
    //  inter_out->s=inter_in->S+inter_in->s0;
    //  inter_out->v=inter_in->ve;
    //  inter_out->a=inter_in->ae;
    inter_out->state = inter_in->stop_state;
    return inter_out->state;  //插补完成
  } else {
    return inter_out->state;
  }
}  // namespace rosc

/*基础插补数据计算
 * inter_in   输入
 * inter_mid 中间输入
 * inter_out 插补结果输出
 *
 *return 0:满足; 1: 运动
 * */
static int DoubleS_computer_down(DoubleS_in *inter_in, DoubleS_mid *inter_mid,
                                 Velocityprofile_out *inter_out) {
  double s = 0;
  double v = 0;
  double a = 0;
  double j = 0;
  double t1 = inter_mid->Taa;
  double t2 = t1 + inter_mid->Tca;
  double t3 = t2 + inter_mid->Tda;
  double t4 = t3 + inter_mid->Tv;
  double t5 = t4 + inter_mid->Tad;
  double t6 = t5 + inter_mid->Tcd;
  double t7 = inter_mid->T;

  double vs = inter_in->v_start;
  double as = inter_in->acc_start;
  double Amax = inter_in->acc_max;
  double Jmax = inter_in->jerk_max;

  t4 = 0;  // vs，as对应的时刻

  double a5 = as - Jmax * (t5 - t4);

  double v5 = vs + as * (t5 - t4) - 0.5 * Jmax * (t5 - t4) * (t5 - t4);
  double v6 = v5 - Amax * (t6 - t5);

  double s5 = vs * (t5 - t4) + 0.5 * as * (t5 - t4) * (t5 - t4) -
              (1.0 / 6.0) * Jmax * (t5 - t4) * (t5 - t4) * (t5 - t4);
  double s6 = s5 + v5 * (t6 - t5) - 0.5 * Amax * (t6 - t5) * (t6 - t5);

  inter_out->state = _velocityprofile_run;

  double tt = inter_out->t + inter_in->dt;
  double t_ = tt - inter_in->t0;
  double t = 0;

  if ((inter_mid->T < 1e-6) && (fabs(inter_in->v_start) < 1e-6)) {
    inter_out->t = tt;
    inter_out->s = inter_in->s0;
    inter_out->v = 0;
    inter_out->a = 0;
    inter_out->j = 0;
    inter_out->state = inter_in->stop_state;
    return inter_out->state;
  }

  if (t_ < 0) {
    t = 0;
    // } else if (t_>=(inter_mid->T-inter_in->dt/2.0))
  } else if (t_ >= (inter_mid->T)) {
    t = inter_mid->T;
  } else {
    t = t_;
  }

  if (t < t5) {
    s = vs * (t - t4) + 0.5 * as * (t - t4) * (t - t4) -
        (1.0 / 6.0) * Jmax * (t - t4) * (t - t4) * (t - t4);
    v = vs + as * (t - t4) - 0.5 * Jmax * (t - t4) * (t - t4);
    a = -Jmax * (t - t4) + as;
    j = -Jmax;
  } else if (t >= t5 && t < t6) {
    s = s5 + v5 * (t - t5) - 0.5 * Amax * (t - t5) * (t - t5);
    v = v5 - Amax * (t - t5);
    a = a5;
    j = 0;
  } else if (t >= t6 && t <= t7) {
    s = s6 + v6 * (t - t6) - 0.5 * Jmax * (t6 - t7) * (t6 - t7) * (t - t6) +
        (1.0 / 6.0) * Jmax * (t - t7) * (t - t7) * (t - t7) -
        (1.0 / 6.0) * Jmax * (t6 - t7) * (t6 - t7) * (t6 - t7);
    v = v6 - 0.5 * Jmax * (t6 - t7) * (t6 - t7) +
        0.5 * Jmax * (t - t7) * (t - t7);
    a = Jmax * (t - t7);
    j = Jmax;
  }
  inter_out->t = tt;
  inter_out->s = s + inter_in->s0;
  inter_out->v = v;
  inter_out->a = a;
  inter_out->j = j;

  //  if (t>=(inter_mid->T-inter_in->dt/2.0))
  if (t >= (inter_mid->T - inter_in->dt)) {
    //确保精确结束
    //  inter_out->v=0;
    //  inter_out->a=0;
    //  inter_out->j=0;
    inter_out->state = inter_in->stop_state;
    return inter_out->state;  //插补完成
  } else {
    return inter_out->state;
  }
}

/*基础插补数据计算
 * inter_in   输入
 * inter_mid 中间输入
 * inter_out 插补结果输出
 *
 *return 0:满足; 1: 运动
 * */
static int DoubleS_computer_full_add(DoubleS_in *inter_in,
                                     DoubleS_mid *inter_mid,
                                     Velocityprofile_out *inter_out) {
  return DoubleS_computer_full(inter_in, inter_mid, inter_out);
}

/*基础插补数据计算
 * inter_in   输入
 * inter_mid 中间输入
 * inter_out 插补结果输出
 *
 *return 0:满足; 1: 运动
 * */
static int DoubleS_computer_full_vmax(DoubleS_in *inter_in,
                                      DoubleS_mid *inter_mid,
                                      Velocityprofile_out *inter_out) {
  int ret = 0;
  double tt = inter_out->t + inter_in->dt;
  double dt = tt - inter_in->t0;
  double T = inter_mid->Taa_1 + inter_mid->Taa_2 + inter_mid->Taa_3;
  if (dt <= T) {

    double s = 0;
    double v = 0;
    double a = 0;
    double j = 0;
    double t4 = 0;
    double t5 = t4 + inter_mid->Taa_3;
    double t6 = t5 + inter_mid->Taa_2;
    double t7 = t6 + inter_mid->Taa_1;

    double vs = inter_in->vs_1;
    double as = inter_in->as_1;
    double Amax = inter_in->acc_max;
    double Jmax = inter_in->jerk_max;

    double a5 = as - Jmax * (t5 - t4);

    double v5 = vs + as * (t5 - t4) - 0.5 * Jmax * (t5 - t4) * (t5 - t4);
    double v6 = v5 - Amax * (t6 - t5);

    double s5 = vs * (t5 - t4) + 0.5 * as * (t5 - t4) * (t5 - t4) -
                (1.0 / 6.0) * Jmax * (t5 - t4) * (t5 - t4) * (t5 - t4);
    double s6 = s5 + v5 * (t6 - t5) - 0.5 * Amax * (t6 - t5) * (t6 - t5);

    inter_out->state = _velocityprofile_run;

    double t = dt;

    if (t < t5) {
      s = vs * (t - t4) + 0.5 * as * (t - t4) * (t - t4) -
          (1.0 / 6.0) * Jmax * (t - t4) * (t - t4) * (t - t4);
      v = vs + as * (t - t4) - 0.5 * Jmax * (t - t4) * (t - t4);
      a = -Jmax * (t - t4) + as;
      j = -Jmax;
    } else if (t >= t5 && t < t6) {
      s = s5 + v5 * (t - t5) - 0.5 * Amax * (t - t5) * (t - t5);
      v = v5 - Amax * (t - t5);
      a = a5;
      j = 0;
    } else if (t >= t6 && t <= t7) {
      s = s6 + v6 * (t - t6) - 0.5 * Jmax * (t6 - t7) * (t6 - t7) * (t - t6) +
          (1.0 / 6.0) * Jmax * (t - t7) * (t - t7) * (t - t7) -
          (1.0 / 6.0) * Jmax * (t6 - t7) * (t6 - t7) * (t6 - t7);
      v = v6 - 0.5 * Jmax * (t6 - t7) * (t6 - t7) +
          0.5 * Jmax * (t - t7) * (t - t7);
      a = Jmax * (t - t7);
      j = Jmax;
    }
    inter_out->t = tt;
    inter_out->s = s + inter_in->s0;
    inter_out->v = v;
    inter_out->a = a;
    inter_out->j = j;

    if (inter_mid->T > 0) {
      ret = inter_out->state;
    } else {
      if (t >= (T - inter_in->dt)) {
        inter_out->state = inter_in->stop_state;
        ret = inter_out->state;  //插补完成
      } else {
        ret = inter_out->state;
      }
    }
  } else {
    inter_out->t =
        inter_out->t - (inter_mid->Taa_1 + inter_mid->Taa_2 + inter_mid->Taa_3);
    ret = DoubleS_computer_full(inter_in, inter_mid, inter_out);
    inter_out->s = inter_out->s + inter_mid->S_1;
    inter_out->t =
        inter_out->t + (inter_mid->Taa_1 + inter_mid->Taa_2 + inter_mid->Taa_3);
  }

  return ret;
}

static int DoubleS_computer_zero_vmax(DoubleS_in *inter_in,
                                      DoubleS_mid *inter_mid,
                                      Velocityprofile_out *inter_out) {
  inter_out->t = inter_out->t + inter_in->dt;
  inter_out->state = _velocityprofile_constant_stop;  // inter_in->StopState;
  return inter_out->state;
}

/*基础插补数据计算
 * inter_in   输入
 * inter_mid 中间输入
 * inter_out 插补结果输出
 *
 *return 0:满足; 1: 运动
 * */
int DoubleS_computer(DoubleS_in *inter_in, DoubleS_mid *inter_mid,
                     Velocityprofile_out *inter_out) {
  int ret = 0;
  // int type = getDoubleSType(inter_mid);
  int type = inter_mid->GetDoubleSType();

  switch (type) {
  case _velocity_profile_full:
    ret = DoubleS_computer_full(inter_in, inter_mid, inter_out);
    break;
  case _velocity_profile_down:
    ret = DoubleS_computer_down(inter_in, inter_mid, inter_out);
    break;
  case _velocity_profile_full_add:
    ret = DoubleS_computer_full_add(inter_in, inter_mid, inter_out);
    break;
  case _velocity_profile_full_vmax:
    ret = DoubleS_computer_full_vmax(inter_in, inter_mid, inter_out);
    break;
  case _velocity_profile_no:
    ret = DoubleS_computer_zero_vmax(inter_in, inter_mid, inter_out);
    break;
  }

  return ret;
}

static double DoubleS_computer_distance_full(DoubleS_in *inter_in,
                                             DoubleS_mid *inter_mid, double t) {
  if ((inter_mid->T < 1e-6) || (inter_in->S < 1e-6)) {
    return inter_in->s0;
  }
  double s = 0;
  double t1 = inter_mid->Taa;
  double t2 = t1 + inter_mid->Tca;
  double t3 = t2 + inter_mid->Tda;
  double t4 = t3 + inter_mid->Tv;
  double t5 = t4 + inter_mid->Tad;
  double t6 = t5 + inter_mid->Tcd;
  double t7 = inter_mid->T;

  double vs = inter_in->v_start;
  double as = inter_in->acc_start;
  double Amax = inter_in->acc_max;
  double Jmax = inter_in->jerk_max;

  double v1 = vs + as * t1 + 0.5 * Jmax * t1 * t1;
  double v2 = v1 + Amax * (t2 - t1);
  double v3 = v2 + 0.5 * Jmax * (t2 - t3) * (t2 - t3);
  double v4 = v3;
  double v5 = v4 - 0.5 * Jmax * (t5 - t4) * (t5 - t4);
  double v6 = v5 - Amax * (t6 - t5);

  double s1 = vs * t1 + 0.5 * as * t1 * t1 + (1.0 / 6.0) * Jmax * t1 * t1 * t1;
  double s2 = s1 + v1 * (t2 - t1) + 0.5 * Amax * (t2 - t1) * (t2 - t1);
  double s3 = s2 + v2 * (t3 - t2) +
              0.5 * Jmax * (t2 - t3) * (t2 - t3) * (t3 - t2) +
              (1.0 / 6.0) * Jmax * (t2 - t3) * (t2 - t3) * (t2 - t3);
  double s4 = s3 + v3 * (t4 - t3);
  double s5 = s4 + v4 * (t5 - t4) -
              (1.0 / 6.0) * Jmax * (t5 - t4) * (t5 - t4) * (t5 - t4);
  double s6 = s5 + v5 * (t6 - t5) - 0.5 * Amax * (t6 - t5) * (t6 - t5);

  t = t - inter_in->t0;

  if (t < 0) {
    t = 0;
  } else if (t > (inter_mid->T - inter_in->dt / 2.0)) {
    t = inter_mid->T;
  }

  if (t >= 0 && t < t1) {
    s = vs * t + 0.5 * as * t * t + (1.0 / 6.0) * Jmax * t * t * t;
  } else if (t >= t1 && t < t2) {
    s = s1 + v1 * (t - t1) + 0.5 * Amax * (t - t1) * (t - t1);
  } else if (t >= t2 && t < t3) {
    s = s2 + v2 * (t - t2) + 0.5 * Jmax * (t2 - t3) * (t2 - t3) * (t - t2) -
        (1.0 / 6.0) * Jmax * (t - t3) * (t - t3) * (t - t3) +
        (1.0 / 6.0) * Jmax * (t2 - t3) * (t2 - t3) * (t2 - t3);
  } else if (t >= t3 && t < t4) {
    s = s3 + v3 * (t - t3);
  } else if (t >= t4 && t < t5) {
    s = s4 + v4 * (t - t4) -
        (1.0 / 6.0) * Jmax * (t - t4) * (t - t4) * (t - t4);
  } else if (t >= t5 && t < t6) {
    s = s5 + v5 * (t - t5) - 0.5 * Amax * (t - t5) * (t - t5);
  } else if (t >= t6 && t <= t7) {
    s = s6 + v6 * (t - t6) - 0.5 * Jmax * (t6 - t7) * (t6 - t7) * (t - t6) +
        (1.0 / 6.0) * Jmax * (t - t7) * (t - t7) * (t - t7) -
        (1.0 / 6.0) * Jmax * (t6 - t7) * (t6 - t7) * (t6 - t7);
  }

  return s + inter_in->s0;
}

static double DoubleS_computer_distance_down(DoubleS_in *inter_in,
                                             DoubleS_mid *inter_mid, double t) {
  if ((inter_mid->T < 1e-6) || (inter_in->S < 1e-6)) {
    return inter_in->s0;
  }
  double s = 0;
  double t1 = inter_mid->Taa;
  double t2 = t1 + inter_mid->Tca;
  double t3 = t2 + inter_mid->Tda;
  double t4 = t3 + inter_mid->Tv;
  double t5 = t4 + inter_mid->Tad;
  double t6 = t5 + inter_mid->Tcd;
  double t7 = inter_mid->T;

  double vs = inter_in->v_start;
  double as = inter_in->acc_start;
  double Amax = inter_in->acc_max;
  double Jmax = inter_in->jerk_max;

  t4 = 0;  // vs，as对应的时刻

  double v5 = vs + as * (t5 - t4) - 0.5 * Jmax * (t5 - t4) * (t5 - t4);
  double v6 = v5 - Amax * (t6 - t5);

  double s5 = vs * (t5 - t4) + 0.5 * as * (t5 - t4) * (t5 - t4) -
              (1.0 / 6.0) * Jmax * (t5 - t4) * (t5 - t4) * (t5 - t4);
  double s6 = s5 + v5 * (t6 - t5) - 0.5 * Amax * (t6 - t5) * (t6 - t5);

  t = t - inter_in->t0;

  if (t < 0) {
    t = 0;
  } else if (t > (inter_mid->T - inter_in->dt / 2.0)) {
    t = inter_mid->T;
  }

  if (t < t5) {
    s = vs * (t - t4) + 0.5 * as * (t - t4) * (t - t4) -
        (1.0 / 6.0) * Jmax * (t - t4) * (t - t4) * (t - t4);
  } else if (t >= t5 && t < t6) {
    s = s5 + v5 * (t - t5) - 0.5 * Amax * (t - t5) * (t - t5);
  } else if (t >= t6 && t <= t7) {
    s = s6 + v6 * (t - t6) - 0.5 * Jmax * (t6 - t7) * (t6 - t7) * (t - t6) +
        (1.0 / 6.0) * Jmax * (t - t7) * (t - t7) * (t - t7) -
        (1.0 / 6.0) * Jmax * (t6 - t7) * (t6 - t7) * (t6 - t7);
  }

  return s + inter_in->s0;
}

static double DoubleS_computer_distance_full_vmax(DoubleS_in *inter_in,
                                                  DoubleS_mid *inter_mid,
                                                  double t) {
  double s = 0;
  double tt = t;
  tt = tt - inter_in->t0;
  if (tt <= (inter_mid->Taa_1 + inter_mid->Taa_2 + inter_mid->Taa_3)) {
    double t4 = 0;
    double t5 = t4 + inter_mid->Taa_3;
    double t6 = t5 + inter_mid->Taa_2;
    double t7 = t6 + inter_mid->Taa_1;

    double vs = inter_in->vs_1;
    double as = inter_in->as_1;
    double Amax = inter_in->acc_max;
    double Jmax = inter_in->jerk_max;

    double v5 = vs + as * (t5 - t4) - 0.5 * Jmax * (t5 - t4) * (t5 - t4);
    double v6 = v5 - Amax * (t6 - t5);

    double s5 = vs * (t5 - t4) + 0.5 * as * (t5 - t4) * (t5 - t4) -
                (1.0 / 6.0) * Jmax * (t5 - t4) * (t5 - t4) * (t5 - t4);
    double s6 = s5 + v5 * (t6 - t5) - 0.5 * Amax * (t6 - t5) * (t6 - t5);
    double t = tt;

    if (t < t5) {
      s = vs * (t - t4) + 0.5 * as * (t - t4) * (t - t4) -
          (1.0 / 6.0) * Jmax * (t - t4) * (t - t4) * (t - t4);
    } else if (t >= t5 && t < t6) {
      s = s5 + v5 * (t - t5) - 0.5 * Amax * (t - t5) * (t - t5);
    } else if (t >= t6 && t <= t7) {
      s = s6 + v6 * (t - t6) - 0.5 * Jmax * (t6 - t7) * (t6 - t7) * (t - t6) +
          (1.0 / 6.0) * Jmax * (t - t7) * (t - t7) * (t - t7) -
          (1.0 / 6.0) * Jmax * (t6 - t7) * (t6 - t7) * (t6 - t7);
    }
    return s + inter_in->s0;
  } else {
    t = t - (inter_mid->Taa_1 + inter_mid->Taa_2 + inter_mid->Taa_3);
    return DoubleS_computer_distance_full(inter_in, inter_mid, t) +
           inter_mid->S_1;
  }
}

double DoubleS_computer_distance(DoubleS_in *inter_in, DoubleS_mid *inter_mid,
                                 double t) {
  double ret = 0;
  // int type = getDoubleSType(inter_mid);
  int type = inter_mid->GetDoubleSType();
  switch (type) {
  case _velocity_profile_full:
    ret = DoubleS_computer_distance_full(inter_in, inter_mid, t);
    break;
  case _velocity_profile_down:
    ret = DoubleS_computer_distance_down(inter_in, inter_mid, t);
    break;
  case _velocity_profile_full_add:
    ret = DoubleS_computer_distance_full(inter_in, inter_mid, t);
    break;
  case _velocity_profile_full_vmax:
    ret = DoubleS_computer_distance_full_vmax(inter_in, inter_mid, t);
    break;
  }
  return ret;
}

static double _doubles_setting_time(double *res, int n, double tlow,
                                    double thigh, double ts) {
  int i = 0;
  int tmp = 0;
  for (i = 0; i < n; i++) {
    if ((res[i] > (tlow - 1e-6)) && (res[i] < (thigh + 1e-6))) {
      tmp = (int)(res[i] / ts + 0.5);
      return ts * tmp;
    }
  }
  return 0;
}

static double DoubleS_computer_time_full(DoubleS_in *inter_in,
                                         DoubleS_mid *inter_mid, double s) {
  if ((inter_mid->T < 1e-6) || (inter_in->S < 1e-6)) {
    return inter_in->t0;
  }
  int i = 0;
  double t1 = inter_mid->Taa;
  double t2 = t1 + inter_mid->Tca;
  double t3 = t2 + inter_mid->Tda;
  double t4 = t3 + inter_mid->Tv;
  double t5 = t4 + inter_mid->Tad;
  double t6 = t5 + inter_mid->Tcd;
  double t7 = inter_mid->T;

  double vs = inter_in->v_start;
  double as = inter_in->acc_start;
  double Amax = inter_in->acc_max;
  double Jmax = inter_in->jerk_max;

  double v1 = vs + as * t1 + 0.5 * Jmax * t1 * t1;
  double v2 = v1 + Amax * (t2 - t1);
  double v3 = v2 + 0.5 * Jmax * (t2 - t3) * (t2 - t3);
  double v4 = v3;
  double v5 = v4 - 0.5 * Jmax * (t5 - t4) * (t5 - t4);
  double v6 = v5 - Amax * (t6 - t5);

  double s1 = vs * t1 + 0.5 * as * t1 * t1 + (1.0 / 6.0) * Jmax * t1 * t1 * t1;
  double s2 = s1 + v1 * (t2 - t1) + 0.5 * Amax * (t2 - t1) * (t2 - t1);
  double s3 = s2 + v2 * (t3 - t2) +
              0.5 * Jmax * (t2 - t3) * (t2 - t3) * (t3 - t2) +
              (1.0 / 6.0) * Jmax * (t2 - t3) * (t2 - t3) * (t2 - t3);
  double s4 = s3 + v3 * (t4 - t3);
  double s5 = s4 + v4 * (t5 - t4) -
              (1.0 / 6.0) * Jmax * (t5 - t4) * (t5 - t4) * (t5 - t4);
  double s6 = s5 + v5 * (t6 - t5) - 0.5 * Amax * (t6 - t5) * (t6 - t5);
  double s7 = s6 + v6 * (t7 - t6) -
              0.5 * Jmax * (t6 - t7) * (t6 - t7) * (t7 - t6) -
              (1.0 / 6.0) * Jmax * (t6 - t7) * (t6 - t7) * (t6 - t7);

  s = s - inter_in->s0;
  double ret = 0;
  double res[3];
  int res_n = 0;
  if (s <= 0) {
    ret = 0;
  }
  if (s > 0 && s <= s1) {
    res_n = quartic_equation((1.0 / 6.0) * Jmax, 0.5 * as, vs, -s, res);
    ret = _doubles_setting_time(res, res_n, 0, t1, inter_in->dt);
  } else if (s > s1 && s <= s2) {
    res_n = square_equation(0.5 * Amax, v1, s1 - s, res);
    for (i = 0; i < res_n; i++) {
      res[i] = res[i] + t1;
    }
    ret = _doubles_setting_time(res, res_n, t1, t2, inter_in->dt);
  } else if (s > s2 && s <= s3) {
    res_n = quartic_equation(
        -(1.0 / 6.0) * Jmax, 0, (v2 + 0.5 * Jmax * (t2 - t3) * (t2 - t3)),
        (s2 + v2 * (t3 - t2) + 0.5 * Jmax * (t2 - t3) * (t2 - t3) * (t3 - t2) +
         (1.0 / 6.0) * Jmax * (t2 - t3) * (t2 - t3) * (t2 - t3) - s),
        res);
    for (i = 0; i < res_n; i++) {
      res[i] = res[i] + t3;
    }
    ret = _doubles_setting_time(res, res_n, t2, t3, inter_in->dt);
  } else if (s > s3 && s <= s4) {
    res_n = first_equation(v3, s3 - s, res);
    for (i = 0; i < res_n; i++) {
      res[i] = res[i] + t3;
    }
    ret = _doubles_setting_time(res, res_n, t3, t4, inter_in->dt);
  } else if (s > s4 && s < s5) {
    res_n = quartic_equation(-(1.0 / 6.0) * Jmax, 0, v4, s4 - s, res);
    for (i = 0; i < res_n; i++) {
      res[i] = res[i] + t4;
    }
    ret = _doubles_setting_time(res, res_n, t4, t5, inter_in->dt);
  } else if (s >= s5 && s < s6) {
    res_n = square_equation(-0.5 * Amax, v5, s5 - s, res);
    for (i = 0; i < res_n; i++) {
      res[i] = res[i] + t5;
    }
    ret = _doubles_setting_time(res, res_n, t5, t6, inter_in->dt);
  } else if (s >= s6 && s < s7) {
    res_n = quartic_equation(
        (1.0 / 6.0) * Jmax, 0, (v6 - 0.5 * Jmax * (t6 - t7) * (t6 - t7)),
        (s6 + v6 * (t7 - t6) - 0.5 * Jmax * (t6 - t7) * (t6 - t7) * (t7 - t6) -
         (1.0 / 6.0) * Jmax * (t6 - t7) * (t6 - t7) * (t6 - t7) - s),
        res);
    for (i = 0; i < res_n; i++) {
      res[i] = res[i] + t7;
    }
    ret = _doubles_setting_time(res, res_n, t6, t7, inter_in->dt);
  } else {
    ret = t7;
  }
  return ret + inter_in->t0;
}

static double DoubleS_computer_time_down(DoubleS_in *inter_in,
                                         DoubleS_mid *inter_mid, double s) {
  if ((inter_mid->T < 1e-6) || (inter_in->S < 1e-6)) {
    return inter_in->t0;
  }
  int i = 0;
  double t1 = inter_mid->Taa;
  double t2 = t1 + inter_mid->Tca;
  double t3 = t2 + inter_mid->Tda;
  double t4 = t3 + inter_mid->Tv;
  double t5 = t4 + inter_mid->Tad;
  double t6 = t5 + inter_mid->Tcd;
  double t7 = inter_mid->T;

  double vs = inter_in->v_start;
  double as = inter_in->acc_start;
  double Amax = inter_in->acc_max;
  double Jmax = inter_in->jerk_max;

  t4 = 0;  // vs，as对应的时刻

  double v5 = vs + as * (t5 - t4) - 0.5 * Jmax * (t5 - t4) * (t5 - t4);
  double v6 = v5 - Amax * (t6 - t5);

  double s5 = vs * (t5 - t4) + 0.5 * as * (t5 - t4) * (t5 - t4) -
              (1.0 / 6.0) * Jmax * (t5 - t4) * (t5 - t4) * (t5 - t4);
  double s6 = s5 + v5 * (t6 - t5) - 0.5 * Amax * (t6 - t5) * (t6 - t5);
  double s7 = s6 + v6 * (t7 - t6) -
              0.5 * Jmax * (t6 - t7) * (t6 - t7) * (t7 - t6) -
              (1.0 / 6.0) * Jmax * (t6 - t7) * (t6 - t7) * (t6 - t7);

  s = s - inter_in->s0;
  double ret = 0;
  double res[3];
  int res_n = 0;
  if (s <= 0) {
    ret = 0;
  }
  if (s > 0 && s <= s5) {
    res_n = quartic_equation(-(1.0 / 6.0) * Jmax, 0.5 * as, vs, -s, res);
    for (i = 0; i < res_n; i++) {
      res[i] = res[i] + t4;
    }
    ret = _doubles_setting_time(res, res_n, t4, t5, inter_in->dt);
  } else if (s > s5 && s <= s6) {
    res_n = square_equation(-0.5 * Amax, v5, s5 - s, res);
    for (i = 0; i < res_n; i++) {
      res[i] = res[i] + t5;
    }
    ret = _doubles_setting_time(res, res_n, t5, t6, inter_in->dt);
  } else if (s > s6 && s <= s7) {
    res_n = quartic_equation(
        (1.0 / 6.0) * Jmax, 0, (v6 - 0.5 * Jmax * (t6 - t7) * (t6 - t7)),
        (s6 + v6 * (t7 - t6) - 0.5 * Jmax * (t6 - t7) * (t6 - t7) * (t7 - t6) -
         (1.0 / 6.0) * Jmax * (t6 - t7) * (t6 - t7) * (t6 - t7) - s),
        res);
    for (i = 0; i < res_n; i++) {
      res[i] = res[i] + t7;
    }
    ret = _doubles_setting_time(res, res_n, t6, t7, inter_in->dt);
  } else {
    ret = t7;
  }

  return ret + inter_in->t0;
}

static double DoubleS_computer_time_full_vmax(DoubleS_in *inter_in,
                                              DoubleS_mid *inter_mid,
                                              double s) {
  double ss = s - inter_in->s0;
  if (ss <= inter_mid->S_1) {
    int i = 0;
    double t4 = 0;
    double t5 = t4 + inter_mid->Taa_3;
    double t6 = t5 + inter_mid->Taa_2;
    double t7 = t6 + inter_mid->Taa_1;

    double vs = inter_in->vs_1;
    double as = inter_in->as_1;
    double Amax = inter_in->acc_max;
    double Jmax = inter_in->jerk_max;

    double v5 = vs + as * (t5 - t4) - 0.5 * Jmax * (t5 - t4) * (t5 - t4);
    double v6 = v5 - Amax * (t6 - t5);

    double s5 = vs * (t5 - t4) + 0.5 * as * (t5 - t4) * (t5 - t4) -
                (1.0 / 6.0) * Jmax * (t5 - t4) * (t5 - t4) * (t5 - t4);
    double s6 = s5 + v5 * (t6 - t5) - 0.5 * Amax * (t6 - t5) * (t6 - t5);
    double s7 = s6 + v6 * (t7 - t6) -
                0.5 * Jmax * (t6 - t7) * (t6 - t7) * (t7 - t6) -
                (1.0 / 6.0) * Jmax * (t6 - t7) * (t6 - t7) * (t6 - t7);

    double ret = 0;
    double res[3];
    int res_n = 0;
    double s = ss;
    if (s <= 0) {
      ret = 0;
    }

    if (s > 0 && s <= s5) {
      res_n = quartic_equation(-(1.0 / 6.0) * Jmax, 0.5 * as, vs, -s, res);
      for (i = 0; i < res_n; i++) {
        res[i] = res[i] + t4;
      }
      ret = _doubles_setting_time(res, res_n, t4, t5, inter_in->dt);
    } else if (s > s5 && s <= s6) {
      res_n = square_equation(-0.5 * Amax, v5, s5 - s, res);
      for (i = 0; i < res_n; i++) {
        res[i] = res[i] + t5;
      }
      ret = _doubles_setting_time(res, res_n, t5, t6, inter_in->dt);
    } else if (s > s6 && s <= s7) {
      res_n = quartic_equation(
          (1.0 / 6.0) * Jmax, 0, (v6 - 0.5 * Jmax * (t6 - t7) * (t6 - t7)),
          (s6 + v6 * (t7 - t6) -
           0.5 * Jmax * (t6 - t7) * (t6 - t7) * (t7 - t6) -
           (1.0 / 6.0) * Jmax * (t6 - t7) * (t6 - t7) * (t6 - t7) - s),
          res);
      for (i = 0; i < res_n; i++) {
        res[i] = res[i] + t7;
      }
      ret = _doubles_setting_time(res, res_n, t6, t7, inter_in->dt);
    } else {
      ret = t7;
    }

    return ret + inter_in->t0;
  } else {
    s = s - inter_mid->S_1;
    return DoubleS_computer_time_full(inter_in, inter_mid, s) +
           (inter_mid->Taa_1 + inter_mid->Taa_2 + inter_mid->Taa_3);
  }
}

double DoubleS_computer_time(DoubleS_in *inter_in, DoubleS_mid *inter_mid,
                             double s) {
  double ret = 0;
  // int type = getDoubleSType(inter_mid);
  int type = inter_mid->GetDoubleSType();
  switch (type) {
  case _velocity_profile_full:
    ret = DoubleS_computer_time_full(inter_in, inter_mid, s);
    break;
  case _velocity_profile_down:
    ret = DoubleS_computer_time_down(inter_in, inter_mid, s);
    break;
  case _velocity_profile_full_add:
    ret = DoubleS_computer_time_full(inter_in, inter_mid, s);
    break;
  case _velocity_profile_full_vmax:
    ret = DoubleS_computer_time_full_vmax(inter_in, inter_mid, s);
    break;
  }
  return ret;
}

double DoubleS_computer_prodfile_time(DoubleS_in *inter_in,
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

int DoubleS_sync_prodfile(DoubleS_in *inter_in, Velocityprofile_out *inter_out,
                          double T, double _T, int stopstate) {
  int err = syncDoubleSVelocityprofile(
      inter_in->v_start, &(inter_in->v_end), &(inter_in->v_max),
      inter_in->acc_max, inter_in->jerk_max, T,
      inter_in->_S + inter_in->s0 - inter_out->s, _T, NULL, stopstate);
  if (0 != err) {
    return -1;
  }
  return 0;
}

/*适用于起始和终止速度、加速度为0的情况*/
int DoubleSTimeToVelocity(double s, double time, double vmax, double amax,
                          double jmax, double *velocity) {
  int i = 0;
  double t = 2.0 * amax / jmax;
  if (s < 1e-6) {
    *velocity = 0;
    return 0;
  }
  if (time <= 2.0 * t) {
    double res[3] = {0, 0, 0};
    int res_n =
        quartic_equation((-jmax / 4.0), (time * jmax / 4.0), 0, -s, res);
    for (i = 0; i < res_n; i++) {
      double v = jmax * res[i] * res[i] / 4.0;
      if ((res[i] > 0) && (res[i] <= time / 2.0) && (v <= vmax) && (v > 0) &&
          (jmax * res[i] / 2.0 <= amax)) {
        *velocity = v;
        return 0;
      }
    }
  } else {
    double sc = (time - 2.0 * amax / jmax) * amax * amax / jmax;
    if (sc >= s) {
      double res[3] = {0, 0, 0};
      int res_n =
          quartic_equation((-jmax / 4.0), (time * jmax / 4.0), 0, -s, res);
      for (i = 0; i < res_n; i++) {
        double v = jmax * res[i] * res[i] / 4.0;
        if ((res[i] > 0) && (res[i] <= time / 2.0) && (v <= vmax) && (v > 0) &&
            (jmax * res[i] / 2.0 <= amax)) {
          *velocity = v;
          return 0;
        }
      }
    } else {
      double res[2] = {0, 0};
      int res_n =
          square_equation(-amax, (amax * time - 3.0 * amax * amax / jmax),
                          (amax * amax * time / jmax -
                           2.0 * amax * amax * amax / (jmax * jmax) - s),
                          res);
      for (i = 0; i < res_n; i++) {
        double v = (res[i] + amax / jmax) * amax;
        if ((res[i] >= 0) && ((res[i] + 2.0 * amax / jmax) <= time / 2.0) &&
            (v <= vmax) && (v > 0)) {
          *velocity = v;
          return 0;
        }
      }
    }
  }

  return -1;
}
}  // namespace rosc
