/********************************************************************
 * @file velocityprofile_base.cc
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-03-21
 *
 * @copyright Copyright (c) 2023 ROSC
 *
 ********************************************************************/
#include <robot_brain/velocityprofile_base.h>
#include <robot_brain/count.h>
#include <stdint.h>
#include <stdio.h>
#include <cmath>

namespace rosc {
/********************************************************************
 * @brief
 *
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
void DoubleS_in::Init(double dt, double S, double s0, double v_start,
                      double v_end, double acc_start, double acc_end,
                      double v_max, double acc_max, double jerk_max, double t0,
                      VelocityprofileStopState stopstate) {
  this->dt = dt;
  this->S = S;
  this->s0 = s0;
  this->v_start = v_start;
  this->v_end = v_end;
  this->acc_start = acc_start;
  this->acc_end = 0;  // ae;
  this->v_max = v_max;
  this->acc_max = acc_max;
  this->jerk_max = jerk_max;
  this->v_min = -v_max;
  this->acc_min = -acc_max;
  this->jerk_min = -jerk_max;
  this->stop_state = stopstate;

  this->_S = S;
  this->_Vmax = v_max;
  this->_Amax = acc_max;
  this->_Jmax = jerk_max;

  this->vs_1 = this->v_start;
  this->as_1 = this->acc_start;

  this->t0 = t0;
  if (this->t0 < 0) {
    this->t0 = 0;
  }
}

/********************************************************************
 * @brief 按照一个比例对速度进行缩放
 *
 * @param coef
 ********************************************************************/
void DoubleS_in::Scale(double coef) {
  this->v_start = this->v_start * coef;
  this->v_end = this->v_end * coef;
  this->acc_start = this->acc_start * coef * coef;
  this->acc_end = this->acc_end * coef * coef;
  this->v_max = this->v_max * coef;
  this->acc_max = this->acc_max * coef * coef;
  this->jerk_max = this->jerk_max * coef * coef * coef;
  this->v_min = -this->v_max * coef;
  this->acc_min = -this->acc_max * coef * coef;
  this->jerk_min = -this->jerk_max * coef * coef * coef;
}

void DoubleS_in::SetVelocityProfileStopState(VelocityprofileStopState state) {
  this->stop_state = state;
}

/********************************************************************
 * @brief 初始化DoubleS_mid结构体
 *
 ********************************************************************/
void DoubleS_mid::Init() {
  this->T = 0;
  this->Taa = 0;
  this->Tca = 0;
  this->Tda = 0;
  this->Tv = 0;
  this->Tad = 0;
  this->Tcd = 0;
  this->Tdd = 0;

  this->ac_Vmax = 0;
  this->ac_Amaxa = 0;
  this->ac_Amaxd = 0;
  this->ac_Jmax = 0;
  this->S_1 = 0;
  this->type = _velocity_profile_full;
}

/********************************************************************
 * @brief
 *
 * @return VelocityProfilePlanType
 ********************************************************************/
VelocityProfilePlanType DoubleS_mid::GetDoubleSType() {
  return this->profile_type;
}

/********************************************************************
 * @brief
 *
 * @param coef
 ********************************************************************/
void DoubleS_mid::Scale(double coef) {
  this->T = this->T * coef;
  this->Taa = this->Taa * coef;
  this->Tca = this->Tca * coef;
  this->Tda = this->Tda * coef;
  this->Tv = this->Tv * coef;
  this->Tad = this->Tad * coef;
  this->Tcd = this->Tcd * coef;
  this->Tdd = this->Tdd * coef;
  this->ac_Vmax = this->ac_Vmax * (1.0 / coef);
  this->ac_Amaxa = this->ac_Amaxa * (1.0 / coef) * (1.0 / coef);
  this->ac_Amaxd = this->ac_Amaxd * (1.0 / coef) * (1.0 / coef);
  this->ac_Jmax = this->ac_Jmax * (1.0 / coef) * (1.0 / coef) * (1.0 / coef);
}

/********************************************************************
 * @brief
 *
 * @param dt
 * @param S
 * @param s0
 * @param v_start
 * @param v_end
 * @param acc_start
 * @param acc_end
 * @param jerk_start
 * @param jerk_end
 * @param t0
 * @param t_end
 * @param n
 * @param stopstate
 ********************************************************************/
void Polynomial_in::Init(double dt, double S, double s0, double v_start,
                         double v_end, double acc_start, double acc_end,
                         double jerk_start, double jerk_end, double t0,
                         double t_end, int n,
                         VelocityprofileStopState stopstate) {
  t_end = static_cast<int>(t_end / dt) * dt;  // 整定规划时间

  if ((_velocityprofile_forcestop == stopstate) ||
      (_velocityprofile_suspend == stopstate)) {
    if ((t_end > VELOCITYPROFILE_STOP_TIME)) {
      t_end = VELOCITYPROFILE_STOP_TIME;
    }
    if (t_end < 1e-6) {
      S = 0;
      t_end = VELOCITYPROFILE_STOP_TIME;
    }
    double _S = v_start / 2.0 * t_end;
    if (S > _S) {
      S = _S;
    }
  }

  this->S = S;
  this->s0 = s0;
  this->v_start = v_start;
  this->v_end = v_end;
  this->acc_start = acc_start;
  this->acc_end = acc_end;
  this->jerk_start = jerk_start;
  this->jerk_end = jerk_end;
  this->t_end = t_end;
  this->dt = dt;
  this->n = n;
  this->t0 = t0;
  if (this->t0 < 0) {
    this->t0 = 0;
  }

  this->stop_state = stopstate;
}

/********************************************************************
 * @brief 设置多项式规划的位移
 *
 * @param s0
 * @param S
 ********************************************************************/
void Polynomial_in::SetS(double s0, double S) {
  this->s0 = s0;
  this->S = S;
}

/********************************************************************
 * @brief 设置多项式规划的S_end
 *
 * @param S
 ********************************************************************/
void Polynomial_in::SetS_end(double S) {
  this->s0 = this->s0 + S;
  this->S = S;
}

/********************************************************************
 * @brief
 *
 * @param state
 ********************************************************************/
void Polynomial_in::SetVelocityProfileStopState(
    VelocityprofileStopState state) {
  this->stop_state = state;
}

void Velocityprofile_out::Init() {
  this->t = 0;
  this->s = 0;
  this->v = 0;
  this->a = 0;
  this->j = 0;

  this->state = 0;
}

void Velocityprofile_out::SetT(double t) { this->t = t; }

void Velocityprofile_out::InitVelocityProfileS(DoubleS_in *ds_in) {
  this->t = ds_in->t0;
  this->s = ds_in->s0;
  this->v = ds_in->v_start;
  this->a = ds_in->acc_start;
  this->j = 0;

  this->state = 0;
}

void Velocityprofile_out::InitVelocityProfilePoly(Polynomial_in *poly_in) {
  this->t = poly_in->t0;
  this->s = poly_in->s0;
  this->v = poly_in->v_start;
  this->a = poly_in->acc_start;
  this->j = poly_in->jerk_start;

  this->state = 0;
}

void Velocityprofile_out::InitVelocityProfileOnlineS(OnlineDoubleS_in *osin) {
  this->t = osin->t0;
  this->s = osin->s0;
  this->v = osin->v_start;
  this->a = osin->acc_start;
  this->j = 0;

  this->state = 0;
}

int Velocityprofile_out::GetState() { return this->state; }

/********************************************************************
 * @brief 判断是否是
 *
 * @param value
 * @return int
 ********************************************************************/
int isVelocityprofileStopState(VelocityprofileStopState value) {
  if (VelocityprofileStopState::_velocityprofile_stop == value) {
    return 1;
  } else {
    return 0;
  }
}

/********************************************************************
 * @brief
 *
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
void OnlineDoubleS_in::Init(double dt, double S, double s0, double v_start,
                            double v_end, double acc_start, double acc_end,
                            double v_max, double acc_max, double jerk_max,
                            double t0, VelocityprofileStopState stopstate) {
  this->dt = dt;
  this->S = S;
  this->s0 = s0;
  this->v_start = v_start;
  this->v_end = v_end;
  this->acc_start = acc_start;
  this->acc_end = acc_end;
  this->v_max = v_max;
  this->acc_max = acc_max;
  this->jerk_max = jerk_max;
  this->v_min = -v_max;
  this->acc_min = -acc_max;
  this->jerk_min = -jerk_max;

  this->_S = S;
  this->_Vmax = v_max;
  this->_Amax = acc_max;
  this->_Jmax = jerk_max;

  this->stop_state = stopstate;
  this->start_moving_v = v_start;

  this->t0 = t0;
  if (this->t0 < 0) {
    this->t0 = 0;
  }
}

void OnlineDoubleS_mid::Init() {
  this->t_stop = -1;
  this->_ts = 0.0001;
  this->ts_times = 1;
  this->Tj2a = 0;
  this->Td = 0;
  this->Tj2b = 0;
}

void OnlineDoubleS_in::UpdateS(double S) { this->S = S; }

void OnlineDoubleS_in::UpdateVMax(double v_max) {
  this->v_max = v_max;
  this->v_min = -v_min;
}

void OnlineDoubleS_in::UpdateAccMax(double acc_max) {
  this->acc_max = acc_max;
  this->acc_min = -acc_max;
}

void OnlineDoubleS_in::UpdateJerkMax(double jerk_max) {
  this->jerk_max = jerk_max;
  this->jerk_min = -jerk_max;
}

void OnlineDoubleS_in::UpdateStartMovingV(double v) {
  this->start_moving_v = v;
}

/********************************************************************
 * @brief 获取双S型规划第一阶段的时间
 *
 * @param flag
 * @param vs
 * @param ve
 * @param vmax
 * @param amax
 * @param jmax
 * @param S
 * @param T
 * @param _S
 * @return int
 ********************************************************************/
static int __getDoubleSVelocityprofileTime01(int flag, double vs, double *ve,
                                             double *vmax, double amax,
                                             double jmax, double S, double *T,
                                             double *_S) {
  if (NULL != _S) {
    *_S = S;
  }
  if (S < 1e-6) {
    *T = 0;
    return 0;
  }
  double zone_S = 0;
  double tstart = 0;
  double tmid = 0;
  double tend = 0;
  double sstart = 0;
  double dvs = fabs(*vmax - vs);
  double aj = ((amax) * (amax)) / jmax;
  if (dvs > aj) {
    tstart = 2 * (amax) / jmax + (dvs - aj) / (amax);
  } else {
    tstart = 2 * sqrt(dvs / jmax);
  }
  sstart = (vs + 0.5 * (*vmax - vs)) * tstart;

  double the_S = 0;
  if (1 == flag) {
    the_S = S - zone_S;
  } else {
    the_S = S / 2.0;
  }

  if (sstart > the_S) {
    double a = 1.0 * static_cast<int>(std::signbit(*vmax - vs)) / (jmax * jmax);
    double b = 0;
    double c = 2.0 * vs / jmax;
    double d = -the_S;
    double res[3];
    int num = quartic_equation(a, b, c, d, res);
    int i = 0;
    for (i = 0; i < num; i++) {
      if ((res[i] > 0) && (res[i] <= (amax + 1e-10)) &&
          (((res[i]) * (res[i])) / jmax < fabs(*vmax - vs))) {
        *vmax = vs + sign(*vmax - vs) * ((res[i]) * (res[i])) / jmax;
        if (1 == flag) {
          *ve = *vmax;
        }
        break;
      }
    }
    if ((0 == num) || (i == num)) {
      a = 0.5 * sign(*vmax - vs) / (amax);
      b = 0.5 * sign(*vmax - vs) * (amax) / jmax + vs / (amax);
      c = vs * (amax) / jmax - the_S;
      num = square_equation(a, b, c, res);
      for (i = 0; i < num; i++) {
        if ((res[i] >= ((amax) * (amax) / jmax - 1e-10)) &&
            (res[i] <= fabs(*vmax - vs) + 1e-10)) {
          *vmax = vs + sign(*vmax - vs) * res[i];
          if (1 == flag) {
            *ve = *vmax;
          }
          break;
        }
      }
      if ((0 == num) || (i == num)) {
        return -2;
      } else {
        tstart = 2.0 * (amax) / jmax +
                 (fabs(*vmax - vs) - (amax) * (amax) / jmax) / (amax);
        sstart = the_S;
      }
    } else {
      tstart = 2.0 * (res[i]) / jmax;
      sstart = the_S;
    }
  }

  if (1 == flag) {
    tmid = (S - sstart) / (*vmax);
    tend = 0;
  } else {
    tmid = (S - 2.0 * sstart) / (*vmax);
    if (tmid < 0) {
      tmid = 0;
    }
    tend = tstart;
  }

  *T = tstart + tmid + tend;
  return 0;
}

/********************************************************************
 * @brief
 *
 * @param vs
 * @param amax
 * @param jmax
 * @param S
 * @param v
 * @return double
 ********************************************************************/
static double __func1(double vs, double amax, double jmax, double S, double v) {
  return 0.5 * v * (amax / jmax + v / amax) + (vs + v) * sqrt((v - vs) / jmax) -
         S;
}

static double __func2(double vs, double amax, double jmax, double S, double v) {
  return v * sqrt(v / jmax) + (vs + v) * sqrt((v - vs) / jmax) - S;
}

static double __func3(double vs, double amax, double jmax, double S, double v) {
  double tstart = 0;
  double tend = 0;
  double sstart = 0;
  double send = 0;
  double dvs = vs - v;
  double aj = ((amax) * (amax)) / jmax;
  if (dvs > aj) {
    tstart = 2 * (amax) / jmax + (dvs - aj) / (amax);
  } else {
    tstart = 2 * sqrt(dvs / jmax);
  }
  sstart = (vs + 0.5 * (v - vs)) * tstart;

  double dve = v;
  if (dve > aj) {
    tend = 2.0 * (amax) / jmax + (dve - aj) / (amax);
  } else {
    tend = 2.0 * sqrt(dve / jmax);
  }
  send = 0.5 * dve * tend;

  return sstart + send - S;
}

/********************************************************************
 * @brief 二分法
 *
 * @param vs
 * @param amax
 * @param jmax
 * @param S
 * @param _xmin
 * @param _xmax
 * @param func 函数指针
 * @return double
 ********************************************************************/
static double _get_func_dichotomy(double vs, double amax, double jmax, double S,
                                  double _xmin, double _xmax,
                                  double (*func)(double, double, double, double,
                                                 double)) {
  double e0 = 1e-3;
  double x = 0;
  double fx = 0;
  double ff = 0;

  double fmin = func(vs, amax, jmax, S, _xmin);
  double fmax = func(vs, amax, jmax, S, _xmax);
  if (fabs(fmin) <= e0) {
    return _xmin;
  } else if (fabs(fmin) <= e0) {
    return _xmax;
  } else {
    if (fmin * fmax > 0) {
      return -1;
    }
    while (1) {
      x = (_xmin + _xmax) / 2.0;
      fx = func(vs, amax, jmax, S, x);
      if (fabs(fx) < e0) {
        break;
      }
      ff = fmin * fx;
      if (ff <= 0) {
        _xmax = x;
      } else {
        _xmin = x;
        fmin = fx;
      }
    }
  }
  return x;
}

static int __getDoubleSVelocityprofileTime2(double vs, double *ve, double *vmax,
                                            double amax, double jmax, double S,
                                            double *T, double *_S) {
  if (NULL != _S) {
    *_S = S;
  }
  if (S < 1e-6) {
    *T = 0;
    return 0;
  }
  double tend = 0;
  double send = 0;
  double dve = vs;
  double aj = ((amax) * (amax)) / jmax;
  if (dve > aj) {
    tend = 2.0 * (amax) / jmax + (dve - aj) / (amax);
  } else {
    tend = 2.0 * sqrt(dve / jmax);
  }
  send = 0.5 * dve * tend;
  if (send > S) {
    return -1;
  } else {
    double tstart1 = 0;
    double tend1 = 0;
    double sstart1 = 0;
    double send1 = 0;
    double dvs = fabs(*vmax - vs);
    double aj = ((amax) * (amax)) / jmax;
    if (dvs > aj) {
      tstart1 = 2 * (amax) / jmax + (dvs - aj) / (amax);
    } else {
      tstart1 = 2 * sqrt(dvs / jmax);
    }
    sstart1 = (vs + 0.5 * (*vmax - vs)) * tstart1;

    double dve = *vmax;
    if (dve > aj) {
      tend1 = 2.0 * (amax) / jmax + (dve - aj) / (amax);
    } else {
      tend1 = 2.0 * sqrt(dve / jmax);
    }
    send1 = 0.5 * dve * tend1;
    if ((sstart1 + send1) <= S) {
      *T = tstart1 + tend1 + (S - (sstart1 + send1)) / (*vmax);
      return 0;
    } else {
      if (*vmax > vs) {
        double aaj = (amax * amax) / jmax;
        double aj = amax / jmax;
        double s1 = (2.0 * vs + aaj) * (aj);
        double s2 = 0.5 * (vs + aaj) * (2.0 * aj + vs / amax);
        // 上升有匀加，下降有匀加，(vs+aaj)<v<*vmax
        if ((s1 + s2) < S) {
          double a = 2.0 / amax;
          double b = 2.0 * amax / jmax;
          double c = vs * (amax / jmax - vs / amax) - 2.0 * S;
          double res[2];
          int num = square_equation(a, b, c, res);
          int i = 0;
          for (i = 0; i < num; i++) {
            if ((res[i] > (vs + aaj)) && (res[i] <= *vmax)) {
              *vmax = res[i];
              *T = 2.0 * aj + (res[i] - vs - aaj) / amax + 2.0 * aj +
                   (res[i] - aaj) / amax;
              return 0;
            }
          }
        } else {  // 上升无匀加
          double vm = aaj;
          if (vm <= vs) {  // 下降有匀加,vs<v<=(vs+aaj)
            *vmax =
                _get_func_dichotomy(vs, amax, jmax, S, vs, vs + aaj, __func1);
            if (*vmax < 0) {
              return -2;
            }
            *T = 2.0 * sqrt((*vmax - vs) / jmax) +
                 ((amax / jmax) + (*vmax / amax));
            return 0;
          } else {
            double xs2 = vm * aj;
            double xs1 = (vs + vm) * sqrt(jmax * (vm - vs)) / jmax;
            if ((xs1 + xs2) < S) {  // 下降有匀加,aaj<v<=(vs+aaj)
              *vmax = _get_func_dichotomy(vs, amax, jmax, S, aaj, vs + aaj,
                                          __func1);
              if (*vmax < 0) {
                return -3;
              }
              *T = 2.0 * sqrt((*vmax - vs) / jmax) +
                   ((amax / jmax) + (*vmax / amax));
              return 0;
            } else {  // 下降无匀加,vs<=v<=aaj
              *vmax = _get_func_dichotomy(vs, amax, jmax, S, vs, aaj, __func2);
              if (*vmax < 0) {
                return -4;
              }
              *T = 2.0 * sqrt((*vmax - vs) / jmax) + (2.0 * sqrt(*vmax / jmax));
              return 0;
            }
          }
        }
      } else {
        // *vmax < v < vs;
        *vmax = _get_func_dichotomy(vs, amax, jmax, S, *vmax, vs, __func3);
        if (*vmax < 0) {
          return -5;
        }
        double tstart = 0;
        double tend = 0;
        double dvs = vs - *vmax;
        double aj = ((amax) * (amax)) / jmax;
        if (dvs > aj) {
          tstart = 2 * (amax) / jmax + (dvs - aj) / (amax);
        } else {
          tstart = 2 * sqrt(dvs / jmax);
        }

        double dve = *vmax;
        if (dve > aj) {
          tend = 2.0 * (amax) / jmax + (dve - aj) / (amax);
        } else {
          tend = 2.0 * sqrt(dve / jmax);
        }
        *T = tstart + tend;
        return 0;
      }
    }
  }

  return -1;
}

static int __getDoubleSVelocityprofileTime3(double vs, double *ve, double *vmax,
                                            double amax, double jmax, double S,
                                            double *T, double *_S) {
  // if (NULL != _S) {
  //   *_S = S;
  // }
  // if (S < 1e-6) {
  //   *T = 0;
  //   return 0;
  // }
  // double tstart = 0;
  // double tmid = 0;
  // double tend = 0;
  // double send = 0;

  // double dve = vs;
  // double aj = ((amax) * (amax)) / jmax;
  // if (dve > aj) {
  //   tend = 2 * (amax) / jmax + (dve - aj) / (amax);
  // } else {
  //   tend = 2 * sqrt(dve / jmax);
  // }
  // send = 0.5 * dve * tend;
  // if (send > S) {
  //   *T = -1;
  //   return -1;
  // } else {
  //   if (NULL != _S) {
  //     *_S = send;
  //   }
  //   tstart = 0;
  //   tmid = 0;
  //   *T = tstart + tmid + tend;
  // }
  *T = 0;
  return 0;
}

static int _DoubleSVelocityprofileType(double vs, double *ve, double *vmax,
                                       int stopstate) {
  int flag = 0;
  if ((_velocityprofile_forcestop == stopstate) ||
      (_velocityprofile_suspend == stopstate)) {
    flag = 3;
  } else {
    if ((*ve < 1e-10) && (vs < 1e-10)) {  // 非轨迹衔接
      flag = 0;
    } else if ((fabs(*vmax - *ve) < 1e-10) && (*ve >= 1e-10)) {  // 轨迹衔接
      *ve = *vmax;
      flag = 1;
    } else if (*ve < 1e-10) {  // 线段终点停止
      *ve = 0;
      flag = 2;
    } else {
      flag = -1;
    }
  }
  return flag;
}

/*计算规划周期，要求起始加速度及终止加速度为0,终止速度为0或等于最大速度
 * 非停止运动：如果给定的规划数据不满足规划要求（最大速度在一半位移之前达到），
 * 则按照规划要求，重新指定规划数据。
 * 停止运动：维持匀速再减速
 * 提前停止运动：直接减速，反馈停止位移动
 * */
/********************************************************************
 * @brief Get the Double S Velocityprofile Time object
 *
 * @param vs
 * @param ve
 * @param vmax
 * @param amax
 * @param jmax
 * @param S
 * @param T
 * @param _S
 * @param stopstate
 * @return int
 ********************************************************************/
int getDoubleSVelocityprofileTime(double vs, double *ve, double *vmax,
                                  double amax, double jmax, double S, double *T,
                                  double *_S, int stopstate) {
  int err = 0;
  switch (_DoubleSVelocityprofileType(vs, ve, vmax, stopstate)) {
  case 0:
    err = __getDoubleSVelocityprofileTime01(0, vs, ve, vmax, amax, jmax, S, T,
                                            _S);
    break;
  case 1:
    err = __getDoubleSVelocityprofileTime01(1, vs, ve, vmax, amax, jmax, S, T,
                                            _S);
    break;
  case 2:
    err = __getDoubleSVelocityprofileTime2(vs, ve, vmax, amax, jmax, S, T, _S);
    break;
  case 3:
    err = __getDoubleSVelocityprofileTime3(vs, ve, vmax, amax, jmax, S, T, _S);
    break;
  default:
    err = -1;
    break;
  }
  return err;
}

static double _get_function(int sign_flag, double vs, double ve, double v,
                            double amax, double jmax, double S, double _T) {
  double tstart = 0;
  double tmid = 0;
  double tend = 0;
  double sstart = 0;
  double send = 0;
  double dvs = fabs(v - vs);
  double aj = ((amax) * (amax)) / jmax;
  if (dvs > aj) {
    tstart = 2 * (amax) / jmax + (dvs - aj) / (amax);
  } else {
    tstart = 2 * sqrt(dvs / jmax);
  }
  sstart = (vs + 0.5 * sign_flag * dvs) * tstart;

  double dve = fabs(v - ve);
  if (dve > aj) {
    tend = 2 * (amax) / jmax + (dve - aj) / (amax);
  } else {
    tend = 2 * sqrt(dve / jmax);
  }
  send = (0.5 * dve) * tend;

  if (v < 1e-10) {
    tmid = (S - sstart - send) / (1e-10);
  } else {
    tmid = (S - sstart - send) / v;
  }

  return tstart + tmid + tend - _T;
}

static double _get_vamx_dichotomy(int sign_flag, double vs, double *ve,
                                  double amax, double jmax, double S, double _T,
                                  double _xmin, double _xmax) {
  double e0 = 1e-3;
  double x = 0;

  double fx = 0;
  double ff = 0;
  double _ve = 0;
  if (NULL != ve) {
    _ve = *ve;
  }
  if (NULL == ve) {
    _ve = _xmin;
  }
  double fmin = _get_function(sign_flag, vs, _ve, _xmin, amax, jmax, S, _T);
  if (NULL == ve) {
    _ve = _xmax;
  }
  double fmax = _get_function(sign_flag, vs, _ve, _xmax, amax, jmax, S, _T);
  if (fabs(fmin) <= e0) {
    return _xmin;
  } else if (fabs(fmax) <= e0) {
    return _xmax;
  } else {
    if (fmin * fmax > 0) {
      return -1;
    }
    while (1) {
      x = (_xmin + _xmax) / 2.0;
      if (NULL == ve) {
        _ve = x;
      }
      fx = _get_function(sign_flag, vs, _ve, x, amax, jmax, S, _T);
      if (fabs(fx) < e0) {
        break;
      }
      ff = fmin * fx;
      if (ff <= 0) {
        _xmax = x;
      } else {
        _xmin = x;
        fmin = fx;
      }
    }
  }
  return x;
}

static int __syncDoubleSVelocityprofileTime0(double vs, double *ve,
                                             double *vmax, double amax,
                                             double jmax, double T, double S,
                                             double _T, double *_S) {
  if (fabs(T - _T) < 1e-3) {
    return 0;
  }
  if ((T < 1e-3) && (S < 1e-6)) {
    return 0;
  }
  if (NULL != _S) {
    *_S = S;
  }

  S = 0.5 * S;
  _T = 0.5 * _T;

  double a = -1.0 / (jmax * jmax);
  double b = _T / jmax;
  double c = 0;
  double d = -S;
  double res[3];
  int num = quartic_equation(a, b, c, d, res);
  int i = 0;
  for (i = 0; i < num; i++) {
    if (((res[i] > 0) && (res[i] <= (amax + 1e-10))) &&
        (_T >= 2.0 * res[i] / jmax) && ((res[i] * res[i] / jmax) <= *vmax)) {
      *vmax = res[i] * res[i] / jmax;
      return 0;
    }
  }

  a = -0.5 * amax;
  b = (amax * _T - 3.0 * amax * amax / (2.0 * jmax));
  c = (amax * amax / jmax) * _T - amax * amax * amax / (jmax * jmax) - S;
  num = square_equation(a, b, c, res);
  i = 0;
  for (i = 0; i < num; i++) {
    if (((res[i] >= 0) && (res[i] <= (_T - 2.0 * amax / jmax))) &&
        (((res[i] + amax / jmax) * amax) <= *vmax)) {
      *vmax = (res[i] + amax / jmax) * amax;
      return 0;
    }
  }
  return -1;
}

static int __syncDoubleSVelocityprofileTime1(double vs, double *ve,
                                             double *vmax, double amax,
                                             double jmax, double T, double S,
                                             double _T, double *_S) {
  if (fabs(T - _T) < 1e-3) {
    return 0;
  }
  if ((T < 1e-3) && (S < 1e-6)) {
    return 0;
  }
  if (NULL != _S) {
    *_S = S;
  }

  double dS = fabs(vs * _T - S);
  if (dS < 1e-6) {
    *vmax = vs;
    *ve = *vmax;
    return 0;
  }

  int sign_flag = 1;
  if (vs * _T > S) {
    sign_flag = -1;
  } else {
    sign_flag = 1;
  }

  double zone_S = 0;

  double a = -sign_flag / (jmax * jmax);
  double b = sign_flag * _T / jmax;
  double c = 0;
  double d = vs * _T - S;
  double res[3];
  int num = quartic_equation(a, b, c, d, res);
  int i = 0;
  // printf("%f,%f,%f,%f,%f,%f,%f,%f\n", vs, *ve, *vmax, amax, jmax, T, S, _T);
  for (i = 0; i < num; i++) {
    // printf("%d:%f,%f,%f,%f,%f,%f,%f,%f,%f\n", i, res[i], amax,
    //        (2.0 * vs + sign_flag * (res[i] * res[i]) / jmax) * res[i] / jmax,
    //        S, _T, T, 2.0 * res[i] / jmax, vs + sign_flag * res[i] * res[i] /
    //        jmax, *vmax);
    if (((res[i] > 0) && (res[i] <= (amax + 1e-10))) &&
        (((2.0 * vs + sign_flag * (res[i] * res[i]) / jmax) * res[i] / jmax) <=
         (S - zone_S)) &&
        (_T >= 2.0 * res[i] / jmax) &&
        ((vs + sign_flag * res[i] * res[i] / jmax) <= *vmax)) {
      *vmax = vs + sign_flag * res[i] * res[i] / jmax;
      *ve = *vmax;
      return 0;
    }
  }

  a = -0.5 * sign_flag * amax;
  b = sign_flag * (amax * _T - 3.0 * amax * amax / (2.0 * jmax));
  c = (vs + sign_flag * amax * amax / jmax) * _T -
      sign_flag * amax * amax * amax / (jmax * jmax) - S;
  num = square_equation(a, b, c, res);
  for (i = 0; i < num; i++) {
    // printf("%d:+:%f,%f,%f,%f\n", i, res[i], (_T - 2.0 * amax / jmax),
    //        vs + sign_flag * (res[i] + amax / jmax) * amax, *vmax);
    if (((res[i] >= 0) && (res[i] <= (_T - 2.0 * amax / jmax))) &&
        ((vs + sign_flag * (res[i] + amax / jmax) * amax) <= *vmax) &&
        (((vs + 0.5 * sign_flag * (res[i] + amax / jmax) * amax) *
          (2.0 * amax / jmax + res[i])) <= (S - zone_S))) {
      *vmax = vs + sign_flag * (res[i] + amax / jmax) * amax;
      *ve = *vmax;
      return 0;
    }
  }

  return -1;
}

static int __syncDoubleSVelocityprofileTime2(double vs, double *ve,
                                             double *vmax, double amax,
                                             double jmax, double T, double S,
                                             double _T, double *_S) {
  if (fabs(T - _T) < 1e-3) {
    return 0;
  }
  if ((T < 1e-3) && (S < 1e-6)) {
    return 0;
  }
  if (NULL != _S) {
    *_S = S;
  }

  double send = 0;
  int sign_flag = 0;
  double tend = 0;
  double dve = vs;
  double aj = ((amax) * (amax)) / jmax;
  if (dve > aj) {
    tend = 2.0 * (amax) / jmax + (dve - aj) / (amax);
  } else {
    tend = 2.0 * sqrt(dve / jmax);
  }
  send = 0.5 * dve * tend;
  if ((send > S) || (tend > _T)) {
    return -1;
  }

  double v_max = 0;
  double v_min = 0;
  if ((send + vs * (_T - tend)) > S) {
    sign_flag = -1;
    v_min = 0;
    v_max = (vs <= *vmax) ? vs : *vmax;
  } else {
    sign_flag = 1;
    v_min = vs;
    v_max = *vmax;
  }
  if (fabs((send + vs * (_T - tend)) - S) < 1e-10) {
    *vmax = vs;
    return 0;
  }
  *vmax =
      _get_vamx_dichotomy(sign_flag, vs, ve, amax, jmax, S, _T, v_min, v_max);
  if (*vmax < 0) {
    return -1;
  }
  return 0;
}

static int __syncDoubleSVelocityprofileTime3(double vs, double *ve,
                                             double *vmax, double amax,
                                             double jmax, double T, double S,
                                             double _T, double *_S) {
  /*
  if ((T<1e-3)&&(S<1e-6))
  {
          return 0;
  }
  double send=0;
  double tend=0;
  double dve=vs;
  double aj=((amax)*(amax))/jmax;
  if (dve>aj)
  {
          tend=2.0*(amax)/jmax+(dve-aj)/(amax);
  }
  else
  {
          tend=2.0*sqrt(dve/jmax);
  }
  send=0.5*dve*tend;
  if ((send>S)||(tend>_T))
  {
          return -1;
  }

  if (NULL!=_S)
  {
          *_S=S;
  }

  if ((0.5*vs*_T)<=S)
  {
          *vmax=vs/2.0;
          if (NULL!=_S)
          {
                  *_S=0.5*vs*_T;
          }
  }
*/
  return 0;
}

/********************************************************************
 * @brief 同步双s型规划速度配置
 *
 * @param vs
 * @param ve
 * @param vmax
 * @param amax
 * @param jmax
 * @param T
 * @param S
 * @param _T
 * @param _S
 * @param stopstate
 * @return int
 ********************************************************************/
int syncDoubleSVelocityprofile(double vs, double *ve, double *vmax, double amax,
                               double jmax, double T, double S, double _T,
                               double *_S, int stopstate) {
  int err = 0;
  switch (_DoubleSVelocityprofileType(vs, ve, vmax, stopstate)) {
  case 0:
    err = __syncDoubleSVelocityprofileTime0(vs, ve, vmax, amax, jmax, T, S, _T,
                                            _S);
    break;
  case 1:
    err = __syncDoubleSVelocityprofileTime1(vs, ve, vmax, amax, jmax, T, S, _T,
                                            _S);
    break;
  case 2:
    err = __syncDoubleSVelocityprofileTime2(vs, ve, vmax, amax, jmax, T, S, _T,
                                            _S);
    break;
  case 3:
    err = __syncDoubleSVelocityprofileTime3(vs, ve, vmax, amax, jmax, T, S, _T,
                                            _S);
    break;
  default:
    err = -1;
    break;
  }
  return err;
}

double intVelocityprofileTime(double in, double ts) {
  if (in < 1e-6) {
    return 0;
  }
  uint64_t tmp = in / ts + 1;
  return tmp * ts;
}
}  // namespace rosc
