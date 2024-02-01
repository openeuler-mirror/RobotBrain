/********************************************************************
 * @file velocityprofile_polynomial.cc
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-08-18
 *
 * @copyright Copyright (c) 2023 ROSC
 *
 ********************************************************************/
#include "robot_brain/velocityprofile_polynomial.h"

namespace rosc {
/********************************************************************
 * @brief 输入初始变量，计算多项式规划的中间变量
 *
 * @param pin
 * @param pmid
 * @return int
 ********************************************************************/
int Polynomial_project(Polynomial_in *pin, Polynomial_mid *pmid) {
  int ret = -1;
  double ss = pin->s0;
  double se = pin->s0 + pin->S;
  double vs = pin->v_start;
  double ve = pin->v_end;
  double as = pin->acc_start;
  double ae = pin->acc_end;
  double js = pin->jerk_start;
  double je = pin->jerk_end;
  double tend = pin->t_end;
  if (tend < 1e-6) {
    return -1;
  }
  if (pin->S < 1e-6) {
    pin->stop_state = _velocityprofile_constant_stop;
  }
  if (1 == pin->n) {
    pmid->a[0] = ss;
    pmid->a[1] = pin->S / tend;
    ret = 0;
  } else if (5 == pin->n) {
    pmid->a[0] = ss;
    pmid->a[1] = vs;
    pmid->a[2] = as / 2.0;
    pmid->a[3] = (20 * se - 20 * ss - (8 * ve + 12 * vs) * tend -
                  (3 * as - ae) * tend * tend) /
                 (2.0 * tend * tend * tend);
    pmid->a[4] = (30 * ss - 30 * se + (14 * ve + 16 * vs) * tend +
                  (3 * as - 2 * ae) * tend * tend) /
                 (2.0 * tend * tend * tend * tend);
    pmid->a[5] = (12 * se - 12 * ss - (6 * ve + 6 * vs) * tend -
                  (as - ae) * tend * tend) /
                 (2.0 * tend * tend * tend * tend * tend);
    ret = 0;
  } else if (7 == pin->n) {
    double tf = tend;
    double tf2 = tend * tend;
    double tf3 = tend * tend * tend;
    double tf4 = tend * tend * tend * tend;
    double tf5 = tend * tend * tend * tend * tend;
    double tf6 = tend * tend * tend * tend * tend * tend;
    double tf7 = tend * tend * tend * tend * tend * tend * tend;

    double b1 = se - js / 6.0 * tf3 - as / 2.0 * tf2 - vs * tf - ss;
    double b2 = ve - js / 2.0 * tf2 - as * tf - vs;
    double b3 = ae - js * tf - as;
    double b4 = je - js;

    pmid->a[0] = ss;
    pmid->a[1] = vs;
    pmid->a[2] = as / 2.0;
    pmid->a[3] = js / 6.0;
    pmid->a[4] = (35 * b1) / tf4 - (15 * b2) / tf3 + (5 * b3) / (2 * tf2) -
                 b4 / (6 * tf);
    pmid->a[5] =
        (39 * b2) / tf4 - (84 * b1) / tf5 - (7 * b3) / tf3 + b4 / (2 * tf2);
    pmid->a[6] = (70 * b1) / tf6 - (34 * b2) / tf5 + (13 * b3) / (2 * tf4) -
                 b4 / (2 * tf3);
    pmid->a[7] =
        (10 * b2) / tf6 - (20 * b1) / tf7 - (2 * b3) / tf5 + b4 / (6 * tf4);
    ret = 0;
  }

  return ret;
}

/********************************************************************
 * @brief
 *
 * @param pin
 * @param pmid
 * @param vpout
 * @return int
 ********************************************************************/
int Polynomial_computer(Polynomial_in *pin, Polynomial_mid *pmid,
                        Velocityprofile_out *vpout) {
  double tt = vpout->t + pin->dt;
  double t_ = tt - pin->t0;
  double t = 0;
  double *a = pmid->a;

  if (t_ < 0) {
    t = 0;
  } else if (t_ >= (pin->t_end - pin->dt / 2.0)) {
    t = pin->t_end;
  } else {
    t = t_;
  }

  if ((pin->S < 1e-6) || (pin->t_end < 1e-6)) {
    vpout->s = pin->s0;
    vpout->v = 0;
    vpout->a = 0;
    vpout->j = 0;
    vpout->t = tt;
    vpout->state = pin->stop_state;
    return pin->stop_state;
  }

  double t2 = t * t;
  double t3 = t * t * t;
  double t4 = t * t * t * t;
  double t5 = t * t * t * t * t;
  double t6 = t * t * t * t * t * t;
  double t7 = t * t * t * t * t * t * t;

  if (1 == pin->n) {
    vpout->s = a[0] + a[1] * t;
    vpout->v = a[1];
    vpout->a = 0;
    vpout->j = 0;
    vpout->t = tt;
  } else if (5 == pin->n) {
    vpout->s = a[0] + a[1] * t + a[2] * t2 + a[3] * t3 + a[4] * t4 + a[5] * t5;
    vpout->v =
        a[1] + 2 * a[2] * t + 3 * a[3] * t2 + 4 * a[4] * t3 + 5 * a[5] * t4;
    vpout->a = 2 * a[2] + 6 * a[3] * t + 12 * a[4] * t2 + 20 * a[5] * t3;
    vpout->j = 6 * a[3] + 24 * a[4] * t + 60 * a[5] * t2;
    vpout->t = tt;
  } else if (7 == pin->n) {
    vpout->s = a[7] * t7 + a[6] * t6 + a[5] * t5 + a[4] * t4 + a[3] * t3 +
               a[2] * t2 + a[1] * t + a[0];
    vpout->v = 7 * a[7] * t6 + 6 * a[6] * t5 + 5 * a[5] * t4 + 4 * a[4] * t3 +
               3 * a[3] * t2 + 2 * a[2] * t + a[1];
    vpout->a = 42 * a[7] * t5 + 30 * a[6] * t4 + 20 * a[5] * t3 +
               12 * a[4] * t2 + 6 * a[3] * t + 2 * a[2];
    vpout->j = 210 * a[7] * t4 + 120 * a[6] * t3 + 60 * a[5] * t2 +
               24 * a[4] * t + 6 * a[3];
    vpout->t = tt;
  } else {
    vpout->s = 0;
    vpout->v = 0;
    vpout->a = 0;
    vpout->j = 0;
    vpout->t = tt;
  }

  if (t_ >= (pin->t_end - pin->dt / 2.0)) {
    // vpout->s =pin->S+pin->s0;
    if (1 != pin->n) {
      vpout->v = pin->v_end;
      vpout->a = pin->acc_end;
    }
    vpout->state = pin->stop_state;
    return pin->stop_state;
  }

  vpout->state = _velocityprofile_run;
  return _velocityprofile_run;
}

double Polynomial_computer_distance(Polynomial_in *pin, Polynomial_mid *pmid,
                                    double t) {
  if ((pin->S < 1e-6) || (pin->t_end < 1e-6)) {
    return pin->s0;
  }
  double *a = pmid->a;

  t = t - pin->t0;
  if (t < 0) {
    t = 0;
  } else if (t > pin->t_end) {
    t = pin->t_end;
  }
  double s = 0;
  double t2 = t * t;
  double t3 = t * t * t;
  double t4 = t * t * t * t;
  double t5 = t * t * t * t * t;
  double t6 = t * t * t * t * t * t;
  double t7 = t * t * t * t * t * t * t;

  if (1 == pin->n) {
    s = a[0] + a[1] * t;
  } else if (5 == pin->n) {
    s = a[0] + a[1] * t + a[2] * t2 + a[3] * t3 + a[4] * t4 + a[5] * t5;
  } else if (7 == pin->n) {
    s = a[7] * t7 + a[6] * t6 + a[5] * t5 + a[4] * t4 + a[3] * t3 + a[2] * t2 +
        a[1] * t + a[0];
  } else {
    s = 0;
  }
  return s;
}

/********************************************************************
 * @brief
 *
 * @param pin
 * @param pmid
 * @param s
 * @return double
 ********************************************************************/
double Polynomial_computer_time(Polynomial_in *pin, Polynomial_mid *pmid,
                                double s) {
  if ((pin->S < 1e-6) || (pin->t_end < 1e-6)) {
    return pin->t0;
  }

  double tmin = pin->t0;
  double tmax = pin->t_end + pin->t0;
  double t = 0;

  if (s <= pin->s0) {
    return tmin;
  }
  if (s >= Polynomial_computer_distance(pin, pmid, tmax)) {
    return tmax;
  }

  double fmin = 0;
  double fx = 0;
  double ff = 0;
  double e = tmax - tmin;
  while (e > pin->dt) {
    t = (tmax + tmin) / 2.0;
    fmin = Polynomial_computer_distance(pin, pmid, tmin);
    fx = Polynomial_computer_distance(pin, pmid, t);
    ff = fmin * fx;
    if (ff < 0) {
      tmax = t;
    } else if (ff > 0) {
      tmin = t;
    } else {
      tmin = t;
      tmax = t;
    }
    e = tmax - tmin;
  }
  t = (tmax + tmin) / 2.0;
  int tmp = static_cast<int>((t - pin->t0) / pin->dt + 0.5);
  return tmp * pin->dt + pin->t0;
}
}  // namespace rosc
