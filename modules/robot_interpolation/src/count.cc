/********************************************************************
 * @file count.cc
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-03-22
 *
 * @copyright Copyright (c) 2023 ROSC
 *
 ********************************************************************/
#include "robot_brain/count.h"

namespace rosc {

/***************************************
 * positive or negative function
 **************************************/

/********************************************************************
 * @brief 返回一个数的符号，正负
 *
 * @param x
 * @return double
 ********************************************************************/
double sign(double x) {
  if (x > 0) {
    return 1;
  } else if (x < 0) {
    return -1;
  } else {
    return 0;
  }
}

/********************************************************************
 * @brief 返回一个数的阶乘
 *
 * @param x
 * @return int
 ********************************************************************/
int factorial(int x) {
  int i;
  int y = 1;
  for (i = 1; i <= x; i++) {
    y = y * i;
  }
  return y;
}
/***************************************
 * function for distance from a[3] to b[3]
 **************************************/

/********************************************************************
 * @brief 计算两个向量的距离
 *
 * @param a
 * @param b
 * @return double
 ********************************************************************/
double dis3(double *a, double *b) {
  double c;

  c = sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) +
           (a[2] - b[2]) * (a[2] - b[2]));

  return c;
}
/*********************************************
 * function for cross multiplication y=x1��x2
 ********************************************/

/********************************************************************
 * @brief 计算两个向量的叉乘
 *
 * @param y
 * @param x1
 * @param x2
 ********************************************************************/
void cross3(double *y, double *x1, double *x2) {
  y[0] = x1[1] * x2[2] - x1[2] * x2[1];
  y[1] = x1[2] * x2[0] - x1[0] * x2[2];
  y[2] = x1[0] * x2[1] - x1[1] * x2[0];
}
/*********************************************
 * function for dot multiplication y=x1.x2
 ********************************************/

/********************************************************************
 * @brief 计算两个向量的点乘
 *
 * @param x1
 * @param x2
 * @return double
 ********************************************************************/
double dot3(double *x1, double *x2) {
  double y = x1[0] * x2[0] + x1[1] * x2[1] + x1[2] * x2[2];
  return y;
}

double degree_to_radian(double value) { return value * (R_PI / 180.0); }

double radian_to_degree(double value) { return value * (180.0 / R_PI); }

/*盛金公式求解ax^3+bx^2+cx+d=0的实根
 * return 实根数目
 * */

/********************************************************************
 * @brief 盛金公式求三次方程的实根数目
 *
 * @param a
 * @param b
 * @param c
 * @param d
 * @param res
 * @return int
 ********************************************************************/
int quartic_equation(double a, double b, double c, double d, double *res) {
  if (0 == a) {
    return square_equation(b, c, d, res);
  }

  // 参数归一
  // double _coef = sqrt(a * a + b * b + c * c + d * d);
  // a = a / _coef;
  // b = b / _coef;
  // c = c / _coef;
  // d = d / _coef;

  double A = b * b - 3.0 * a * c;
  double B = b * c - 9.0 * a * d;
  double C = c * c - 3.0 * b * d;
  double delt = B * B - 4.0 * A * C;

  if (delt > 0) {
    double Y1 = A * b + 1.5 * a * (-B + sqrt(delt));
    double Y2 = A * b + 1.5 * a * (-B - sqrt(delt));
    res[0] = (-b - (sign(Y1) * pow(fabs(Y1), 1.0 / 3.0) +
                    sign(Y2) * pow(fabs(Y2), 1.0 / 3.0))) /
             (3.0 * a);
    res[1] = res[0];
    res[2] = res[0];
    return 1;
  } else if (fabs(delt) == 0) {
    if (fabs(A) < 1e-15) {
      res[0] = res[1] = res[2] = -b / (3.0 * a);
      return 3;
    } else {
      double K = B / A;
      res[0] = -b / a + K;
      res[1] = res[2] = -K / 2.0;
      return 3;
    }
  } else {  // delt<0
    // 不可能存在，若过存在为计算误差，实际为delt==0,A==0,B==0的情况
    if (A <= 1e-15) {
      res[0] = res[1] = res[2] = -b / (3.0 * a);
      return 3;
    } else {
      double T = (2.0 * A * b - 3.0 * a * B) / (2.0 * A * sqrt(A));
      //考虑计算误差
      if (T > 1) {
        T = 1;
      }
      if (T < -1) {
        T = -1;
      }
      double Q = acos(T);
      double cQ3 = cos(Q / 3.0);
      double sQ3 = sin(Q / 3.0);
      res[0] = (-b - 2.0 * sqrt(A) * cQ3) / (3.0 * a);
      res[1] = (-b + sqrt(A) * (cQ3 + sqrt(3.0) * sQ3)) / (3.0 * a);
      res[2] = (-b + sqrt(A) * (cQ3 - sqrt(3.0) * sQ3)) / (3.0 * a);
      return 3;
    }
  }
  return 0;
}

/*ax^2+bx+c=0的实根
 * return 实根数目
 * */

/********************************************************************
 * @brief 计算二次方程的实根数目
 *
 * @param a
 * @param b
 * @param c
 * @param res
 * @return int
 ********************************************************************/
int square_equation(double a, double b, double c, double *res) {
  if (0 == a) {
    return first_equation(b, c, res);
  }
  // //参数归一
  // b = b / a;
  // c = c / a;
  // a = 1;
  double delt = b * b - 4.0 * a * c;
  if (delt >= 0) {
    res[0] = (-b + sqrt(delt)) / (2 * a);
    res[1] = (-b - sqrt(delt)) / (2 * a);
    return 2;
  } else {
    return 0;
  }
  return 0;
}

/*ax+b=0的实根
 * return 实根数目
 * */

/********************************************************************
 * @brief 计算一次方程的实根数目
 *
 * @param a
 * @param b
 * @param res
 * @return int
 ********************************************************************/
int first_equation(double a, double b, double *res) {
  if (0 == a) {
    return 0;
  }
  res[0] = -b / a;
  return 1;
}
}  // namespace rosc
