/********************************************************************
 * @file count.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-03-22
 *
 * @copyright Copyright (c) 2023 ROSC
 *
 ********************************************************************/
#ifndef MODULES_ROBOT_INTERPOLATION_INCLUDE_ROBOT_BRAIN_COUNT_H_
#define MODULES_ROBOT_INTERPOLATION_INCLUDE_ROBOT_BRAIN_COUNT_H_
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
namespace rosc {

#define min(X, Y) ((X) < (Y) ? (X) : (Y))
#define max(X, Y) ((X) > (Y) ? (X) : (Y))
#ifndef R_PI
#define R_PI 3.1415926535898
#endif

double dot3(double *x1, double *x2);
void cross3(double *y, double *x1, double *x2);
double dis3(double *a, double *b);
double sign(double x);
int factorial(int x);

double degree_to_radian(double value);

double radian_to_degree(double value);

/*盛金公式求解ax^3+bx^2+cx+d=0的实根
 * return 实根数目
 * */
int quartic_equation(double a, double b, double c, double d, double *res);

/*ax^2+bx+c=0的实根
 * return 实根数目
 * */
int square_equation(double a, double b, double c, double *res);

/*ax+b=0的实根
 * return 实根数目
 * */
int first_equation(double a, double b, double *res);

}  // namespace rosc
#endif  // MODULES_ROBOT_INTERPOLATION_INCLUDE_ROBOT_BRAIN_COUNT_H_
