/********************************************************************
 * @file velocityprofile_polynomial.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-08-18
 *
 * @copyright Copyright (c) 2023 ROSC
 *
 ********************************************************************/
#ifndef MODULES_ROBOT_INTERPOLATION_INCLUDE_ROBOT_BRAIN_VELOCITYPROFILE_POLYNOMIAL_H_
#define MODULES_ROBOT_INTERPOLATION_INCLUDE_ROBOT_BRAIN_VELOCITYPROFILE_POLYNOMIAL_H_
#include "robot_brain/velocityprofile_base.h"
#include <math.h>
namespace rosc {
int Polynomial_project(Polynomial_in *pin, Polynomial_mid *pmid);

int Polynomial_computer(Polynomial_in *pin, Polynomial_mid *pmid,
                        Velocityprofile_out *vpout);

double Polynomial_computer_distance(Polynomial_in *pin, Polynomial_mid *pmid,
                                    double t);

double Polynomial_computer_time(Polynomial_in *pin, Polynomial_mid *pmid,
                                double s);
}  // namespace rosc

#endif  // MODULES_ROBOT_INTERPOLATION_INCLUDE_ROBOT_BRAIN_VELOCITYPROFILE_POLYNOMIAL_H_
