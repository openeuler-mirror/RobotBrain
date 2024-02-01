/********************************************************************
 * @file doubleS_test.cc
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-04-08
 *
 * @copyright Copyright (c) 2023 ROSC
 *
 ********************************************************************/
#include <gtest/gtest.h>
#include <glog/logging.h>
#include "robot_brain/velocityprofile.h"
#include "robot_brain/velocityprofile_doubleS.h"

// TEST(INTERPOLATION, double_s) {
//   rosc::VelocityProfile vp;
//   double s1 = 550;

//   double Vmax = 1460;
//   double Amax = 10000;
//   double Jmax = 100000;
//   double dt = 0.01;
//   double time = 0.0;
//   //
//   计算实际可以到到达的实际速度和实际时间，设置期望的时间和速度，如果期望时间比实际最小时间还小，那么按照实际最短时间来计算；如果最大速度达不到，那么以实际最大速度来计算
//   int err = rosc::DoubleSTimeToVelocity(s1, time, 10000, Amax, Jmax, &Vmax);
//   std::cout << Vmax << std::endl;
//   rosc::init_VelocityProfile_DoubleS(
//       &vp, dt, s1, 0, 0, 0, 0, 0, Vmax, Amax, Jmax, 0,
//       rosc::VelocityprofileStopState::_velocityprofile_stop, 1, 1);
//   rosc::project_VelocityProfile_DoubleS(&vp);
//   std::cout << vp.doubleS_mid.T << std::endl;
//   while (true) {
//     getVelocityProfile(&vp);

//     printf("时间：%.3f,位移：%.3f,速度：%.3f,加速度：%.3f,加加速度：%.3f\n",
//            vp.velocityprofile_out.t, vp.velocityprofile_out.s,
//            vp.velocityprofile_out.v, vp.velocityprofile_out.a,
//            vp.velocityprofile_out.j);
//     if (vp.velocityprofile_out.t < vp.doubleS_mid.T) {
//       continue;
//     }
//     if (0 == vp.velocityprofile_out.state) {
//       break;
//     }
//     // if (fabs(vp.velocityprofile_out.t - 0.300) < 1e-6) {
//     //   rosc::updateVelocityProfile_Vmax1(&vp, 2.0);
//     // }
//   }

//   std::cout << vp.doubleS_mid.T << std::endl;
// }
