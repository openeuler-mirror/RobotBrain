// /**
//  * @file kdl_test.cc
//  * @author your name (you@domain.com)
//  * @brief
//  * @version 0.1
//  * @date 2021-10-12
//  *
//  * @copyright Copyright (c) 2021 ROSC
//  *
//  */

#include "robot_brain/config.h"
#include "robot_brain/core/robot_model.h"
#include "robot_brain/robot_exception/robot_status.h"
#include "robot_brain/robot_planning/double_hump.h"
#include <bits/stdint-uintn.h>
#include <cstdio>
#include <gtest/gtest.h>
#include <glog/logging.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/joint.hpp>
#include <kdl/utilities/utility.h>
#include <math.h>
#include <robot_brain/robot_planning.hpp>

#include <iostream>
#include <memory>
#include <ostream>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <string>

TEST(VelocityProfile, s_vel_interpolation) {
  rosc::DoubleHump dh;
  dh.DoubleHumpVelocityProfile_S(0, 5000, 10000, 250000, 950000, 2500, 1500,
                                 2500);
  double dt = 0.001;
  // for (double t = 0.0; t < dh.Duration() + dt; t += dt) {
  //   std::cout << dh.Pos(t + dt) - dh.Pos(t) << std::endl;
  // }
  // LOG(INFO) << "Duration: " << dh.Duration();
  rosc::RobotModelCobot *robot = new rosc::RobotModelCobot();
  KDL::JntArray joint(6);
  KDL::SetToZero(joint);
  KDL::Frame cart_pos;
  robot->ForwordKinematicsPos(joint, &cart_pos, 3);
  double r, p, y;
  cart_pos.M.GetRPY(r, p, y);
  std::cout << cart_pos.p.x() << " " << cart_pos.p.y() << " " << cart_pos.p.z()
            << " " << r << " " << p << " " << y << std::endl;
  delete robot;
}
