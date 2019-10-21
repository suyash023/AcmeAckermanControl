/**
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Ishan Patel, Nakul Patel, Suyash Yeotikar
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *  list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @file PidTest.cpp
 * @date 13th October 2019
 * @author Ishan Patel (driver), Navigator (Suyash Yeotikar), Design Keeper(Nakul Patel)
 * @brief Provides testing cases for the Pid Class.
 */

#include <eigen3/Eigen/Core>
#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include "Pid.hpp"

/**
 * @brief Runs and tests getter and setter of Proportional Gain
 * function for certain inputs and check the validity of inputs
 *
 */
TEST(SetKpTest, testSetKpValid) {
  Pid testObj = Pid();
  Eigen::Matrix<double, 2, 3> kp1;
  kp1 << 1, 2, 3, 4, 5, 6;
  bool b1 = testObj.setKp(kp1);
  EXPECT_EQ(true, b1);
  Eigen::Matrix<double, 2, 3> kp2;
  kp2 << -1, 2, -3, 4, 5, 6;
  bool b2 = testObj.setKp(kp2);
  EXPECT_EQ(false, b2);
}

/**
 * @brief Runs and tests getter and setter of Integral Gain
 * function for certain inputs and check the validity of inputs
 *
 */
TEST(SetKiTest, testSetKiValid) {
  Pid testObj = Pid();
  Eigen::Matrix<double, 2, 3> ki1;
  ki1 << 1, 2, 3, 4, 5, 6;
  bool b1 = testObj.setKi(ki1);
  EXPECT_EQ(true, b1);
  Eigen::Matrix<double, 2, 3> ki2;
  ki2 << -1, 2, -3, 4, 5, 6;
  bool b2 = testObj.setKi(ki2);
  EXPECT_EQ(false, b2);
}

/**
 * @brief Runs and tests getter and setter of Differential Gain
 * function for certain inputs and check the validity of inputs
 *
 */
TEST(SetKdTest, testSetKdValid) {
  Pid testObj = Pid();
  Eigen::Matrix<double, 2, 3> kd1;
  kd1 << 1, 2, 3, 4, 5, 6;
  bool b1 = testObj.setKd(kd1);
  EXPECT_EQ(true, b1);
  Eigen::Matrix<double, 2, 3> kd2;
  kd2 << -1, 2, -3, 4, 5, 6;
  bool b2 = testObj.setKd(kd2);
  EXPECT_EQ(false, b2);
}

/**
 * @brief Runs and tests getControllerOutput
 * function for certain inputs and check the validity of outputs
 *
 */
TEST(GetControllerOutputTest, testGetControllerOutputValid) {
  Pid testObj = Pid();
  Eigen::Matrix<double, 2, 3> kp1;
  kp1 << 1, 2, 3, 1, 1, 1;
  testObj.setKp(kp1);
  Eigen::Matrix<double, 2, 3> ki1;
  ki1 << 1, 2, 3, 1, 1, 1;
  testObj.setKi(ki1);
  Eigen::Matrix<double, 2, 3> kd1;
  kd1 << 1, 2, 3, 1, 1, 1;
  testObj.setKd(kd1);
  Eigen::Matrix<double, 2, 3> kp = testObj.getKp();
  Eigen::Matrix<double, 2, 3> ki = testObj.getKi();
  Eigen::Matrix<double, 2, 3> kd = testObj.getKd();
  Eigen::Vector3d targetState;
  targetState << 10, 10, 45;
  Eigen::Vector3d actualState;
  actualState << 5, 5, 30;
  Eigen::Vector2d output = testObj.getControllerOutput(targetState,
                                                       actualState);
  Eigen::Vector2d test(-180, -75);
  EXPECT_EQ(output[0], test[0]);
  EXPECT_EQ(output[1], test[1]);
}
