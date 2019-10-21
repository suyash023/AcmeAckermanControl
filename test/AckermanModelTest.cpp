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
 * @file MapTest.cpp
 * @date 15th October 2019
 * @author Nakul Patel (driver), Ishan Patel (Navigator), Suyash Yeotikar(Design Keeper)
 * @brief AckermanKinematicModel module test source file containing unit tests.
 */

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include "AckermanKinematicModel.hpp"

/**
 * @brief Test if Wheel base length is valid
 * Set valid coordinates for wheel base length
 *
 */
TEST(SetWheelBaseTest, testSetWheelBaseValid) {
  AckermanKinematicModel testObj = AckermanKinematicModel();
  bool b1 = testObj.setWheelBase(2.5);
  EXPECT_EQ(true, b1);
  bool b2 = testObj.setWheelBase(-2.0);
  EXPECT_NE(true, b2);
}

/**
 * @brief Test if axle width is valid
 * Set valid coordinates for axle width
 *
 */
TEST(SetAxleWidthTest, testAxleWidthValid) {
  AckermanKinematicModel testObj = AckermanKinematicModel();
  bool b1 = testObj.setAxleWidth(1.5);
  EXPECT_EQ(true, b1);
  bool b2 = testObj.setAxleWidth(-1.0);
  EXPECT_NE(true, b2);
}

/**
 * @brief Test if car velocity and steering angle are valid
 * Set valid values for car velocity and steering angle
 *
 */
TEST(SetCarVelocitySteeringAngleTest, testSetCarVelocitySteeringAngleValid) {
  AckermanKinematicModel testObj = AckermanKinematicModel();
  Eigen::Vector2d velocityAngle1(25.0, 30.0);
  bool a1 = testObj.setWheelBase(2.0);
  bool a2 = testObj.setAxleWidth(1.0);
  bool b1 = testObj.setCarVelocityAndSteeringAngle(velocityAngle1);
  EXPECT_EQ(true, b1);
  // EXPECT_GT(0, carVel);
  // EXPECT_LT(steerAngle, 45);
  Eigen::Vector2d velocityAngle2(30.0, 45.0);
  bool b2 = testObj.setCarVelocityAndSteeringAngle(velocityAngle2);
  EXPECT_NE(true, b2);

}

/**
 * @brief Test if car state is valid
 * Set valid values for components of the car state
 *
 */
TEST(SetCarStateTest, testSetCarStateValid) {
  AckermanKinematicModel testObj = AckermanKinematicModel();
  cv::Point3f state1;
  state1.x = 50;
  state1.y = 50;
  state1.z = 30;
  bool b1 = testObj.setCarState(state1);
  cv::Point3f state = testObj.getCarState();
  EXPECT_EQ(true, b1);
  EXPECT_EQ(state.x, state1.x);
  EXPECT_EQ(state.y, state1.y);
  EXPECT_EQ(state.z, state1.z);
}

/**
 * @brief test for verifying the car state parameters
 *
 * This test verifies the x-coordinate, y-coordinate and
 * heading angle values of the car state
 *
 */
TEST(CalcAckermanParameterTest, testCalcAckermanParameter) {
  AckermanKinematicModel testObj = AckermanKinematicModel();
  bool b1 = testObj.setWheelBase(2.0);
  EXPECT_EQ(true, b1);
  Eigen::Vector2d vec(50.0, 40.0);
  bool b2 = testObj.setCarVelocityAndSteeringAngle(vec);
  EXPECT_EQ(true, b2);
  cv::Point3f initialState;
  initialState.x = 0.0;
  initialState.y = 0.0;
  initialState.z = 0.0;
  bool b3 = testObj.setCarState(initialState);
  EXPECT_EQ(true, b3);
  cv::Point3f updatedState = testObj.calcAckermanParameters();
  EXPECT_NEAR(0.5, updatedState.x, 0.05);
  EXPECT_NEAR(0.001831, updatedState.y, 0.05);
  EXPECT_NEAR(0.209774, updatedState.z, 0.05);
}

/**
 * @brief test for angle verification
 *
 * This test verifies if the calculated values of left and right
 * steering angles are valid
 *
 */
TEST(CheckAngleConstraintTest, testCheckAngleConstraint) {
  AckermanKinematicModel testObj = AckermanKinematicModel();
  bool b1 = testObj.setWheelBase(2.0);
  bool b2 = testObj.setAxleWidth(1.0);
  Eigen::Vector2d vec(25.0, 30.0);
  bool b3 = testObj.setCarVelocityAndSteeringAngle(vec);
  bool b = testObj.checkAngleConstraints();
  EXPECT_EQ(true, b);
}
