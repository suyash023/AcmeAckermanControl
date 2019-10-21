/**
 BSD 3-Clause License

 Copyright (c) 2019, Ishan Patel, Nakul Patel, Suyash Yeotikar
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its
 contributors may be used to endorse or promote products derived from
 this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


 * @file AckermanKinematicModel.cpp
 *
 * @author Nakul Patel
 *
 * @brief implementation of AckermanKinematicModel class
 *
 * @version 1
 *
 * @date 2019-10-13
 *
 * @section DESCRIPTION
 *
 * This is the .cpp file that implements the
 * methods for AckermanKinematicModel class
 *
 */


#include "AckermanKinematicModel.hpp"
#include <math.h>
#include <vector>


double AckermanKinematicModel::getWheelBase() {
  return wheelBase;
}

double AckermanKinematicModel::getAxleWidth() {
  return axleWidth;
}

double AckermanKinematicModel::getSteeringAngle() {
  return steeringAngle;
}

double AckermanKinematicModel::getCarVelocity() {
  return carVelocity;
}

cv::Point3f AckermanKinematicModel::getCarState() {
  return carState;
}

bool AckermanKinematicModel::setWheelBase(double l) {
  bool checkWheelBase = false;
  if (l >= 0) {
    wheelBase = l;
    checkWheelBase = true;
  }
  return checkWheelBase;
}

bool AckermanKinematicModel::setAxleWidth(double w) {
  bool checkAxleWidth = false;
  if (w >= 0) {
    axleWidth = w;
    checkAxleWidth = true;
  }
  return checkAxleWidth;
}

bool AckermanKinematicModel::setCarVelocityAndSteeringAngle(
    Eigen::Vector2d controllerOutput) {
  if (controllerOutput(0) > 1000) {
    carVelocity = 1000;
  } else {
    carVelocity = controllerOutput(0);
  }
  steeringAngle = controllerOutput(1);
  bool checkSteeringAngle = checkAngleConstraints();
  bool setSteeringAngle = false;
  if (checkSteeringAngle == true) {
    setSteeringAngle = true;
  }
  return setSteeringAngle;
}

bool AckermanKinematicModel::setCarState(cv::Point3f state) {
  carState = state;
  cv::Point3f checkState = AckermanKinematicModel::getCarState();
  bool checkCarState = false;
  if ((checkState.x == state.x) && (checkState.y == state.y)
      && (checkState.z == state.z)) {
    checkCarState = true;
  }
  return checkCarState;
}

cv::Point3f AckermanKinematicModel::calcAckermanParameters() {
  double deltaTheta = (carVelocity / wheelBase)
      * std::tan(steeringAngle * M_PI / 180);
  double deltaX = carVelocity
      * std::cos((carState.z + (deltaTheta * dt)) * M_PI / 180) * dt;
  double deltaY = carVelocity
      * std::sin((carState.z + (deltaTheta * dt)) * M_PI / 180) * dt;
  carState.x = carState.x + deltaX;
  carState.y = carState.y + deltaY;
  carState.z = carState.z + deltaTheta * dt;
  return carState;
}

bool AckermanKinematicModel::checkAngleConstraints() {
  double innerSteerAngle = 180 / M_PI
      * std::atan(
          (2 * wheelBase * std::sin(steeringAngle * M_PI / 180))
              / (2 * wheelBase * std::cos(steeringAngle * M_PI / 180)
                  - axleWidth * std::sin(steeringAngle * M_PI / 180)));
  double outerSteerAngle = 180 / M_PI
      * std::atan(
          (2 * wheelBase * std::sin(steeringAngle * M_PI / 180))
              / (2 * wheelBase * std::cos(steeringAngle * M_PI / 180)
                  + axleWidth * std::sin(steeringAngle * M_PI / 180)));
  bool checkAngle = false;
  if ((innerSteerAngle <= 45) && (outerSteerAngle <= 45)) {
    checkAngle = true;
  }
  return checkAngle;
}


