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


 * @file AckermanKinematicModel.hpp
 *
 * @author Nakul Patel
 *
 * @brief declaration of AckermanKinematicModel class
 *
 * @version 1
 *
 * @date 2019-10-13
 *
 * @section DESCRIPTION
 *
 * This is the header file that will be used for the declaration
 * of class attributes and methods for a Ackermann Kinematic Model
 *
 */

#ifndef INCLUDE_ACKERMANKINEMATICMODEL_HPP_
#define INCLUDE_ACKERMANKINEMATICMODEL_HPP_

#include <eigen3/Eigen/Core>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>


/**
 *
 * @brief declaration of AckermanKinematicModel class
 *
 */
class AckermanKinematicModel {
 private:
  double wheelBase;
  double axleWidth;
  double steeringAngle;
  double carVelocity;
  cv::Point3f carState;
  double dt = 0.01;

 public:
  /**
   *
   * @brief getter method for wheelbase
   * @param none
   * @return wheelBase
   *
   */
  double getWheelBase();

  /**
   *
   * @brief getter method for axleWidth
   * @param none
   * @return axleWidth
   *
   */
  double getAxleWidth();

  /**
   *
   * @brief getter method for steeringAngle
   * @param none
   * @return steeringAngle
   *
   */
  double getSteeringAngle();

  /**
   *
   * @brief getter method for carVelocity
   * @param none
   * @return carVelocity
   *
   */
  double getCarVelocity();

  /**
   *
   * @brief getter method for carState
   * @param none
   * @return carState
   *
   */
  cv::Point3f getCarState();

  /**
   *
   * @brief setter method for wheelbase
   * @param double l variable for wheel base
   * @return bool
   *
   */
  bool setWheelBase(double l);

  /**
   *
   * @brief setter method for axleWidth
   * @param double w variable for axle width
   * @return bool
   *
   */
  bool setAxleWidth(double w);

  /**
   *
   * @brief setter method for steeringAngle
   * @param Eigen::Vector2d controllerOutput
   * @return bool
   *
   */
  bool setCarVelocityAndSteeringAngle(Eigen::Vector2d controllerOutput);


  /**
   *
   * @brief setter method for carState
   * @param Point3f state - x, y, theta
   * @return bool
   *
   */
  bool setCarState(cv::Point3f state);

  /**
   *
   * @brief calculation of ackermann model parameters
   * @param none
   * @return Point3f - variable containing the states
   *
   */
  cv::Point3f calcAckermanParameters();

  /**
   *
   * @brief checks for constraints of angles
   * @param none
   * @return bool
   *
   */
  bool checkAngleConstraints();
};

#endif   // INCLUDE_ACKERMANKINEMATICMODEL_HPP_
