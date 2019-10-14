/**
 BSD 3-Clause License
<<<<<<< HEAD
ssssssss Copyright (c) 2019, Ishan Patel, Nakul Patel, Suyash Yeotikar
=======

 Copyright (c) 2019, Ishan Patel, Nakul Patel, Suyash Yeotikar
>>>>>>> 69a6cbbbbdffaf7e429ab28a94dba5ba788d0f0a
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


<<<<<<< HEAD
 * @file Pid.hpp
=======
 * @file PID.hpp
>>>>>>> 69a6cbbbbdffaf7e429ab28a94dba5ba788d0f0a
 *
 * @author Ishan Patel
 *
 * @brief declaration of PID class
 *
 * @version 1
 *
 * @date 2019-10-13
 *
 * @section DESCRIPTION
 *
 * This is the header file that will be used for the declaration
 * of class attributes and methods for PID controller
 *
 */

#ifndef INCLUDE_PID_HPP_
#define INCLUDE_PID_HPP_

#include <iostream>
<<<<<<< HEAD
#include <eigen3/Eigen/Core>

/**
 *
 * @brief declaration of Pid class
 *
 */
class Pid {
private:
	Eigen::MatrixXd kp;
	Eigen::Matrix2d ki;
	Eigen::Matrix2d kd;
	Eigen::Vector3d lastError = Eigen::Vector3d::Zero();
	Eigen::Vector3d errorSum = Eigen::Vector3d::Zero();
public:
	/**
	 *
	 * @brief getter method for kp gains
	 *
	 * @param none
	 *
	 * @return 2x3 matrix of kp gains
	 *
	 */
	Eigen::MatrixXd getKp();

	/**
	 *
	 * @brief getter method for ki gains
	 *
	 * @param none
	 *
	 * @return 2x3 matrix of ki gains
	 *
	 */
	Eigen::MatrixXd getKi();

	/**
	 *
	 * @brief getter method for kd gains
	 *
	 * @param none
	 *
	 * @return 2x3 matrix of kd gains
	 *
	 */
	Eigen::MatrixXd getKd();

	/**
	 *
	 * @brief setter method for kp gains
	 *
	 * @param kpIn 2X3 matrix of kp gains
	 *
	 * @return bool
	 *
	 */
	Eigen::MatrixXd setKp(Eigen::MatrixXd kpIn);
	/**
	 *
	 * @brief setter method for ki gains
	 *
	 * @param kiIn 2X3 matrix of ki gains
	 *
	 * @return bool
	 *
	 */
	Eigen::MatrixXd setKi(Eigen::MatrixXd kiIn);

	/**
	 *
	 * @brief setter method for kd gains
	 *
	 * @param kdIn 2X3 matrix of kd gains
	 *
	 * @return bool
	 *
	 */
	Eigen::MatrixXd setKd(Eigen::MatrixXd kdIn);

	/**
	 *
	 * @brief Estimate velocity and steering angle given target state and actual state
	 *
	 * @param targetState The desired state of the system (x,y,theta)
	 *
	 * @param actualState The actual state of the system (x,y,theta)
	 *
	 * @return vector of velocity and steering angle
	 *
	 */
	Eigen::Vector2d getControllerOutput(Eigen::Vector3d targetState,
			Eigen::Vector3d currentState);

	/**
	 *
	 * @brief Reset the last error and sum of errors
	 *
	 * @param none
	 *
	 * @return none
	 *
	 */
	void resetErrors();
};
=======
#include <Eigen/Dense>

/**
 *
 * @brief declaration of AckermanKinematicModel class
 *
 */
class Pid



>>>>>>> 69a6cbbbbdffaf7e429ab28a94dba5ba788d0f0a

#endif /* INCLUDE_PID_HPP_ */
