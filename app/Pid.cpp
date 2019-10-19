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


 * @file Pid.cpp
 *
 * @author Ishan Patel
 *
 * @brief implementation of Pid controller class
 *
 * @version 1
 *
 * @date 2019-10-13
 *
 * @section DESCRIPTION
 *
 * This is the .cpp file that implements the
 * methods for Pid controller class
 *
 */

#include "Pid.hpp"
#include <eigen3/Eigen/Core>


namespace Eigen {
    auto begin(Eigen::Matrix<double, 2, 3> const& m) {
        return m.data();
    }
    auto end(Eigen::Matrix<double, 2, 3> const& m) {
        return m.data()+m.size();
    }
}

Eigen::Matrix<double, 2, 3> Pid::getKp() {
    return kp;
}

Eigen::Matrix<double, 2, 3> Pid::getKi() {
    return ki;
}

Eigen::Matrix<double, 2, 3> Pid::getKd() {
    return kd;
}

bool Pid::setKp(Eigen::Matrix<double, 2, 3> kpIn) {
    for ( auto element : kpIn ) {
        if ( element < 0 ) {
            return false;
        }
    }
    kp = kpIn;
    return true;
}

bool Pid::setKi(Eigen::Matrix<double, 2, 3> kiIn) {
    for ( auto element : kiIn ) {
        if ( element < 0 ) {
            return false;
        }
    }
    ki = kiIn;
    return true;
}

bool Pid::setKd(Eigen::Matrix<double, 2, 3> kdIn) {
    for ( auto element : kdIn ) {
        if ( element < 0 ) {
            return false;
        }
    }
    kd = kdIn;
    return true;
}

Eigen::Vector2d Pid::getControllerOutput(
        Eigen::Vector3d targetState, Eigen::Vector3d currentState) {
    Eigen::Vector3d error;
    Eigen::Vector3d errorDiff;
    Eigen::Vector2d controlOut;
    error = targetState - currentState;
    errorDiff = error - lastError;
    errorSum = errorSum + error;
    controlOut = kp*error + ki*errorSum + kd*errorDiff;
    lastError = error;
    return controlOut;
}

void Pid::resetErrors() {
    lastError = Eigen::Vector3d::Zero();
    errorSum = Eigen::Vector3d::Zero();
}
