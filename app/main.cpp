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
 *
 * @file main.cpp
 *
 * @date 2019-10-19
 *
 * @author Ishan Patel, Nakul Patel, Suyash Yeotikar
 *
 * @brief main program for Acme Robotics' Ackermann Controller.
 *
 * @version 1
 *
 */
#include <iostream>
#include <sstream>
#include <fstream>
#include <boost/tokenizer.hpp>
#include "Pid.hpp"
#include "Map.hpp"
#include "AckermanKinematicModel.hpp"

/**
 * @brief Main function
 * @param none
 * @return 0
 */
int main(int argc, char *argv[]) {
    Pid pidObj;
    Map mapObj;
    AckermanKinematicModel akmObj;
    bool checkValid;
    if ( argc != 2 ) {
        std::cout << "Entered incorrect command line parameters! "
              "argument is file name" << std::endl;
        return 0;
    }
    std::ifstream inputFile(argv[1]);
    if ( !inputFile ) {
        std::cout << "Could not read file. Please check if it exists"
              << std::endl;
        return 0;
    }

    std::string input;
    std::getline(inputFile, input);
    std::cout << "Robot wheel base read was : " << input << std::endl;
    double length = std::atof(input.c_str());
    checkValid = akmObj.setWheelBase(length);
    if ( !checkValid ) {
        std::cout << "Problem with parameter. Please check the file."
              << std::endl;
        return 0;
    }

    std::getline(inputFile, input);
    std::cout << "Robot axle width read was : " << input << std::endl;
    double width = std::atof(input.c_str());
    checkValid = akmObj.setAxleWidth(width);
    if ( !checkValid ) {
        std::cout << "Problem with parameter. Please check the file."
                << std::endl;
        return 0;
    }

    std::getline(inputFile, input);
    std::cout << "Map bounds read was: " << input <<std::endl;
    boost::char_delimiters_separator<char> sep(" ");
    boost::tokenizer<boost::char_delimiters_separator<char> > tok(input, sep);
    boost::tokenizer<>::iterator iter =
            tok.begin();
    cv::Point bounds;
    bounds.x = std::atoi((*iter).c_str());
    iter++;
    bounds.y = std::atoi((*iter).c_str());
    checkValid = mapObj.InitializeMap(bounds, length, width);
    if ( !checkValid ||  length*10 > 0.2*bounds.x || width*10 > 0.2*bounds.y ) {
        std::cout << "Check bounds and wheel base and axlewidth of robot. "
                "Wheel base and axle width should not be"
                " more than 20% of size of map"
                << std::endl;
        return 0;
    }
    mapObj.DisplayMapImage();

    std::getline(inputFile, input);
    std::cout << "Start coordinates read was: " << input << std::endl;
    boost::tokenizer<boost::char_delimiters_separator<char> > tok1(input, sep);
    boost::tokenizer<>::iterator iter1 =
            tok1.begin();
    cv::Point3f start;
    start.x = std::atof((*iter1).c_str());
    iter1++;
    start.y = std::atof((*iter1).c_str());
    iter1++;
    start.z = std::atof((*iter1).c_str());
    checkValid = mapObj.SetStartCoordinates(start);
    if ( !checkValid ) {
        std::cout << "Incorrect start coordinates "<< std::endl;
        return 0;
    }
    akmObj.setCarState(start);

    std::getline(inputFile, input);
    std::cout << "Destination coordinates read was: " << input << std::endl;
    boost::tokenizer<boost::char_delimiters_separator<char> > tok2(input, sep);
    boost::tokenizer<>::iterator iter2 =
            tok2.begin();
    cv::Point3f destination;
    destination.x = std::atof((*iter2).c_str());
    iter2++;
    destination.y = std::atof((*iter2).c_str());
    iter2++;
    destination.z = std::atof((*iter2).c_str());
    checkValid = mapObj.SetDestinationCoordinates(destination);
    if ( !checkValid ||  length*10 > 0.2*bounds.x || width*10 > 0.2*bounds.y ) {
        std::cout << "Incorrect destination coordinates." << std::endl;
        return 0;
    }

    std::getline(inputFile, input);
    std::cout << "Kp matrix read was: " << input << std::endl;
    boost::tokenizer<boost::char_delimiters_separator<char> > tok3(input, sep);
    std::vector<double> tempVec;
    for ( auto str : tok3 ) {
        tempVec.push_back(std::atof(str.c_str()));
    }
    tempVec.clear();
    Eigen::Matrix<double, 2, 3> kp(tempVec.data());
    checkValid = pidObj.setKp(kp);
    if ( !checkValid ) {
        std::cout << "Incorrect kp values" << std::endl;
        return 0;
    }

    std::getline(inputFile, input);
    std::cout << "Ki matrix read was: " << input << std::endl;
    boost::tokenizer<boost::char_delimiters_separator<char> > tok4(input, sep);
    for ( auto str : tok4 ) {
        tempVec.push_back(std::atof(str.c_str()));
    }
    Eigen::Matrix<double, 2, 3> ki(tempVec.data());
    checkValid = pidObj.setKi(ki);
    if ( !checkValid ) {
        std::cout << "Incorrect ki values" << std::endl;
        return 0;
    }

    std::getline(inputFile, input);
    std::cout << "Kd matrix read was: " << input << std::endl;
    boost::tokenizer<boost::char_delimiters_separator<char> > tok5(input, sep);
    for ( auto str : tok5 ) {
        tempVec.push_back(std::atof(str.c_str()));
    }
    Eigen::Matrix<double, 2, 3> kd(tempVec.data());
    checkValid = pidObj.setKd(kd);
    if ( !checkValid ) {
        std::cout << "Incorrect kd values" << std::endl;
        return 0;
    }

    std::cout << "Starting PID loop... " << std::endl;
    Eigen::Vector3d destinationCoord;
    destinationCoord << destination.x, destination.y, destination.z;
    while (1) {
        if (mapObj.CheckReachedDestination()) {
            break;
        }
        cv::Point3f currPos = mapObj.GetRobotCoordinates();
        Eigen::Vector3d currCoord, destinationCoord;
        Eigen::Vector2d out;
        currCoord << currPos.x, currPos.y, currPos.z;
        out = pidObj.getControllerOutput(currCoord, destinationCoord);
        std::cout << "Command velocity is: " << out << std::endl;
        checkValid = akmObj.setCarVelocityAndSteeringAngle(out);
        if ( !checkValid ) {
            std::cout << "Cannot move to the point!" << std::endl;
            break;
        }
        cv::Point3f setState;
        setState = akmObj.calcAckermanParameters();
        checkValid = mapObj.UpdateRobotLocation(setState);
        if (!checkValid) {
            std::cout << "Robot tried to move to an invalid state" << std::endl;
            break;
        }
        mapObj.DisplayMapImage();
    }

    return 0;
}

