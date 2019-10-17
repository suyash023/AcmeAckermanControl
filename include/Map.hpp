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
 * @file Map.hpp
 * @date 12th October 2019
 * @author Suyash Yeotikar (driver), Nakul Patel (Navigator), Ishan Patel (Design Keeper)
 * @brief Map module header containing function definitions and variables.
 * @mainpage
 **/

#ifndef INCLUDE_MAP_HPP_
#define INCLUDE_MAP_HPP_

#include <string>
#include <opencv2/opencv.hpp>

class Map {
 private:
    cv::Point mapBounds;
    cv::Mat robotImage;
    cv::Mat currMapImage;
    cv::Point3f startCoords;
    cv::Point3f destinationCoords;
    cv::Point3f currCoords;
    std::string robotImageLocation = "/home/suyash/Desktop/"
            "software_dev_for_robotics/midterm-project/AcmeAckermanControl/"
            "images/Robot_image.png";

 public:
    bool SetStartCoordinates(cv::Point3f inputCoordinates);

    bool SetDestinationCoordinates(cv::Point3f inputCoordinates);

    bool UpdateRobotLocation(cv::Point3f coordinates);

    cv::Point3f GetRobotCoordinates();

    bool InitializeMap(cv::Point mapBounds);

    bool CheckReachedDestination();

    bool CheckValidCoordinates(cv::Point3f inputCoordinates);

    bool DisplayMapImage();

    bool LoadRobotImage();
};


#endif  // INCLUDE_MAP_HPP_
