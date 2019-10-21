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
 * @file Map.cpp
 * @date 12th October 2019
 * @author Suyash Yeotikar (driver), Nakul Patel (navigator), Ishan Patel(design keeper)
 * @brief Map module source file containing function implementations.
 */

#include "Map.hpp"

/**
 * @brief function to set start coordinates based on user input
 * @param inputCoordinates Coordinates specified by user
 * @return bool true or false depending on if the value was set or not
 **/
bool Map::SetStartCoordinates(cv::Point3f inputCoordinates) {
  bool b = UpdateRobotLocation(inputCoordinates);
  return b;
}


/**
 * @brief function to set Destination coordinates.
 * @param inputCoordinates Coordinates specific to destination on the map
 * @return bool true or false depending on value was set or not
 */
bool Map::SetDestinationCoordinates(cv::Point3f inputCoordinates) {
  bool valid = CheckValidCoordinates(inputCoordinates);
  if (valid) {
    destinationCoords = inputCoordinates;
    return true;
  } else {
    return false;
  }
}

/**
 * @brief function to set the location of robot.
 * Output from ackerman kinematic model will be used here
 * to set robot location
 * @param coordinates Coordinates output from ackermann model that
 * need to be set so that it is reflected in the map
 */
bool Map::UpdateRobotLocation(cv::Point3f coordinates) {
  bool val = CheckValidCoordinates(coordinates);
  if (val) {
    currCoords = coordinates;

    cv::circle(currMapImage, cv::Point(currCoords.x, currCoords.y),
               30.0, cv::Scalar(0, 0, 255), 1, 8, 0);
    cv::circle(currMapImage, cv::Point(destinationCoords.x, destinationCoords.y),
                   10.0, cv::Scalar(255, 0, 0), -1, 8, 0);

    cv::Point2f center(coordinates.x, coordinates.y);
    cv::Size2f robotSize(length, width);
    float angle = coordinates.z;
    /// creating a rotated rectangle for updated position
    cv::RotatedRect rotatedRectangle = cv::RotatedRect(center, robotSize,
                                                       angle);
    std::vector<cv::Point2f> rectPoints;
    std::vector<cv::Point2f> rotatedPoints;
    rectPoints.push_back(cv::Point2f(-length / 2, -width / 2));
    rectPoints.push_back(cv::Point2f(length / 2, -width / 2));
    rectPoints.push_back(cv::Point2f(length / 2, width / 2));
    rectPoints.push_back(cv::Point2f(-length / 2, width / 2));
    cv::Point2f translationPoint;
    translationPoint.x = center.x * cos(angle * 3.14 / 180)
        + center.y * sin(angle * 3.14 / 180);
    translationPoint.y = -center.x * sin(angle * 3.14 / 180)
        + center.y * cos(angle * 3.14 / 180);

    for (auto p : rectPoints) {
      cv::Point2f rotPoint;
      rotPoint.x = p.x * cos(angle * M_PI / 180) - p.y * sin(angle * M_PI / 180)
          + center.x;
      rotPoint.y = p.x * sin(angle * M_PI / 180) + p.y * cos(angle * M_PI / 180)
          + center.y;
      rotatedPoints.push_back(rotPoint);
    }
    std::vector<cv::Point> corners;
    std::transform(rotatedPoints.begin(), rotatedPoints.begin(),
                   std::back_inserter(corners),
                   [](const cv::Point2f& p) {return (cv::Point)p;});

    for (auto p : rotatedPoints) {
      corners.push_back(p);
    }
    /// using fillConvexPoly to fill rectangle with color
    cv::fillConvexPoly(currMapImage, corners.data(), 4,
                       cv::Scalar(0, 255, 0));
    bool b = DisplayMapImage();
    /// Resetting the image
    currMapImage = cv::Mat::zeros(mapBounds.y, mapBounds.x, CV_8UC3);
    /// Resetting the image
    currMapImage.setTo(cv::Scalar(255, 255, 255));
    return true;
  } else {
    std::cout << "Could not update the coordinates." << std::endl;
    return false;
  }
}

/**
 * @brief function to get the current robot coordinates in map.
 * Used in feed back for PID controller module to get
 * current robot location in the map.
 * @return the current position (x,y) and heading angle
 */
cv::Point3f Map::GetRobotCoordinates() {
  return currCoords;
}


/**
 * @brief function to initialize the map in which robot will move.
 * Robot image is loaded and stored in robot image
 * Map will be initialized based on bounds.
 * Function returns the map image.
 * @param none
 * @return Mat initialized map image with robot at one corner.
 */
bool Map::InitializeMap(cv::Point inMapBounds, double robotLength,
                        double robotWidth) {
  length = robotLength * 10.0;
  width = robotWidth * 10.0;
  int bound = 1000;
  if (inMapBounds.x < bound && inMapBounds.y < bound && inMapBounds.x > 0
      && inMapBounds.y > 0) {
    mapBounds.x = inMapBounds.x;
    mapBounds.y = inMapBounds.y;
    currMapImage = cv::Mat::zeros(mapBounds.y, mapBounds.x, CV_8UC3);
    currMapImage.setTo(cv::Scalar(255, 255, 255));
    UpdateRobotLocation(
        cv::Point3f((length / 2), (width / 2), 0));
    return true;
  } else {
    return false;
  }
}

/**
 * @brief function to check if robot has reached the set target point or not.
 * Function compares the the current location and destination location.
 * Distance should be 2 for robot to have reached location
 * @param none
 * @return true or false depending on if robot has reached location
 */
bool Map::CheckReachedDestination() {
  if (abs(currCoords.x - destinationCoords.x) < 7 && abs(currCoords.y - destinationCoords.y) < 7) {
    std::cout << "Robot reached within 7 pixels distance of destination. Exiting!" << std::endl;
      return true;
  } else {
      return false;
  }
}

/**
 * @brief check if the input coordinates are valid,
 * based on if they are within the bounds of map or not.
 * Use transformation matrices to translate and rotate points of rectangle
 * and then determine if they are within bounds.
 * @param Point3f input coordinates to be checks if they are valid or not
 * @return bool true or false depending on if the coordinates are
 * within bounds or not
 */
bool Map::CheckValidCoordinates(cv::Point3f inputCoordinates) {
  cv::Point2f center(inputCoordinates.x, inputCoordinates.y);
  cv::Size2f robotSize(width, length);
  float angle = inputCoordinates.z;
  /// creating a rotated rectangle for given parameters
  cv::RotatedRect rotatedRectangle = cv::RotatedRect(center, robotSize, angle);
  std::vector<cv::Point2f> rectPoints;
  std::vector<cv::Point2f> rotatedPoints;
  rectPoints.push_back(cv::Point2f(-length / 2, -width / 2));
  rectPoints.push_back(cv::Point2f(length / 2, -width / 2));
  rectPoints.push_back(cv::Point2f(length / 2, width / 2));
  rectPoints.push_back(cv::Point2f(-length / 2, width / 2));
  cv::Point2f translationPoint;
  translationPoint.x = center.x * cos(angle) + center.y * sin(angle);
  translationPoint.y = -center.x * sin(angle) + center.y * cos(angle);

  for (auto p : rectPoints) {
    cv::Point2f rotPoint;
    rotPoint.x = p.x * cos(angle) - p.y * sin(angle) + center.x;
    rotPoint.y = p.x * sin(angle) + p.y * cos(angle) + center.y;
    rotatedPoints.push_back(rotPoint);
  }

  for (auto p : rotatedPoints) {
    std::cout << (p.y > mapBounds.y);
    std::cout << (p.x > mapBounds.x);
    if (p.x < 0 || p.y < 0 || p.x > mapBounds.x || p.y > mapBounds.y) {
      return false;
    }
  }
  return true;

}

/**
 * @brief Function to display map with the
 * updated location of the robot
 * @param none
 * @return bool true or false depending on map is successfully displayed
 * or not.
 */
bool Map::DisplayMapImage() {
  if (currMapImage.rows > 0 && currMapImage.cols > 0) {
    /// Create a window for display.
    cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);
    /// Show the image inside it
    cv::imshow("Display window", currMapImage);
    cv::waitKey(10);
    return true;
  } else {
    std::cout << "Exception: failed to display image.";
    return false;
  }
}

