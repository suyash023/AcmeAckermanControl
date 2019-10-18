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
bool Map::InitializeMap(cv::Point mapBounds) {

}

/**
 * @brief function to check if robot has reached the set target point or not.
 * Function compares the the current location and destination location.
 * Distance should be 2 for robot to have reached location
 * @param none
 * @return true or false depending on if robot has reached location
 */
bool Map::CheckReachedDestination() {
  if (currCoords == destinationCoords) {
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
 * @param Point3f input coordinates to be checkes if they are valid or not
 * @return bool true or false depending on if the coordinates are
 * within bounds or not
 */
bool Map::CheckValidCoordinates(cv::Point3f inputCoordinates) {
  float tempX = inputCoordinates.x;
  float tempY = inputCoordinates.y;

  return false;
}

/**
 * @brief Function to display map with the
 * updated location of the robot
 * @param none
 * @return bool true or false depnding on map is successfully displayed
 * or not.
 */
bool Map::DisplayMapImage() {
  try {
    /// Create a window for display.
    cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);
    /// Show our image inside it
    cv::imshow("Display window", currMapImage);
    cv::waitKey(0);
    return true;
  } catch (cv::Exception& e) {
    std::cout << "Exception: failed to display image." << e.what();
    return false;
  }
}

