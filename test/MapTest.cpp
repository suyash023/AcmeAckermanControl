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
 * @date 12th October 2019
 * @author Suyash Yeotikar (driver), Navigator (Nakul Patel), Design Keeper(Ishan Patel)
 * @brief Map module test source file containing function tests for map module.
 */

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include "Map.hpp"

/**
 * @brief Test if Start Coordinates and get coordinates functions works properly
 * Set valid coordinates that are within the map
 * @param none
 * @return none
 */

TEST(SetStartCoordinates, testSetStartCoordinatesValid ) {
    Map testObj = Map();
    testObj.InitializeMap(cv::Point(500, 500), 2.5, 3.5);
    cv::Point3f testPoint;
    testPoint.x = 100;
    testPoint.y = 100;
    testPoint.z = (3.14)/2;
    ASSERT_TRUE(testObj.SetStartCoordinates(testPoint));
    cv::Point3f currRobotPointNew = testObj.GetRobotCoordinates();
    ASSERT_EQ(currRobotPointNew.x, testPoint.x);
    ASSERT_EQ(currRobotPointNew.y, testPoint.y);
    ASSERT_EQ(currRobotPointNew.z, testPoint.z);
}

/**
 * @brief Test if Start Coordinates function works properly
 * Set invalid coordinates that are out of bounds.
 * Set function should return false
 * @param none
 * @return none
 */

TEST(SetStartCoordinates, testSetStartCoordinatesInvalid) {
    Map testObj = Map();
    testObj.InitializeMap(cv::Point(500, 500), 2.5, 3.5);
    cv::Point3f currRobotPoint = testObj.GetRobotCoordinates();
    cv::Point3f testPoint1;
    testPoint1.x = -1;
    testPoint1.y = -1;
    testPoint1.z = 0;
    ASSERT_FALSE(testObj.SetStartCoordinates(testPoint1));
    cv::Point3f currRobotPointNew = testObj.GetRobotCoordinates();
    ASSERT_EQ(currRobotPoint.x, currRobotPointNew.x);
    ASSERT_EQ(currRobotPoint.y, currRobotPointNew.z);
    ASSERT_EQ(currRobotPoint.z, currRobotPointNew.z);
    cv::Point3f testPoint2;
    testPoint2.x = 500;
    testPoint2.y = 500;
    testPoint2.z = 0;
    ASSERT_FALSE(testObj.SetStartCoordinates(testPoint1));
    currRobotPointNew = testObj.GetRobotCoordinates();
    ASSERT_EQ(currRobotPoint.x, currRobotPointNew.x);
    ASSERT_EQ(currRobotPoint.y, currRobotPointNew.z);
    ASSERT_EQ(currRobotPoint.z, currRobotPointNew.z);
}

/**
 * @brief Test if InitializeMap function works properly
 * Set valid map bounds that are corret.
 * function should return true
 * @param none
 * @return none
 */

TEST(InitializeMap, testInitializeMapValid) {
    Map testObj = Map();
    ASSERT_TRUE(testObj.InitializeMap(cv::Point(500, 500), 2.5, 3.5));
    ASSERT_TRUE(testObj.DisplayMapImage());
}

/**
 * @brief Test if InitializeMap function works properly
 * Set inva valid map bounds that are incorrect.
 * function should return false
 * @param none
 * @return none
 */

TEST(InitializeMap, testInitializeMapInvalid) {
    Map testObj = Map();
    ASSERT_FALSE(testObj.InitializeMap(cv::Point(-100, -100), 2.5 , 3.5));
    ASSERT_FALSE(testObj.DisplayMapImage());
}


/**
 * @brief Test if Destination Coordinates function works properly
 * Set valid coordinates that are within the map
 * @param none
 * @return none
 */

TEST(SetDestinationCoordinates, testSetDestinationCoordinatesValid ) {
    Map testObj = Map();
    testObj.InitializeMap(cv::Point(500, 500), 2.5, 3.5);
    cv::Point3f testPoint;
    testPoint.x = 10;
    testPoint.y = 10;
    testPoint.z = (3.14)/2;
    ASSERT_TRUE(testObj.SetDestinationCoordinates(testPoint));
}

/**
 * @brief Test if Destination Coordinates function works properly
 * Set invalid coordinates that are out of bounds.
 * Set function should return false
 * @param none
 * @return none
 */

TEST(SetDestinationCoordinates, testSetDestinationCoordinatesInvalid) {
    Map testObj = Map();
    testObj.InitializeMap(cv::Point(500, 500), 2.5, 3.5);
    cv::Point3f currRobotPoint = testObj.GetRobotCoordinates();
    cv::Point3f testPoint1;
    testPoint1.x = -1;
    testPoint1.y = -1;
    testPoint1.z = 0;
    ASSERT_FALSE(testObj.SetDestinationCoordinates(testPoint1));
    cv::Point3f testPoint2;
    testPoint2.x = 500;
    testPoint2.y = 500;
    testPoint2.z = 0;
    ASSERT_FALSE(testObj.SetDestinationCoordinates(testPoint2));
}






