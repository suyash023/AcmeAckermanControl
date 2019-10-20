# Acme's Controller for Ackermann Kinematic Model

[![Build Status](https://travis-ci.org/suyash023/AcmeAckermanControl.svg?branch=iteration2)](https://travis-ci.org/suyash023/AcmeAckermanControl)
[![Coverage Status](https://coveralls.io/repos/github/suyash023/AcmeAckermanControl/badge.svg?branch=iteration2)](https://coveralls.io/github/suyash023/AcmeAckermanControl?branch=iteration2)
---

## Project Overview and Description:

This repository contains the implementation of the Ackermann Module, with the visualization, and three modules: Map, AckermanKinematicModel and Pid. The map module consists of a 2D visualisation in which the map and robot are represented as images, with robot as a smaller image. The user provides target position and heading angle. The PID controller takes current and target state(x, y, theta) from the Map class and gives steering angle and car(robot) velocity as output. The AckermanKinemticModel takes the car velocity and steering angle as input from the PID class, and updates the state by calculating the variation in them and adding it to each state, which in turn is given as input to Map class, to update the robot image. 

## Group Members

Ishan Patel
Nakul Patel
Suyash Yeotikar


## Standard install via command-line:
```
git clone --recursive https://github.com/suyash023/AcmeAckermanControl.git
cd <path to repository>
mkdir build
cd build
cmake ..
make
Run tests: ./test/cpp-test
Run program: ./app/shell-app
```

## Google Sheet for AIP: 

https://docs.google.com/spreadsheets/d/1cCmYP9rGVsWmyAf39ayQJ9aq3QtM9_lUnefQbatCKaI/edit?usp=sharing


## Sprint Planning Notes:

https://docs.google.com/document/d/1JT7k-D8IgDNpvIvN3qUCVz5M1oPum1FsyDsK1AOXwtk/edit?usp=sharing



## Agile Iteration Process for this project:

## Iteration 1:
Repository setup, Travis and Coveralls setup, License selection,
class design, UML diagrams and writing class stubs.

Map: Suyash(Driver), Nakul(Navigator) and Ishan(Design Keeper)

AckermanKinematicModel: Nakul(Driver), Ishan(Navigator) and Suyash(Design Keeper)

Pid: Ishan(Driver), Suyash(Navigator) and Nakul(Design Keeper)


## Doxygen Documentation:

Will be updated at the end of iteration2


## Building for code coverage:
```
sudo apt-get install lcov
cmake -D COVERAGE=ON -D CMAKE_BUILD_TYPE=Debug ../
make
make code_coverage
```
This generates a index.html page in the build/coverage sub-directory that can be viewed locally in a web browser.

## Working with Eclipse IDE ##
## Installation

In your Eclipse workspace directory (or create a new one), checkout the repo (and submodules)
```
mkdir -p ~/workspace
cd ~/workspace
git clone --recursive https://github.com/suyash023/AcmeAckermanControl.git
```
In your work directory, use cmake to create an Eclipse project for an [out-of-source build] of cpp-boilerplate

```
cd ~/workspace
mkdir -p boilerplate-eclipse
cd boilerplate-eclipse
cmake -G "Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug -D CMAKE_ECLIPSE_VERSION=4.7.0 -D CMAKE_CXX_COMPILER_ARG1=-std=c++14 ../AcmeAckermanControl/
```






