/*
 * stoc_controller.h
 Implementation of Stochastic Controller in Robot Operating System(ROS) to test on various robotic hardware.
   
   The details of stochastic controller design can be found in the paper submitted to Transactions of Automatic Control, which is under review. Shridhar Shah, Herbert Tanner, Chetan Pahlajani,"Stochastic receding horizon control of nonlinear stochastic systems with probabilistic state constraints", Transactions of Automatic Control (In Review)

   Please go through the paper at http://arxiv.org/abs/1211.4038 to understand the stochastic controller.
 
   Copyright (C) 2011 - 20012, Shridhar Shah,
   All rights reserved.                          

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:

     1. Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.

     2. Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.

     3. The names of its contributors may not be used to endorse or promote 
        products derived from this software without specific prior written 
        permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
   A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
   PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

   Any Feedback is welcome:
   shridhar [at] udel [dot] edu
*/

#ifndef STOC_CONTROLLER_H_
#define STOC_CONTROLLER_H_

#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"

#ifndef PI
#define PI 3.14159265358979323846
#endif

using namespace std;
using std::vector;

typedef vector<double> Row;
typedef vector<Row> Matrix;
typedef vector<Matrix> Darray;

class stoc_controller {
protected:
	geometry_msgs::Pose2D goal, localCurrentPose;
	Row lpwm, rpwm, lvel, avel;
	int commandL, commandR;
	double command_lvel, command_avel;

	ros::Publisher command_pub;
	ros::Subscriber pose_updater;

public:

	double Region;
	double epsilon;
	int Nxinput, Nyinput, Nwinput;
	Darray inputw;
	Darray gridpinput;
	Darray gridqinput;
	Darray gridwinput;

	double dxinput, dyinput, dwinput;

	void set_goal(geometry_msgs::Pose2D&); // Used for setting intermediate goal
	void pose2DCallback(const geometry_msgs::Pose2D::ConstPtr& );
	stoc_controller(ros::NodeHandle &, geometry_msgs::Pose2D &);
	//~stoc_controller();

	void readVWData(); // Reads numerical data for control inputs
	void drive();

	void interp3(geometry_msgs::Pose2D&); // Interpolate control inputs for current position

};



#endif /* STOC_CONTROLLER_H_ */
