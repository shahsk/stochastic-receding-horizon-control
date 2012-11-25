/*
 * stoc_controller.cpp
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

#include "stoc_controller.h"
//#include <iostream>
//#include <fstream>
//#include <stdlib.h>
//#include <stdio.h>
using namespace std;
using std::vector;

stoc_controller::stoc_controller(ros::NodeHandle& n, geometry_msgs::Pose2D & setgoal){

// Constructor - Define all variables necessory

	this->Region = 0.5;
	this->epsilon = 0.1;
	this->Nxinput = 41;
	this->Nyinput = 41;
	this->Nwinput = 41;
	this->dxinput = 0.025;
	this->dyinput = 0.025;
	this->dwinput = PI/20;

	this->commandL = 0;
	this->commandR = 0;

	this->command_lvel = 0.0;
	this->command_avel = 0.0;


	this->inputw.resize(this->Nxinput);
	this->gridpinput.resize(this->Nxinput);
	this->gridqinput.resize(this->Nxinput);
	this->gridwinput.resize(this->Nxinput);
	for (int i = 0; i < this->Nxinput; i++)
	{
		this->inputw[i].resize(this->Nyinput);
		this->gridpinput[i].resize(this->Nyinput);
		this->gridqinput[i].resize(this->Nyinput);
		this->gridwinput[i].resize(this->Nyinput);
	    for (int j = 0; j < this->Nyinput; j++)
	    {
	    	this->inputw[i][j].resize(this->Nwinput);
	    	this->gridpinput[i][j].resize(this->Nwinput);
	    	this->gridqinput[i][j].resize(this->Nwinput);
	    	this->gridwinput[i][j].resize(this->Nwinput);
	    }
	}



/*	Darray inputw(this->Nxinput,Matrix(this->Nyinput,Row(this->Nwinput)));
	Darray gridpinput(this->Nxinput,Matrix(this->Nyinput,Row(this->Nwinput)));
	Darray gridqinput(this->Nxinput,Matrix(this->Nyinput,Row(this->Nwinput)));
	Darray gridwinput(this->Nxinput,Matrix(this->Nyinput,Row(this->Nwinput)));
*/
	// Initialize message publisher and subscriber

	this->command_pub = n.advertise<geometry_msgs::Point>("servocommand", 1);
	this->pose_updater = n.subscribe("vicon_pose2D", 1, &stoc_controller::pose2DCallback, this);

	cout << "Begin reading data" << endl;

	this->readVWData();


	ros::spinOnce();

	this->set_goal(setgoal);
}


void stoc_controller::set_goal(geometry_msgs::Pose2D & setgoal)
{
// Function to set robot goal position
	this->goal.x = setgoal.x;
	this->goal.y = setgoal.y;
	this->goal.theta = setgoal.theta;


}

void stoc_controller::pose2DCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
//Callback function to update robot pose

this->localCurrentPose.x = msg->x - this->goal.x;
this->localCurrentPose.y = msg->y - this->goal.y;
this->localCurrentPose.theta = msg->theta;

}


void stoc_controller::readVWData()
{
//    vector<double> ls, rs, lv, av;
// Read numerical controller data
	int i, j, k;
    ifstream inFile;
    inFile.open("VWChart.csv");
    string line;
    int linenum = 0;
    while (getline (inFile, line))
    {
        linenum++;
        //cout << "\nLine #" << linenum << ":" << endl;
        istringstream linestream(line);
        string item;
        int itemnum = 0;
        while (getline (linestream, item, ','))
        {
	    if(itemnum == 0)
		this->lpwm.push_back(strtod(item.c_str(),NULL));

	    if(itemnum == 1)
		this->rpwm.push_back(strtod(item.c_str(),NULL));
	    if(itemnum == 2)
		this->lvel.push_back(strtod(item.c_str(),NULL));
	    if(itemnum == 3)
		this->avel.push_back(strtod(item.c_str(),NULL));

            itemnum++;

          cout << "Item #" << itemnum << ": " << item << endl;
        }
    }

    string valw;
    double numw;
    ifstream inputfile_w; //inputfile_x, inputfile_y,
    //inputfile_x.open("demofile_x");
    //inputfile_y.open("demofile_y");
    inputfile_w.open("demofile_w");

    cout << "begin reading file" << endl;
    for (k = 0;k<this->Nwinput;k++){
    	for (j = 0;j<this->Nyinput;j++){
    		for (i=0;i<this->Nxinput;i++){
    			getline(inputfile_w, valw);
    			numw = strtod(valw.c_str(),NULL);
    			this->inputw[i][j][k] = numw;
        }
      }
    }

    inputfile_w.close();

cout << this->lpwm[0] << "," << this->lpwm[1] << endl;
cout << "myvector stores " << (int) this->lpwm.size() << " numbers.\n";

for (i=0;i<this->Nxinput;i++){
  for (j = 0;j<this->Nyinput;j++){
    for (k = 0;k<this->Nwinput;k++){

    	this->gridpinput[i][j][k] = (-this->Region + j*this->dxinput);
    	this->gridqinput[i][j][k] = (-this->Region + i*this->dyinput);
    	this->gridwinput[i][j][k] = (-PI + k*this->dwinput);

    }
  }
}

}


void stoc_controller::interp3(geometry_msgs::Pose2D& currentPose)
{
	//Implementation of MATLAB interp3 function

	double F[3];
	int i;
	int i0,j0,k0;
	double x, y, w;
	i0 = 0;
	j0 = 0;
	k0 = 0;

	x = currentPose.x - this->goal.x;
	y = currentPose.y - this->goal.y;
	w = currentPose.theta;

	w = fmod(w,2*PI);
	if(w > PI) {
	        w = w -  2*PI;
	}

	//Compute F
	for(int a = 0; a < this->Nwinput; a++ )
	{
	   if ( this->gridwinput[0][0][a] <= w && this->gridwinput[0][0][a+1] >= w)
	   {
		k0 = a;
		break;
	   }
	   else if (this->gridwinput[0][0][a] >= w && this->gridwinput[0][0][a+1] <= w )
	   {

		k0 = a;
		break;
	   }
	   else
	   {
	   }
	}

	//cout << "computed w0, x, y , w= " <<x << " " <<y << " " <<w << endl;
	for(int b = 0; b < this->Nyinput; b++ )
	{
		if ( this->gridqinput[b][0][0] <= y && this->gridqinput[b+1][0][0] >= y )
	       	{
		j0 = b;
		break;
		}
	}


	for(int c = 0; c < this->Nxinput; c++ )
	{
		if ( this->gridpinput[0][c][0] <= x && this->gridpinput[0][c+1][0] >= x )
	     	{
		i0 = c;
		break;
		}
	}

	//cout << "i0 =" << i0 << "j0 =" << j0 << "k0 =" << k0 << endl;

	//if(k0 != Nwinput && i0 != Nxinput && j0 != Nxinput ){
	     			double diffx = (x - this->gridpinput[j0][i0][k0])/(this->gridpinput[j0][i0+1][k0] - this->gridpinput[j0][i0][k0]);
				double diffy = (y - this->gridqinput[j0][i0][k0])/(this->gridqinput[j0+1][i0][k0] - this->gridqinput[j0][i0][k0]);
				double diffw = (w - this->gridwinput[j0][i0][k0])/(this->gridwinput[j0][i0][k0+1] - this->gridwinput[j0][i0][k0]);
	//cout << diffx << diffy << diffw << endl;
			/*	double c0 = inputx[j0][i0][k0];
				double c1 = inputx[j0][i0+1][k0] - inputx[j0][i0][k0];
				double c2 = inputx[j0+1][i0][k0] - inputx[j0][i0][k0];
				double c3 = inputx[j0][i0][k0+1] - inputx[j0][i0][k0];
				double c4 = inputx[j0+1][i0+1][k0] + inputx[j0][i0][k0] - inputx[j0][i0+1][k0] - inputx[j0+1][i0][k0];
				double c5 = inputx[j0+1][i0][k0+1] + inputx[j0][i0][k0] - inputx[j0][i0][k0+1] - inputx[j0+1][i0][k0];
				double c6 = inputx[j0][i0+1][k0+1] + inputx[j0][i0][k0] - inputx[j0][i0+1][k0] - inputx[j0][i0][k0+1];
				double c7 = inputx[j0+1][i0+1][k0+1] -inputx[j0+1][i0+1][k0] -inputx[j0+1][i0][k0+1] - inputx[j0][i0+1][k0+1] - inputx[j0][i0][k0] + inputx[j0][i0+1][k0] + inputx[j0+1][i0][k0] + inputx[j0][i0][k0+1];
			        F[0] = (c0 +  c1*diffx + c2*diffy + c3*diffw + c4*diffx*diffy + c5*diffy*diffw + c6*diffw*diffx + c7*diffx*diffy*diffw);




				 c0 = inputy[j0][i0][k0];
				 c1 = inputy[j0][i0+1][k0] - inputy[j0][i0][k0];
				 c2 = inputy[j0+1][i0][k0] - inputy[j0][i0][k0];
				 c3 = inputy[j0][i0][k0+1] - inputy[j0][i0][k0];
				 c4 = inputy[j0+1][i0+1][k0] + inputy[j0][i0][k0] - inputy[j0][i0+1][k0] - inputy[j0+1][i0][k0];
				 c5 = inputy[j0+1][i0][k0+1] + inputy[j0][i0][k0] - inputy[j0][i0][k0+1] - inputy[j0+1][i0][k0];
				 c6 = inputy[j0][i0+1][k0+1] + inputy[j0][i0][k0] - inputy[j0][i0+1][k0] - inputy[j0][i0][k0+1];
				 c7 = inputy[j0+1][i0+1][k0+1] -inputy[j0+1][i0+1][k0] -inputy[j0+1][i0][k0+1] - inputy[j0][i0+1][k0+1] - inputy[j0][i0][k0] + inputy[j0][i0+1][k0] + inputy[j0+1][i0][k0] + inputy[j0][i0][k0+1];
			        F[1] = (c0 +  c1*diffx + c2*diffy + c3*diffw + c4*diffx*diffy + c5*diffy*diffw + c6*diffw*diffx + c7*diffx*diffy*diffw);
			*/

				 double c0 = this->inputw[j0][i0][k0];
				 double c1 = this->inputw[j0][i0+1][k0] - this->inputw[j0][i0][k0];
				 double c2 = this->inputw[j0+1][i0][k0] - this->inputw[j0][i0][k0];
				 double c3 = this->inputw[j0][i0][k0+1] - this->inputw[j0][i0][k0];
				 double c4 = this->inputw[j0+1][i0+1][k0] + this->inputw[j0][i0][k0] - this->inputw[j0][i0+1][k0] - this->inputw[j0+1][i0][k0];
				 double c5 = this->inputw[j0+1][i0][k0+1] + this->inputw[j0][i0][k0] - this->inputw[j0][i0][k0+1] - this->inputw[j0+1][i0][k0];
				 double c6 = this->inputw[j0][i0+1][k0+1] + this->inputw[j0][i0][k0] - this->inputw[j0][i0+1][k0] - this->inputw[j0][i0][k0+1];
				 double c7 = this->inputw[j0+1][i0+1][k0+1] - this->inputw[j0+1][i0+1][k0] -this->inputw[j0+1][i0][k0+1] - this->inputw[j0][i0+1][k0+1] - this->inputw[j0][i0][k0] + this->inputw[j0][i0+1][k0] + this->inputw[j0+1][i0][k0] + this->inputw[j0][i0][k0+1];
			     F[2] = 0.25*(c0 +  c1*diffx + c2*diffy + c3*diffw + c4*diffx*diffy + c5*diffy*diffw + c6*diffw*diffx + c7*diffx*diffy*diffw);

	cout << F[2] << endl;

	if(isnan(F[2]))
	F[2] = 0.0;

	if (F[2] > 6.0){
	    F[2] = 6.0;}

	if (F[2] < -6.0){
	    F[2] = -6.0;}

	F[0] = 0.09;
	F[1] = 0.09;

	this->command_lvel = 0.1;
	this->command_avel = -F[2];

	//Wref[21] = {-0.1, -0.09, -0.08,-0.07,-0.06,-0.05,-0.04,-0.03,-0.02,-0.01,0.0,0.01,0.02,0.03,0.04,0.05,0.06,0.07,0.08,0.09,0.1};

	//Lpwm[21] = {106,103,99,96,92,89,86,84,81,84,89,94,98,103,108,108,106,104,101,99,97};

	//Rpwm[21] = {76,76,77,77,77,78,78,79,79,79,79,79,79,79,79,79,78,78,77,77,76};


	for (i = 0;i<(int)this->lpwm.size();i++)
	{
		if (F[2] > this->avel[i])
		{
		}
		else
		break;
	}
	//int commandL = ceil(Lpwm[i] + (F[2] - Wref[i])*(Lpwm[i+1] - Lpwm[i])/(Wref[i+1]-Wref[i]))
	//int commandR = ceil(Rpwm[i] + (F[2] - Wref[i])*(Rpwm[i+1] - Lpwm[i])/(Wref[i+1]-Wref[i]))

	this->commandL = this->lpwm[i];
	this->commandR = this->rpwm[i];



}



void stoc_controller::drive(){
	//Robot driver function

	ros::Rate loop_rate(50); // Send commands at 50 Hz
	geometry_msgs::Point command;

	while (ros::ok())
		{

			ros::spinOnce();

			interp3(this->localCurrentPose);

			command.x = this->commandL;
			command.y = this->commandR;
			command.z = 0.0;


		    if((pow(this->localCurrentPose.x,2) + pow(this->localCurrentPose.y,2)) < pow(this->epsilon,2))
			{
		    command.x = 0.0;
		    command.y = 0.0;
			}

		    if(pow(this->localCurrentPose.x,2) + pow(this->localCurrentPose.y,2) > pow(this->Region,2))
			{
		    command.x = 0.0;
		    command.y = 0.0;
			}

		    //Publish computed robot command
		    this->command_pub.publish(command);


		    loop_rate.sleep();

		  }

}
