/*
   EM Euler-Maruyama method to simulate an SDE
   Compute solution of the PDE using Feynman-Kac formula
   Parallel code to simulate SDEs and compute function g(q)
   Simulation for Omni-directional robot x, y , theta are the variables

   
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

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <omp.h>
#include <vector>

//Define constants

#define PI 3.14159265358979323846
#define R 1.0
#define EPSILON 0.1

using namespace std;
using std::vector;

typedef vector<double> Row;
typedef vector<Row> Matrix;
typedef vector<Matrix> Darray;
typedef vector<Darray> DxDarray;

double eulermaruyamaprob(double*);

int main()
{
double x, y, theta;
int i,j,k,l,Nx, Ny, Ntheta;
double xMax, yMax, thetaMax;

// Definition of grid size for all three variables
Nx = 41;
Ny = 41;
Ntheta = 41;
//Defines the boundary of variable, -Max to +Max value
xMax = 1.0;
yMax = 1.0;
thetaMax = PI;
//Compute grid distance based on size and Max values
double dx = (double)2.0*xMax/(Nx - 1.0);
double dy = (double)2.0*yMax/(Ny - 1.0);
double dtheta = (double)2.0*thetaMax/(Ntheta - 1.0);

double x0[3];

//Define 3D arrays to store grid values
Darray gridx(Nx, Matrix(Ny, Row(Ntheta)));
Darray gridy(Nx, Matrix(Ny, Row(Ntheta)));
Darray gridtheta(Nx, Matrix(Ny, Row(Ntheta)));
Darray prob(Nx, Matrix(Ny, Row(Ntheta)));

//To varify computation
//cout << "dx = " << dx << " dy = " << dy << " dtheta = " << dtheta << endl;

// Create a grid of data points
for (i=0;i<Nx;i++){
  for (j = 0;j<Ny;j++){
    for (k = 0;k<Ntheta;k++){
      gridx[i][j][k] = (-xMax + i*dx);
      gridy[i][j][k] = (-yMax + j*dy);
      gridtheta[i][j][k] = (-thetaMax + k*dtheta);
      prob[i][j][k] = 0.0;
    }
  }
}
//Compute loop for each data point
#define CHUNKSIZE 1 //defines the chunk size as 1 contiguous iteration
 //Starts the threads
#pragma omp parallel private(j,k,x0) //non-shared variables between threads
{
//Starts the work sharing construct
#pragma omp for schedule(dynamic, CHUNKSIZE)
  for (i=0;i<Nx;i++){
    for (j = 0;j<Ny;j++){
      for (k = 0;k<Ntheta;k++){
    	  if(sqrt(pow(gridx[i][j][k],2)+ pow(gridy[i][j][k],2) ) >= R){
    		  prob[i][j][k] = 0.0;
    	  }
    	  else if(sqrt(pow(gridx[i][j][k],2)+ pow(gridy[i][j][k],2) + pow((cos(gridtheta[i][j][k]) - cos(0.0)),2) + pow((sin(gridtheta[i][j][k]) - sin(0.0)),2) ) <= EPSILON){
    		  prob[i][j][k] = 1.0;
    	  }
    	  else{
    		  x0[0] = gridx[i][j][k];
    		  x0[1] = gridy[i][j][k];
    		  x0[2] = gridtheta[i][j][k];
    		  prob[i][j][k] = eulermaruyamaprob(x0);
    		  cout << "(" << x0[0] << ", " << x0[1] << ", " << x0[2] << ")" << endl;
    		  cout << "g(q) = " << prob[i][j][k] << endl;
    	  }
      	  } // End of k loop
    	} // End of j loop
  	  }// End of i loop
}//End of parallel loop

//Print output in MATLAB compatible (meshgrid/ndgrid/reshape) format
ofstream myfile;
myfile.open("Output");
for (k=0;k<Ntheta;k++){
  for (j=0;j<Ny;j++){
    for (i=0;i<Nx;i++){
      myfile << prob[i][j][k] << endl;
    }
  }
}
myfile.close();
return 0;
}




