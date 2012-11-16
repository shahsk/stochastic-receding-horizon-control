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
#include<math.h> 
#include <iostream> 
#include <fstream> 
#include <time.h> 
#include "mt19937ar.h" 
#include <vector>

//Define Constants
#define FALSE 0 
#define TRUE 1 
#define STEP 1000 
#define PATHS 500
#define VAR 3
#define PI 3.14159265358979323846
#define R 1.0
#define EPSILON 0.1

using namespace std; 
using std::vector;

typedef vector<double> Row;
typedef vector<Row> Matrix;

double randn_trig(Matrix&, long);
extern void init_genrand(unsigned long);
extern double genrand_real1(void);
extern double expdev(void), gasdev(void);

double eulermaruyamaprob(double *x0)
{
double dt = 0.05;
int nPeriods = STEP;
int nPaths = PATHS;
int i, k, j, t0 = 0;
double T;
double t[STEP+1];
T = dt*nPeriods;

int reached_goal = 0;
int reached_exit = 0;
double probability = 0.0;

int len=sizeof(x0)/sizeof(double);
long seedvalue; /* seed is long integer */
int maxStates = 2;
while(true){

Matrix X1ptr(2,Row(PATHS));
Matrix dW1ptr(nPeriods+1,Row(PATHS));
Matrix X2ptr(2,Row(PATHS));
Matrix dW2ptr(nPeriods+1,Row(PATHS));
Matrix X3ptr(2,Row(PATHS));
Matrix dW3ptr((nPeriods+1),Row(PATHS));
Matrix randomnumbers(nPeriods,Row(PATHS));


// Different independent Brownian motions
//Generate Brownian noise vector
seedvalue = (long)time(NULL);
randn_trig(randomnumbers, seedvalue+1);
for (i=0;i<nPeriods;i++){
	for (k=0;k<PATHS;k++){
		(dW1ptr[i][k]) = sqrt(dt)*((randomnumbers[i][k]));
	} 
} 
//Generate Brownian noise vector
seedvalue = (long)time(NULL);
randn_trig(randomnumbers, seedvalue+2);
for (i=0;i<nPeriods;i++){
	for (k=0;k<PATHS;k++){
		(dW2ptr[i][k]) = sqrt(dt)*((randomnumbers[i][k]));
	}
} 
//Generate Brownian noise vector
seedvalue = (long)time(NULL);
randn_trig(randomnumbers, seedvalue+3);
for (i=0;i<nPeriods;i++){
	for (k=0;k<PATHS;k++){
		(dW3ptr[i][k]) = sqrt(dt)*((randomnumbers[i][k]));
	}
} 


double F[3] = {0.0, 0.0, 0.0};
double G[3][3] = {{0.2, 0.0, 0.0},{0.0, 0.2, 0.0},{0.0, 0.0, 0.2}};

//dW = sqrt(dt)*randn(nPeriods, n, nPaths); % Brownian increments
// Simulate Stochastic System using Euler-Maruyama algorithm
//Assign Initial Conditions
	for (k=0;k<PATHS;k++)
	{
		(X1ptr[0][k]) = *x0;
		(X2ptr[0][k]) = *(x0+1);
		(X3ptr[0][k]) = *(x0+2);
	}

i = 1;
//Integrate Brownian Paths
//Stores only current state and next state of the path to reduce memory burden
for (k=0;k<PATHS;k++){
	for (j = 1; j<=nPeriods;j++){
		F[0] = G[0][0]*dW1ptr[j-1][k];
		F[1] = G[1][1]*dW2ptr[j-1][k];
		F[2] = G[2][2]*dW3ptr[j-1][k];
		//Integrate the next states
		(X1ptr[i][k]) = (X1ptr[i-1][k])+  2.0*cos(X3ptr[i-1][k] + PI/6.0)*F[0]/3.0 - 2.0*cos(X3ptr[i-1][k] - PI/6.0)*F[1]/3.0 + 2.0*sin(X3ptr[i-1][k])*F[2]/3.0;
		(X2ptr[i][k]) = (X2ptr[i-1][k])+  2.0*sin(X3ptr[i-1][k] + PI/6.0)*F[0]/3.0 - 2.0*sin(X3ptr[i-1][k] - PI/6.0)*F[1]/3.0 - 2.0*cos(X3ptr[i-1][k])*F[2]/3.0;
		(X3ptr[i][k]) = (X3ptr[i-1][k])+  1.0*(F[0] + F[1] + F[2])/0.3; // L = 0.1 length

		if(sqrt(pow(X1ptr[i][k],2)+pow(X2ptr[i][k],2)) >= R || sqrt(pow(X1ptr[i][k],2)+pow(X2ptr[i][k],2) + pow((cos(X3ptr[i][k]) - cos(0.0)),2) + pow((sin(X3ptr[i][k]) - sin(0.0)),2)) <= EPSILON ){
			break;
		}
		else{
			(X1ptr[i-1][k]) = (X1ptr[i][k]);
			(X2ptr[i-1][k]) = (X2ptr[i][k]);
			(X3ptr[i-1][k]) = (X3ptr[i][k]);
		}
	}
}

//Compute Probability
reached_exit = 0;
reached_goal = 0;

for (i = 0;i<PATHS;i++){
	for (j = 0; j<2;j++){
		if(sqrt(pow(X1ptr[j][i],2)+ pow(X2ptr[j][i],2)) >= R){
			reached_exit++;
			break;
		}
		else if(sqrt(pow(X1ptr[j][i],2)+ pow(X2ptr[j][i],2) + + pow((cos(X3ptr[j][i]) - cos(0.0)),2) + pow((sin(X3ptr[j][i]) - sin(0.0)),2)) <= EPSILON){
			reached_goal++;
			break;
		}
		else{
		}
	}
}	

//If 90% of the total path succeed or fail, the simulation ends or else
//it will continue with larger simulation time.
//If after 90% threshhold can not be reached, after 10 iterations give the best possible answer
if ((reached_goal + reached_exit) < PATHS*0.9){
	nPeriods = nPeriods + nPeriods;
	if (nPeriods > 10*STEP){
		return (double)reached_goal/((double)reached_exit+ (double)reached_goal);
	}
}
else{
	probability = (double)reached_goal/((double)reached_exit+ (double)reached_goal);
	return probability;
    //cout << "Probability = " << probability << endl;
}

// free allocated memory
X1ptr.clear();
X2ptr.clear();
X3ptr.clear();
dW1ptr.clear();
dW2ptr.clear();
dW3ptr.clear();
randomnumbers.clear();

} //End of while loop

}

// Convert uniform random numbers into Gaussian Random numbers "mu = 0.0 and sigma = 1.0" 
// Takes a pointer to the array of random numbers 


double randn_trig(Matrix& randarray, long seed)
 {
	double rand_1, rand_2;

	double mu=0.0, sigma=1.0;	
	static bool deviateAvailable=false;	//	flag
	static float storedDeviate;			//	deviate from previous calculation
	double dist, angle;
	int i,j,k;
	
	init_genrand(seed);

for (i=0;i<STEP;i++){
  for (k=0;k<PATHS;k++){
	
	rand_1 = genrand_real1();
	rand_2 = genrand_real1();

	if (!deviateAvailable) {
		
		dist=sqrt( -2.0 * log(rand_1));
		angle=2.0 * PI * (rand_2);
		
		//	calculate and store first deviate and set flag
		storedDeviate=dist*cos(angle);
		deviateAvailable=true;
		
		//	calcaulate return second deviate
		(randarray[i][k]) = dist * sin(angle) * sigma + mu;
		//printf("%f",*randarray);
	}
		else {
		deviateAvailable=false;
		(randarray[i][k]) = storedDeviate*sigma + mu;
	}
  }
}
return 0;
}

