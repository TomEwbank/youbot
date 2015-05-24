/* Updates a map based on the data gathered by Hokuyo sensors of a robot
 * (see file "ptsToCellmap.m" for more details about input and output) */
#include <math.h>
#include "mex.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	#define X_IN prhs[0]
	#define Y_IN prhs[1]
	#define XC_IN prhs[2]
	#define YC_IN prhs[3]
	#define MAP_IN prhs[4]
	#define S_IN prhs[5]
	#define P_IN prhs[6]
	#define D_IN prhs[7]

	#define MAP_OUT plhs[0]
    
	double cell_size = mxGetScalar(S_IN);
	double d = mxGetScalar(D_IN);
    
	double *x, *y, *pose, *new_map;
	
    int L = mxGetNumberOfElements(X_IN);
	int M = mxGetM(MAP_IN);
    int N = mxGetN(MAP_IN);
    int i, m, n;
    
    double x_ref;
	double y_ref;
	double theta;

	x = mxGetPr(X_IN);
	y = mxGetPr(Y_IN);
    MAP_OUT = mxDuplicateArray(MAP_IN);
    new_map = mxGetPr(MAP_OUT);
	pose = mxGetPr(P_IN);
    x_ref = pose[0];
	y_ref = pose[1];
	theta = pose[2];


	/* adding explored cells to the map */
	for(i = 0; i<L; ++i)
	{
		m = floor((cos(theta)*x[i]-sin(theta)*y[i]+x_ref+d)/cell_size);
		n = floor((sin(theta)*x[i]+cos(theta)*y[i]+y_ref+d)/cell_size);
        /* Do not mark as explored a point that has already been
           identified as an obstacle */
        if (m >= 0 && n >=0 && m < M && n < N && new_map[m + M*n] != 1)
            new_map[m + M*n] = 0;
	}

	L = mxGetNumberOfElements(XC_IN);
	x = mxGetPr(XC_IN);
	y = mxGetPr(YC_IN);

	/* adding obstacles to the map */
	for(i=0; i<L; ++i)
	{
		m = floor((cos(theta)*x[i]-sin(theta)*y[i]+x_ref+d)/cell_size);
		n = floor((sin(theta)*x[i]+cos(theta)*y[i]+y_ref+d)/cell_size);
		if (m >= 0 && n >=0 && m < M && n < N)
            new_map[m + M*n] = 1;
	}

	return;
}