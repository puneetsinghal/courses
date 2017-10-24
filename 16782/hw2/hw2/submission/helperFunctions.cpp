#include <math.h>
#include "mex.h"
#include "helperFunctions.hpp"

double findMinorArc(double difference)
{
  if (difference > PI)
    return (difference - 2*PI);
  else if (difference < -PI)
    return (2*PI + difference);
  else 
    return difference;
}

double distance(double* startConfig, double* endConfig, int numofDOFs)
{
	double dist = 0.0;
	int i = 0;
	for (i = 0; i < numofDOFs; ++i)
	{
		dist += pow(findMinorArc((endConfig[i] - startConfig[i])),2);
	}
	dist = pow(dist,0.5);
  // mexPrintf("dis is %g \n", dist);
	return dist;
}

void print_joints(double * theta, int numofDOFs)
{
  int i = 0;
  for (i = 0; i < numofDOFs; ++i)
  {
    mexPrintf("%g    ",theta[i]);
  }
  mexPrintf("\n");
}