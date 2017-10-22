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
		dist += pow(findMinorArc((newSample[i] - it->theta[i])),2);
	}
	dist = pow(dist,0.5);
	return dist;
}
void print_vertex(vertex currentNode, int numofDOFs)
{
  int i = 0;
  mexPrintf("vertex is: \n");
  for (i = 0; i < numofDOFs; ++i)
  {
    mexPrintf("%g    ",currentNode.theta[i]);
  }
  mexPrintf("\n ID is: %d\n \n", currentNode.vertexID);
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