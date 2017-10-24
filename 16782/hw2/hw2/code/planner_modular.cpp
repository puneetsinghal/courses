/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include <math.h>
#include "mex.h"
#include <stdlib.h>     /* srand, rand */
#include <list>
#include <queue>
#include <random>
#include <ctime>
#include <chrono>
#include "helperFunctions.hpp"
#include "tree.hpp"
#include "graph.hpp"

/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ARMSTART_IN	prhs[1]
#define	ARMGOAL_IN     prhs[2]
#define PLANNER_ID_IN     prhs[3]

/* Planner Ids */
#define RRT         0
#define RRTCONNECT  1
#define RRTSTAR     2
#define PRM         3


/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]
#define NUMSAMPLES_OUT  plhs[2]


#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

//the length of each link in the arm (should be the same as the one used in runtest.m)
#define LINKLENGTH_CELLS 10

typedef struct {
  int X1, Y1;
  int X2, Y2;
  int Increment;
  int UsingYIndex;
  int DeltaX, DeltaY;
  int DTerm;
  int IncrE, IncrNE;
  int XIndex, YIndex;
  int Flipped;
} bresenham_param_t;

void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size)
{
    double cellsize = 1.0;
	//take the nearest cell
	*pX = (int)(x/(double)(cellsize));
	if( x < 0) *pX = 0;
	if( *pX >= x_size) *pX = x_size-1;

	*pY = (int)(y/(double)(cellsize));
	if( y < 0) *pY = 0;
	if( *pY >= y_size) *pY = y_size-1;
}

void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params)
{
  params->UsingYIndex = 0;

  if (fabs((double)(p2y-p1y)/(double)(p2x-p1x)) > 1)
    (params->UsingYIndex)++;

  if (params->UsingYIndex)
    {
      params->Y1=p1x;
      params->X1=p1y;
      params->Y2=p2x;
      params->X2=p2y;
    }
  else
    {
      params->X1=p1x;
      params->Y1=p1y;
      params->X2=p2x;
      params->Y2=p2y;
    }

   if ((p2x - p1x) * (p2y - p1y) < 0)
    {
      params->Flipped = 1;
      params->Y1 = -params->Y1;
      params->Y2 = -params->Y2;
    }
  else
    params->Flipped = 0;

  if (params->X2 > params->X1)
    params->Increment = 1;
  else
    params->Increment = -1;

  params->DeltaX=params->X2-params->X1;
  params->DeltaY=params->Y2-params->Y1;

  params->IncrE=2*params->DeltaY*params->Increment;
  params->IncrNE=2*(params->DeltaY-params->DeltaX)*params->Increment;
  params->DTerm=(2*params->DeltaY-params->DeltaX)*params->Increment;

  params->XIndex = params->X1;
  params->YIndex = params->Y1;
}

void get_current_point(bresenham_param_t *params, int *x, int *y)
{
  if (params->UsingYIndex)
    {
      *y = params->XIndex;
      *x = params->YIndex;
      if (params->Flipped)
        *x = -*x;
    }
  else
    {
      *x = params->XIndex;
      *y = params->YIndex;
      if (params->Flipped)
        *y = -*y;
    }
}

int get_next_point(bresenham_param_t *params)
{
  if (params->XIndex == params->X2)
    {
      return 0;
    }
  params->XIndex += params->Increment;
  if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
    params->DTerm += params->IncrE;
  else
    {
      params->DTerm += params->IncrNE;
      params->YIndex += params->Increment;
    }
  return 1;
}

int IsValidLineSegment(double x0, double y0, double x1, double y1, double*	map,
		   int x_size,
 		   int y_size)
{
	bresenham_param_t params;
	int nX, nY; 
    short unsigned int nX0, nY0, nX1, nY1;

    //mexPrintf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);
    
	//make sure the line segment is inside the environment
	if(x0 < 0 || x0 >= x_size ||
		x1 < 0 || x1 >= x_size ||
		y0 < 0 || y0 >= y_size ||
		y1 < 0 || y1 >= y_size)
		return 0;

	ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
	ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

    //mexPrintf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

	//iterate through the points on the segment
	get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
	do {
		get_current_point(&params, &nX, &nY);
		if(map[GETMAPINDEX(nX,nY,x_size,y_size)] == 1)
            return 0;
	} while (get_next_point(&params));

	return 1;
}

int IsValidArmConfiguration(double* angles, int numofDOFs, double*	map,
		   int x_size, int y_size)
{
  double x0,y0,x1,y1;
  int i;
    
 	//iterate through all the links starting with the base
	x1 = ((double)x_size)/2.0;
    y1 = 0;
	for(i = 0; i < numofDOFs; i++)
	{
		//compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-angles[i]);
		y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-angles[i]);

		//check the validity of the corresponding line segment
		if(!IsValidLineSegment(x0,y0,x1,y1,map,x_size,y_size))
				return 0;
	}    
}

bool findNextStep(double* nearestVertexAngles, double* newSample, double* nextConfig, 
        int numofDOFs, double*  map, int x_size, int y_size, bool* reached)
{
  double epsilonStep = 0.2;
  int i = 0;
  double norm = 0.0;
  double normalizedDifference[numofDOFs];
  double nextAngle;

  for (i = 0; i < numofDOFs; ++i)
  {
    normalizedDifference[i] = findMinorArc((newSample[i] - nearestVertexAngles[i]));
    norm += pow(normalizedDifference[i],2);    
  }
  norm = pow(norm, 0.5);
  // mexPrintf("norm value is: %g \n", norm);
  // mexPrintf("normalized difference is:  ");
  // print_joints(normalizedDifference, numofDOFs);

  if(norm>epsilonStep)
  {
    for (i = 0; i < numofDOFs; ++i)
    {
      nextAngle = nearestVertexAngles[i] + epsilonStep*(normalizedDifference[i])/norm;
      // mexPrintf("Next angle test = %g  ", nextAngle);
      if (nextAngle < 0)
        nextConfig[i] = 2*PI + nextAngle;
      else if (nextAngle > 2*PI)
        nextConfig[i] = nextAngle - 2*PI;
      else
        nextConfig[i] = nextAngle;
    }
    *reached = false;
  }
  else
  {
    for (i = 0; i < numofDOFs; ++i)
      nextConfig[i] = newSample[i];
    *reached = true;
  }
  // mexPrintf("temp next config is:  ");
  // print_joints(nextConfig, numofDOFs);

  if(IsValidArmConfiguration(nextConfig, numofDOFs, map, x_size, y_size) == 0)
    return false;

  return true;
}

int extend(double* map,
       int x_size,
       int y_size,
       tree *rrtTree,
       double *newSample, 
       int numofDOFs)
{
  int IDNUMBER = (rrtTree->vertices).size();
  vertex newVertex;
  vertex *nearestVertex;
  double nextConfig[numofDOFs];
  int i = 0;
  bool reached = false;

  // find closet vertex: nearestVertex
  rrtTree->findNearestVertex(newSample, &nearestVertex, numofDOFs);

  // find the next configuration by taking small epsilon step
  // return true if no collision happens
  if(!findNextStep(nearestVertex->theta, newSample, nextConfig, numofDOFs, map, x_size, y_size, &reached))
    return 0;

  IDNUMBER++;
  // mexPrintf("ID number is %d \n", IDNUMBER);

  for (i = 0; i < numofDOFs; ++i)
    newVertex.theta[i] = nextConfig[i];
  newVertex.vertexID = IDNUMBER;
  newVertex.parentID = nearestVertex->vertexID;
  // rrtTree->print_vertex(&newVertex, numofDOFs);
  rrtTree->vertices.push_back(newVertex);

  if (reached)
    return 2;
  return 1;
}

void rrtImplementation (double* map,
       int x_size,
       int y_size,
       double* armstart_anglesV_rad,
       double* armgoal_anglesV_rad,
       int numofDOFs,
       double*** plan,
       int* planlength,
       int* numSamples)
{
  unsigned startSeed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(startSeed);
  std::uniform_real_distribution<double> distribution(0.0,1.0);

  double newSample[numofDOFs];
  int i = 0, j=0;

  // std::list<vertex> vertices;
  tree rrtTree;
  int goalReached = 1; // 0 is trapped, 1 is advance, 2 is Reached
  bool goalIsSample = false;
  int iterations = 0;
  int maxIterations = 40000;

  // add starting configuration to vertices list
  vertex startVertex;
  for (i = 0; i < numofDOFs; ++i)
    startVertex.theta[i] = armstart_anglesV_rad[i];
  startVertex.vertexID = 1; 
  rrtTree.vertices.push_back(startVertex);
  // rrtTree.print_vertex(startVertex, numofDOFs);
  while(iterations < maxIterations)
  {
    // check sampling bias
    if (distribution(generator) > 0.1)
    {
      for (i = 0; i < numofDOFs; ++i)
        newSample[i] = 2*PI*(distribution(generator));
      goalIsSample = false;
    }
    else
    {
      for (i = 0; i < numofDOFs; ++i)
        newSample[i] = armgoal_anglesV_rad[i];
      goalIsSample = true;
    }

    goalReached = extend(map, x_size, y_size, &rrtTree, newSample, numofDOFs);
    if (goalReached==2 && goalIsSample)
    {
      break;
    }
    iterations++;
  }

  // check if iterations are over
  *numSamples = iterations;
  if (iterations >= maxIterations)
  {
    mexPrintf("no solution found. Tried for 40000 iterations");
    return;
  }
  
  std::list<int> pathVertices;
  double nextConfig[numofDOFs];
  int pathVertex;
  
  rrtTree.getPathFromTree(&pathVertices, rrtTree.vertices.size());
  *planlength = pathVertices.size();
  
  *plan = (double**) malloc((*planlength) *sizeof(double*));
  i = 0;
  for (std::list<int>::iterator it= pathVertices.end(); it != pathVertices.begin(); --it)
  {
    (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
    rrtTree.findConfigurationForVertex(*it, nextConfig, numofDOFs);
    for(j = 0; j < numofDOFs; j++)
    {
      (*plan)[i][j] = nextConfig[j];
    }
    i++;
  }
}

void swap(tree * rrtTreeA,
       tree * rrtTreeB)
{
  tree * rrtTreeTemp; 
  rrtTreeTemp = rrtTreeA;
  rrtTreeA = rrtTreeB;
  rrtTreeB = rrtTreeTemp;
  return;
}

int connect(double* map,
       int x_size,
       int y_size,
       tree *rrtTree,
       double *newSample, 
       int numofDOFs)
{
  int state = 1;
  while(state==1)
  {
    state = extend(map, x_size, y_size, rrtTree, newSample, numofDOFs);
  }
  return state;
}

void rrtConnectImplementation (double* map,
       int x_size,
       int y_size,
       double* armstart_anglesV_rad,
       double* armgoal_anglesV_rad,
       int numofDOFs,
       double*** plan,
       int* planlength,
       int *numSamples)
{
  unsigned startSeed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(startSeed);
  std::uniform_real_distribution<double> distribution(0.0,1.0);

  double newSample[numofDOFs];
  int i = 0, j=0;

  // std::list<vertex> verticesA, verticesB;
  tree rrtTreeA, rrtTreeB;

  int iterations = 0;
  int maxIterations = 10000;

  // add starting configuration to vertices list
  vertex startVertex;
  for (i = 0; i < numofDOFs; ++i)
    startVertex.theta[i] = armstart_anglesV_rad[i];
  startVertex.vertexID = 1; 
  startVertex.parentID = 0; 

  rrtTreeA.vertices.push_back(startVertex);

  for (i = 0; i < numofDOFs; ++i)
    startVertex.theta[i] = armgoal_anglesV_rad[i];
  startVertex.vertexID = 1; 
  startVertex.parentID = 0; 
  rrtTreeB.vertices.push_back(startVertex);

  bool completed = false;
  while(iterations < maxIterations && !completed)
  {
    // check sampling bias
    for (i = 0; i < numofDOFs; ++i)
      newSample[i] = 2*PI*(distribution(generator));

    if(extend(map, x_size, y_size, &rrtTreeA, newSample, numofDOFs)!=0)
    {
      for (i = 0; i < numofDOFs; ++i)
        newSample[i] = rrtTreeA.vertices.back().theta[i];

      // mexPrintf("q_new before calling connect:  ");
      // print_joints(newSample, numofDOFs);
      int state = connect(map, x_size, y_size, &rrtTreeB, newSample, numofDOFs);
      if(state == 2)
      { 
        // mexPrintf("completed with state: %d \n", state);
        completed = true;
        continue;
      }
    }
    swap(&rrtTreeA, &rrtTreeB);
    iterations++;
  }
  *numSamples = iterations;
  // check if iterations are over
  if (iterations >= maxIterations)
  {
    mexPrintf("no solution found. Tried for 40000 iterations");
    return;
  }
  
  std::list<int> pathVerticesA;
  std::list<int> pathVerticesB;
  double nextConfig[numofDOFs];
  int pathVertex;

  int checkAForwardTree = 0;
  for (i = 0; i < numofDOFs; ++i)
  {
    if(fabs(rrtTreeA.vertices.front().theta[i] - armstart_anglesV_rad[i])<0.0001)
      checkAForwardTree++;
  } 
  if(checkAForwardTree != 5)
  {
    swap(&rrtTreeA, &rrtTreeB);
  }
  rrtTreeA.getPathFromTree(&pathVerticesA, rrtTreeA.vertices.size());
  rrtTreeB.getPathFromTree(&pathVerticesB, rrtTreeB.vertices.size());

  *planlength = pathVerticesA.size() + pathVerticesB.size();

  *plan = (double**) malloc((*planlength) *sizeof(double*));
  i = 0;
  for (std::list<int>::iterator it= pathVerticesA.end(); it != pathVerticesA.begin(); --it)
  {
    (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
    rrtTreeA.findConfigurationForVertex(*it, nextConfig, numofDOFs);
    for(j = 0; j < numofDOFs; j++)
    {
      (*plan)[i][j] = nextConfig[j];
    }
    i++;
  }

  for (std::list<int>::iterator it= pathVerticesB.begin(); it != pathVerticesB.end(); ++it)
  {
    (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double));
    rrtTreeB.findConfigurationForVertex(*it, nextConfig, numofDOFs);
    for(j = 0; j < numofDOFs; j++)
    {
      (*plan)[i][j] = nextConfig[j];
    }
    i++;
  }
  return;
}

bool obstacleFree(double* map,
       int x_size,
       int y_size,
       double *startConfig,
       double *endConfig,
       int numofDOFs)
{
  double stepSize = 0.01;
  int steps;
  int i, j;
  double nextConfig[numofDOFs];
  double nextAngle; 
  double normalizedDifference[numofDOFs];
  double dist = 0.0;
  for (i = 0; i < numofDOFs; ++i)
  {
    normalizedDifference[i] = findMinorArc((endConfig[i] - startConfig[i]));
    dist += pow(normalizedDifference[i],2);    
  }
  dist = pow(dist, 0.5);
  steps = ceil(dist/stepSize);

  for (i = 0; i < steps; ++i)
  {
    for (j = 0; j < numofDOFs; ++j)
    {
      nextAngle = startConfig[j] + stepSize*(i)*normalizedDifference[j]/dist;
      if (nextAngle < 0)
        nextConfig[j] = 2*PI + nextAngle;
      else if (nextAngle > 2*PI)
        nextConfig[j] = nextAngle - 2*PI;
      else
        nextConfig[j] = nextAngle;
    }
    if(!IsValidArmConfiguration(nextConfig, numofDOFs, map, x_size, y_size))
      return false;
  }
  return true;
}

void changeCostOfChildren(vertex **currentNode, int numofDOFs)
{
  double cost = 0.0;
  // vertex node = *currentNode;
  cost = (*currentNode)->cost;
  for(std::list<vertex*>::iterator it= (*currentNode)->children.begin(); it != (*currentNode)->children.end(); ++it)
  {
    (*it)->cost = cost + distance((*currentNode)->theta, (*it)->theta, numofDOFs);
    changeCostOfChildren(&(*it), numofDOFs);
  }
  return;
}

int extendRRTStar(double* map,
       int x_size,
       int y_size,
       tree *rrtTree,
       double *newSample, 
       int numofDOFs)
{
  int IDNUMBER = (rrtTree->vertices).size();
  vertex newVertex;
  vertex *nearestVertex;
  double nextConfig[numofDOFs];
  // double nearestVertexAngles[numofDOFs];
  // int nearestVertexID; 
  int i = 0;
  bool reached = false;

  // find closet vertex: nearestVertex
  rrtTree->findNearestVertex(newSample, &nearestVertex, numofDOFs);

  // find the next configuration by taking small epsilon step
  // return true if no collision happens
  if(!findNextStep(nearestVertex->theta, newSample, nextConfig, numofDOFs, map, x_size, y_size, &reached))
    return 0;

  IDNUMBER++;
  // mexPrintf("ID number is %d \n", IDNUMBER);
  // print_joints(nextConfig, numofDOFs);
  for (i = 0; i < numofDOFs; ++i)
    newVertex.theta[i] = nextConfig[i];

  // rrtTree->print_vertex(&newVertex, numofDOFs);
  newVertex.vertexID = IDNUMBER;
  newVertex.parentID = nearestVertex->vertexID;
  newVertex.parentAddress = nearestVertex;
  newVertex.cost = nearestVertex->cost + distance(newVertex.theta, nearestVertex->theta, numofDOFs);
  // rrtTree->print_vertex(&newVertex, numofDOFs);
  (rrtTree->vertices).push_back(newVertex);
  nearestVertex->children.push_back(&(rrtTree->vertices.back()));

  vertex * minVertex;
  minVertex = nearestVertex;
  std::list<vertex*> neighborHood;
  rrtTree->createNeighborhood(&newVertex, &neighborHood, numofDOFs);  
  
  int numNeighbors = neighborHood.size();
  double newCost = 0.0;
  vertex *nearVertexTemp;
  for (std::list<vertex*>::iterator it= neighborHood.begin(); it != neighborHood.end(); ++it)
  {
    nearVertexTemp = *it;
    if(obstacleFree(map, x_size, y_size, nearVertexTemp->theta, newVertex.theta, numofDOFs))
    {
      newCost = nearVertexTemp->cost + distance(nearVertexTemp->theta, newVertex.theta, numofDOFs);
      if(newCost < newVertex.cost)
      {
        minVertex = *it;
      }
    }
  }
  // add the edge from nearestVertex to IDNUMBER
  nearestVertex->children.remove(&(rrtTree->vertices.back()));
  rrtTree->addEdge(minVertex, numofDOFs);

  vertex *newVertexPointer;

  newVertexPointer = &(rrtTree->vertices.back());
  // remove the minVertex from neighborHood
  // int parent;
  for (std::list<vertex*>::iterator it= neighborHood.begin(); it != neighborHood.end(); ++it)
  {
    nearVertexTemp = *it;
    if(nearVertexTemp!=minVertex)
    {
      newCost = newVertexPointer->cost + distance(newVertexPointer->theta, nearVertexTemp->theta, numofDOFs);
      if(obstacleFree(map, x_size, y_size, newVertexPointer->theta, nearVertexTemp->theta, numofDOFs) 
        && ((nearVertexTemp->cost) > newCost))
      {
        // parent = nearVertexTemp->parentID;
        // change parent to newVertexPointer
        nearVertexTemp->parentAddress->children.remove(nearVertexTemp); // remove nearVertexTemp from children list of it parent
        nearVertexTemp->parentID = newVertexPointer->vertexID; // change parent ID of nearVertexTemp
        nearVertexTemp->parentAddress = newVertexPointer; // change parent address of nearVertexTemp

        // add list of children to newVertexPointer
        newVertexPointer->children.push_back(nearVertexTemp);

        // change cost of all children
        changeCostOfChildren(&newVertexPointer, numofDOFs);
      }
    }
  }

  if (reached)
    return 2;
  return 1;
}

void rrtStar(double* map,
       int x_size,
       int y_size,
       double* armstart_anglesV_rad,
       double* armgoal_anglesV_rad,
       int numofDOFs,
       double*** plan,
       int* planlength,
       int *numSamples)
{
  unsigned startSeed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(startSeed);
  std::uniform_real_distribution<double> distribution(0.0,1.0);

  double newSample[numofDOFs];
  int i = 0, j=0;

  tree rrtTree;

  int goalReached = 1; // 0 is trapped, 1 is advance, 2 is Reached
  bool goalIsSample = false;
  int iterations = 0;
  int maxIterations = 40000;
  int goalID = 0;
  // add starting configuration to vertices list
  vertex startVertex;
  for (i = 0; i < numofDOFs; ++i)
    startVertex.theta[i] = armstart_anglesV_rad[i];
  startVertex.vertexID = 1; 
  startVertex.cost = 0.0;
  startVertex.parentID = 0;
  startVertex.parentAddress = NULL;
  rrtTree.vertices.push_back(startVertex);
  rrtTree.print_vertex(&startVertex, numofDOFs);
  bool samplingGoal = true; 
  int changingMaxIterations = maxIterations;

  while(iterations < changingMaxIterations)
  {
    // check sampling bias
    if (distribution(generator) <0.1 && samplingGoal)
    {
      for (i = 0; i < numofDOFs; ++i)
        newSample[i] = armgoal_anglesV_rad[i];
      goalIsSample = true;
    }
    else
    {
      for (i = 0; i < numofDOFs; ++i)
        newSample[i] = 2*PI*(distribution(generator));
      goalIsSample = false;
    }

    goalReached = extendRRTStar(map, x_size, y_size, &rrtTree, newSample, numofDOFs);
    if (goalReached==2 && goalIsSample && samplingGoal)
    {
      // mexPrintf("The goal cost is %g \n", (rrtTree.vertices.back()).cost);
      samplingGoal = false;
      *numSamples = iterations;
      iterations = 0;
      changingMaxIterations = 15000;
      goalID = rrtTree.vertices.size();
    }
    iterations++;
    // check if iterations are over
    if (iterations >= maxIterations & changingMaxIterations == maxIterations)
    {
      // mexPrintf("no solution found. Tried for %d iterations\n", maxIterations);
      // mexPrintf("Number of vertices in tree = %d \n", rrtTree.vertices.size());
      // mexPrintf("Trying for 20000 more iterations \n");
      changingMaxIterations = maxIterations + 20000;
    }
  }
  // mexPrintf("Number of vertices in tree = %d \n", rrtTree.vertices.size());
  if (iterations >= maxIterations)
  {
    mexPrintf("no solution found. Tried for %d iterations\n", MAX(maxIterations,changingMaxIterations));
    return;
  }
  std::list<int> pathVertices;
  double nextConfig[numofDOFs];
  int pathVertex;

  for (std::list<vertex>::iterator it= rrtTree.vertices.begin(); it != rrtTree.vertices.end(); ++it)
  {    
    if (it->vertexID == goalID)
    { 
      // mexPrintf("New goal cost is %g \n", it->cost);
      break;
    }
  }
  
  rrtTree.getPathFromTree(&pathVertices, goalID);
  *planlength = pathVertices.size();
  // mexPrintf("length of path: %d \n", *planlength);
  *plan = (double**) malloc((*planlength) *sizeof(double*));
  i = 0;
  for (std::list<int>::iterator it= pathVertices.end(); it != pathVertices.begin(); --it)
  {
    (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
    rrtTree.findConfigurationForVertex(*it, nextConfig, numofDOFs);
    for(j = 0; j < numofDOFs; j++)
    {
      (*plan)[i][j] = nextConfig[j];
    }
    i++;
  }
  return;
}

void prmRoadmap(double* map,
       int x_size,
       int y_size,
       int numofDOFs,
       graph *roadmap,
       int *numSamples)
{
  unsigned startSeed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(startSeed);
  std::uniform_real_distribution<double> distribution(0.0,1.0);

  double newSample[numofDOFs];
  int itr = 0, i = 0, j=0;

  int maxIterations = 30000;
  *numSamples = maxIterations;
  int iterations = 0;

  int IDNUMBER = 1;

  graphVertex *newVertex;
  double radius = 0.8;
  std::list<graphVertex*> neighborHood;
  int targetDensity = 20;

  while (iterations < maxIterations)
  {
    neighborHood.clear();

    for (i = 0; i < numofDOFs; ++i)
      newSample[i] = 2*PI*(distribution(generator));
    // }

    if(!IsValidArmConfiguration(newSample, numofDOFs, map, x_size, y_size))
      continue;

    // print_joints(newSample, numofDOFs);

    roadmap->addVertexGraph(newSample, IDNUMBER, numofDOFs);
    newVertex = &((roadmap->gVertices).back());
    // mexPrintf("address is: %p \n", newVertex);

    iterations++;
    IDNUMBER++;
    // roadmap->print_vertex(newVertex, numofDOFs);
    roadmap->createNeighborhoodGraph(&newVertex, &neighborHood, numofDOFs, radius);
    if(neighborHood.size() == 0)
    { 
      // mexPrintf("no neighborhood\n");
      continue;
    }
    // mexPrintf("neighborhood size is: %d \n", neighborHood.size());
    for (std::list<graphVertex*>::iterator it= neighborHood.begin(); it != neighborHood.end(); ++it)
    {
      if((*it)->numConnected < targetDensity )
      {
        // roadmap->print_vertex(newVertex, numofDOFs);
        // roadmap->print_vertex(*it, numofDOFs);
        if(obstacleFree(map, x_size, y_size, newVertex->theta, (*it)->theta, numofDOFs))
        {
          roadmap->addEdgeGraph(newVertex, (*it), numofDOFs);
          // roadmap->printLastEdge(numofDOFs);
        }
      }
    }
  }
  // mexPrintf("total edges = %d\n", (roadmap->gEdges).size());
  return;
}

void dijkstra(double*  map,
       int x_size,
       int y_size,
       double* armstart_anglesV_rad,
       double* armgoal_anglesV_rad,
       int numofDOFs,
       double*** plan,
       int* planlength,
       graph *roadmap)
{

  std::priority_queue<graphVertex*> openList;

  int i, j, k;

  double radius = 0.8;
  int startID = (roadmap->gVertices).size() + 1;
  int endID = startID + 1;

  roadmap->addVertexGraph(armstart_anglesV_rad, startID, numofDOFs);
  graphVertex *startNode = &(roadmap->gVertices).back();
  startNode->gValue = 0;

  roadmap->addVertexGraph(armgoal_anglesV_rad, endID, numofDOFs);
  graphVertex *goalNode = &(roadmap->gVertices).back();

  for(std::list<graphVertex>::iterator it= (roadmap->gVertices).begin(); it != (roadmap->gVertices).end(); ++it)
  {
    if(distance(armstart_anglesV_rad, it->theta, numofDOFs) < radius)
    {
      if(obstacleFree(map, x_size, y_size, armstart_anglesV_rad, it->theta, numofDOFs))
      {
        roadmap->addEdgeGraph(startNode, &(*it), numofDOFs);
      }
    }
    if(distance(armgoal_anglesV_rad, it->theta, numofDOFs) < radius)
    {
      if(obstacleFree(map, x_size, y_size, armgoal_anglesV_rad, it->theta, numofDOFs))
      {
        roadmap->addEdgeGraph(goalNode, &(*it), numofDOFs);
      }
    }
  }

  openList.push(startNode);

  graphVertex* currentNode;
  graphVertex* connectingNode;

  double updatedCost = 0;
  
  while (!openList.empty())
  {
    currentNode = openList.top();
    currentNode->isClosed = true;
    openList.pop();

    // roadmap->print_vertex(currentNode, numofDOFs);
    if ((currentNode->connections).size()==0)
    {
      continue;
    }
    for (std::list<graphEdge*>::iterator it= (currentNode->connections).begin(); it != (currentNode->connections).end(); ++it)
    {
      if(currentNode == (*it)->startAddress)
        connectingNode = (*it)->endAddress;
      else
        connectingNode = (*it)->startAddress;

      updatedCost = currentNode->gValue + (*it)->cost;

      if(!connectingNode->isClosed)
      {
        if( updatedCost < connectingNode->gValue)
        {
          connectingNode->gValue = updatedCost;
          connectingNode->parentAddress = currentNode;
          openList.push(connectingNode); 
        }
      }
    }
    if(currentNode==goalNode)
      break;
    // currentNode = openList.top();
    // currentNode->isClosed = true;
  }

  std::list<graphVertex*> pathVertices;
  pathVertices.push_back(goalNode);
  graphVertex *predecessor = (pathVertices.back())->parentAddress;

  while(predecessor != startNode)
  {
    pathVertices.push_back(predecessor);
    predecessor = (pathVertices.back())->parentAddress;
  }
  pathVertices.push_back(predecessor);

  *planlength = pathVertices.size();

  *plan = (double**) malloc((*planlength) *sizeof(double*));
  i = 0;
  for (std::list<graphVertex*>::iterator it = pathVertices.end(); it != pathVertices.begin(); --it)
  {
    (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
    for(j = 0; j < numofDOFs; j++)
    {
      (*plan)[i][j] = (*it)->theta[j];
    }
    i++;
  }
  return;
}
static void planner(
		   double*	map,
		   int x_size,
 		   int y_size,
       double* armstart_anglesV_rad,
       double* armgoal_anglesV_rad,
  	   int numofDOFs,
  	   double*** plan,
  	   int* planlength, 
       int planner_id,
       int *numSamples)
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;
  *numSamples = 0;
    
    //for now just do straight interpolation between start and goal checking for the validity of samples
    //but YOU  WILL WANT TO REPLACE THE CODE BELOW WITH YOUR PLANNER
    
    //RRT
    //vector of 
    //sample random
    // mexPrintf("started planning \n");
    if (planner_id ==0)
      rrtImplementation(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength, numSamples);
    else if (planner_id ==1)
      rrtConnectImplementation(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength, numSamples);
    else if (planner_id==2)
      rrtStar(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength, numSamples);
    else if (planner_id==3)
    {
      graph roadmap;
      prmRoadmap(map, x_size, y_size, numofDOFs, &roadmap, numSamples);
      dijkstra(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength, &roadmap);
    }
    //nearest neighbor
    
    //
    
    
    // double distance = 0;
    // int i,j;
    // for (j = 0; j < numofDOFs; j++){
    //     if(distance < fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]))
    //         distance = fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]);
    // }
    // int numofsamples = (int)(distance/(PI/20));
    // if(numofsamples < 2){
    //     mexPrintf("the arm is already at the goal\n");
    //     return;
    // }
    // *plan = (double**) malloc(numofsamples*sizeof(double*));

    // int firstinvalidconf = 1;
    // for (i = 0; i < numofsamples; i++){
    //     (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
    //     for(j = 0; j < numofDOFs; j++){
    //         (*plan)[i][j] = armstart_anglesV_rad[j] + ((double)(i)/(numofsamples-1))*(armgoal_anglesV_rad[j] - armstart_anglesV_rad[j]);
    //     }
    //     if(!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size) && firstinvalidconf)
    //     {
    //         firstinvalidconf = 1;
    //         mexPrintf("ERROR: Invalid arm configuration!!!\n");
    //     }
    // }    
    // *planlength = numofsamples;
    
    return;
}

//prhs contains input parameters (3): 
//1st is matrix with all the obstacles
//2nd is a row vector of start angles for the arm 
//3nd is a row vector of goal angles for the arm 
//plhs should contain output parameters (2): 
//1st is a 2D matrix plan when each plan[i][j] is the value of jth angle at the ith step of the plan
//(there are D DoF of the arm (that is, D angles). So, j can take values from 0 to D-1
//2nd is planlength (int)
void mexFunction( int nlhs, mxArray *plhs[], 
		  int nrhs, const mxArray*prhs[])
     
{ 
    
    /* Check for proper number of arguments */    
    if (nrhs != 4) { 
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Four input arguments required."); 
    } else if (nlhs != 3) {
	    mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required."); 
    } 
        
    /* get the dimensions of the map and the map matrix itself*/     
    int x_size = (int) mxGetM(MAP_IN);
    int y_size = (int) mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* get the start and goal angles*/     
    int numofDOFs = (int) (MAX(mxGetM(ARMSTART_IN), mxGetN(ARMSTART_IN)));
    if(numofDOFs <= 1){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidnumofdofs",
                "it should be at least 2");         
    }
    double* armstart_anglesV_rad = mxGetPr(ARMSTART_IN);
    if (numofDOFs != MAX(mxGetM(ARMGOAL_IN), mxGetN(ARMGOAL_IN))){
        	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidnumofdofs",
                "numofDOFs in startangles is different from goalangles");         
    }
    double* armgoal_anglesV_rad = mxGetPr(ARMGOAL_IN);

 
    //get the planner id
    int planner_id = (int)*mxGetPr(PLANNER_ID_IN);
    if(planner_id < 0 || planner_id > 3)
    {
     mexErrMsgIdAndTxt( "MATLAB:planner:invalidplanner_id",
                "planner id should be between 0 and 3 inclusive");         
    }

        
    //call the planner
    double** plan = NULL;
    int planlength = 0;
    int numSamples = 0;
    
    // mexPrintf("calling planner\n");

    // int validSamples = 0;
    // int i,j;
    // int maxSamples = 20;
    // double startPos[maxSamples][5], goalPos[maxSamples][5];

    // unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    // std::default_random_engine generator(seed);
    // std::uniform_real_distribution<double> uni_distribution(0.0,1.0);

    // while (validSamples < maxSamples){
    //     for (j =0; j<numofDOFs; ++j){
    //         startPos[validSamples][j] = 2*PI*uni_distribution(generator);
    //         goalPos[validSamples][j] = 2*PI*uni_distribution(generator);
    //     }
    //     if (IsValidArmConfiguration(&startPos[validSamples][0], numofDOFs, map, x_size, y_size) 
    //       && IsValidArmConfiguration(&goalPos[validSamples][0], numofDOFs, map, x_size, y_size)){
    //         validSamples++;
    //     }
    // }

    // FILE *fp;
    // fp = fopen("startConfiguration.txt", "w");
    // for (i=0;i<maxSamples;i++){
    //     for (j=0;j<numofDOFs;j++){
    //         fprintf(fp,"%f,",startPos[i][j]);
    //     }
    //     fprintf(fp,"\n");
    // }
    // fclose (fp);

    // fp = fopen("goalConfigurations.txt", "w");
    // for (i=0;i<20;i++){
    //     for (j=0;j<numofDOFs;j++){
    //         fprintf(fp,"%f,", goalPos[i][j]);
    //     }
    //     fprintf(fp,"\n");
    // }
    // fclose (fp);


    planner(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength, planner_id, &numSamples); 
    mexPrintf("planner returned plan of length= %d with samples = %d\n", planlength, numSamples); 
    
    /* Create return values */
    if(planlength > 0)
    {
        PLAN_OUT = mxCreateNumericMatrix( (mwSize)planlength, (mwSize)numofDOFs, mxDOUBLE_CLASS, mxREAL); 
        double* plan_out = mxGetPr(PLAN_OUT);        
        //copy the values
        int i,j;
        for(i = 0; i < planlength; i++)
        {
            for (j = 0; j < numofDOFs; j++)
            {
                plan_out[j*planlength + i] = plan[i][j];
            }
        }
    }
    else
    {
        PLAN_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)numofDOFs, mxDOUBLE_CLASS, mxREAL); 
        double* plan_out = mxGetPr(PLAN_OUT);
        //copy the values
        int j;
        for(j = 0; j < numofDOFs; j++)
        {
          plan_out[j] = armstart_anglesV_rad[j];
        }     
    }
    PLANLENGTH_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxINT8_CLASS, mxREAL); 
    int* planlength_out = (int*)mxGetPr(PLANLENGTH_OUT);
    *planlength_out = planlength;

    NUMSAMPLES_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxINT32_CLASS, mxREAL); 
    int* samples_out = (int*)mxGetPr(NUMSAMPLES_OUT);
    *samples_out = numSamples;
    
    return;
    
}





