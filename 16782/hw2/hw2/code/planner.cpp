/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include "mex.h"
#include <stdlib.h>     /* srand, rand */
#include <list>


/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ARMSTART_IN	prhs[1]
#define	ARMGOAL_IN     prhs[2]


/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654

//the length of each link in the arm (should be the same as the one used in runtest.m)
#define LINKLENGTH_CELLS 10

typedef struct {
    double theta[5];
    int vertexID;
} vertex;

typedef struct {
    double startID;
    double endID;
} edge;

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
        int numofDOFs, double*  map, int x_size, int y_size)
{
  double epsilonStep = 0.1;
  int i = 0;
  double norm = 0.0;
  double normalizedDifference[numofDOFs];

  for (i = 0; i < numofDOFs; ++i)
    norm += pow((newSample[i] - nearestVertexAngles[i]),2);
  norm = pow(norm, 0.5);
  
  if(norm>epsilonStep)
  {
    for (i = 0; i < numofDOFs; ++i)
      nextConfig[i] = nearestVertexAngles[i] + epsilonStep*(newSample[i] - nearestVertexAngles[i])/norm;
  }
  else
  {
    for (i = 0; i < numofDOFs; ++i)
      nextConfig[i] = newSample[i];
  }

  if(!IsValidArmConfiguration(nextConfig, numofDOFs, map, x_size, y_size) == 0)
    return false;

  return true;
}

void findNearestVertex(std::list<vertex> *vertices, double *nearestVertexAngles, 
  int *nearestVertexID, int numofDOFs)
{
  
  double shortestDistance = 10000.0;
  double distance = 0;
  int i = 0;
  
  for (std::list<vertex>::iterator it= vertices->begin(); it != vertices->end(); ++it)
  {
    
    for (i = 0; i < numofDOFs; ++i)
      distance += pow(it->theta[i],2);

    distance = pow(distance,0.5);
    
    if (distance < shortestDistance)
    {
      shortestDistance = distance;
      *nearestVertexID = it->vertexID;
      
      for (i = 0; i < numofDOFs; ++i)
        nearestVertexAngles[i] = it->theta[i];
    }
  }


}


void rrtImplementation (double* map,
       int x_size,
       int y_size,
       double* armstart_anglesV_rad,
       double* armgoal_anglesV_rad,
       int numofDOFs,
       double*** plan,
       int* planlength)
{

  double newSample[numofDOFs];
  double nextConfig[numofDOFs];
  double nearestVertexAngles[numofDOFs];
  int nearestVertexID; 

  int i = 0;
  int IDNUMBER = 1;

  std::list<vertex> vertices;
  std::list<edge> edges;

  bool goalReached = false;
  int jointsMatching = 0;
  int iterations = 0;

  // add starting configuration to vertices list
  vertex newVertex;
  for (i = 0; i < numofDOFs; ++i)
    newVertex.theta[i] = armstart_anglesV_rad[i];
  newVertex.vertexID = IDNUMBER; 
  vertices.push_back(newVertex);

  edge newEdge;

  while(~goalReached)
  {
    
    // check sampling bias
    if ((rand() %  1) > 0.1)
    {
      // generate new sample
      for (i = 0; i < numofDOFs; ++i)
        newSample[i] = rand() %  360;
    }
    else
      for (i = 0; i < numofDOFs; ++i)
        newSample[i] = armgoal_anglesV_rad[i];


    // find closet vertex: nearestVertex
    findNearestVertex(&vertices, nearestVertexAngles, &nearestVertexID, numofDOFs);

    // find the next configuration by taking small epsilon step
    // return true if no collision happens
    if(~findNextStep(nearestVertexAngles, newSample, nextConfig, numofDOFs, map, x_size, y_size))
      continue;

    for (i = 0; i < numofDOFs; ++i)
    {
      if(nextConfig[i] == armgoal_anglesV_rad[i])
        jointsMatching++;
    }
    // check if goal is reached
    if (jointsMatching == numofDOFs)
      goalReached = true;
    else
      jointsMatching = 0;

    IDNUMBER++;

    for (i = 0; i < numofDOFs; ++i)
      newVertex.theta[i] = nextConfig[i];
    newVertex.vertexID = IDNUMBER;

    vertices.push_back(newVertex);

    // add the edge from nearestVertex to IDNUMBER
    newEdge.startID = nearestVertexID;
    newEdge.endID = IDNUMBER;
    edges.push_back(newEdge);
    iterations++;

    // check if iterations are over
    if (iterations > 10000)
    {
      mexPrintf("no solution found. Tried for 10000 iterations");
      return;
    }
  }
  *plan = (double**) malloc(iterations*sizeof(double*));
  for (i = 0; i < iterations; i++)
  {
    (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
    for(j = 0; j < numofDOFs; j++)
    {
      (*plan)[i][j] = ;
    }
}
static void planner(
		   double*	map,
		   int x_size,
 		   int y_size,
       double* armstart_anglesV_rad,
       double* armgoal_anglesV_rad,
  	   int numofDOFs,
  	   double*** plan,
  	   int* planlength)
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;
    
    //for now just do straight interpolation between start and goal checking for the validity of samples
    //but YOU  WILL WANT TO REPLACE THE CODE BELOW WITH YOUR PLANNER
    
    //RRT
    //vector of 
    //sample random
    rrtImplementation(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength);
    
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
    if (nrhs != 3) { 
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Three input arguments required."); 
    } else if (nlhs != 2) {
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
        
    //call the planner
    double** plan = NULL;
    int planlength = 0;
    
    planner(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength); 
    
    mexPrintf("planner returned plan of length=%d\n", planlength); 
    
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

    
    return;
    
}





