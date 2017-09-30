/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include "mex.h"
#include <stdio.h>
#include <stdlib.h>

/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ROBOT_IN	prhs[1]
#define	GOAL_IN     prhs[2]


/* Output Arguments */
#define	ACTION_OUT	plhs[0]

/*access to the map is shifted to account for 0-based indexing in the map, whereas
1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)*/
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

/* Primitives Information */
#define NUMOFDIRS 8
#define NUMOFPRIMS 5
#define NUMOFINTERSTATES 10
#define NUMOFDIM 3

#define RES 0.1

typedef float PrimArray[NUMOFDIRS][NUMOFPRIMS][NUMOFINTERSTATES][NUMOFDIM];

typedef struct node_t {
    int x;
    int y;
    float theta;
    float gValue;
    struct node_t * next;
    struct node_t * prev;

} node_t;

int temp = 0;

bool applyaction(double *map, int x_size, int y_size, float robotposeX, float robotposeY, float robotposeTheta,
                 float *newx, float *newy, float *newtheta, PrimArray mprim, int dir, int prim)
{
    int i;
    for (i = 0; i < NUMOFINTERSTATES; i++) {
        *newx = robotposeX + mprim[dir][prim][i][0];
        *newy = robotposeY + mprim[dir][prim][i][1];
        *newtheta = mprim[dir][prim][i][2];
        
        int gridposx = (int)(*newx / RES + 0.5);
        int gridposy = (int)(*newy / RES + 0.5);

        /* check validity */
        if (gridposx < 1 || gridposx > x_size || gridposy < 1 || gridposy > y_size)
        {
            return false;
        }

        if ((int)map[GETMAPINDEX(gridposx, gridposy, x_size, y_size)] != 0)
        {
            return false;
        }
    }


    return true;
}

int getPrimitiveDirectionforRobotPose(float angle)
{
    /* returns the direction index with respect to the PrimArray */
    /* normalize bw 0 to 2pi */
    if (angle < 0.0) {
        angle += 2 * M_PI;
    }
    int dir = (int)(angle / (2 * M_PI / NUMOFDIRS) + 0.5);
    if (dir == 8) {
        dir = 0;
    }
    return dir;
}

void addToQueue(node_t ** tailPointer, node_t ** headPointer, node_t * newState)
{
    // printf("At start of adddToQueue, head is at: %p and tail is at: %p \n", *headPointer, *tailPointer);
    node_t * movingPointer = NULL;
    node_t * tail = NULL;
    node_t * head = NULL;

    movingPointer = *tailPointer;
    tail = *tailPointer;
    head = * headPointer;
    // printf("newG: %d, tailG: %d\n", newState->gValue, movingPointer->gValue);
    if(movingPointer==NULL)
    {
        // return newState;
        // printf("adding to queue as head \n");
        *headPointer = newState;
        *tailPointer = newState;
    }
    else if (newState->gValue >= movingPointer->gValue)
    {
        // printf("newG: %d, tailG: %d\n", newState->gValue, movingPointer->gValue);
        // printf("adding to queue as tail \n");
        tail->next = newState;
        newState->prev = tail;
        *tailPointer = newState;
    }
    else
    {
        // printf("Moving pointer previous is at: %p \n", movingPointer->prev);
        while(movingPointer->prev != NULL)
        {
            if (newState->gValue < movingPointer->gValue)
            {
                newState->prev = movingPointer->prev;
                movingPointer->prev->next = newState;
                newState->next = movingPointer;
                movingPointer = newState->prev;
            }
            else
            {
                break;
            }
        }
        // printf("adding to queue in beween \n");

        if(movingPointer->prev != NULL && newState->gValue < movingPointer->gValue)
        {
            // printf("adding to queue as head due to minimum \n");
            
            newState->next = head;
            newState->prev = NULL;
            *headPointer = newState;
            // return newState;
        }
    }
    // printf("At end of adddToQueue, head is at: %p and tail is at: %p \n", *headPointer, *tailPointer);

}

void deleteFromQueue(node_t * tail, node_t * head, node_t * newState)
{
    node_t * movingPointer = NULL;

    if(newState->next == NULL)
    {
        tail = newState->prev;
    }
    else
    {
        newState->prev->next = newState->next;
        newState->next->prev = newState->prev;
        newState->prev = NULL;
        newState->next = NULL;
    }
}

void findHeuristic(double*  map,
           int x_size,
           int y_size,
            int startX,
            int startY)
{
    // bool openSet[x_size, y_size];
    // for (int i = 0; i < x_size; ++i)
    // {
    //     for (int j = 0; j < y_size; ++j)
    //     {
    //         heuristic[i][j] = 100000;
    //         openSet[i][j] = 1;
    //     }
    // }
    // heuristic[goalX][goalY] = 0;
    node_t * pointerArray[x_size][y_size];
    int i, j, k; // parameters for FOR loop
    bool closedSet[x_size][y_size];
    int heuristic[x_size][y_size];

    for (i = 0; i < x_size; ++i)
    {
        for (j = 0; j < y_size; ++j)
        {
            pointerArray[i][j] = NULL;
            closedSet[i][j] = 0;
            heuristic[i][j] = 10000;
        }                  
    }
    // start the head for open list
    node_t * head = NULL;
    // head = malloc(sizeof(node_t));
    // printf("Head is at: %p\n", head);

    // start the tail for open list
    node_t * tail = NULL;
    // tail = malloc(sizeof(node_t));
    // printf("Tail is at: %p\n",tail);

    node_t * forDeletionOfHead = NULL;

    // add start position to open set. Modify the pointerArray
    pointerArray[startX][startY] = malloc(sizeof(node_t));
    pointerArray[startX][startY]->x = startX;
    pointerArray[startX][startY]->y = startY;
    pointerArray[startX][startY]->theta = 0; // randomly initializing theta value for dijkstra (2D case considered)
    pointerArray[startX][startY]->gValue = 0;
    pointerArray[startX][startY]->next = NULL;
    pointerArray[startX][startY]->prev = NULL;
    heuristic[startX][startY] = 0;

    // Initialize head and tail of open list as starting points
    head = pointerArray[startX][startY];
    tail = pointerArray[startX][startY];

    //  create 8 connected grid motion primitives
    int eightConnectedPrim[8][2] = {{-1, 1}, {-1, 0}, {-1, -1}, {0, 1}, {0, -1}, {1, 1}, {1, 0}, {1, -1}};

    int newStateX, newStateY;
    // printf("Head is at: %p and tail is at %p \n", head, tail);
    // int iteration = 0;
    printf("check1");
    while (head != NULL)
    {
        // run a loop for 8 connected grids

        for (i = 0; i < 8; ++i) // 8 connected grid
        {
            newStateX = head->x + eightConnectedPrim[i][0];
            newStateY = head->y + eightConnectedPrim[i][1];
            // printf("%d, %d\n", newStateX, newStateY);

            // check the in bound condition
            if (newStateX < 0 || newStateX >= x_size || newStateY < 0 || newStateY >= y_size)
                continue;
            // check that it is free space
            // printf("%d, %d \n", GETMAPINDEX(newStateX, newStateY, x_size, y_size), (int)map[GETMAPINDEX(newStateX, newStateY, x_size, y_size)]);
            if ((int)map[GETMAPINDEX(newStateX  + 1, newStateY + 1, x_size, y_size)] != 0)
                continue;
            // check that the state is not closed
            if (closedSet[newStateX][newStateY] != 0)
                continue;
            
            if (pointerArray[newStateX][newStateY] == NULL)
            {
                pointerArray[newStateX][newStateY] = malloc(sizeof(node_t));
                pointerArray[newStateX][newStateY]->x = newStateX;
                pointerArray[newStateX][newStateY]->y = newStateY;
                pointerArray[newStateX][newStateY]->theta = 0; // randomly initializing theta value for dijkstra (2D case considered)
                pointerArray[newStateX][newStateY]->gValue = head->gValue + 1;
                heuristic[newStateX][newStateY] = head->gValue + 1;
                pointerArray[newStateX][newStateY]->next = NULL;
                pointerArray[newStateX][newStateY]->prev = NULL;
                // printf("New node is at: %p\n", pointerArray[newStateX][newStateY]);
                addToQueue(&tail, &head, pointerArray[newStateX][newStateY]);
                // printf("Head is at: %p and tail is at %p \n", head, tail);
            }
            else if(heuristic[newStateX][newStateY] > (head->gValue + 1))
            {
                pointerArray[newStateX][newStateY]->gValue = head->gValue + 1;
                heuristic[newStateX][newStateY] = head->gValue + 1;
                deleteFromQueue(tail, head, pointerArray[newStateX][newStateY]);
                addToQueue(&tail, &head, pointerArray[newStateX][newStateY]);
            }
        }
        closedSet[head->x][head->y] = 1;
        // printf("head prev: %p, head: %p, head next: %p \n", head->prev, head, head->next);
        if(head->next==NULL)
        {
            free(head);
            head = NULL;
        }
        else
        {
            forDeletionOfHead = head->next;
            forDeletionOfHead->prev = NULL;
            free(head);
            head = forDeletionOfHead;
        }
        // iteration++;
        // if (iteration == 2)
        //     break;
    }
    printf("check2");
    FILE *fp;
    fp = fopen("Output.txt", "w");
    for (i=0;i<x_size;i++){
        for (j=0;j<y_size;j++){
        fprintf(fp,"%d,",heuristic[i][j]);
        }
        fprintf(fp,"\n");
    }
    return;
}

static void planner(
		   double*	map,
		   int x_size,
 		   int y_size,
           float robotposeX,
            float robotposeY,
            float robotposeTheta,
            float goalposeX,
            float goalposeY,
            PrimArray mprim,
            int *prim_id)
{   
    printf("temp=%d\n", temp);
    temp = temp+1;

    *prim_id = 0; /* arbitrary action */
    
    /*printf("robot: %d %d; ", robotposeX, robotposeY);*/
    /*printf("goal: %d %d;", goalposeX, goalposeY);*/
    
	/*for now greedily move towards the target, */
	/*but this is where you can put your planner */
	double mindisttotarget = 1000000;
    int dir;
    int prim;
	dir = getPrimitiveDirectionforRobotPose(robotposeTheta);
    

    int startX = (int) robotposeX/RES;
    int startY = (int) robotposeY/RES;
    float startTheta = robotposeTheta;
    int goalX = (int) goalposeX/RES;
    int goalY = (int) goalposeY/RES;
    
    findHeuristic(map, x_size, y_size, goalX, goalY);

    // define heuristic cost as Euclidean distance to target (in discrete space)
    // int h_cost[x_size][y_size];
    // float g_cost[x_size][y_size];
    // Euclidean heuristic
    // for (i=0; i<x_size; i++)
    // {
    //     for (j=0; j<y_size; j++)
    //     {
    //         h_cost[i,j] = sqrt((i-goalX)*(i-goalX) + (j-goalY)*(j-goalY));
    //         g_cost[i,j] = 100000;
    //     }
    // }
    // int i, j, k;
    // for (i = 0; i < x_size; ++i)
    // {
    //     for (j = 0; j < y_size; ++j)
    //     {
    //         h_cost[i][j] = 0;
    //     }
    // }
    //  Dynamic programming for heuristic
    // int eightConnectedPrim[8][2] = {{-1, 1}, {-1, 0}, {-1, -1}, {0, 1}, {0, -1}, {1, 1}, {1, 0}, {1, -1}};
    // int lowestNumber;
    // int newStateX;
    // int newStateY;
    // int l;
    // int num = MAX(x_size, y_size)/2;
    // for (l = 0; l < num; ++l)
    // {
    //     for (i=0; i<x_size; i++)
    //     {
    //         for (j=0; j<y_size; j++)
    //         {
    //             if((int)map[GETMAPINDEX(i, j, x_size, y_size)] == 0)
    //             {    
    //                 if(i != startX && j != startY)
    //                 { 
    //                     lowestNumber = 100000;
    //                     for (k = 0; k < 8; ++k) // 8 connected grid
    //                     {
    //                         newStateX = i + eightConnectedPrim[k][0];
    //                         newStateY = j + eightConnectedPrim[k][1];

    //                         if (newStateX >= 0 && newStateX <= x_size && newStateY >= 0 && newStateY <= y_size && 
    //                             (int)map[GETMAPINDEX(newStateX, newStateY, x_size, y_size)] == 0)
    //                         {
    //                             if (h_cost[newStateX][newStateY] < lowestNumber)
    //                             {
    //                                 lowestNumber = h_cost[newStateX][newStateY];
    //                             }
    //                         }
    //                     }
    //                     h_cost[i][j] = lowestNumber+1;
    //                 }
    //             }
    //         }
    //     }
    // }
    // printf("Done");
    // for (i = 0; i < 10; ++i)
    // {
    //     printf("%d \n", h_cost[i][30]);
    // }


    // g_cost[startX, startY] = 0;
    // node_t * head = NULL;
    // head = malloc(sizeof(node_t));

    // head->x = startX;
    // head->y = startY;
    // head->theta = startTheta;
    // head->g_value = 0;
    // head->f_value = head->g_value + h[x, y]
    // head->next = NULL;
    // bool goalExpanded = false;
    // node_t * tail = .head;
    // node_t * current = head;

    // while ~goalExpanded
    // {
    //     for (prim = 0; prim < NUMOFPRIMS; prim++) 
    //     {
    //         float newx, newy, newtheta;
    //         bool ret;
    //         dir = getPrimitiveDirectionforRobotPose();
    //         ret = applyaction(map, x_size, y_size, robotposeX, robotposeY, robotposeTheta, &newx, &newy, &newtheta, mprim, dir, prim);
    //         /* skip action that leads to collision */
    //         if (ret) 
    //         {
    //             double disttotarget = (double)sqrt(((newx-goalposeX)*(newx-goalposeX) + (newy-goalposeY)*(newy-goalposeY)));
    //             if(disttotarget < mindisttotarget)
    //             {
    //                 mindisttotarget = disttotarget;
                    
    //                 *prim_id = prim;
    //             }            
    //         }
    //     }
    // }


    // for (prim = 0; prim < NUMOFPRIMS; prim++) {
    //     float newx, newy, newtheta;
    //     bool ret;
    //     ret = applyaction(map, x_size, y_size, robotposeX, robotposeY, robotposeTheta, &newx, &newy, &newtheta, mprim, dir, prim);
    //         /* skip action that leads to collision */
    //     if (ret) {
    //         double disttotarget = (double)sqrt(((newx-goalposeX)*(newx-goalposeX) + (newy-goalposeY)*(newy-goalposeY)));
    //         if(disttotarget < mindisttotarget){
    //             mindisttotarget = disttotarget;
                
    //             *prim_id = prim;
    //         }            
    //     }

    // }
    printf("action %d\n", *prim_id);
    return;
}

/*prhs contains input parameters (3): 
1st is matrix with all the obstacles
2nd is a row vector <x,y> for the robot pose
3rd is a row vector <x,y> for the target pose
plhs should contain output parameters (1): 
1st is a row vector <dx,dy> which corresponds to the action that the robot should make*/

void parseMotionPrimitives(PrimArray mprim)
{
    FILE * fp;
    fp = fopen ("unicycle_8angles.mprim", "r+");
    char skip_c[100];
    int skip_f;
    float resolution;
    int num_angles;
    int num_mprims;
    fscanf(fp, "%s %f", skip_c, &resolution);
    fscanf(fp, "%s %d", skip_c, &num_angles);
    fscanf(fp, "%s %d", skip_c, &num_mprims);

    int i, j, k;
    for (i = 0; i < NUMOFDIRS; ++i) {
        for (j = 0; j < NUMOFPRIMS; ++j) {
            fscanf(fp, "%s %d", skip_c, &skip_f);
            for (k = 0; k < NUMOFINTERSTATES; ++k) {
                fscanf(fp, "%f %f %f", &mprim[i][j][k][0], &mprim[i][j][k][1], &mprim[i][j][k][2]);
            }

        }
    }
}

void mexFunction( int nlhs, mxArray *plhs[], 
		  int nrhs, const mxArray*prhs[] )
     
{

    /* Read motion primtives */
    PrimArray motion_primitives;
    parseMotionPrimitives(motion_primitives);

    /* Check for proper number of arguments */    
    if (nrhs != 3) { 
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Three input arguments required."); 
    } else if (nlhs != 1) {
	    mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required."); 
    } 
        
    /* get the dimensions of the map and the map matrix itself*/  
    printf("scheck");   
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* get the dimensions of the robotpose and the robotpose itself*/     
    int robotpose_M = mxGetM(ROBOT_IN);
    int robotpose_N = mxGetN(ROBOT_IN);
    if(robotpose_M != 1 || robotpose_N != 3){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidrobotpose",
                "robotpose vector should be 1 by 3.");         
    }
    double* robotposeV = mxGetPr(ROBOT_IN);
    float robotposeX = (float)robotposeV[0];
    float robotposeY = (float)robotposeV[1];
    float robotposeTheta = (float)robotposeV[2];
    
    /* get the dimensions of the goalpose and the goalpose itself*/     
    int goalpose_M = mxGetM(GOAL_IN);
    int goalpose_N = mxGetN(GOAL_IN);
    if(goalpose_M != 1 || goalpose_N != 3){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidgoalpose",
                "goalpose vector should be 1 by 3.");         
    }
    double* goalposeV = mxGetPr(GOAL_IN);
    float goalposeX = (float)goalposeV[0];
    float goalposeY = (float)goalposeV[1];
        
    /* Create a matrix for the return action */ 
    ACTION_OUT = mxCreateNumericMatrix( 1, 1, mxINT8_CLASS, mxREAL); 
    int* action_ptr = (int*) mxGetData(ACTION_OUT);

    /* Do the actual planning in a subroutine */
    printf("check 0 ");
    planner(map, x_size, y_size, robotposeX, robotposeY, robotposeTheta, goalposeX, goalposeY, motion_primitives, &action_ptr[0]);

    return;
    
}





