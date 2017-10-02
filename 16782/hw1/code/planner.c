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
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y)*XSIZE + (X-1))

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
    int theta;
    int gValue;
    struct node_t * next;
    struct node_t * prev;

} node_t;

typedef struct node3_t {
    int x;
    int y;
    int theta;
    int gValue;
    int hValue;
    int primValue;
    struct node3_t * next;
    struct node3_t * prev;

} node3_t;



int temp = 0;

void print_linked_list_val(node3_t * currentNode){
    while(currentNode != NULL){
        printf("g value : %d\n", currentNode->gValue);
        printf("Current Node Address : %p\n", currentNode);
        printf("Prim Id : %d\n",currentNode->primValue);
        printf("X index : %d\n", currentNode->x);
        printf("Y index : %d\n", currentNode->y);
        printf("Next Node Address : %p\n", currentNode->next);
        printf("Prev Node Address : %p\n", currentNode->prev);
        currentNode = currentNode->next;
    } 
}

void print_node_val3d(node3_t* currentNode){
    if(currentNode != NULL){
        printf("g value: %d\n", currentNode->gValue);
        printf("h value : %d\n", currentNode->hValue);
        printf("g + eps(h) value : %d\n", currentNode->gValue + currentNode->hValue);
        printf("Prim Id : %d\n",currentNode->primValue);
        printf("X index : %d\n", currentNode->x);
        printf("Y index : %d\n", currentNode->y);
        printf("Theta index : %d\n", currentNode->theta);
        printf("Current Node Address : %p\n", currentNode);
        printf("Next Node Address : %p\n", currentNode->next);
        printf("Prev Node Address : %p\n\n", currentNode->prev);
    } 
}

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
        if (gridposx < 0 || gridposx >= x_size || gridposy < 0 || gridposy >= y_size)
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
        angle += 2.0 * M_PI;
    }
    int dir = (int)(angle / (2.0 * M_PI / NUMOFDIRS) + 0.5);
    if (dir == 8) {
        dir = 0;
    }
    return dir;
}

void addToQueue(node_t ** tailPointer, node_t ** headPointer, node_t * newState)
{
   // printf("At start of adddToQueue, head is at: %p and tail is at: %p \n", *headPointer, *tailPointer);
    node_t * movingPointer = NULL;
    node_t * nextPointer = NULL;
    node_t * tail = NULL;
    node_t * head = NULL;
    // node_t * newState = NULL;

    movingPointer = *tailPointer;
    tail = *tailPointer;
    head = * headPointer;
    // newState = *newS;
    // printf("newG: %d, tailG: %d\n", newState->gValue, movingPointer->gValue);
    // printf("newH: %d, tailH: %d\n", newState->hValue, movingPointer->hValue);

    if(movingPointer==NULL)
    {
        // return newState;
        // printf("adding to queue as head \n");
        *headPointer = newState;
        *tailPointer = newState;
        return;
    }
    else if(newState->gValue < head->gValue)
    { // add to head
        newState->next = head;
        head->prev = newState;
        newState->prev = NULL;
        *headPointer = newState;
        // *newS = newState;
        return;
    }
    else if (newState->gValue >= movingPointer->gValue)
    { //add to tail
        // printf("newG: %d, tailG: %d\n", newState->gValue, movingPointer->gValue);
        // printf("adding to queue as tail \n");
        tail->next = newState;
        newState->prev = tail;
        newState->next = NULL;
        *tailPointer = newState;
        // *newS = newState;
        return;
    }
    else
    {
        // printf("Moving pointer previous is at: %p \n", movingPointer->prev);
        while(movingPointer->prev != NULL)
        {
            if (newState->gValue < movingPointer->gValue)
            {
                // printf("newF: %d, currentF: %d with moving pointer at: %p\n", 
                //     (newState->gValue + newState->hValue), (movingPointer->gValue + movingPointer->hValue), movingPointer);    
                movingPointer = movingPointer->prev;
            }
            else
            {
                nextPointer = movingPointer->next;
                nextPointer->prev = newState;
                newState->next = nextPointer;
                newState->prev = movingPointer;
                movingPointer->next = newState;
                // *newS = newState;
                return;
            }
        }
        return;
    }
    // printf("At end of adddToQueue, head is at: %p and tail is at: %p \n", *headPointer, *tailPointer);

}

void deleteFromQueue(node_t ** tailPointer, node_t ** headPointer, node_t * newState)
{
    node_t * movingPointer = NULL;
    node_t * previousPointer = NULL;
    node_t * nextPointer = NULL;
    
    if(newState->next == NULL)
    {
        *tailPointer = newState->prev;
        newState->prev->next = NULL;
    }
    else
    {
        previousPointer = newState->prev;
        nextPointer = newState->next;
        previousPointer->next = nextPointer;
        nextPointer->prev = previousPointer;
        // newState->prev->next = newState->next;
        // newState->next->prev = newState->prev;
        // newState->prev = NULL;
        // newState->next = NULL;
    }
}

void findHeuristic(double*  map, int * heuristic,
           int x_size,
           int y_size,
            int startX,
            int startY)
{
    node_t * pointerArray[x_size][y_size];
    int i, j, k; // parameters for FOR loop
    bool closedSet[x_size][y_size];
    // int heuristic[x_size][y_size];

    for (i = 0; i < x_size; ++i)
    {
        for (j = 0; j < y_size; ++j)
        {
            pointerArray[i][j] = NULL;
            closedSet[i][j] = 0;
            // heuristic[i][j] = 10000;
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
    heuristic[GETMAPINDEX(startX, startY, x_size, y_size)] = 0;

    // Initialize head and tail of open list as starting points
    head = pointerArray[startX][startY];
    tail = pointerArray[startX][startY];

    //  create 8 connected grid motion primitives
    int eightConnectedPrim[8][2] = {{-1, 1}, {-1, 0}, {-1, -1}, {0, 1}, {0, -1}, {1, 1}, {1, 0}, {1, -1}};

    int newStateX, newStateY;
    // printf("Head is at: %p and tail is at %p \n", head, tail);
    // int iteration = 0;
    // printf("check1");
    int index;
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
            index = GETMAPINDEX(newStateX, newStateY, x_size, y_size);
            if ((int)map[index] != 0)
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
                heuristic[index] = head->gValue + 1;
                pointerArray[newStateX][newStateY]->next = NULL;
                pointerArray[newStateX][newStateY]->prev = NULL;
                // printf("New node is at: %p\n", pointerArray[newStateX][newStateY]);
                addToQueue(&tail, &head, pointerArray[newStateX][newStateY]);
                // printf("Head is at: %p and tail is at %p \n", head, tail);
            }
            else if(heuristic[index] > (head->gValue + 1))
            {
                pointerArray[newStateX][newStateY]->gValue = head->gValue + 1;
                heuristic[index] = head->gValue + 1;
                deleteFromQueue(&tail, &head, pointerArray[newStateX][newStateY]);
                pointerArray[newStateX][newStateY]->next = NULL;
                pointerArray[newStateX][newStateY]->prev = NULL;
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
    // printf("check2");
    // FILE *fp;
    // fp = fopen("Output.txt", "w");
    // for (i=0;i<x_size;i++){
    //     for (j=0;j<y_size;j++){
    //     fprintf(fp,"%d,",heuristic[j*x_size + i]);
    //     }
    //     fprintf(fp,"\n");
    // }
    return;
}

void addToQueueAStar(node3_t ** tailPointer, node3_t ** headPointer, node3_t * newState)
{
    // printf("At start of adddToQueue, head is at: %p and tail is at: %p \n", *headPointer, *tailPointer);
    float epsilon = 1;
    node3_t * movingPointer = NULL;
    node3_t * nextPointer = NULL;
    node3_t * tail = NULL;
    node3_t * head = NULL;
    // node3_t * newState = NULL;

    movingPointer = *tailPointer;
    tail = *tailPointer;
    head = * headPointer;
    // newState = *newS;
    // printf("newG: %d, tailG: %d\n", newState->gValue, movingPointer->gValue);
    // printf("newH: %d, tailH: %d\n", newState->hValue, movingPointer->hValue);

    if(movingPointer==NULL)
    {
        // return newState;
        // printf("adding to queue as head \n");
        *headPointer = newState;
        *tailPointer = newState;
        return;
    }
    else if((newState->gValue + epsilon*newState->hValue) < (head->gValue + epsilon*head->hValue))
    { // add to head
        newState->next = head;
        head->prev = newState;
        newState->prev = NULL;
        *headPointer = newState;
        // *newS = newState;
        return;
    }
    else if ((newState->gValue + epsilon*newState->hValue) >= (movingPointer->gValue + epsilon*movingPointer->hValue))
    { // add to tail
        // printf("newG: %d, tailG: %d\n", newState->gValue, movingPointer->gValue);
        // printf("adding to queue as tail \n");
        tail->next = newState;
        newState->prev = tail;
        newState->next = NULL;
        *tailPointer = newState;
        // *newS = newState;
        return;
    }
    else
    {
        // printf("Moving pointer previous is at: %p \n", movingPointer->prev);
        while(movingPointer->prev != NULL)
        {
            if ((newState->gValue + epsilon*newState->hValue) < (movingPointer->gValue + epsilon*movingPointer->hValue))
            {
                // printf("newF: %d, currentF: %d with moving pointer at: %p\n", 
                //     (newState->gValue + newState->hValue), (movingPointer->gValue + movingPointer->hValue), movingPointer);    
                movingPointer = movingPointer->prev;
            }
            else
            {
                nextPointer = movingPointer->next;
                nextPointer->prev = newState;
                newState->next = nextPointer;
                newState->prev = movingPointer;
                movingPointer->next = newState;
                // *newS = newState;
                return;
            }
        }
        return;
    }
    // printf("At end of adddToQueue, head is at: %p and tail is at: %p \n", *headPointer, *tailPointer);
}

void deleteFromQueueAStar(node3_t ** tailPointer, node3_t ** headPointer, node3_t * newState)
{
    node3_t * movingPointer = NULL;
    node3_t * previousPointer = NULL;
    node3_t * nextPointer = NULL;
    
    if(newState->next == NULL)
    {
        *tailPointer = newState->prev;
        newState->prev->next = NULL;
    }
    else
    {
        previousPointer = newState->prev;
        nextPointer = newState->next;
        previousPointer->next = nextPointer;
        nextPointer->prev = previousPointer;
        // newState->prev->next = newState->next;
        // newState->next->prev = newState->prev;
        // newState->prev = NULL;
        // newState->next = NULL;
    }
}

void findPath(double*  map, int * hValue,
           int x_size, int y_size,
            int startX, int startY, float startTheta,
            int goalX, int goalY,
            PrimArray mprim, int *prim_id)
{
    node3_t * pointerArray[x_size][y_size][NUMOFDIRS];
    int i, j, k; // parameters for FOR loop
    // bool closedSet[x_size][y_size][NUMOFDIRS];
    // int gValue[x_size][y_size][NUMOFDIRS];
    // float fValue[x_size][y_size][NUMOFDIRS];
    // int heuristic[x_size][y_size];
    bool ***closedSet = (bool ***)malloc(x_size * sizeof(bool**));
    for (i=0; i<x_size; i++)
    {
        closedSet[i] = (bool **)malloc(y_size * sizeof(bool*));
        for (j = 0; j < y_size; ++j)
        {
            closedSet[i][j] = (bool *)malloc(NUMOFDIRS * sizeof(bool));
        }
    }
    for (i = 0; i < x_size; ++i)
    {
        for (j = 0; j < y_size; ++j)
        {
            for (k = 0; k < NUMOFDIRS; ++k)
            {
                pointerArray[i][j][k] = NULL;
                closedSet[i][j][k] = 0;
            }
        }
    }

    // start the head for open list
    node3_t * head = NULL;
    // printf("Head is at: %p\n", head);

    // start the tail for open list
    node3_t * tail = NULL;
    // printf("Tail is at: %p\n",tail);

    node3_t * forDeletionOfHead = NULL;

    // add start position to open set. Modify the pointerArray
    // float startTheta = 0;
    int dir;
    dir = getPrimitiveDirectionforRobotPose(startTheta);
    pointerArray[startX][startY][dir] = malloc(sizeof(node3_t));
    pointerArray[startX][startY][dir]->x = startX;
    pointerArray[startX][startY][dir]->y = startY;
    pointerArray[startX][startY][dir]->theta = dir; // randomly initializing theta value for dijkstra (2D case considered)
    pointerArray[startX][startY][dir]->gValue = 0;
    pointerArray[startX][startY][dir]->hValue = hValue[GETMAPINDEX(startX, startY, x_size, y_size)];
    pointerArray[startX][startY][dir]->primValue = 0;

    // gValue[startX][startY][dir] = 0;
    pointerArray[startX][startY][dir]->next = NULL;
    pointerArray[startX][startY][dir]->prev = NULL;
    // fValue[startX][startY][dir] = 0;

    // Initialize head and tail of open list as starting points
    head = pointerArray[startX][startY][dir];
    tail = pointerArray[startX][startY][dir];

    int newStateX, newStateY, newTheta, newDir;
    // printf("Head is at: %p and tail is at %p \n", head, tail);
    int iteration = 0;
    // printf("check1");
    int prim;

    // printf("%d, %d, %d, %d\n", head->x, head->y, goalX, goalY);
    // bool check = terminalCondition(head->x, head->y, goalX, goalY);
    // printf("%d \n", check);
    // printf("check0");
    // printf("%d \n", (abs(head->x - goalX) > 1 || abs(head->y - goalY)>1));
    float currentPoseX, currentPoseY, currentPoseTheta;
    bool firstIteration = 1;
    // printf("%p\n", head);
    // return;
    node3_t * currentNode = NULL;

    while (head != NULL)
    {
        free(currentNode);
        currentNode = head; 
        // printf("head prim value: %d\n", head->primValue);
        *prim_id = currentNode->primValue;

        if (head->next !=NULL)
        {
            head = currentNode->next;
            head->prev = NULL;
        }
        else
        {
            head = NULL;
            tail = NULL;
        }
        // print_node_val3d(head);
        // printf("----------------------------------------------------\n");
        // printf("new loop with %d, %d \n",head->x, head->y);
        if (currentNode->hValue < 4)
            break;
        for (prim = 0; prim < NUMOFPRIMS; ++prim) 
        {
            float newx, newy, newtheta;
            bool ret;
            currentPoseX = (currentNode->x*RES);
            currentPoseY = (currentNode->y*RES);
            currentPoseTheta = 2.0*M_PI/NUMOFDIRS*(currentNode->theta);
            ret = applyaction(map, x_size, y_size, currentPoseX, currentPoseY, currentPoseTheta, &newx, &newy, &newtheta, mprim, currentNode->theta, prim);
            // printf("%f, %f, %d\n", newx, newy, ret);
            /* skip action that leads to collision */
            if (ret) 
            {
                // printf("%d, %d\n", newStateX, newStateY);

                // check the in bound condition. No need as applyaction is already checking it.
                // check that it is free space. No need as applyaction is already checking it.
                newStateX = (int) (newx/RES + 0.5);
                newStateY = (int) (newy/RES + 0.5);
                newDir = getPrimitiveDirectionforRobotPose(newtheta);
                // printf("%f, %f, %f, %d, %d, %d\n", newx, newy, newtheta, newStateX, newStateY, newDir);
                
                // check that the state is not closed
                if (closedSet[newStateX][newStateY][newDir] != 0)
                    continue;
                
                if (pointerArray[newStateX][newStateY][newDir] == NULL)
                {
                    // printf("does not exist \n");
                    pointerArray[newStateX][newStateY][newDir] = malloc(sizeof(node3_t));
                    pointerArray[newStateX][newStateY][newDir]->x = newStateX;
                    pointerArray[newStateX][newStateY][newDir]->y = newStateY;
                    pointerArray[newStateX][newStateY][newDir]->theta = newDir; // randomly initializing theta value for dijkstra (2D case considered)
                    pointerArray[newStateX][newStateY][newDir]->gValue = currentNode->gValue + 1;
                    // gValue[newStateX][newStateY][newDir] = currentNode->gValue + 1;
                    pointerArray[newStateX][newStateY][newDir]->hValue = hValue[GETMAPINDEX(newStateX +1, newStateY + 1, x_size, y_size)];
                    pointerArray[newStateX][newStateY][newDir]->next = NULL;
                    pointerArray[newStateX][newStateY][newDir]->prev = NULL;
                    // printf("New node is at: %p\n", pointerArray[newStateX][newStateY]);
                    if(firstIteration==1)
                        pointerArray[newStateX][newStateY][newDir]->primValue = prim;
                    else
                        pointerArray[newStateX][newStateY][newDir]->primValue = currentNode->primValue;
                    
                   addToQueueAStar(&tail, &head, pointerArray[newStateX][newStateY][newDir]);
                    // printf("Head is at: %p and tail is at %p \n", head, tail);
                }
                else if(pointerArray[newStateX][newStateY][newDir]->gValue > (currentNode->gValue + 1))
                {
                    pointerArray[newStateX][newStateY][newDir]->gValue = currentNode->gValue + 1;
                    // gValue[newStateX][newStateY][newDir] = currentNode->gValue + 1;
                    pointerArray[newStateX][newStateY][newDir]->primValue = currentNode->primValue;
                    // heuristic[index] = currentNode->gValue + 1;
                    deleteFromQueueAStar(&tail, &head, pointerArray[newStateX][newStateY][newDir]);
                    // pointerArray[newStateX][newStateY][newDir]->next = NULL;
                    // pointerArray[newStateX][newStateY][newDir]->prev = NULL;
                    addToQueueAStar(&tail, &head, pointerArray[newStateX][newStateY][newDir]);
                }
                // print_node_val3d(pointerArray[newStateX][newStateY][newDir]);
                // printf("\n");
            }
        }
        // print_linked_list_val(head);
        // return;
        // printf("check1 %d \n", firstIteration);
        firstIteration = 0;

        closedSet[currentNode->x][currentNode->y][currentNode->theta] = 1;
        // printf("head prev: %p, head: %p, head next: %p \n", head->prev, head, head->next);
        // if(head->next==NULL)
        // {
        //     free(head);
        //     head = NULL;
        // }
        // else
        // {
        //     forDeletionOfHead = head->next;
        //     forDeletionOfHead->prev = NULL;
        //     free(head);
        //     head = forDeletionOfHead;
        // }
        // check = terminalCondition(head->x, head->y, goalX, goalY);
        // iteration++;
        // if (iteration == 10)
        //     return;
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
    // int prim;
	dir = getPrimitiveDirectionforRobotPose(robotposeTheta);
    

    int startX = (int) (robotposeX/RES);
    int startY = (int) (robotposeY/RES);
    float startTheta = robotposeTheta;
    int goalX = (int) (goalposeX/RES);
    int goalY = (int) (goalposeY/RES);
    
    int totalSize = x_size*y_size; 
    // initializing heuristic values
    int hValue[totalSize];
    int i;

    for (i = 0; i < totalSize; ++i)
    {
        hValue[i] = 10000;
    }
    findHeuristic(map, hValue, x_size, y_size, goalX, goalY);

    printf("%d",hValue[10]);
    // return;

    findPath(map, hValue, x_size, y_size, startX, startY, startTheta, goalX, goalY, mprim, prim_id);

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
    // printf("check");   
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    // printf("%d, %d \n", x_size, y_size);
    // float pose = 0.1;
    // printf("%d \n", (int) (pose/RES + 0.5));
    
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
    // printf("check 0 ");
    planner(map, x_size, y_size, robotposeX, robotposeY, robotposeTheta, goalposeX, goalposeY, motion_primitives, &action_ptr[0]);

    return;
    
}





