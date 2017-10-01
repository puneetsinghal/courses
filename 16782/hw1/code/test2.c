/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
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
    int theta;
    int gValue;
    int hValue;
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
        angle += 2 * M_PI;
    }
    int dir = (int)(angle / (2 * M_PI / NUMOFDIRS) + 0.5);
    if (dir == 8) {
        dir = 0;
    }
    return dir;
}

bool terminalCondition(int currentX, int currentY, int goalX, int goalY)
{
    if(abs(currentX - goalX) > 1 || abs(currentY - goalY) > 1)
    {
        return true;
    }
    return false;
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

void findHeuristic(double*  map, int * heuristic,
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
    heuristic[GETMAPINDEX(startX  + 1, startX + 1, x_size, y_size)] = 0;

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
            index = GETMAPINDEX(newStateX  + 1, newStateY + 1, x_size, y_size);
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
    // printf("check2");
    FILE *fp;
    fp = fopen("Output.txt", "w");
    for (i=0;i<x_size;i++){
        for (j=0;j<y_size;j++){
        fprintf(fp,"%d,",heuristic[i*y_size + j]);
        }
        fprintf(fp,"\n");
    }
    return;
}

void addToQueueAStar(node3_t ** tailPointer, node3_t ** headPointer, node3_t * newState)
{
    // printf("At start of adddToQueue, head is at: %p and tail is at: %p \n", *headPointer, *tailPointer);
    node3_t * movingPointer = NULL;
    node3_t * previousPointer = NULL;
    node3_t * tail = NULL;
    node3_t * head = NULL;

    movingPointer = *tailPointer;
    tail = *tailPointer;
    head = * headPointer;
    // printf("newG: %d, tailG: %d\n", newState->gValue, movingPointer->gValue);
    // printf("newH: %d, tailH: %d\n", newState->hValue, movingPointer->hValue);

    if(movingPointer==NULL)
    {
        // return newState;
        printf("adding to queue as head \n");
        *headPointer = newState;
        *tailPointer = newState;
    }
    else if ((newState->gValue + newState->hValue) >= (movingPointer->gValue + movingPointer->hValue))
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
            if ((newState->gValue + newState->hValue) < (movingPointer->gValue + movingPointer->hValue))
            {
                // printf("newF: %d, currentF: %d with moving pointer at: %p\n", 
                //     (newState->gValue + newState->hValue), (movingPointer->gValue + movingPointer->hValue), movingPointer);    
                previousPointer = movingPointer->prev;
                previousPointer->next = newState;
                newState->prev = previousPointer;
                newState->next = movingPointer;
                movingPointer->prev = newState;
                movingPointer = previousPointer;
                // newState->prev = movingPointer->prev;
                // movingPointer->prev->next = newState;
                // newState->next = movingPointer;
                // movingPointer = newState->prev;
            }
            else
            {
                break;
            }
        }
        // printf("adding to queue in beween \n");

        if(movingPointer->prev != NULL && (newState->gValue + newState->hValue) < (movingPointer->gValue + movingPointer->hValue))
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

void deleteFromQueueAStar(node3_t ** tailPointer, node3_t ** headPointer, node3_t * newState)
{
    node3_t * movingPointer = NULL;
    node3_t * previousPointer = NULL;
    node3_t * nextPointer = NULL;
    
    if(newState->next == NULL)
    {
        *tailPointer = newState->prev;
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
    bool closedSet[x_size][y_size][NUMOFDIRS];
    int gValue[x_size][y_size][NUMOFDIRS];
    // float fValue[x_size][y_size][NUMOFDIRS];
    // int heuristic[x_size][y_size];

    for (i = 0; i < x_size; ++i)
    {
        for (j = 0; j < y_size; ++j)
        {
            for (k = 0; k < NUMOFDIRS; ++k)
            {
                pointerArray[i][j][k] = NULL;
                closedSet[i][j][k] = 0;
                gValue[i][j][k] = 10000;
                // fValue[i][j][k] = 10000;
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
    gValue[startX][startY][dir] = 0;
    pointerArray[startX][startY][dir]->hValue = hValue[GETMAPINDEX(startX  + 1, startX + 1, x_size, y_size)];
    pointerArray[startX][startY][dir]->next = NULL;
    pointerArray[startX][startY][dir]->prev = NULL;
    // fValue[startX][startY][dir] = 0;

    // Initialize head and tail of open list as starting points
    head = pointerArray[startX][startY][dir];
    tail = pointerArray[startX][startY][dir];

    int newStateX, newStateY, newTheta, newDir;
    printf("Head is at: %p and tail is at %p \n", head, tail);
    // int iteration = 0;
    // printf("check1");
    int prim;

    printf("%d, %d, %d, %d\n", head->x, head->y, goalX, goalY);
    // bool check = terminalCondition(head->x, head->y, goalX, goalY);
    // printf("%d \n", check);
    // printf("check0");
    // printf("%d \n", (abs(head->x - goalX) > 1 || abs(head->y - goalY)>1));
    float currentPoseX, currentPoseY, currentPoseTheta;
    bool firstIteration = 1;
    while (abs(head->x - goalX) > 1 || abs(head->y - goalY)>1 && head != NULL)
    {
        // printf("check1");
        
        printf("new loop with %d, %d \n",head->x, head->y);
        for (prim = 0; prim < NUMOFPRIMS; ++prim) 
        {
            float newx, newy, newtheta;
            bool ret;
            currentPoseX = (head->x*RES);
            currentPoseY = (head->y*RES);
            currentPoseTheta = M_PI/4*(head->theta);
            ret = applyaction(map, x_size, y_size, currentPoseX, currentPoseY, currentPoseTheta, &newx, &newy, &newtheta, mprim, dir, prim);
            // printf("%f, %f, %d\n", newx, newy, ret);
            /* skip action that leads to collision */
            if (ret) 
            {
                // printf("%d, %d\n", newStateX, newStateY);

                // check the in bound condition. No need as applyaction is already checking it.
                
                // if (newStateX < 0 || newStateX >= x_size || newStateY < 0 || newStateY >= y_size)
                //     continue;
                
                // check that it is free space. No need as applyaction is already checking it.
                
                // printf("%d, %d \n", GETMAPINDEX(newStateX, newStateY, x_size, y_size), (int)map[GETMAPINDEX(newStateX, newStateY, x_size, y_size)]);
                newStateX = (int) (newx/RES + 0.5);
                newStateY = (int) (newy/RES + 0.5);
                newDir = getPrimitiveDirectionforRobotPose(newtheta);
                printf("%f, %f, %f, %d, %d, %d\n", newx, newy, newtheta, newStateX, newStateY, newDir);

                // index = GETMAPINDEX((int)newStateX  + 1, (int)newStateY + 1, x_size, y_size);
                // if ((int)map[index] != 0)
                //     continue;
                
                // check that the state is not closed
                if (closedSet[newStateX][newStateY][newDir] != 0)
                    continue;
        
                if (pointerArray[newStateX][newStateY][newDir] == NULL)
                {
                    printf("does not exist \n");
                    pointerArray[newStateX][newStateY][newDir] = malloc(sizeof(node3_t));
                    // printf("check in stuck position 0\n");
                    pointerArray[newStateX][newStateY][newDir]->x = newStateX;
                    // printf("check in stuck position 1\n");
                    pointerArray[newStateX][newStateY][newDir]->y = newStateY;
                    // printf("check in stuck position 2\n");
                    pointerArray[newStateX][newStateY][newDir]->theta = newDir; // randomly initializing theta value for dijkstra (2D case considered)
                    // printf("check in stuck position 3\n");
                    pointerArray[newStateX][newStateY][newDir]->gValue = head->gValue + 1;
                    // printf("check in stuck position 4\n");
                    gValue[newStateX][newStateY][newDir] = head->gValue + 1;
                    // printf("check in stuck position 5\n");
                    pointerArray[newStateX][newStateY][newDir]->hValue = hValue[GETMAPINDEX(newStateX+1, newStateY+1, x_size, y_size)];
                    // printf("check in stuck position 6\n");
                    // fValue[newStateX][newStateY][newDir] = head->gValue + 1;
                    pointerArray[newStateX][newStateY][newDir]->next = NULL;
                    // printf("check in stuck position 7\n");
                    pointerArray[newStateX][newStateY][newDir]->prev = NULL;
                    // printf("check in stuck position 8\n");
                    // printf("New node is at: %p\n", pointerArray[newStateX][newStateY]);
                    if(firstIteration==1)
                    {
                        // printf("check in stuck position 9\n");
                        pointerArray[newStateX][newStateY][newDir]->primValue = prim;
                    }
                    else
                    {
                        // printf("check in stuck position 10\n");
                        pointerArray[newStateX][newStateY][newDir]->primValue = head->primValue;
                    }
                    // printf("check in stuck position 11\n");
                    addToQueueAStar(&tail, &head, pointerArray[newStateX][newStateY][newDir]);
                    printf("Head is at: %p and tail is at %p \n", head, tail);
                }
                else if(pointerArray[newStateX][newStateY][newDir]->gValue > (head->gValue + 1))
                {
                    printf("exists \n");

                    pointerArray[newStateX][newStateY][newDir]->gValue = head->gValue + 1;
                    gValue[newStateX][newStateY][newDir] = head->gValue + 1;
                    pointerArray[newStateX][newStateY][newDir]->primValue = head->primValue;
                    // heuristic[index] = head->gValue + 1;
                    deleteFromQueueAStar(&tail, &head, pointerArray[newStateX][newStateY][newDir]);
                    pointerArray[newStateX][newStateY][newDir]->next = NULL;
                    pointerArray[newStateX][newStateY][newDir]->prev = NULL;
                    addToQueueAStar(&tail, &head, pointerArray[newStateX][newStateY][newDir]);
                }
            }
        }
        printf("check1 %d \n", firstIteration);
        if(firstIteration==1)
            firstIteration = 0;
        else
        {
            printf("head prim value: %d\n", head->primValue);
            *prim_id = head->primValue;
        }
            
        closedSet[head->x][head->y][head->theta] = 1;
        printf("head prev: %p, head: %p, head next: %p \n", head->prev, head, head->next);
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
        // check = terminalCondition(head->x, head->y, goalX, goalY);
        // iteration++;
        // if (iteration == 2)
        //     break;
    }
    // printf("check2");
    FILE *fp;
    fp = fopen("Output2.txt", "w");
    for (i=0;i<x_size;i++){
        for (j=0;j<y_size;j++){
        fprintf(fp,"%d,",gValue[i][j][0]);
        }
        fprintf(fp,"\n");
    }
    return;
}

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

int main()
{
    // node_t * head = NULL;

    // printf("%p\n",&head);   
    int x_size = 40;
    int y_size = 40;
    int startX = 5;
    int startY = 5;
    float startTheta = 0;
    int goalX = 15;
    int goalY = 15;
    // double map[100] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    //     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    //     0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    int totalSize = x_size*y_size;
    double map[totalSize];
    int hValue[totalSize];
    int i;

    for (i = 0; i < totalSize; ++i)
    {
        hValue[i] = 10000;
        map[i] = 0;
    }
    findHeuristic(map, hValue, x_size, y_size, goalX, goalY);
    // return 0;
    /* Read motion primtives */
    PrimArray motion_primitives;
    parseMotionPrimitives(motion_primitives);
    int * action_ptr;
    action_ptr = malloc(sizeof(int));
    // printf("%d \n", *action_ptr);
    // return 0;
    findPath(map, hValue, x_size, y_size, startX, startY, startTheta, goalX, goalY, motion_primitives, action_ptr);

    return 0;
}