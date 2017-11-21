/*
 * tree.hpp
 * Contains a class that defines tree structure
 */

#ifndef GRAPH__
#define GRAPH__

#include <list>
#include <queue>
#include <vector>

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

typedef struct literals{
    int ON[2];
    bool operator<(literals & newLiterals) const
    {
        return ON[0] < newLiterals.ON[0];
    }
} literals;

typedef struct literalsStruct{
    std::list<literals> onLiterals;
    std::list<int> clearLiterals;
    literalsStruct* parentAddress;
    int plan[4];
    int gValue;
    int hValue;
    int fValue;
    bool isClosed;
    bool operator<(literalsStruct & newVertex) const
    {
        return fValue < newVertex.fValue;
    }

} literalsStruct;

class Comparator
{
public:
    int operator() (const literalsStruct* pointer1, const literalsStruct* pointer2) const
    {
        return (pointer1->fValue > pointer2->fValue);
    }
};

class graph
{

/*
 */
public:
	graph(int* blocksV, int numofblocks, int* trianglesV, int numoftriangles, int TableIndex, 
            int** onV_start, int onV_start_length, int* clearV_start, int numofclear_start, 
            int** onV_goal, int onV_goal_length, int* clearV_goal, int numofclear_goal);
    graph(literalsStruct currentNode);
    bool isBlock(int blockID);
    void moveTo(literalsStruct* currentNode, int x, int y, int z);
    void heuristicMoveTo(graph* heuristicGraph, literalsStruct *currentNode, int x, int y, int z);
    void moveToTable(literalsStruct* currentNode, int x, int y);
    void heuristicMoveToTable(graph* heuristicGraph, literalsStruct *currentNode, int x, int y);
    bool findBaseBlock(literalsStruct *currentNode, int x, int* y);
    bool findBaseBlock(literalsStruct *currentNode, int x, std::list<int> *baseBlocks);
    void takeAction(literalsStruct* currentNode);
    void heuristicTakeAction(graph* heuristicGraph, literalsStruct *currentNode);
    bool reachedGoalState(literalsStruct *currentNode);
    void distanceFromGoal(literalsStruct* currentNode, int* distance);
    void findHeuristicValue(literalsStruct *baseNode, int* heuristicValue);
    void printGraph();
    void printList();
	void printVertex(literalsStruct currentNode);

public:
	std::list<int> blocks;
    std::list<int> triangles;
    int table;
    literalsStruct goalNode;
    std::list<literalsStruct> gVertices;
    std::priority_queue<literalsStruct*, std::vector<literalsStruct*>, Comparator> openList;
};

#endif // GRAPH__