/*
 * tree.hpp
 * Contains a class that defines tree structure
 */

#ifndef GRAPH__
#define GRAPH__

#include <list>

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

struct graphEdge;

typedef struct graphVertex{
    double theta[5];
    int vertexID;
    int numConnected;
    bool isClosed;
    double gValue;
    std::list<graphEdge*> connections;
    graphVertex* parentAddress;

    bool operator<(graphVertex & newVertex) const
    {
    	return gValue < newVertex.gValue; 
    }

} graphVertex;

typedef struct graphEdge{
    graphVertex *startAddress;
    graphVertex *endAddress;
    double cost;
}graphEdge;

class graph
{

/*
 */
public:
	void addVertexGraph(double *sample, int idNumber, int numofDOFs);
	void addEdgeGraph(graphVertex *newVertex, graphVertex *nearVertex, int numofDOFs);
	void createNeighborhoodGraph(graphVertex **newVertex, std::list<graphVertex*> *neighborHood, int numofDOFs, double radius);
	void printLastEdge(int numofDOFs);
	void print_vertex(graphVertex *currentNode, int numofDOFs);

public:
	std::list<graphVertex> gVertices;
	std::list<graphEdge> gEdges;
};

#endif // GRAPH__