/*
 * tree.hpp
 * Contains a class that defines tree structure
 */

#ifndef TREE__
#define TREE__

#include <list>

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

typedef struct vertex{
    double theta[5];
    int vertexID;
    double cost; 
    std::list<vertex*> children;
    int parentID;
    vertex* parentAddress;
} vertex;

typedef struct edge {
    double startID;
    double endID;
} edge;

class tree
{

/*
 * find vertices
 */
public:
	void 	findNearestVertex(double *newSample, vertex **nearestVertex, int numofDOFs);
	void 	findConfigurationForVertex(int ID, double *nextConfig, int numofDOFs);
	void 	findParentVertex(int childVertex, int *parentVertex);
	void 	createNeighborhood(vertex *newVertex, std::list<vertex*> *neighborHood, int numofDOFs);
	void 	addEdge(vertex *minVertex, int numofDOFs);
	void 	getPathFromTree(std::list<int> * pathVertices, int goalID);
	void 	print_vertex(vertex *currentNode, int numofDOFs);

public:
	std::list<vertex> vertices;

};

#endif // TREE__