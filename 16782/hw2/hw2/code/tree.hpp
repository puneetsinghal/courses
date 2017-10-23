/*
 * tree.hpp
 * Contains a class that defines tree structure
 */

#ifndef TREE__
#define TREE__

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif


class tree{

/*
 * find vertices
 */
public:
	void 	findNearestVertex(double *newSample, vertex **nearestVertex, int numofDOFs);
	void 	findConfigurationForVertex(int ID, double *nextConfig, int numofDOFs);
	void 	findParentVertex(int childVertex, int *parentVertex);
	void 	createNeighborhood(vertex *newVertex, std::list<vertex*> *neighborHood, int numofDOFs);
	void 	addEdge(vertex *minVertex, int numofDOFs);

public:
	std::list<vertex> vertices;

};

#endif // TREE__