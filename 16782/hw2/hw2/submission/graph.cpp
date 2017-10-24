#include <math.h>
#include "mex.h"
#include "helperFunctions.hpp"
#include "tree.hpp"
#include "graph.hpp"



void graph::addVertexGraph(double *sample, int idNumber, int numofDOFs)
{	
	graphVertex newVertex;
	int i;
	for(i = 0; i < numofDOFs; ++i)
		newVertex.theta[i] = sample[i];

	newVertex.vertexID = idNumber;
	newVertex.numConnected = 0;
	newVertex.isClosed = false;
	newVertex.gValue = 10000.0;
	newVertex.parentAddress = NULL;
	gVertices.push_back(newVertex);
	return;
}

// void graph::addEdgeGraph(graphVertex **newVertex, graphVertex **nearVertex, int numofDOFs)
// {
// 	graphEdge newEdge;
// 	newEdge.startAddress = *newVertex;
// 	newEdge.endAddress = *nearVertex;
// 	double cost = distance((*newVertex)->theta, (*nearVertex)->theta, numofDOFs);
// 	newEdge.cost = cost;
// 	gEdges.push_back(newEdge);
// 	// print_vertex(*newVertex, numofDOFs);
//     // print_vertex(*nearVertex, numofDOFs);

//     // printLastEdge(numofDOFs);
//     graphEdge *edgePointer = &(gEdges.back());
// 	(*newVertex)->numConnected = (*newVertex)->numConnected + 1;
// 	(*nearVertex)->numConnected = (*nearVertex)->numConnected + 1;
// 	(*newVertex)->connections.push_back(edgePointer);
// 	(*nearVertex)->connections.push_back(edgePointer);

// }

void graph::addEdgeGraph(graphVertex *newVertex, graphVertex *nearVertex, int numofDOFs)
{
	graphEdge newEdge;
	newEdge.startAddress = newVertex;
	newEdge.endAddress = nearVertex;
	double cost = distance((newVertex)->theta, (nearVertex)->theta, numofDOFs);
	newEdge.cost = cost;
	gEdges.push_back(newEdge);
	// print_vertex(*newVertex, numofDOFs);
    // print_vertex(*nearVertex, numofDOFs);

    // printLastEdge(numofDOFs);
    graphEdge *edgePointer = &(gEdges.back());
	(newVertex)->numConnected = (newVertex)->numConnected + 1;
	(nearVertex)->numConnected = (nearVertex)->numConnected + 1;
	(newVertex)->connections.push_back(edgePointer);
	(nearVertex)->connections.push_back(edgePointer);

}

void graph::createNeighborhoodGraph(graphVertex **newVertex, 
	std::list<graphVertex*> *neighborHood,
  	int numofDOFs, double radius)
{

  for (std::list<graphVertex>::iterator it= gVertices.begin(); it != gVertices.end(); ++it)
  {
    // print_vertex(&(*it), numofDOFs);
    // print_vertex(*newVertex, numofDOFs);
    if(distance((*it).theta, (*newVertex)->theta, numofDOFs) > radius )
    {
    	// mexPrintf("distance is %g \n", distance(it->theta, (*newVertex)->theta, numofDOFs));
    	continue;    	
    }

    if(&(*it) == *newVertex)
    {
    	// mexPrintf("same point %d \n", (&(*it) == *newVertex));
    	continue;
    }
    // mexPrintf("distance is %g \n", distance(it->theta, (*newVertex)->theta, numofDOFs));
	// print_vertex(&(*it), numofDOFs);
  	neighborHood->push_back(&(*it));	
  }
  return;
}

void graph::printLastEdge(int numofDOFs)
{
  int i = 0;
  mexPrintf("\nStart node is:   ");
  graphVertex *tempVertex = (gEdges.back()).startAddress;
  for (i = 0; i < numofDOFs; ++i)
  {
    mexPrintf("%f    ", tempVertex->theta[i]);
  }

  mexPrintf("\n End node is:   ");
  tempVertex = (gEdges.back()).endAddress;
  for (i = 0; i < numofDOFs; ++i)
  {
    mexPrintf("%f    ",tempVertex->theta[i]);
  }
}

void graph::print_vertex(graphVertex *currentNode, int numofDOFs)
{
  int i = 0;
  mexPrintf("vertex is: \n");
  for (i = 0; i < numofDOFs; ++i)
  {
    mexPrintf("%g    ",currentNode->theta[i]);
  }
  mexPrintf("\n ID is: %d\n \n", currentNode->vertexID);
}
