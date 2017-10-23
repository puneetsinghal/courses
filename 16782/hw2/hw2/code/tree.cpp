#include <math.h>
#include "mex.h"
#include "helperFunctions.hpp"
#include "tree.hpp"


void tree::findNearestVertex(double *newSample, vertex **nearestVertex, int numofDOFs)
{
  double shortestDistance = 10000.0;
  double distance = 0;
  int i = 0;
  double difference[numofDOFs];
  
  for (std::list<vertex>::iterator it= vertices.begin(); it != vertices.end(); ++it)
  {
    distance = 0;
    for (i = 0; i < numofDOFs; ++i)
    {
      distance += pow(findMinorArc((newSample[i] - it->theta[i])),2);
    }

    distance = pow(distance,0.5);
    
    if (distance < shortestDistance)
    {
      shortestDistance = distance;
      *nearestVertex = &(*it);
    }
  }
}

void tree::findConfigurationForVertex(int ID, double *nextConfig, int numofDOFs)
{
  int i = 0;
  for (std::list<vertex>::iterator it= vertices.begin(); it != vertices.end(); ++it)
  {    
    if (ID == it->vertexID)
    { 
      for (i = 0; i < numofDOFs; ++i)
        nextConfig[i] = it->theta[i];
      return;
    }
  }
}

void tree::findParentVertex(int childVertex, int *parentVertex)
{
  for (std::list<vertex>::iterator it= vertices.begin(); it != vertices.end(); ++it)
  {    
    if (it->vertexID == childVertex)
    { 
      *parentVertex = it->parentID;
      return;
    }
  }
}

void tree::createNeighborhood(vertex *newVertex, 
  std::list<vertex*> *neighborHood,
  int numofDOFs)
{
  int numVertices = vertices.size();
  double radius = MIN((GAMMA*log(numVertices)/(DELTA*numVertices)), numVertices);
  for (std::list<vertex>::iterator it= vertices.begin(); it != vertices.end(); ++it)
  {
    if(distance(it->theta, newVertex->theta, numofDOFs) < radius)
      neighborHood->push_back(&(*it));
  }
  return;
}

void tree::addEdge(vertex *minVertex, int numofDOFs)
{
  vertex *newVertex = &vertices.back();
  newVertex->parentID = minVertex->vertexID;
  newVertex->parentAddress = minVertex;
  newVertex->cost = minVertex->cost + distance(newVertex->theta, minVertex->theta, numofDOFs);
  minVertex->children.push_back(newVertex);
  return;
}