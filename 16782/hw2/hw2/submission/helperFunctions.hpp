#ifndef HELPER_FUNCTIONS_H__
#define HELPER_FUNCTIONS_H__
#include <list>

// typedef struct vertex{
//     double theta[5];
//     int vertexID;
//     double cost; 
//     std::list<vertex*> children;
//     int parentID;
//     vertex* parentAddress;
// } vertex;

// typedef struct edge {
//     double startID;
//     double endID;
// } edge;

#define PI 3.141592654

#define DELTA 5.264

#define GAMMA 1.5

double findMinorArc(double difference);

double distance(double* startConfig, double* endConfig, int numofDOFs);

void print_joints(double * theta, int numofDOFs);

#endif // HELPER_FUNCTIONS_H__