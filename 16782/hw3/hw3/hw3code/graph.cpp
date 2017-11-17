#include <math.h>
#include "mex.h"
#include "graph.hpp"


graph::graph(int* blocksV, int numofblocks, int* trianglesV, int numoftriangles, int TableIndex, 
            int** onV_start, int onV_start_length, int* clearV_start, int numofclear_start, 
            int** onV_goal, int onV_goal_length, int* clearV_goal, int numofclear_goal)
{
  int i;
  for (i = 0; i < numofblocks; ++i)
    blocks.push_back(blocksV[i]);

  for (i = 0; i < numoftriangles; ++i)
    triangles.push_back(trianglesV[i]);

  table = TableIndex;
  
  literalsStruct startNode;

  for (i = 0; i < numofclear_start; ++i)
    startNode.clearLiterals.push_back(clearV_start[i]);

  startNode.clearLiterals.sort();
  
  literals onLiteralNew;
  for (i = 0; i < onV_start_length; ++i)
  {
    onLiteralNew.ON[0] = onV_start[i][0];
    onLiteralNew.ON[1] = onV_start[i][1];
    startNode.onLiterals.push_back(onLiteralNew);
  }
  startNode.onLiterals.sort();

  startNode.gValue = 0;

  startNode.parentAddress = NULL;
  int heuristicValue =0;
  // findHeuristicValue(startNode, &heuristicValue);
  startNode.hValue = heuristicValue;
  startNode.fValue = startNode.gValue + startNode.hValue;
  startNode.plan[0] = 0;
  startNode.plan[1] = 0;
  startNode.plan[2] = 0;
  startNode.plan[3] = 0;
  gVertices.push_back(startNode);
  openList.push(&gVertices.back());

  // findHeuristicValue(startNode, *startNode.hValue);

  for (i = 0; i < numofclear_goal; ++i)
    goalNode.clearLiterals.push_back(clearV_goal[i]);

  for (i = 0; i < onV_goal_length; ++i)
  {
    onLiteralNew.ON[0] = onV_goal[i][0];
    onLiteralNew.ON[1] = onV_goal[i][1];
    goalNode.onLiterals.push_back(onLiteralNew);
  }
  goalNode.gValue = 0;
  mexPrintf("initialized \n");
}

graph::graph(literalsStruct currentNode)
{
  gVertices.push_back(currentNode);
  openList.push(&(gVertices.back()));
}

bool isBlockClear(literalsStruct *currentNode, int blockID)
{
  for (std::list<int>::iterator it= (currentNode->clearLiterals).begin(); it != (currentNode->clearLiterals).end(); ++it)
  {
    if (*it == blockID)
    {
      return true;
    }
  }
  return false;
}

bool graph::isBlock(int blockID)
{
  for (std::list<int>::iterator it= blocks.begin(); it != blocks.end(); ++it)
  {
    if (*it == blockID)
    {
      return true;
    }
  }
  return false;
}

bool isBlockXonBlockY(literalsStruct *currentNode, int topBlock, int baseBlock)
{
  for (std::list<literals>::iterator it= (currentNode->onLiterals).begin(); it != (currentNode->onLiterals).end(); ++it)
  {
    if (it->ON[0] == topBlock && it->ON[1] == baseBlock)
    {
      return true;
    }
  }
  return false;
}

void addLiteral(literalsStruct* currentNode, literals newLiteral)
{
  for (std::list<literals>::iterator it= (currentNode->onLiterals).begin(); it != (currentNode->onLiterals).end(); ++it)
  {
    if((it->ON[0] == newLiteral.ON[0]) && (it->ON[1] == newLiteral.ON[1]))
    {
      return;
    }
  }
  (currentNode->onLiterals).push_back(newLiteral);
  return;
}

void graph::moveTo(literalsStruct* currentNode, int x, int y, int z)
{
  // check that Z is clear
  if (!isBlockClear(currentNode, z))
    return;

  // check that Z is not triangle
  // bool ZisTriangle = false;
  // for (std::list<int>::iterator it= triangles.begin(); it != triangles.end(); ++it)
  // {
  //   if (*it == z)
  //   {
  //     ZisTriangle = true;
  //   }
  // }

  // if (ZisTriangle || z==table)
  //   return;
  if(!isBlock(z))
    return;

  // check that X is clear
  if (!isBlockClear(currentNode,x))
    return;

  // check that x is on y
  if (!isBlockXonBlockY(currentNode, x, y))
    return;

  literals newLiteral;
  newLiteral.ON[0] = x;
  newLiteral.ON[1] = z;
  literalsStruct newNode;
  newNode = *currentNode;
  for (std::list<literals>::iterator it= (newNode.onLiterals).begin(); it != (newNode.onLiterals).end(); ++it)
  {
    if (it->ON[0] == x && it->ON[1] == y)
    {
      newNode.onLiterals.erase(it);
    }
  }
  newNode.onLiterals.push_back(newLiteral);
  // newNode.onLiterals.sort();
  newNode.clearLiterals.remove(z);
  if(y!=table)
  {
    newNode.clearLiterals.push_back(y);
  }
  // newNode.clearLiterals.sort();
  newNode.gValue = currentNode->gValue + 1;
  newNode.parentAddress = currentNode;
  newNode.plan[0] = 0;
  newNode.plan[1] = x;
  newNode.plan[2] = y;
  newNode.plan[3] = z;
  int heuristicValue = 0;
  findHeuristicValue(&newNode, &heuristicValue);
  newNode.hValue = heuristicValue;
  newNode.fValue = newNode.gValue + newNode.hValue;
  gVertices.push_back(newNode);
  openList.push(&gVertices.back());

  // mexPrintf("Action taken is: MoveTo(%d, %d, %d) \n", x, y, z);
}

void graph::heuristicMoveTo(graph* heuristicGraph, literalsStruct *currentNode, int x, int y, int z)
{
  // check that Z is clear
  if (!isBlockClear(currentNode, z))
    return;

  // check that Z is not triangle
  // bool ZisTriangle = false;
  // for (std::list<int>::iterator it= triangles.begin(); it != triangles.end(); ++it)
  // {
  //   if (*it == z)
  //   {
  //     ZisTriangle = true;
  //   }
  // }

  // if (ZisTriangle || z==table)
  //   return;

  if(!isBlock(z))
    return;

  // check that X is clear
  if (!isBlockClear(currentNode,x))
    return;

  // check that x is on y
  if (!isBlockXonBlockY(currentNode, x, y))
    return;

  literals newLiteral;

  newLiteral.ON[0] = x;
  newLiteral.ON[1] = z;

  literalsStruct newNode;
  newNode = *currentNode;
  // newNode.onLiterals.push_back(newLiteral);
  addLiteral(&newNode, newLiteral);
  // newNode.onLiterals.sort();
  if(y!=table)
  {
    newNode.clearLiterals.remove(y);
    newNode.clearLiterals.push_back(y);
  }
  // newNode.clearLiterals.sort();
  newNode.gValue = currentNode->gValue + 1;
  
  int heuristicValue =0;
  distanceFromGoal(&newNode,&heuristicValue);

  newNode.hValue = heuristicValue;
  newNode.fValue = newNode.gValue + newNode.hValue;
  // mexPrintf("Move TO function \n");
  // mexPrintf("current node is: ");
  // print_vertex(currentNode);
  // mexPrintf("New node is: ");
  // print_vertex(newNode);
  (heuristicGraph->gVertices).push_back(newNode);
  (heuristicGraph->openList).push(&((heuristicGraph->gVertices).back()));
}

void graph::moveToTable(literalsStruct* currentNode, int x, int y)
{
  // if(y == table)
  // {
  //   return;
  // }
  if(!isBlock(y))
    return;

  // check that X is clear
  if (!isBlockClear(currentNode,x))
    return;

  // check that x is on y
  if (!isBlockXonBlockY(currentNode, x, y))
    return;

  literals newLiteral;

  newLiteral.ON[0] = x;
  newLiteral.ON[1] = table;

  literalsStruct newNode;
  newNode = *currentNode;
  for (std::list<literals>::iterator it= (newNode.onLiterals).begin(); it != (newNode.onLiterals).end(); ++it)
  {
    if (it->ON[0] == x && it->ON[1] == y)
    {
      newNode.onLiterals.erase(it);
    }
  }
  newNode.onLiterals.push_back(newLiteral);
  // newNode.onLiterals.sort();
  // newNode.clearLiterals.remove(y);
  newNode.clearLiterals.push_back(y);
  // newNode.clearLiterals.sort();
  newNode.gValue = currentNode->gValue + 1;
  newNode.parentAddress = currentNode;
  newNode.plan[0] = 1;
  newNode.plan[1] = x;
  newNode.plan[2] = y;
  newNode.plan[3] = table;
  int heuristicValue =0;
  findHeuristicValue(&newNode, &heuristicValue);
  newNode.hValue = heuristicValue;
  newNode.fValue = newNode.gValue + newNode.hValue;
  gVertices.push_back(newNode);
  openList.push(&gVertices.back());
  // mexPrintf("Action taken is: MoveToTable(%d, %d) \n", x, y);
}

void graph::heuristicMoveToTable(graph* heuristicGraph, literalsStruct *currentNode, int x, int y)
{
  // if(y == table)
  // {
  //   return;
  // }
  if(!isBlock(y))
    return;
  // check that X is clear
  if (!isBlockClear(currentNode,x))
    return;

  // check that X is on Y
  if (!isBlockXonBlockY(currentNode, x, y))
    return;

  literals newLiteral;

  newLiteral.ON[0] = x;
  newLiteral.ON[1] = table;

  literalsStruct newNode;
  newNode = *currentNode;
  // newNode.onLiterals.push_back(newLiteral);
  addLiteral(&newNode, newLiteral);
  // newNode.onLiterals.sort();
  newNode.clearLiterals.remove(y);
  newNode.clearLiterals.push_back(y);
  // newNode.clearLiterals.sort();
  newNode.gValue = currentNode->gValue + 1;
  int heuristicValue = 0;
  distanceFromGoal(&newNode,&heuristicValue);

  newNode.hValue = heuristicValue;
  newNode.fValue = newNode.gValue + newNode.hValue;
  // mexPrintf("current node is: ");
  // print_vertex(currentNode);
  // mexPrintf("New node is: ");
  // print_vertex(newNode);
  (heuristicGraph->gVertices).push_back(newNode);

  (heuristicGraph->openList).push(&((heuristicGraph->gVertices).back()));
}

bool graph::findBaseBlock(literalsStruct *currentNode, int x, int* y)
{
  for (std::list<literals>::iterator it= (currentNode->onLiterals).begin(); it != (currentNode->onLiterals).end(); ++it)
  {
    if(it->ON[0] == x)
    {
      *y = it->ON[1];
      return true;
    }
  }
  return false;
}

bool graph::findBaseBlock(literalsStruct *currentNode, int x, std::list<int> *baseBlocks)
{
  for (std::list<literals>::iterator it= (currentNode->onLiterals).begin(); it != (currentNode->onLiterals).end(); ++it)
  {
    if(it->ON[0] == x)
    {
      baseBlocks->push_back(it->ON[1]);
      // return true;
    }
  }
  if(baseBlocks->size()>0)
    return true;
  else
    return false;
}

void graph::takeAction(literalsStruct* currentNode)
{
  int x, y, z;
  // for (std::list<int>::iterator it1= (currentNode->clearLiterals).begin(); it1 != (currentNode->clearLiterals).end(); ++it1)
  // {
  //   x = *it1;
  //   // mexPrintf("x is %d \n",x);
  //   if(!findBaseBlock(currentNode, x, &y))
  //   {
  //     mexPrintf("Fault finding base block for block: \n", x);
  //   }
  //   for (std::list<int>::iterator it2= (currentNode->clearLiterals).begin(); it2 != (currentNode->clearLiterals).end(); ++it2)
  //   {
  //     if(*it2 != *it1)
  //     {
  //       z = *it2;
  //       // mexPrintf("z is %d \n",z);
  //       moveTo(currentNode, x, y, z);
  //     }
  //   }
  //   if (y != table)
  //   {
  //     moveToTable(currentNode, x, y);
  //   }
  // }
  for (x = 0; x < table; ++x)
  {
    for (y = 0; y <= table; ++y)
    {  
      for (z = 0; z < table; ++z)
      {
        if(x!=z && x!=y)
        {        
          moveTo(currentNode, x, y, z);
        }
      }
      if(x!=y && y!=table)
      {
        moveToTable(currentNode, x, y);
      }
    }
  }
  return;
}

void graph::heuristicTakeAction(graph* heuristicGraph, literalsStruct *currentNode)
{
  int x, y, z;
  // std::list<int> baseBlocks;
  // for (std::list<int>::iterator it1= (currentNode->clearLiterals).begin(); it1 != (currentNode->clearLiterals).end(); ++it1)
  // {
  //   x = *it1;
  //   // mexPrintf("x is %d \n",x);
  //   if(!findBaseBlock(currentNode, x, &baseBlocks))
  //   {
  //     mexPrintf("Fault finding base block for: %d\n", x);
  //   }
  //   for (std::list<int>::iterator it2= (currentNode->clearLiterals).begin(); it2 != (currentNode->clearLiterals).end(); ++it2)
  //   {
  //     if(*it2 != *it1)
  //     {
  //       z = *it2;
  //       // mexPrintf("z is %d \n",z);
  //       for (std::list<int>::iterator itBase= baseBlocks.begin(); itBase != baseBlocks.end(); ++itBase)
  //       {
  //         y = *itBase;
  //         heuristicMoveTo(heuristicGraph, currentNode, x, y, z);
  //       }
  //     }
  //   }
  //   for (std::list<int>::iterator itBase= baseBlocks.begin(); itBase != baseBlocks.end(); ++itBase)
  //   {
  //     y = *itBase;
  //     if (y != table)
  //     {
  //       heuristicMoveToTable(heuristicGraph, currentNode, x, y);
  //     }
  //   }
  // }
  for (x = 0; x < table; ++x)
  {
    for (y = 0; y <= table; ++y)
    {  
      for (z = 0; z < table; ++z)
      {
        if(x!=z && x!=y)
        {        
          heuristicMoveTo(heuristicGraph, currentNode, x, y, z);
        }
      }
      if(x!=y && y!=table)
      {
        heuristicMoveToTable(heuristicGraph, currentNode, x, y);
      }
    }
  }
  return;
}

// bool graph::stateExists(literalsStruct *currentNode)
// {
//   int counter;
//   bool clearLiteralsSame = false;
//   bool onLiteralsSame = false;
//   for(std::list<literalsStruct> iterator it = gVertices.begin(); it != gVertices.end(); it++)
//   {
//     counter = 0;
//     if(currentNode->clearLiterals.size() != it->clearLiterals.size())
//       continue;

//     if(currentNode->onLiterals.size() != it->onLiterals.size())
//       continue;

//     for(std::list<int> iterator it2=currentNode->clearLiterals.begin(); it2!= currentNode->clearLiterals.end(); it2++)
//     {
//       if(isBlockClear(currentNode,*it2))
//       {
//         counter++;
//       }
//     }
//     if(counter==currentNode->clearLiterals.size())
//     {
//       clearLiteralsSame = true;
//     }

//     counter = 0;

//     for(std::list<literals> iterator it3=currentNode->onLiterals.begin(); it2!= currentNode->onLiterals.end(); it3++)
//     {
//       if(isBlockXonBlockY(currentNode, it3.ON[0], it3.ON[1]))
//       {
//         counter++;
//       }
//     }
//     if(counter==currentNode->onLiterals.size())
//     {
//       onLiteralsSame = true;
//     }
//     if(clearLiteralsSame && onLiteralsSame)
//     {
//       if(currentNode->fValue < it->fValue)
//       {
//         gVertices.erase(it);
//       }
//       return true;
//     }

//     onLiteralsSame = false;
//     clearLiteralsSame = false;
//   }
//   return false;
// }

bool graph::reachedGoalState(literalsStruct *currentNode)
{
  for (std::list<int>::iterator itGoal= (goalNode.clearLiterals).begin(); itGoal != (goalNode.clearLiterals).end(); ++itGoal)
  {
    if(!isBlockClear(currentNode,*itGoal))
      return false;
  }

  for (std::list<literals>::iterator itGoal= (goalNode.onLiterals).begin(); itGoal != (goalNode.onLiterals).end(); ++itGoal)
  {
    if(!isBlockXonBlockY(currentNode, itGoal->ON[0], itGoal->ON[1]))
      return false;
  }
  return true;
}

void graph::distanceFromGoal(literalsStruct *currentNode, int* distance)
{
  int heuristicValue = 0;
  for (std::list<literals>::iterator itGoal= (goalNode.onLiterals).begin(); itGoal != (goalNode.onLiterals).end(); ++itGoal)
  {
    for (std::list<literals>::iterator it= (currentNode->onLiterals).begin(); it != (currentNode->onLiterals).end(); ++it)
    {
      if((it->ON[0]==itGoal->ON[0]) && (it->ON[1]==itGoal->ON[1]))
        heuristicValue++;
    }
  }

  for (std::list<int>::iterator itGoal= (goalNode.clearLiterals).begin(); itGoal != (goalNode.clearLiterals).end(); ++itGoal)
  {
    for (std::list<int>::iterator it= (currentNode->clearLiterals).begin(); it != (currentNode->clearLiterals).end(); ++it)
    {
      if(*it == *itGoal)
        heuristicValue++;
    }
  }
  *distance = goalNode.clearLiterals.size() + goalNode.onLiterals.size() - heuristicValue;
}

void graph::findHeuristicValue(literalsStruct *baseNode, int* heuristicValue)
{
  literalsStruct currNode;
  currNode = *baseNode;
  currNode.gValue = 0;
  currNode.fValue = 0;
  graph heuristicGraph(currNode);
  (heuristicGraph.openList).push(&((heuristicGraph.gVertices).front()));

  literalsStruct *currentNode;
  currentNode = heuristicGraph.openList.top();
  // heuristicGraph.openList.pop();

  // bool reachedGoal;
  // reachedGoal = reachedGoalState(currentNode);
  int i = 0;
  while(!reachedGoalState(currentNode) && !heuristicGraph.openList.empty())
  {
    heuristicTakeAction(&heuristicGraph, currentNode);
    heuristicGraph.openList.pop();
    currentNode = heuristicGraph.openList.top();
    // heuristicGraph.printGraph();
    // reachedGoal = reachedGoalState(currentNode);
    i++;
    // if(i>3)
    //   return;
  }
  // heuristicGraph.printGraph();
  // mexPrintf("number of states in heuristic finder is: %d \n", (heuristicGraph.gVertices).size());
  *heuristicValue = currentNode->gValue;
  return;
}

void graph::printVertex(literalsStruct currentNode)
{
  int i = 0;
  mexPrintf("On Literals are: ");
  for (std::list<literals>::iterator it= (currentNode.onLiterals).begin(); it != (currentNode.onLiterals).end(); ++it)
  {
    mexPrintf("[%d, %d];  ", it->ON[0], it->ON[1]);
  }
  mexPrintf("\nClear Literals are: ");
  for (std::list<int>::iterator it= (currentNode.clearLiterals).begin(); it != (currentNode.clearLiterals).end(); ++it)
  {
    mexPrintf("%d    ",*it);
  }
  mexPrintf("\n");
  mexPrintf("G value of state is: %d \n", currentNode.gValue);
  mexPrintf("H value of state is: %d \n", currentNode.hValue);
  mexPrintf("F value of state is: %d \n", currentNode.fValue);
  mexPrintf("Plan is [%d, %d, %d, %d] \n", currentNode.plan[0],currentNode.plan[1],currentNode.plan[2],currentNode.plan[3]);
}

void graph::printGraph()
{
  int i = 0;
  int stateNum = 1;
  mexPrintf("\n *******************PRINTING GRAPH********************** \n");
  for (std::list<literalsStruct>::iterator it= gVertices.begin(); it != gVertices.end(); ++it)
  {
    mexPrintf("\n State %d: \n", stateNum);
    printVertex(*it);
    stateNum++;
  }
}

void graph::printList()
{
  int i = 0;
  int stateNum = 1;
  std::priority_queue<literalsStruct*, std::vector<literalsStruct*>, Comparator> tempOpenList;
  tempOpenList = openList;
  literalsStruct* currentNode;
  currentNode = tempOpenList.top();
  // tempOpenList.pop();
  mexPrintf("\n *******************PRINTING OPEN LIST********************** \n");
  while(!tempOpenList.empty())
  {
    mexPrintf("\n State: %d with address: %p \n", stateNum, currentNode);
    printVertex(*currentNode);
    tempOpenList.pop();
    currentNode = tempOpenList.top();
    stateNum++;
    // if (stateNum==100)
    //   return;
  }
}