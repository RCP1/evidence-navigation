// STL A* Search implementation
// (C)2001 Justin Heyes-Jones
// Changed by Robin Petereit

#ifndef MAPSEARCHNODE_H
#define MAPSEARCHNODE_H

#include "stlastar.h" // See header for copyright and usage information

#include <iostream>
#include <stdio.h>
#include "gridmap.h"


class MapSearchNode
{
public:
	int x;	 // the (x,y) positions of the node
	int y;
  MapSearchNode();
	MapSearchNode(GridMap* gridMap);
	MapSearchNode(GridMap* gridMap, int px, int py);

  int GetMap( int x, int y );
	int GoalDistanceEstimate( MapSearchNode &nodeGoal );
	bool IsGoal( MapSearchNode &nodeGoal );
	bool GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node );
	int GetCost( MapSearchNode &successor );
	bool IsSameState( MapSearchNode &rhs );

	void PrintNodeInfo();
  GridMap* gridMap_;
};

#endif // MAPSEARCHNODE_H