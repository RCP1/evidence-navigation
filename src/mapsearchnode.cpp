// STL A* Search implementation
// (C)2001 Justin Heyes-Jones
// Changed by Robin Petereit

#include "mapsearchnode.h"

using namespace std;


MapSearchNode::MapSearchNode()
{
  x = y = 0;
}

MapSearchNode::MapSearchNode(GridMap* gridMap)
  : gridMap_(gridMap)
{
  x = y = 0;
}


MapSearchNode::MapSearchNode(GridMap* gridMap, int px, int py)
  : gridMap_(gridMap)
{
  x = px;
  y = py;
}

int MapSearchNode::GetMap( int x, int y )
{
	if( x < 0 || x >= gridMap_->getSize() ||
		  y < 0 || y >= gridMap_->getSize())
	{
		return 9;
	}
  //std::cout << gridMap_->getOccupancy(x, y) << std::endl;

	return gridMap_->getOccupancy(x, y);
}

bool MapSearchNode::IsSameState( MapSearchNode &rhs )
{

	// same state in a maze search is simply when (x,y) are the same
	if( (x == rhs.x) &&
		(y == rhs.y) )
	{
		return true;
	}
	else
	{
		return false;
	}

}

void MapSearchNode::PrintNodeInfo()
{
	char str[100];
	sprintf( str, "Node position : (%d,%d)\n", x,y );

	cout << str;
}

// Here's the heuristic function that estimates the distance from a Node
// to the Goal.

int MapSearchNode::GoalDistanceEstimate( MapSearchNode &nodeGoal )
{
	int xd = int( abs( x - nodeGoal.x ) );
	int yd = int( abs( y - nodeGoal.y) );

	return xd + yd;

}

bool MapSearchNode::IsGoal( MapSearchNode &nodeGoal )
{

	if( (x == nodeGoal.x) &&
		(y == nodeGoal.y) )
	{
		return true;
	}

	return false;
}

// This generates the successors to the given Node. It uses a helper function called
// AddSuccessor to give the successors to the AStar class. The A* specific initialisation
// is done for each node internally, so here you just set the state information that
// is specific to the application
bool MapSearchNode::GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node )
{

	int parent_x = -1;
	int parent_y = -1;

	if( parent_node )
	{
		parent_x = parent_node->x;
		parent_y = parent_node->y;
	}


	MapSearchNode NewNode;

	// push each possible move except allowing the search to go backwards

	if( (GetMap( x-1, y ) < 9)
		&& !((parent_x == x-1) && (parent_y == y))
	  )
	{
		NewNode = MapSearchNode( gridMap_, x-1, y );
		astarsearch->AddSuccessor( NewNode );
	}

	if( (GetMap( x, y-1 ) < 9)
		&& !((parent_x == x) && (parent_y == y-1))
	  )
	{
		NewNode = MapSearchNode( gridMap_, x, y-1 );
		astarsearch->AddSuccessor( NewNode );
	}

	if( (GetMap( x+1, y ) < 9)
		&& !((parent_x == x+1) && (parent_y == y))
	  )
	{
		NewNode = MapSearchNode( gridMap_, x+1, y );
		astarsearch->AddSuccessor( NewNode );
	}


	if( (GetMap( x, y+1 ) < 9)
		&& !((parent_x == x) && (parent_y == y+1))
		)
	{
		NewNode = MapSearchNode( gridMap_, x, y+1 );
		astarsearch->AddSuccessor( NewNode );
	}

  /*if( (GetMap( x+1, y+1 ) < 9)
  && !((parent_x == x+1) && (parent_y == y+1))
  )
	{
		NewNode = MapSearchNode( gridMap_, x+1, y+1 );
		astarsearch->AddSuccessor( NewNode );
	}

  if( (GetMap( x-1, y+1 ) < 9)
  && !((parent_x == x-1) && (parent_y == y+1))
  )
	{
		NewNode = MapSearchNode( gridMap_, x-1, y+1 );
		astarsearch->AddSuccessor( NewNode );
	}

  if( (GetMap( x-1, y-1 ) < 9)
  && !((parent_x == x-1) && (parent_y == y-1))
  )
	{
		NewNode = MapSearchNode( gridMap_, x-1, y-1 );
		astarsearch->AddSuccessor( NewNode );
	}

  if( (GetMap( x+1, y-1 ) < 9)
  && !((parent_x == x+1) && (parent_y == y-1))
  )
	{
		NewNode = MapSearchNode( gridMap_, x+1, y-1 );
		astarsearch->AddSuccessor( NewNode );
	}*/

	return true;
}

// given this node, what does it cost to move to successor. In the case
// of our map the answer is the map terrain value at this node since that is
// conceptually where we're moving

int MapSearchNode::GetCost( MapSearchNode &successor )
{
	return GetMap( x, y );

}