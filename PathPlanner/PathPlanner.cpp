#include "PathPlanner.h"

// Gets the map, starting position and target position
// Returns the shortest path
void PathPlanner::calculatePath(NodeMap* map, Node* startingPosition, Node* targetPosition)
{	
	set<Node*> openList;
	set<Node*> closedList;
	Node* currNode;

	initializeHuristicValues(map, targetPosition);
	closedList.insert(startingPosition);
	startingPosition->isInClosedList = true;
	calculateNeighbors(map, startingPosition, targetPosition, &openList, &closedList);

	while(!openList.empty())
	{
		currNode = getMinimalFNode(&openList);

		openList.erase(currNode);
		currNode->isInOpenList = false;
		closedList.insert(currNode);
		currNode->isInClosedList = true;

		calculateNeighbors(map, currNode, targetPosition, &openList, &closedList);
	}
}

// creates the waypoint array, loops through the planned route and creates waypoitns
std::list<Node* > PathPlanner::getWaypoints(Node * start, Node * currNode)
{
	std::list<Node* > waypoints;
	Node* firstNode, *secondNode, *thirdNode;
	firstNode = currNode;
	secondNode = currNode->parent;
	int skipped = 0;

	// if theres no parent to the target goal, it cannot be reached from start point
	if(secondNode == NULL)
	{
		return waypoints;
	}

	// runs until reaches start point
	while (firstNode->x != start->x || firstNode->y != start->y)
	{

		thirdNode = secondNode->parent;

		if(thirdNode == NULL)
		{
			waypoints.push_back(secondNode);
			return waypoints;
		}

		// checks if the second node changes the direction slope
		// if it does we need the create new waypoint
		if((getSlope(firstNode,secondNode) != getSlope(secondNode,thirdNode)) ||
				skipped >= WAYPOINT_TOLERENCE)
		{
			secondNode->isWaypoint = true;
			waypoints.push_back(secondNode);
			skipped = 0;
		}
		else
		{
			skipped++;
		}


		firstNode = secondNode;
		secondNode = thirdNode;
	}

	return waypoints;
}

void PathPlanner::initializeHuristicValues(NodeMap* map, Node* goalNode)
{
	for (int rowIndex = 0; rowIndex < map->getHeight(); rowIndex++)
	{
		for (int colIndex = 0; colIndex < map->getWidth(); colIndex++)
		{
			Node* currNode = map->getNodeByCoordinates(colIndex, rowIndex);

			if (!currNode->isObstacle)
			{
				currNode->h = calculateDistance(currNode, goalNode);
			}
		}
	}
}

// Calculates the distance between two nodes
double PathPlanner::calculateDistance(Node* start, Node* target)
{
	return sqrt(pow(start->x - target->x, 2) + pow(start->y - target->y, 2));
}

// calculates the cost of the surrounding cells and updates the path as well
void PathPlanner::calculateNeighbors(NodeMap* map, Node* currNode, Node* targetNode,
		set<Node*>* openList, set<Node*>* closedList)
{
	// runs through the rows behind and after current node
	for (int rowIndex = currNode->y - 1; rowIndex <= currNode->y + 1; rowIndex++)
	{
		// runs through the cols behind and after current node
		for (int colIndex = currNode->x - 1; colIndex <= currNode->x + 1; colIndex++)
		{
			// checks if we reached the bounds or the node is obstacle
			if (rowIndex >= 0 && colIndex >= 0 &&  
				colIndex < map->getWidth() && rowIndex < map->getHeight() &&
				!map->getNodeByCoordinates(colIndex, rowIndex)->isObstacle)
			{
				// we dont check the current node
				if (colIndex != currNode->x || rowIndex != currNode->y)
				{
					Node* currNeighbor = map->getNodeByCoordinates(colIndex, rowIndex);
					
					// checks if the neighbor is not in the closed list
					if (!currNeighbor->isInClosedList)
					{
						double gCost =
							calculateDistance(currNode, currNeighbor) + currNode->g;

						// Checks if the current neighbor is not in the open list
						if (!currNeighbor->isInOpenList)
						{
							currNeighbor->g = gCost;
							currNeighbor->f = currNeighbor->h + gCost;
							currNeighbor->parent = currNode;

							// Checking if we have reached the goal
							if (targetNode->x == colIndex && targetNode->y == rowIndex)
							{
								// clears the openList and done with checking the nodes
								openList->clear();
								return;
							}

							// Adding the node to the open list
							openList->insert(currNeighbor);
							currNeighbor->isInOpenList = true;
						}
						// check if we found a shorter path since the node is already in the open list
						else
						{
							if (gCost < currNeighbor->g)
							{
								// updates the cost and the parent node
								currNeighbor->g = gCost;
								currNeighbor->f = currNeighbor->h + gCost;
								currNeighbor->parent = currNode;
							}
						}
					}
				}
			}
		}
	}
}

Node* PathPlanner::getMinimalFNode(set<Node*>* openList)
{
	// starts as the first node
	Node* minNode = *(openList->begin());

	// iterates the open list and searches for the lowest f value
	for (set<Node*>::iterator iter = openList->begin(); iter != openList->end(); iter++)
	{
		if ((*iter)->f < minNode->f)
		{
			minNode = *iter;
		}
	}

	return minNode;
}

// returns the slope between two nodes
double PathPlanner::getSlope(Node *a, Node * b)
{
	// checks if there's slope
	if (b->x - a->x == 0)
		return 0;

	return (b->y - a->y) / (b->x - a->x);
}



