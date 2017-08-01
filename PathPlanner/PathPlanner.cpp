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
	startingPosition->setIsInClosedList(true);
	calculateNeighbors(map, startingPosition, targetPosition, &openList, &closedList);

	while(!openList.empty())
	{
		currNode = getMinimalFNode(&openList);

		openList.erase(currNode);
		currNode->setIsInOpenList(false);
		closedList.insert(currNode);
		currNode->setIsInClosedList(true);

		calculateNeighbors(map, currNode, targetPosition, &openList, &closedList);

	}
}

void PathPlanner::initializeHuristicValues(NodeMap* map, Node* goalNode)
{
	for (int rowIndex = 0; rowIndex < map->getHeight(); rowIndex++)
	{
		for (int colIndex = 0; colIndex < map->getWidth(); colIndex++)
		{
			Node* currNode = map->getNodeAtIndex(colIndex, rowIndex);

			if (!currNode->getIsObstacle())
			{
				currNode->setH(calculateDistance(currNode, goalNode));
			}
		}
	}
}

// calculates the distance between two nodes
double PathPlanner::calculateDistance(Node* start, Node* target)
{
	return sqrt(pow(start->getX() - target->getX(), 2) + pow(start->getY() - target->getY(), 2));
}

// calculates the cost of the surrounding cells and updates the path as well
void PathPlanner::calculateNeighbors(NodeMap* map, Node* currNode, Node* targetNode,
		set<Node*>* openList, set<Node*>* closedList)
{
	// runs through the rows behind and after current node
	for (int rowIndex = currNode->getY() - 1; rowIndex <= currNode->getY() + 1; rowIndex++)
	{
		// runs through the cols behind and after current node
		for (int colIndex = currNode->getX() - 1; colIndex <= currNode->getX() + 1; colIndex++)
		{
			// checks if we reached the bounds or the node is obstacle
			if (rowIndex >= 0 && colIndex >= 0 &&  
				colIndex < map->getWidth() && rowIndex < map->getHeight() &&
				!map->getNodeAtIndex(colIndex, rowIndex)->getIsObstacle())
			{
				// we dont check the current node
				if (colIndex != currNode->getX() || rowIndex != currNode->getY())
				{
					Node* currNeighbor = map->getNodeAtIndex(colIndex, rowIndex);
					
					// checks if the neighbor is not in the closed list
					if (!currNeighbor->getIsInClosedList())
					{
						double gCost =
							calculateDistance(currNode, currNeighbor) + currNode->getG();

						// Checks if the current neighbor is not in the open list
						if (!currNeighbor->getIsInOpenList())
						{
							currNeighbor->setG(gCost);
							currNeighbor->setF(currNeighbor->getH() + gCost);
							currNeighbor->setParent(currNode);

							// Checking if we have reached the goal
							if (targetNode->getX() == colIndex && targetNode->getY() == rowIndex)
							{
								// clears the openList and done with checking the nodes
								openList->clear();
								return;
							}

							// Adding the node to the open list
							openList->insert(currNeighbor);
							currNeighbor->setIsInOpenList(true);
						}
						// check if we found a shorter path since the node is already in the open list
						else
						{
							if (gCost < currNeighbor->getG())
							{
								// updates the cost and the parent node
								currNeighbor->setG(gCost);
								currNeighbor->setF(currNeighbor->getH() + gCost);
								currNeighbor->setParent(currNode);
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

	for (set<Node*>::iterator iter = openList->begin(); iter != openList->end(); iter++)
	{
		if ((*iter)->getF() < minNode->getF())
		{
			minNode = *iter;
		}
	}

	return minNode;
}

std::list<Node* > PathPlanner::markWaypoints(Node * start, Node * currNode)
{
	std::list<Node* > waypoints;
	Node* firstNode, *secondNode, *thirdNode;
	firstNode = currNode;
	secondNode = currNode->getParent();
	int skipCounter = 0;

	if(secondNode == NULL)
	{
		return waypoints;
	}

	while (firstNode->getX() != start->getX() || firstNode->getY() != start->getY())
	{

		thirdNode = secondNode->getParent();

		if(thirdNode == NULL)
		{
			waypoints.push_back(secondNode);
			return waypoints;
		}

		if((getSlope(firstNode,secondNode) != getSlope(secondNode,thirdNode)) ||
				skipCounter >= WAYPOINT_TOLERENCE)
		{
			secondNode->setIsWaypoint(true);
			waypoints.push_back(secondNode);
			skipCounter = 0;
		}
		else
		{
			skipCounter++;
		}


		firstNode = secondNode;
		secondNode = thirdNode;
	}

	return waypoints;
}

// returns the slope between two nodes
double PathPlanner::getSlope(Node *a, Node * b)
{
	// checks if there's slope
	if (b->getX() - a->getX() == 0)
		return 0;

	return (b->getY() - a->getY()) / (b->getX() - a->getX());
}



