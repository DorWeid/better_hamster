#include "PathPlanner.h"

void PathPlanner::findShortestPath(NodeMap* map, Node* startNode, Node* goalNode)
{
	set<Node*> openList;
	set<Node*> closedList;
	Node* currNode;

	initializeHuristicValues(map, goalNode);
	closedList.insert(startNode);
	startNode->isInClosedList = true;
	handleNeighbors(map, startNode, goalNode, &openList, &closedList);

	while(!openList.empty())
	{
		currNode = getMinimalFNode(&openList);

		openList.erase(currNode);
		currNode->isInOpenList = false;
		closedList.insert(currNode);
		currNode->isInClosedList = true;

		handleNeighbors(map, currNode, goalNode, &openList, &closedList);
	}
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

double PathPlanner::calculateDistance(Node* source, Node* target)
{
	return sqrt(pow(source->x - target->x, 2) + pow(source->y - target->y, 2));
}

void PathPlanner::handleNeighbors(NodeMap* map, Node* currNode, Node* goalNode,
		set<Node*>* openList, set<Node*>* closedList)
{
	for (int rowIndex = currNode->y - 1; rowIndex <= currNode->y + 1; rowIndex++)
	{
		for (int colIndex = currNode->x - 1; colIndex <= currNode->x + 1; colIndex++)
		{
			//cout << "X: " << colIndex << " Y: " << rowIndex << endl;

			// Checks if we're out of bounds and if the current neighbor is not an obstacle
			if (colIndex >= 0 && rowIndex >= 0 &&
				colIndex < map->getWidth() && rowIndex < map->getHeight() &&
				!map->getNodeByCoordinates(colIndex, rowIndex)->isObstacle)
			{
				// Makes sure the current node is not scanned
				if (colIndex != currNode->x || rowIndex != currNode->y)
				{
					// Checks if the current neighbor is in the closed list

					//if (!isNodeInList(closedList, colIndex, rowIndex))
					//{
					Node* currNeighbor = map->getNodeByCoordinates(colIndex, rowIndex);

					if (!currNeighbor->isInClosedList)
					{
						double tempGCost =
							calculateDistance(currNode, currNeighbor) + currNode->g;

						// Checks if the current neighbor is already in the open list
						if (!currNeighbor->isInOpenList)
						{
							currNeighbor->g = tempGCost;
							currNeighbor->f = currNeighbor->h + tempGCost;
							currNeighbor->parent = currNode;

							// Checking if we have reached the goal
							if (goalNode->x == colIndex && goalNode->y == rowIndex)
							{
								openList->clear();
								return;
							}

							// Adding the node to the open list for the first time
							openList->insert(currNeighbor);
							currNeighbor->isInOpenList = true;
						}
						// The node was already in the open list, check if we found a shorter path
						else
						{
							if (tempGCost < currNeighbor->g)
							{
								currNeighbor->g = tempGCost;
								currNeighbor->f = currNeighbor->h + tempGCost;
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
	Node* minNode = *(openList->begin());
	Node* currNode;

	for (set<Node*>::iterator iter = openList->begin(); iter != openList->end(); iter++)
	{
		currNode = *iter;

		if (currNode->f < minNode->f)
		{
			minNode = currNode;
		}
	}

	return minNode;
}

std::list<Node* > PathPlanner::markWaypoints(Node * start, Node * currNode)
{
	std::list<Node* > waypoints;
	Node* firstNode, *secondNode, *thirdNode;
	firstNode = currNode;
	secondNode = currNode->parent;
	int skipCounter = 0;

	if(secondNode == NULL)
	{
		return waypoints;
	}

	while (firstNode->x != start->x || firstNode->y != start->y)
	{

		thirdNode = secondNode->parent;

		if(thirdNode == NULL)
		{
			waypoints.push_back(secondNode);
			return waypoints;
		}

		if((getShipua(firstNode,secondNode) != getShipua(secondNode,thirdNode)) ||
				skipCounter >= WAYPOINT_TOLERENCE)
		{
			secondNode->isWaypoint = true;
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

double PathPlanner::getShipua(Node *a, Node * b)
{
	if (b->x - a->x == 0)
		return 0;

	return (b->y - a->y) / (b->x - a->x);
}



