/**
 * This class responsible of planning the robot's path
 */

#ifndef PATHPLANNER_H_
#define PATHPLANNER_H_

#include <set>
#include <math.h>
#include "../Helpers/Structures.h"
#include "../Helpers/Constants.h"
#include "../NodeMap/NodeMap.h"

using namespace std;

class PathPlanner
{
public:
	double calculateDistance(Node* source, Node* target);
	void initializeHuristicValues(NodeMap* map, Node* goalNode);
	bool isNodeInList(set<Node>* list, int rowIndex, int colIndex);
	void calculateNeighbors(NodeMap* map, Node* currNode, Node* goalNode,
			set<Node*>* openList, set<Node*>* closedList);
	void calculatePath(NodeMap* map, Node* startNode, Node* goalNode);
	std::list<Node* > getWaypoints(Node * start, Node * currNode);

private:
	Node* getMinimalFNode(set<Node*>* openList);
	double getSlope(Node* a, Node* b);
};

#endif  PATHPLANNER_H_

