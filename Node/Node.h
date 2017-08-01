/**
 * Represents a node on the map
 */

#ifndef NODE_H_
#define NODE_H_

#include <cstdlib>

class Node
{
	public:
		// Is the node an obstacle
		bool isObstacle;

		// Is the node a waypoint
		bool isWaypoint;

		//
		bool isInOpenList;
		bool isInClosedList;

		// the cost of the path from the start node to current node
		double g;

		// estimates the cost of the cheapest path from current node to goal node
		double h;
		// total weights of g and h
		double f;

		// Coordinates
		double x;
		double y;

		// Node's parent (previous link)
		Node* parent;

		Node();
		Node(double x, double y);
		~Node();

		// Override the '<' op
		bool operator<(const Node& node) const
		{
			if ((y < node.y) || (y == node.y && x <= node.x))
			{
				return true;
			}

			return false;
		}
};

#endif

