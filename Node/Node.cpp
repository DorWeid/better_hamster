#include "Node.h"
#include "stdlib.h"

Node::Node() :
	parent(NULL), x(0), y(0), g(0), h(0), f(0), isObstacle(false), isInClosedList(false), isInOpenList(false), isWaypoint(false) {}

Node::Node(double x, double y) :
	Node()
{
	this->x = x;
	this->y = y;
}

Node::~Node() {}
