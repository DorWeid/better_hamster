#ifndef POSITION_H_
#define POSITION_H_
#include <math.h>
#include "../Node/Node.h"
#include "Constants.h"
#include "Position.h"
#include "Structures.h"

Node ConvertToHamsterLocation(Node* waypoint)
{
	Node hamsterLocation(waypoint->x- ROBOT_START_X, waypoint->y - ROBOT_START_Y);

	return hamsterLocation;
}


#endif
