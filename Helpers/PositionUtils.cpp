#ifndef UTILS_POSITIONUTILS_H_
#define UTILS_POSITIONUTILS_H_
#include <math.h>
#include "../Node/Node.h"
#include "Constants.h"
#include "Structures.h"

Node ConvertToHamsterLocation(Node* waypoint)
{
	Node hamsterLocation(waypoint->x- ROBOT_START_X, waypoint->y - ROBOT_START_Y);

	return hamsterLocation;
}


#endif  UTILS_POSITIONUTILS_H_
