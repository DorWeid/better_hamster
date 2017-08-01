/*
 * MovementManager.h
 *
 *  Created on: Jul 2, 2017
 *      Author: user
 */

#ifndef MOVEMENTMANAGER_H_
#define MOVEMENTMANAGER_H_

#include "../Helper/AngleUtils.h"
#include "HamsterAPIClientCPP/Hamster.h"
#include "../Robot/Robot.h"
#include <vector>
#include <math.h>
#include "Constants.h"
using namespace std;
using namespace HamsterAPI;

#define DISTANCE_FROM_WAYPOINT_TOLERANCE 5

class MovementManager
{
	
public:
	MovementManager(Robot * robot, MapDrawer* mapDrawer);
	void NavigateToWaypoint(Node * waypoint);
	virtual ~MovementManager();
private:
	Robot * robot;
	MapDrawer* mapDrawer;
	Node * waypoint;
	double distanceFromWaypoint;
	double targetYaw, deltaYaw;

	void turnToTarget();
	void moveForward();
	double GetAdjustedYaw(double yawToAdjust) const;
	double calculateTurningDirection();
	void recalculateDistanceFromTarget();
	double calculateTurnSpeed();
	double calculateForwardSpeed();
	bool isNeedAngleAdjustment();
	bool isDeltaAngleOnEndOfCiricle();
	float calculateWheelsAngle();
	void calculateTargetYaw(Node* waypoint);
	void stopMoving();
	void recalculateDeltaYaw();
};

#endif /* MOVEMENTMANAGER_H_ */
