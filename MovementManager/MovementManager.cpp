#include "MovementManager.h"
#include <stdio.h>
#include <stdlib.h>
#include <sstream>

MovementManager::MovementManager(Robot * robot, MapDrawer* mapDrawer) : 
				robot(robot), mapDrawer(mapDrawer)
{
}

// Navigate to a waypoint
void MovementManager::NavigateToWaypoint(Node* waypoint)
{
	this->waypoint = waypoint;
	
	// Calculate the robot location 
	robot->realLocation = robot->prevBeliefedLocation = robot->currBeliefedLocation = robot->getRealHamsterLocation();

	// According to the location of the robot calculate the distance from the waypoint
	recalculateDistanceFromTarget();

	// While the distance from the wayPoint is bigger than the tolerance distance
	while (distanceFromWaypoint > DISTANCE_FROM_WAYPOINT_TOLERANCE)
	{
		// The previous belified location of the robot is the current belifed location
		robot->prevBeliefedLocation = robot->currBeliefedLocation;
		
		// recalculate the current location 
		robot->realLocation = robot->currBeliefedLocation = robot->getRealHamsterLocation();
		
		// Recalculate the delta between last angle and current angle
		recalculateDeltaYaw();
		
		// Calculate the angle from the robot to the waypoint
		calculateTargetYaw(waypoint);

		// Check if the robot need to adjust is angle  
		if (isNeedAngleAdjustment())
		{
			cout << "Turning, targetYaw: " << targetYaw << " currYaw: " << robot->currBeliefedLocation.yaw << " deltaYaw: " << deltaYaw << " w: (" << waypoint->getX() << ", " << waypoint->getY() <<
					") r: (" << robot->currBeliefedLocation.pos.x << ", " << robot->currBeliefedLocation.pos.y << ")" << endl;
			// Adjust the angle
			turnToTarget();
		}
		else
		{
			cout << "Forward, waypoint: (" << waypoint->getX() << ", " << waypoint->getY() <<
					") robot: (" << robot->currBeliefedLocation.pos.x << ", " << robot->currBeliefedLocation.pos.y << ")" << endl;

			// Move forward to the waypoint
			moveForward();
		}

		// Recalculate distance from the wayPoint
		recalculateDistanceFromTarget();
		//mapDrawer->DrawRobot(robot->GetRealHamsterLocation());
		
		// Draw the location of the robot on the map
		mapDrawer->Show(robot->getRealHamsterLocation());
		sleep(1.5);
	}

	// The waypoint has been reached
	cout << "The waypoint: (" << waypoint->getX() << ", " << waypoint->getY() << ") has been reached" << endl;
	
	// Stop to move
	stopMoving();

	return;
}

// Calculate the distance to the waypoint
void MovementManager::recalculateDistanceFromTarget()
{
	distanceFromWaypoint =
		sqrt(pow(robot->currBeliefedLocation.pos.x - waypoint->getX(), 2) +
			 pow(robot->currBeliefedLocation.pos.y - waypoint->getY(), 2));
}

// Turn to the waypoint
void MovementManager::turnToTarget()
{
	robot->setHamsterSpeed(calculateTurnSpeed(), calculateTurningDirection());
}

// Move forward to the waypoint
void MovementManager::moveForward()
{
	robot->setHamsterSpeed(calculateForwardSpeed(), 0.0);
}

// Stop moving the robot
void MovementManager::stopMoving()
{
	robot->setHamsterSpeed(0.0, 0.0);
}

// Calculate the turning direction
double MovementManager::calculateTurningDirection()
{
	// If the robot angle is bigger than the target angle
	if (robot->currBeliefedLocation.yaw > targetYaw)
	{
		// Check if the robot needs to turn right or left according to wich turn will get to the target angle faster
		if (360 - robot->currBeliefedLocation.yaw + targetYaw <
					robot->currBeliefedLocation.yaw - targetYaw)
		{
			return calculateWheelsAngle() * -1;
		}
		else
		{
			return calculateWheelsAngle();
		}
	}
	else
	{
		// Check if the robot needs to turn right or left according to wich turn will get to the target angle faster
		if (360 - targetYaw + robot->currBeliefedLocation.yaw <
					targetYaw - robot->currBeliefedLocation.yaw)
		{
			return calculateWheelsAngle();
		}
		else
		{
			return calculateWheelsAngle() * -1;
		}
	}
}

// Calculate the wheels angle according to delta yaw
float MovementManager::calculateWheelsAngle()
{
	if(deltaYaw > MAX_WHEELS_ANGLE)
	{
		return MAX_WHEELS_ANGLE;
	}

	if(deltaYaw < MIN_WHEELS_ANGLE)
	{
		return MIN_WHEELS_ANGLE;
	}

	return deltaYaw;
}

// Calculate the turn speed
double MovementManager::calculateTurnSpeed()
{
	// Check the ratio beetween delta angle and target angle
	double yawRatio = deltaYaw / targetYaw;
	
	// If the ratio is bigger than 1 then inverse deltayaw and targetyaw to get a ratio smaller than 1
	if(yawRatio > 1)
	{
		// the ratio will be inverse
		yawRatio = 1 / yawRatio;
	}

	// return a speed according to the previous ratio
	return MIN_TURN_SPEED + ((MAX_TURN_SPEED - MIN_TURN_SPEED) *  yawRatio);
}

// Calculate the speed to go forward
double MovementManager::calculateForwardSpeed()
{
	// If the distance is too far, return full speed
	if (distanceFromWaypoint > MIN_DISTANCE_FOR_FULL_SPEED)
	{
		return MAX_MOVE_SPEED;
	}
	
	// Return a speed according to the distance from the waypoint and a factor
	return (double)distanceFromWaypoint / FORWARD_SPEED_FACTOR;
}

// Calculate the Delta of the target angle with the robot angle
void MovementManager::recalculateDeltaYaw()
{
	// Is in circle
	if((targetYaw > 360 - DELTA_ANGLE_TOL && robot->currBeliefedLocation.yaw < DELTA_ANGLE_TOL) ||
	   (robot->currBeliefedLocation.yaw > 360 - DELTA_ANGLE_TOL && targetYaw < DELTA_ANGLE_TOL))
	{
		deltaYaw = fabs((targetYaw + robot->currBeliefedLocation.yaw) - 360);
	}
	else
	{
		deltaYaw = fabs(targetYaw - robot->currBeliefedLocation.yaw);
	}
}

// Return if there is a need to adjust the angle of the robot
bool MovementManager::isNeedAngleAdjustment()
{
	// If the delta angle is bigger than the angle tolerance
	return !isDeltaAngleOnEndOfCiricle() && deltaYaw > YAW_TOLERANCE;
}

// Forward movement more stable
bool MovementManager::isDeltaAngleOnEndOfCiricle() {
	return (targetYaw > (360 - YAW_TOLERANCE) && robot->currBeliefedLocation.yaw < YAW_TOLERANCE) ||
		   (robot->currBeliefedLocation.yaw > (360 - YAW_TOLERANCE) && targetYaw < YAW_TOLERANCE);
}

// Calculate the angle of the waypoint angle
void MovementManager::calculateTargetYaw(Node* waypoint)
{
	targetYaw = getYawInOneCiricle(convertRadiansToDegrees(atan2((waypoint->getY() - robot->currBeliefedLocation.pos.y),
																(waypoint->getX() - robot->currBeliefedLocation.pos.x))));
}

MovementManager::~MovementManager() {
}
