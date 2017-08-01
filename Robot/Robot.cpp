#include "Robot.h"

// Constructor
Robot::Robot(Hamster * h, LocalizationManager * locMng, double height, double width, double res) :
leHamster(h), localizationManager(locMng), mapHeight(height), mapWidth(width), resolution(res) {}

// Get the hamster's current location by returning a structure of {positionState} type
positionState Robot::getRealHamsterLocation()
{
	// Get location of robot using getPose()
	// TODO: Stop using getPose() and start using Particles!
	Pose currPose = leHamster->getPose();

	// Calculate {x,y} by resolution
	double x = currPose.getX() / resolution;
	double y = currPose.getY() * (-1) / resolution;

	// Hamster's angle
	double currYaw = getYawInOneCiricle(currPose.getHeading());

	position currentPos = { .x = x, .y = y };
	positionState currPosState = { .pos = currentPos, .yaw = 360 - currYaw };

	return currPosState;
}

// Update the hamster's location
//
void Robot::updateHamsterRealLocation()
{
	realLocation = getRealHamsterLocation();
}

// Move hamster in given speed and angle
void Robot::setHamsterSpeed(double speed, double wheelsAngle)
{
	this->leHamster->sendSpeed(speed, wheelsAngle);
}

// Destructor
Robot::~Robot()
{
	this->leHamster = NULL;
}

